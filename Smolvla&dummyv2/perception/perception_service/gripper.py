"""Bounded commissioning controls for the verified installed HAND firmware.

HAND_O/C apply signed current, not a position target. The installed firmware
rejects a HAND_I value of zero and exposes no independent stop command. Jog
release therefore disables motor output while retaining an operator-ready
software latch; the next press re-enables the motor before applying direction.
"""
from __future__ import annotations

from collections import deque
import logging
import math
import re
import threading
import time
from typing import Any, Callable

from .common import ServiceError, monotonic_ns, wall_time_ms


class GripperController:
    # Enabling creates a persistent operator-ready session. The actual motor
    # is disabled on jog release because this firmware cannot apply 0 A.
    JOG_WATCHDOG_SECONDS = 2.0
    CURRENT_MIN = 0.05
    CURRENT_MAX = 1.0

    def __init__(self, request: Callable, io_lock: Any, event_callback: Callable,
                 motion_active: Callable[[], bool]):
        self._request = request
        self._io_lock = io_lock
        self._event = event_callback
        self._motion_active = motion_active
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._timer: threading.Timer | None = None
        self._epoch = 0
        self._deadline: float | None = None
        self._stop_pending = False
        self._closing = False
        self._busy = False
        self._may_be_powered = False
        self._faulted = False
        self._latch = "unknown"
        self._current = 0.60
        self._applied_current: float | None = None
        self._control_ready = False
        self._angle: float | None = None
        self._angle_at: int | None = None
        self._jog_direction: str | None = None
        self._last_jog_direction: str | None = None
        self._history: deque[dict] = deque(maxlen=12)

    def active(self) -> bool:
        """Whether a bounded gripper transaction is using robot control IO."""
        with self._lock:
            return bool(self._busy or self._stop_pending or self._jog_direction
                        or self._deadline is not None)

    def enabled(self) -> bool:
        with self._lock:
            return self._latch == "confirmed" and self._may_be_powered

    def status(self) -> dict:
        with self._lock:
            return {
                "enabled_latch": self._latch,
                "control_ready": self._control_ready,
                "hardware_enabled": self._latch == "confirmed" and self._may_be_powered,
                "busy": self._busy or self._stop_pending,
                "current_a": self._current,
                "applied_current_a": self._applied_current,
                "holding": (self._control_ready and self._jog_direction is None
                            and not self._may_be_powered),
                "current_range_a": [self.CURRENT_MIN, self.CURRENT_MAX],
                "remaining_ms": max(0, round((self._deadline - time.monotonic()) * 1000))
                    if self._deadline is not None else None,
                "faulted": self._faulted,
                "reported_motor_angle": self._angle,
                "angle_reported_at_ms": self._angle_at,
                "calibration_available": False,
                "position_available": False,
                "history": list(self._history),
                "last_transaction": self._history[-1] if self._history else None,
                "jog_direction": self._jog_direction,
                "jog_watchdog_ms": max(0, round((self._deadline - time.monotonic()) * 1000))
                    if self._jog_direction and self._deadline is not None else None,
            }

    def invalidate_latch(self) -> None:
        # Only called with shared IO held, after verifying no bounded action is active.
        with self._lock:
            self._latch = "unknown"
            self._control_ready = False

    def _send(self, action: str, command: str, expected: str, source: str) -> dict:
        # Caller owns the robot IO lock. Keep the raw bridge result even when
        # transport succeeded but the expected firmware reply did not arrive.
        transaction = {"action": action, "command": command, "source": source,
                       "at_ms": wall_time_ms(), "monotonic_ns": monotonic_ns(),
                       "acknowledged": False, "raw_response": ""}
        try:
            bridge = self._request("POST", "/command",
                                   {"command": command, "read_timeout_ms": 150}, timeout=4)
            raw = bridge.get("raw_response", "")
            transaction.update(bridge=bridge, raw_response=raw)
            lines = [line.strip() for line in raw.splitlines()]
            ack = expected in lines
            if action in {"current", "jog_hold_current", "jog_restore"}:
                target = float(command.split()[1])
                ack = any(re.fullmatch(r"ok hand current [-+0-9.eE]+", line)
                          and math.isclose(float(line.split()[-1]), target, abs_tol=1e-5)
                          for line in lines)
            transaction["acknowledged"] = bool(bridge.get("sent") is True
                                                  and not bridge.get("error") and ack)
            for match in re.finditer(r"real_angle:([-+0-9.eE]+)", raw):
                try:
                    angle = float(match.group(1))
                    if math.isfinite(angle):
                        with self._lock:
                            self._angle, self._angle_at = angle, transaction["at_ms"]
                except ValueError:
                    pass
        except Exception as exc:
            transaction["error"] = f"{type(exc).__name__}: {exc}"
        with self._lock:
            self._history.append(transaction)
        try:
            self._event({"type": "gripper_action", **transaction})
        except Exception:
            logging.exception("Could not record gripper event")
        return transaction

    def _arm_deadline(self, seconds: float, *, jog: bool = False) -> None:
        with self._lock:
            if self._stop_pending or self._closing:
                raise ServiceError(409, "gripper_stopping", "夹爪正在停止，请稍后重试。")
            self._epoch += 1
            if self._timer:
                self._timer.cancel()
            self._deadline = time.monotonic() + seconds
            callback = self.jog_stop if jog else self.disable
            self._timer = threading.Timer(seconds, callback,
                                          kwargs={"source": "watchdog", "epoch": self._epoch})
            self._timer.daemon = True
            self._timer.start()

    def _clear_deadline(self) -> None:
        with self._lock:
            self._epoch += 1
            self._deadline = None
            if self._timer:
                self._timer.cancel()
                self._timer = None

    def _require_jog_ready(self, direction: str) -> None:
        if direction not in {"open", "close"}:
            raise ServiceError(422, "gripper_direction_invalid", "点动方向只能是 open 或 close。")
        if self._motion_active():
            raise ServiceError(409, "motion_active", "运动、急停或原始命令模式下不能使用夹爪点动。")
        with self._lock:
            if self._closing or self._stop_pending or self._faulted:
                raise ServiceError(409, "gripper_not_ready", "请先确认夹爪失能成功，再进行点动。")
            if not self._control_ready:
                raise ServiceError(409, "gripper_not_enabled", "请先在本页面使能夹爪，再按住点动按钮。")
            if self._jog_direction and self._jog_direction != direction:
                raise ServiceError(409, "gripper_opposite_direction", "另一方向仍在点动，请先松开并停止。")
            if (self._deadline is not None and self._history
                    and self._history[-1]["action"] in {"open", "close"}):
                raise ServiceError(409, "gripper_pulse_active", "短时开合尚未结束，请等待自动失能。")

    def jog_start(self, direction: str) -> dict:
        self._require_jog_ready(direction)
        if not self._action_lock.acquire(blocking=False):
            raise ServiceError(409, "gripper_busy", "上一条夹爪命令尚未返回。")
        try:
            with self._io_lock:
                self._require_jog_ready(direction)
                command, expected = {
                    "open": ("!HAND_O", "ok hand open"),
                    "close": ("!HAND_C", "ok hand close"),
                }[direction]
                # Restore current first. After a prior jog release the motor
                # is physically disabled, so transparently re-enable it before
                # applying the requested direction.
                self._arm_deadline(self.JOG_WATCHDOG_SECONDS, jog=True)
                with self._lock:
                    self._jog_direction = direction
                    self._last_jog_direction = direction
                    needs_enable = self._latch != "confirmed" or not self._may_be_powered
                restore = self._send("jog_restore", f"!HAND_I {self._current:.4g}", "", "operator")
                if not restore["acknowledged"]:
                    cleanup = self.disable(source="command_failure")
                    return {"acknowledged": False, "transaction": restore,
                            "cleanup": cleanup, "gripper": self.status()}
                enable_result = None
                if needs_enable:
                    enable_result = self._send("jog_enable", "!HAND_EN", "ok hand enable", "operator")
                    if not enable_result["acknowledged"]:
                        cleanup = self.disable(source="command_failure")
                        return {"acknowledged": False, "transaction": enable_result,
                                "current_transaction": restore, "cleanup": cleanup,
                                "gripper": self.status()}
                    with self._lock:
                        self._latch = "confirmed"
                        self._may_be_powered = True
                result = self._send(f"jog_{direction}", command, expected, "operator")
                if not result["acknowledged"]:
                    cleanup = self.disable(source="command_failure")
                    return {"acknowledged": False, "transaction": result,
                            "current_transaction": restore, "cleanup": cleanup,
                            "gripper": self.status()}
                with self._lock:
                    self._applied_current = self._current
                return {"acknowledged": True, "transaction": result,
                        "current_transaction": restore,
                        "enable_transaction": enable_result,
                        "gripper": self.status()}
        finally:
            self._action_lock.release()

    def jog_heartbeat(self, direction: str) -> dict:
        if direction not in {"open", "close"}:
            raise ServiceError(422, "gripper_direction_invalid", "点动方向只能是 open 或 close。")
        with self._lock:
            if self._jog_direction != direction or not self._may_be_powered:
                raise ServiceError(409, "gripper_jog_inactive", "该方向当前没有活动的点动控制。")
            if self._closing or self._stop_pending or self._faulted:
                raise ServiceError(409, "gripper_not_ready", "夹爪正在停止或状态异常。")
        self._arm_deadline(self.JOG_WATCHDOG_SECONDS, jog=True)
        return {"acknowledged": True, "gripper": self.status()}

    def jog_stop(self, *, source: str = "operator", epoch: int | None = None) -> dict:
        """Stop jog motion while preserving the operator-ready session."""
        with self._lock:
            if epoch is not None and epoch != self._epoch:
                return {"skipped": True}
            direction = self._jog_direction
            if direction not in {"open", "close"}:
                self._jog_direction = None
                self._clear_deadline()
                return {"acknowledged": True, "skipped": True, "gripper": self.status()}
            self._epoch += 1
            stop_epoch = self._epoch
            self._stop_pending = True
            self._deadline = None
            self._jog_direction = None
            if self._timer:
                self._timer.cancel()
                self._timer = None
        with self._io_lock:
            result = self._send("jog_stop", "!HAND_DIS", "ok hand disable", source)
            if not result["acknowledged"]:
                cleanup = self.disable(source="jog_hold_failure")
                return {"acknowledged": False, "transaction": result,
                        "cleanup": cleanup, "gripper": self.status()}
            with self._lock:
                self._applied_current = 0.0
                self._latch = "disabled"
                self._may_be_powered = False
                self._faulted = False
                if stop_epoch == self._epoch:
                    self._stop_pending = False
        return {"acknowledged": True, "transaction": result, "gripper": self.status()}

    def disable(self, *, source: str = "operator", epoch: int | None = None) -> dict:
        # Mark the stop before waiting for IO: new actions must not overtake it.
        with self._lock:
            if epoch is not None and epoch != self._epoch:
                return {"skipped": True}
            self._epoch += 1
            stop_epoch = self._epoch
            self._stop_pending = True
            self._deadline = None
            self._jog_direction = None
            if self._timer:
                self._timer.cancel()
        with self._io_lock:
            result = self._send("disable", "!HAND_DIS", "ok hand disable", source)
            with self._lock:
                self._latch = "disabled" if result["acknowledged"] else "unknown"
                self._may_be_powered = not result["acknowledged"]
                self._control_ready = False
                self._applied_current = 0.0 if result["acknowledged"] else None
                self._faulted = not result["acknowledged"]
                if stop_epoch == self._epoch:
                    self._stop_pending = False
        return {"acknowledged": result["acknowledged"], "transaction": result,
                "gripper": self.status()}

    def execute(self, action: str, body: dict) -> dict:
        if action in {"position", "calibrate", "zero"}:
            raise ServiceError(409, "gripper_mapping_unverified",
                               "夹爪行程与校准逻辑尚未验证，暂不开放位置或自动校准。")
        fields = {"enable": set(), "disable": set(), "open": {"duration_ms"},
                  "close": {"duration_ms"}, "current": {"current"},
                  "jog-start": {"direction"}, "jog-heartbeat": {"direction"},
                  "jog-stop": set()}
        if action not in fields:
            raise ServiceError(404, "unknown_gripper_action", "不支持该夹爪动作。")
        if set(body) - fields[action]:
            raise ServiceError(422, "unknown_fields", "夹爪动作包含不支持的参数。")
        if action in {"jog-start", "jog-heartbeat"}:
            direction = body.get("direction")
            if not isinstance(direction, str):
                raise ServiceError(422, "gripper_direction_invalid", "点动方向只能是 open 或 close。")
            if action == "jog-start":
                return self.jog_start(direction)
            return self.jog_heartbeat(direction)
        if action == "jog-stop":
            return self.jog_stop()
        duration = body.get("duration_ms", 500)
        if action in {"open", "close"} and (type(duration) is not int or not 100 <= duration <= 1000):
            raise ServiceError(422, "gripper_duration_invalid", "短时开合时长须为 100～1000 毫秒。")
        current = body.get("current")
        if action == "current" and (type(current) not in (int, float)
                or not math.isfinite(current) or not self.CURRENT_MIN <= current <= self.CURRENT_MAX):
            raise ServiceError(422, "gripper_current_invalid", "调试开合电流须为 0.05～1.00 A。")
        if action == "disable":
            return self.disable()
        if self._motion_active():
            raise ServiceError(409, "motion_active", "运动、急停或原始命令模式下不能使用夹爪动作按钮；请先急停并确认成功。")
        if not self._action_lock.acquire(blocking=False):
            raise ServiceError(409, "gripper_busy", "上一条夹爪命令尚未返回。")
        try:
            with self._io_lock:
                with self._lock:
                    if self._closing or self._stop_pending or self._faulted:
                        raise ServiceError(409, "gripper_not_ready", "请先确认夹爪失能成功，再进行操作。")
                    if action in {"open", "close"} and self._jog_direction:
                        raise ServiceError(409, "gripper_jog_active", "点动控制正在运行，请先松开并停止。")
                    if self._motion_active():
                        raise ServiceError(409, "motion_active", "机械臂预设运动期间不能操作夹爪。")
                    if action in {"open", "close"} and self._latch != "confirmed":
                        raise ServiceError(409, "gripper_not_enabled", "请先在本页面使能夹爪。")
                    if action in {"current", "enable"} and (self._may_be_powered or self._control_ready):
                        raise ServiceError(409, "gripper_already_enabled", "请先失能夹爪，再设置电流或重新使能。")
                    if action in {"open", "close"} and self._deadline is not None:
                        # An enable deadline may be replaced by one pulse, but
                        # repeated open/close requests must not extend that pulse.
                        if self._history and self._history[-1]["action"] in {"open", "close"}:
                            raise ServiceError(409, "gripper_pulse_active", "短时开合尚未结束，请等待自动失能。")
                    self._busy = True
                if action in {"enable", "current"}:
                    value = current if action == "current" else self._current
                    result = self._send("current", f"!HAND_I {value:.4g}", "", "operator")
                    if result["acknowledged"]:
                        with self._lock:
                            self._current = float(value)
                            self._applied_current = float(value)
                    else:
                        return {"acknowledged": False, "transaction": result, "gripper": self.status()}
                if action != "current":
                    # Pulses get a bounded disable watchdog. Jog control uses
                    # its own operator-ready latch and safely disables motor
                    # output when its heartbeat is lost.
                    if action == "enable":
                        self._clear_deadline()
                    else:
                        self._arm_deadline(duration / 1000)
                    with self._lock:
                        self._may_be_powered = True
                    commands = {"enable": ("!HAND_EN", "ok hand enable"),
                                "open": ("!HAND_O", "ok hand open"),
                                "close": ("!HAND_C", "ok hand close")}
                    command, expected = commands[action]
                    result = self._send(action, command, expected, "operator")
                    if not result["acknowledged"]:
                        cleanup = self.disable(source="command_failure")
                        return {"acknowledged": False, "transaction": result,
                                "cleanup": cleanup, "gripper": self.status()}
                    with self._lock:
                        if not self._stop_pending:
                            self._latch = "confirmed"
                            if action == "enable":
                                self._control_ready = True
                return {"acknowledged": True, "transaction": result, "gripper": self.status()}
        finally:
            with self._lock:
                self._busy = False
            self._action_lock.release()

    def close(self) -> None:
        with self._lock:
            self._closing = True
            should_disable = self._may_be_powered or self._stop_pending
            self._control_ready = False
            if self._timer:
                self._timer.cancel()
        if should_disable:
            self.disable(source="service_shutdown")
