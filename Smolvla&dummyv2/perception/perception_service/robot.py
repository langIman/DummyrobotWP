from __future__ import annotations

import math
import logging
import json
from collections import deque
import re
import threading
import time
import uuid
from typing import Any, Callable

import requests

from .common import ServiceError, monotonic_ns, wall_time_ms
from .gripper import GripperController
from .release_stop import release_destination
from motion.joint_contract import HARDWARE_LIMITS


JOINT_PATTERN = re.compile(
    r"^(?:ok|okok)\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s*$"
)

COMMAND_LIMITS = HARDWARE_LIMITS
STREAM_LIMIT_TOLERANCE_DEG = 1.5
STREAM_NOOP_TOLERANCE_DEG = 0.02
STREAM_COMMAND_SPEED = 50
STREAM_MAX_COMMAND_SPEED = 50
FEEDBACK_PERIOD_SECONDS = 0.2
# A deliberately generous plausibility envelope, not a motion speed setting.
# A 90 -> 10 degree jump in one 40 ms sample is rejected before model/control use.
FEEDBACK_JUMP_MIN_DEG = 8.0
FEEDBACK_JUMP_RATE_DEG_S = 360.0
FEEDBACK_JUMP_WINDOW_SECONDS = 0.5
FEEDBACK_READ_ATTEMPTS = 3

PRESETS = {
    "home": (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
    "rest": (0.0, -75.0, 180.0, 0.0, 0.0, 0.0),
    "ready": (0.0, 0.0, 90.0, 0.0, 45.0, 0.0),
}
PRESET_SPEEDS = {"home": 4, "rest": 4, "ready": 12}

# The installed firmware reports the gripper's volatile motor angle.  The
# service can provide a convenient logical coordinate without writing an
# offset to the controller: the first valid sample is assumed to be the
# physically closed pose (115 degrees).
GRIPPER_CLOSED_REFERENCE_DEG = 115.0
GRIPPER_OPEN_REFERENCE_DEG = -115.0

CONFIRMATIONS = {"home": "MOVE_HOME", "rest": "MOVE_REST", "ready": "MOVE_READY"}


class BridgeFailure(RuntimeError):
    pass


class RobotClient:
    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8765",
        *,
        recording_active: Callable[[], bool] | None = None,
        state_callback: Callable[[dict[str, Any]], None] | None = None,
        event_callback: Callable[[dict[str, Any]], None] | None = None,
        start_sampler: bool = True,
    ):
        self.base_url = base_url.rstrip("/")
        self._recording_active = recording_active or (lambda: False)
        self._state_callback = state_callback or (lambda _state: None)
        self._event_callback = event_callback or (lambda _event: None)
        self._session = requests.Session()
        self._state_lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._motion_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._motion_cancel = threading.Event()
        self._enabled_latch = "unknown"
        self._bridge_status = "checking"
        self._bridge_reason: str | None = None
        self._last_state: dict[str, Any] = self._empty_state("not_sampled")
        self._stream_target_gap: dict[str, Any] | None = None
        self._last_good_feedback: dict[str, Any] | None = None
        self._release_feedback_history: deque[dict[str, Any]] = deque(maxlen=12)
        self._last_feedback_attempt_ns: int | None = None
        self._feedback_pending_limit: float | None = None
        self._feedback_times: deque[float] = deque(maxlen=250)
        self._motion: dict[str, Any] | None = None
        self._emergency_count = 0
        self._raw_busy = False
        self._raw_mode = False
        self._raw_history: deque[dict] = deque(maxlen=30)
        self._gripper_read_lock = threading.Lock()
        self._gripper_reference_raw: float | None = None
        self._gripper_reference_at_ms: int | None = None
        self._gripper_reference_source = "startup_assumed_closed"
        self._gripper_feedback = {"status": "unavailable", "angle_deg": None,
                                  "sampled_at_ms": None, "monotonic_ns": None,
                                  "source": "stlink_mainboard_cache", "unit": "degree",
                                  "reason": "not_sampled", "message": "尚未读取夹爪角度。"}
        self.gripper = GripperController(lambda *args, **kwargs: self._request(*args, **kwargs), self._io_lock,
                                         self._event_callback, self._motion_in_progress)
        self._sampler = threading.Thread(target=self._sample_loop, name="robot-sampler", daemon=True)
        self._gripper_sampler = threading.Thread(target=self._sample_gripper_loop, name="gripper-feedback", daemon=True)
        self._sampler_started = start_sampler
        if start_sampler:
            self._sampler.start()
            self._gripper_sampler.start()

    @staticmethod
    def _empty_state(reason: str) -> dict[str, Any]:
        return {
            "positions": None,
            "joints": [f"J{index}" for index in range(1, 7)],
            "unit": "degree",
            "coordinate_space": "hardware_joint",
            "sampled_at_ms": None,
            "monotonic_ns": monotonic_ns(),
            "reason": reason,
        }

    def _motion_in_progress(self) -> bool:
        with self._state_lock:
            return bool(self._emergency_count or self._raw_mode or self._raw_busy
                        or (self._motion and self._motion.get("status") == "running"))

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None, timeout: float = 4) -> dict[str, Any]:
        data = self._transport_request(method, path, body, timeout)
        if data.get("error"):
            raise BridgeFailure(str(data["error"]))
        return data

    def _transport_request(self, method: str, path: str, body: dict[str, Any] | None = None,
                           timeout: float = 4) -> dict[str, Any]:
        try:
            response = self._session.request(method, self.base_url + path, json=body, timeout=timeout)
            data = response.json()
        except (requests.RequestException, ValueError) as exc:
            raise BridgeFailure(f"bridge_unreachable: {type(exc).__name__}: {exc}") from exc
        if not isinstance(data, dict):
            raise BridgeFailure("bridge_invalid_response")
        if response.status_code >= 400:
            data.setdefault("error", {"code": f"bridge_http_{response.status_code}"})
        return data

    def raw_command(self, body: object) -> dict:
        if not isinstance(body, dict):
            raise ServiceError(422, "json_object_required", "请求正文必须是 JSON 对象。")
        if set(body) - {"command", "read_timeout_ms"}:
            raise ServiceError(422, "unknown_fields", "只接受 command 和 read_timeout_ms 字段。")
        command = body.get("command")
        # Firmware motion queue entries are 64 bytes: reserve newline and NUL.
        if (not isinstance(command, str) or not command.strip() or len(command) > 62
                or any(ord(c) < 32 or ord(c) > 126 for c in command)):
            raise ServiceError(422, "ascii_command_invalid", "请输入单行可打印 ASCII 命令，最多 62 字节，不含换行。")
        wait_ms = body.get("read_timeout_ms", 500)
        if type(wait_ms) is not int or not 0 <= wait_ms <= 2000:
            raise ServiceError(422, "read_timeout_invalid", "回复等待须为 0～2000 毫秒整数。")
        if not self._io_lock.acquire(blocking=False):
            raise ServiceError(409, "robot_io_busy", "上一条通信尚未结束，请稍后手动重试。")
        try:
            if self._shutdown.is_set():
                raise ServiceError(409, "service_closing", "服务正在关闭。")
            if self.gripper.active():
                raise ServiceError(409, "gripper_active", "请先停止短时夹爪控制，再发送原始命令。")
            with self._state_lock:
                if self._emergency_count or (self._motion and self._motion.get("status") == "running"):
                    raise ServiceError(409, "motion_active", "预设运动或急停正在执行，不能发送原始命令。")
                self._raw_busy = True
                self._raw_mode = True
                self._enabled_latch = "unknown"
            self.gripper.invalidate_latch()
            transaction = {"id": uuid.uuid4().hex[:12], "command": command,
                           "read_timeout_ms": wait_ms, "at_ms": wall_time_ms(),
                           "monotonic_ns": monotonic_ns(), "sent": None,
                           "execution_status": "unknown", "raw_response": ""}
            try:
                bridge = self._transport_request("POST", "/command",
                                                 {"command": command, "read_timeout_ms": wait_ms})
                transaction.update(bridge=bridge, sent=bridge.get("sent"),
                                   bytes_written=bridge.get("bytes_written"),
                                   raw_response=bridge.get("raw_response", ""),
                                   pending_response=bridge.get("pending_response", ""))
                if bridge.get("error") or bridge.get("sent") is not True:
                    transaction["error"] = {"code": "raw_transport_unconfirmed",
                                            "message": "发送或读取未确认；请查看桥接详情，不要盲目重发。"}
            except BridgeFailure as exc:
                transaction["error"] = {"code": "raw_transport_unavailable",
                                        "message": "通信失败，命令是否执行未知；请检查连接，不要盲目重发。"}
                transaction["transport_error"] = str(exc)
            with self._state_lock:
                self._raw_history.append(transaction)
            try:
                self._event_callback({"type": "robot_raw_command", **transaction})
            except Exception:
                logging.exception("Could not record raw command event")
            return transaction
        finally:
            with self._state_lock:
                self._raw_busy = False
            self._io_lock.release()

    def _require_structured_mode(self) -> None:
        with self._state_lock:
            if self._raw_mode or self._raw_busy:
                raise ServiceError(409, "raw_mode_active", "原始模式下状态未知；请先点击急停并确认成功，再使用使能和预设按钮。")

    def _safe_request(
        self, method: str, path: str, body: dict[str, Any] | None = None, timeout: float = 4
    ) -> dict[str, Any]:
        try:
            return {"ok": True, "response": self._request(method, path, body, timeout)}
        except BridgeFailure as exc:
            return {"ok": False, "error": str(exc)}

    def _probe_health(self) -> None:
        try:
            data = self._request("GET", "/health", timeout=1)
            valid = data.get("protocol_version") == 4 and data.get("mode") == "transport"
            with self._state_lock:
                self._bridge_status = "ready" if valid else "incompatible"
                self._bridge_reason = None if valid else "bridge_contract_mismatch"
        except BridgeFailure as exc:
            with self._state_lock:
                self._bridge_status = "unavailable"
                self._bridge_reason = str(exc)

    @staticmethod
    def _parse_positions(raw_response: str) -> list[float] | None:
        for line in raw_response.splitlines():
            match = JOINT_PATTERN.match(line.strip())
            if match:
                values = [float(value) for value in match.groups()]
                if all(math.isfinite(value) for value in values):
                    return values
        return None

    def gripper_feedback(self) -> dict:
        with self._state_lock:
            result = dict(self._gripper_feedback)
        stamp = result.get("monotonic_ns")
        result["age_ms"] = max(0, (monotonic_ns() - stamp) / 1_000_000) if stamp else None
        result["stale"] = result["age_ms"] is None or result["age_ms"] > 2500
        return result

    def _with_gripper_reference(self, value: dict[str, Any], *, capture_if_missing: bool) -> dict[str, Any]:
        """Add a process-local logical angle while retaining the raw reading."""
        result = dict(value)
        raw = result.get("angle_deg")
        if (result.get("status") != "available" or type(raw) not in (int, float)
                or not math.isfinite(raw)):
            return result
        with self._state_lock:
            if capture_if_missing and self._gripper_reference_raw is None:
                self._gripper_reference_raw = float(raw)
                self._gripper_reference_at_ms = result.get("sampled_at_ms")
            reference = self._gripper_reference_raw
            reference_at = self._gripper_reference_at_ms
            source = self._gripper_reference_source
        if reference is None:
            return result
        unclamped = GRIPPER_CLOSED_REFERENCE_DEG + (float(raw) - reference)
        result.update({
            "raw_angle_deg": float(raw),
            "logical_angle_deg": max(GRIPPER_OPEN_REFERENCE_DEG,
                                      min(GRIPPER_CLOSED_REFERENCE_DEG, unclamped)),
            "logical_unclamped_angle_deg": unclamped,
            "reference_raw_angle_deg": reference,
            "reference_angle_deg": GRIPPER_CLOSED_REFERENCE_DEG,
            "reference_captured_at_ms": reference_at,
            "reference_source": source,
            "reference_assumption": "首次有效读数视为上电时夹爪全闭；如未保持全闭，请手动重新记录。",
        })
        return result

    def capture_gripper_reference(self) -> dict[str, Any]:
        """Record the current raw angle as the software's closed reference.

        This is read-only from the robot's perspective: no serial command,
        enable, disable, or motion is sent.
        """
        current = self.read_gripper_state()
        angle = current.get("angle_deg")
        if (current.get("status") != "available" or type(angle) not in (int, float)
                or not math.isfinite(angle)):
            raise ServiceError(503, "gripper_feedback_unavailable", "无法读取夹爪角度，未建立新的软件参考。")
        raw = float(current.get("raw_angle_deg", current["angle_deg"]))
        with self._state_lock:
            self._gripper_reference_raw = raw
            self._gripper_reference_at_ms = current.get("sampled_at_ms")
            self._gripper_reference_source = "operator_confirmed_closed"
            self._gripper_feedback = self._with_gripper_reference(current, capture_if_missing=False)
            result = dict(self._gripper_feedback)
        result["age_ms"] = 0
        result["stale"] = False
        self._event_callback({"type": "gripper_reference", "source": "operator",
                              "raw_angle_deg": raw, "logical_angle_deg": GRIPPER_CLOSED_REFERENCE_DEG,
                              "at_ms": wall_time_ms(), "monotonic_ns": monotonic_ns()})
        return result

    def read_gripper_state(self) -> dict:
        # SWD is independent of serial IO; never hold the emergency/command lock.
        with self._gripper_read_lock:
            try:
                value = self._request("GET", "/gripper/state", timeout=4)
                angle = value.get("angle_deg")
                if value.get("status") == "available" and (
                        type(angle) not in (int, float) or not math.isfinite(angle)
                        or type(value.get("monotonic_ns")) is not int
                        or type(value.get("sampled_at_ms")) is not int):
                    raise BridgeFailure("gripper_feedback_invalid")
            except BridgeFailure as exc:
                value = {"status": "unavailable", "angle_deg": None, "sampled_at_ms": None,
                         "monotonic_ns": None, "source": "stlink_mainboard_cache", "unit": "degree",
                         "reason": str(exc), "message": "夹爪角度读取失败，请检查桥接和 ST-LINK。"}
            with self._state_lock:
                self._gripper_feedback = value
            with self._state_lock:
                self._gripper_feedback = self._with_gripper_reference(
                    self._gripper_feedback, capture_if_missing=True)
        return self.gripper_feedback()

    def _sample_gripper_loop(self) -> None:
        initial = True
        while not self._shutdown.is_set():
            with self._state_lock:
                enabled = self._enabled_latch == "confirmed"
                raw_mode = self._raw_mode
            if not raw_mode and (initial or enabled or self._recording_active() or self.gripper.enabled()):
                initial = False
                self.read_gripper_state()
                self._shutdown.wait(1)
            else:
                self._shutdown.wait(0.25)

    def _feedback_event(self, action: str, **fields: Any) -> None:
        event = {"type": "joint_feedback", "action": action,
                 "at_ms": wall_time_ms(), **fields}
        # Keep evidence even when no recording is active.
        logging.getLogger(__name__).warning("joint_feedback %s", json.dumps(event))
        self._event_callback(event)

    def _read_verified_feedback(self) -> dict[str, Any]:
        """Caller owns IO through reading, validation and publication."""
        previous = self._last_good_feedback
        limit = self._feedback_pending_limit
        if previous is not None and limit is None:
            age = max(0., (monotonic_ns() - previous["monotonic_ns"]) / 1e9)
            if age <= FEEDBACK_JUMP_WINDOW_SECONDS:
                limit = max(FEEDBACK_JUMP_MIN_DEG, 2. + FEEDBACK_JUMP_RATE_DEG_S * age)
        # Freeze this envelope across retries: repeated bad samples must not
        # become plausible merely because time passed while verifying them.
        for attempt in range(FEEDBACK_READ_ATTEMPTS):
            if self._emergency_count or self._shutdown.is_set():
                raise BridgeFailure("joint_feedback_interrupted")
            sampled_wall, sampled_mono = wall_time_ms(), monotonic_ns()
            result = self._request("POST", "/command",
                                   {"command": "#GETJPOS", "read_timeout_ms": 100})
            positions = self._parse_positions(result.get("raw_response", ""))
            gap = None if positions is None or previous is None else max(
                abs(a-b) for a, b in zip(positions, previous["positions"]))
            suspicious = positions is None or (limit is not None and gap is not None and gap > limit)
            if suspicious:
                self._feedback_pending_limit = limit
                with self._state_lock:
                    self._last_state = self._empty_state("joint_feedback_verifying")
                self._feedback_event("rejected", attempt=attempt+1, positions=positions,
                                     previous_positions=previous["positions"] if previous else None,
                                     gap_deg=gap, limit_deg=limit,
                                     raw_response=result.get("raw_response", ""))
                continue
            state = {
                "positions": positions, "joints": [f"J{i}" for i in range(1, 7)],
                "unit": "degree", "coordinate_space": "hardware_joint",
                "sampled_at_ms": sampled_wall, "monotonic_ns": sampled_mono,
                "reason": None, "port": result.get("port"), "device_id": result.get("device_id"),
                "verification_retries": attempt,
            }
            if attempt or self._feedback_pending_limit is not None:
                self._feedback_event("recovered", positions=positions, retries=attempt)
            self._feedback_pending_limit = None
            self._last_good_feedback = dict(state)
            with self._state_lock:
                self._feedback_times.append(time.monotonic())
                self._release_feedback_history.append(dict(state))
                self._bridge_status = "ready"
                self._bridge_reason = None
                self._last_state = state
            return state
        self._feedback_event("verification_failed", attempts=FEEDBACK_READ_ATTEMPTS)
        raise BridgeFailure("joint_feedback_unreliable")

    def read_state(self, *, automatic: bool = False, include_gripper: bool = False,
                   force: bool = False) -> dict[str, Any]:
        # All normal consumers share one 5 Hz hardware sample, including failed
        # attempts. Keep its original timestamps; a cache hit is not a sample.
        sampled = False
        with self._io_lock:
            try:
                if automatic and self._raw_mode:
                    with self._state_lock:
                        return dict(self._last_state)
                now = monotonic_ns()
                if (not force and self._last_feedback_attempt_ns is not None
                        and now - self._last_feedback_attempt_ns < FEEDBACK_PERIOD_SECONDS * 1e9):
                    with self._state_lock:
                        state = dict(self._last_state)
                else:
                    self._last_feedback_attempt_ns = now
                    sampled = True
                    state = self._read_verified_feedback()
            except BridgeFailure as exc:
                state = self._empty_state(str(exc))
                state["sampled_at_ms"] = wall_time_ms()
                with self._state_lock:
                    self._last_state = state
                    if str(exc).startswith("bridge_unreachable"):
                        self._bridge_status = "unavailable"
                        self._bridge_reason = str(exc)
                        self._enabled_latch = "unknown"
        if include_gripper:
            self.read_gripper_state()
        state["gripper"] = self.gripper_feedback()
        if sampled:
            self._state_callback(dict(state))
        return state

    def _sample_loop(self) -> None:
        next_health = 0.0
        while not self._shutdown.is_set():
            with self._state_lock:
                enabled = self._enabled_latch == "confirmed"
                motion_running = bool(self._motion and self._motion.get("status") == "running")
            if self._raw_mode:
                self._shutdown.wait(0.1)
                continue
            if enabled or motion_running or self._recording_active():
                started = time.monotonic()
                self.read_state(automatic=True)
                self._shutdown.wait(max(0.001, FEEDBACK_PERIOD_SECONDS - (time.monotonic() - started)))
            else:
                if time.monotonic() >= next_health:
                    self._probe_health()
                    next_health = time.monotonic() + 2
                self._shutdown.wait(0.25)

    def _command(self, command: str, expected: str, timeout_ms: int = 500) -> dict[str, Any]:
        with self._io_lock:
            result = self._request(
                "POST", "/command", {"command": command, "read_timeout_ms": timeout_ms}
            )
        acknowledged = expected in result.get("raw_response", "").splitlines()
        return {"acknowledged": acknowledged, "bridge": result}

    def enable(self) -> dict[str, Any]:
        self._require_structured_mode()
        with self._io_lock:
            return self._enable_locked()

    def _enable_locked(self) -> dict[str, Any]:
        self._require_structured_mode()
        if self._emergency_count:
            raise ServiceError(409, "emergency_active", "急停正在执行，请等待确认。")
        if self.gripper.active():
            raise ServiceError(409, "gripper_active", "请先停止夹爪，再使能六轴机械臂。")
        result = self._command("!START", "Started ok")
        with self._state_lock:
            self._enabled_latch = "confirmed" if result["acknowledged"] else "unknown"
        self._event_callback({"type": "robot_action", "action": "enable", **result})
        if not result["acknowledged"]:
            raise ServiceError(503, "robot_enable_unconfirmed", "机械臂未确认使能。")
        return self.status()

    def disable(self) -> dict[str, Any]:
        self._motion_cancel.set()
        result = self._command("!DISABLE", "Disabled ok")
        with self._state_lock:
            self._enabled_latch = "disabled" if result["acknowledged"] else "unknown"
        self._event_callback({"type": "robot_action", "action": "disable", **result})
        if not result["acknowledged"]:
            raise ServiceError(503, "robot_disable_unconfirmed", "机械臂未确认失能。")
        return self.status()

    def prepare_stream(self) -> None:
        """Select interruptible targets, so new targets replace rather than queue."""
        self._require_structured_mode()
        with self._io_lock:
            if self.status()['enabled_latch'] != 'confirmed':
                raise ServiceError(409, 'robot_not_armed', '六轴尚未使能。')
            result = self._command('#CMDMODE 2', 'ok Set command mode to [2]')
            if not result['acknowledged']:
                raise ServiceError(503, 'stream_mode_unconfirmed', '主控未确认连续目标模式，未进入实机控制。')
            with self._state_lock:
                self._stream_target_gap = None
                self._release_feedback_history.clear()
            self._event_callback({'type':'robot_stream_mode','mode':2,'at_ms':wall_time_ms(),
                                  'acknowledged':True})

    def hold_stream(self, *, predictive: bool = False) -> dict[str, Any]:
        """Replace a pending target once; prediction is for normal release only."""
        with self._io_lock:
            state = self.read_state(force=True)
            positions = state.get('positions')
            if (state.get('reason') or not isinstance(positions, list) or len(positions) != 6
                    or not all(type(value) in (int, float) and math.isfinite(value) for value in positions)):
                reason = state.get('reason') or 'invalid_joint_reply'
                self._feedback_event('hold_rejected', reason=reason)
                # This is failed hardware feedback, not an invalid MoveIt target.
                # Never replace it with zero angles or a cached old target.
                raise ServiceError(503, 'hold_feedback_unavailable',
                                   f'无法读取有效关节反馈，未发送位置保持目标。通信详情：{reason}')
            plan = None
            if predictive:
                with self._state_lock:
                    history = list(self._release_feedback_history)
                plan = release_destination(state, history, COMMAND_LIMITS, monotonic_ns())
                positions = plan['positions']
            # Clamp small endpoint feedback noise for an explicit hold only.
            # A substantially out-of-range pose must fail rather than move far.
            positions = [min(max(q, low), high) if low-STREAM_LIMIT_TOLERANCE_DEG <= q <= high+STREAM_LIMIT_TOLERANCE_DEG else q
                         for q, (low, high) in zip(positions, COMMAND_LIMITS)]
            result = self.send_stream_target(positions, speed=STREAM_COMMAND_SPEED, force=True)
            if plan is not None:
                event = {'type': 'robot_release_stop', 'at_ms': wall_time_ms(),
                         'speed': STREAM_COMMAND_SPEED, **plan}
                logging.getLogger(__name__).warning('release_stop %s', json.dumps(event))
                self._event_callback(event)
                return {**result, 'release_stop': plan}
            return result

    def send_stream_target(self, positions: object, *, speed: float = STREAM_COMMAND_SPEED, force: bool = False) -> dict[str, Any]:
        """Forward one MoveIt-generated joint target after local safety checks."""
        # No target may pass validation against a sample that another reader
        # replaces while this sender is waiting for the serial channel.
        with self._io_lock:
            return self._send_stream_target_locked(positions, speed=speed, force=force)

    def _send_stream_target_locked(self, positions: object, *, speed: float, force: bool) -> dict[str, Any]:
        if (not isinstance(positions, list) or len(positions) != 6
                or not all(type(value) in (int, float) and math.isfinite(value)
                            for value in positions)):
            raise ServiceError(422, "stream_target_invalid", "MoveIt 必须返回六个有限关节角。")
        target = [float(value) for value in positions]
        for index, (value, (low, high)) in enumerate(zip(target, COMMAND_LIMITS), start=1):
            if not low <= value <= high:
                raise ServiceError(422, "stream_target_outside_limit",
                                   f"J{index} 目标 {value:.3f}° 超出主控范围 {low:g}～{high:g}°，未发送。")
        if type(speed) not in (int, float) or not math.isfinite(speed) or not 1 <= speed <= STREAM_MAX_COMMAND_SPEED:
            raise ServiceError(422, "stream_speed_invalid", f"流式控制速度必须在 1 到 {STREAM_MAX_COMMAND_SPEED} 之间。")
        with self._state_lock:
            if self._enabled_latch != "confirmed":
                raise ServiceError(409, "robot_not_armed", "六轴机械臂尚未由本服务成功使能。")
            if self._emergency_count or self._raw_mode or self._raw_busy:
                raise ServiceError(409, "robot_control_busy", "急停或原始命令正在占用机械臂。")
            if self._motion and self._motion.get("status") == "running":
                raise ServiceError(409, "motion_active", "预设运动期间不能使用流式控制。")
            current = self._last_state.get("positions")
            sampled_at = self._last_state.get("sampled_at_ms")
            feedback_reason = self._last_state.get("reason")
        if feedback_reason in {"joint_feedback_verifying", "joint_feedback_unreliable"}:
            raise ServiceError(503, feedback_reason, "关节反馈异常，运动目标未下发。")
        if not isinstance(current, list) or len(current) != 6 or type(sampled_at) is not int:
            raise ServiceError(503, "joint_feedback_unavailable", "缺少实机反馈，已拒绝流式目标。")
        if wall_time_ms() - sampled_at > 1500:
            raise ServiceError(503, "joint_feedback_stale", "实机反馈过期，已拒绝流式目标。")
        deltas = [abs(goal - actual) for goal, actual in zip(target, current)]
        largest_index = max(range(6), key=deltas.__getitem__)
        largest_step = deltas[largest_index]
        # Capture a consistent target/feedback pair for display only. The
        # operator disabled gap-based rejection; other send checks remain.
        with self._state_lock:
            self._stream_target_gap = {
                "gap_deg": largest_step, "joint": f"J{largest_index + 1}",
                "target_deg": target[largest_index], "actual_deg": current[largest_index],
                "sampled_at_ms": sampled_at, "compared_at_ms": wall_time_ms(),
                "over_limit": False, "limit_enabled": False,
            }
        if largest_step < STREAM_NOOP_TOLERANCE_DEG and not force:
            return {"accepted": True, "skipped": "target_matches_feedback"}
        with self._io_lock:
            # A disable/emergency can occur while this request waits for USB.
            with self._state_lock:
                if self._enabled_latch != 'confirmed' or self._emergency_count:
                    raise ServiceError(409, 'robot_not_armed', '机械臂已停止，旧目标已丢弃。')
            result = self._request(
                "POST", "/motion",
                {"positions": target, "speed": float(speed), "unit": "degree",
                 "coordinate_space": "hardware_joint"},
                timeout=1,
            )
        raw_reply = str(result.get("raw_response", ""))
        if "terminate called after throwing an instance of" in raw_reply:
            with self._state_lock:
                self._enabled_latch = "unknown"
            logging.error("controller_firmware_exception: %r", result)
            raise BridgeFailure(f"controller_firmware_exception: reply={raw_reply[:160]!r}")
        if not result.get("accepted"):
            raise BridgeFailure(f"stream_motion_not_accepted: sent={result.get('sent')}, "
                                f"reply={str(result.get('raw_response', ''))[:160]!r}")
        self._event_callback({'type':'robot_stream_target', 'positions':target, 'speed':speed,
                              'hold':force, 'at_ms':wall_time_ms(),
                              'monotonic_ns':monotonic_ns(), 'acknowledged':True})
        return result

    def emergency_stop(self, source: str = "operator") -> dict[str, Any]:
        with self._state_lock:
            self._emergency_count += 1
        try:
            return self._perform_emergency_stop(source)
        finally:
            with self._state_lock:
                self._emergency_count -= 1

    def _perform_emergency_stop(self, source: str) -> dict[str, Any]:
        self._motion_cancel.set()
        # Prioritize the arm STOP/DISABLE. A stuck gripper transaction must not
        # consume its entire USB deadline before the arm stop is attempted.
        with self._io_lock:
            stopped = self._safe_request("POST", "/stop", {})
            disabled = self._safe_request(
                "POST", "/command", {"command": "!DISABLE", "read_timeout_ms": 500}
            )
        gripper = self.gripper.disable(source=source)
        with self._state_lock:
            stop_response = stopped.get("response") or {}
            disable_raw = (disabled.get("response") or {}).get("raw_response", "")
            stop_acknowledged = bool((stop_response.get("stop") or {}).get("acknowledged"))
            disable_acknowledged = "Disabled ok" in disable_raw.splitlines()
            self._enabled_latch = "disabled" if "Disabled ok" in disable_raw.splitlines() else "unknown"
        result = {
            "source": source,
            "stop": {**stopped, "acknowledged": stop_acknowledged},
            "disable": {**disabled, "acknowledged": disable_acknowledged},
            "gripper": gripper,
            "overall_acknowledged": stop_acknowledged and disable_acknowledged and gripper["acknowledged"],
        }
        if result["overall_acknowledged"]:
            with self._state_lock:
                self._raw_mode = False
        self._event_callback({"type": "robot_action", "action": "emergency_stop", **result})
        return result

    def start_preset(self, name: str, confirmation: object) -> dict[str, Any]:
        self._require_structured_mode()
        with self._io_lock:
            return self._start_preset_locked(name, confirmation)

    def _start_preset_locked(self, name: str, confirmation: object) -> dict[str, Any]:
        self._require_structured_mode()
        if self._emergency_count:
            raise ServiceError(409, "emergency_active", "急停正在执行，请等待确认。")
        if self.gripper.active():
            raise ServiceError(409, "gripper_active", "请先停止夹爪，再执行机械臂预设动作。")
        if name not in PRESETS:
            raise ServiceError(404, "unknown_robot_action", "不支持该机械臂动作。")
        if confirmation != CONFIRMATIONS[name]:
            raise ServiceError(422, "motion_confirmation_required", "预设运动缺少有效确认。")
        with self._state_lock:
            if self._enabled_latch != "confirmed":
                raise ServiceError(409, "robot_not_armed", "必须先由本服务成功使能机械臂。")
            if self._motion and self._motion.get("status") == "running":
                raise ServiceError(409, "motion_active", "已有预设运动正在执行。")
            motion_id = uuid.uuid4().hex[:12]
            self._motion = {
                "id": motion_id,
                "name": name,
                "status": "running",
                "target": list(PRESETS[name]),
                "speed": PRESET_SPEEDS[name],
                "started_at_ms": wall_time_ms(),
                "completed_at_ms": None,
                "reason": None,
            }
            self._motion_cancel.clear()
        thread = threading.Thread(
            target=self._run_preset, args=(motion_id, name), name=f"motion-{name}", daemon=True
        )
        thread.start()
        self._event_callback(
            {"type": "robot_action", "action": name, "phase": "started", "motion_id": motion_id}
        )
        return self.status()

    def _set_motion_result(self, motion_id: str, status: str, reason: str | None) -> None:
        with self._state_lock:
            if self._motion and self._motion.get("id") == motion_id:
                self._motion.update(
                    status=status, reason=reason, completed_at_ms=wall_time_ms()
                )

    def _run_preset(self, motion_id: str, name: str) -> None:
        if not self._motion_lock.acquire(blocking=False):
            self._set_motion_result(motion_id, "failed", "motion_executor_busy")
            return
        try:
            target = PRESETS[name]
            speed = PRESET_SPEEDS[name]
            current = self.read_state().get("positions")
            if current is None:
                raise BridgeFailure("initial_joint_feedback_unavailable")
            for index, (value, (low, high)) in enumerate(zip(target, COMMAND_LIMITS), start=1):
                if not low <= value <= high:
                    raise BridgeFailure(f"target_J{index}_outside_limit")
            largest_move = max(abs(goal - actual) for goal, actual in zip(target, current))
            timeout = max(8.0, largest_move / (speed * 0.45) + 8.0)
            with self._io_lock:
                accepted = self._request(
                    "POST",
                    "/motion",
                    {
                        "positions": list(target),
                        "speed": speed,
                        "unit": "degree",
                        "coordinate_space": "hardware_joint",
                    },
                )
            if not accepted.get("accepted"):
                raise BridgeFailure("motion_not_accepted")

            deadline = time.monotonic() + timeout
            best_error = largest_move
            last_progress = time.monotonic()
            settled = 0
            previous_sample_ns = None
            while time.monotonic() < deadline:
                if self._motion_cancel.wait(0.2):
                    raise BridgeFailure("motion_cancelled")
                state = self.read_state()
                positions = state.get("positions")
                if positions is None:
                    raise BridgeFailure("joint_feedback_lost")
                sample_ns = state.get("monotonic_ns")
                if sample_ns is not None and sample_ns == previous_sample_ns:
                    continue
                previous_sample_ns = sample_ns
                error = max(abs(goal - actual) for goal, actual in zip(target, positions))
                if error < best_error - 0.35:
                    best_error = error
                    last_progress = time.monotonic()
                if error <= 1.5:
                    settled += 1
                    if settled >= 3:
                        self._set_motion_result(motion_id, "complete", None)
                        self._event_callback(
                            {
                                "type": "robot_action",
                                "action": name,
                                "phase": "complete",
                                "motion_id": motion_id,
                                "positions": positions,
                            }
                        )
                        return
                else:
                    settled = 0
                if error > 3 and time.monotonic() - last_progress > 3:
                    raise BridgeFailure("motion_no_progress")
            raise BridgeFailure("motion_timeout")
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"
            with self._state_lock:
                externally_stopped = self._enabled_latch != "confirmed"
            if "motion_cancelled" in reason and externally_stopped:
                self._set_motion_result(motion_id, "cancelled", reason)
                self._event_callback(
                    {
                        "type": "robot_action",
                        "action": name,
                        "phase": "cancelled",
                        "motion_id": motion_id,
                    }
                )
                return
            self._set_motion_result(motion_id, "failed", reason)
            emergency = self.emergency_stop("motion_guard")
            self._event_callback(
                {
                    "type": "robot_action",
                    "action": name,
                    "phase": "failed",
                    "motion_id": motion_id,
                    "reason": reason,
                    "emergency": emergency,
                }
            )
        finally:
            self._motion_lock.release()

    def status(self) -> dict[str, Any]:
        gripper = self.gripper.status()
        with self._state_lock:
            return {
                "bridge_status": self._bridge_status,
                "bridge_reason": self._bridge_reason,
                "bridge_url": self.base_url,
                "feedback_rate_hz": round(1 / FEEDBACK_PERIOD_SECONDS),
                "measured_feedback_hz": round(sum(t > time.monotonic()-5 for t in self._feedback_times)/5, 1),
                "enabled_latch": self._enabled_latch,
                "stream_target_gap": dict(self._stream_target_gap) if self._stream_target_gap else None,
                "state": {**self._last_state, "gripper": self.gripper_feedback()},
                "motion": dict(self._motion) if self._motion else None,
                "presets": {name: list(values) for name, values in PRESETS.items()},
                "preset_speeds": dict(PRESET_SPEEDS),
                "gripper": gripper,
                "raw_console": {"busy": self._raw_busy, "active": self._raw_mode,
                                "history": list(self._raw_history), "max_command_bytes": 62},
            }

    def close(self) -> None:
        self.gripper.close()
        with self._state_lock:
            motion_running = bool(self._motion and self._motion.get("status") == "running")
        if motion_running:
            self.emergency_stop("service_shutdown")
        self._shutdown.set()
        if self._sampler_started:
            self._sampler.join(timeout=2)
            self._gripper_sampler.join(timeout=4)
        self._session.close()
