"""Guarded bridge between the browser and an external MoveIt Servo gateway.

The gateway performs kinematics only.  This process remains the sole owner of
the DummyV2 USB bridge and validates every joint target before forwarding it.
"""
from __future__ import annotations

import math
import secrets
from collections import deque
import threading
import time
from typing import Any, Callable

import requests

from .common import ServiceError, wall_time_ms
from .robot import STREAM_COMMAND_SPEED, FEEDBACK_PERIOD_SECONDS, COMMAND_LIMITS


class MoveItFailure(RuntimeError):
    pass


class LiveControl:
    ARM_CONFIRMATION = "ENABLE_LIVE_CONTROL"
    INPUT_TIMEOUT_SECONDS = 0.35
    OUTPUT_PERIOD_SECONDS = 0.025
    LINEAR_SPEED_M_S = 0.05
    ANGULAR_SPEED_RAD_S = 0.24

    def __init__(
        self,
        robot: Any,
        base_url: str = "http://127.0.0.1:8801",
        *,
        event_callback: Callable[[dict[str, Any]], None] | None = None,
        request: Callable[[str, str, dict[str, Any] | None, float], dict[str, Any]] | None = None,
        start_worker: bool = True,
    ):
        self.robot = robot
        self.base_url = base_url.rstrip("/")
        self._event = event_callback or (lambda _event: None)
        self._request_override = request
        self._session = requests.Session()
        self._lock = threading.RLock()
        self._send_lock = threading.RLock()
        self._session_id: str | None = None
        self._input_generation = 0
        self._motion_active = False
        self._release_pending = False
        self._release_required = False
        self._progress_positions: list[float] | None = None
        self._progress_at = 0.0
        self._output_times: deque[float] = deque(maxlen=250)
        self._armed = False
        self._backend_status = "checking"
        self._backend_reason: str | None = None
        self._tool_frame: dict[str, Any] = {}
        self._last_input_monotonic: float | None = None
        self._last_input: dict[str, Any] = self._zero_command()
        self._last_trajectory_seq = -1
        self._last_output_monotonic = 0.0
        self._last_cycle_monotonic = 0.0
        self._feedback_sampled_at_ms = None
        self._diagnostics: dict[str, Any] = {"output": "idle"}
        self._zero_sent = True
        self._paused_reason: str | None = None
        self._last_stop_reason: str | None = None
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._run, name="moveit-live-control", daemon=True)
        self._worker_started = start_worker
        if start_worker:
            self._worker.start()

    @staticmethod
    def _zero_command() -> dict[str, float]:
        return {name: 0.0 for name in (
            "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"
        )}

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None,
                 timeout: float = 0.5) -> dict[str, Any]:
        if self._request_override:
            return self._request_override(method, path, body, timeout)
        try:
            response = self._session.request(method, self.base_url + path, json=body, timeout=timeout)
            value = response.json()
        except (requests.RequestException, ValueError) as exc:
            raise MoveItFailure(f"moveit_unreachable: {type(exc).__name__}: {exc}") from exc
        if not isinstance(value, dict):
            raise MoveItFailure("moveit_invalid_response")
        if response.status_code >= 400 or value.get("error"):
            raise MoveItFailure(str(value.get("error") or f"moveit_http_{response.status_code}"))
        return value

    def _record(self, action: str, **fields: Any) -> None:
        self._event({"type": "live_control", "action": action, "at_ms": wall_time_ms(), **fields})

    def is_armed(self) -> bool:
        with self._lock:
            return self._armed

    def probe(self) -> None:
        try:
            health = self._request("GET", "/health", timeout=0.35)
            compatible = health.get("service") == "dummy-moveit-safe-gateway" \
                and health.get("protocol_version") == 7 \
                and health.get("command_frame") == "gripper_tool"
            with self._lock:
                self._tool_frame = {key: health.get(key) for key in (
                    'command_frame', 'translation_frame', 'rotation_frame',
                    'tcp_offset_in_link6_m', 'tcp_calibrated',
                    'tcp_configured', 'tcp_reference_approximate',
                    'max_target_lead_deg', 'target_lead_limit_enabled')}
                self._backend_status = ("ready" if health.get('status') == 'ready' else 'checking') if compatible else "incompatible"
                self._backend_reason = None if compatible else "moveit_gateway_contract_mismatch"
        except MoveItFailure as exc:
            with self._lock:
                self._backend_status = "unavailable"
                self._backend_reason = str(exc)

    def status(self) -> dict[str, Any]:
        target_gap = self.robot.status().get("stream_target_gap")
        with self._lock:
            age_ms = None if self._last_input_monotonic is None else max(
                0, round((time.monotonic() - self._last_input_monotonic) * 1000)
            )
            return {
                "armed": self._armed,
                "backend_status": self._backend_status,
                "backend_reason": self._backend_reason,
                "backend_url": self.base_url,
                "tool_frame": dict(self._tool_frame),
                "command_frame": "gripper_tool",
                "translation_frame": "gripper_tool",
                "rotation_frame": "gripper_tool",
                "input_age_ms": age_ms,
                "input_command": dict(self._last_input),
                "diagnostics": dict(self._diagnostics),
                "linear_speed_m_s": self.LINEAR_SPEED_M_S,
                "angular_speed_rad_s": self.ANGULAR_SPEED_RAD_S,
                "stream_command_speed": STREAM_COMMAND_SPEED,
                "max_target_gap_deg": None,
                "target_gap_limit_enabled": False,
                "joint_limits_deg": COMMAND_LIMITS,
                "max_target_lead_deg": self._tool_frame.get('max_target_lead_deg'),
                "target_lead_limit_enabled": bool(self._tool_frame.get('target_lead_limit_enabled')),
                "target_feedback": target_gap,
                "input_timeout_ms": round(self.INPUT_TIMEOUT_SECONDS * 1000),
                "output_rate_hz": round(1 / self.OUTPUT_PERIOD_SECONDS),
                "feedback_rate_hz": round(1 / FEEDBACK_PERIOD_SECONDS),
                "measured_output_hz": round(sum(t > time.monotonic()-5 for t in self._output_times)/5, 1),
                "idle_auto_disarm": False,
                "paused_reason": self._paused_reason,
                "release_required": self._release_required,
                "last_stop_reason": self._last_stop_reason,
            }

    @staticmethod
    def _validate_input(body: object) -> tuple[float, float, list[str]]:
        if not isinstance(body, dict):
            raise ServiceError(400, "json_object_required", "控制输入必须是 JSON 对象。")
        if set(body) - {"yaw", "pitch", "keys", "at_client_ms", "session_id", "stop_mode"}:
            raise ServiceError(422, "unknown_fields", "控制输入只接受 yaw、pitch、keys、at_client_ms、session_id 和 stop_mode。")
        if body.get('stop_mode', 'immediate') not in ('immediate', 'release'):
            raise ServiceError(422, 'control_stop_mode_invalid', '停止模式必须为 immediate 或 release。')
        values = []
        for name in ("yaw", "pitch"):
            value = body.get(name, 0.0)
            if type(value) not in (int, float) or not math.isfinite(value) or not -1 <= value <= 1:
                raise ServiceError(422, "control_value_invalid", "视角输入必须是 -1 到 1 的有限数值。")
            values.append(float(value))
        keys = body.get("keys", [])
        if not isinstance(keys, list) or any(key not in {"w", "a", "s", "d", "arrowup", "arrowdown"} for key in keys):
            raise ServiceError(422, "control_keys_invalid", "移动按键只能包含 W、A、S、D、↑、↓。")
        client_ms = body.get("at_client_ms")
        if client_ms is not None and (type(client_ms) is not int or client_ms < 0):
            raise ServiceError(422, "client_timestamp_invalid", "客户端时间戳必须是非负整数。")
        return values[0], values[1], sorted(set(keys))

    def _feedback(self) -> list[float]:
        state = self.robot.status().get("state") or {}
        if state.get("reason") in {"joint_feedback_verifying", "joint_feedback_unreliable"}:
            raise ServiceError(503, state["reason"], "关节反馈正在复核或复核失败。")
        positions = state.get("positions")
        sampled_at = state.get("sampled_at_ms")
        if not isinstance(positions, list) or len(positions) != 6:
            state = self.robot.read_state()
            if state.get("reason") in {"joint_feedback_verifying", "joint_feedback_unreliable"}:
                raise ServiceError(503, state["reason"], "关节反馈正在复核或复核失败。")
            positions = state.get("positions")
            sampled_at = state.get("sampled_at_ms")
        if (not isinstance(positions, list) or len(positions) != 6
                or not all(type(value) in (int, float) and math.isfinite(value) for value in positions)):
            raise ServiceError(503, "joint_feedback_unavailable", "没有可用于 MoveIt 的六轴实机反馈。")
        if type(sampled_at) is not int or wall_time_ms() - sampled_at > 500:
            raise ServiceError(503, "joint_feedback_stale", "六轴反馈已过期，不能进入流式控制。")
        self._feedback_sampled_at_ms = sampled_at
        return [float(value) for value in positions]

    def _publish_feedback(self, positions: list[float]) -> dict[str, Any]:
        # Sending the same 5 Hz sample at the control rate must not make it
        # appear freshly measured to the gateway watchdog.
        return self._request("POST", "/feedback", {
            "positions": positions,
            "age_ms": max(0, wall_time_ms() - self._feedback_sampled_at_ms),
        }, timeout=0.35)

    def arm(self, body: object) -> dict[str, Any]:
        with self._send_lock:
            return self._arm_locked(body)

    def _arm_locked(self, body: object) -> dict[str, Any]:
        if not isinstance(body, dict) or set(body) - {"confirmation"}:
            raise ServiceError(422, "unknown_fields", "进入实机控制只接受 confirmation 字段。")
        if body.get("confirmation") != self.ARM_CONFIRMATION:
            raise ServiceError(422, "live_control_confirmation_required", "需要长按确认后才能进入实机控制。")
        robot = self.robot.status()
        if robot.get("bridge_status") != "ready":
            raise ServiceError(503, "robot_bridge_unavailable", "DummyV2 通信桥接不可用。")
        if robot.get("enabled_latch") != "confirmed":
            raise ServiceError(409, "robot_not_armed", "请先在本页面使能六轴机械臂。")
        if robot.get("motion") and robot["motion"].get("status") == "running":
            raise ServiceError(409, "motion_active", "预设运动尚未结束，不能进入流式控制。")
        positions = self._feedback()
        if self.is_armed():
            raise ServiceError(409, 'live_control_active', '已有页面正在控制，请先退出。')
        with self._lock:
            generation = self._input_generation
        self.probe()
        if self.status()["backend_status"] != "ready":
            raise ServiceError(503, "moveit_unavailable", "MoveIt 安全网关尚未启动。")
        self.robot.prepare_stream()
        # Switching controller mode can hold serial IO for over 500 ms. Do
        # not arm the gateway using the sample taken before that operation.
        self.robot.read_state()
        positions = self._feedback()
        try:
            self._publish_feedback(positions)
            self._request("POST", "/arm", {"confirmation": self.ARM_CONFIRMATION}, timeout=1)
            self._request("POST", "/command", self._zero_command(), timeout=0.5)
        except MoveItFailure as exc:
            self._record('arm_failed', reason=str(exc))
            try:
                self._request('POST','/disarm',{},timeout=.25)
            except MoveItFailure:
                pass
            raise ServiceError(503, "moveit_arm_failed",
                               f"MoveIt 未能进入控制状态，六轴没有收到运动命令。详情：{exc}") from exc
        with self._lock:
            if generation != self._input_generation or self.robot.status().get('enabled_latch') != 'confirmed':
                try:
                    self._request('POST', '/disarm', {}, timeout=.25)
                except MoveItFailure:
                    pass
                raise ServiceError(409, 'live_control_cancelled', '进入控制期间已停止或失能，请重新确认。')
            self._armed = True
            self._session_id = secrets.token_urlsafe(24)
            self._input_generation += 1
            self._motion_active = False
            self._release_pending = False
            self._release_required = False
            self._progress_positions = positions
            self._progress_at = time.monotonic()
            self._output_times.clear()
            self._last_input_monotonic = time.monotonic()
            self._last_input = self._zero_command()
            self._last_trajectory_seq = -1
            self._last_output_monotonic = 0.0
            self._last_cycle_monotonic = 0.0
            self._diagnostics = {"output": "idle"}
            self._zero_sent = True
            self._paused_reason = None
            self._last_stop_reason = None
        self._record("armed")
        return {**self.status(), 'session_id':self._session_id}

    def update(self, body: object) -> dict[str, Any]:
        yaw, pitch, keys = self._validate_input(body)
        with self._lock:
            if not self._armed:
                raise ServiceError(409, "live_control_not_armed", "尚未进入实机控制。")
            if body.get('session_id') != self._session_id:
                raise ServiceError(409, 'control_session_mismatch', '仅进入控制的页面可以操作；其他页面只读。')
        forward = float(("w" in keys) - ("s" in keys))
        lateral = float(("a" in keys) - ("d" in keys))
        vertical = float(("arrowup" in keys) - ("arrowdown" in keys))
        # Tool X forward, Y left, Z up. Combined translation adds components.
        command = {
            "linear_x": forward * self.LINEAR_SPEED_M_S,
            "linear_y": lateral * self.LINEAR_SPEED_M_S,
            "linear_z": vertical * self.LINEAR_SPEED_M_S,
            "angular_x": 0.0,
            # 8/2 rotate about tool -/+Y; 4/6 about tool +/-Z.
            # Gateway rotates both velocity vectors into base coordinates.
            "angular_y": -pitch * self.ANGULAR_SPEED_RAD_S,
            "angular_z": -yaw * self.ANGULAR_SPEED_RAD_S,
        }
        with self._lock:
            # Recheck with the mutation: a stop/rearm may overlap validation above.
            if not self._armed:
                raise ServiceError(409, 'live_control_not_armed', '尚未进入实机控制。')
            if body.get('session_id') != self._session_id:
                raise ServiceError(409, 'control_session_mismatch', '控制会话已改变，旧输入已丢弃。')
            self._last_input_monotonic = time.monotonic()
            changed = command != self._last_input
            if changed:
                self._input_generation += 1
            generation = self._input_generation
            self._last_input = command
            if self._release_required and any(command.values()):
                return {'accepted':True,'paused':True,'control':self.status()}
            self._zero_sent = not any(command.values())
            if self._zero_sent:
                self._paused_reason = None
                self._release_required = False
                self._progress_positions = None
                self._progress_at = time.monotonic()
        try:
            with self._send_lock:
                with self._lock:
                    if not self._armed or generation != self._input_generation:
                        return {'accepted':False, 'superseded':True}
                self._request("POST", "/command", command, timeout=0.35)
                if not any(command.values()):
                    if body.get('stop_mode') == 'release':
                        self._release_without_target()
                    else:
                        self._hold_if_needed()
        except MoveItFailure as exc:
            self._failsafe(f"command_failed: {exc}")
            raise ServiceError(503, "moveit_command_failed", "MoveIt 控制命令发送失败，已执行安全停止。") from exc
        except Exception as exc:
            self._failsafe(f'input_stop_failed: {getattr(exc, "code", type(exc).__name__)}: {exc}')
            raise
        if changed:
            self._record('input', command=command, command_frame='gripper_tool',
                         tool_frame=dict(self._tool_frame))
        return {"accepted": True, "command": command, "control": self.status()}

    def _release_without_target(self) -> None:
        # Gateway zeroing cancels future integration only. Keep the last
        # hardware destination unchanged, including throughout idle heartbeats.
        if self._motion_active:
            self._motion_active = False
            self._release_pending = True
            self._diagnostics['output'] = 'released'
            self._diagnostics['release_stop'] = None
            self._record('release', policy='keep_last_target', hardware_command_sent=False)

    def _hold_if_needed(self) -> None:
        # Faults / explicit immediate stops must still cancel a destination
        # left in flight by normal release, even if no new input was sent.
        if self._motion_active or self._release_pending:
            self.robot.hold_stream()
            self._motion_active = False
            self._release_pending = False
            self._diagnostics['output'] = 'holding'
            self._diagnostics['release_stop'] = None
            self._record('hold', predictive=False)

    def disarm(self, reason: str = "operator", *, disable_robot: bool = True) -> dict[str, Any]:
        with self._lock:
            was_armed = self._armed
            self._armed = False
            self._input_generation += 1
            self._session_id = None
            self._last_input = self._zero_command()
            self._zero_sent = True
            self._paused_reason = None
            self._last_stop_reason = reason
            self._motion_active = False
            self._release_pending = False
            self._release_required = False
            self._diagnostics['output'] = 'stopped'
        gateway = {}
        for action, path, command in (
            ('zero', '/command', self._zero_command()), ('disarm', '/disarm', {})
        ):
            try:
                gateway[action] = self._request('POST', path, command, timeout=0.35)
            except MoveItFailure as exc:
                gateway[action] = {'error': str(exc)}
        robot_result = None
        gripper_result = None
        acknowledged = None
        if was_armed and disable_robot:
            # Both belong to the same control session. Attempt each even when
            # the other fails; a stopped jog must also lose its ready latch.
            try:
                gripper_result = self.robot.gripper.disable(source='live_control_exit')
            except Exception as exc:
                gripper_result = {"acknowledged": False, "error": f"{type(exc).__name__}: {exc}"}
            try:
                robot_result = self.robot.disable()
            except Exception as exc:  # Emergency route remains available if a normal disable fails.
                robot_result = {"error": f"{type(exc).__name__}: {exc}"}
            acknowledged = (gripper_result.get('acknowledged') is True
                            and robot_result.get('enabled_latch') == 'disabled')
        if was_armed:
            self._record("disarmed", reason=reason, gateway=gateway,
                         robot=robot_result, gripper=gripper_result,
                         overall_acknowledged=acknowledged)
        return {"armed": False, "reason": reason, "gateway": gateway, "robot": robot_result,
                "gripper": gripper_result, "overall_acknowledged": acknowledged}

    def emergency_reset(self) -> None:
        with self._lock:
            self._armed = False
            self._input_generation += 1
            self._session_id = None
            self._last_input = self._zero_command()
            self._zero_sent = True
            self._paused_reason = None
            self._last_stop_reason = "emergency_stop"
            self._motion_active = False
            self._release_pending = False
            self._release_required = False
            self._diagnostics['output'] = 'stopped'
        try:
            self._request("POST", "/disarm", {}, timeout=0.25)
        except MoveItFailure:
            pass

    def _failsafe(self, reason: str) -> None:
        with self._lock:
            if not self._armed:
                return
            self._armed = False
            self._paused_reason = None
            self._last_stop_reason = reason
            self._input_generation += 1
            self._session_id = None
            self._zero_sent = True
            self._last_input = self._zero_command()
            self._diagnostics['output'] = 'stopped'
            self._diagnostics['emergency_acknowledged'] = None
            self._motion_active = False
            self._release_pending = False
            self._release_required = False
        try:
            emergency = self.robot.emergency_stop("live_control_guard")
        except Exception as exc:
            emergency = {'overall_acknowledged': False, 'error': f'{type(exc).__name__}: {exc}'}
        with self._lock:
            self._diagnostics['emergency_acknowledged'] = emergency.get('overall_acknowledged') is True
        try:
            self._request("POST", "/disarm", {}, timeout=0.25)
        except MoveItFailure:
            pass
        self._record("failsafe", reason=reason, emergency=emergency)

    def _pause_motion(self, reason: str) -> None:
        """Command zero velocity without changing either enable latch."""
        with self._lock:
            if not self._armed:
                return
            already_zero = self._zero_sent
            previous_reason = self._paused_reason
        with self._send_lock:
            if not already_zero:
                self._request("POST", "/command", self._zero_command(), timeout=0.25)
            self._hold_if_needed()
        with self._lock:
            if not self._armed:
                return
            self._zero_sent = True
            self._paused_reason = reason
            self._release_required = reason != 'browser_input_idle'
            self._diagnostics["output"] = "paused"
        if previous_reason != reason:
            self._record("paused", reason=reason)

    def _cycle(self) -> None:
        with self._lock:
            if not self._armed:
                return
            last_input = self._last_input_monotonic
            zero_sent = self._zero_sent
            generation = self._input_generation
            last_cycle = self._last_cycle_monotonic
        now = time.monotonic()
        if now - last_cycle < self.OUTPUT_PERIOD_SECONDS:
            return
        with self._lock:
            self._last_cycle_monotonic = now
        # Keep the model current even while idle or paused. The gateway rejects
        # twists once its feedback is older than 0.5 seconds.
        positions = self._feedback()
        reply = self._publish_feedback(positions)
        trajectory = reply.get('trajectory')
        if trajectory is None:
            trajectory = self._request("GET", "/trajectory", timeout=0.35)
        with self._lock:
            self._diagnostics.update({
                "servo_status": trajectory.get("servo_status"),
                "guard_warning": trajectory.get("guard_warning"),
                "trajectory_sequence": trajectory.get("sequence"),
                "trajectory_age_ms": trajectory.get("age_ms"),
                "lead_limited": trajectory.get("lead_limited", False),
                "joint_limit_limited": trajectory.get("joint_limit_limited", False),
                "joint_limit_axes": trajectory.get("joint_limit_axes", []),
            })
            # Network requests above can overlap browser input or a disarm.
            if not self._armed or generation != self._input_generation:
                return
            last_input = self._last_input_monotonic
            zero_sent = self._zero_sent
        age = float("inf") if last_input is None else time.monotonic() - last_input
        if age > self.INPUT_TIMEOUT_SECONDS:
            self._pause_motion("browser_input_idle")
            return
        if zero_sent:
            servo = trajectory.get('servo_status') or {}
            if self._release_pending and servo.get('code') in {2, 5, 6}:
                self._pause_motion(f"moveit_servo_{servo['code']}: {servo.get('message', 'status_unknown')}")
                return
            if self._release_pending and (trajectory.get('guard_warning') or {}).get('code') == 'singularity_stop':
                self._pause_motion('moveit_servo_2: singularity_stop')
                return
            with self._lock:
                self._diagnostics["output"] = "paused" if self._paused_reason else 'released' if self._release_pending else "idle"
            return
        seq = trajectory.get("sequence")
        target = trajectory.get("positions")
        fresh = trajectory.get("age_ms")
        servo_status = trajectory.get("servo_status") or {}
        servo_code = servo_status.get("code")
        if type(servo_code) is int and servo_code in {2, 5, 6}:
            with self._lock:
                self._diagnostics["output"] = "servo_blocked"
            message = str(servo_status.get("message") or "status_unknown")[:120]
            self._pause_motion(f"moveit_servo_{servo_code}: {message}")
            return
        warning = trajectory.get("guard_warning") or {}
        if warning.get("code") == "singularity_stop":
            self._pause_motion("moveit_servo_2: singularity_stop")
            return
        missing_age = trajectory.get("missing_age_ms")
        if target is None and type(missing_age) in (int, float):
            if not math.isfinite(missing_age) or missing_age > 250:
                self._pause_motion("moveit_trajectory_stale")
            else:
                with self._lock:
                    self._diagnostics["output"] = "waiting_trajectory"
            return
        if type(fresh) in (int, float) and fresh > 250:
            self._pause_motion("moveit_trajectory_stale")
            return
        if type(seq) is not int or seq <= self._last_trajectory_seq:
            with self._lock:
                self._diagnostics["output"] = "waiting_trajectory"
            return
        if type(fresh) not in (int, float) or fresh > 250:
            self._pause_motion("moveit_trajectory_stale")
            return
        if isinstance(target,list) and len(target) == 6:
            gap = max(abs(a-b) for a,b in zip(target,positions))
            with self._lock:
                self._diagnostics['target_gap_deg'] = round(gap,3)
            progress = self._progress_positions
            if progress is None or max(abs(a-b) for a,b in zip(positions,progress)) > .1 or gap < .35:
                self._progress_positions = positions
                self._progress_at = time.monotonic()
            elif self._motion_active and time.monotonic()-self._progress_at > 3:
                self._pause_motion('motion_no_progress')
                return
        with self._lock:
            self._diagnostics["output"] = "sending_target"
        with self._send_lock:
            with self._lock:
                if not self._armed or self._zero_sent or generation != self._input_generation:
                    return
            result = self.robot.send_stream_target(target, speed=STREAM_COMMAND_SPEED)
            if not result.get('skipped'):
                self._motion_active = True
                self._release_pending = False
                self._output_times.append(time.monotonic())
        with self._lock:
            if not self._armed:
                return
            self._diagnostics["output"] = "target_matches_feedback" if result.get("skipped") else "target_sent"
            self._last_trajectory_seq = seq
            self._last_output_monotonic = time.monotonic()
            self._paused_reason = None

    def _run(self) -> None:
        next_probe = 0.0
        delay = 0.0
        while not self._shutdown.wait(delay):
            started = time.monotonic()
            if self.is_armed():
                try:
                    self._cycle()
                except ServiceError as exc:
                    if exc.code == "joint_feedback_verifying":
                        # A short, bounded reread is in progress. Do not feed
                        # suspect angles to MoveIt, send targets, or disable.
                        with self._lock:
                            self._diagnostics["output"] = "feedback_verifying"
                    elif exc.code in {"joint_feedback_unavailable", "joint_feedback_stale"}:
                        try:
                            self._pause_motion(exc.code)
                        except Exception as pause_exc:
                            self._failsafe(f"pause_failed: {getattr(pause_exc, 'code', type(pause_exc).__name__)}: {pause_exc}")
                    else:
                        self._failsafe(f"{type(exc).__name__}: {exc.code}: {exc}")
                except Exception as exc:
                    self._failsafe(f"{type(exc).__name__}: {exc}")
                # HTTP/serial time is part of the 25 ms period, not an extra
                # sleep afterwards. Overruns never queue catch-up targets.
                delay = max(0.001, self.OUTPUT_PERIOD_SECONDS - (time.monotonic() - started))
            elif time.monotonic() >= next_probe:
                self.probe()
                next_probe = time.monotonic() + 2
                delay = 0.05
            else:
                delay = 0.05

    def close(self) -> None:
        if self.is_armed():
            self.disarm("service_shutdown", disable_robot=True)
        self._shutdown.set()
        if self._worker_started:
            self._worker.join(timeout=2)
        self._session.close()
