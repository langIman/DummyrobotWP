#!/usr/bin/env python3
"""Loopback-only ROS 2 MoveIt Servo gateway with no hardware access."""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from moveit_msgs.msg import ServoStatus
from rclpy.node import Node
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from tool_frame import ToolFrame, COMMAND_FRAME
from continuous_target import ContinuousTarget
from joint_contract import HARDWARE_LIMITS, MODEL_DIRECTIONS, MODEL_OFFSETS_RAD, near_limit_axes


JOINT_NAMES = [f"Joint{index}" for index in range(1, 7)]
OFFSET_RAD = MODEL_OFFSETS_RAD
DIRECTION = MODEL_DIRECTIONS
COMMAND_FIELDS = {
    "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"
}


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def hardware_to_model(values: list[float]) -> list[float]:
    return [math.radians(value) * direction - offset
            for value, direction, offset in zip(values, DIRECTION, OFFSET_RAD)]


def model_to_hardware(values: list[float]) -> list[float]:
    return [math.degrees((value + offset) * direction)
            for value, direction, offset in zip(values, DIRECTION, OFFSET_RAD)]


class GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_moveit_safe_gateway")
        self.lock = threading.RLock()
        self.tool_frame = ToolFrame()
        self.continuous_target = ContinuousTarget()
        self.armed = False
        self.feedback: list[float] | None = None
        self.feedback_at = 0.0
        self.command = {name: 0.0 for name in COMMAND_FIELDS}
        self.command_at = 0.0
        self.trajectory: list[float] | None = None
        self.trajectory_at = 0.0
        self.trajectory_sequence = -1
        self.halt_messages = 0
        self.servo_status = {"code": 0, "message": "", "age_ms": None}
        self.servo_status_at = 0.0
        self.guard_warning = None
        self.guard_warning_at = 0.0
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        # Kept for MoveIt Servo topic discovery; browser control uses Cartesian twist only.
        self.jog_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.create_subscription(Float64MultiArray, "/servo_node/joint_velocities", self.on_velocities, 1)
        self.create_subscription(ServoStatus, "/servo_node/status", self.on_servo_status, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 100)
        self.switch_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self.pause_client = self.create_client(SetBool, "/servo_node/pause_servo")
        self.timer = self.create_timer(0.025, self.tick)

    def feedback_fresh(self) -> bool:
        with self.lock:
            return self.feedback is not None and time.monotonic() - self.feedback_at <= 0.5

    def set_feedback(self, positions: list[float], age_ms: float = 0.0) -> None:
        with self.lock:
            self.feedback = hardware_to_model(positions)
            self.feedback_at = time.monotonic() - age_ms / 1000.0

    def set_command(self, command: dict[str, float]) -> None:
        with self.lock:
            if not any(command.values()) or not any(self.command.values()):
                self.continuous_target.reset()
                self.trajectory = None
                # Start a bounded first-output wait when leaving idle. Never
                # reuse a target from the previous key press.
                self.trajectory_at = time.monotonic()
            self.command = dict(command)
            self.command_at = time.monotonic()

    @staticmethod
    def wait_for_result(future, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def configure_servo(self) -> tuple[bool, str]:
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            return False, "servo_command_type_service_unavailable"
        switch = ServoCommandType.Request()
        switch.command_type = ServoCommandType.Request.TWIST
        result = self.wait_for_result(self.switch_client.call_async(switch))
        if not result or not result.success:
            return False, "servo_twist_mode_failed"
        if not self.pause_client.wait_for_service(timeout_sec=2.0):
            return False, "servo_pause_service_unavailable"
        pause = SetBool.Request()
        pause.data = False
        result = self.wait_for_result(self.pause_client.call_async(pause))
        if not result or not result.success:
            return False, "servo_unpause_failed"
        return True, "servo_twist_mode_ready"

    def arm(self) -> tuple[bool, str]:
        if not self.feedback_fresh():
            return False, "joint_feedback_stale"
        if self.twist_pub.get_subscription_count() < 1:
            return False, "servo_command_subscriber_unavailable"
        okay, reason = self.configure_servo()
        if not okay:
            return False, reason
        with self.lock:
            self.armed = True
            self.command = {name: 0.0 for name in COMMAND_FIELDS}
            self.command_at = time.monotonic()
            self.trajectory = None
            self.trajectory_sequence = -1
            self.continuous_target.reset()
        return True, "armed"

    def disarm(self) -> None:
        with self.lock:
            self.armed = False
            self.command = {name: 0.0 for name in COMMAND_FIELDS}
            self.halt_messages = 6
            self.continuous_target.reset()
            self.trajectory = None
        if self.pause_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            self.pause_client.call_async(request)

    def tick(self) -> None:
        now = time.monotonic()
        with self.lock:
            feedback = list(self.feedback) if self.feedback is not None else None
            feedback_fresh = feedback is not None and now - self.feedback_at <= 0.5
            armed = self.armed
            command = dict(self.command)
            command_fresh = now - self.command_at <= 0.25
            if not feedback_fresh or not command_fresh or not armed:
                self.continuous_target.reset()
                self.trajectory = None
            halt = self.halt_messages > 0
            if halt:
                self.halt_messages -= 1
        if feedback_fresh:
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = "base_link"
            state.name = JOINT_NAMES
            state.position = feedback
            self.joint_pub.publish(state)
        if armed or halt:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            if armed and feedback_fresh and command_fresh:
                # Translation uses base axes; rotation uses the measured tool
                # orientation, with TCP compensation on every tick.
                command = self.tool_frame.to_base(command, feedback)
                twist.twist.linear.x = command["linear_x"]
                twist.twist.linear.y = command["linear_y"]
                twist.twist.linear.z = command["linear_z"]
                twist.twist.angular.x = command["angular_x"]
                twist.twist.angular.y = command["angular_y"]
                twist.twist.angular.z = command["angular_z"]
            self.twist_pub.publish(twist)

    def on_velocities(self, message: Float64MultiArray) -> None:
        with self.lock:
            now = time.monotonic()
            if (not self.armed or not self.feedback_fresh() or now-self.command_at > .25
                    or not any(self.command.values()) or self.servo_status['code'] in {2, 5, 6}):
                self.continuous_target.reset()
                self.trajectory = None
                return
            velocities = list(message.data)
            if len(velocities) != 6 or not all(math.isfinite(value) for value in velocities):
                self.continuous_target.reset()
                self.trajectory = None
                return
            # Velocity conversion uses axis direction only, never position offsets.
            velocities = [math.degrees(v)*d for v, d in zip(velocities, DIRECTION)]
            self.trajectory = self.continuous_target.advance(
                velocities, model_to_hardware(self.feedback), now)
            self.trajectory_at = now
            self.trajectory_sequence += 1

    def on_servo_status(self, message: ServoStatus) -> None:
        with self.lock:
            self.servo_status = {"code": int(message.code), "message": str(message.message)}
            self.servo_status_at = time.monotonic()
            if int(message.code) in {2, 5, 6}:
                self.continuous_target.reset()
                self.trajectory = None

    def on_log(self, message: Log) -> None:
        # Servo's single status code can report collision deceleration while
        # its singularity check has already suppressed the trajectory.
        if 'servo' not in message.name or message.level < Log.WARN:
            return
        if 'Very close to a singularity' in message.msg:
            with self.lock:
                self.guard_warning = {"code": "singularity_stop", "message": message.msg}
                self.guard_warning_at = time.monotonic()

    def health(self) -> dict:
        with self.lock:
            return {
                "service": "dummy-moveit-safe-gateway",
                "protocol_version": 7,
                "joint_limits_deg": HARDWARE_LIMITS,
                "feedback_age_supported": True,
                "publish_rate_hz": 40,
                **self.tool_frame.status(),
                "output_mode": ("lead_limited_velocity_integration" if self.continuous_target.MAX_LEAD_DEG is not None
                                else "joint_limited_velocity_integration"),
                "max_target_lead_deg": self.continuous_target.MAX_LEAD_DEG,
                "target_lead_limit_enabled": self.continuous_target.MAX_LEAD_DEG is not None,
                "status": "ready" if (self.twist_pub.get_subscription_count()
                                        and self.switch_client.service_is_ready()
                                        and self.pause_client.service_is_ready())
                          else "waiting_for_servo",
                "armed": self.armed,
                "feedback_fresh": self.feedback_fresh(),
                "servo_subscribers": self.twist_pub.get_subscription_count(),
                "time_ms": now_ms(),
            }

    def latest_trajectory(self) -> dict:
        with self.lock:
            age = None if self.trajectory is None else round(
                (time.monotonic() - self.trajectory_at) * 1000, 2
            )
            status = dict(self.servo_status)
            status["age_ms"] = None if not self.servo_status_at else round(
                (time.monotonic() - self.servo_status_at) * 1000, 2
            )
            warning = None
            if self.guard_warning and time.monotonic() - self.guard_warning_at < 1.0:
                warning = dict(self.guard_warning)
                warning["age_ms"] = round((time.monotonic() - self.guard_warning_at) * 1000, 2)
            return {
                "guard_warning": warning,
                "sequence": self.trajectory_sequence,
                "positions": list(self.trajectory) if self.trajectory is not None else None,
                "age_ms": age,
                "missing_age_ms": (round((time.monotonic()-self.trajectory_at)*1000, 2)
                                   if self.trajectory is None else None),
                "unit": "degree",
                "coordinate_space": "hardware_joint",
                "servo_status": status,
                "lead_limited": self.continuous_target.lead_limited,
                "joint_limit_limited": self.continuous_target.joint_limited,
                "joint_limit_axes": (near_limit_axes(model_to_hardware(self.feedback), math.degrees(.15))
                                     if status.get('code') == 6 and self.feedback is not None
                                     else list(self.continuous_target.joint_limit_axes)),
            }


class RequestError(Exception):
    def __init__(self, status: int, code: str):
        super().__init__(code)
        self.status = status
        self.code = code


def handler_for(node: GatewayNode):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status: int, value: dict) -> None:
            raw = json.dumps(value, ensure_ascii=True, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(raw)

        def body(self) -> dict:
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise RequestError(415, "json_required")
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise RequestError(400, "invalid_content_length") from exc
            if not 0 < length <= 4096:
                raise RequestError(413, "body_size_invalid")
            try:
                value = json.loads(self.rfile.read(length).decode("utf-8"))
            except (ValueError, UnicodeError) as exc:
                raise RequestError(400, "invalid_json") from exc
            if not isinstance(value, dict):
                raise RequestError(400, "json_object_required")
            return value

        def run_request(self) -> None:
            try:
                if self.headers.get("Host", "") not in {"127.0.0.1:8801", "localhost:8801"}:
                    raise RequestError(403, "loopback_host_required")
                path = urlsplit(self.path).path
                if self.command == "GET" and path == "/health":
                    return self.reply(200, node.health())
                if self.command == "GET" and path == "/trajectory":
                    return self.reply(200, node.latest_trajectory())
                if self.command != "POST":
                    raise RequestError(404, "not_found")
                value = self.body()
                if path == "/feedback":
                    if "positions" not in value or set(value) - {"positions", "age_ms"}:
                        raise RequestError(422, "feedback_fields_invalid")
                    positions = value["positions"]
                    if (not isinstance(positions, list) or len(positions) != 6
                            or not all(type(item) in (int, float) and math.isfinite(item)
                                       for item in positions)):
                        raise RequestError(422, "six_finite_axes_required")
                    age_ms = value.get("age_ms", 0.0)
                    if type(age_ms) not in (int, float) or not math.isfinite(age_ms) or age_ms < 0:
                        raise RequestError(422, "feedback_age_invalid")
                    node.set_feedback([float(item) for item in positions], age_ms)
                    return self.reply(200, {"accepted": True, "trajectory": node.latest_trajectory()})
                if path == "/arm":
                    if value != {"confirmation": "ENABLE_LIVE_CONTROL"}:
                        raise RequestError(422, "arm_confirmation_required")
                    okay, reason = node.arm()
                    return self.reply(200 if okay else 503, {"armed": okay, "reason": reason})
                if path == "/disarm":
                    if value:
                        raise RequestError(422, "empty_object_required")
                    node.disarm()
                    return self.reply(200, {"armed": False})
                if path == "/command":
                    if set(value) != COMMAND_FIELDS:
                        raise RequestError(422, "command_fields_invalid")
                    if not node.armed:
                        raise RequestError(409, "gateway_not_armed")
                    if not all(type(item) in (int, float) and math.isfinite(item)
                               for item in value.values()):
                        raise RequestError(422, "finite_command_required")
                    if any(abs(value[name]) > 0.10 for name in ("linear_x", "linear_y", "linear_z")):
                        raise RequestError(422, "linear_speed_out_of_range")
                    if any(abs(value[name]) > 0.24 for name in ("angular_x", "angular_y", "angular_z")):
                        raise RequestError(422, "angular_speed_out_of_range")
                    node.set_command({name: float(value[name]) for name in COMMAND_FIELDS})
                    return self.reply(200, {"accepted": True})
                raise RequestError(404, "not_found")
            except RequestError as exc:
                self.reply(exc.status, {"error": {"code": exc.code}})
            except Exception as exc:
                node.get_logger().error(f"HTTP request failed: {type(exc).__name__}: {exc}")
                self.reply(500, {"error": {"code": "internal_error"}})

        do_GET = run_request
        do_POST = run_request

        def log_message(self, _format: str, *_args) -> None:
            return

    return Handler


def main() -> None:
    rclpy.init()
    node = GatewayNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    spin_thread.start()
    server = ThreadingHTTPServer(("127.0.0.1", 8801), handler_for(node))
    print("Dummy MoveIt safe gateway: http://127.0.0.1:8801 (disarmed)", flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.disarm()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
