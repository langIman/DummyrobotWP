from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

from .cameras import OakCamera, WristCamera, discover_oak_devices, discover_wrist_cameras
from .common import ServiceError, wall_time_ms
from .configuration import ConfigStore, validate_oak, validate_wrist
from .recording import RecordingManager
from .robot import RobotClient
from .live_control import LiveControl


class PerceptionService:
    def __init__(self, root: Path, bridge_url: str = "http://127.0.0.1:8765",
                 moveit_url: str = "http://127.0.0.1:8801"):
        self.root = root
        self.runtime_dir = root / "runtime"
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.config_store = ConfigStore(root / "config.json")
        self.recorder = RecordingManager(root / "data" / "recordings")
        self.wrist = WristCamera(self.config_store.get("wrist"), self.recorder.submit_wrist)
        self.oak = OakCamera(self.config_store.get("oak"), self.recorder.submit_oak)
        self.recorder.set_camera_health(self.camera_health)
        self.robot = RobotClient(
            bridge_url,
            recording_active=self.recorder.is_active,
            state_callback=self.recorder.record_robot_state,
            event_callback=self.recorder.record_event,
        )
        self.live_control = LiveControl(
            self.robot, moveit_url, event_callback=self.recorder.record_event
        )
        self._closed = False
        self._close_lock = threading.Lock()
        self.wrist.start()
        self.oak.start()

    def camera_health(self) -> dict[str, bool]:
        return {
            "wrist": self.wrist.preview.get(2_000) is not None,
            "oak": (
                self.oak.rgb_preview.get(2_000) is not None
                and self.oak.depth_preview.get(2_000) is not None
                and self.oak.raw_depth(2_000) is not None
            ),
        }

    def health(self) -> dict[str, Any]:
        return {
            "status": "alive",
            "service": "dummyv2-perception",
            "version": "0.8.18",
            "time_ms": wall_time_ms(),
            "listen": "127.0.0.1:8770",
        }

    def status(self) -> dict[str, Any]:
        return {
            "health": self.health(),
            "cameras": {"wrist": self.wrist.status(), "oak": self.oak.status()},
            "recording": self.recorder.current(),
            "disk": self.recorder.disk(),
            "robot": self.robot.status(),
            "control": self.live_control.status(),
        }

    def discover(self) -> dict[str, Any]:
        return {"wrist": discover_wrist_cameras(), "oak": discover_oak_devices()}

    def camera_config(self, camera: str) -> dict[str, Any]:
        if camera == "wrist":
            return {
                "configuration": self.wrist.config(),
                "supported_resolutions": [[1280, 720], [1920, 1080]],
                "supported_fps": [30, 60, 120],
                "jpeg_quality": {"min": 40, "max": 95},
            }
        if camera == "oak":
            return {
                "configuration": self.oak.config(),
                "supported_resolutions": [[640, 360], [1280, 720], [1920, 1080]],
                "supported_fps": [15, 30],
                "jpeg_quality": {"min": 40, "max": 95},
                "depth_resolution": [640, 360],
                "depth_unit": "millimeter",
            }
        raise ServiceError(404, "camera_not_found", "不存在该摄像头。")

    def configure_camera(self, camera: str, update: object) -> dict[str, Any]:
        if self.recorder.is_active():
            raise ServiceError(409, "recording_active", "录制期间不能修改摄像头配置。")
        if camera == "wrist":
            config = validate_wrist(self.wrist.config(), update)
            status = self.wrist.configure(config)
            self.config_store.set("wrist", config)
        elif camera == "oak":
            config = validate_oak(self.oak.config(), update)
            status = self.oak.configure(config)
            self.config_store.set("oak", config)
        else:
            raise ServiceError(404, "camera_not_found", "不存在该摄像头。")
        return {"configuration": config, "status": status, "persisted": True}

    def control_camera(self, camera: str, enabled: bool) -> dict[str, Any]:
        if self.recorder.is_active():
            raise ServiceError(409, "recording_active", "录制期间不能停止摄像头。")
        target = self.wrist if camera == "wrist" else self.oak if camera == "oak" else None
        if target is None:
            raise ServiceError(404, "camera_not_found", "不存在该摄像头。")
        target.set_enabled(enabled)
        return target.status()

    def start_recording(self, body: object) -> dict[str, Any]:
        if body is None:
            body = {}
        if not isinstance(body, dict):
            raise ServiceError(400, "json_object_required", "请求正文必须是 JSON 对象。")
        if set(body) - {"label"}:
            raise ServiceError(422, "unknown_fields", "录制请求只接受 label 字段。")
        robot = self.robot.status()
        warning = None
        if robot["bridge_status"] != "ready":
            warning = "robot_bridge_unavailable"
        elif robot["state"].get("positions") is None:
            warning = "joint_feedback_unavailable"
        return self.recorder.start(
            body.get("label", ""),
            camera_config=self.config_store.all(),
            oak_calibration=self.oak.calibration(),
            robot_warning=warning,
        )

    def record_control_event(self, body: object) -> dict[str, Any]:
        """Store a validated data-collection input without touching robot IO."""
        if not isinstance(body, dict):
            raise ServiceError(400, "json_object_required", "请求正文必须是 JSON 对象。")
        allowed = {"kind", "yaw", "pitch", "keys", "at_client_ms"}
        if set(body) - allowed:
            raise ServiceError(422, "unknown_fields", "控制记录只接受 kind、yaw、pitch、keys 和 at_client_ms。")
        kind = body.get("kind")
        if kind not in {"look", "move"}:
            raise ServiceError(422, "control_kind_invalid", "控制记录类型只能是 look 或 move。")
        values: dict[str, Any] = {"type": "data_collection_input", "kind": kind,
                                  "command_frame": "gripper_tool", "control_protocol_version": 7}
        for name in ("yaw", "pitch"):
            value = body.get(name, 0.0)
            if type(value) not in (int, float) or not (-1.0 <= float(value) <= 1.0):
                raise ServiceError(422, "control_value_invalid", "转向输入必须是 -1 到 1 之间的数值。")
            values[name] = round(float(value), 5)
        keys = body.get("keys", [])
        if not isinstance(keys, list) or any(key not in {"w", "a", "s", "d", "arrowup", "arrowdown"} for key in keys):
            raise ServiceError(422, "control_keys_invalid", "移动按键只能包含 W、A、S、D、↑、↓。")
        values["keys"] = sorted(set(keys))
        client_ms = body.get("at_client_ms")
        if client_ms is not None and (type(client_ms) is not int or client_ms < 0):
            raise ServiceError(422, "client_timestamp_invalid", "客户端时间戳必须是非负整数。")
        if client_ms is not None:
            values["at_client_ms"] = client_ms
        if not self.recorder.is_active():
            raise ServiceError(409, "recording_not_active", "请先开始录制，再记录数据采集输入。")
        self.recorder.record_event(values)
        return {"accepted": True, "recording": self.recorder.current()}

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
        self.live_control.close()
        self.recorder.close()
        self.wrist.close()
        self.oak.close()
        self.robot.close()
