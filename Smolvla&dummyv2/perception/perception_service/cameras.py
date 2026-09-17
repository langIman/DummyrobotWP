from __future__ import annotations

import json
import threading
import time
from collections import deque
from datetime import timedelta
from typing import Any, Callable

from .common import ServiceError, fourcc_text, monotonic_ns, wall_time_ms
from .frames import FrameStore


FrameCallback = Callable[..., None]


def discover_wrist_cameras() -> list[dict[str, Any]]:
    try:
        import cv2
        from cv2_enumerate_cameras import enumerate_cameras

        result = []
        for info in enumerate_cameras(cv2.CAP_DSHOW):
            path = str(info.path) if info.path else ""
            device_id = path or f"dshow:{info.index}:{info.name}"
            lowered = str(info.name).lower()
            is_virtual = any(
                marker in lowered
                for marker in ("todesk", "obs", "virtual", "manycam", "snap camera")
            )
            result.append(
                {
                    "device_id": device_id,
                    "name": str(info.name),
                    "path": path or None,
                    "vid": info.vid,
                    "pid": info.pid,
                    "index": info.index,
                    "backend": info.backend,
                    "is_virtual": is_virtual,
                    "auto_candidate": not is_virtual and bool(path or info.vid or info.pid),
                }
            )
        return result
    except Exception:
        return []


def discover_oak_devices() -> list[dict[str, Any]]:
    try:
        import depthai as dai

        result = []
        for info in dai.Device.getAllAvailableDevices():
            result.append(
                {
                    "mx_id": str(info.getDeviceId()),
                    "state": str(getattr(info, "state", "unknown")),
                    "protocol": str(getattr(info, "protocol", "unknown")),
                }
            )
        return result
    except Exception:
        return []


class CameraWorkerBase:
    def __init__(self, name: str, config: dict[str, Any]):
        self.name = name
        self.preview = FrameStore()
        self._condition = threading.Condition(threading.RLock())
        self._config = dict(config)
        self._enabled = True
        self._shutdown = False
        self._restart = False
        self._revision = 0
        self._applied_revision = -1
        self._failed_revision = -1
        self._state = "starting"
        self._reason: str | None = None
        self._selected_device: dict[str, Any] | None = None
        self._actual_mode: dict[str, Any] | None = None
        self._frame_count = 0
        self._frame_times: deque[float] = deque(maxlen=240)
        self._thread = threading.Thread(target=self._run, name=f"{name}-capture", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        backoff = 1.0
        while True:
            with self._condition:
                if self._shutdown:
                    return
                if not self._enabled:
                    self._state = "stopped"
                    self._reason = None
                    self._selected_device = None
                    self._actual_mode = None
                    self.preview.clear()
                    self._condition.wait_for(lambda: self._enabled or self._shutdown)
                    continue
                self._restart = False
                revision = self._revision
                config = dict(self._config)
                self._state = "connecting"
                self._reason = None
                self._condition.notify_all()
            try:
                self._capture_session(config, revision)
                backoff = 1.0
            except Exception as exc:
                with self._condition:
                    self._state = "unavailable"
                    self._reason = f"{type(exc).__name__}: {exc}"
                    self._actual_mode = None
                    self.preview.clear()
                    if revision == self._revision and revision > self._applied_revision:
                        self._failed_revision = max(self._failed_revision, revision)
                    self._condition.notify_all()
                deadline = time.monotonic() + backoff
                while time.monotonic() < deadline:
                    with self._condition:
                        if self._shutdown or self._restart or not self._enabled:
                            break
                    time.sleep(0.05)
                backoff = min(backoff * 2, 8.0)

    def _capture_session(self, config: dict[str, Any], revision: int) -> None:
        raise NotImplementedError

    def _should_stop_session(self, revision: int) -> bool:
        with self._condition:
            return self._shutdown or not self._enabled or self._restart or revision != self._revision

    def _frame_arrived(
        self,
        revision: int,
        selected_device: dict[str, Any],
        actual_mode: dict[str, Any],
    ) -> None:
        with self._condition:
            if revision != self._revision:
                return
            self._selected_device = dict(selected_device)
            self._actual_mode = dict(actual_mode)
            self._state = "streaming"
            self._reason = None
            self._applied_revision = max(self._applied_revision, revision)
            self._frame_count += 1
            self._frame_times.append(time.monotonic())
            self._condition.notify_all()

    def configure(self, config: dict[str, Any], timeout: float = 12.0) -> dict[str, Any]:
        with self._condition:
            if not self._enabled:
                raise ServiceError(409, "camera_not_running", "摄像头启动后才能验证并保存配置。")
            previous = dict(self._config)
            self._config = dict(config)
            self._revision += 1
            revision = self._revision
            self._restart = True
            self._condition.notify_all()
            applied = self._condition.wait_for(
                lambda: self._applied_revision >= revision or self._failed_revision >= revision or self._shutdown,
                timeout=timeout,
            )
            if applied and self._applied_revision >= revision:
                return self.status()
            failure_reason = self._reason
            self._config = previous
            self._revision += 1
            self._restart = True
            self._condition.notify_all()
        raise ServiceError(
            503,
            "camera_configuration_failed",
            "摄像头未能采用新配置，已恢复上次配置。" + (f" ({failure_reason})" if failure_reason else ""),
        )

    def set_enabled(self, enabled: bool) -> None:
        with self._condition:
            if self._enabled == enabled:
                return
            self._enabled = enabled
            self._restart = True
            self._condition.notify_all()
            if not enabled:
                self._condition.wait_for(
                    lambda: self._state == "stopped" or self._shutdown,
                    timeout=5,
                )
        if not enabled:
            self.preview.clear()

    def status(self) -> dict[str, Any]:
        with self._condition:
            now = time.monotonic()
            recent = [stamp for stamp in self._frame_times if now - stamp <= 4]
            fps = (
                (len(recent) - 1) / (recent[-1] - recent[0])
                if len(recent) > 1 and now - recent[-1] < 2
                else 0.0
            )
            result = {
                "camera": self.name,
                "enabled": self._enabled,
                "state": self._state,
                "reason": self._reason,
                "requested_mode": dict(self._config),
                "actual_mode": dict(self._actual_mode) if self._actual_mode else None,
                "selected_device": dict(self._selected_device) if self._selected_device else None,
                "measured_fps": round(fps, 2),
                "frames": self._frame_count,
                "applied_revision": self._applied_revision,
            }
        result.update(self.preview.metadata())
        return result

    def config(self) -> dict[str, Any]:
        with self._condition:
            return dict(self._config)

    def close(self) -> None:
        with self._condition:
            self._shutdown = True
            self._restart = True
            self._condition.notify_all()
        self._thread.join(timeout=3)


class WristCamera(CameraWorkerBase):
    def __init__(self, config: dict[str, Any], on_frame: FrameCallback):
        super().__init__("wrist", config)
        self._on_frame = on_frame

    @staticmethod
    def discover() -> list[dict[str, Any]]:
        return discover_wrist_cameras()

    @staticmethod
    def _resolve(config: dict[str, Any]) -> dict[str, Any]:
        devices = discover_wrist_cameras()
        requested = config.get("device_id")
        if requested:
            matches = [device for device in devices if device["device_id"] == requested]
            if len(matches) == 1:
                return matches[0]
            raise OSError("configured_wrist_camera_not_found")
        preferred = [
            device for device in devices
            if "decxin" in device["name"].lower() or "ar0234" in device["name"].lower()
        ]
        physical = [device for device in devices if device.get("auto_candidate")]
        candidates = preferred or physical
        if not candidates:
            raise OSError("wrist_camera_not_found")
        if len(candidates) > 1:
            raise OSError("ambiguous_wrist_cameras_select_device")
        return candidates[0]

    def _capture_session(self, config: dict[str, Any], revision: int) -> None:
        import cv2

        device = self._resolve(config)
        capture = cv2.VideoCapture(device["index"], device["backend"])
        if not capture.isOpened():
            capture.release()
            raise OSError("wrist_camera_open_failed")
        try:
            # This exact order is required by the verified DECXIN driver.
            applied = {
                "width": bool(capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])),
                "height": bool(capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])),
                "fps": bool(capture.set(cv2.CAP_PROP_FPS, config["fps"])),
                "fourcc": bool(
                    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                ),
            }
            driver_mode = {
                "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": round(float(capture.get(cv2.CAP_PROP_FPS)), 3),
                "fourcc": fourcc_text(capture.get(cv2.CAP_PROP_FOURCC)),
                "property_set_return": applied,
            }
            if (
                driver_mode["width"] != config["width"]
                or driver_mode["height"] != config["height"]
                or driver_mode["fourcc"] != "MJPG"
            ):
                raise OSError("wrist_camera_mode_not_applied")

            while not self._should_stop_session(revision):
                ok, frame = capture.read()
                captured_mono = monotonic_ns()
                captured_wall = wall_time_ms()
                if not ok or frame is None:
                    raise OSError("wrist_camera_frame_read_failed")
                ok, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, config["jpeg_quality"]]
                )
                if not ok:
                    raise OSError("wrist_camera_jpeg_encode_failed")
                frame_id = self._frame_count + 1
                jpeg = encoded.tobytes()
                height, width = frame.shape[:2]
                self.preview.put(
                    frame_id,
                    jpeg,
                    width,
                    height,
                    captured_at_ms=captured_wall,
                    captured_monotonic_ns=captured_mono,
                )
                self._frame_arrived(revision, device, driver_mode)
                self._on_frame(
                    {
                        "source_frame_id": frame_id,
                        "captured_at_ms": captured_wall,
                        "monotonic_ns": captured_mono,
                        "width": width,
                        "height": height,
                    },
                    jpeg,
                )
        finally:
            capture.release()


class OakCamera(CameraWorkerBase):
    def __init__(self, config: dict[str, Any], on_frame: FrameCallback):
        super().__init__("oak", config)
        self.rgb_preview = self.preview
        self.depth_preview = FrameStore()
        self._raw_depth_lock = threading.Lock()
        self._raw_depth: tuple[Any, dict[str, Any]] | None = None
        self._on_frame = on_frame
        self._calibration: dict[str, Any] | None = None

    @staticmethod
    def discover() -> list[dict[str, Any]]:
        return discover_oak_devices()

    @staticmethod
    def _resolve(config: dict[str, Any]) -> dict[str, Any]:
        devices = discover_oak_devices()
        requested = config.get("mx_id")
        if requested:
            matches = [device for device in devices if device["mx_id"] == requested]
            if len(matches) == 1:
                return matches[0]
            raise OSError("configured_oak_not_found")
        if not devices:
            raise OSError("oak_not_found")
        if len(devices) > 1:
            raise OSError("ambiguous_oak_devices_select_mx_id")
        return devices[0]

    @staticmethod
    def _device_timestamp_ns(message: Any) -> int | None:
        try:
            return int(message.getTimestampDevice().total_seconds() * 1_000_000_000)
        except Exception:
            return None

    @staticmethod
    def _calibration_dict(device: Any, dai: Any, config: dict[str, Any], mx_id: str) -> dict[str, Any]:
        result: dict[str, Any] = {"mx_id": mx_id, "available": False}
        try:
            calibration = device.readCalibration()
            cameras = {}
            sizes = {
                "rgb": (dai.CameraBoardSocket.CAM_A, config["rgb_width"], config["rgb_height"]),
                "left": (dai.CameraBoardSocket.CAM_B, 640, 400),
                "right": (dai.CameraBoardSocket.CAM_C, 640, 400),
            }
            for name, (socket, width, height) in sizes.items():
                cameras[name] = {
                    "socket": str(socket),
                    "resolution": [width, height],
                    "intrinsics": calibration.getCameraIntrinsics(socket, width, height),
                    "distortion": calibration.getDistortionCoefficients(socket),
                }
            result.update(
                available=True,
                cameras=cameras,
                rgb_to_left=calibration.getCameraExtrinsics(
                    dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B
                ),
                rgb_to_right=calibration.getCameraExtrinsics(
                    dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_C
                ),
            )
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
        return result

    @staticmethod
    def _colorize_depth(depth: Any, minimum: int, maximum: int) -> Any:
        import cv2
        import numpy as np

        invalid = depth == 0
        clipped = np.clip(depth, minimum, maximum)
        normalized = ((clipped - minimum) * (255.0 / max(1, maximum - minimum))).astype(np.uint8)
        normalized = 255 - normalized
        normalized[invalid] = 0
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        colored[invalid] = 0
        return colored

    def _capture_session(self, config: dict[str, Any], revision: int) -> None:
        import cv2
        import depthai as dai

        selected = self._resolve(config)
        info = dai.DeviceInfo(selected["mx_id"])
        device = dai.Device(info)
        pipeline = dai.Pipeline(device)
        try:
            rgb_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            left_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
            stereo.setLeftRightCheck(True)
            if hasattr(stereo, "setOutputSize"):
                stereo.setOutputSize(config["depth_width"], config["depth_height"])
            sync = pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(timedelta(seconds=1 / (2 * config["fps"])))

            rgb_output = rgb_camera.requestOutput(
                size=(config["rgb_width"], config["rgb_height"]),
                fps=float(config["fps"]),
                enableUndistortion=True,
            )
            left_output = left_camera.requestOutput(size=(640, 400), fps=float(config["fps"]))
            right_output = right_camera.requestOutput(size=(640, 400), fps=float(config["fps"]))
            left_output.link(stereo.left)
            right_output.link(stereo.right)
            rgb_output.link(stereo.inputAlignTo)
            rgb_output.link(sync.inputs["rgb"])
            stereo.depth.link(sync.inputs["depth"])
            output_queue = sync.out.createOutputQueue()
            if hasattr(output_queue, "setMaxSize"):
                output_queue.setMaxSize(4)
            if hasattr(output_queue, "setBlocking"):
                output_queue.setBlocking(False)

            pipeline.start()
            selected["usb_speed"] = str(device.getUsbSpeed()) if hasattr(device, "getUsbSpeed") else "unknown"
            selected["usb3"] = "SUPER" in selected["usb_speed"]
            self._calibration = self._calibration_dict(device, dai, config, selected["mx_id"])
            while not self._should_stop_session(revision):
                group = output_queue.tryGet() if hasattr(output_queue, "tryGet") else None
                if group is None:
                    time.sleep(0.005)
                    continue
                rgb_message = group["rgb"]
                depth_message = group["depth"]
                rgb = rgb_message.getCvFrame()
                depth = depth_message.getFrame()
                if rgb is None or depth is None:
                    continue
                if depth.dtype.name != "uint16":
                    depth = depth.astype("uint16")
                desired_depth = (config["depth_width"], config["depth_height"])
                source_depth_dimensions = [int(depth.shape[1]), int(depth.shape[0])]
                if (depth.shape[1], depth.shape[0]) != desired_depth:
                    depth = cv2.resize(depth, desired_depth, interpolation=cv2.INTER_NEAREST)

                captured_mono = monotonic_ns()
                captured_wall = wall_time_ms()
                device_timestamp = self._device_timestamp_ns(rgb_message)
                ok, rgb_encoded = cv2.imencode(
                    ".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, config["jpeg_quality"]]
                )
                if not ok:
                    raise OSError("oak_rgb_jpeg_encode_failed")
                depth_color = self._colorize_depth(
                    depth, config["depth_min_mm"], config["depth_max_mm"]
                )
                ok, depth_encoded = cv2.imencode(
                    ".jpg", depth_color, [cv2.IMWRITE_JPEG_QUALITY, 85]
                )
                if not ok:
                    raise OSError("oak_depth_preview_encode_failed")

                frame_id = self._frame_count + 1
                rgb_jpeg = rgb_encoded.tobytes()
                rgb_height, rgb_width = rgb.shape[:2]
                if (rgb_width, rgb_height) != (config["rgb_width"], config["rgb_height"]):
                    raise OSError("oak_rgb_mode_not_applied")
                self.rgb_preview.put(
                    frame_id,
                    rgb_jpeg,
                    rgb_width,
                    rgb_height,
                    captured_at_ms=captured_wall,
                    captured_monotonic_ns=captured_mono,
                    device_timestamp_ns=device_timestamp,
                )
                self.depth_preview.put(
                    frame_id,
                    depth_encoded.tobytes(),
                    depth.shape[1],
                    depth.shape[0],
                    captured_at_ms=captured_wall,
                    captured_monotonic_ns=captured_mono,
                    device_timestamp_ns=device_timestamp,
                )
                raw_metadata = {
                    "source_frame_id": frame_id,
                    "captured_at_ms": captured_wall,
                    "monotonic_ns": captured_mono,
                    "device_timestamp_ns": device_timestamp,
                    "rgb_width": rgb_width,
                    "rgb_height": rgb_height,
                    "depth_width": int(depth.shape[1]),
                    "depth_height": int(depth.shape[0]),
                    "source_depth_dimensions": source_depth_dimensions,
                }
                with self._raw_depth_lock:
                    self._raw_depth = (depth.copy(), dict(raw_metadata))
                actual_mode = {
                    "rgb_width": rgb_width,
                    "rgb_height": rgb_height,
                    "depth_width": int(depth.shape[1]),
                    "depth_height": int(depth.shape[0]),
                    "fps": config["fps"],
                    "depth_unit": "millimeter",
                    "depth_dtype": "uint16",
                }
                self._frame_arrived(revision, selected, actual_mode)
                self._on_frame(raw_metadata, rgb_jpeg, depth)
        finally:
            try:
                if hasattr(pipeline, "stop"):
                    pipeline.stop()
            finally:
                device.close()

    def raw_depth(self, max_age_ms: int = 2_000) -> tuple[Any, dict[str, Any]] | None:
        with self._raw_depth_lock:
            value = self._raw_depth
            if value is None:
                return None
            depth, metadata = value
            age = (monotonic_ns() - metadata["monotonic_ns"]) // 1_000_000
            if age > max_age_ms:
                return None
            return depth.copy(), dict(metadata)

    def calibration(self) -> dict[str, Any] | None:
        with self._condition:
            return json.loads(json.dumps(self._calibration)) if self._calibration else None

    def status(self) -> dict[str, Any]:
        result = super().status()
        result["rgb"] = self.rgb_preview.metadata()
        result["depth"] = self.depth_preview.metadata()
        result["calibration_available"] = bool(self._calibration and self._calibration.get("available"))
        return result

    def set_enabled(self, enabled: bool) -> None:
        super().set_enabled(enabled)
        if not enabled:
            self.depth_preview.clear()
            with self._raw_depth_lock:
                self._raw_depth = None

    def close(self) -> None:
        super().close()
        self.depth_preview.clear()
