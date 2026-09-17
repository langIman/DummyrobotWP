from __future__ import annotations

import copy
import threading
from pathlib import Path
from typing import Any

from .common import ServiceError, atomic_write_json, load_json


DEFAULT_CONFIG: dict[str, Any] = {
    "wrist": {
        "device_id": None,
        "width": 1280,
        "height": 720,
        "fps": 30,
        "jpeg_quality": 85,
    },
    "oak": {
        "mx_id": None,
        "rgb_width": 1280,
        "rgb_height": 720,
        "fps": 30,
        "jpeg_quality": 85,
        "depth_width": 640,
        "depth_height": 360,
        "depth_min_mm": 200,
        "depth_max_mm": 3000,
    },
}

WRIST_RESOLUTIONS = {(1280, 720), (1920, 1080)}
WRIST_FPS = {30, 60, 120}
OAK_RESOLUTIONS = {(640, 360), (1280, 720), (1920, 1080)}
OAK_FPS = {15, 30}


class ConfigStore:
    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.RLock()
        loaded = load_json(path, {})
        self._value = copy.deepcopy(DEFAULT_CONFIG)
        if isinstance(loaded, dict):
            for section in ("wrist", "oak"):
                if isinstance(loaded.get(section), dict):
                    self._value[section].update(loaded[section])
        try:
            self._value["wrist"] = validate_wrist(self._value["wrist"])
            self._value["oak"] = validate_oak(self._value["oak"])
        except ServiceError:
            self._value = copy.deepcopy(DEFAULT_CONFIG)

    def get(self, section: str) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._value[section])

    def set(self, section: str, value: dict[str, Any]) -> None:
        with self._lock:
            self._value[section] = copy.deepcopy(value)
            atomic_write_json(self.path, self._value)

    def all(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._value)


def _require_ints(value: dict[str, Any], fields: tuple[str, ...]) -> None:
    if any(type(value.get(field)) is not int for field in fields):
        raise ServiceError(422, "camera_config_integers_required", "分辨率、帧率和质量必须是整数。")


def _merge(current: dict[str, Any], update: object, allowed: set[str]) -> dict[str, Any]:
    if not isinstance(update, dict):
        raise ServiceError(400, "json_object_required", "请求正文必须是 JSON 对象。")
    unknown = set(update) - allowed
    if unknown:
        raise ServiceError(422, "unknown_fields", "包含不支持的配置字段：" + ", ".join(sorted(unknown)))
    merged = dict(current)
    merged.update(update)
    return merged


def validate_wrist(value: dict[str, Any], update: object | None = None) -> dict[str, Any]:
    allowed = {"device_id", "width", "height", "fps", "jpeg_quality"}
    merged = _merge(value, update, allowed) if update is not None else dict(value)
    _require_ints(merged, ("width", "height", "fps", "jpeg_quality"))
    if merged.get("device_id") is not None and not isinstance(merged["device_id"], str):
        raise ServiceError(422, "invalid_device_id", "腕部相机设备标识必须是字符串或空值。")
    if (merged["width"], merged["height"]) not in WRIST_RESOLUTIONS:
        raise ServiceError(422, "unsupported_camera_resolution", "腕部相机不支持该分辨率。")
    if merged["fps"] not in WRIST_FPS:
        raise ServiceError(422, "unsupported_camera_fps", "腕部相机不支持该帧率。")
    if not 40 <= merged["jpeg_quality"] <= 95:
        raise ServiceError(422, "jpeg_quality_out_of_range", "JPEG 质量必须在 40 到 95 之间。")
    return merged


def validate_oak(value: dict[str, Any], update: object | None = None) -> dict[str, Any]:
    allowed = {
        "mx_id", "rgb_width", "rgb_height", "fps", "jpeg_quality",
        "depth_width", "depth_height", "depth_min_mm", "depth_max_mm",
    }
    merged = _merge(value, update, allowed) if update is not None else dict(value)
    _require_ints(
        merged,
        (
            "rgb_width", "rgb_height", "fps", "jpeg_quality", "depth_width",
            "depth_height", "depth_min_mm", "depth_max_mm",
        ),
    )
    if merged.get("mx_id") is not None and not isinstance(merged["mx_id"], str):
        raise ServiceError(422, "invalid_mx_id", "OAK MX ID 必须是字符串或空值。")
    if (merged["rgb_width"], merged["rgb_height"]) not in OAK_RESOLUTIONS:
        raise ServiceError(422, "unsupported_camera_resolution", "OAK 不支持该 RGB 分辨率。")
    if merged["fps"] not in OAK_FPS:
        raise ServiceError(422, "unsupported_camera_fps", "OAK RGB-D 不支持该帧率。")
    if (merged["depth_width"], merged["depth_height"]) != (640, 360):
        raise ServiceError(422, "unsupported_depth_resolution", "第一版深度输出固定为 640 x 360。")
    if not 40 <= merged["jpeg_quality"] <= 95:
        raise ServiceError(422, "jpeg_quality_out_of_range", "JPEG 质量必须在 40 到 95 之间。")
    if not 0 <= merged["depth_min_mm"] < merged["depth_max_mm"] <= 65_535:
        raise ServiceError(422, "invalid_depth_range", "深度显示范围无效。")
    return merged
