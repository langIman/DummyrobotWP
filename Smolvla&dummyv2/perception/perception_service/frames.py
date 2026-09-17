from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from .common import monotonic_ns, wall_time_ms


@dataclass(frozen=True)
class EncodedFrame:
    frame_id: int
    captured_at_ms: int
    monotonic_ns: int
    data: bytes
    width: int
    height: int
    content_type: str = "image/jpeg"
    device_timestamp_ns: int | None = None

    def age_ms(self) -> int:
        return max(0, (monotonic_ns() - self.monotonic_ns) // 1_000_000)


class FrameStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: EncodedFrame | None = None

    def put(
        self,
        frame_id: int,
        data: bytes,
        width: int,
        height: int,
        *,
        captured_at_ms: int | None = None,
        captured_monotonic_ns: int | None = None,
        content_type: str = "image/jpeg",
        device_timestamp_ns: int | None = None,
    ) -> EncodedFrame:
        frame = EncodedFrame(
            frame_id=frame_id,
            captured_at_ms=captured_at_ms if captured_at_ms is not None else wall_time_ms(),
            monotonic_ns=captured_monotonic_ns if captured_monotonic_ns is not None else monotonic_ns(),
            data=data,
            width=width,
            height=height,
            content_type=content_type,
            device_timestamp_ns=device_timestamp_ns,
        )
        with self._condition:
            self._frame = frame
            self._condition.notify_all()
        return frame

    def get(self, max_age_ms: int | None = None) -> EncodedFrame | None:
        with self._condition:
            frame = self._frame
        if frame is not None and max_age_ms is not None and frame.age_ms() > max_age_ms:
            return None
        return frame

    def wait_after(self, frame_id: int, timeout: float = 1.0) -> EncodedFrame | None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._frame is not None and self._frame.frame_id != frame_id,
                timeout=timeout,
            )
            return self._frame

    def clear(self) -> None:
        with self._condition:
            self._frame = None
            self._condition.notify_all()

    def metadata(self) -> dict[str, Any]:
        frame = self.get()
        if frame is None:
            return {
                "frame_available": False,
                "frame_id": None,
                "captured_at_ms": None,
                "frame_age_ms": None,
                "dimensions": None,
            }
        return {
            "frame_available": frame.age_ms() <= 2_000,
            "frame_id": frame.frame_id,
            "captured_at_ms": frame.captured_at_ms,
            "frame_age_ms": frame.age_ms(),
            "dimensions": [frame.width, frame.height],
        }

