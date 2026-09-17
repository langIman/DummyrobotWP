from __future__ import annotations

import bisect
import json
import queue
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from .common import (
    ServiceError,
    append_jsonl,
    atomic_write_json,
    directory_size,
    disk_status,
    monotonic_ns,
    safe_label,
    wall_time_ms,
)


GIB = 1024**3


class RecordingManager:
    def __init__(self, root: Path, camera_health: Callable[[], dict[str, bool]] | None = None):
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        self._camera_health = camera_health or (lambda: {"wrist": False, "oak": False})
        self._lock = threading.RLock()
        self._file_lock = threading.Lock()
        self._active: dict[str, Any] | None = None
        self._last: dict[str, Any] | None = None
        self._shutdown = threading.Event()
        self._watchdog = threading.Thread(target=self._watch, name="recording-watchdog", daemon=True)
        self._watchdog.start()

    def set_camera_health(self, callback: Callable[[], dict[str, bool]]) -> None:
        self._camera_health = callback

    def disk(self) -> dict[str, int | float]:
        return disk_status(self.root)

    def is_active(self) -> bool:
        with self._lock:
            return bool(self._active and self._active.get("accepting"))

    def start(
        self,
        label: object,
        *,
        camera_config: dict[str, Any],
        oak_calibration: dict[str, Any] | None,
        robot_warning: str | None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._active is not None:
                raise ServiceError(409, "recording_active", "已有录制正在进行。")
            free = int(self.disk()["free_bytes"])
            if free < 8 * GIB:
                raise ServiceError(409, "insufficient_disk_space", "剩余空间不足 8 GB，不能开始录制。")
            health = self._camera_health()
            if not health.get("wrist") or not health.get("oak"):
                raise ServiceError(409, "cameras_not_ready", "腕部与 OAK RGB-D 都有新鲜帧后才能录制。")

            clean_label = safe_label(label)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_id = f"{stamp}_{uuid.uuid4().hex[:8]}"
            if clean_label:
                session_id += f"_{clean_label}"
            path = self.root / session_id
            (path / "wrist" / "rgb").mkdir(parents=True)
            (path / "oak" / "rgb").mkdir(parents=True)
            (path / "oak" / "depth").mkdir(parents=True)

            started_wall = wall_time_ms()
            started_mono = monotonic_ns()
            active: dict[str, Any] = {
                "session_id": session_id,
                "label": clean_label,
                "path": path,
                "status": "recording",
                "reason": None,
                "accepting": True,
                "started_at_ms": started_wall,
                "started_monotonic_ns": started_mono,
                "counts": {"wrist": 0, "oak": 0, "robot": 0, "events": 0},
                "dropped": {"wrist": 0, "oak": 0},
                "errors": [],
                "wrist_timeline": [],
                "oak_timeline": [],
                "robot_timeline": [],
                "wrist_queue": queue.Queue(maxsize=90),
                "oak_queue": queue.Queue(maxsize=60),
                "threads": [],
                "camera_config": camera_config,
                "robot_warning": robot_warning,
            }
            session = {
                "schema_version": 1,
                "session_id": session_id,
                "label": clean_label,
                "status": "recording",
                "reason": None,
                "started_at_ms": started_wall,
                "started_monotonic_ns": started_mono,
                "ended_at_ms": None,
                "duration_ms": None,
                "camera_config": camera_config,
                "robot_warning": robot_warning,
                "depth": {"unit": "millimeter", "dtype": "uint16", "invalid_value": 0},
                "cross_camera_sync": "host_monotonic_nearest_neighbor",
            }
            atomic_write_json(path / "session.json", session)
            if oak_calibration:
                atomic_write_json(path / "oak" / "calibration.json", oak_calibration)

            wrist_thread = threading.Thread(
                target=self._wrist_writer, args=(active,), name=f"wrist-writer-{session_id}", daemon=True
            )
            oak_thread = threading.Thread(
                target=self._oak_writer, args=(active,), name=f"oak-writer-{session_id}", daemon=True
            )
            active["threads"] = [wrist_thread, oak_thread]
            self._active = active
            wrist_thread.start()
            oak_thread.start()
            return self.current()

    def submit_wrist(self, metadata: dict[str, Any], jpeg: bytes) -> None:
        with self._lock:
            active = self._active
            if not active or not active["accepting"]:
                return
            target = active["wrist_queue"]
        try:
            target.put_nowait((dict(metadata), bytes(jpeg)))
        except queue.Full:
            with self._lock:
                if self._active is active:
                    active["dropped"]["wrist"] += 1

    def submit_oak(self, metadata: dict[str, Any], rgb_jpeg: bytes, depth: Any) -> None:
        with self._lock:
            active = self._active
            if not active or not active["accepting"]:
                return
            target = active["oak_queue"]
        try:
            target.put_nowait((dict(metadata), bytes(rgb_jpeg), depth.copy()))
        except queue.Full:
            with self._lock:
                if self._active is active:
                    active["dropped"]["oak"] += 1

    def record_robot_state(self, state: dict[str, Any]) -> None:
        with self._lock:
            active = self._active
            if not active or not active["accepting"]:
                return
            path = active["path"] / "robot_state.jsonl"
            sequence = active["counts"]["robot"]
            row = {"sequence": sequence, **state}
            active["counts"]["robot"] += 1
            positions = state.get("positions")
            if positions is not None and state.get("monotonic_ns") is not None:
                active["robot_timeline"].append((state["monotonic_ns"], sequence))
        with self._file_lock:
            try:
                append_jsonl(path, row)
            except Exception as exc:
                self._metadata_write_failed(active, "robot", exc)

    def record_event(self, event: dict[str, Any]) -> None:
        with self._lock:
            active = self._active
            if not active or not active["accepting"]:
                return
            path = active["path"] / "events.jsonl"
            sequence = active["counts"]["events"]
            row = {"sequence": sequence, "at_ms": wall_time_ms(), "monotonic_ns": monotonic_ns(), **event}
            active["counts"]["events"] += 1
        with self._file_lock:
            try:
                append_jsonl(path, row)
            except Exception as exc:
                self._metadata_write_failed(active, "events", exc)

    def _metadata_write_failed(self, active: dict[str, Any], stream: str, exc: Exception) -> None:
        with self._lock:
            if self._active is active:
                active["errors"].append(
                    {"stream": stream, "error": f"{type(exc).__name__}: {exc}"}
                )

    def _wrist_writer(self, active: dict[str, Any]) -> None:
        target_dir: Path = active["path"] / "wrist" / "rgb"
        metadata_path: Path = active["path"] / "wrist" / "frames.jsonl"
        q: queue.Queue[Any] = active["wrist_queue"]
        try:
            while True:
                item = q.get()
                if item is None:
                    break
                metadata, jpeg = item
                with self._lock:
                    saved_sequence = active["counts"]["wrist"]
                filename = f"{saved_sequence:08d}.jpg"
                (target_dir / filename).write_bytes(jpeg)
                row = {"sequence": saved_sequence, "file": f"rgb/{filename}", **metadata}
                append_jsonl(metadata_path, row)
                with self._lock:
                    active["counts"]["wrist"] += 1
                    active["wrist_timeline"].append((metadata["monotonic_ns"], saved_sequence))
        except Exception as exc:
            self._writer_failed(active, "wrist", exc)

    def _oak_writer(self, active: dict[str, Any]) -> None:
        import cv2

        rgb_dir: Path = active["path"] / "oak" / "rgb"
        depth_dir: Path = active["path"] / "oak" / "depth"
        metadata_path: Path = active["path"] / "oak" / "frames.jsonl"
        q: queue.Queue[Any] = active["oak_queue"]
        try:
            while True:
                item = q.get()
                if item is None:
                    break
                metadata, rgb_jpeg, depth = item
                with self._lock:
                    saved_sequence = active["counts"]["oak"]
                rgb_name = f"{saved_sequence:08d}.jpg"
                depth_name = f"{saved_sequence:08d}.png"
                (rgb_dir / rgb_name).write_bytes(rgb_jpeg)
                ok, encoded = cv2.imencode(".png", depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not ok:
                    raise OSError("depth_png_encode_failed")
                (depth_dir / depth_name).write_bytes(encoded.tobytes())
                row = {
                    "sequence": saved_sequence,
                    "rgb_file": f"rgb/{rgb_name}",
                    "depth_file": f"depth/{depth_name}",
                    **metadata,
                }
                append_jsonl(metadata_path, row)
                with self._lock:
                    active["counts"]["oak"] += 1
                    active["oak_timeline"].append((metadata["monotonic_ns"], saved_sequence))
        except Exception as exc:
            self._writer_failed(active, "oak", exc)

    def _writer_failed(self, active: dict[str, Any], stream: str, exc: Exception) -> None:
        with self._lock:
            active["errors"].append({"stream": stream, "error": f"{type(exc).__name__}: {exc}"})

    def _watch(self) -> None:
        while not self._shutdown.wait(0.5):
            with self._lock:
                active = self._active
                if not active or not active["accepting"]:
                    continue
                elapsed = (monotonic_ns() - active["started_monotonic_ns"]) / 1_000_000_000
                errors = bool(active["errors"])
            if elapsed >= 300:
                self.stop("max_duration", complete=True)
                continue
            if errors:
                self.stop("write_error", complete=False)
                continue
            if int(self.disk()["free_bytes"]) < 5 * GIB:
                self.stop("low_disk_space", complete=False)
                continue
            health = self._camera_health()
            if not health.get("wrist") or not health.get("oak"):
                self.stop("camera_stream_lost", complete=False)

    def stop(self, reason: str = "user_stop", *, complete: bool = True) -> dict[str, Any]:
        with self._lock:
            active = self._active
            if active is None:
                raise ServiceError(409, "recording_not_active", "当前没有正在进行的录制。")
            if not active["accepting"]:
                return self._snapshot(active)
            active["accepting"] = False
            active["status"] = "stopping"
            active["reason"] = reason
            queues = [active["wrist_queue"], active["oak_queue"]]
            threads = list(active["threads"])

        for target in queues:
            try:
                target.put(None, timeout=15)
            except queue.Full:
                active["errors"].append({"stream": "writer", "error": "writer_queue_did_not_drain"})
                complete = False
        for thread in threads:
            thread.join(timeout=30)
            if thread.is_alive():
                active["errors"].append({"stream": thread.name, "error": "writer_shutdown_timeout"})
                complete = False

        if active["errors"]:
            complete = False
        self._write_alignment(active)
        ended = wall_time_ms()
        final = {
            "schema_version": 1,
            "session_id": active["session_id"],
            "label": active["label"],
            "status": "complete" if complete else "incomplete",
            "reason": reason,
            "started_at_ms": active["started_at_ms"],
            "started_monotonic_ns": active["started_monotonic_ns"],
            "ended_at_ms": ended,
            "duration_ms": max(0, ended - active["started_at_ms"]),
            "camera_config": active["camera_config"],
            "robot_warning": active["robot_warning"],
            "depth": {"unit": "millimeter", "dtype": "uint16", "invalid_value": 0},
            "cross_camera_sync": "host_monotonic_nearest_neighbor",
            "counts": dict(active["counts"]),
            "dropped": dict(active["dropped"]),
            "errors": list(active["errors"]),
        }
        atomic_write_json(active["path"] / "session.json", final)
        final["size_bytes"] = directory_size(active["path"])
        final["path"] = str(active["path"])
        with self._lock:
            active["status"] = final["status"]
            self._last = final
            if self._active is active:
                self._active = None
        return dict(final)

    @staticmethod
    def _nearest(timeline: list[tuple[int, int]], target: int) -> tuple[int | None, float | None]:
        if not timeline:
            return None, None
        times = [entry[0] for entry in timeline]
        position = bisect.bisect_left(times, target)
        candidates = []
        if position < len(timeline):
            candidates.append(timeline[position])
        if position:
            candidates.append(timeline[position - 1])
        timestamp, sequence = min(candidates, key=lambda item: abs(item[0] - target))
        return sequence, round((timestamp - target) / 1_000_000, 3)

    def _write_alignment(self, active: dict[str, Any]) -> None:
        path: Path = active["path"] / "alignment.jsonl"
        wrist = sorted(active["wrist_timeline"])
        robot = sorted(active["robot_timeline"])
        with path.open("w", encoding="utf-8", newline="\n") as stream:
            for oak_time, oak_sequence in sorted(active["oak_timeline"]):
                wrist_sequence, wrist_delta = self._nearest(wrist, oak_time)
                robot_sequence, robot_delta = self._nearest(robot, oak_time)
                row = {
                    "oak_sequence": oak_sequence,
                    "wrist_sequence": wrist_sequence,
                    "wrist_delta_ms": wrist_delta,
                    "wrist_within_50ms": wrist_delta is not None and abs(wrist_delta) <= 50,
                    "robot_sequence": robot_sequence,
                    "robot_delta_ms": robot_delta,
                }
                stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")

    def _snapshot(self, active: dict[str, Any]) -> dict[str, Any]:
        elapsed = max(0, wall_time_ms() - active["started_at_ms"])
        return {
            "active": active["accepting"],
            "session_id": active["session_id"],
            "label": active["label"],
            "status": active["status"],
            "reason": active["reason"],
            "started_at_ms": active["started_at_ms"],
            "duration_ms": elapsed,
            "remaining_ms": max(0, 300_000 - elapsed),
            "counts": dict(active["counts"]),
            "dropped": dict(active["dropped"]),
            "errors": list(active["errors"]),
            "path": str(active["path"]),
        }

    def current(self) -> dict[str, Any]:
        with self._lock:
            if self._active:
                return self._snapshot(self._active)
            return {"active": False, "status": "idle", "last": self._last}

    def list_sessions(self, limit: int = 30) -> list[dict[str, Any]]:
        sessions = []
        for path in sorted(self.root.glob("*/session.json"), reverse=True)[:limit]:
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
                value["path"] = str(path.parent)
                value["size_bytes"] = directory_size(path.parent)
                sessions.append(value)
            except (OSError, ValueError):
                sessions.append({"session_id": path.parent.name, "status": "unreadable", "path": str(path.parent)})
        return sessions

    def close(self) -> None:
        self._shutdown.set()
        if self.is_active():
            try:
                self.stop("service_shutdown", complete=False)
            except ServiceError:
                pass
        self._watchdog.join(timeout=1)
