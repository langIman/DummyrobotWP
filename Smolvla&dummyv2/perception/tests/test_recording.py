import json
from pathlib import Path

import cv2
import numpy as np

from perception_service.recording import RecordingManager


def test_recording_writes_frames_metadata_and_alignment(tmp_path: Path):
    manager = RecordingManager(tmp_path, lambda: {"wrist": True, "oak": True})
    manager.disk = lambda: {"free_bytes": 20 * 1024**3, "free_gb": 20.0}
    current = manager.start(
        "测试 clip",
        camera_config={"wrist": {}, "oak": {}},
        oak_calibration={"available": True},
        robot_warning=None,
    )
    base = 10_000_000_000
    manager.submit_wrist(
        {"source_frame_id": 1, "captured_at_ms": 1, "monotonic_ns": base, "width": 2, "height": 2},
        b"jpeg-bytes",
    )
    manager.submit_oak(
        {
            "source_frame_id": 1,
            "captured_at_ms": 1,
            "monotonic_ns": base + 8_000_000,
            "device_timestamp_ns": 5,
            "rgb_width": 2,
            "rgb_height": 2,
            "depth_width": 2,
            "depth_height": 2,
            "source_depth_dimensions": [2, 2],
        },
        b"oak-jpeg",
        np.array([[0, 500], [1000, 2000]], dtype=np.uint16),
    )
    manager.record_robot_state({"positions": [0] * 6, "sampled_at_ms": 1, "monotonic_ns": base + 5_000_000})
    result = manager.stop()
    manager.close()

    path = Path(result["path"])
    session = json.loads((path / "session.json").read_text(encoding="utf-8"))
    alignment = json.loads((path / "alignment.jsonl").read_text(encoding="utf-8").splitlines()[0])
    depth_bytes = np.fromfile(path / "oak" / "depth" / "00000000.png", dtype=np.uint8)
    depth = cv2.imdecode(depth_bytes, cv2.IMREAD_UNCHANGED)
    assert current["active"] is True
    assert session["status"] == "complete"
    assert session["counts"]["wrist"] == session["counts"]["oak"] == 1
    assert alignment["wrist_sequence"] == 0 and alignment["wrist_delta_ms"] == -8.0
    assert alignment["robot_sequence"] == 0
    assert depth.dtype == np.uint16 and depth[1, 1] == 2000
