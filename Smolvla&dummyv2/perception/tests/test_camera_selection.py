import pytest

from perception_service import cameras
from perception_service.cameras import WristCamera


def test_auto_selection_does_not_bind_virtual_camera(monkeypatch):
    monkeypatch.setattr(
        cameras,
        "discover_wrist_cameras",
        lambda: [
            {
                "device_id": "dshow:0:ToDesk Camera",
                "name": "ToDesk Camera",
                "is_virtual": True,
                "auto_candidate": False,
            }
        ],
    )
    with pytest.raises(OSError, match="wrist_camera_not_found"):
        WristCamera._resolve({"device_id": None})


def test_explicit_selection_can_still_choose_any_enumerated_source(monkeypatch):
    selected = {
        "device_id": "dshow:2:Test Source",
        "name": "Test Source",
        "is_virtual": True,
        "auto_candidate": False,
    }
    monkeypatch.setattr(cameras, "discover_wrist_cameras", lambda: [selected])
    assert WristCamera._resolve({"device_id": selected["device_id"]}) == selected
