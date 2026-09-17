from types import SimpleNamespace

import pytest

from app import create_app
from perception_service.common import ServiceError
from perception_service.service import PerceptionService


class FakeRecorder:
    def __init__(self, active=True):
        self.active = active
        self.events = []

    def is_active(self):
        return self.active

    def record_event(self, value):
        self.events.append(value)

    def current(self):
        return {"active": self.active, "session_id": "test"}


def service_with_recorder(active=True):
    service = PerceptionService.__new__(PerceptionService)
    service.recorder = FakeRecorder(active)
    return service


def test_control_events_are_validated_and_recorded_without_robot_io():
    service = service_with_recorder()
    result = service.record_control_event({"kind": "move", "yaw": 0.2, "pitch": -0.5,
                                           "keys": ["w", "a"], "at_client_ms": 123})
    assert result["accepted"] is True
    assert service.recorder.events == [{"type": "data_collection_input", "kind": "move",
                                       "command_frame": "gripper_tool", "control_protocol_version": 7,
                                       "yaw": 0.2, "pitch": -0.5, "keys": ["a", "w"],
                                       "at_client_ms": 123}]
    with pytest.raises(ServiceError) as exc:
        service.record_control_event({"kind": "look", "yaw": 2})
    assert exc.value.code == "control_value_invalid"
    inactive = service_with_recorder(False)
    with pytest.raises(ServiceError) as exc:
        inactive.record_control_event({"kind": "look"})
    assert exc.value.code == "recording_not_active"


@pytest.mark.parametrize('key', ['arrowup', 'arrowdown'])
def test_vertical_input_is_recorded_without_robot_io(key):
    service = service_with_recorder()
    result = service.record_control_event({'kind': 'move', 'keys': ['w', key]})
    assert result['accepted']
    assert service.recorder.events[-1]['keys'] == [key, 'w']


def test_collection_page_and_event_endpoint_are_exposed():
    service = service_with_recorder()
    runtime = SimpleNamespace(robot=SimpleNamespace(), record_control_event=service.record_control_event)
    app = create_app(runtime)
    app.testing = True
    client = app.test_client()
    page = client.get("/collect")
    assert page.status_code == 200
    assert b"control-keyboard" in page.data
    assert "0 闭合".encode() in page.data
    assert b"collect-gripper-disable" not in page.data
    assert b"collect-arm-hold" in page.data
    response = client.post("/api/recordings/control-event", json={"kind": "look"})
    assert response.status_code == 200
    assert response.json["accepted"] is True
