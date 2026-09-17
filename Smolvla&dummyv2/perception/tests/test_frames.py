from perception_service.frames import FrameStore


def test_frame_store_reports_latest_frame_and_metadata():
    store = FrameStore()
    store.put(7, b"jpeg", 1280, 720)
    frame = store.get(2_000)
    assert frame is not None and frame.frame_id == 7
    assert store.metadata()["dimensions"] == [1280, 720]
    store.clear()
    assert store.get() is None

