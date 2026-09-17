import pytest

from perception_service.common import ServiceError
from perception_service.configuration import DEFAULT_CONFIG, validate_oak, validate_wrist


def test_wrist_allows_verified_modes():
    result = validate_wrist(DEFAULT_CONFIG["wrist"], {"width": 1920, "height": 1080, "fps": 60})
    assert (result["width"], result["height"], result["fps"]) == (1920, 1080, 60)


def test_wrist_rejects_driver_fallback_mode():
    with pytest.raises(ServiceError) as error:
        validate_wrist(DEFAULT_CONFIG["wrist"], {"width": 640, "height": 480})
    assert error.value.code == "unsupported_camera_resolution"


def test_oak_depth_shape_and_range_are_fixed_and_validated():
    result = validate_oak(DEFAULT_CONFIG["oak"], {"depth_min_mm": 300, "depth_max_mm": 2500})
    assert (result["depth_width"], result["depth_height"]) == (640, 360)
    with pytest.raises(ServiceError) as error:
        validate_oak(DEFAULT_CONFIG["oak"], {"depth_min_mm": 3000, "depth_max_mm": 200})
    assert error.value.code == "invalid_depth_range"

