#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main() -> int:
    service = LaunchService()
    source = PythonLaunchDescriptionSource(str(Path(__file__).with_name("safe_servo.launch.py")))
    service.include_launch_description(IncludeLaunchDescription(source))
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
