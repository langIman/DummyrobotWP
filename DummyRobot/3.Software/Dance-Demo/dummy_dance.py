#!/usr/bin/env python3
"""A guarded, low-speed dance demo for the Dummy Robot arm."""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import serial
from serial.tools import list_ports


USB_VID = 0x1209
USB_PID = 0x0D32
BAUD_RATE = 115200

# Limits from Core-STM32F4-fw/Robot/instances/dummy_robot.cpp.  The wider
# observation limits allow the small boot-position differences seen on this arm.
COMMAND_LIMITS = (
    (-170.0, 170.0),
    (-73.0, 90.0),
    (35.0, 180.0),
    (-180.0, 180.0),
    (-120.0, 120.0),
    (-720.0, 720.0),
)
OBSERVATION_LIMITS = (
    (-175.0, 175.0),
    (-82.0, 95.0),
    (30.0, 190.0),
    (-185.0, 185.0),
    (-125.0, 125.0),
    (-730.0, 730.0),
)

REST = (0.0, -73.0, 180.0, 0.0, 0.0, 0.0)
TEST_READY = (0.0, -62.0, 168.0, 0.0, 0.0, 0.0)
DANCE_READY = (0.0, -42.0, 140.0, 0.0, 0.0, 0.0)

JOINT_PATTERN = re.compile(
    r"^ok\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s*$"
)


class RobotError(RuntimeError):
    pass


@dataclass(frozen=True)
class DanceStep:
    name: str
    joints: tuple[float, float, float, float, float, float]


SELF_TEST = (
    DanceStep("J1 +5 deg", (5.0, -73.0, 180.0, 0.0, 0.0, 0.0)),
    DanceStep("J1 return", REST),
    DanceStep("J2 +5 deg", (0.0, -68.0, 180.0, 0.0, 0.0, 0.0)),
    DanceStep("J2 return", REST),
    DanceStep("J3 -5 deg", (0.0, -73.0, 175.0, 0.0, 0.0, 0.0)),
    DanceStep("J3 return", REST),
    DanceStep("open for wrist test", TEST_READY),
    DanceStep("J4 +8 deg", (0.0, -62.0, 168.0, 8.0, 0.0, 0.0)),
    DanceStep("J4 return", TEST_READY),
    DanceStep("J5 +8 deg", (0.0, -62.0, 168.0, 0.0, 8.0, 0.0)),
    DanceStep("J5 return", TEST_READY),
    DanceStep("J6 +12 deg", (0.0, -62.0, 168.0, 0.0, 0.0, 12.0)),
    DanceStep("J6 return", TEST_READY),
)

DANCE = (
    DanceStep("stage opening", DANCE_READY),
    DanceStep("wide reach left", (-40.0, -38.0, 132.0, -45.0, 22.0, -80.0)),
    DanceStep("rise on left", (-28.0, -22.0, 112.0, -20.0, -28.0, -100.0)),
    DanceStep("high center", (0.0, -18.0, 105.0, 0.0, 30.0, 0.0)),
    DanceStep("rise on right", (28.0, -22.0, 112.0, 20.0, 28.0, 100.0)),
    DanceStep("wide reach right", (40.0, -38.0, 132.0, 45.0, -22.0, 80.0)),
    DanceStep("stage reset", DANCE_READY),
    DanceStep("drop left", (-32.0, -58.0, 165.0, 45.0, 28.0, 100.0)),
    DanceStep("cross up right", (20.0, -28.0, 118.0, -55.0, -30.0, -100.0)),
    DanceStep("drop right", (32.0, -58.0, 165.0, -45.0, -28.0, -100.0)),
    DanceStep("cross up left", (-20.0, -28.0, 118.0, 55.0, 30.0, 100.0)),
    DanceStep("stage reset", DANCE_READY),
    DanceStep("spiral left", (-18.0, -35.0, 130.0, -60.0, 30.0, -100.0)),
    DanceStep("spiral right", (18.0, -30.0, 120.0, 60.0, -30.0, 100.0)),
    DanceStep("shoulder pop left", (-35.0, -25.0, 115.0, 50.0, 20.0, -90.0)),
    DanceStep("shoulder pop right", (35.0, -25.0, 115.0, -50.0, -20.0, 90.0)),
    DanceStep("low center", (0.0, -55.0, 160.0, 0.0, -25.0, -80.0)),
    DanceStep("high center finale", (0.0, -20.0, 110.0, 0.0, 30.0, 80.0)),
    DanceStep("finale left", (-40.0, -35.0, 128.0, -50.0, 25.0, -100.0)),
    DanceStep("finale right", (40.0, -35.0, 128.0, 50.0, -25.0, 100.0)),
    DanceStep("finish center", DANCE_READY),
)


def format_joints(joints: Sequence[float]) -> str:
    return "[" + ", ".join(f"{value:7.2f}" for value in joints) + "]"


def validate_target(joints: Sequence[float]) -> None:
    if len(joints) != 6:
        raise RobotError("A target must contain exactly six joint angles.")
    for index, (value, limits) in enumerate(zip(joints, COMMAND_LIMITS), start=1):
        low, high = limits
        if not low <= value <= high:
            raise RobotError(
                f"J{index} target {value:.2f} is outside [{low:.2f}, {high:.2f}]."
            )


def validate_observation(joints: Sequence[float]) -> None:
    for index, (value, limits) in enumerate(zip(joints, OBSERVATION_LIMITS), start=1):
        low, high = limits
        if not low <= value <= high:
            raise RobotError(
                f"J{index} feedback {value:.2f} is outside the safety range "
                f"[{low:.2f}, {high:.2f}]."
            )


def looks_like_boot_rest(joints: Sequence[float]) -> bool:
    expected = (0.0, -75.0, 182.0, 0.0, 0.0, 0.0)
    tolerances = (8.0, 10.0, 12.0, 12.0, 12.0, 15.0)
    return all(
        abs(value - reference) <= tolerance
        for value, reference, tolerance in zip(joints, expected, tolerances)
    )


def find_robot_port(requested: str | None) -> str:
    if requested:
        return requested

    matches = [
        port.device
        for port in list_ports.comports()
        if port.vid == USB_VID and port.pid == USB_PID
    ]
    if not matches:
        available = ", ".join(port.device for port in list_ports.comports()) or "none"
        raise RobotError(
            "Dummy Robot serial port was not found "
            f"(VID={USB_VID:04X}, PID={USB_PID:04X}). Available ports: {available}"
        )
    if len(matches) > 1:
        raise RobotError(
            "More than one Dummy Robot was found. Use --port to choose one: "
            + ", ".join(matches)
        )
    return matches[0]


class DummyRobotSerial:
    def __init__(self, port: str, verbose: bool = False):
        self.port = port
        self.verbose = verbose
        self.serial: serial.Serial | None = None
        self.enabled = False

    def __enter__(self) -> "DummyRobotSerial":
        try:
            self.serial = serial.Serial(
                self.port,
                BAUD_RATE,
                timeout=0.15,
                write_timeout=1.0,
            )
        except serial.SerialException as exc:
            raise RobotError(
                f"Cannot open {self.port}. Close DummyStudio and other serial tools first: {exc}"
            ) from exc

        time.sleep(1.2)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        try:
            if self.enabled or exc_type is not None:
                self.emergency_disable()
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()

    def _write(self, command: str) -> None:
        if not self.serial or not self.serial.is_open:
            raise RobotError("Serial port is not open.")
        if self.verbose:
            print(f"  TX {command}")
        try:
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()
        except serial.SerialException as exc:
            raise RobotError(f"Serial write failed: {exc}") from exc

    def _readline(self) -> str:
        if not self.serial:
            return ""
        try:
            line = self.serial.readline().decode("ascii", errors="replace").strip()
        except serial.SerialException as exc:
            raise RobotError(f"Serial read failed: {exc}") from exc
        if line and self.verbose:
            print(f"  RX {line}")
        return line

    def _wait_for_text(self, expected: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        seen: list[str] = []
        while time.monotonic() < deadline:
            line = self._readline()
            if not line:
                continue
            seen.append(line)
            if expected.lower() in line.lower():
                return
        suffix = f" Last replies: {seen[-4:]}" if seen else " No reply was received."
        raise RobotError(f"Timed out waiting for '{expected}'.{suffix}")

    def enable(self) -> None:
        self._write("!START")
        self._wait_for_text("Started ok", 3.0)
        self.enabled = True

    def disable(self) -> None:
        self._write("!DISABLE")
        self._wait_for_text("Disabled ok", 2.0)
        self.enabled = False

    def emergency_disable(self) -> None:
        if not self.serial or not self.serial.is_open:
            return
        try:
            # STOP clears queued movement; DISABLE then removes holding current.
            self._write("!STOP")
            time.sleep(0.05)
            self._write("!DISABLE")
            time.sleep(0.1)
            self.enabled = False
        except Exception:
            pass

    def get_joints(self, timeout: float = 2.0) -> tuple[float, ...]:
        if not self.enabled:
            raise RobotError("Firmware only returns joint positions while enabled.")
        self._write("#GETJPOS")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._readline()
            if not line:
                continue
            match = JOINT_PATTERN.match(line)
            if match:
                joints = tuple(float(value) for value in match.groups())
                validate_observation(joints)
                return joints
        raise RobotError("No valid six-joint reply was received from #GETJPOS.")

    def move_to(self, target: Sequence[float], speed: float) -> tuple[float, ...]:
        validate_target(target)
        current = self.get_joints()
        largest_move = max(abs(goal - actual) for goal, actual in zip(target, current))
        timeout = max(8.0, largest_move / max(speed * 0.45, 0.5) + 8.0)
        command = ">" + ",".join(f"{value:.2f}" for value in target) + f",{speed:.2f}"
        self._write(command)

        deadline = time.monotonic() + timeout
        best_error = largest_move
        last_progress = time.monotonic()
        settled_samples = 0
        last_feedback = current

        while time.monotonic() < deadline:
            time.sleep(0.12)
            feedback = self.get_joints()
            error = max(abs(goal - actual) for goal, actual in zip(target, feedback))
            last_feedback = feedback

            if error < best_error - 0.35:
                best_error = error
                last_progress = time.monotonic()

            if error <= 1.5:
                settled_samples += 1
                if settled_samples >= 3:
                    return feedback
            else:
                settled_samples = 0

            if error > 3.0 and time.monotonic() - last_progress > 3.0:
                raise RobotError(
                    "Joint feedback stopped progressing. Possible obstruction or stall. "
                    f"Target={format_joints(target)}, feedback={format_joints(feedback)}"
                )

        raise RobotError(
            "Movement timed out. "
            f"Target={format_joints(target)}, feedback={format_joints(last_feedback)}"
        )


def print_sequence(title: str, steps: Iterable[DanceStep]) -> None:
    print(f"\n{title}")
    for index, step in enumerate(steps, start=1):
        print(f"  {index:02d}. {step.name:20s} {format_joints(step.joints)}")


def run_steps(
    robot: DummyRobotSerial,
    steps: Iterable[DanceStep],
    speed: float,
) -> None:
    for step in steps:
        print(f"  -> {step.name:20s} {format_joints(step.joints)}")
        feedback = robot.move_to(step.joints, speed)
        print(f"     feedback             {format_joints(feedback)}")


def confirm_start() -> None:
    print(
        "\nBefore continuing:\n"
        "  1. Put the arm in its physical folded storage pose, then power-cycle it.\n"
        "  2. Bolt or firmly hold the base; clear people and objects from the arm.\n"
        "  3. Keep one hand ready to unplug 12 V power.\n"
        "  4. Close DummyStudio so it releases the COM port.\n"
        "\nDo not continue if any joint is hot or showing a repeating fault blink."
    )
    answer = input("\nType FOLD to enable the motors: ").strip().upper()
    if answer != "FOLD":
        raise KeyboardInterrupt


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Guarded low-speed dance demo for Dummy Robot."
    )
    parser.add_argument("--port", help="serial port, for example COM7 (auto-detected by default)")
    parser.add_argument(
        "--speed",
        type=float,
        default=30.0,
        help="dance speed in firmware units, 1 to 30 (default: 30)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        choices=range(1, 4),
        default=1,
        help="number of dance cycles, 1 to 3 (default: 1)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print all targets without opening the serial port",
    )
    parser.add_argument("--verbose", action="store_true", help="print raw serial traffic")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 1.0 <= args.speed <= 30.0:
        raise RobotError("--speed must be between 1 and 30.")

    for step in (*SELF_TEST, *DANCE, DanceStep("rest", REST)):
        validate_target(step.joints)

    if args.dry_run:
        print_sequence("Self-test", SELF_TEST)
        print_sequence("Dance", DANCE)
        print_sequence("Finish", (DanceStep("rest and disable", REST),))
        return 0

    confirm_start()
    port = find_robot_port(args.port)
    print(f"\nOpening {port} at {BAUD_RATE} baud...")

    with DummyRobotSerial(port, verbose=args.verbose) as robot:
        print("Enabling motors. Press Ctrl+C or unplug 12 V immediately if motion is abnormal.")
        robot.enable()
        initial = robot.get_joints()
        print(f"Boot joint feedback: {format_joints(initial)}")
        if not looks_like_boot_rest(initial):
            raise RobotError(
                "The reported boot pose is not close to the expected folded pose "
                "[0, -75, 182, 0, 0, 0]. Motors have been disabled."
            )

        # A short enable-only probe catches the previously observed fault blink
        # before any requested movement is sent.  The user inspects it while the
        # arm is disabled, so the prompt cannot leave the motors holding forever.
        time.sleep(1.2)
        holding_feedback = robot.get_joints()
        robot.disable()
        holding_drift = max(
            abs(after - before) for after, before in zip(holding_feedback, initial)
        )
        if holding_drift > 3.0:
            raise RobotError(
                "A joint moved during the enable-only check. "
                f"Before={format_joints(initial)}, after={format_joints(holding_feedback)}"
            )

        answer = input(
            "\nEnable-only check finished and motors are now disabled. "
            "If no motor showed a repeating two/three-blink fault, type TEST: "
        ).strip().upper()
        if answer != "TEST":
            print("Self-test and dance cancelled. Motors remain disabled.")
            return 0

        robot.enable()
        reenabled = robot.get_joints()
        if not looks_like_boot_rest(reenabled):
            raise RobotError("Joint feedback changed unexpectedly before the self-test.")

        print("\nRunning one-joint-at-a-time self-test...")
        run_steps(robot, SELF_TEST, min(args.speed, 4.0))

        answer = input(
            "\nCheck all six motors: no collision, no heating, and no repeating "
            f"two/three-blink fault. Dance speed is {args.speed:g}; "
            "the next sequence uses large six-axis movements. Type RUN to dance: "
        ).strip().upper()
        if answer != "RUN":
            print("Dance cancelled; returning to the folded reference pose.")
            robot.move_to(REST, min(args.speed, 4.0))
            robot.disable()
            return 0

        for cycle in range(1, args.cycles + 1):
            print(f"\nDance cycle {cycle}/{args.cycles}")
            run_steps(robot, DANCE, args.speed)

        print("\nReturning to the folded reference pose...")
        robot.move_to(REST, min(args.speed, 4.0))
        robot.disable()
        print("Dance complete. Motors are disabled.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nCancelled. Motors were disabled if the serial port had been opened.")
        raise SystemExit(130)
    except RobotError as exc:
        print(f"\nSAFETY STOP: {exc}", file=sys.stderr)
        raise SystemExit(1)
    except serial.SerialException as exc:
        print(f"\nSERIAL ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
