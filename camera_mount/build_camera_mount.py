"""Parametric printable mount for a DECXIN 38 mm camera on Dummy Joint 6.

Coordinate system used by the assembly preview:
  X: motor shaft and camera optical axis, positive toward the gripper
  Y: camera width
  Z: up
"""

from dataclasses import dataclass
import os
from pathlib import Path
import sys

import cadquery as cq


@dataclass(frozen=True)
class Parameters:
    # Dummy Joint 6 motor, taken from Dummy v164.step (PK513PA-H50S v1).
    motor_size: float = 20.0
    motor_fit: float = 0.4
    clamp_length: float = 16.0
    clamp_wall: float = 4.0
    clamp_screw_diameter: float = 3.4
    clamp_screw_y: float = 18.5
    m3_nut_af: float = 5.8
    m3_nut_depth: float = 2.6

    # Hinge and camera position.
    pivot_diameter: float = 3.4
    pivot_x: float = 2.0
    pivot_z: float = 20.0
    camera_center_z: float = 41.0
    camera_frame_size: float = 42.0
    camera_frame_depth: float = 3.2
    camera_opening: float = 26.0
    hinge_ear_depth: float = 6.6
    hinge_ear_width: float = 4.0
    hinge_ear_height: float = 8.0
    base_tab_height: float = 12.4
    hinge_side_clearance: float = 0.3

    # DECXIN-3081V1 camera drawing.
    pcb_size: float = 38.0
    pcb_thickness: float = 1.6
    camera_hole_pitch: float = 34.0
    camera_screw_diameter: float = 2.4
    standoff_diameter: float = 6.0
    standoff_height: float = 3.0


P = Parameters()
ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "output"


def motor_void() -> cq.Workplane:
    fit = P.motor_size + P.motor_fit
    return cq.Workplane("XY").box(P.clamp_length + 2, fit, fit)


def clamp_bottom() -> cq.Workplane:
    fit = P.motor_size + P.motor_fit
    outside = fit + 2 * P.clamp_wall
    half_height = fit / 2 + P.clamp_wall
    flange_thickness = 4.0

    shell = (
        cq.Workplane("XY")
        .box(P.clamp_length, outside, half_height)
        .translate((0, 0, -half_height / 2))
        .cut(motor_void())
    )
    flange_width = P.clamp_screw_y - outside / 2 + 4.0
    for side in (-1, 1):
        y_center = side * (outside / 2 + flange_width / 2)
        flange = (
            cq.Workplane("XY")
            .box(P.clamp_length, flange_width, flange_thickness)
            .translate((0, y_center, -flange_thickness / 2))
        )
        shell = shell.union(flange)

    for side in (-1, 1):
        hole = (
            cq.Workplane("XY", origin=(0, side * P.clamp_screw_y, -flange_thickness - 1))
            .circle(P.clamp_screw_diameter / 2)
            .extrude(flange_thickness + 2)
        )
        nut = (
            cq.Workplane("XY", origin=(0, side * P.clamp_screw_y, -flange_thickness - 0.01))
            .polygon(6, P.m3_nut_af / 0.8660254)
            .extrude(P.m3_nut_depth)
        )
        shell = shell.cut(hole).cut(nut)
    return shell.clean()


def clamp_top() -> cq.Workplane:
    fit = P.motor_size + P.motor_fit
    outside = fit + 2 * P.clamp_wall
    half_height = fit / 2 + P.clamp_wall
    tab_y = (
        P.camera_frame_size / 2
        + P.hinge_ear_width
        + P.hinge_side_clearance
        + P.hinge_ear_width / 2
    )
    bridge_width = 2 * (tab_y + P.hinge_ear_width / 2)

    shell = (
        cq.Workplane("XY")
        .box(P.clamp_length, outside, half_height)
        .translate((0, 0, half_height / 2))
        .cut(motor_void())
    )
    bridge = (
        cq.Workplane("XY")
        .box(P.clamp_length, bridge_width, P.clamp_wall)
        .translate((0, 0, fit / 2 + P.clamp_wall / 2))
    )
    shell = shell.union(bridge)

    for side in (-1, 1):
        boss = (
            cq.Workplane("XY")
            .circle(4.0)
            .extrude(half_height)
            .translate((0, side * P.clamp_screw_y, 0))
        )
        shell = shell.union(boss)
        hole = (
            cq.Workplane("XY", origin=(0, side * P.clamp_screw_y, -1))
            .circle(P.clamp_screw_diameter / 2)
            .extrude(half_height + 2)
        )
        shell = shell.cut(hole)

        tab = (
            cq.Workplane("XY")
            .box(P.clamp_length, P.hinge_ear_width, P.base_tab_height)
            .translate((0, side * tab_y, P.pivot_z))
        )
        pivot_hole = (
            cq.Workplane(
                "XZ",
                origin=(P.pivot_x, side * (tab_y - P.hinge_ear_width / 2 - 1.0), P.pivot_z),
            )
            .circle(P.pivot_diameter / 2)
            .extrude(side * (P.hinge_ear_width + 2.0))
        )
        shell = shell.union(tab).cut(pivot_hole)
    return shell.clean()


def camera_frame() -> cq.Workplane:
    half = P.camera_frame_size / 2
    frame = (
        cq.Workplane("YZ")
        .rect(P.camera_frame_size, P.camera_frame_size)
        .rect(P.camera_opening, P.camera_opening)
        .extrude(P.camera_frame_depth)
        .translate((-P.camera_frame_depth / 2, 0, P.camera_center_z))
    )

    # Open the lower center for the upright Type-C connector and cable.
    cable_slot = (
        cq.Workplane("XY")
        .box(P.camera_frame_depth + 2, 14.0, P.camera_frame_size / 2)
        .translate((0, 0, P.camera_center_z - P.camera_frame_size * 3 / 8))
    )
    frame = frame.cut(cable_slot)

    pitch = P.camera_hole_pitch / 2
    front_x = P.camera_frame_depth / 2
    for y in (-pitch, pitch):
        for z in (P.camera_center_z - pitch, P.camera_center_z + pitch):
            standoff = (
                cq.Workplane("YZ", origin=(front_x, y, z))
                .circle(P.standoff_diameter / 2)
                .extrude(P.standoff_height)
            )
            screw = (
                cq.Workplane("YZ", origin=(-P.camera_frame_depth, y, z))
                .circle(P.camera_screw_diameter / 2)
                .extrude(P.camera_frame_depth + P.standoff_height + 3)
            )
            frame = frame.union(standoff).cut(screw)

    ear_y = half + P.hinge_ear_width / 2
    for side in (-1, 1):
        ear = (
            cq.Workplane("XY")
            .box(P.hinge_ear_depth, P.hinge_ear_width, P.hinge_ear_height)
            .translate(((P.hinge_ear_depth - P.camera_frame_depth) / 2,
                        side * ear_y, P.pivot_z))
        )
        pivot_hole = (
            cq.Workplane(
                "XZ",
                origin=(P.pivot_x, side * (ear_y - P.hinge_ear_width / 2 - 1.0), P.pivot_z),
            )
            .circle(P.pivot_diameter / 2)
            .extrude(side * (P.hinge_ear_width + 2.0))
        )
        frame = frame.union(ear).cut(pivot_hole)
    return frame.clean()


def motor_reference() -> cq.Workplane:
    body = cq.Workplane("XY").box(50.0, P.motor_size, P.motor_size)
    boss = cq.Workplane("YZ", origin=(25.0, 0, 0)).circle(6.0).extrude(2.0)
    shaft = cq.Workplane("YZ", origin=(27.0, 0, 0)).circle(2.5).extrude(8.0)
    return body.union(boss).union(shaft)


def camera_reference() -> cq.Workplane:
    board_x = P.camera_frame_depth / 2 + P.standoff_height + P.pcb_thickness / 2
    board = (
        cq.Workplane("XY")
        .box(P.pcb_thickness, P.pcb_size, P.pcb_size)
        .translate((board_x, 0, P.camera_center_z))
    )
    holder_x = board_x + P.pcb_thickness / 2
    holder = (
        cq.Workplane("XY")
        .box(10.4, 17.0, 17.0)
        .translate((holder_x + 5.2, 0, P.camera_center_z))
    )
    lens = (
        cq.Workplane("YZ", origin=(holder_x + 10.4, 0, P.camera_center_z))
        .circle(8.0)
        .extrude(6.7)
    )
    connector = (
        cq.Workplane("XY")
        .box(8.2, 9.5, 4.0)
        .translate((board_x - P.pcb_thickness / 2 - 4.1, 0, P.camera_center_z - 13.0))
    )
    return board.union(holder).union(lens).union(connector)


def export_part(shape: cq.Workplane, stem: str) -> None:
    cq.exporters.export(shape, str(OUTPUT / f"{stem}.step"))
    cq.exporters.export(shape, str(OUTPUT / f"{stem}.stl"), tolerance=0.05, angularTolerance=0.1)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    bottom = clamp_bottom()
    top = clamp_top()
    frame = camera_frame()
    motor = motor_reference()
    camera = camera_reference()

    parts = {
        "dummy_j6_motor_clamp_bottom": bottom,
        "dummy_j6_motor_clamp_top": top,
        "decxin_camera_frame": frame,
        "decxin_camera_reference": camera,
    }
    for name, part in parts.items():
        solid_count = len(part.solids().vals())
        if solid_count != 1 or not part.val().isValid():
            raise RuntimeError(f"Expected one valid solid for {name}, got {solid_count}")
        export_part(part, name)

    checks = {
        "clamp halves": top.intersect(bottom),
        "camera frame and clamp": frame.intersect(top),
        "motor and clamp top": motor.intersect(top),
        "motor and clamp bottom": motor.intersect(bottom),
        "camera and frame": camera.intersect(frame),
    }
    for description, overlap in checks.items():
        volume = sum(s.Volume() for s in overlap.solids().vals())
        if volume > 0.01:
            raise RuntimeError(f"Unexpected overlap ({description}): {volume:.3f} mm^3")

    assembly = cq.Assembly(name="dummy_joint6_camera_mount")
    assembly.add(motor, name="motor_reference", color=cq.Color(0.25, 0.28, 0.32))
    assembly.add(bottom, name="clamp_bottom", color=cq.Color(0.92, 0.35, 0.12))
    assembly.add(top, name="clamp_top", color=cq.Color(0.92, 0.35, 0.12))
    assembly.add(frame, name="camera_frame", color=cq.Color(0.1, 0.55, 0.82))
    assembly.add(camera, name="camera_reference", color=cq.Color(0.15, 0.55, 0.32))
    assembly.save(str(OUTPUT / "dummy_joint6_camera_mount_assembly.step"))
    assembly.save(str(OUTPUT / "dummy_joint6_camera_mount_assembly.glb"))

    reference_dir = ROOT / "reference"
    reference_dir.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(motor, str(reference_dir / "generated_motor_reference.stl"))

    preview = cq.Compound.makeCompound(
        [motor.val(), bottom.val(), top.val(), frame.val(), camera.val()]
    )
    cq.exporters.export(
        preview,
        str(OUTPUT / "dummy_joint6_camera_mount_preview.stl"),
        tolerance=0.05,
        angularTolerance=0.1,
    )
    artifact_count = len(parts) * 2 + 3
    print(f"Generated {artifact_count} files in {OUTPUT}; interference checks passed")


if __name__ == "__main__":
    main()
    # The pip OCP wheel can fail during interpreter finalization on Windows.
    if sys.platform == "win32":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
