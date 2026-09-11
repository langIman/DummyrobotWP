"""Verify printable meshes and the camera mount's pitch clearance."""

import json
import os
import struct
import sys
from collections import Counter
from pathlib import Path

import cadquery as cq


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output"
sys.path.insert(0, str(ROOT))

from build_camera_mount import (  # noqa: E402
    P,
    camera_frame,
    camera_reference,
    clamp_bottom,
    clamp_top,
    motor_reference,
)


PRINT_FILES = (
    "dummy_j6_motor_clamp_bottom.stl",
    "dummy_j6_motor_clamp_top.stl",
    "decxin_camera_frame.stl",
)


def overlap_volume(first: cq.Workplane, second: cq.Workplane) -> float:
    overlap = first.intersect(second)
    return sum(solid.Volume() for solid in overlap.solids().vals())


def mesh_report(path: Path) -> dict:
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError(f"Invalid binary STL: {path.name}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise RuntimeError(f"Unexpected binary STL size: {path.name}")

    edges = Counter()
    coordinates = [[], [], []]
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", data, 84 + index * 50)
        vertices = []
        for vertex_index in range(3):
            vertex = tuple(round(values[3 + vertex_index * 3 + axis], 5) for axis in range(3))
            vertices.append(vertex)
            for axis, coordinate in enumerate(vertex):
                coordinates[axis].append(coordinate)
        for start, end in ((0, 1), (1, 2), (2, 0)):
            edges[tuple(sorted((vertices[start], vertices[end])))] += 1

    bad_edges = sum(1 for count in edges.values() if count != 2)
    return {
        "file": path.name,
        "triangles": triangle_count,
        "boundary_or_nonmanifold_edges": bad_edges,
        "size_mm": [round(max(axis) - min(axis), 2) for axis in coordinates],
    }


def pitch_report() -> dict:
    top = clamp_top()
    bottom = clamp_bottom()
    motor = motor_reference()
    frame = camera_frame()
    camera = camera_reference()
    fixed = top.union(bottom).union(motor)
    axis_start = (P.pivot_x, 0, P.pivot_z)
    axis_end = (P.pivot_x, 1, P.pivot_z)

    samples = []
    for angle in range(-45, 46, 5):
        moved_frame = frame.rotate(axis_start, axis_end, angle)
        moved_camera = camera.rotate(axis_start, axis_end, angle)
        volume = overlap_volume(moved_frame, fixed) + overlap_volume(moved_camera, fixed)
        samples.append({"angle_deg": angle, "overlap_mm3": round(volume, 4)})

    clear = [sample["angle_deg"] for sample in samples if sample["overlap_mm3"] <= 0.01]
    return {
        "tested_range_deg": [-45, 45],
        "sample_step_deg": 5,
        "clear_sample_range_deg": [min(clear), max(clear)] if clear else None,
        "samples": samples,
    }


def main() -> None:
    meshes = [mesh_report(OUTPUT / filename) for filename in PRINT_FILES]
    if any(item["boundary_or_nonmanifold_edges"] for item in meshes):
        raise RuntimeError("At least one printable STL is open or non-manifold")

    for step_file in OUTPUT.glob("*.step"):
        imported = cq.importers.importStep(str(step_file))
        if not imported.solids().vals():
            raise RuntimeError(f"No solids found after reopening {step_file.name}")

    result = {
        "printable_meshes": meshes,
        "pitch_clearance": pitch_report(),
        "status": "passed",
    }
    report_file = OUTPUT / "verification.json"
    report_file.write_text(json.dumps(result, indent=2), encoding="ascii")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
    # The pip OCP wheel can fail during interpreter finalization on Windows.
    if sys.platform == "win32":
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
