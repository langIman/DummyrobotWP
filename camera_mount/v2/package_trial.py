"""Freeze the user-reviewed V2 geometry into an unambiguous trial-print ZIP."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "review_cad"
PACKAGE = ROOT / "V2_trial_print_2026-09-12.zip"


def main():
    report = json.loads((SOURCE / "verification.json").read_text(encoding="utf-8"))
    params = json.loads((SOURCE / "design_parameters.json").read_text(encoding="utf-8"))
    if report.get("cad_checks_passed") is not True:
        raise RuntimeError("Cannot package geometry with failed CAD checks")
    if len(report["stl_meshes"]) != 3 or any(
        m["boundary_or_nonmanifold_edges"] != 0 for m in report["stl_meshes"]
    ):
        raise RuntimeError("Expected three closed printable meshes")

    files = {
        "STL/01_lower_clamp_V2.stl": SOURCE / "01_lower_clamp_DRAFT.stl",
        "STL/02_upper_clamp_V2.stl": SOURCE / "02_upper_clamp_DRAFT.stl",
        "STL/03_camera_carrier_V2.stl": SOURCE / "03_front_camera_carrier_DRAFT.stl",
        "PRINT_AND_ASSEMBLY_打印与安装.md": ROOT / "PRINT_AND_ASSEMBLY.md",
        "reference/V2_assembly.step": SOURCE / "V2_REVIEW_assembly.step",
        "reference/01_lower_clamp_V2.step": SOURCE / "01_lower_clamp_DRAFT.step",
        "reference/02_upper_clamp_V2.step": SOURCE / "02_upper_clamp_DRAFT.step",
        "reference/03_camera_carrier_V2.step": SOURCE / "03_front_camera_carrier_DRAFT.step",
    }
    for drawing in sorted((ROOT / "drawings").glob("*.png")):
        files[f"reference/drawings/{drawing.name}"] = drawing
    content = {name: path.read_bytes() for name, path in files.items()}
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in content.items()}
    manifest = {
        "release_status": "USER_REVIEWED_TRIAL_PRINT",
        "physical_fit_status": "NOT_YET_TRIAL_FITTED",
        "review_date": "2026-09-12",
        "review_evidence": "用户：嗯好，我看完了觉得没什么问题",
        "geometry_change_since_review": False,
        "units": "mm",
        "scale_percent": 100,
        "print_quantity_per_STL_for_one_set": 1,
        "parameters": params["parameters"],
        "sha256": hashes,
        "source_verification_sha256": hashlib.sha256(
            (SOURCE / "verification.json").read_bytes()).hexdigest(),
        "limitations": report["limits"],
    }
    content["manifest.json"] = json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8")
    # The original report remains unchanged and correctly records unverified physical fit.
    content["reference/cad_verification.json"] = (SOURCE / "verification.json").read_bytes()
    with zipfile.ZipFile(PACKAGE, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in content.items():
            archive.writestr(name, data)
    with zipfile.ZipFile(PACKAGE) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("ZIP integrity failure")
        stls = [n for n in archive.namelist() if n.endswith(".stl")]
        if len(stls) != 3 or not all(n.startswith("STL/") for n in stls):
            raise RuntimeError("Print package must contain exactly three STL files")
        for name, digest in hashes.items():
            if hashlib.sha256(archive.read(name)).hexdigest() != digest:
                raise RuntimeError(f"Packaged file differs from reviewed source: {name}")
    (ROOT / "trial_print_manifest.json").write_bytes(content["manifest.json"])
    print(json.dumps({"package": str(PACKAGE), "stl_count": len(stls),
                      "source_bytes_preserved": True, "zip_integrity": "passed",
                      "size_bytes": PACKAGE.stat().st_size}, indent=2))


if __name__ == "__main__":
    main()
