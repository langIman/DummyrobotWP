"""Geometric checks for V2. A passing CAD check is not physical fit approval."""
import json
import os
from pathlib import Path
import sys

from build_v2 import (P, OUT, ROOT, box, cylinder, compound, clamp_bottom,
                      clamp_top, camera_frame, camera_references, hardware_references)
sys.path.insert(0,str(ROOT.parent/"tools"))
from verify_mount import mesh_report
import cadquery as cq


def volume(shape):
    return sum(s.Volume() for s in shape.Solids())


def overlap(a,b):
    a=a.val() if isinstance(a,cq.Workplane) else a
    b=b.val() if isinstance(b,cq.Workplane) else b
    return volume(a.intersect(b))


def main():
    bottom,top,frame=clamp_bottom(),clamp_top(),camera_frame()
    refs=camera_references()
    motor=box(-17.5,17.5,-17.5,17.5,-17.5,17.5)
    fixed_hardware,moving_hardware=hardware_references()
    fixed=compound([bottom,top,motor]+fixed_hardware)
    checks=[]
    def check(name,value,limit=0.001):
        checks.append({"check":name,"overlap_mm3":round(value,6),"passed":value<=limit})
    check("clamp halves separated",overlap(bottom,top))
    check("motor versus upper clamp",overlap(motor,top))
    check("motor versus lower clamp",overlap(motor,bottom))
    check("camera PCB versus carrier",overlap(refs["pcb"],frame))
    check("lens envelope versus carrier",overlap(refs["lens_envelope"],frame))
    for name,keepout in refs.items():
        if "keepout" in name:
            check(name+" versus carrier",overlap(keepout,frame))
            check(name+" versus camera hardware",overlap(keepout,compound(moving_hardware)))
    for side in (-1,1):
        ya,yb=sorted((side*17.2,side*22))
        check(f"pivot nut wrench envelope {side} versus carrier",
              overlap(frame,cylinder(4.5,yb-ya,(P.pivot_x,ya,P.pivot_z),(0,1,0))))
        ya,yb=sorted((side*22,side*28))
        check(f"frame pivot hole {side} fully open",
              overlap(frame,cylinder(1.6,yb-ya,(P.pivot_x,ya,P.pivot_z),(0,1,0))))
        ya,yb=sorted((side*28.4,side*32.4))
        check(f"support pivot hole {side} fully open",
              overlap(top,cylinder(1.6,yb-ya,(P.pivot_x,ya,P.pivot_z),(0,1,0))))
        for name,part in [("upper",top),("lower",bottom)]:
            check(f"{name} clamp bolt hole {side} fully open",
                  overlap(part,cylinder(1.6,32,(0,side*P.bolt_y,-9),(0,0,1))))
        # Hex key corridor before the camera carrier is installed.
        check(f"clamp screw tool access {side}",overlap(top,
              cylinder(2,70,(0,side*P.bolt_y,P.clamp_head_seat_z),(0,0,1))))
    for y in (-17,17):
        for z in (P.pivot_z-17,P.pivot_z+17):
            check(f"M2 carrier hole {y},{z} fully open",
                  overlap(frame,cylinder(1.1,8,(-1,y,z),(1,0,0))))
    check("fixed hardware versus plastic",overlap(compound(fixed_hardware),compound([top,bottom])))
    check("camera hardware versus plastic and PCB",overlap(compound(moving_hardware),compound([frame,refs["pcb"],top,bottom])))
    # Endpoint samples + intermediate samples; not a mathematical continuous-motion proof.
    axis0=(P.pivot_x,0,P.pivot_z)
    axis1=(P.pivot_x,1,P.pivot_z)
    moving=compound([frame,*refs.values(),*moving_hardware])
    samples=[]
    for angle in range(-30,31,5):
        v=overlap(moving.rotate(axis0,axis1,angle),fixed)
        samples.append({"angle_deg":angle,"overlap_mm3":round(v,6),"passed":v<=0.001})
    meshes=[mesh_report(p) for p in sorted(OUT.glob("0*.stl"))]
    steps=[]
    for p in sorted(OUT.glob("0*.step")):
        shape=cq.importers.importStep(str(p))
        steps.append({"file":p.name,"valid":shape.val().isValid(),"solids":len(shape.solids().vals())})
    # Verify the installed halves do not reach each other before contacting the motor.
    gap=P.split_gap
    engagement={"clamp_M3x25_thread_beyond_nut_mm":25-(P.clamp_head_seat_z-(P.flange_bottom+P.nut_pocket_depth-2.4)),
                "pivot_M3x16_thread_beyond_two_nuts_mm":16-(4+0.4+6+2*2.4),
                "camera_M2x10_thread_beyond_nut_mm":10-(P.pcb_thickness+P.frame_front_x+1.6)}
    passed=(all(c["passed"] for c in checks) and all(s["passed"] for s in samples)
            and all(m["boundary_or_nonmanifold_edges"]==0 for m in meshes)
            and all(s["valid"] and s["solids"]==1 for s in steps)
            and gap>0 and all(v>=0.5 for v in engagement.values()))
    report={"cad_checks_passed":passed,"physical_fit_status":"UNCONFIRMED_REVIEW_DRAFT",
            "checks":checks,"split_gap_at_nominal_motor_mm":gap,"nominal_thread_margins":engagement,
            "pitch_samples":{"range_deg":[-30,30],"step_deg":5,"samples":samples},
            "stl_meshes":meshes,"step_roundtrip":steps,
            "limits":["No real gripper, wiring or adjacent-joint CAD has been supplied.",
                      "Connector volumes are proposed clearance envelopes, not measured geometry.",
                      "The rear lower M2 head is only 0.1 mm outside the proposed 3Pin corridor; measure actual plug clearance.",
                      "No strength, thermal-creep, cable-bend or printing-tolerance qualification.",
                      "Wrench envelope uses radius 4.5 mm; actual tool dimensions may differ.",
                      "Pitch scan samples every 5 degrees and does not prove continuous-motion clearance."]}
    (OUT/"verification.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps({"passed":passed,"failed_checks":[c for c in checks if not c["passed"]],
                      "failed_pitch":[s for s in samples if not s["passed"]],
                      "mesh_checks":meshes,"threads":engagement},indent=2),flush=True)
    return 0 if passed else 1


if __name__=="__main__":
    result=main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(result) if sys.platform=="win32" else sys.exit(result)
