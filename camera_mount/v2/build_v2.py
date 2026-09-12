"""V2 REVIEW DRAFT. Dimensions in mm; never imports the Dummy STEP motor.

Assembly axes: X toward gripper/lens, Y across PCB, Z above motor.
The camera front PCB surface is X=0. Its rear is X=-1.6.
Connector volumes are proposed keepouts, NOT measured connector models.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import math
import os
import sys

import cadquery as cq

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "review_cad"


@dataclass(frozen=True)
class Parameters:
    motor_size: float = 35.0           # User measurement 2026-09-12
    motor_length: float = 35.0         # User measurement; usable band unconfirmed
    clamp_band: float = 12.0           # PROPOSED, requires exposed-band measurement
    cavity_width: float = 35.6         # 0.3 per side, top/bottom contact at 35
    wall: float = 4.0
    split_gap: float = 2.0             # Closure reserve at nominal motor contact
    bolt_y: float = 25.5
    bolt_diameter: float = 3.4
    nut_af_clearance: float = 5.8
    nut_pocket_depth: float = 2.8
    flange_bottom: float = -7.0
    clamp_head_seat_z: float = 17.5   # Counterbore for user's M3x25 screws
    pivot_x: float = 11.0
    pivot_z: float = 60.0
    pivot_radius: float = 7.0
    frame_ear_inner: float = 22.0
    frame_ear_outer: float = 28.0
    hinge_gap: float = 0.4
    fixed_ear_thickness: float = 4.0
    frame_size: float = 48.0
    frame_opening: float = 30.0
    frame_rear_x: float = 3.0
    frame_front_x: float = 6.0
    pcb_size: float = 38.0             # Manufacturer drawing, user confirmed fit
    pcb_thickness: float = 1.6
    pcb_pitch: float = 34.0
    pcb_hole: float = 2.0              # Manufacturer drawing diameter callout
    printed_m2_hole: float = 2.4
    standoff_diameter: float = 5.5
    lens_front_extent: float = 25.3    # Drawing envelope, relative to PCB front
    usb_keepout_width: float = 16.0    # PROPOSED plug + handling envelope
    pin_keepout_width: float = 12.0    # PROPOSED, both PCB sides kept clear
    rear_keepout_depth: float = 40.0   # PROPOSED connector/plug corridor; not cable radius


P = Parameters()


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1-x0, y1-y0, z1-z0).translate(
        ((x0+x1)/2, (y0+y1)/2, (z0+z1)/2))


def cylinder(radius, length, start, direction):
    # Explicit world directions prevent the V1 XZ-plane sign bug.
    return cq.Workplane(obj=cq.Solid.makeCylinder(
        radius, length, cq.Vector(*start), cq.Vector(*direction)))


def xz_prism(points, y0, y1):
    return cq.Workplane("XZ").polyline(points).close().extrude(-(y1-y0)).translate((0,y0,0))


def hex_z(x, y, z, height, af):
    return cq.Workplane("XY", origin=(x,y,z)).polygon(6,af/math.cos(math.pi/6)).extrude(height)


def inner_void():
    h = P.motor_size/2
    return box(-P.clamp_band/2-1,P.clamp_band/2+1,
               -P.cavity_width/2,P.cavity_width/2,-h,h)


def clamp_bottom():
    h, w, b = P.motor_size/2+P.wall, P.cavity_width/2+P.wall, P.clamp_band/2
    shape = box(-b,b,-w,w,-h,-P.split_gap/2).cut(inner_void())
    for side in (-1,1):
        ya,yb = sorted((side*(w-1),side*(P.bolt_y+4)))
        shape = shape.union(box(-b,b,ya,yb,P.flange_bottom,-P.split_gap/2))
        shape = shape.cut(cylinder(P.bolt_diameter/2,12,(0,side*P.bolt_y,-9),(0,0,1)))
        shape = shape.cut(hex_z(0,side*P.bolt_y,P.flange_bottom-0.01,
                                P.nut_pocket_depth+0.01,P.nut_af_clearance))
    return shape.clean()


def clamp_top():
    h, w, b = P.motor_size/2+P.wall, P.cavity_width/2+P.wall, P.clamp_band/2
    support_inner=P.frame_ear_outer+P.hinge_gap
    support_outer=support_inner+P.fixed_ear_thickness
    shape=box(-b,b,-w,w,P.split_gap/2,h).cut(inner_void())
    shape=shape.union(box(-b,b,-support_outer,support_outer,h-P.wall,h))
    for side in (-1,1):
        ya,yb=sorted((side*(w-1),side*(P.bolt_y+4)))
        shape=shape.union(box(-b,b,ya,yb,P.split_gap/2,7))
        shape=shape.union(cylinder(4,h-P.split_gap/2,
                                  (0,side*P.bolt_y,P.split_gap/2),(0,0,1)))
        ya,yb=sorted((side*support_inner,side*support_outer))
        # Broad continuous upright with a round top, overlapping the full bridge thickness.
        upright=xz_prism([(-b,h-P.wall),(b,h-P.wall),
                          (P.pivot_x+P.pivot_radius,P.pivot_z),
                          (P.pivot_x-P.pivot_radius,P.pivot_z)],ya,yb)
        upright=upright.union(cylinder(P.pivot_radius,yb-ya,
                                     (P.pivot_x,ya,P.pivot_z),(0,1,0)))
        shape=shape.union(upright)
        shape=shape.cut(cylinder(P.bolt_diameter/2,yb-ya+2,
                                (P.pivot_x,ya-1,P.pivot_z),(0,1,0)))
        shape=shape.cut(cylinder(P.bolt_diameter/2,h+2,
                                (0,side*P.bolt_y,0),(0,0,1)))
        shape=shape.cut(cylinder(3.1,h-P.clamp_head_seat_z+1,
                                (0,side*P.bolt_y,P.clamp_head_seat_z),(0,0,1)))
    return shape.clean()


def camera_frame():
    s,o,z=P.frame_size/2,P.frame_opening/2,P.pivot_z
    shape=box(P.frame_rear_x,P.frame_front_x,-s,s,z-s,z+s)
    shape=shape.cut(box(P.frame_rear_x-1,P.frame_front_x+1,-o,o,z-o,z+o))
    for y in (-P.pcb_pitch/2,P.pcb_pitch/2):
        for zz in (z-P.pcb_pitch/2,z+P.pcb_pitch/2):
            shape=shape.union(cylinder(P.standoff_diameter/2,P.frame_rear_x,
                                      (0,y,zz),(1,0,0)))
            shape=shape.cut(cylinder(P.printed_m2_hole/2,P.frame_front_x+2,
                                    (-1,y,zz),(1,0,0)))
    for side in (-1,1):
        ya,yb=sorted((side*P.frame_ear_inner,side*P.frame_ear_outer))
        shape=shape.union(cylinder(P.pivot_radius,yb-ya,
                                  (P.pivot_x,ya,P.pivot_z),(0,1,0)))
        shape=shape.cut(cylinder(P.bolt_diameter/2,yb-ya+2,
                                (P.pivot_x,ya-1,P.pivot_z),(0,1,0)))
    return shape.clean()


def camera_references():
    z=P.pivot_z
    pcb=box(-P.pcb_thickness,0,-19,19,z-19,z+19)
    for y in (-17,17):
        for zz in (z-17,z+17):
            pcb=pcb.cut(cylinder(P.pcb_hole/2,3,(-2,y,zz),(1,0,0)))
    holder=box(0,10.4,-8.5,8.5,z-8.5,z+8.5)
    # Bounding shape only; does not assert the actual intermediate lens profile.
    lens=cylinder(8,P.lens_front_extent-10.4,(10.4,0,z),(1,0,0))
    # The plug outlines cannot be measured reliably from the photograph.
    # Backside is kept open on BOTH sides, so photo mirroring is immaterial to fit.
    usb=box(-P.pcb_thickness-P.rear_keepout_depth,-P.pcb_thickness,-8,8,z-21,z-7)
    pins=[box(-P.pcb_thickness-P.rear_keepout_depth,-P.pcb_thickness,ya,yb,z-15,z-3)
          for ya,yb in [(-25,-13),(13,25)]]
    return {"pcb":pcb,"lens_envelope":holder.union(lens),
            "usb_plug_keepout_UNMEASURED":usb,
            "pin_left_keepout_UNMEASURED":pins[0],
            "pin_right_keepout_UNMEASURED":pins[1]}


def hardware_references():
    """Nominal shafts + conservative head/nut envelopes; not thread models."""
    fixed=[]
    moving=[]
    top=P.clamp_head_seat_z
    for side in (-1,1):
        y=side*P.bolt_y
        fixed.append(cylinder(1.5,25,(0,y,top),(0,0,-1)).union(
            cylinder(2.85,3,(0,y,top),(0,0,1))))
        # Ordinary M3 nut seats against the top of its pocket.
        fixed.append(hex_z(0,y,P.flange_bottom+P.nut_pocket_depth-2.4,2.4,5.5).cut(
            cylinder(1.5,4,(0,y,-8),(0,0,1))))
        outer=P.frame_ear_outer+P.hinge_gap+P.fixed_ear_thickness
        # M3x16; bearing face at the outside of the fixed upright.
        start=(P.pivot_x,side*outer,P.pivot_z)
        moving.append(cylinder(1.5,16,start,(0,-side,0)).union(
            cylinder(2.85,3,start,(0,side,0))))
        # Two ordinary M3 nuts, matching user's hardware; jam them after adjustment.
        for offset in (0,2.4):
            nut=cylinder(3.2,2.4,(P.pivot_x,side*(P.frame_ear_inner-offset),P.pivot_z),(0,-side,0))
            nut=nut.cut(cylinder(1.5,7,(P.pivot_x,side*(P.frame_ear_inner+1),P.pivot_z),(0,-side,0)))
            moving.append(nut)
    for y in (-17,17):
        for z in (P.pivot_z-17,P.pivot_z+17):
            # Head at PCB rear, nut at carrier front: smaller rear footprint near 3Pin.
            moving.append(cylinder(1,10,(-P.pcb_thickness,y,z),(1,0,0)).union(
                cylinder(1.9,2,(-P.pcb_thickness,y,z),(-1,0,0))))
            nut=cylinder(2.35,1.6,(P.frame_front_x,y,z),(1,0,0))
            moving.append(nut.cut(cylinder(1,3,(P.frame_front_x-0.5,y,z),(1,0,0))))
    return fixed,moving


def compound(items):
    return cq.Compound.makeCompound([s.val() if isinstance(s,cq.Workplane) else s for s in items])


def normalized(shape, rotation=None):
    if rotation:
        shape=shape.rotate((0,0,0),rotation[0],rotation[1])
    bb=shape.val().BoundingBox()
    return shape.translate((-(bb.xmin+bb.xmax)/2,-(bb.ymin+bb.ymax)/2,-bb.zmin))


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/"reference_only").mkdir(exist_ok=True)
    parts={"01_lower_clamp_DRAFT":clamp_bottom(),"02_upper_clamp_DRAFT":clamp_top(),
           "03_front_camera_carrier_DRAFT":camera_frame()}
    # Orientations are candidates. Final support/orientation must be checked in a slicer.
    orientations={"01_lower_clamp_DRAFT":((0,1,0),90),
                  "02_upper_clamp_DRAFT":((0,1,0),90),
                  "03_front_camera_carrier_DRAFT":((0,1,0),-90)}
    colors=[(0.92,0.39,0.17),(0.96,0.56,0.18),(0.12,0.55,0.76)]
    assembly=cq.Assembly(name="V2_REVIEW_DRAFT_35mm_motor")
    metadata=[]
    for (name,shape),color in zip(parts.items(),colors):
        if len(shape.solids().vals())!=1 or not shape.val().isValid():
            raise RuntimeError(f"Invalid or disconnected part: {name}")
        cq.exporters.export(shape,str(OUT/f"{name}.step"))
        posed=normalized(shape,orientations[name])
        cq.exporters.export(posed,str(OUT/f"{name}.stl"),tolerance=0.03,angularTolerance=0.08)
        cq.exporters.export(shape,str(OUT/"reference_only"/f"{name}_assembly_pose.stl"),tolerance=0.04)
        assembly.add(shape,name=name,color=cq.Color(*color))
        bb=posed.val().BoundingBox()
        metadata.append({"name":name,"volume_mm3":shape.val().Volume(),
                         "stl_size_mm":[bb.xlen,bb.ylen,bb.zlen]})
    refs=camera_references()
    refs["motor_35mm_USER_MEASUREMENT"]=box(-17.5,17.5,-17.5,17.5,-17.5,17.5)
    fixed,moving=hardware_references()
    refs["fixed_hardware_envelopes"]=compound(fixed)
    refs["camera_hardware_envelopes"]=compound(moving)
    for name,shape in refs.items():
        cq.exporters.export(shape,str(OUT/"reference_only"/f"{name}.stl"),tolerance=0.04)
        color=(0.2,0.58,0.36) if name=="pcb" else (0.25,0.29,0.34)
        if "keepout" in name: color=(0.72,0.28,0.66,0.25)
        assembly.add(shape,name=name,color=cq.Color(*color))
    assembly.save(str(OUT/"V2_REVIEW_assembly.step"))
    assembly.save(str(OUT/"V2_REVIEW_assembly.glb"))
    metadata={"status":"REVIEW_DRAFT_NOT_RELEASED_FOR_PRINT", "parameters":asdict(P),
              "parts":metadata,
              "unconfirmed":["12 mm unobstructed motor band and location", "motor shell tolerance and clamping surfaces",
                             "connector/plug envelopes and cable bend space",
                             "front PCB component clearance at support pads", "gripper, robot and full motion clearance"]}
    (OUT/"design_parameters.json").write_text(json.dumps(metadata,indent=2),encoding="utf-8")
    print(json.dumps(metadata,indent=2),flush=True)


if __name__=="__main__":
    main()
    if sys.platform=="win32":
        sys.stdout.flush(); sys.stderr.flush(); os._exit(0)
