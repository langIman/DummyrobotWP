"""Dimensioned review PNGs rendered from the exported assembly geometry."""
import math
import os
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import vtk
from build_v2 import P, ROOT, OUT

DRAWINGS=ROOT/"drawings"
FONT="C:/Windows/Fonts/msyh.ttc"
INK="#172d3e"
MUTED="#526573"
ORANGE="#ce5d18"
TEAL="#087e8b"
PURPLE="#955399"


def font(size): return ImageFont.truetype(FONT,size)


def text(im,xy,s,size=24,fill=INK,anchor=None):
    ImageDraw.Draw(im).text(xy,s,font=font(size),fill=fill,anchor=anchor)


def pill(im,box,s,fill="#e9f2f4",color=INK,size=23):
    d=ImageDraw.Draw(im)
    d.rounded_rectangle(box,12,fill=fill)
    text(im,((box[0]+box[2])/2,(box[1]+box[3])/2),s,size,color,"mm")


def render(name,items,view="iso",size=(850,850),center=(0,0,30),scale=68):
    ren=vtk.vtkRenderer(); ren.SetBackground(0.974,0.984,0.990)
    win=vtk.vtkRenderWindow(); win.SetOffScreenRendering(True); win.SetMultiSamples(4)
    win.SetSize(*size); win.AddRenderer(ren)
    for stem,color,opacity in items:
        reader=vtk.vtkSTLReader(); reader.SetFileName(str(OUT/"reference_only"/(stem+".stl"))); reader.Update()
        mapper=vtk.vtkPolyDataMapper(); mapper.SetInputConnection(reader.GetOutputPort())
        actor=vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color); actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetAmbient(0.35); actor.GetProperty().SetDiffuse(0.65)
        ren.AddActor(actor)
    direction={"iso":(1.6,-1.8,1.15),"front":(1,0,0),"side":(0,-1,0),"rear":(-1,0,0)}[view]
    cam=ren.GetActiveCamera(); cam.SetFocalPoint(*center)
    cam.SetPosition(*(center[i]+direction[i]*250 for i in range(3)))
    cam.SetViewUp(0,0,1); cam.ParallelProjectionOn(); cam.SetParallelScale(scale)
    ren.ResetCameraClippingRange(); win.Render()
    grab=vtk.vtkWindowToImageFilter(); grab.SetInput(win); grab.ReadFrontBufferOff(); grab.Update()
    writer=vtk.vtkPNGWriter(); writer.SetFileName(str(DRAWINGS/(name+"_render.png")))
    writer.SetInputConnection(grab.GetOutputPort()); writer.Write()
    im=Image.open(DRAWINGS/(name+"_render.png")).convert("RGB")
    def project(point):
        ren.SetWorldPoint(*point,1); ren.WorldToDisplay(); x,y,_=ren.GetDisplayPoint()
        return (x,size[1]-y)
    return im,project,win


def arrow(d,a,b,color=INK,width=2,ends=True):
    d.line([a,b],fill=color,width=width)
    dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
    if length<1:return
    ux,uy=dx/length,dy/length
    for tip,sign in [(a,1),(b,-1)] if ends else [(b,-1)]:
        bx,by=tip[0]+sign*ux*10,tip[1]+sign*uy*10
        d.polygon([tip,(bx-uy*4,by+ux*4),(bx+uy*4,by-ux*4)],fill=color)


def dimension(im,project,a,b,offset,label,color=INK,size=22,shift=(0,0)):
    d=ImageDraw.Draw(im); a=project(a); b=project(b)
    c=(a[0]+offset[0],a[1]+offset[1]); e=(b[0]+offset[0],b[1]+offset[1])
    d.line([a,c],fill=color,width=1); d.line([b,e],fill=color,width=1)
    arrow(d,c,e,color)
    mid=((c[0]+e[0])/2+shift[0],(c[1]+e[1])/2+shift[1])
    bounds=d.textbbox(mid,label,font=font(size),anchor="mm")
    d.rounded_rectangle((bounds[0]-5,bounds[1]-3,bounds[2]+5,bounds[3]+3),4,fill="#f8fbfc")
    text(im,mid,label,size,color,"mm")


def page(title,subtitle,size=(1900,1400)):
    im=Image.new("RGB",size,"#f8fbfc"); d=ImageDraw.Draw(im)
    d.rectangle((0,0,size[0],12),fill=TEAL)
    text(im,(55,35),title,40)
    text(im,(55,100),subtitle,23,MUTED)
    text(im,(55,size[1]-43),"V2 尺寸核对草案 · 单位 mm · 2026-09-12 · 示意图不代表实物装配已验证",20,MUTED)
    return im


PARTS=[("01_lower_clamp_DRAFT_assembly_pose",(0.91,0.37,0.12),1),
       ("02_upper_clamp_DRAFT_assembly_pose",(0.98,0.58,0.19),1),
       ("03_front_camera_carrier_DRAFT_assembly_pose",(0.10,0.55,0.75),1)]
MOTOR=("motor_35mm_USER_MEASUREMENT",(0.42,0.47,0.52),1)
CAM=[("pcb",(0.13,0.46,0.27),1),("lens_envelope",(0.20,0.23,0.26),1)]
HARD=[("fixed_hardware_envelopes",(0.52,0.58,0.63),1),
      ("camera_hardware_envelopes",(0.52,0.58,0.63),1)]
KEEP=[("usb_plug_keepout_UNMEASURED",(0.74,0.37,0.72),0.28),
      ("pin_left_keepout_UNMEASURED",(0.74,0.37,0.72),0.20),
      ("pin_right_keepout_UNMEASURED",(0.74,0.37,0.72),0.20)]


def main():
    DRAWINGS.mkdir(exist_ok=True)
    windows=[]
    # An immediate visual answer to the user's question about the 12 mm band.
    im=page("12 mm 指的是哪里？", "35 mm 是电机大小；12 mm 是夹箍沿电机长度占用的宽度。",(1500,1040))
    view,pr,win=render("band",[PARTS[0],MOTOR],size=(900,720),center=(0,0,-2),scale=37)
    windows.append(win)
    dimension(view,pr,(-6,-29.5,-7),(6,-29.5,-7),(0,65),"12  夹箍宽度",ORANGE,25)
    dimension(view,pr,(-17.5,17.5,17.5),(17.5,17.5,17.5),(0,-45),"35  电机长度",TEAL,24)
    im.paste(view,(20,155))
    pill(im,(935,210,1435,278),"灰色：你说的 35 mm 电机")
    pill(im,(935,303,1435,371),"橙色：下半个夹箍",fill="#fff0e3",color=ORANGE)
    for i,line in enumerate(["夹箍只包住电机的一小段。","上半个装上后，像腰带合拢。", "", "我想确认的是：", "这条 12 mm 宽的腰带放上去，", "会不会碰到电线或旁边零件。", "", "12 是设计提案，可以改小或改位置。"]):
        text(im,(945,414+i*43),line,24,INK)
    text(im,(65,908),"这里暂时只画下半夹箍，方便看见电机。电机旁边的夹爪、支架和电线还没有纳入模型。",24,MUTED)
    im.save(DRAWINGS/"00_what_12mm_means.png")

    im=page("V2 / 装配尺寸总览", "橙色夹具 · 蓝色摄像头框 · 灰色电机 · 紫色插头预留空间（待实测）")
    iso,pr,win=render("assembly_iso",PARTS+[MOTOR]+CAM+HARD,size=(600,770),center=(0,0,29),scale=67)
    windows.append(win); im.paste(iso,(20,195)); text(im,(65,153),"01  立体装配",25)
    front,pr,win=render("assembly_front",PARTS+[MOTOR]+CAM+HARD,"front",(630,770),center=(0,0,29),scale=69)
    windows.append(win)
    dimension(front,pr,(0,-24,P.pivot_z+24),(0,24,P.pivot_z+24),(0,-40),"框外宽 48",TEAL)
    dimension(front,pr,(0,-32.4,21.5),(0,32.4,21.5),(0,105),"打印件外宽 64.8",ORANGE,20)
    dimension(front,pr,(0,-17.5,-17.5),(0,17.5,-17.5),(0,40),"电机宽 35",TEAL)
    dimension(front,pr,(0,32.4,-21.5),(0,32.4,P.pivot_z+24),(60,0),f"{P.pivot_z+45.5:g}",INK,21,shift=(15,0))
    im.paste(front,(620,195)); text(im,(665,153),"02  从镜头前方看",25)
    side,pr,win=render("assembly_side",PARTS+[MOTOR]+CAM+HARD+KEEP,"side",(620,770),center=(-5,0,29),scale=69)
    windows.append(win)
    dimension(side,pr,(-17.5,0,-17.5),(17.5,0,-17.5),(0,47),"电机长 35",TEAL)
    dimension(side,pr,(-6,0,-21.5),(6,0,-21.5),(0,80),"夹持宽 12（待确认）",ORANGE,19)
    dimension(side,pr,(0,0,P.pivot_z),(25.3,0,P.pivot_z),(0,-135),"镜头前伸 25.3",INK,20)
    dimension(side,pr,(-41.6,0,P.pivot_z-21),(-1.6,0,P.pivot_z-21),(0,48),"拟留 40",PURPLE,21)
    dimension(side,pr,(25.3,0,0),(25.3,0,P.pivot_z),(65,0),f"{P.pivot_z:g}",INK,22,shift=(17,0))
    im.paste(side,(1250,195)); text(im,(1295,153),"03  从侧面看",25)
    pill(im,(55,990,605,1050),"已确认：电机 35 × 35 × 35",color=TEAL)
    pill(im,(655,990,1245,1050),"已确认：PCB 38 × 38 / 孔距 34",color=TEAL)
    pill(im,(1290,990,1840,1050),"待确认：夹持位置、插头及线空间",fill="#fff0e3",color=ORANGE,size=21)
    lines=["夹具左右内宽 35.6；上下接触面距离 35.0。夹住标称电机时，两半之间仍留 2.0 收紧间隙。",
           "摄像头框改在镜头侧，PCB 背面不跨横梁；两个接口都按插着线预留。",
           f"{P.pivot_z:g} 是电机中心到摄像头中心的高度。为容纳暂拟的插头空间，草案整体较高，实测后可再优化。",
           "图中不含真实夹爪与邻近机构；35 mm 电机是按实测值生成的方盒，未引用 Dummy STEP。"]
    for i,line in enumerate(lines):text(im,(65,1090+i*52),line,25,MUTED)
    im.save(DRAWINGS/"01_assembly_dimensions.png")

    im=page("V2 / 摄像头框与接口空间", "安装方向改变：打印框在镜头这一侧；接口所在的背面尽量敞开。")
    front,pr,win=render("carrier_front",[PARTS[2]],"front",(850,660),center=(0,0,P.pivot_z),scale=35)
    windows.append(win)
    dimension(front,pr,(6,-24,P.pivot_z+24),(6,24,P.pivot_z+24),(0,-40),"外框 48")
    dimension(front,pr,(6,-17,P.pivot_z-17),(6,17,P.pivot_z-17),(0,58),"孔距 34",TEAL)
    dimension(front,pr,(6,17,P.pivot_z-17),(6,17,P.pivot_z+17),(145,0),"34",TEAL,22,shift=(18,0))
    dimension(front,pr,(6,-15,P.pivot_z),(6,15,P.pivot_z),(0,0),"中央开口 30",TEAL)
    im.paste(front,(35,205));text(im,(65,157),"04  打印框正面 / 四个 M2 通孔 Ø2.4",25)
    # True side geometry, plus a deliberately spread-out dimension chain below.
    side,pr,win=render("carrier_side",[PARTS[2]]+CAM+HARD[1:]+KEEP,"side",(900,660),center=(-8,0,P.pivot_z-3),scale=36)
    windows.append(win)
    dimension(side,pr,(-41.6,0,P.pivot_z-21),(-1.6,0,P.pivot_z-21),(0,65),"插头预留深度 40（拟定）",PURPLE,22)
    dimension(side,pr,(-1.6,0,P.pivot_z+19),(0,0,P.pivot_z+19),(0,-50),"PCB 厚 1.6",TEAL,20,shift=(-65,0))
    dimension(side,pr,(0,0,P.pivot_z+24),(3,0,P.pivot_z+24),(0,-45),"垫高 3",ORANGE,20,shift=(-40,-22))
    dimension(side,pr,(3,0,P.pivot_z+24),(6,0,P.pivot_z+24),(0,-20),"框厚 3",TEAL,20,shift=(58,0))
    im.paste(side,(930,205));text(im,(965,157),"05  侧面 / 镜头向右、插线向左",25)
    rows=[("四个支点", "Ø5.5，高 3；只支撑 PCB 四角的安装孔周围。"),
          ("M2×10 安装", "螺丝从电路板背面穿向镜头侧，M2 螺母位于打印框前面。"),
          ("USB-C 预留", "草案空间：宽 16 × 高 14 × 深 40；这些是设计预留值，不是插头实测值。"),
          ("3Pin 预留", "草案空间：宽 12 × 高 12 × 深 40；左右两侧均留空，方便换装方向。"),
          ("仍需核对", "3Pin 插头很靠近角孔，要确认下角 M2 螺丝头能放下，并留出插拔空间。"),
          ("线缆弯曲", "紫色方框只表示插头及直出空间，不代表线能在这里任意急弯。")]
    for i,(label,body) in enumerate(rows):
        yy=915+i*62
        text(im,(65,yy),label,24,TEAL);text(im,(300,yy),body,24,INK)
    im.save(DRAWINGS/"02_camera_and_connectors.png")

    im=page("V2 / 夹具、轴孔与现有螺丝", "所有尺寸对应当前 CAD 草案；螺纹余量按普通 M3 螺母厚 2.4、M2 螺母厚 1.6 估算。")
    # Dimension-only cross section at the clamp band; circles depict actual screw channels.
    d=ImageDraw.Draw(im); ox,oy,k=520,600,9
    def p(y,z):return (ox+y*k,oy-z*k)
    def rect(y0,z0,y1,z1,fill,outline=INK):
        a,b=p(y0,z1),p(y1,z0);d.rectangle((*a,*b),fill=fill,outline=outline,width=2)
    rect(-21.8,1,21.8,21.5,"#ffc27b");rect(-21.8,-21.5,21.8,-1,"#ee9149")
    rect(-17.8,-17.5,17.8,17.5,"#f8fbfc");rect(-17.5,-17.5,17.5,17.5,"#bac7ce")
    rect(-29.5,-7,-20.8,-1,"#ee9149");rect(20.8,-7,29.5,-1,"#ee9149")
    rect(-29.5,1,-20.8,21.5,"#ffc27b");rect(20.8,1,29.5,21.5,"#ffc27b")
    for y in (-25.5,25.5):
        rect(y-1.7,-7,y+1.7,21.5,"#f8fbfc")
        rect(y-3.1,17.5,y+3.1,21.5,"#f8fbfc")
    pr=lambda xyz:p(xyz[1],xyz[2])
    dimension(im,pr,(0,-17.8,17.5),(0,17.8,17.5),(0,-70),"内宽 35.6",TEAL,25)
    dimension(im,pr,(0,17.5,-17.5),(0,17.5,17.5),(-440,0),"35.0",TEAL,25,shift=(-32,0))
    dimension(im,pr,(0,-25.5,-7),(0,25.5,-7),(0,250),"两颗夹紧螺丝中心距 51",INK,23)
    arrow(d,p(29.5,0),(850,540),ORANGE,3,False);text(im,(805,498),"间隙 2.0",25,ORANGE)
    text(im,(145,215),"06  夹箍截面示意（为展示内腔，省略上方长支座）",24)
    text(im,(1060,220),"07  每套支架实际用量",30)
    rows=[("夹紧电机", "M3×25 × 2", "普通 M3 螺母 × 2"),
          ("调节俯仰", "M3×16 × 2", "普通 M3 螺母 × 4（每侧两颗对锁）"),
          ("固定 PCB", "M2×10 × 4", "M2 螺母 × 4")]
    for i,(a,b,c) in enumerate(rows):
        yy=300+i*155
        text(im,(1060,yy),a,25,TEAL);text(im,(1060,yy+44),b,29);text(im,(1060,yy+90),c,24,MUTED)
    text(im,(1060,805),"M3 通孔 Ø3.4",26)
    text(im,(1060,852),"夹紧螺丝头座 Ø6.2，沉入 4",26)
    text(im,(1060,899),"下夹块螺母槽：对边 5.8，深 2.8",26)
    text(im,(1060,946),"俯仰耳：活动侧厚 6 / 固定侧厚 4",25)
    text(im,(1060,993),"两耳之间留 0.4；耳外圆 Ø14",25)
    pill(im,(65,1100,1835,1164),"收紧原理：先接触电机的上、下表面；两半还留着缝，因此螺丝可以继续施加夹紧力。",size=25)
    text(im,(75,1198),"现有螺丝名义长度可覆盖装配厚度；单独拧紧两颗对锁螺母，避免靠过度压紧塑料来防松。",24,MUTED)
    text(im,(75,1244),"打印精度、实际螺母厚度和电机尺寸误差仍需试装核对；这不是已经完成的强度认证。",24,MUTED)
    im.save(DRAWINGS/"03_clamp_and_fasteners.png")
    # Only retain finished pages; raw render files are implementation intermediates.
    for path in DRAWINGS.glob("*_render.png"): path.unlink()
    print("Generated four dimensioned review pages",flush=True)


if __name__=="__main__":
    main()
    sys.stdout.flush();sys.stderr.flush();os._exit(0)
