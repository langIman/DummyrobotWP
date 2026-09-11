"""Local USB camera preview. Run with: python camera_stream.py"""
import os
import threading
import time
from collections import deque

import cv2
from flask import Flask, Response, jsonify, request

app = Flask(__name__)
condition = threading.Condition()
latest = None
sequence = 0
dimensions = None
error = None
frame_times = deque(maxlen=120)
settings = {'width': 1920, 'height': 1080, 'fps': 120}
revision = 0
applied_revision = -1
reported_fps = None
enabled = True
device_active = False


def capture():
    global latest, error
    while True:
        with condition:
            condition.wait_for(lambda: enabled)
        try:
            capture_session()
        except Exception as exc:
            print(f'Camera reconnect: {exc}', flush=True)
        with condition:
            latest = None
            frame_times.clear()
            error = "摄像头已断开，正在自动重连…" if enabled else None
        time.sleep(1)


def capture_session():
    global latest, sequence, dimensions, error, applied_revision, reported_fps, device_active
    with condition:
        if not enabled:
            return
        target, version = settings.copy(), revision
        device_active = True
    cap = None
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            error = "无法打开摄像头，请关闭其他占用摄像头的软件。"
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, target['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target['height'])
        cap.set(cv2.CAP_PROP_FPS, target['fps'])
        while True:
            if not enabled or version != revision:
                return
            ok, frame = cap.read()
            if not ok:
                error = "暂时无法读取摄像头画面。"
                return
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                continue
            with condition:
                if not enabled or version != revision:
                    return
                latest = jpeg.tobytes()
                applied_revision = version
                reported_fps = cap.get(cv2.CAP_PROP_FPS)
                dimensions = [frame.shape[1], frame.shape[0]]
                error = None
                sequence += 1
                frame_times.append(time.monotonic())
                condition.notify_all()
    finally:
        if cap is not None:
            cap.release()
        with condition:
            device_active = False
            condition.notify_all()


@app.get('/')
def index():
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>腕部摄像头 · 实时预览</title>
<style>
body{margin:0;background:#10151e;color:#e7edf6;font:16px system-ui,sans-serif}
main{max-width:1100px;margin:40px auto;padding:0 24px}h1{font-size:28px;margin-bottom:8px}
p{color:#a9b7cb;line-height:1.7}.bar{display:flex;align-items:center;gap:16px;margin:22px 0}
.bar{flex-wrap:wrap}select{background:#202c3e;color:#e7edf6;border:1px solid #45546a;border-radius:8px;padding:9px;font-size:15px}label{display:flex;gap:8px;align-items:center}button:disabled{opacity:.5;cursor:wait}
button{background:#7de0c3;color:#10251e;border:0;border-radius:9px;padding:11px 20px;cursor:pointer;font-size:15px}
.screen{background:#05080d;border:1px solid #2a3648;border-radius:16px;overflow:hidden;min-height:220px;display:flex;align-items:center;justify-content:center}
img{display:block;width:100%;height:auto;max-height:75vh;object-fit:contain}
#status{color:#7de0c3}small{color:#8190a6}
</style><main><h1>腕部摄像头</h1><p>DECXIN Camera · 本机实时预览</p>
<div class="bar">
<label>分辨率 <select id="resolution"><option value="640x360">640 × 360</option><option value="1280x720">1280 × 720</option><option value="1920x1080" selected>1920 × 1080</option></select></label>
<label>目标采集帧率 <select id="fps"><option>15</option><option>30</option><option>60</option><option selected>120</option></select></label>
<button id="apply">应用设置</button>
<label>网页预览上限 <select id="preview-fps"><option>15</option><option selected>30</option><option>60</option><option>120</option></select> FPS</label>
</div><p id="setting-note" aria-live="polite">可尝试不同组合；设备可能回退到支持的模式，以实际输出为准。</p>
<div class="bar"><button id="toggle" disabled>正在连接…</button><span id="status">正在连接…</span></div>
<div class="screen"><img id="video" alt="等待摄像头画面"></div>
<p><small>仅本机访问，不录制视频。停止摄像头会释放设备、停止采集，USB 仍供电。关闭网页不会自动停止摄像头。</small></p></main>
<script>
let paused=true, generation=0, controller=null, imageUrl=null, changing=false;
let initialized=false, pendingVersion=null;
const res=document.getElementById('resolution'), fpsInput=document.getElementById('fps'), apply=document.getElementById('apply'), note=document.getElementById('setting-note');
apply.onclick=async()=>{
 apply.disabled=true;note.textContent='正在切换摄像头模式…';
 try{const [width,height]=res.value.split('x').map(Number);
 const r=await fetch('/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({width,height,fps:Number(fpsInput.value)})});
 const d=await r.json();if(!r.ok)throw new Error(d.error||'设置失败');pendingVersion=d.revision;
 }catch(e){note.textContent='应用失败：'+e.message;}finally{apply.disabled=false;}
};
const v=document.getElementById('video'), b=document.getElementById('toggle'), s=document.getElementById('status');
async function preview(token){
 if(paused || token!==generation)return;
 const request=new AbortController();controller=request;
 const started=performance.now();
 const timeout=setTimeout(()=>request.abort(),3000);let delay=0;
 try{
  const response=await fetch('/frame?t='+Date.now(),{cache:'no-store',signal:request.signal});
  if(!response.ok)throw new Error('Waiting for camera');
  const blob=await response.blob();
  if(paused || token!==generation)return;
  const old=imageUrl;imageUrl=URL.createObjectURL(blob);v.src=imageUrl;
  if(old)URL.revokeObjectURL(old);
 }catch(e){delay=700;}finally{
  clearTimeout(timeout);
  if(controller===request)controller=null;
  const interval=1000/Number(document.getElementById('preview-fps').value);
  if(!paused && token===generation)setTimeout(()=>preview(token),Math.max(delay,interval-(performance.now()-started),0));
 }
}
function syncPreview(running){
 if(paused===!running)return;
 paused=!running;generation++;if(controller)controller.abort();
 if(paused){v.removeAttribute('src');v.alt='摄像头已停止';if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=null;}
 else{v.alt='等待摄像头画面';preview(generation);}
}
b.onclick=async()=>{
 changing=true;b.disabled=true;s.textContent=paused?'正在启动摄像头…':'正在停止摄像头…';
 try{const r=await fetch('/camera',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:paused})});
 if(!r.ok)throw new Error('操作失败');
 }catch(e){s.textContent=e.message;}finally{changing=false;await status();}
};
async function status(){try{const r=await fetch('/status');const d=await r.json();
 if(!initialized){res.value=`${d.settings.width}x${d.settings.height}`;fpsInput.value=String(d.settings.fps);initialized=true;}
 if(!changing){syncPreview(d.enabled);b.textContent=d.enabled?'停止摄像头':'启动摄像头';b.disabled=!d.enabled && d.device_active;}
 if(pendingVersion!==null && !d.enabled){note.textContent='设置已保存，下次启动摄像头时生效。';pendingVersion=null;}
 if(pendingVersion!==null && d.dimensions && d.applied_revision>=pendingVersion){
  note.textContent=`已应用请求：${d.settings.width} × ${d.settings.height} / ${d.settings.fps} FPS；实际输出：${d.dimensions.join(' × ')}，设备报告 ${Number(d.reported_fps).toFixed(1)} FPS。实测见下方。`;
  pendingVersion=null;apply.disabled=false;
 }
 if(!paused)s.textContent=d.error||(d.dimensions?`直播中 · ${d.dimensions[0]} × ${d.dimensions[1]} · 实测采集 ${d.fps.toFixed(1)} FPS`:'正在连接…');
 else s.textContent=d.device_active?'正在停止摄像头…':'摄像头已停止 · 设备已释放';
 }catch(e){s.textContent='服务已断开';}}
setInterval(status,1500);status();
</script></html>'''


@app.get('/status')
def status():
    with condition:
        now = time.monotonic()
        recent = [t for t in frame_times if now - t <= 2.0]
        fps = ((len(recent) - 1) / (recent[-1] - recent[0])
               if len(recent) > 1 and now - recent[-1] < 1.0 else 0.0)
        return jsonify(dimensions=dimensions, error=error, frames=sequence, fps=round(fps, 1),
                       settings=settings, applied_revision=applied_revision, reported_fps=reported_fps,
                       enabled=enabled, device_active=device_active)


@app.post('/camera')
def camera_control():
    global enabled, revision, latest, dimensions, error
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or type(data.get('enabled')) is not bool:
        return jsonify(error='enabled must be a boolean'), 400
    with condition:
        if enabled != data['enabled']:
            enabled = data['enabled']
            revision += 1
            latest = None
            dimensions = None
            frame_times.clear()
            error = '正在启动摄像头…' if enabled else None
            condition.notify_all()
        if not enabled:
            condition.wait_for(lambda: not device_active, timeout=5)
        return jsonify(enabled=enabled, device_active=device_active)


@app.post('/settings')
def configure():
    global settings, revision, latest, dimensions, error
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or any(type(data.get(k)) is not int for k in ('width', 'height', 'fps')):
        return jsonify(error='请输入有效的分辨率和帧率。'), 400
    if (data['width'], data['height']) not in ((640, 360), (1280, 720), (1920, 1080)) or data['fps'] not in (15, 30, 60, 120):
        return jsonify(error='不支持的选项。'), 400
    with condition:
        settings = {k: data[k] for k in ('width', 'height', 'fps')}
        revision += 1
        latest = None
        dimensions = None
        frame_times.clear()
        error = '正在切换摄像头模式…'
        return jsonify(revision=revision)


@app.get('/video')
def video():
    def frames():
        seen = -1
        while True:
            with condition:
                condition.wait_for(lambda: sequence != seen and latest is not None, timeout=5)
                if latest is None or sequence == seen:
                    continue
                data, seen = latest, sequence
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
    return Response(frames(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store'})


@app.get('/frame')
def frame():
    with condition:
        data = latest
    if data is None:
        return Response(status=503, headers={'Cache-Control': 'no-store'})
    return Response(data, mimetype='image/jpeg', headers={'Cache-Control': 'no-store'})


if __name__ == '__main__':
    host = os.environ.get('WRIST_CAMERA_HOST', '127.0.0.1')
    port = int(os.environ.get('WRIST_CAMERA_PORT', '8766'))
    threading.Thread(target=capture, daemon=True).start()
    print(f'Wrist camera preview: http://{host}:{port}/', flush=True)
    app.run(host=host, port=port, threaded=True, debug=False)
