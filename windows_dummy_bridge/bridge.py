"""V4 loopback HTTP -> USB CDC ASCII transport and passive monitor."""
import json
import math
import logging
from logging.handlers import RotatingFileHandler
import multiprocessing as mp
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from device import worker
from gripper_feedback import GripperFeedback
from camera_backend import serve as camera_worker
from motion_protocol import format_motion, motion_acknowledged

ROOT = Path(__file__).parent
JOINTS = [f'J{i}' for i in range(1, 7)]
CAMERA_CONFIG_PATH = ROOT / 'camera_config.json'
DEFAULT_CAMERA_CONFIG = dict(width=1280, height=720, fps=30, jpeg_quality=85)
SUPPORTED_CAMERA_RESOLUTIONS = [(1280, 720), (1920, 1080)]
SUPPORTED_CAMERA_FPS = [30, 60, 120]


def now_ms():
    return time.time_ns() // 1_000_000


class BridgeError(Exception):
    def __init__(self, status, code):
        self.status, self.code = status, code


class USBWorker:
    def __init__(self):
        self.proc = self.pipe = None

    def request(self, payload, read_timeout_ms):
        sent_to_worker = False
        try:
            if self.proc is None:
                ctx = mp.get_context('spawn')
                self.pipe, child = ctx.Pipe()
                self.proc = ctx.Process(target=worker, args=(child,), daemon=True)
                self.proc.start(); child.close()
            self.pipe.send(dict(payload=payload, read_timeout_ms=read_timeout_ms)); sent_to_worker = True
            if not self.pipe.poll(3.2): raise TimeoutError('local_worker_deadline')
            return self.pipe.recv()
        except (OSError, EOFError, TimeoutError) as exc:
            self.close()
            return dict(sent=None if sent_to_worker else False, bytes_written=None if sent_to_worker else 0,
                        write_status='unknown' if sent_to_worker else 'not_attempted', raw_response='', raw_response_hex='',
                        read_timed_out=True, error=dict(code='worker_unavailable', message=str(exc)), response_capture_complete=False)

    def close(self):
        if self.proc:
            if self.proc.is_alive() and self.pipe:
                try: self.pipe.send('close')
                except (OSError, EOFError): pass
                self.proc.join(.3)
            if self.proc.is_alive(): self.proc.terminate(); self.proc.join(.5)
            if self.proc.is_alive(): self.proc.kill(); self.proc.join(.5)
        if self.pipe: self.pipe.close()
        self.proc = self.pipe = None


class CameraWorker:
    """OpenCV runs separately so camera stalls cannot block serial requests."""
    def __init__(self, config):
        self.proc = self.pipe = None
        self.lock = threading.RLock()
        self.config = dict(config)

    def _request(self, message, timeout):
        try:
            if self.proc is None:
                ctx = mp.get_context('spawn')
                self.pipe, child = ctx.Pipe()
                self.proc = ctx.Process(
                    target=camera_worker, args=(child, dict(self.config)), daemon=True
                )
                self.proc.start()
                child.close()
            self.pipe.send(message)
            if not self.pipe.poll(timeout):
                raise TimeoutError('camera_worker_deadline')
            return self.pipe.recv()
        except (OSError, EOFError, TimeoutError, AssertionError) as exc:
            self._close()
            return dict(camera_status='unavailable', reason=str(exc), jpeg=None)

    def request(self, message, timeout=8):
        with self.lock:
            return self._request(message, timeout)

    def configure(self, config):
        with self.lock:
            previous = self.config
            self.config = dict(config)
            self._close()
            result = self._request('status', 10)
            driver = result.get('driver_mode') or {}
            applied = (
                driver.get('width') == config['width']
                and driver.get('height') == config['height']
                and driver.get('fourcc') == 'MJPG'
            )
            result['configuration_applied'] = applied
            if applied:
                result['configuration'] = dict(config)
                return result
            self.config = previous
            self._close()
            result['error'] = dict(code='camera_configuration_failed')
            return result

    def get_config(self):
        with self.lock:
            return dict(self.config)

    def close(self):
        with self.lock:
            self._close()

    def _close(self):
        if self.proc:
            if self.proc.is_alive() and self.pipe:
                try: self.pipe.send('close')
                except (OSError,EOFError): pass
                self.proc.join(.3)
            if self.proc.is_alive(): self.proc.terminate(); self.proc.join(.5)
        if self.pipe: self.pipe.close()
        self.proc=self.pipe=None

class Bridge:
    def __init__(self):
        self.usb = USBWorker()
        self.gripper_feedback = GripperFeedback()
        self.camera = CameraWorker(self.load_camera_config())
        self.instance_id = str(uuid.uuid4())
        self.gate = threading.Lock()
        self.waiter = threading.BoundedSemaphore(1)
        self.monitor_gate = threading.Lock()
        self.events = deque(maxlen=100)
        self.event_id = 0
        self.last_state = dict(usb_status='not_checked',positions=None,feedback_status='unavailable',received_at_ms=None)
        self.generation = 0
        self.signature = None
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.logger = logging.getLogger('dummy.v4')
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False
        self.log_handler = RotatingFileHandler(ROOT/'runtime'/'transport_trace.jsonl',
                                              maxBytes=2_097_152, backupCount=3, encoding='utf-8')
        self.logger.addHandler(self.log_handler)

    def health(self):
        from motion_backend import SERIAL_READ_STRATEGY
        return dict(status='alive', protocol_version=4, mode='transport',
                    serial_read_strategy=SERIAL_READ_STRATEGY,
                    instance_id=self.instance_id, bridge_time_ms=now_ms(), device_enabled=None)

    def capabilities(self):
        return dict(protocol_version=4, transport='usb_cdc_ascii',
                    endpoints=['GET /health','GET /capabilities','GET /state','GET /gripper/state','POST /command',
                               'POST /motion','POST /stop','GET /camera/status','GET /camera/frame','GET /camera/stream',
                               'GET /camera/config','POST /camera/config','GET /monitor','GET /'],
                    raw_ascii=True, multiline=True, fibre_supported=False,
                    joints=JOINTS, unit='degree', coordinate_space='hardware_joint',
                    default_speed=100, bridge_range_enforcement=False,
                    limits_informational=[[-170,170],[-75,90],[35,180],[-180,180],[-120,120],[-720,720]],
                    limits_source='repository_with_operator_j2_correction; installed firmware owns enforcement',
                    serial=dict(baudrate=115200, format='8N1', write_timeout_ms=250,
                                read_mode='nonblocking', read_poll_interval_ms=1),
                    io=dict(default_read_timeout_ms=500,max_read_timeout_ms=2000,max_command_bytes=16384,
                            max_response_bytes=65536,worker_deadline_ms=3200,max_waiting_requests=1),
                    confirmation='ASCII parsing acknowledgment only; not physical completion',
                    stop=dict(command='!STOP', automatic=False, disconnect_stops_motion=False),
                    feedback=dict(command='#GETJPOS',source='device_reported',device_timestamp=False),
                    gripper_feedback=dict(endpoint='/gripper/state',source='stlink_mainboard_cache',
                                          requires_stlink=True,read_only=True,device_timestamp=False),
                    camera=dict(transport='local_windows_uvc',index=0,endpoint='/camera/frame',
                                stream_endpoint='/camera/stream',stream_max_fps=30,
                                status_endpoint='/camera/status',config_endpoint='/camera/config',
                                supported_resolutions=[list(value) for value in SUPPORTED_CAMERA_RESOLUTIONS],
                                supported_fps=SUPPORTED_CAMERA_FPS,jpeg_quality_range=[40,95],independent_process=True),
                    deprecated=['/session','/unlock','/heartbeat','/lock','/validate-motion'])

    def log(self, kind, **data):
        with self.monitor_gate:
            self.event_id += 1
            event = dict(id=self.event_id, at_ms=now_ms(), kind=kind, **data)
            self.events.append(event)
            self.logger.info(json.dumps(event, ensure_ascii=True))

    def monitor(self):
        with self.monitor_gate:
            return dict(health=self.health(), state=self.last_state, events=list(self.events),
                        serial_busy=self.gate.locked(), camera_busy=False, passive=True)

    def camera_status(self):
        return self.camera.request('status')

    def camera_frame(self):
        result=self.camera.request('frame')
        jpeg=result.pop('jpeg',None)
        return result,jpeg

    @staticmethod
    def validate_camera_config(value, current=None):
        if not isinstance(value, dict):
            raise BridgeError(400, 'json_object_required')
        if set(value) - {'width', 'height', 'fps', 'jpeg_quality'}:
            raise BridgeError(422, 'unknown_fields')
        merged = dict(current or DEFAULT_CAMERA_CONFIG)
        merged.update(value)
        if any(type(merged.get(name)) is not int for name in ('width', 'height', 'fps', 'jpeg_quality')):
            raise BridgeError(422, 'camera_config_integers_required')
        if (merged['width'], merged['height']) not in SUPPORTED_CAMERA_RESOLUTIONS:
            raise BridgeError(422, 'unsupported_camera_resolution')
        if merged['fps'] not in SUPPORTED_CAMERA_FPS:
            raise BridgeError(422, 'unsupported_camera_fps')
        if not 40 <= merged['jpeg_quality'] <= 95:
            raise BridgeError(422, 'jpeg_quality_out_of_range')
        return merged

    @classmethod
    def load_camera_config(cls):
        try:
            value = json.loads(CAMERA_CONFIG_PATH.read_text(encoding='utf-8'))
            return cls.validate_camera_config(value)
        except (OSError, ValueError, BridgeError):
            return dict(DEFAULT_CAMERA_CONFIG)

    def camera_config(self):
        return dict(
            configuration=self.camera.get_config(),
            supported_resolutions=[list(value) for value in SUPPORTED_CAMERA_RESOLUTIONS],
            supported_fps=SUPPORTED_CAMERA_FPS,
            jpeg_quality=dict(min=40, max=95),
            persisted=CAMERA_CONFIG_PATH.exists(),
        )

    def set_camera_config(self, body):
        config = self.validate_camera_config(body, self.camera.get_config())
        result = self.camera.configure(config)
        if result.get('configuration_applied'):
            temporary = CAMERA_CONFIG_PATH.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
            temporary.replace(CAMERA_CONFIG_PATH)
            result['persisted'] = True
        return result

    def transact(self, command, timeout, path):
        payload = command.encode('ascii')
        if payload and not payload.endswith(b'\n'):
            payload += b'\n'
        if len(payload) > 16384: raise BridgeError(413,'command_too_large')
        started = time.monotonic()
        self.log('tx_attempt',path=path,command=payload.decode('ascii'),bytes_requested=len(payload))
        result = self.usb.request(payload,timeout)
        self.log('result',path=path,elapsed_ms=round((time.monotonic()-started)*1000,2),**result)
        return result

    def state(self):
        result = self.transact('#GETJPOS',500,'/state')
        positions = None
        for line in result['raw_response'].splitlines():
            fields = line.split()
            if len(fields)==7 and fields[0] in ('ok','okok'):
                try:
                    q=[float(x) for x in fields[1:]]
                    if all(math.isfinite(x) for x in q): positions=q
                except ValueError: pass
        status='error' if result.get('error') else ('responsive' if positions is not None else 'unresponsive')
        signature=(status,result.get('port'),result.get('device_id'))
        if signature!=self.signature: self.generation+=1
        self.signature=signature
        state=dict(usb_status=status,feedback_status='device_reported' if positions is not None else 'unavailable',
                   positions=positions,unit='degree',coordinate_space='hardware_joint',joints=JOINTS,
                   received_at_ms=result.get('received_at_ms') if positions is not None else None,
                   device_sample_at_ms=None,sample_age_ms=None,device_enabled=None,
                   port=result.get('port'),device_id=result.get('device_id'),generation=self.generation,
                   bridge_time_ms=now_ms(),transaction=result)
        with self.monitor_gate: self.last_state=state
        return state

    @staticmethod
    def finite(value):
        try: return type(value) in (int,float) and math.isfinite(value)
        except OverflowError: return False

    def dispatch(self,method,path,body=None):
        if method=='GET':
            if path=='/health': return self.health()
            if path=='/capabilities': return self.capabilities()
            if path=='/monitor': return self.monitor()
            if path=='/gripper/state': return self.gripper_feedback.read()
            if path=='/camera/status': return self.camera_status()
            if path=='/camera/config': return self.camera_config()
            if path=='/camera/frame':
                result,jpeg=self.camera_frame()
                result['jpeg_bytes']=len(jpeg) if jpeg else 0
                result['_jpeg']=jpeg
                return result
        if method=='POST' and path=='/camera/config':
            return self.set_camera_config(body)
        if path in ('/session','/unlock','/heartbeat','/lock','/validate-motion'):
            raise BridgeError(410,'endpoint_removed_in_v4')
        if not (method=='GET' and path=='/state') and not (method=='POST' and path in ('/command','/motion','/stop')):
            raise BridgeError(404,'not_found')
        if method=='POST' and not isinstance(body,dict): raise BridgeError(400,'json_object_required')
        if not self.gate.acquire(blocking=False):
            if not self.waiter.acquire(blocking=False): raise BridgeError(503,'io_busy_not_sent')
            try: acquired=self.gate.acquire(timeout=2.75)
            finally: self.waiter.release()
            if not acquired: raise BridgeError(503,'io_busy_not_sent')
        try:
            if method=='GET': return self.state()
            if path=='/command':
                if set(body)-{'command','read_timeout_ms'}: raise BridgeError(422,'unknown_fields')
                command=body.get('command')
                if not isinstance(command,str) or not command.isascii(): raise BridgeError(422,'ascii_text_required')
                timeout=body.get('read_timeout_ms',500)
                if type(timeout) is not int or not 0<=timeout<=2000: raise BridgeError(422,'read_timeout_ms_out_of_range')
                return self.transact(command,timeout,path)
            if path=='/motion':
                if set(body)-{'positions','speed','unit','coordinate_space'}: raise BridgeError(422,'unknown_fields')
                if body.get('unit')!='degree' or body.get('coordinate_space')!='hardware_joint':
                    raise BridgeError(422,'unit_or_coordinate_mismatch')
                q=body.get('positions')
                if not isinstance(q,list) or len(q)!=6 or not all(self.finite(v) for v in q):
                    raise BridgeError(422,'six_finite_axes_required')
                speed=body.get('speed',100)
                if not self.finite(speed): raise BridgeError(422,'finite_speed_required')
                try:
                    command = format_motion(q, speed)
                except ValueError:
                    raise BridgeError(422, 'motion_command_exceeds_firmware_buffer')
                result=self.transact(command,500,path)
                result.update(accepted=motion_acknowledged(result),execution_complete=False)
                return result
            if body: raise BridgeError(422,'empty_object_required')
            result=self.transact('!STOP',500,path)
            result['stop']=dict(acknowledged=result.get('sent') is True and not result.get('error') and
                                'Stopped ok' in result['raw_response'].splitlines())
            return result
        finally:
            self.gate.release()

    def close(self):
        with self.gate: self.usb.close()
        self.camera.close()
        self.log_handler.close()
        self.logger.removeHandler(self.log_handler)


def handler_for(bridge):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(8)

        def reply(self,status,data,html=False):
            raw=data if html else json.dumps(data,ensure_ascii=True,allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store')
            self.send_header('Connection','close')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(raw)

        def stream_camera(self):
            boundary = b'frame'
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            last_frame_id = None
            try:
                while True:
                    cycle_started = time.monotonic()
                    result, jpeg = bridge.camera_frame()
                    frame_id = result.get('frame_id')
                    if jpeg and frame_id != last_frame_id:
                        headers = (
                            b'--' + boundary + b'\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            + f'Content-Length: {len(jpeg)}\r\n'.encode('ascii')
                            + f'X-Frame-Id: {frame_id}\r\n'.encode('ascii')
                            + f"X-Captured-At-Ms: {result.get('captured_at_ms', '')}\r\n\r\n".encode('ascii')
                        )
                        self.wfile.write(headers)
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                        last_frame_id = frame_id
                    elapsed = time.monotonic() - cycle_started
                    time.sleep(max(.002, (1 / 30) - elapsed))
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError, OSError):
                self.close_connection = True

        def log_message(self, format, *args):
            if urlsplit(self.path).path not in ('/camera/frame', '/camera/stream'):
                super().log_message(format, *args)

        def run_request(self):
            try:
                if self.headers.get('Host','') not in ('127.0.0.1:8765','localhost:8765','127.0.0.1:18765','localhost:18765'):
                    raise BridgeError(403,'loopback_host_required')
                origin=self.headers.get('Origin')
                if origin and origin not in ('http://127.0.0.1:8765','http://localhost:8765','http://127.0.0.1:18765','http://localhost:18765'):
                    raise BridgeError(403,'foreign_browser_origin')
                path=urlsplit(self.path).path
                if self.command=='GET' and path=='/':
                    return self.reply(200,(ROOT/'dashboard.html').read_bytes(),html=True)
                if self.command=='GET' and path=='/camera/stream':
                    return self.stream_camera()
                body=None
                if self.command=='POST':
                    if self.headers.get('Content-Type','').split(';')[0]!='application/json': raise BridgeError(415,'json_required')
                    if self.headers.get('Transfer-Encoding'): raise BridgeError(400,'chunked_not_supported')
                    try: length=int(self.headers.get('Content-Length','0'))
                    except ValueError: raise BridgeError(400,'invalid_content_length')
                    if not 0<length<=65536: raise BridgeError(413,'body_size_invalid')
                    try:
                        def invalid_constant(value): raise ValueError(value)
                        body=json.loads(self.rfile.read(length).decode('utf-8'),parse_constant=invalid_constant)
                    except (ValueError,UnicodeError): raise BridgeError(400,'invalid_json')
                result=bridge.dispatch(self.command,path,body)
                if path=='/camera/frame' and result.get('_jpeg'):
                    jpeg=result.pop('_jpeg')
                    self.send_response(200)
                    self.send_header('Content-Type','image/jpeg')
                    self.send_header('Content-Length',str(len(jpeg)))
                    self.send_header('Cache-Control','no-store')
                    self.send_header('X-Frame-Id',str(result.get('frame_id','')))
                    self.send_header('X-Captured-At-Ms',str(result.get('captured_at_ms','')))
                    self.send_header('Connection','close')
                    self.end_headers(); self.wfile.write(jpeg); return
                if path=='/camera/frame':
                    result.pop('_jpeg',None)
                    return self.reply(503,result)
                self.reply(503 if result.get('error') else 200,result)
            except BridgeError as exc:
                self.reply(exc.status,dict(error=dict(code=exc.code),sent=False,bytes_written=0))
            except (TimeoutError,ConnectionError): self.close_connection=True
            except Exception:
                logging.exception('HTTP request failed')
                self.reply(500,dict(error=dict(code='internal_error'),sent=None,bytes_written=None))

        do_GET=run_request
        do_POST=run_request

    return Handler


def main():
    bridge=Bridge()
    server=ThreadingHTTPServer(('127.0.0.1',8765),handler_for(bridge))
    print('Dummy bridge V4 transport: http://127.0.0.1:8765 (no startup hardware commands)',flush=True)
    try: server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt: pass
    finally:
        server.server_close()
        bridge.close()


if __name__=='__main__':
    mp.freeze_support()
    main()
