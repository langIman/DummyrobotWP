"""Windows UVC acquisition using modes verified on the DECXIN camera."""
from collections import deque
import time


def fourcc_text(value):
    value = int(value) & 0xffffffff
    return ''.join(chr((value >> (8 * index)) & 0xff) for index in range(4))


def serve(pipe, requested_mode):
    cap = None
    latest = None
    captured_at_ms = None
    frame_time = 0.
    frame_id = 0
    dimensions = None
    capture_times = deque(maxlen=240)
    error = None
    try:
        import cv2
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            raise RuntimeError('camera_not_opened')
        # This driver only keeps MJPG when FOURCC is applied after size and FPS.
        applied = dict(
            width=bool(cap.set(cv2.CAP_PROP_FRAME_WIDTH, requested_mode['width'])),
            height=bool(cap.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_mode['height'])),
            fps=bool(cap.set(cv2.CAP_PROP_FPS, requested_mode['fps'])),
            fourcc=bool(cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))),
        )
        driver_mode = dict(
            width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fps=round(float(cap.get(cv2.CAP_PROP_FPS)), 3),
            fourcc=fourcc_text(cap.get(cv2.CAP_PROP_FOURCC)),
        )
        while True:
            if pipe.poll(.01):
                message = pipe.recv()
                if message == 'close':
                    break
                fresh = latest is not None and time.monotonic() - frame_time <= 2
                measured_fps = ((len(capture_times) - 1) / (capture_times[-1] - capture_times[0])
                                if len(capture_times) > 1 else None)
                result = dict(
                    camera_status='responsive' if fresh else 'unavailable',
                    reason=error if error else (None if fresh else 'waiting_for_frame'),
                    frame_available=fresh,
                    dimensions=dimensions,
                    frame_id=frame_id,
                    captured_at_ms=captured_at_ms,
                    frame_age_ms=round((time.monotonic()-frame_time)*1000) if latest else None,
                    mode='explicit_mjpg',
                    index=0,
                    requested_mode=dict(width=requested_mode['width'], height=requested_mode['height'],
                                        fps=requested_mode['fps'], fourcc='MJPG',
                                        jpeg_quality=requested_mode['jpeg_quality']),
                    driver_mode=driver_mode,
                    actual_mode=dict(width=dimensions[0], height=dimensions[1],
                                     measured_fps=round(measured_fps, 2) if measured_fps else None)
                                if dimensions else None,
                    property_order=['width', 'height', 'fps', 'fourcc'],
                    property_set_return=applied,
                )
                if message == 'frame':
                    result['jpeg'] = latest if fresh else None
                pipe.send(result)
            ok, frame = cap.read()
            if not ok:
                latest = None
                error = 'camera_frame_read_failed'
                time.sleep(.05)
                continue
            received = time.time_ns() // 1_000_000
            captured = time.monotonic()
            capture_times.append(captured)
            while len(capture_times) > 2 and captured - capture_times[0] > 4:
                capture_times.popleft()
            ok, jpeg = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, requested_mode['jpeg_quality']]
            )
            if not ok:
                latest = None
                error = 'camera_jpeg_encode_failed'
                continue
            latest = jpeg.tobytes()
            captured_at_ms = received
            frame_time = captured
            frame_id += 1
            dimensions = [frame.shape[1], frame.shape[0]]
            error = None
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        # Keep the request-response pairing even when camera initialization fails.
        try:
            while True:
                message = pipe.recv()
                if message == 'close': break
                pipe.send(dict(camera_status='unavailable', reason=type(exc).__name__+': '+str(exc),
                               frame_available=False, jpeg=None) if message=='frame' else
                          dict(camera_status='unavailable', reason=type(exc).__name__+': '+str(exc),frame_available=False))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        if cap is not None: cap.release()
        pipe.close()
