from __future__ import annotations

import logging
import mimetypes
import os
import threading
import time
from pathlib import Path
from typing import Any

from flask import Flask, Response, jsonify, request, send_from_directory
from werkzeug.exceptions import HTTPException

from perception_service.common import ServiceError
from perception_service.service import PerceptionService


ROOT = Path(__file__).resolve().parent
ALLOWED_HOSTS = {"127.0.0.1:8770", "localhost:8770"}
ALLOWED_ORIGINS = {"http://127.0.0.1:8770", "http://localhost:8770"}


def create_app(service: PerceptionService | None = None) -> Flask:
    mimetypes.add_type("application/javascript", ".js")
    mimetypes.add_type("text/css", ".css")
    application = Flask(__name__, static_folder=str(ROOT / "static"), static_url_path="/static")
    runtime = service or PerceptionService(
        ROOT,
        os.environ.get("DUMMY_BRIDGE_URL", "http://127.0.0.1:8765"),
        os.environ.get("DUMMY_MOVEIT_URL", "http://127.0.0.1:8801"),
    )
    application.config["PERCEPTION_SERVICE"] = runtime

    @application.before_request
    def local_only() -> Response | None:
        if application.testing:
            return None
        if request.host not in ALLOWED_HOSTS:
            return jsonify(error={"code": "loopback_host_required", "message": "服务仅允许本机访问。"}), 403
        origin = request.headers.get("Origin")
        if origin and origin not in ALLOWED_ORIGINS:
            return jsonify(error={"code": "foreign_origin_rejected", "message": "已拒绝外部网页请求。"}), 403
        return None

    @application.after_request
    def secure_response(response: Response) -> Response:
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; img-src 'self' blob:; style-src 'self'; "
            "script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'"
        )
        return response

    @application.errorhandler(ServiceError)
    def service_error(exc: ServiceError) -> tuple[Response, int]:
        return jsonify(exc.payload()), exc.status

    @application.errorhandler(Exception)
    def internal_error(exc: Exception) -> tuple[Response, int]:
        if isinstance(exc, HTTPException):
            return jsonify(error={"code": exc.name.lower().replace(" ", "_"), "message": exc.description}), exc.code
        logging.exception("Unhandled perception request error")
        return jsonify(error={"code": "internal_error", "message": "服务内部错误。"}), 500

    def json_body(optional: bool = False) -> Any:
        if optional and not request.data:
            return {}
        if request.mimetype != "application/json":
            raise ServiceError(415, "json_required", "请求必须使用 application/json。")
        value = request.get_json(silent=True)
        if value is None:
            raise ServiceError(400, "invalid_json", "请求 JSON 无效。")
        return value

    @application.get("/")
    def index() -> Response:
        return send_from_directory(application.static_folder, "index.html")

    @application.get("/collect")
    def collect() -> Response:
        return send_from_directory(application.static_folder, "collect.html")

    @application.get("/favicon.ico")
    def favicon() -> Response:
        return Response(status=204)

    @application.get("/api/health")
    def health() -> Response:
        return jsonify(runtime.health())

    @application.get("/api/status")
    def status() -> Response:
        return jsonify(runtime.status())

    @application.get("/api/cameras/discover")
    def discover() -> Response:
        return jsonify(runtime.discover())

    @application.get("/api/cameras/<camera>/config")
    def camera_config(camera: str) -> Response:
        return jsonify(runtime.camera_config(camera))

    @application.put("/api/cameras/<camera>/config")
    def update_camera_config(camera: str) -> Response:
        return jsonify(runtime.configure_camera(camera, json_body()))

    @application.post("/api/cameras/<camera>/<action>")
    def camera_action(camera: str, action: str) -> Response:
        if action not in {"start", "stop"}:
            raise ServiceError(404, "camera_action_not_found", "不存在该摄像头操作。")
        return jsonify(runtime.control_camera(camera, action == "start"))

    def select_store(stream: str):
        stores = {
            "wrist": runtime.wrist.preview,
            "oak-rgb": runtime.oak.rgb_preview,
            "oak-depth": runtime.oak.depth_preview,
        }
        if stream not in stores:
            raise ServiceError(404, "stream_not_found", "不存在该图像流。")
        return stores[stream]

    def frame_response(stream: str) -> Response:
        frame = select_store(stream).get(2_000)
        if frame is None:
            raise ServiceError(503, "frame_unavailable", "当前没有新鲜图像。")
        response = Response(frame.data, mimetype=frame.content_type)
        response.headers["X-Frame-Id"] = str(frame.frame_id)
        response.headers["X-Captured-At-Ms"] = str(frame.captured_at_ms)
        response.headers["X-Frame-Age-Ms"] = str(frame.age_ms())
        return response

    @application.get("/api/frames/<stream>")
    def latest_frame(stream: str) -> Response:
        return frame_response(stream)

    @application.get("/api/frames/oak-depth/raw.png")
    def raw_depth() -> Response:
        import cv2

        value = runtime.oak.raw_depth()
        if value is None:
            raise ServiceError(503, "frame_unavailable", "当前没有新鲜深度图。")
        depth, metadata = value
        ok, encoded = cv2.imencode(".png", depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise ServiceError(503, "depth_encode_failed", "深度 PNG 编码失败。")
        response = Response(encoded.tobytes(), mimetype="image/png")
        response.headers["X-Frame-Id"] = str(metadata["source_frame_id"])
        response.headers["X-Captured-At-Ms"] = str(metadata["captured_at_ms"])
        response.headers["X-Depth-Unit"] = "millimeter"
        response.headers["X-Depth-Dtype"] = "uint16"
        return response

    @application.get("/api/streams/<stream>.mjpeg")
    def stream(stream: str) -> Response:
        store = select_store(stream)

        def generate():
            seen = -1
            while True:
                started = time.monotonic()
                frame = store.wait_after(seen, timeout=1)
                if frame is not None and frame.frame_id != seen and frame.age_ms() <= 2_000:
                    seen = frame.frame_id
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame.data)}\r\n".encode("ascii")
                        + f"X-Frame-Id: {frame.frame_id}\r\n".encode("ascii")
                        + f"X-Captured-At-Ms: {frame.captured_at_ms}\r\n\r\n".encode("ascii")
                        + frame.data
                        + b"\r\n"
                    )
                elapsed = time.monotonic() - started
                if elapsed < 1 / 15:
                    time.sleep((1 / 15) - elapsed)

        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @application.post("/api/recordings/start")
    def start_recording() -> Response:
        return jsonify(runtime.start_recording(json_body(optional=True))), 201

    @application.post("/api/recordings/stop")
    def stop_recording() -> Response:
        json_body(optional=True)
        return jsonify(runtime.recorder.stop("user_stop", complete=True))

    @application.get("/api/recordings")
    def recordings() -> Response:
        return jsonify(recordings=runtime.recorder.list_sessions())

    @application.get("/api/recordings/current")
    def current_recording() -> Response:
        return jsonify(runtime.recorder.current())

    @application.post("/api/recordings/control-event")
    def recording_control_event() -> Response:
        return jsonify(runtime.record_control_event(json_body()))

    @application.get("/api/control/status")
    def control_status() -> Response:
        return jsonify(runtime.live_control.status())

    @application.post("/api/control/arm")
    def control_arm() -> Response:
        return jsonify(runtime.live_control.arm(json_body()))

    @application.post("/api/control/input")
    def control_input() -> Response:
        return jsonify(runtime.live_control.update(json_body()))

    @application.post("/api/control/disarm")
    def control_disarm() -> Response:
        body = json_body(optional=True)
        if not isinstance(body, dict) or set(body) - {"reason"}:
            raise ServiceError(422, "unknown_fields", "退出控制只接受 reason 字段。")
        reason = body.get("reason", "operator")
        if not isinstance(reason, str) or len(reason) > 40:
            raise ServiceError(422, "control_reason_invalid", "退出原因必须是不超过 40 字的字符串。")
        return jsonify(runtime.live_control.disarm(reason))

    @application.get("/api/robot/state")
    def robot_state() -> Response:
        state = runtime.robot.read_state(include_gripper=True)
        if state.get("positions") is None:
            result = runtime.robot.status()
            result["error"] = {"code": "joint_feedback_unavailable", "message": "六轴反馈不可用，夹爪读取结果已单独更新。"}
            return jsonify(result), 503
        return jsonify(runtime.robot.status())

    @application.post("/api/robot/actions/<action>")
    def robot_action(action: str) -> tuple[Response, int] | Response:
        body = json_body(optional=True)
        if not isinstance(body, dict):
            raise ServiceError(400, "json_object_required", "请求正文必须是 JSON 对象。")
        if action == "enable":
            if body:
                raise ServiceError(422, "unknown_fields", "使能操作不接受参数。")
            return jsonify(runtime.robot.enable())
        if action == "disable":
            if body:
                raise ServiceError(422, "unknown_fields", "失能操作不接受参数。")
            live = getattr(runtime, "live_control", None)
            if live and live.is_armed():
                live.disarm("operator_disable", disable_robot=False)
            return jsonify(runtime.robot.disable())
        if action in {"home", "rest", "ready"}:
            live = getattr(runtime, "live_control", None)
            if live and live.is_armed():
                raise ServiceError(409, "live_control_active", "请先退出数据采集实机控制。")
            if set(body) - {"confirmation"}:
                raise ServiceError(422, "unknown_fields", "预设运动只接受 confirmation 字段。")
            return jsonify(runtime.robot.start_preset(action, body.get("confirmation"))), 202
        if action.startswith("gripper-"):
            result = runtime.robot.gripper.execute(action.removeprefix("gripper-"), body)
            if not result["acknowledged"]:
                result["error"] = {"code": "gripper_command_unconfirmed",
                                   "message": "夹爪命令未确认，请查看原始回复；必要时再次点击夹爪停止。"}
            return jsonify(result), 200 if result["acknowledged"] else 503
        raise ServiceError(404, "unknown_robot_action", "不支持该机械臂动作。")

    @application.post("/api/robot/command")
    def raw_robot_command() -> Response:
        live = getattr(runtime, "live_control", None)
        if live and live.is_armed():
            raise ServiceError(409, "live_control_active", "请先退出数据采集实机控制。")
        result = runtime.robot.raw_command(json_body())
        return jsonify(result), 503 if result.get("error") else 200

    @application.post("/api/robot/emergency-stop")
    def emergency_stop() -> Response:
        body = json_body(optional=True)
        if body:
            raise ServiceError(422, "unknown_fields", "急停操作不接受参数。")
        live = getattr(runtime, "live_control", None)
        try:
            result = runtime.robot.emergency_stop()
        finally:
            if live:
                live.emergency_reset()
        return jsonify(result)

    @application.post("/api/robot/gripper/reference")
    def gripper_reference() -> Response:
        body = json_body(optional=True)
        if body:
            raise ServiceError(422, "unknown_fields", "建立夹爪参考不接受参数。")
        return jsonify(runtime.robot.capture_gripper_reference())

    @application.post("/api/shutdown")
    def shutdown() -> Response:
        body = json_body(optional=True)
        if body:
            raise ServiceError(422, "unknown_fields", "关闭操作不接受参数。")

        def close_after_response() -> None:
            time.sleep(0.2)
            runtime.close()
            os._exit(0)

        threading.Thread(target=close_after_response, name="service-shutdown", daemon=True).start()
        return jsonify(status="shutting_down")

    return application


def main() -> None:
    application = create_app()
    service: PerceptionService = application.config["PERCEPTION_SERVICE"]
    (ROOT / "runtime").mkdir(exist_ok=True)
    (ROOT / "runtime" / "perception.pid").write_text(str(os.getpid()), encoding="ascii")
    try:
        application.run(host="127.0.0.1", port=8770, threaded=True, debug=False, use_reloader=False)
    finally:
        service.close()


if __name__ == "__main__":
    main()
