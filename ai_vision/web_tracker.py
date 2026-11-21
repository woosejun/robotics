"""Operator dashboard for the vision follower."""

import signal
import threading
import time

from flask import Flask, Response, jsonify, render_template_string, request

import config
import shared_state as state
from controller import control_thread, request_registration, set_motion_enabled
from robot_comm import robot_comm_thread
from uwb_comm import Ranges, start_anchor_workers, validate_anchor_ports

app = Flask(__name__)
ranges = Ranges(config.UWB_ANCHOR_BASELINE_M)


def uses_camera():
    return config.FOLLOW_MODE == "vision" or config.UWB_FRONT_GUARD


def generate_frames():
    import cv2
    from visualization import draw_frame

    seen = float("-inf")
    while state.running:
        with state.frame_lock:
            frame, frame_at = state.current_frame, state.frame_at
        if frame is None or frame_at == seen:
            time.sleep(0.01)
            continue
        seen = frame_at
        ok, jpeg = cv2.imencode(
            ".jpg", draw_frame(frame.copy()), [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY]
        )
        if ok:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"


@app.route("/")
def index():
    return render_template_string("""
    <!doctype html>
    <html lang="ko">
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>사용자 추종 카트</title>
        <style>
            body { margin:0 auto; padding:20px; max-width:960px; background:#0b0f14;
                   color:#fff; font-family:system-ui,sans-serif; }
            img { width:100%; max-height:65vh; object-fit:contain; background:#17202b; }
            .controls { display:flex; flex-wrap:wrap; gap:8px; margin:16px 0; }
            button { padding:10px 14px; font:inherit; border:0; border-radius:6px; cursor:pointer; }
            #stop { background:#ef4444; color:#111; }
            p { overflow-wrap:anywhere; }
        </style>
    </head>
    <body>
        <h1>사용자 추종 카트</h1>
        <p>{{ mode_label }}</p>
        {% if camera %}
        <img src="/video_feed" alt="카메라 영상">
        {% endif %}
        <div class="controls">
            {% if camera %}
            <button type="button" id="register">대상 등록</button>
            {% endif %}
            <button type="button" id="enable">주행 활성화</button>
            <button type="button" id="stop">정지</button>
        </div>
        <p id="notice" role="status"></p>
        <p id="tracking">카메라 대기 중</p>
        <p id="serial">모터 연결 대기 중</p>
        <p id="uwb">UWB 거리 대기 중</p>
        <script>
        const labels = {
            UNREGISTERED:"등록 전", PENDING:"등록 중", REGISTERED:"등록 완료",
            TIMED_OUT:"등록 시간 초과", DISCONNECTED:"연결 안 됨", WAITING:"거리 대기",
            RANGING:"거리 수신", STALE:"거리 수신 중단", ID_MISMATCH:"앵커 ID 불일치",
            UNSYNCED:"측정 시차 초과", INVALID_GEOMETRY:"유효한 좌표 없음",
            UWB_READY:"UWB 주행 준비", UWB_TRACK:"UWB 추종", UWB_CLOSE:"거리 도달 · 정지",
            UWB_SIGNAL_LOST:"UWB 거리 확인 필요", UWB_INPUT_STALE:"UWB 입력 만료",
            UWB_MEASURING:"정지 후 거리 측정 중",
            UWB_FRONT_LOST:"카메라 정면 확인 필요", UWB_OUTSIDE:"UWB 동작 범위 밖"
        };
        const field = (id, text) => document.getElementById(id).textContent = text;
        async function action(url, payload) {
            try {
                const response = await fetch(url, {
                    method:"POST", headers:{"Content-Type":"application/json"},
                    body:JSON.stringify(payload)
                });
                const data = await response.json();
                field("notice", data.message);
            } catch (error) {
                field("notice", "요청을 확인하지 못했습니다.");
            }
        }
        const register = document.getElementById("register");
        if (register) register.onclick = () => action("/save", {});
        document.getElementById("enable").onclick = () => action("/motion", {enabled:true});
        document.getElementById("stop").onclick = () => action("/motion", {enabled:false});
        function formatAnchor(label, anchor) {
            const distance = anchor.range_m === null ? "" : " " + anchor.range_m.toFixed(2) + " m";
            return label + " " + (labels[anchor.status] || anchor.status) + distance;
        }
        async function update() {
            try {
                const response = await fetch("/get_data", {cache:"no-store"});
                const data = await response.json();
                const status = labels[data.robot_state] || data.robot_state;
                field("tracking", data.follow_mode === "uwb"
                    ? status + " · " + (data.motion_enabled ? "주행 활성" : "정지 상태")
                        + " · UWB 판단 " + data.uwb_intent
                    : (labels[data.registration_status] || data.registration_status)
                        + " · " + status + " · 박스 " + data.target_height
                        + " px · 가로 오차 " + data.control_error_x + " px");
                field("serial", "모터 " + (data.serial_enabled
                    ? (data.serial_connected ? "ACK " + data.serial_last_ack : "연결 대기 중")
                    : "출력 비활성") + " · 제어 명령 " + data.motor_command);
                const position = data.follow_mode === "uwb" ? data.uwb_drive_position_m : data.uwb_position_m;
                field("uwb", formatAnchor("왼쪽", data.uwb_anchors.left)
                    + " · " + formatAnchor("오른쪽", data.uwb_anchors.right)
                    + " · " + (position ? "좌우 / 전방 " + position
                        : (labels[data.uwb_pair_status] || "주행용 좌표 대기")));
            } catch (error) {
                field("tracking", "상태 조회 실패");
            } finally {
                setTimeout(update, 200);
            }
        }
        update();
        </script>
    </body>
    </html>
    """, camera=uses_camera(), mode_label=(
        "영상 추종" if config.FOLLOW_MODE == "vision" else
        "UWB 추종 · 카메라 정면 확인" if config.UWB_FRONT_GUARD else "UWB 단독 시험 모드"
    ))


@app.route("/video_feed")
def video_feed():
    if not uses_camera():
        return Response(status=404)
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/get_data")
def get_data():
    with state.data_lock, state.control_lock, state.robot_serial_lock:
        data = {
            "registration_status": state.registration_status,
            "control_error_x": state.control_error_x,
            "distance_zone": state.distance_zone,
            "motor_command": state.motor_command,
            "robot_state": state.robot_state,
            "target_height": state.target_height,
            "motion_enabled": state.motion_enabled,
            "follow_mode": config.FOLLOW_MODE,
            "uwb_front_guard": config.UWB_FRONT_GUARD,
            "uwb_intent": state.uwb_intent,
            "serial_enabled": config.ROBOT_SERIAL_ENABLED,
            "serial_connected": state.robot_serial_connected,
            "serial_last_ack": state.robot_serial_last_ack,
        }

    uwb = ranges.snapshot()
    position = uwb["position_m"]
    drive_fix, _, _, _ = ranges.motion_sample(time.monotonic())
    left, right = uwb["anchors"]["left"], uwb["anchors"]["right"]
    connected = (left["status"] != "DISCONNECTED", right["status"] != "DISCONNECTED")
    connection_labels = {
        (False, False): "연결 안 됨", (True, False): "왼쪽만 연결",
        (False, True): "오른쪽만 연결", (True, True): "좌/우 연결",
    }
    data.update(
        uwb_status=connection_labels[connected],
        uwb_anchors=uwb["anchors"],
        uwb_pair_status=uwb["pair_status"],
        uwb_position_xy_m=position,
        uwb_position_m=f"{position[0]:+.2f} / {position[1]:.2f} m" if position else None,
        uwb_drive_position_m=(
            f"{drive_fix.x_m:+.2f} / {drive_fix.y_m:.2f} m" if drive_fix else None
        ),
        uwb_left_distance_m=left["range_m"],
        uwb_right_distance_m=right["range_m"],
    )
    return jsonify(data)


@app.route("/save", methods=["POST"])
def save_target():
    if not request.is_json:
        return jsonify(message="JSON 요청이 필요합니다."), 415
    if not uses_camera():
        return jsonify(message="이 모드에서는 UWB 태그를 사용합니다."), 409
    request_registration()
    return jsonify(message="등록 요청을 받았습니다. 대상은 카메라 중앙에 서 주세요."), 202


@app.route("/motion", methods=["POST"])
def motion():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict) or type(payload.get("enabled")) is not bool:
        return jsonify(message="enabled 값은 true 또는 false여야 합니다."), 400
    enabled = payload["enabled"]
    error = set_motion_enabled(enabled, ranges)
    if error:
        return jsonify(message=error), 409
    return jsonify(message="주행을 활성화했습니다." if enabled else "정지했습니다.")


def main():
    ports = {"left": config.UWB_LEFT_PORT, "right": config.UWB_RIGHT_PORT}
    if config.FOLLOW_MODE not in {"vision", "uwb"}:
        raise SystemExit("ROBOTICS_FOLLOW_MODE 값은 vision 또는 uwb여야 합니다.")
    if config.FOLLOW_MODE == "uwb" and (not config.UWB_ENABLED or not all(ports.values())):
        raise SystemExit("UWB 추종에는 활성화된 왼쪽·오른쪽 앵커 포트가 필요합니다.")
    if config.UWB_ENABLED:
        validate_anchor_ports(ports, config.ROBOT_SERIAL_PORT)

    def terminate(signum, frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, terminate)
    stop = threading.Event()
    workers = [
        threading.Thread(target=control_thread, args=(ranges,), name="control", daemon=True),
        threading.Thread(target=robot_comm_thread, name="motor-serial", daemon=True),
    ]
    if uses_camera():
        from camera import camera_thread
        from inference import inference_thread

        workers.extend(
            threading.Thread(target=worker, name=worker.__name__, daemon=True)
            for worker in (camera_thread, inference_thread)
        )
    for worker in workers:
        worker.start()
    if config.UWB_ENABLED:
        workers.extend(start_anchor_workers(ports, ranges, stop))
    try:
        app.run(host=config.WEB_HOST, port=config.WEB_PORT, threaded=True, use_reloader=False)
    finally:
        set_motion_enabled(False)
        state.running = False
        stop.set()
        for worker in workers:
            worker.join(timeout=0.5)


if __name__ == "__main__":
    main()
