from flask import Flask, Response, jsonify

import threading
import time
import cv2

import shared_state as state
import config

from camera import camera_thread
from inference import inference_thread
from controller import control_thread
from robot_comm import robot_comm_thread
from uwb_comm import uwb_comm_thread
from visualization import draw_frame

# =========================================================
# Flask
# =========================================================
app = Flask(__name__)

# =========================================================
# Generate Frames
# =========================================================
def generate_frames():

    while True:

        with state.frame_lock:

            if state.current_frame is None:

                time.sleep(0.01)

                continue

            frame = (
                state.current_frame.copy()
            )

        frame = draw_frame(frame)

        ret, buffer = cv2.imencode(
            '.jpg',
            frame,
            [
                int(
                    cv2.IMWRITE_JPEG_QUALITY
                ),
                config.JPEG_QUALITY
            ]
        )

        if not ret:

            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            +
            buffer.tobytes()
            +
            b'\r\n'
        )

# =========================================================
# Main Page
# =========================================================
@app.route('/')
def index():

    return '''
    <html lang="ko">
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>Vision Follower</title>
        <style>
            body { margin:0; padding:20px; text-align:center; background:#0b0f14;
                   color:white; font-family:sans-serif; }
            .camera { width:min(960px, 100%); border:2px solid #334155;
                      border-radius:12px; }
            .status-grid { display:grid; grid-template-columns:repeat(auto-fit,
                           minmax(140px, 1fr)); gap:12px; max-width:960px;
                           margin:16px auto; }
            .card { padding:14px; background:#17202b; border-radius:10px; }
            .label { color:#94a3b8; font-size:14px; }
            .value { margin-top:6px; font-size:25px; font-weight:bold; }
            #distance { color:#38bdf8; }
            #command { color:#facc15; }
            .save { color:#4ade80; font-size:24px; }
        </style>
    </head>
    <body>
        <h1>카메라 거리 및 주행 상태</h1>
        <img class="camera" src="/video_feed">
        <div class="status-grid">
            <div class="card"><div class="label">카메라 거리</div>
                <div class="value" id="distance">-</div></div>
            <div class="card"><div class="label">사람 박스 높이(px)</div>
                <div class="value" id="height">0</div></div>
            <div class="card"><div class="label">가로 오차(px)</div>
                <div class="value" id="error">0</div></div>
            <div class="card"><div class="label">ESP 전송 명령</div>
                <div class="value" id="command">S</div></div>
            <div class="card"><div class="label">추적 상태</div>
                <div class="value" id="state">IDLE</div></div>
            <div class="card"><div class="label">USB 통신</div>
                <div class="value" id="serial">연결 안 됨</div></div>
            <div class="card"><div class="label">UWB 앵커</div>
                <div class="value" id="uwb">연결 안 됨</div></div>
            <div class="card"><div class="label">UWB 태그 위치 (좌/전방)</div>
                <div class="value" id="uwb-position">-</div></div>
        </div>
        <p><a class="save" href="/save">주인 등록</a></p>
        <script>
        const distanceNames = {
            FAR: "멀리", TARGET: "적정 거리", NEAR: "가까이", LOST: "대상 없음"
        };
        async function updateStatus() {
            try {
                const response = await fetch("/get_data");
                const data = await response.json();
                document.getElementById("distance").textContent =
                    distanceNames[data.distance_zone] || data.distance_zone;
                document.getElementById("height").textContent = data.target_height;
                document.getElementById("error").textContent = data.control_error_x;
                document.getElementById("command").textContent = data.motor_command;
                document.getElementById("state").textContent = data.robot_state;
                document.getElementById("serial").textContent =
                    data.serial_connected ? "정상" : "연결 안 됨";
                document.getElementById("uwb").textContent = data.uwb_status;
                document.getElementById("uwb-position").textContent =
                    data.uwb_position_m === null ? "-" : data.uwb_position_m;
            } catch (error) {
                document.getElementById("serial").textContent = "상태 조회 실패";
            }
        }
        updateStatus();
        setInterval(updateStatus, 200);
        </script>
    </body>
    </html>
    '''

# =========================================================
# Video Feed
# =========================================================
@app.route('/video_feed')
def video_feed():

    return Response(
        generate_frames(),
        mimetype=
        'multipart/x-mixed-replace; boundary=frame'
    )

# =========================================================
# Get Camera Distance and Robot Status
# =========================================================
@app.route('/get_data')
def get_data():

    with state.control_lock:
        control_data = {
            "control_error_x": state.control_error_x,
            "distance_zone": state.distance_zone,
            "motor_command": state.motor_command,
            "robot_state": state.robot_state,
            "target_height": state.target_height,
        }

    with state.robot_serial_lock:
        control_data["serial_connected"] = (
            state.robot_serial_connected
        )
        control_data["serial_last_ack"] = (
            state.robot_serial_last_ack
        )

    with state.uwb_lock:
        left = state.uwb_left_connected
        right = state.uwb_right_connected
        if left and right:
            control_data["uwb_status"] = "좌/우 연결"
        elif left:
            control_data["uwb_status"] = "왼쪽만 연결"
        elif right:
            control_data["uwb_status"] = "오른쪽만 연결"
        else:
            control_data["uwb_status"] = "연결 안 됨"
        if state.uwb_position_x_m is None or state.uwb_position_y_m is None:
            control_data["uwb_position_m"] = None
        else:
            control_data["uwb_position_m"] = (
                f"{state.uwb_position_x_m:+.2f} / {state.uwb_position_y_m:.2f} m"
            )
        control_data["uwb_left_distance_m"] = state.uwb_left_distance_m
        control_data["uwb_right_distance_m"] = state.uwb_right_distance_m

    return jsonify(control_data)

# =========================================================
# Save Target
# =========================================================
@app.route('/save')
def save_target():

    with state.save_lock:

        state.save_requested = True

    return '''

    <h1>
        등록 요청 완료!
    </h1>

    <p>
        카메라 앞에 서 있으면
        자동 등록됩니다.
    </p>

    <a href="/">
        돌아가기
    </a>

    '''

# =========================================================
# Main
# =========================================================
if __name__ == "__main__":

    print(
        "[INFO] Camera Thread 시작"
    )

    cam_t = threading.Thread(
        target=camera_thread
    )

    cam_t.daemon = True

    cam_t.start()

    print(
        "[INFO] TensorRT 추론 스레드 시작"
    )

    infer_t = threading.Thread(
        target=inference_thread
    )

    infer_t.daemon = True

    infer_t.start()

    print(
        "[INFO] Control Thread 시작"
    )

    ctrl_t = threading.Thread(
        target=control_thread
    )

    ctrl_t.daemon = True

    ctrl_t.start()

    print(
        "[INFO] Robot USB serial 스레드 시작"
    )

    robot_t = threading.Thread(
        target=robot_comm_thread
    )

    robot_t.daemon = True

    robot_t.start()

    print(
        "[INFO] UWB 앵커 수신 스레드 시작"
    )

    uwb_t = threading.Thread(
        target=uwb_comm_thread
    )

    uwb_t.daemon = True

    uwb_t.start()

    print(
        "[INFO] Flask 서버 시작"
    )

    app.run(
        host='0.0.0.0',
        port=5000,
        threaded=True,
        debug=False,
        use_reloader=False
    )
