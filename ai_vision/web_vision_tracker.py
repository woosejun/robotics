from flask import Flask, Response

import threading
import time
import cv2

import shared_state as state
import config

from camera import camera_thread
from inference import inference_thread
from controller import control_thread
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
    <html>

    <body style="
        text-align:center;
        background:black;
        color:white;
    ">

        <h1>
            UWB Vision Follower
        </h1>

        <img
            src="/video_feed"
            width="960"
        >

        <div id="data"></div>

        <script>

        setInterval(() => {

            fetch("/get_data")

            .then(r => r.text())

            .then(t => {

                document
                .getElementById("data")
                .innerHTML =

                "<h2>"
                +
                "Control Error X: "
                +
                t
                +
                "</h2>"

            })

        }, 100)

        </script>

        <p>

            <a
                href="/save"

                style="
                    font-size:30px;
                    color:lime;
                "
            >

                주인 등록

            </a>

        </p>

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
# Get Control Error
# =========================================================
@app.route('/get_data')
def get_data():

    with state.control_lock:

        return str(
            state.control_error_x
        )

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
        "[INFO] Flask 서버 시작"
    )

    app.run(
        host='0.0.0.0',
        port=5000,
        threaded=True,
        debug=False,
        use_reloader=False
    )