import cv2
import numpy as np
from ultralytics import YOLO
from flask import Flask, Response
import threading
import time

# =========================================================
# Flask
# =========================================================
app = Flask(__name__)

# =========================================================
# TensorRT 엔진 로드
# =========================================================
print("[INFO] TensorRT 엔진 로드 중...")

model = YOLO(
    "yolov8n.engine",
    task="detect"
)

print("[SUCCESS] TensorRT 엔진 로드 완료")

# =========================================================
# Camera Config
# =========================================================
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

CENTER_X = CAMERA_WIDTH // 2

# =========================================================
# Control Config
# =========================================================
CONTROL_DT = 0.03

MAX_STEP = 15

ALPHA = 0.15

START_TURN = 40
STOP_TURN = 20

SEARCH_ERROR = 120

# =========================================================
# USB Webcam (Logitech C920)
# =========================================================
gst_pipeline = (
    "v4l2src device=/dev/video0 ! "
    f"image/jpeg,width={CAMERA_WIDTH},"
    f"height={CAMERA_HEIGHT},"
    f"framerate={CAMERA_FPS}/1 ! "
    "jpegdec ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true sync=false"
)

print("[INFO] GStreamer 카메라 오픈 시도 중...")

cap = cv2.VideoCapture(
    gst_pipeline,
    cv2.CAP_GSTREAMER
)

# =========================================================
# GStreamer 실패 시 백업 모드
# =========================================================
if not cap.isOpened():

    print(
        "[WARNING] GStreamer 실패 "
        "-> OpenCV MJPEG 모드"
    )

    cap = cv2.VideoCapture(0)

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG')
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        CAMERA_WIDTH
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        CAMERA_HEIGHT
    )

    cap.set(
        cv2.CAP_PROP_FPS,
        CAMERA_FPS
    )

    time.sleep(2)

# =========================================================
# 카메라 최종 확인
# =========================================================
if not cap.isOpened():

    print(
        "[CRITICAL] 카메라를 열 수 없습니다!"
    )

    exit()

print("[SUCCESS] 카메라 연결 완료")

# =========================================================
# 전역 변수
# =========================================================
current_frame = None

frame_lock = threading.Lock()

# =========================================================
# Detection Data
# =========================================================
data_lock = threading.Lock()

detections = []

sim_scores = []

target_idx = -1

# =========================================================
# Target Histogram
# =========================================================
target_hist = None

target_hist_lock = threading.Lock()

# =========================================================
# Control State
# =========================================================
control_lock = threading.Lock()

control_error_x = 0

robot_state = "IDLE"

last_direction = 0

turning = False

# =========================================================
# Internal State
# =========================================================
virtual_error_x = 0

prev_control_error = 0

lost_start_time = time.time()

# =========================================================
# Save Request
# =========================================================
save_requested = False

save_lock = threading.Lock()

# =========================================================
# Running
# =========================================================
running = True

# =========================================================
# Histogram
# =========================================================
def get_hist(frame, box):

    x1, y1, x2, y2 = map(
        int,
        box
    )

    h, w, _ = frame.shape

    x1 = max(0, x1)
    y1 = max(0, y1)

    x2 = min(w, x2)
    y2 = min(h, y2)

    # =====================================================
    # 몸통 ROI만 사용
    # =====================================================
    roi = frame[
        y1 + (y2 - y1)//4 : y2,
        x1 + (x2 - x1)//4 :
        x2 - (x2 - x1)//4
    ]

    if roi.size == 0:

        return None

    roi_small = cv2.resize(
        roi,
        (64, 128)
    )

    hsv = cv2.cvtColor(
        roi_small,
        cv2.COLOR_BGR2HSV
    )

    hist = cv2.calcHist(
        [hsv],
        [0, 1],
        None,
        [16, 16],
        [0, 180, 0, 256]
    )

    cv2.normalize(
        hist,
        hist,
        0,
        1,
        cv2.NORM_MINMAX
    )

    return hist

# =========================================================
# Camera Thread
# =========================================================
def camera_thread():

    global current_frame

    while running:

        success, frame = cap.read()

        if success:

            with frame_lock:

                current_frame = frame.copy()

# =========================================================
# TensorRT 추론 스레드
# =========================================================
def inference_thread():

    global detections
    global sim_scores
    global target_idx
    global target_hist
    global save_requested

    while running:

        with frame_lock:

            if current_frame is None:

                frame_copy = None

            else:

                frame_copy = (
                    current_frame.copy()
                )

        if frame_copy is None:

            time.sleep(0.01)

            continue

        try:

            # =================================================
            # YOLO 추론
            # =================================================
            results = model(
                frame_copy,
                classes=0,
                conf=0.45,
                verbose=False,
                imgsz=640
            )

            local_boxes = (
                results[0].boxes
            )

            local_scores = []

            local_target_idx = -1

            best_sim = -1

            # =================================================
            # 현재 타겟 히스토그램
            # =================================================
            with target_hist_lock:

                current_target_hist = (
                    target_hist
                )

            # =================================================
            # 사람 비교
            # =================================================
            for i, box in enumerate(
                local_boxes
            ):

                b = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                curr_hist = get_hist(
                    frame_copy,
                    b
                )

                if (
                    current_target_hist
                    is not None
                    and
                    curr_hist
                    is not None
                ):

                    sim = cv2.compareHist(
                        current_target_hist,
                        curr_hist,
                        cv2.HISTCMP_CORREL
                    )

                else:

                    sim = 0.0

                local_scores.append(sim)

                if (
                    sim > 0.4
                    and
                    sim > best_sim
                ):

                    best_sim = sim

                    local_target_idx = i

            # =================================================
            # 히스토그램 업데이트
            # =================================================
            if (
                local_target_idx != -1
                and
                current_target_hist
                is not None
            ):

                best_box = (
                    local_boxes[
                        local_target_idx
                    ]
                    .xyxy[0]
                    .cpu()
                    .numpy()
                )

                new_hist = get_hist(
                    frame_copy,
                    best_box
                )

                if (
                    new_hist is not None
                    and
                    local_scores[
                        local_target_idx
                    ] > 0.6
                ):

                    updated_hist = (
                        cv2.addWeighted(
                            current_target_hist,
                            0.9,
                            new_hist,
                            0.1,
                            0
                        )
                    )

                    with target_hist_lock:

                        target_hist = (
                            updated_hist
                        )

            # =================================================
            # SAVE 요청 처리
            # =================================================
            with save_lock:

                do_save = save_requested

                if do_save:

                    save_requested = False

            if (
                do_save
                and
                len(local_boxes) > 0
            ):

                boxes_np = (
                    local_boxes
                    .xyxy
                    .cpu()
                    .numpy()
                )

                centers = [

                    (b[0] + b[2]) / 2

                    for b in boxes_np
                ]

                dist = [

                    abs(c - CENTER_X)

                    for c in centers
                ]

                best_idx = dist.index(
                    min(dist)
                )

                new_hist = get_hist(
                    frame_copy,
                    boxes_np[best_idx]
                )

                if new_hist is not None:

                    with target_hist_lock:

                        target_hist = new_hist

                    print(
                        "[INFO] "
                        "타겟 등록 완료"
                    )

            # =================================================
            # Detection Update
            # =================================================
            with data_lock:

                detections = list(
                    local_boxes
                )

                sim_scores = list(
                    local_scores
                )

                target_idx = (
                    local_target_idx
                )

        except Exception as e:

            print(
                "[TensorRT ERROR]",
                e
            )

        time.sleep(0.02)

# =========================================================
# Control Thread
# =========================================================
def control_thread():

    global control_error_x
    global robot_state
    global last_direction
    global virtual_error_x
    global prev_control_error
    global lost_start_time
    global turning

    while running:

        # =====================================================
        # Detection Copy
        # =====================================================
        with data_lock:

            local_target_idx = (
                target_idx
            )

            local_detections = (
                list(detections)
            )

        # =====================================================
        # TRACK
        # =====================================================
        if (
            local_target_idx != -1
            and
            local_target_idx <
            len(local_detections)
        ):

            box = local_detections[
                local_target_idx
            ]

            b = (
                box.xyxy[0]
                .cpu()
                .numpy()
                .astype(int)
            )

            x1, y1, x2, y2 = b

            center_x = (
                x1 + x2
            ) // 2

            # =============================================
            # Raw Error
            # =============================================
            raw_error_x = (
                center_x -
                CENTER_X
            )

            # =============================================
            # Virtual Target
            # =============================================
            virtual_error_x += (

                ALPHA * (

                    raw_error_x -
                    virtual_error_x
                )
            )

            target_error = int(
                virtual_error_x
            )

            # =============================================
            # Rate Limiter
            # =============================================
            delta = (
                target_error -
                prev_control_error
            )

            delta = max(
                -MAX_STEP,
                min(MAX_STEP, delta)
            )

            new_error = (
                prev_control_error
                + delta
            )

            prev_control_error = (
                new_error
            )

            # =============================================
            # Hysteresis Turning Logic
            # =============================================
            if turning == False:

                if abs(new_error) > START_TURN:

                    turning = True

            else:

                if abs(new_error) < STOP_TURN:

                    turning = False

            if turning == False:

                new_error = 0

            # =============================================
            # Direction Memory
            # =============================================
            if new_error > 0:

                last_direction = 1

            elif new_error < 0:

                last_direction = -1

            # =============================================
            # State Update
            # =============================================
            lost_start_time = (
                time.time()
            )

            new_state = "TRACK"

        # =====================================================
        # LOST FSM
        # =====================================================
        else:

            lost_duration = (
                time.time()
                -
                lost_start_time
            )

            # =============================================
            # SEARCH
            # =============================================
            if lost_duration < 1.0:

                new_state = "SEARCH"

                new_error = 0

            # =============================================
            # LOST RIGHT
            # =============================================
            elif lost_duration < 3.0:

                new_state = (
                    "LOST_RIGHT"
                )

                new_error = (
                    last_direction
                    * SEARCH_ERROR
                )

            # =============================================
            # LOST LEFT
            # =============================================
            elif lost_duration < 5.0:

                new_state = (
                    "LOST_LEFT"
                )

                new_error = (
                    -last_direction
                    * SEARCH_ERROR
                )

            # =============================================
            # WAIT UWB
            # =============================================
            else:

                new_state = (
                    "WAIT_UWB"
                )

                new_error = 0

            # =============================================
            # Internal Sync
            # =============================================
            virtual_error_x = (
                new_error
            )

            prev_control_error = (
                new_error
            )

            turning = False

        # =====================================================
        # Final Update
        # =====================================================
        with control_lock:

            control_error_x = (
                new_error
            )

            robot_state = (
                new_state
            )

        time.sleep(CONTROL_DT)

# =========================================================
# Flask 스트리밍
# =========================================================
def generate_frames():

    while True:

        # =====================================================
        # Frame Copy
        # =====================================================
        with frame_lock:

            if current_frame is None:

                time.sleep(0.01)

                continue

            frame = current_frame.copy()

        # =====================================================
        # Detection Copy
        # =====================================================
        with data_lock:

            local_detections = (
                list(detections)
            )

            local_scores = (
                list(sim_scores)
            )

            local_target_idx = (
                target_idx
            )

        # =====================================================
        # Control Copy
        # =====================================================
        with control_lock:

            local_error_x = (
                control_error_x
            )

            local_state = (
                robot_state
            )

            local_direction = (
                last_direction
            )

        # =====================================================
        # Center Line
        # =====================================================
        cv2.line(
            frame,
            (CENTER_X, 0),
            (CENTER_X, CAMERA_HEIGHT),
            (0, 255, 255),
            2
        )

        # =====================================================
        # Detection Draw
        # =====================================================
        for i, box in enumerate(
            local_detections
        ):

            if i >= len(local_scores):

                break

            b = (
                box.xyxy[0]
                .cpu()
                .numpy()
                .astype(int)
            )

            x1, y1, x2, y2 = b

            center_x = (
                x1 + x2
            ) // 2

            is_target = (
                i == local_target_idx
            )

            color = (
                (0, 255, 0)
                if is_target
                else (255, 0, 0)
            )

            score_str = (
                f"{local_scores[i]:.2f}"
            )

            # =============================================
            # MASTER
            # =============================================
            if is_target:

                label = (
                    f"MASTER "
                    f"{score_str} "
                    f"| ERR: "
                    f"{local_error_x}"
                )

                cv2.line(
                    frame,
                    (CENTER_X, 240),
                    (center_x, 240),
                    (0, 255, 0),
                    2
                )

            # =============================================
            # PERSON
            # =============================================
            else:

                label = (
                    f"PERSON "
                    f"{score_str}"
                )

            # =============================================
            # Draw Box
            # =============================================
            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                color,
                2
            )

            cv2.putText(
                frame,
                label,
                (
                    x1,
                    max(y1 - 5, 12)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )

        # =====================================================
        # 상태 표시
        # =====================================================
        cv2.putText(
            frame,
            f"STATE: {local_state}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"ERR_X: {local_error_x}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"DIR: {local_direction}",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        # =====================================================
        # WAIT UWB 표시
        # =====================================================
        if local_state == "WAIT_UWB":

            cv2.putText(
                frame,
                "UWB RECOVERY",
                (180, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3
            )

        # =====================================================
        # JPEG Encode
        # =====================================================
        ret, buffer = cv2.imencode(
            '.jpg',
            frame,
            [
                int(
                    cv2.IMWRITE_JPEG_QUALITY
                ),
                45
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

        time.sleep(0.03)

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
            Vision Follower
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
# Control Error
# =========================================================
@app.route('/get_data')
def get_data():

    with control_lock:

        return str(
            control_error_x
        )

# =========================================================
# Save Target
# =========================================================
@app.route('/save')
def save_target():

    global save_requested

    with save_lock:

        save_requested = True

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
