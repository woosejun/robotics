import cv2
import time

import config
import shared_state as state

# =========================================================
# GStreamer
# =========================================================
gst_pipeline = (
    "v4l2src device=/dev/video0 ! "
    f"image/jpeg,width={config.CAMERA_WIDTH},"
    f"height={config.CAMERA_HEIGHT},"
    f"framerate={config.CAMERA_FPS}/1 ! "
    "jpegdec ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true sync=false"
)

print("[INFO] 카메라 오픈 중...")

cap = cv2.VideoCapture(
    gst_pipeline,
    cv2.CAP_GSTREAMER
)

# =========================================================
# Fallback
# =========================================================
if not cap.isOpened():

    print("[WARNING] OpenCV fallback")

    cap = cv2.VideoCapture(0)

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG')
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        config.CAMERA_WIDTH
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        config.CAMERA_HEIGHT
    )

    cap.set(
        cv2.CAP_PROP_FPS,
        config.CAMERA_FPS
    )

    time.sleep(2)

if not cap.isOpened():

    print("[CRITICAL] 카메라 실패")

    exit()

print("[SUCCESS] 카메라 연결 완료")

# =========================================================
# Camera Thread
# =========================================================
def camera_thread():

    while state.running:

        success, frame = cap.read()

        if success:

            with state.frame_lock:

                state.current_frame = (
                    frame.copy()
                )