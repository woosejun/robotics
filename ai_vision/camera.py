import cv2
import glob
import sys
import time

import config
import shared_state as state

# =========================================================
# Camera Open
# =========================================================
def make_gst_pipeline(device):

    return (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={config.CAMERA_WIDTH},"
        f"height={config.CAMERA_HEIGHT},"
        f"framerate={config.CAMERA_FPS}/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true sync=false"
    )


def configure_capture(capture):

    capture.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG')
    )

    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        config.CAMERA_WIDTH
    )

    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        config.CAMERA_HEIGHT
    )

    capture.set(
        cv2.CAP_PROP_FPS,
        config.CAMERA_FPS
    )


def open_camera():

    devices = sorted(
        glob.glob("/dev/video*")
    )

    if not devices:

        devices = [
            "/dev/video0"
        ]

    print(
        "[INFO] 카메라 후보:",
        ", ".join(devices)
    )

    for device in devices:

        print(
            f"[INFO] GStreamer 카메라 오픈 시도: {device}"
        )

        capture = cv2.VideoCapture(
            make_gst_pipeline(device),
            cv2.CAP_GSTREAMER
        )

        if capture.isOpened():

            return capture

        capture.release()

    print("[WARNING] OpenCV fallback")

    for device in devices:

        print(
            f"[INFO] OpenCV 카메라 오픈 시도: {device}"
        )

        capture = cv2.VideoCapture(device)

        configure_capture(capture)

        time.sleep(0.5)

        if capture.isOpened():

            return capture

        capture.release()

    return None

print("[INFO] 카메라 오픈 중...")

cap = open_camera()

if cap is None:
    print("[CRITICAL] 카메라 실패")

    sys.exit(1)

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
