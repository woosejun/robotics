import cv2
import glob
import os
import subprocess
import sys
import time

import config
import shared_state as state

# =========================================================
# Camera Open
# =========================================================
def is_capture_device(device):
    if not os.path.exists(device):
        return False

    try:
        result = subprocess.run(
            [
                "v4l2-ctl",
                "--device",
                device,
                "--all",
            ],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return True

    output = result.stdout + result.stderr

    if result.returncode != 0:
        return True

    if "Device Caps" in output:
        device_caps = output.split("Device Caps", 1)[1]
        return "Video Capture" in device_caps

    return "Video Capture" in output


def get_camera_devices():
    devices = sorted(
        glob.glob("/dev/video*")
    )

    capture_devices = [
        device
        for device in devices
        if is_capture_device(device)
    ]

    return capture_devices or devices or ["/dev/video0"]


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

    devices = get_camera_devices()

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


def print_camera_help():
    print("[HINT] /dev/video0 busy이면 카메라를 쓰는 프로그램을 먼저 닫으세요.")
    print("[HINT] 확인: sudo fuser -v /dev/video0")
    print("[HINT] 종료: sudo fuser -k /dev/video0")

print("[INFO] 카메라 오픈 중...")

cap = open_camera()

if cap is None:
    print("[CRITICAL] 카메라 실패")
    print_camera_help()

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
