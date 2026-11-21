"""Capture UVC frames, keeping their acquisition times across reconnects."""

import glob
import time

import cv2

import config
import shared_state as state


def open_camera():
    devices = [config.CAMERA_DEVICE] if config.CAMERA_DEVICE else sorted(glob.glob("/dev/video*"))
    for device in devices:
        pipeline = (
            f'v4l2src device="{device}" ! '
            f"image/jpeg,width={config.CAMERA_WIDTH},height={config.CAMERA_HEIGHT},"
            f"framerate={config.CAMERA_FPS}/1 ! jpegdec ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false"
        )
        capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if capture.isOpened():
            return capture
        capture.release()

    for device in devices:
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            capture.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
            return capture
        capture.release()
    return None


def camera_thread():
    while state.running:
        capture = None
        try:
            capture = open_camera()
            if capture is None:
                print("[WARNING] 카메라를 열 수 없습니다. 연결과 점유 상태를 확인하세요.")
            else:
                while state.running:
                    success, frame = capture.read()
                    if not success:
                        break
                    captured_at = time.monotonic()
                    if frame.shape[:2] != (config.CAMERA_HEIGHT, config.CAMERA_WIDTH):
                        frame = cv2.resize(frame, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))
                    with state.frame_lock:
                        state.current_frame = frame
                        state.frame_at = captured_at
        except cv2.error as exc:
            print(f"[WARNING] 카메라 오류: {exc}")
        finally:
            if capture is not None:
                capture.release()
            with state.frame_lock:
                state.current_frame = None
                state.frame_at = float("-inf")
        time.sleep(config.CAMERA_RETRY_DT)
