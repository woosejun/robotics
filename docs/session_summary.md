# Robotics 프로젝트 작업 정리

## 1. 문제 상황
- `robotics/ai_vision/web_tracker.py` 실행 시 카메라 접근 실패로 서버가 시작되지 않음.
- 실행 중 Flask 서버는 정상 작동하지만, 로봇 USB 직렬 포트가 존재하지 않아 연결 실패 로그가 반복됨.

## 2. 조사한 파일
- `robotics/ai_vision/web_tracker.py`
  - Flask 웹 서버
  - 영상 스트리밍 `/video_feed`
  - 로봇 상태 API `/get_data`
  - 카메라/객체 인식/제어 스레드 실행
- `robotics/ai_vision/camera.py`
  - 카메라 장치 열기
  - GStreamer 파이프라인과 OpenCV 기본 캡쳐 시도
- `robotics/ai_vision/config.py`
  - 카메라, YOLO, 직렬 포트 설정
  - USB 직렬 포트 경로가 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`로 지정됨
- `robotics/motor_control/esp32_usb_controller/esp32_usb_controller.ino`
  - ESP32 Arduino 펌웨어
  - USB 직렬로 문자 명령을 수신하고 74HC595 시프트 레지스터로 모터 드라이버 신호를 출력

## 3. 문제 원인 및 확인 결과
- 카메라 문제
  - `/dev/video0` 장치가 다른 프로세스에 의해 점유된 상태였음.
  - 점유 프로세스를 종료 후 `web_tracker.py`가 정상 시작됨.
- USB 직렬 문제
  - 지정된 경로에 장치 파일이 없음.
  - 로봇 USB 장치가 연결되어 있지 않거나 포트 이름이 다름.
  - 따라서 `Robot USB serial 연결 실패` 경고가 계속 출력됨.

## 4. ESP32 모터 명령 의미
- `F` : 앞으로 이동
- `B` : 뒤로 이동
- `L` : 왼쪽 이동
- `R` : 오른쪽 이동
- `S` : 정지
- `C` : 시계 방향 회전
- `K` : 반시계 방향 회전

## 5. 현재 상태
- `web_tracker.py`는 실행 가능하며 웹 요청에 응답함.
- 카메라 스트림과 `/get_data` API 호출이 동작함.
- 로봇 USB 직렬 연결은 아직 해결되지 않음.

## 6. 다음 작업 제안
- 실제 USB 직렬 장치를 연결하거나, 올바른 `/dev/serial/...` 경로로 설정 수정
- ESP32에 Arduino 스케치를 올리고, 포트가 열리는지 확인
- 필요 시 `web_tracker.py`의 직렬 연결 재시도 로직 수정
