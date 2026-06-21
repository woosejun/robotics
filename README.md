# 🤖 Vision 기반 자율주행 사용자 추종 플랫폼

본 프로젝트는 **NVIDIA Jetson Nano**의 영상 인식 기능을 활용해 사용자를 추종하는 스마트 카트입니다.

---

## 🛠 시스템 구성 (System Architecture)

### 1. 연산 및 제어부 (Processing & Control)
*   **Main Host (NVIDIA Jetson Nano):** 로봇의 메인 프로세서로, CSI 카메라를 통해 입력되는 영상 데이터를 처리합니다. TensorRT 가속을 이용한 YOLO 모델을 구동하여 사용자의 바운딩 박스(Bounding Box)와 중앙점 좌표를 실시간으로 연산합니다.
*   **Sub Controller (ESP32):** 젯슨 나노로부터 UART 통신을 통해 이동 명령(속도, 방향 벡터)을 수신합니다. 수신된 데이터를 바탕으로 메카넘 휠 구동을 위한 PWM 신호를 생성하고, 4개의 모터 드라이버를 독립적으로 제어합니다.

### 2. 센서 및 통신부 (Sensing & Communication)
*   **Vision System (Logitech USB Webcam) :**광각 Logitech USB Webcam(C920 계열)을 사용하여 사용자의 위치를 실시간으로 인식합니다. MJPEG 기반 스트리밍을 활용하여 Jetson Nano 환경에서도 안정적인 프레임 처리 성능을 확보합니다.
*   **Internal Communication:** 젯슨 나노와 각 제어 보드 간의 데이터 전송 효율을 극대화하기 위해 USB-to-UART 통신을 사용하며, 데이터 유실 방지를 위한 체크섬(Checksum) 포함 패킷 프로토콜을 적용합니다.

### 3. 구동 및 전원부 (Actuator & Power)
*   **Mecanum Wheel Drive:** 4개의 독립 구동 메카넘 휠을 통해 전후좌우 평행 이동 및 제자리 회전이 가능합니다. 이는 좁은 실내 공간이나 복도에서도 원활한 방향 전환을 가능하게 합니다.
*   **Dual-Source Power System:** 고성능 연산을 수행하는 젯슨 나노와 순간적인 고전류를 소모하는 모터 구동부의 전원 간섭을 최소화하기 위해 독립 전원 계통을 설계합니다. 고출력 보조배터리를 통해 DC 5V/3A 이상의 안정적인 전력을 공급합니다.

---

## 🚀 핵심 구현 기술 (Key Technologies)

### 1. AI Vision 기반 사용자 인식 및 추적
*   **YOLO (You Only Look Once) 최적화:** 실시간성 확보를 위해 가벼운 모델을 젯슨 나노 환경에 맞게 최적화하여 15~20 FPS 이상의 추론 속도를 구현합니다.
*   **ROI Tracking:** 인식된 사용자의 영역을 관심 영역(ROI)으로 설정하여 연산 부하를 줄이고 추적 안정성을 높입니다.

### 2. 기구부 최적화 설계
*   **복층형 레이아웃:** 제한된 크기의 섀시 공간을 효율적으로 활용하기 위해 PCB 서포트를 이용한 층별 배치를 채택합니다. 하단부에는 배터리와 구동부를, 상단부에는 제어기 및 센서류를 배치하여 전파 간섭을 최소화하고 유지보수 편의성을 확보합니다.

---

## 📁 폴더 구조 (Project Structure)
*   `/ai_vision`: 카메라 입력, YOLO 추론, 추적 제어, Flask 모니터링 코드
*   `/ai_vision/models`: YOLO/TensorRT 모델 파일 (`.pt`, `.onnx`, `.engine`)
*   `/motor_control`: ESP32 기반 메카넘 휠 주행 제어 펌웨어
*   `/docs`: 시스템 설명, 작업 기록, 레거시 실험 코드
