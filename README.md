# 사용자 위치 인식 기반 자율주행 카트

제9회 AI ROBOTICS 융합 아이디어 경진대회에서 제작한 사용자 추종 카트입니다. Jetson Orin Nano가 영상 또는 UWB 거리로 이동 명령을 만들고, ESP32가 AceBot 모터 실드를 통해 네 바퀴를 구동합니다.

- **영상 추종**은 YOLOv8n 검출, 등록 대상의 HSV 색상 특징 비교, 박스 높이 기반 거리 판단을 사용합니다.
- **주행 제어**는 대상 등록, 운영자의 주행 활성화, ESP32의 정지 명령 ACK를 확인한 뒤 동작합니다. 영상·제어 명령 만료와 통신 장애 시 정지합니다.
- **UWB 추종**은 좌우 DWM3000 앵커의 거리로 태그 위치를 추정합니다. 선택한 UWB 모드에서 위치에 따라 ESP32에 전진·회전·정지 명령을 보냅니다.

## Jetson에서 실행

1. JetPack에 맞는 Python, PyTorch 및 TensorRT를 준비하고 OpenCV, Ultralytics와 Flask를 설치합니다. 대회 당시 JetPack과 패키지의 정확한 버전은 발표자료에 기록되어 있지 않습니다. [Jetson 설치 가이드](https://docs.ultralytics.com/guides/nvidia-jetson/), [TensorRT 엔진 호환성](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/support-matrix.html#engine-portability)
2. 사용할 Jetson에서 TensorRT 엔진을 생성합니다. 기본 위치는 `ai_vision/models/yolov8n.engine`입니다. [모델 준비](ai_vision/models/README.md)
3. 아래 명령으로 실행하고 Jetson에서 `http://127.0.0.1:5000`을 엽니다. 다른 컴퓨터에서는 SSH 포트 포워딩을 통해 같은 주소를 사용합니다.

```bash
python3 ai_vision/web_tracker.py
```

모터 송신은 기본값으로 비활성화됩니다. 바퀴를 띄운 상태에서 장치 경로와 정지 ACK를 확인한 뒤 모터 출력을 활성화합니다. 웹의 **대상 등록**을 누르고 등록 완료 상태를 확인한 후 **주행 활성화**를 누릅니다.

```bash
export ROBOTICS_MOTOR_PORT=/dev/serial/by-id/REPLACE_WITH_MOTOR_ID
python3 ai_vision/robot_comm.py
ROBOTICS_MOTOR_ENABLED=1 python3 ai_vision/web_tracker.py
```

`ROBOTICS_CAMERA_DEVICE`와 `ROBOTICS_MODEL`로 카메라 및 모델 경로를 지정할 수 있습니다. 카메라를 교체하면 시야각과 장착 높이에 맞춰 거리 임계값을 다시 측정합니다. [설정값](ai_vision/config.py), [모터 매핑과 장비 이식](docs/motor_setup.md)

## UWB와 가상 테스트

`ROBOTICS_FOLLOW_MODE=uwb`로 실행하면 UWB가 전진·제자리 회전·정지를 결정합니다. 기본 UWB 모드는 카메라에서 등록한 사람이 전방에 보일 때 주행을 허용합니다. 모터를 연결한 Jetson에서는 실제 USB 포트를 설정하고 바퀴를 띄운 상태에서 아래 명령으로 확인합니다. [포트 설정과 벤치 시험](docs/uwb_drive.md)

```bash
ROBOTICS_FOLLOW_MODE=uwb ROBOTICS_MOTOR_ENABLED=1 python3 ai_vision/web_tracker.py
```

```bash
python3 ai_vision/uwb_test.py --raw --duration 10
python3 -m unittest discover -s tests -v
```

최종 Windows 검증에서 Python 테스트 22개와 C++ 모의 시험 2개가 통과했습니다. 가상 앵커를 실제 ESP32 스케치의 모의 실행과 연결해 전진·좌우 회전·정지 마스크와 앵커 단절 후 재활성화 조건을 확인했습니다. 별도의 차체 모델에서는 좌우 두 초기 위치 모두 15.40초에 태그와 약 0.82m 떨어져 정지했습니다. 실제 무선 수신과 정지거리는 장비에서 실측합니다. [시뮬레이션 조건과 통합 시험 재현](docs/uwb_drive.md#통합-시험-재현)

- [UWB 배선·보드 프로필·펌웨어 빌드](uwb/README.md)
- [Jetson 포트 설정과 증상별 진단](docs/uwb_setup.md)
- [소프트웨어 구조와 정지 조건](docs/software_architecture.md)
- [포트폴리오에 쓸 구현·검증 기록](docs/portfolio.md)
