# UWB 추종과 ESP32 모터 통합

`ANCHOR1,거리`와 `ANCHOR2,거리`를 Jetson이 읽고, 좌우·전방 좌표로 바꿉니다. 전방을 바라볼 때 태그가 카트 오른쪽이면 `C`, 왼쪽이면 `K`, 정면에서 멀면 `F`를 보냅니다. ESP32가 각각 `0xAC`, `0x53`, `0xA3`를 74HC595에 출력합니다. 정지 `S`의 마스크는 `0x00`입니다. [모터 매핑](motor_setup.md)

## Jetson에서 연결

1. [UWB 펌웨어 가이드](../uwb/README.md)에 따라 태그·왼쪽 앵커·오른쪽 앵커를 준비합니다. 두 앵커의 안테나 중심 사이 거리를 재고, 각 보드의 실제 USB 장치 경로를 찾습니다.
2. 아래 값의 예시 포트를 실제 장치로 바꿉니다. 장치가 여러 개면 `/dev/serial/by-id/` 또는 `/dev/serial/by-path/` 경로를 사용할 수 있습니다.
3. 카트를 들어 바퀴가 바닥에 닿지 않게 한 뒤, UWB 로그와 ESP32의 `OK S` 응답부터 확인합니다.

```bash
export ROBOTICS_UWB_LEFT=/dev/ttyACM1
export ROBOTICS_UWB_RIGHT=/dev/ttyACM0
export ROBOTICS_UWB_BASELINE_M=0.30
export ROBOTICS_MOTOR_PORT=/dev/ttyUSB0

python3 ai_vision/uwb_test.py --raw --duration 10
python3 ai_vision/robot_comm.py
ROBOTICS_FOLLOW_MODE=uwb ROBOTICS_MOTOR_ENABLED=1 python3 ai_vision/web_tracker.py
```

`http://127.0.0.1:5000`에서 태그를 든 사람을 **대상 등록**합니다. 태그를 카트 앞에 두고 UWB 준비 상태와 모터 연결을 확인한 다음 **주행 활성화**를 누릅니다. 바퀴를 들어 올린 채 중앙·오른쪽·왼쪽·근거리에서 바퀴 방향과 정지를 확인합니다. 실제 차체는 고정 속도 모터를 사용하므로, 바닥 시험에서 관성에 따른 이동량과 정지거리를 별도로 측정합니다.

## 카메라 없는 벤치 모드

두 앵커의 거리를 만족하는 위치는 앵커 기준선 앞뒤에 대칭으로 생깁니다. 기본 UWB 모드는 등록한 대상을 카메라에서 확인하는 동안만 주행을 허용합니다. 카메라와 UWB가 같은 사람을 보고 있는지는 태그를 등록한 사람에게 건네는 절차로 맞춥니다. 대상이 영상에서 사라지면 정지하며, 뒤쪽에 있는 태그를 단독으로 구별하려면 세 번째 비공선 기준점 등 별도 전후 관측이 필요합니다.

바퀴를 들어 올리고 **태그를 앞쪽에서만 움직이는 벤치 시험**에는 아래처럼 카메라와 모델 의존성 없이 실행할 수 있습니다.

```bash
ROBOTICS_FOLLOW_MODE=uwb ROBOTICS_UWB_FRONT_GUARD=0 ROBOTICS_MOTOR_ENABLED=1 \
  python3 ai_vision/web_tracker.py
```

모터를 연결하지 않고 제어 방향만 확인할 때는 `ROBOTICS_MOTOR_ENABLED=0`을 지정합니다. 웹에서 활성화했을 때 제어 명령은 표시되고 USB 송신 작업은 실행되지 않습니다.

## 주행 기준

| 항목 | 기본값과 동작 |
| --- | --- |
| 입력 조건 | 좌우 포트에서 맞는 ID의 새 거리 두 개. 수신 시각 차이 최대 0.32초, 각 거리 수명 0.65초 |
| 주행 범위 | 반경 3.0m 이내, 정면 기준 좌우 55° 이내, 전방 좌표 0.20m 이상 |
| 방향 | 약 18° 이상에서 제자리 회전, 10° 미만에서 다시 직진. 처음 움직이거나 방향을 바꿀 때 두 개의 유효한 측정 쌍에서 같은 판단을 확인 |
| 근거리 | 중심 거리 또는 최신 앵커 거리 중 하나가 0.85m 이하이면 정지. 정지 후 중심 거리 1.05m 이상에서 이동을 다시 허용 |
| 시간 제한 | 제어 명령 허용 구간은 0.14초이며 USB 송신은 50ms 주기. 정지 명령을 보낼 시간과 0.10초의 안정화 시간을 더 기다린 뒤 양쪽 새 거리로 다음 위치를 계산 |
| 장애 | 앵커 연결 끊김, 측정 만료, ID 오류, 불가능한 위치, 전방 카메라 확인 상실 또는 모터 ACK 상실 시 정지 후 주행 활성화 해제 |

모터 펌웨어의 초음파 정지와 직렬 워치독은 함께 동작합니다. 수신 시각은 Jetson에서 기록하며, 거리 오차는 설치 높이 차이, 안테나 간격과 반사 환경에 따라 달라집니다. 위 값은 소프트웨어의 초기 설정이며 실제 차체에서 보정합니다. [제어 설정](../ai_vision/config.py)

## 통합 시험 재현

Python 3와 Flask가 있는 환경에서 아래 명령을 실행합니다.

```bash
python3 -m pip install Flask
python3 -m unittest discover -s tests -p test_uwb_drive.py -v
```

차체 모델은 좌우 앵커를 0.23초와 0.27초 주기로 따로 갱신합니다. 컨트롤러의 명령을 받아 가상의 차체 위치와 방향을 갱신하고 다음 거리값을 생성합니다. 모델의 이동 속도는 0.45m/s, 제자리 회전 속도는 80°/s, 거리값에 더한 변동의 최대 진폭은 2mm입니다. 좌우 1.0m, 전방 2.3m에서 시작한 두 경우 모두 15.40초에 약 0.82m 거리에서 정지했습니다.

아래처럼 가상 ESP32 대신 **실제 Arduino 스케치**를 가짜 GPIO로 실행할 수도 있습니다. 통합 테스트는 가상 앵커 스트림, 주행 API, 운영 직렬 송신기, 스케치의 ACK와 모터 마스크를 한 경로로 연결합니다.

```bash
g++ -std=c++11 -I tests/firmware tests/firmware/motor_bridge.cpp -o motor_bridge
ROBOTICS_TEST_MOTOR_FIRMWARE="$PWD/motor_bridge" \
  python3 -m unittest discover -s tests -p test_uwb_drive.py -v
```

Windows에서는 실행 파일 확장자 `.exe`를 포함한 절대 경로를 환경변수에 넣습니다. 두 방법 모두 제어 코드의 동작을 검증합니다. 실제 RF 수신과 모터 토크·정지거리는 장비를 연결해 측정합니다.
