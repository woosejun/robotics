# UWB 연결 설정

로봇에는 왼쪽/오른쪽 앵커를 부착하고, 사람이 태그를 소지한다. 프로그램은
두 앵커가 출력하는 태그까지의 거리로 태그의 좌우·전방 위치를 계산한다.

## 포트 설정

`ai_vision/config.py`에서 다음 값을 실제 장치에 맞춘다.

```python
UWB_RIGHT_PORT = "/dev/ttyACM0"
UWB_LEFT_PORT = "/dev/ttyACM1"
UWB_ANCHOR_BASELINE_M = 0.30
```

`UWB_ANCHOR_BASELINE_M`은 로봇에 부착한 두 앵커의 UWB 안테나 모듈 중심 사이
실제 거리(m)다. 예를 들어 모듈 중앙끼리 30cm면 `0.30`으로 설정한다.
현재 확인한 오른쪽 앵커는 `/dev/ttyACM0`이며 부팅 시 `DEVICE ID: deca0302`을
출력한다. 왼쪽 앵커를 연결한 뒤 `ls /dev/ttyACM*`로 포트를 확인해 설정한다.

## 앵커 펌웨어 출력

`ai_vision/uwb_comm.py`는 포트에 명령을 전송하지 않고, 줄 단위 거리 보고를
수신한다. 지원되는 예시는 아래와 같다.

```text
DISTANCE: 1.42 m
range=142cm
RANGE right 1.42 m
{"distance_m": 1.42}
```

앵커가 부팅 로그만 출력하고 위와 같은 거리 보고를 보내지 않으면, 태그와
range를 수행하도록 해당 UWB 펌웨어를 설정해야 한다. 이때 해당 모듈의
펌웨어 매뉴얼에 있는 명령 형식을 사용한다.

## 확인

`python3 ai_vision/web_tracker.py` 실행 후 웹 페이지의 `UWB 앵커` 항목에서
연결 상태를 확인한다. 두 앵커가 모두 태그 거리를 보내면 `UWB 태그 위치`에
`좌우 / 전방 m` 형식으로 표시된다. 좌우 값에서 음수는 로봇 중앙보다 왼쪽,
양수는 오른쪽이다.
