# Jetson에서 UWB 연결 확인

좌우 앵커를 각각 USB로 연결하고 태그에 전원을 켭니다. 앵커의 안테나 중심 간격을 재서 미터로 설정합니다. Linux에서 `ls -l /dev/serial/by-id/`와 `ls -l /dev/serial/by-path/`로 안정적인 장치 경로를 찾아 지정합니다. 장치를 하나씩 꽂아 왼쪽 `ANCHOR1`, 오른쪽 `ANCHOR2`의 포트를 확인합니다.

```bash
export ROBOTICS_UWB_LEFT=/dev/ttyACM1
export ROBOTICS_UWB_RIGHT=/dev/ttyACM0
export ROBOTICS_UWB_BASELINE_M=0.30
python3 ai_vision/uwb_test.py --raw --duration 10
```

`--left`, `--right`, `--baud`, `--baseline`으로 진단 중에 값을 덮어쓸 수 있습니다. 앵커 하나만 검사하려면 다른 쪽 인자에 빈 문자열을 지정합니다. 대시보드와 진단 도구는 같은 수신 코드를 쓰므로 장치 진단 중에는 대시보드 프로세스를 먼저 종료합니다.

앵커 없이 카메라만 확인할 때는 `ROBOTICS_UWB_ENABLED=0`으로 대시보드를 실행합니다.

| 출력 또는 상태 | 다음으로 확인할 부분 |
| --- | --- |
| `DISCONNECTED` | 케이블, 장치 경로, 현재 사용자 권한 |
| `DEVICE ID` 또는 `READY`만 출력 | 해당 보드가 거리 측정 스케치를 실행하는지, 양쪽 전원과 스케치 ID |
| `UWB INIT FAILED` | 실제 ESP32 변형, GPIO, SPI·리셋 배선과 전원 |
| `NO VALID RESPONSE` | 태그 전원과 위치, 두 보드의 동일한 펌웨어 프로토콜, 앵커 한 개씩의 무선 통신 |
| `ID_MISMATCH` | 왼쪽과 오른쪽의 포트 매핑 |
| `STALE` 또는 `UNSYNCED` | 앵커별 출력 끊김, 동시 측정 시각 차이 |
| `INVALID_GEOMETRY` | 안테나 간격 측정, 거리 보정, 장애물과 다중경로 |
| `OK` | 좌·중앙·우 기준 위치에서 좌표 방향과 거리 오차를 검증 |

앵커별 최근 유효 거리는 1.5초간 유지합니다. 좌우의 **Jetson 수신 시각** 차이가 0.45초를 넘으면 두 점을 짝짓지 않습니다. 두 거리로 만들 수 있는 삼각형인 경우에만 좌표를 산출합니다. 두 앵커의 삼각측량은 앞·뒤 대칭이므로 좌표의 전방값은 태그가 카트 앞에 있다는 가정을 사용합니다.

[펌웨어 빌드와 핀 프로필](../uwb/README.md), [측정값을 모터에 연결하는 방법](uwb_drive.md)
