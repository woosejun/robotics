# DWM3000 앵커와 태그 펌웨어

앵커 두 개가 태그에 Poll을 보내고, 태그의 지연 응답에 담긴 타임스탬프로 거리를 계산합니다. 왼쪽은 `ANCHOR1,1.234`, 오른쪽은 `ANCHOR2,1.234` 형식으로 거리를 미터 단위로 냅니다. 태그는 Poll을 보낸 앵커 ID와 시퀀스를 응답에 포함합니다. 앵커는 같은 시퀀스의 응답만 거리 계산에 씁니다.

## 보드별 설정

각 스케치의 `BOARD`에 장비별 배선과 안테나 지연을 적습니다.

| 항목 | 왼쪽 앵커 | 오른쪽 앵커 / 태그 |
| --- | --- | --- |
| 스케치 | `anchor_left/anchor_left.ino` | `anchor_right/anchor_right.ino`, `tag/tag.ino` |
| 기존 파일의 RST, IRQ | 5, 4 | 5, 4 |
| 기존 파일의 SCK, MISO, MOSI | 21, 22, 23 | 21, 22, 23 |
| 기존 파일의 CS | 2 | 20 |
| 초기 TX/RX 안테나 지연 | 16385 / 16385 | 16385 / 16385 |

기존 핀 값은 원본 저장소에서 가져왔습니다. 스케치는 현재 ESP32 변형에서 사용할 수 없는 GPIO를 발견하면 `UWB INIT FAILED`를 출력합니다. 일반 ESP32에서는 CS로 지정된 GPIO20을 쓸 수 없습니다. ESP32-S3에서는 기존 MISO/MOSI에 지정된 GPIO22/23이 없으며, GPIO20을 USB D+로 사용하는 보드가 있습니다. 실제 보드 실크와 회로도에서 SPI·리셋·USB 핀을 확인하고 해당 스케치의 `BOARD`를 맞춥니다. [ESP32 GPIO](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html), [ESP32-S3 GPIO](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32s3/api-reference/peripherals/gpio.html), [ESP32-S3 DevKitC 핀 배치](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)

두 앵커는 230ms와 270ms의 서로 다른 Poll 주기에 작은 지터를 더합니다. 응답이 끊기면 유한 시간 이후 무선 상태를 정리하고 다음 측정을 시도합니다. RX/TX 안테나 지연은 실제 줄자 거리와 비교해 보드마다 보정합니다.

## Arduino 빌드

공유 무선 코드 `CartUwb`는 이 디렉터리의 `libraries`에 있습니다. Arduino IDE에서는 `CartUwb`와 Makerfabs의 `Dw3000`을 스케치북 `libraries`에 넣습니다. Arduino CLI에서는 아래처럼 라이브러리 경로를 지정합니다. 컴파일 예시의 ESP32-S3 보드 정의는 호환성 검사에 쓰이며, 업로드할 보드 정의는 장비에 맞춰 선택합니다.

```bash
git clone https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000.git
git -C Makerfabs-ESP32-UWB-DW3000 checkout 9d449b8bdb07f464791cf09ec3b36aeb3c4f6595
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@2.0.17 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
for role in anchor_left anchor_right tag; do
  arduino-cli compile --fqbn esp32:esp32:esp32s3 \
    --library Makerfabs-ESP32-UWB-DW3000/Dw3000 --libraries uwb/libraries "uwb/$role"
done
```

[Makerfabs DW3000 드라이버](https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000/tree/9d449b8bdb07f464791cf09ec3b36aeb3c4f6595/Dw3000)

## 펌웨어 모의 실행

```bash
g++ -std=c++11 -I tests/firmware tests/firmware/uwb_test.cpp -o uwb_test
./uwb_test
```

저장소 루트에서 실행하면 DW3000의 입출력과 클록을 흉내 내 두 앵커의 Poll, 태그 응답, 시퀀스 검사, 타임스탬프 순환과 응답 타임아웃을 시험합니다.

## 실물 확인 순서

1. 태그와 앵커 한 개에 펌웨어를 업로드합니다. 가까운 거리에서 `READY`와 `ANCHORn,거리`가 이어서 출력되는지 확인합니다.
2. 같은 절차를 다른 앵커에 반복한 뒤 두 앵커를 동시에 켭니다. 두 쪽 거리와 태그 응답이 이어지는지 확인합니다.
3. 앵커를 카트에 장착했다면 안테나 중심 간격을 설정합니다. 모터 출력을 비활성화한 상태에서 태그를 좌·중앙·우에 놓고 Jetson 좌표를 비교합니다. [Jetson 진단 절차](../docs/uwb_setup.md)
