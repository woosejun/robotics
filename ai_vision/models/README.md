# YOLOv8n 모델 준비

Jetson에서 JetPack과 호환되는 PyTorch, Ultralytics 및 TensorRT Python 바인딩을 설치합니다. 해당 환경에서 `yolov8n.pt`로 TensorRT 엔진을 내보낸 뒤 이 디렉터리에 저장합니다.

```bash
yolo export model=yolov8n.pt format=engine imgsz=640 device=0 half=True
cp yolov8n.engine ai_vision/models/yolov8n.engine
```

다른 파일을 쓰려면 `ROBOTICS_MODEL`에 파일 경로를 지정합니다. Git은 모델 바이너리를 제외합니다. 재현용 기록에는 JetPack, TensorRT, PyTorch 및 Ultralytics 버전과 내보내기 옵션을 함께 남깁니다. [Ultralytics 내보내기](https://docs.ultralytics.com/modes/export/), [NVIDIA TensorRT 엔진 호환성](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/support-matrix.html#engine-portability)
