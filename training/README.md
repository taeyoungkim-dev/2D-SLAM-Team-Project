# training/ — YOLO11n 모델 학습

TurtleBot3 장애물 인식(전선, 배설물, 수건 등)용 `yolo11n.pt` 모델의 데이터셋 준비, 학습, 평가, 내보내기를 담당하는 디렉토리.

## 구조

```text
training/
├── configs/    # 데이터셋 yaml(data.yaml), 하이퍼파라미터 설정
├── datasets/   # 이미지/라벨 (images/{train,val,test}, labels/{train,val,test}) - git 제외
├── scripts/    # 학습(train), 검증(val), 추론(predict), 내보내기(export) 스크립트
├── runs/       # Ultralytics 학습 결과 (로그, 그래프, 체크포인트) - git 제외
└── weights/    # 사전학습 가중치(yolo11n.pt) 및 최종 모델(best.pt) - git 제외
```

## 데이터 만들기: 녹화 → 자동 라벨링

장애물 3종(`power_cable`, `cardboard_box`, `urine_puddle`)의 배치는 `assets/objects/spawn_household_obstacles.launch.py` 에서 읽는다.

### 1. 영상 녹화 (`scripts/record_video.py`)

Gazebo + 장애물 소환 후 로봇을 돌리면서 길게 한 번 녹화한다. 카메라 자세는 TF 의 `odom` 기준으로 저장한다. `custom_burger` 의 diff drive 는 기본 설정(odometry_source=WORLD)이라 `odom` 좌표가 곧 Gazebo world 좌표이므로(`record_video.py` 의 `ODOM_IN_WORLD` = 항등), 로봇 소환 위치와 무관하게 라벨링이 맞는다.

```bash
source /opt/ros/humble/setup.bash
python3 training/scripts/record_video.py --name house_run1
```

```text
datasets/recordings/house_run1/
├── rgb.mp4         # RGB 영상 (기본 10fps)
├── depth/*.png     # 프레임별 16-bit depth, mm (가려짐 판정용)
├── frames.jsonl    # 프레임별 타임스탬프, odom 기준 카메라 자세
└── meta.json       # 해상도, camera_info, odom->world 변환
```

### 2. 장면 선별 + 자동 라벨링 (`scripts/build_dataset.py`)

```bash
python3 training/scripts/build_dataset.py training/datasets/recordings/house_run1
```

장애물 world 좌표를 카메라 자세와 `camera_info` 로 영상에 투영하고, depth 로 가려진 부분을 제외해 bbox 를 만든다. 장애물이 보이는 프레임만 학습 이미지로 채택한다.

```text
datasets/obstacles/
├── images/{train,val}/*.jpg
├── labels/{train,val}/*.txt   # YOLO: class cx cy w h (정규화)
├── data.yaml                  # 학습에 바로 사용
├── preview/*.jpg              # 라벨 확인용 (학습 전 반드시 확인)
└── report_*.json
```

- train/val 은 프레임이 아니라 연속 구간(기본 10초) 단위로 나눈다.
- 장애물이 없는 배경 프레임을 10% 섞는다(`--negatives`).
- 여러 녹화를 한 번에 합칠 수 있다: `build_dataset.py rec1 rec2 ...`

## 학습 (`scripts/train.py`)

```bash
# 최초 1회: ROS 의 numpy/opencv 와 섞이지 않도록 가상환경(training/.venv)에 설치한다
python3 -m venv training/.venv && training/.venv/bin/pip install ultralytics

training/.venv/bin/python training/scripts/train.py --check-only   # 데이터 개수/클래스 점검
training/.venv/bin/python training/scripts/train.py --name obstacles_v1   # yolo11n.pt 로 fine-tuning (기본 100 epoch)
```

- 결과는 `runs/<name>/`, 최종 모델은 `weights/<name>_best.pt` 에 저장된다.
- 최초 실행 때 `yolo11n.pt` 다운로드를 위해 인터넷이 필요하다 (이후는 `weights/` 의 파일을 사용).
- GPU 가 없으면 CPU 로 학습하므로 `--epochs 30 --batch 8` 등으로 줄인다.

## 워크플로우

1. 영상 녹화 → 자동 라벨링 → `datasets/obstacles/`
2. `datasets/obstacles/data.yaml` 확인 (클래스와 경로는 자동 생성됨)
3. `scripts/` 의 학습 스크립트로 `weights/yolo11n.pt` 기반 fine-tuning
4. 결과(`runs/`)에서 `best.pt` 선정 → `weights/` 에 보관
5. 로봇(`robot_ws`)의 YOLO 노드에서 해당 가중치 사용

## 참고

- 의존성: `pip install ultralytics`
- 대용량 파일(데이터셋, 가중치, 학습 결과)은 `.gitignore` 로 커밋에서 제외한다.
