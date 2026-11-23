#!/usr/bin/env python3
"""yolo11n.pt 를 장애물 데이터셋(build_dataset.py 결과)으로 fine-tuning 한다.

흐름
    1. 사전 점검: data.yaml, train/val 이미지·라벨 개수, 클래스별 bbox 개수
    2. 가중치 준비: training/weights/yolo11n.pt (없으면 자동 다운로드, 최초 1회 인터넷 필요)
    3. 학습: 결과는 training/runs/<name>/ (로그, 그래프, weights/best.pt, last.pt)
    4. 검증 후 best.pt 를 training/weights/<name>_best.pt 로 복사

사용 예
    pip install ultralytics                       # 최초 1회
    python3 training/scripts/train.py             # 기본값 (100 epoch, imgsz 640)
    python3 training/scripts/train.py --check-only          # 데이터만 점검
    python3 training/scripts/train.py --epochs 30 --batch 8 # GPU 가 없을 때
    python3 training/scripts/train.py --resume runs/obstacles_20261123_2100/weights/last.pt   # 중단된 학습 이어서

학습 데이터는 시뮬레이션 영상 한 번의 주행에서 나온 것이라 장면이 비슷하다. val 점수가 높아도
실제 환경/다른 맵에서의 성능은 보장되지 않으므로, 맵이나 장애물 배치를 바꿔 녹화한 영상으로 따로 확인할 것.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

TRAINING_DIR = Path(__file__).resolve().parent.parent
DEFAULT_DATA = TRAINING_DIR / "datasets" / "obstacles" / "data.yaml"
RUNS_DIR = TRAINING_DIR / "runs"
WEIGHTS_DIR = TRAINING_DIR / "weights"
BASE_MODEL = "yolo11n.pt"


# --------------------------------------------------------------------------- #
# 사전 점검 (ultralytics 없이도 동작)
# --------------------------------------------------------------------------- #
def read_data_yaml(path: Path) -> dict:
    """data.yaml(build_dataset.py 가 만든 단순 형식)을 PyYAML 없이 읽는다."""
    cfg: dict = {"names": {}}
    in_names = False
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if in_names and (m := re.match(r"\s+(\d+):\s*(.+)$", line)):
            cfg["names"][int(m.group(1))] = m.group(2).strip()
            continue
        in_names = False
        key, _, value = line.partition(":")
        if key.strip() == "names":
            in_names = True
        else:
            cfg[key.strip()] = value.strip()
    return cfg


def preflight(data_yaml: Path) -> dict:
    if not data_yaml.exists():
        sys.exit(f"data.yaml 이 없습니다: {data_yaml}\n먼저 build_dataset.py 로 데이터셋을 만드세요.")
    cfg = read_data_yaml(data_yaml)
    root = Path(cfg["path"])
    names = cfg["names"]
    print(f"데이터셋: {root}")
    print(f"클래스: {', '.join(f'{i}={n}' for i, n in sorted(names.items()))}")

    summary = {}
    for split in ("train", "val"):
        img_dir = root / cfg[split]
        lbl_dir = root / cfg[split].replace("images", "labels", 1)
        images = sorted(img_dir.glob("*.jpg"))
        missing = [p.name for p in images if not (lbl_dir / f"{p.stem}.txt").exists()]
        counts: Counter = Counter()
        empty = 0
        for p in images:
            txt = lbl_dir / f"{p.stem}.txt"
            lines = [ln for ln in txt.read_text().splitlines() if ln.strip()] if txt.exists() else []
            empty += not lines
            for ln in lines:
                cls = int(ln.split()[0])
                if cls not in names:
                    sys.exit(f"{txt}: 정의되지 않은 클래스 id {cls}")
                counts[cls] += 1
        print(f"  {split}: 이미지 {len(images)}장 (배경 {empty}장), bbox " + ", ".join(f"{names[i]} {counts[i]}" for i in sorted(names)))
        if not images:
            sys.exit(f"{split} 이미지가 없습니다: {img_dir}")
        if missing:
            sys.exit(f"{split}: 라벨 파일이 없는 이미지 {len(missing)}개 (예: {missing[0]})")
        summary[split] = counts

    for split, counts in summary.items():
        zero = [names[i] for i in names if counts[i] == 0]
        if zero:
            print(f"  [경고] {split} 에 bbox 가 없는 클래스: {', '.join(zero)} -> 해당 클래스는 {'학습' if split == 'train' else '검증'}되지 않습니다.")
    return cfg


# --------------------------------------------------------------------------- #
def pick_device(requested: str) -> str:
    if requested != "auto":
        return requested
    try:
        import torch

        if torch.cuda.is_available():
            return "0"
    except ImportError:
        pass
    return "cpu"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="yolo11n 장애물 detection 학습", formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data", default=str(DEFAULT_DATA), help="data.yaml 경로")
    p.add_argument("--model", default=str(WEIGHTS_DIR / BASE_MODEL), help="시작 가중치 (기본 training/weights/yolo11n.pt)")
    p.add_argument("--name", default=datetime.now().strftime("obstacles_%Y%m%d_%H%M"), help="runs/ 아래 결과 폴더 이름")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640, help="학습 입력 크기 (원본 640x480)")
    p.add_argument("--batch", type=int, default=16, help="-1 이면 GPU 메모리에 맞춰 자동")
    p.add_argument("--device", default="auto", help="auto | cpu | 0 | 0,1")
    p.add_argument("--patience", type=int, default=30, help="val 이 이 epoch 동안 안 좋아지면 조기 종료")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--resume", metavar="LAST_PT", help="중단된 학습의 last.pt 로 이어서 학습")
    p.add_argument("--check-only", action="store_true", help="데이터 점검만 하고 종료")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = preflight(Path(args.data))
    if args.check_only:
        print("\n점검 완료 (--check-only).")
        return

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ultralytics 가 설치되어 있지 않습니다.  pip install ultralytics")

    device = pick_device(args.device)
    if device == "cpu":
        print("\n[알림] GPU 를 쓸 수 없어 CPU 로 학습합니다. 느리면 --epochs 30 --batch 8 정도로 줄여 보세요.")

    if args.resume:
        model = YOLO(args.resume)
        print(f"\n이어서 학습: {args.resume}")
        model.train(resume=True)
        return

    model_path = Path(args.model)
    if not model_path.exists():
        from ultralytics.utils.downloads import attempt_download_asset

        WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
        print(f"\n{model_path.name} 이 없어 다운로드합니다 -> {model_path}")
        attempt_download_asset(str(model_path))
        if not model_path.exists():
            sys.exit(f"가중치를 받지 못했습니다. 인터넷 연결을 확인하거나 {model_path} 에 직접 넣어 주세요.")

    print(f"\n학습 시작: model={model_path.name} device={device} epochs={args.epochs} imgsz={args.imgsz} batch={args.batch}\n")
    model = YOLO(str(model_path))
    model.train(
        data=str(Path(args.data).resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        patience=args.patience,
        workers=args.workers,
        seed=args.seed,
        project=str(RUNS_DIR),
        name=args.name,
        exist_ok=False,
        plots=True,
    )

    save_dir = Path(model.trainer.save_dir)
    best = save_dir / "weights" / "best.pt"
    if not best.exists():
        sys.exit(f"best.pt 를 찾을 수 없습니다: {best}")

    print("\n검증 (best.pt):")
    metrics = YOLO(str(best)).val(data=str(Path(args.data).resolve()), imgsz=args.imgsz, device=device, plots=False)
    names = cfg["names"]
    print(f"  전체  mAP50 {metrics.box.map50:.3f}  mAP50-95 {metrics.box.map:.3f}  P {metrics.box.mp:.3f}  R {metrics.box.mr:.3f}")
    for i, cls_idx in enumerate(metrics.box.ap_class_index):
        print(f"  {names[int(cls_idx)]:<14} mAP50 {metrics.box.ap50[i]:.3f}  mAP50-95 {metrics.box.ap[i]:.3f}")

    WEIGHTS_DIR.mkdir(parents=True, exist_ok=True)
    final = WEIGHTS_DIR / f"{args.name}_best.pt"
    shutil.copy2(best, final)
    print(f"\n완료. 결과: {save_dir}\n최종 모델: {final}")


if __name__ == "__main__":
    main()
