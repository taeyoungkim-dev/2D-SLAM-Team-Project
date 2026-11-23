#!/usr/bin/env python3
"""녹화 영상에서 장애물이 보이는 장면만 골라 자동 라벨링하고 YOLO 학습데이터로 가공한다.

입력은 ``record_video.py`` 가 만든 녹화 폴더(rgb.mp4, depth/, frames.jsonl, meta.json).
ROS 없이 numpy + OpenCV 만으로 동작한다.

자동 라벨링 방식 (Gazebo ground truth 투영)
    장애물 3종의 world 좌표와 모양은 assets/objects/spawn_household_obstacles.launch.py 와
    각 SDF/메시 파일에 이미 정의되어 있다. 프레임마다 아래 순서로 bounding box 를 계산한다.

    1. 장애물 표면을 점으로 샘플링 (box/cylinder/mesh) 하고 launch 파일의 x, y, yaw 로 world 에 배치
    2. meta.json 의 odom_in_world(odom->world 변환, 기본 항등) + 프레임별 odom->카메라 자세 = world 기준 카메라 자세
    3. camera_info 의 K 로 영상에 투영
    4. depth 영상과 비교해 "실제로 카메라에 보이는 점"만 남김 (벽/가구/다른 물체에 가려진 점 제외)
    5. 보이는 점의 외접 사각형 = bounding box. 너무 작거나 멀거나 잘린 것은 버림

    라벨이 하나라도 남은 프레임만 학습 이미지로 채택한다. 로봇이 거의 안 움직인 중복 프레임은
    건너뛰고, 장애물이 전혀 안 보이는 프레임을 일부(--negatives) 섞어 오탐을 줄인다.

출력 (<--out>/<--name>/)
    images/{train,val}/*.jpg     학습/검증 이미지
    labels/{train,val}/*.txt     YOLO 형식 "class cx cy w h" (0~1 정규화)
    data.yaml                    Ultralytics 학습 설정 (경로, 클래스 이름)
    preview/*.jpg                라벨을 그린 확인용 이미지 (꼭 눈으로 확인할 것)
    report_<녹화이름>.json       프레임/클래스별 통계

    train/val 은 프레임이 아니라 연속 구간(--block-seconds) 단위로 나눈다. 인접 프레임은 거의
    같은 장면이라 섞어 나누면 검증 점수가 부풀려지기 때문이다.

사용 예
    python3 training/scripts/build_dataset.py training/datasets/recordings/house_run1
    python3 training/scripts/build_dataset.py rec1 rec2 --name obstacles --negatives 0.2
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import random
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent.parent
REPO_DIR = TRAINING_DIR.parent
DEFAULT_LAUNCH = REPO_DIR / "assets" / "objects" / "spawn_household_obstacles.launch.py"
DEFAULT_OUT = TRAINING_DIR / "datasets"

# launch 파일의 모델 이름 -> assets/objects 기준 SDF 경로 (기본값은 "<이름>.sdf")
SDF_FILES = {"power_cable": "tangled_power_cable/model.sdf"}

SAMPLE_SPACING = 0.01  # 표면 샘플 간격 (m)


# --------------------------------------------------------------------------- #
# 기하 유틸
# --------------------------------------------------------------------------- #
def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_matrix(values) -> np.ndarray:
    x, y, z, r, p, yw = values
    m = np.eye(4)
    m[:3, :3] = rpy_to_rot(r, p, yw)
    m[:3, 3] = (x, y, z)
    return m


def parse_pose(elem: Optional[ET.Element]) -> np.ndarray:
    if elem is None or not (elem.text or "").strip():
        return np.eye(4)
    vals = [float(v) for v in elem.text.split()]
    return pose_matrix(vals + [0.0] * (6 - len(vals)))


def transform(points: np.ndarray, m: np.ndarray) -> np.ndarray:
    return points @ m[:3, :3].T + m[:3, 3]


# --------------------------------------------------------------------------- #
# SDF -> 표면 점군
# --------------------------------------------------------------------------- #
def _linspace(length: float) -> np.ndarray:
    n = max(2, int(math.ceil(length / SAMPLE_SPACING)) + 1)
    return np.linspace(-length / 2, length / 2, n)


def box_points(size) -> np.ndarray:
    sx, sy, sz = size
    pts = []
    for axis, (a, b) in enumerate(((1, 2), (0, 2), (0, 1))):
        ga, gb = np.meshgrid(_linspace(size[a]), _linspace(size[b]), indexing="ij")
        for sign in (-1, 1):
            face = np.zeros((ga.size, 3))
            face[:, a], face[:, b], face[:, axis] = ga.ravel(), gb.ravel(), sign * size[axis] / 2
            pts.append(face)
    return np.vstack(pts)


def cylinder_points(radius: float, length: float) -> np.ndarray:
    xs = np.arange(-radius, radius + 1e-9, SAMPLE_SPACING)
    gx, gy = np.meshgrid(xs, xs)
    disc = np.column_stack([gx.ravel(), gy.ravel()])
    disc = disc[np.hypot(disc[:, 0], disc[:, 1]) <= radius]
    pts = [np.column_stack([disc, np.full(len(disc), z)]) for z in (-length / 2, length / 2)]
    n_theta = max(16, int(2 * math.pi * radius / SAMPLE_SPACING))
    theta = np.linspace(0, 2 * math.pi, n_theta, endpoint=False)
    for z in np.linspace(-length / 2, length / 2, max(2, int(math.ceil(length / SAMPLE_SPACING)) + 1)):
        pts.append(np.column_stack([radius * np.cos(theta), radius * np.sin(theta), np.full(n_theta, z)]))
    return np.vstack(pts)


def mesh_points(path: Path, scale) -> np.ndarray:
    """COLLADA(.dae) 의 POSITION 정점. 파일의 unit(meter) 값을 반영한다."""
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    unit = root.find(f"{ns}asset/{ns}unit")
    unit_scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    source_id = None
    for inp in root.iter(f"{ns}input"):
        if inp.get("semantic") == "POSITION":
            source_id = inp.get("source", "").lstrip("#")
            break
    arrays = {a.get("id"): a for a in root.iter(f"{ns}float_array")}
    # <vertices id=X><input semantic=POSITION source=#positions/></vertices> 를 따라간다.
    for v in root.iter(f"{ns}vertices"):
        if v.get("id") == source_id:
            for inp in v.findall(f"{ns}input"):
                if inp.get("semantic") == "POSITION":
                    source_id = inp.get("source", "").lstrip("#")
    source = next((s for s in root.iter(f"{ns}source") if s.get("id") == source_id), None)
    array = source.find(f"{ns}float_array") if source is not None else None
    if array is None:
        array = next(iter(arrays.values()))
    pts = np.array(array.text.split(), dtype=float).reshape(-1, 3)
    return pts * unit_scale * np.array(scale)


def resolve_mesh(uri: str, sdf_path: Path, asset_dir: Path) -> Path:
    if uri.startswith("model://"):
        return asset_dir / uri[len("model://"):]
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    p = Path(uri)
    return p if p.is_absolute() else sdf_path.parent / p


def sdf_surface_points(sdf_path: Path, asset_dir: Path) -> np.ndarray:
    """SDF 의 <visual> 만 모아 모델 좌표계의 표면 점군을 만든다 (collision 은 제외)."""
    model = ET.parse(sdf_path).getroot().find("model")
    if model is None:
        raise ValueError(f"{sdf_path}: <model> 이 없습니다.")
    model_pose = parse_pose(model.find("pose"))
    chunks = []
    for link in model.findall("link"):
        link_pose = model_pose @ parse_pose(link.find("pose"))
        for visual in link.findall("visual"):
            geom = visual.find("geometry")
            if geom is None:
                continue
            m = link_pose @ parse_pose(visual.find("pose"))
            if (b := geom.find("box")) is not None:
                pts = box_points([float(v) for v in b.find("size").text.split()])
            elif (c := geom.find("cylinder")) is not None:
                pts = cylinder_points(float(c.find("radius").text), float(c.find("length").text))
            elif (me := geom.find("mesh")) is not None:
                scale_elem = me.find("scale")
                scale = [float(v) for v in scale_elem.text.split()] if scale_elem is not None else [1, 1, 1]
                pts = mesh_points(resolve_mesh(me.find("uri").text.strip(), sdf_path, asset_dir), scale)
            else:
                print(f"  [경고] {sdf_path.name}: 지원하지 않는 geometry 건너뜀")
                continue
            chunks.append(transform(pts, m))
    if not chunks:
        raise ValueError(f"{sdf_path}: 사용할 수 있는 <visual> 이 없습니다.")
    return np.vstack(chunks)


# --------------------------------------------------------------------------- #
# launch 파일에서 장애물 배치 읽기
# --------------------------------------------------------------------------- #
@dataclass
class Obstacle:
    name: str
    cls: int
    points: np.ndarray  # world 좌표 (N, 3)


def load_obstacles(launch_file: Path) -> tuple[list[str], list[Obstacle]]:
    """launch 파일의 ``objects = ((이름, sdf, x, y, yaw), ...)`` 를 읽어 world 점군으로 만든다."""
    tree = ast.parse(launch_file.read_text())
    entries = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "objects" for t in node.targets):
            entries = node.value
            break
    if not isinstance(entries, (ast.Tuple, ast.List)):
        raise ValueError(f"{launch_file} 에서 'objects = (...)' 를 찾지 못했습니다.")

    asset_dir = launch_file.parent
    names: list[str] = []
    obstacles: list[Obstacle] = []
    for entry in entries.elts:
        if not isinstance(entry, ast.Tuple) or len(entry.elts) < 5:
            continue
        name = ast.literal_eval(entry.elts[0])
        x, y, yaw = (float(ast.literal_eval(e)) for e in entry.elts[-3:])
        sdf_path = asset_dir / SDF_FILES.get(name, f"{name}.sdf")
        local = sdf_surface_points(sdf_path, asset_dir)
        world = transform(local, pose_matrix((x, y, 0.0, 0.0, 0.0, yaw)))
        if name not in names:
            names.append(name)
        obstacles.append(Obstacle(name, names.index(name), world))
        print(f"  장애물 {name}: ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) 점 {len(world)}개")
    if not obstacles:
        raise ValueError(f"{launch_file} 에 장애물이 없습니다.")
    return names, obstacles


# --------------------------------------------------------------------------- #
# 투영 / 가시성 / bbox
# --------------------------------------------------------------------------- #
@dataclass
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def camera_in_world(pose: dict, odom_in_world: tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """odom 기준 카메라 자세 + odom 원점의 world 좌표(x, y, z, yaw) -> world 기준 (R, t).

    Gazebo diff drive 가 odometry_source=WORLD(기본값)이면 odom == world 이므로 항등이다.
    """
    ox, oy, oz, oyaw = odom_in_world
    r_wo = rpy_to_rot(0, 0, oyaw)
    r_oc = quat_to_rot(pose["q"])
    t_oc = np.array(pose["t"])
    return r_wo @ r_oc, r_wo @ t_oc + np.array([ox, oy, oz])


def project(points_w: np.ndarray, r_wc: np.ndarray, t_wc: np.ndarray, cam: Camera, near: float):
    pc = (points_w - t_wc) @ r_wc  # = R_wc^T (p - t)
    z = pc[:, 2]
    front = z > near
    safe_z = np.where(front, z, 1.0)
    u = cam.fx * pc[:, 0] / safe_z + cam.cx
    v = cam.fy * pc[:, 1] / safe_z + cam.cy
    return u, v, z, front


def depth_consistent(depth_m: np.ndarray, u: np.ndarray, v: np.ndarray, z: np.ndarray) -> np.ndarray:
    """투영점의 깊이가 depth 영상과 일치하면 True (= 가려지지 않은 표면 점).

    경계 화소 반올림 오차를 줄이려고 3x3 이웃 중 하나라도 맞으면 인정한다. depth 0(무효)은 불인정.
    """
    h, w = depth_m.shape
    padded = np.full((h + 2, w + 2), np.nan, dtype=np.float32)
    padded[1:-1, 1:-1] = np.where(depth_m > 0, depth_m, np.nan)
    ui = np.clip(np.rint(u).astype(int), 0, w - 1)
    vi = np.clip(np.rint(v).astype(int), 0, h - 1)
    tol = 0.03 + 0.02 * z
    ok = np.zeros(len(z), dtype=bool)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            with np.errstate(invalid="ignore"):
                ok |= np.abs(padded[vi + dy, ui + dx] - z) <= tol
    return ok


@dataclass
class Label:
    cls: int
    name: str
    box: tuple[float, float, float, float]  # x1, y1, x2, y2 (px)
    visible_points: int
    range_m: float


def label_frame(
    obstacles: list[Obstacle],
    r_wc: np.ndarray,
    t_wc: np.ndarray,
    cam: Camera,
    load_depth,
    args: argparse.Namespace,
) -> tuple[list[Label], bool]:
    """프레임 하나의 라벨 목록과, 어떤 장애물도 시야에 걸리지 않는 '깨끗한 배경'인지 여부."""
    projections = []
    for ob in obstacles:
        u, v, z, front = project(ob.points, r_wc, t_wc, cam, args.near)
        inframe = front & (u >= 0) & (u < cam.width) & (v >= 0) & (v < cam.height)
        projections.append((ob, u, v, z, front, inframe))

    any_inframe = any(p[5].any() for p in projections)
    if not any_inframe:
        return [], True

    candidates = [p for p in projections if p[5].any() and float(np.median(p[3][p[5]])) <= args.max_range]
    if not candidates:
        return [], False

    depth = load_depth() if not args.no_depth_check else None
    if depth is None and not args.no_depth_check:
        return [], False  # depth 가 없으면 가려짐을 판단할 수 없어 라벨을 만들지 않는다.

    labels: list[Label] = []
    for ob, u, v, z, front, inframe in candidates:
        visible = inframe.copy()
        if depth is not None:
            idx = np.flatnonzero(inframe)
            visible[idx] = depth_consistent(depth, u[idx], v[idx], z[idx])
        n_vis = int(visible.sum())
        if n_vis < args.min_points:
            continue
        if inframe.sum() / max(1, front.sum()) < args.min_inframe:
            continue  # 대부분이 화면 밖으로 잘림
        pad = 1.5
        x1 = max(0.0, float(u[visible].min()) - pad)
        y1 = max(0.0, float(v[visible].min()) - pad)
        x2 = min(float(cam.width), float(u[visible].max()) + pad)
        y2 = min(float(cam.height), float(v[visible].max()) + pad)
        w, h = x2 - x1, y2 - y1
        if w < args.min_px or h < args.min_px or w * h < args.min_area:
            continue
        labels.append(Label(ob.cls, ob.name, (x1, y1, x2, y2), n_vis, float(np.median(z[visible]))))
    return labels, False


# --------------------------------------------------------------------------- #
# 녹화 처리
# --------------------------------------------------------------------------- #
def rotation_angle(q1, q2) -> float:
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


def moved_enough(pose: dict, last: Optional[dict], min_move: float, min_rot: float) -> bool:
    if last is None:
        return True
    dist = math.dist(pose["t"], last["t"])
    return dist >= min_move or rotation_angle(pose["q"], last["q"]) >= min_rot


@dataclass
class Selected:
    frame: int
    labels: list[Label]
    block: int


def scan_recording(rec_dir: Path, obstacles: list[Obstacle], args: argparse.Namespace):
    meta = json.loads((rec_dir / "meta.json").read_text())
    if not meta.get("complete"):
        print("  [경고] meta.json 이 complete=false 입니다. 녹화가 비정상 종료됐을 수 있습니다.")
    info = meta["camera_info"]
    cam = Camera(info["fx"], info["fy"], info["cx"], info["cy"], meta["width"], meta["height"])
    ow = meta.get("odom_in_world", {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})  # 없으면 odom == world
    odom_in_world = (ow["x"], ow["y"], ow.get("z", 0.0), ow["yaw"])
    if args.odom_in_world:
        odom_in_world = (args.odom_in_world[0], args.odom_in_world[1], odom_in_world[2], args.odom_in_world[2])
    print(
        f"  odom->world: x={odom_in_world[0]:.2f} y={odom_in_world[1]:.2f} z={odom_in_world[2]:.4f} yaw={odom_in_world[3]:.2f}"
        f"  fps={meta['fps']}  프레임={meta['frames']}"
    )

    frames = [json.loads(line) for line in (rec_dir / "frames.jsonl").read_text().splitlines() if line.strip()]
    depth_scale = meta.get("depth_scale", 0.001)
    min_rot = math.radians(args.min_rot_deg)

    positives: list[Selected] = []
    negatives: list[int] = []
    last_pos_pose = None
    last_neg_pose = None
    no_pose = 0
    block_size = max(1, int(round(args.block_seconds * meta["fps"])))

    for fr in frames:
        pose = fr.get("pose")
        if pose is None:
            no_pose += 1
            continue
        r_wc, t_wc = camera_in_world(pose, odom_in_world)

        def load_depth(fr=fr):
            if not fr.get("depth"):
                return None
            raw = cv2.imread(str(rec_dir / fr["depth"]), cv2.IMREAD_UNCHANGED)
            return None if raw is None else raw.astype(np.float32) * depth_scale

        labels, clean = label_frame(obstacles, r_wc, t_wc, cam, load_depth, args)
        if labels:
            if moved_enough(pose, last_pos_pose, args.min_move, min_rot):
                positives.append(Selected(fr["frame"], labels, fr["frame"] // block_size))
                last_pos_pose = pose
        elif clean and moved_enough(pose, last_neg_pose, args.min_move, min_rot):
            negatives.append(fr["frame"])
            last_neg_pose = pose

    print(f"  pose 없는 프레임 {no_pose}, 장애물 프레임 {len(positives)}, 배경 후보 {len(negatives)}")
    return meta, positives, negatives, block_size


def assign_splits(items: list[tuple[str, int, int]], val_ratio: float, seed: int) -> dict[tuple[str, int], str]:
    """(녹화, 프레임, 블록) -> 'train' | 'val'. 연속 구간(블록) 단위로 나눠 누수를 막는다."""
    blocks = sorted({(rec, blk) for rec, _, blk in items})
    rng = random.Random(seed)
    rng.shuffle(blocks)
    n_val = int(round(len(blocks) * val_ratio)) if len(blocks) >= 2 else 0
    if val_ratio > 0 and len(blocks) >= 2:
        n_val = max(1, min(n_val, len(blocks) - 1))
    val_blocks = set(blocks[:n_val])
    split = {}
    for rec, frame, blk in items:
        split[(rec, frame)] = "val" if (rec, blk) in val_blocks else "train"
    if not val_blocks and val_ratio > 0:
        print("  [경고] 구간이 1개뿐이라 프레임 5개당 1개를 val 로 씁니다. 더 길게 녹화하는 것을 권장합니다.")
        for i, (rec, frame, _) in enumerate(items):
            if i % 5 == 4:
                split[(rec, frame)] = "val"
    return split


def yolo_line(label: Label, width: int, height: int) -> str:
    x1, y1, x2, y2 = label.box
    return f"{label.cls} {(x1 + x2) / 2 / width:.6f} {(y1 + y2) / 2 / height:.6f} {(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"


def draw_preview(img: np.ndarray, labels: list[Label]) -> np.ndarray:
    palette = [(60, 60, 255), (60, 200, 60), (255, 160, 40), (200, 60, 200)]
    out = img.copy()
    for lb in labels:
        x1, y1, x2, y2 = (int(round(v)) for v in lb.box)
        color = palette[lb.cls % len(palette)]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, f"{lb.name} {lb.range_m:.1f}m", (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def write_dataset(out_root: Path, names: list[str], jobs: list[dict], args: argparse.Namespace) -> dict:
    """영상을 순차로 읽으면서 선택된 프레임만 이미지/라벨로 저장한다."""
    for split in ("train", "val"):
        (out_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_root / "labels" / split).mkdir(parents=True, exist_ok=True)
    preview_dir = out_root / "preview"
    preview_dir.mkdir(exist_ok=True)

    stats = {"images": {"train": 0, "val": 0}, "negatives": 0, "boxes": {n: 0 for n in names}}
    preview_budget = args.preview
    positives_total = sum(1 for j in jobs if j["labels"])
    preview_step = max(1, positives_total // max(1, args.preview))
    seen_positive = 0

    by_rec: dict[str, list[dict]] = {}
    for j in jobs:
        by_rec.setdefault(j["rec_name"], []).append(j)

    for rec_name, rec_jobs in by_rec.items():
        # 같은 녹화를 다시 처리할 때 이전 결과(다른 split 에 남은 파일 포함)를 먼저 지운다.
        for split in ("train", "val"):
            for sub, ext in (("images", "jpg"), ("labels", "txt")):
                for old in (out_root / sub / split).glob(f"{rec_name}_*.{ext}"):
                    old.unlink()
        for old in preview_dir.glob(f"{rec_name}_*.jpg"):
            old.unlink()

        wanted = {j["frame"]: j for j in rec_jobs}
        cap = cv2.VideoCapture(str(rec_jobs[0]["video"]))
        index = 0
        remaining = len(wanted)
        while remaining:
            ok, frame = cap.read()
            if not ok:
                print(f"  [경고] {rec_name}: 영상이 프레임 {index} 에서 끝나 {remaining}개를 저장하지 못했습니다.")
                break
            job = wanted.get(index)
            if job is not None:
                remaining -= 1
                stem = f"{rec_name}_{index:06d}"
                split = job["split"]
                cv2.imwrite(str(out_root / "images" / split / f"{stem}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality])
                h, w = frame.shape[:2]
                lines = [yolo_line(lb, w, h) for lb in job["labels"]]
                (out_root / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
                stats["images"][split] += 1
                if job["labels"]:
                    for lb in job["labels"]:
                        stats["boxes"][lb.name] += 1
                    if preview_budget > 0 and seen_positive % preview_step == 0:
                        cv2.imwrite(str(preview_dir / f"{stem}.jpg"), draw_preview(frame, job["labels"]))
                        preview_budget -= 1
                    seen_positive += 1
                else:
                    stats["negatives"] += 1
            index += 1
        cap.release()
    return stats


def write_data_yaml(out_root: Path, names: list[str]) -> None:
    lines = [f"path: {out_root.resolve()}", "train: images/train", "val: images/val", "names:"]
    lines += [f"  {i}: {n}" for i, n in enumerate(names)]
    (out_root / "data.yaml").write_text("\n".join(lines) + "\n")


def main() -> None:
    args = parse_args()
    print(f"장애물 정의: {args.launch_file}")
    names, obstacles = load_obstacles(Path(args.launch_file))

    out_root = Path(args.out) / args.name
    jobs: list[dict] = []
    split_items: list[tuple[str, int, int]] = []
    per_rec_report = {}

    for rec in args.recordings:
        rec_dir = Path(rec)
        print(f"\n녹화 처리: {rec_dir}")
        meta, positives, negatives, _ = scan_recording(rec_dir, obstacles, args)
        rec_name = re.sub(r"[^A-Za-z0-9_-]+", "_", rec_dir.resolve().name)
        video = rec_dir / meta["video"]

        n_neg = min(len(negatives), int(round(len(positives) * args.negatives)))
        neg_frames = [negatives[int(i * len(negatives) / n_neg)] for i in range(n_neg)] if n_neg else []
        block_size = max(1, int(round(args.block_seconds * meta["fps"])))

        for s in positives:
            jobs.append({"rec_name": rec_name, "frame": s.frame, "labels": s.labels, "video": video, "block": s.block})
            split_items.append((rec_name, s.frame, s.block))
        for f in neg_frames:
            jobs.append({"rec_name": rec_name, "frame": f, "labels": [], "video": video, "block": f // block_size})
            split_items.append((rec_name, f, f // block_size))
        per_rec_report[rec_name] = {
            "recording": str(rec_dir),
            "frames_total": meta["frames"],
            "positive_frames": len(positives),
            "negative_frames": len(neg_frames),
        }

    if not any(j["labels"] for j in jobs):
        raise SystemExit(
            "\n라벨이 생성된 프레임이 없습니다. 확인할 것:\n"
            "  - odom->world 변환(meta.json 의 odom_in_world)이 맞는지 (diff drive odometry_source 확인)\n"
            "  - 장애물이 실제로 카메라에 찍혔는지, --max-range / --min-px 가 너무 엄격하지 않은지"
        )

    splits = assign_splits(split_items, args.val_ratio, args.seed)
    for j in jobs:
        j["split"] = splits[(j["rec_name"], j["frame"])]
    jobs.sort(key=lambda j: (j["rec_name"], j["frame"]))

    print(f"\n저장: {out_root}")
    stats = write_dataset(out_root, names, jobs, args)
    write_data_yaml(out_root, names)
    for rec_name, rep in per_rec_report.items():
        (out_root / f"report_{rec_name}.json").write_text(json.dumps({**rep, **stats}, indent=2, ensure_ascii=False))

    print(f"\n완료: train {stats['images']['train']}장, val {stats['images']['val']}장 (배경 {stats['negatives']}장)")
    for n, c in stats["boxes"].items():
        print(f"  {n}: bbox {c}개" + ("   <- 0개: 학습할 수 없습니다. odom->world 정렬/녹화 경로를 확인하세요" if c == 0 else ""))
    print(f"학습 설정: {out_root / 'data.yaml'}")
    print(f"라벨 확인: {out_root / 'preview'}  (학습 전에 꼭 눈으로 확인하세요)")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="녹화 영상 -> 자동 라벨링된 YOLO 데이터셋", formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recordings", nargs="+", help="record_video.py 가 만든 녹화 폴더 (여러 개 가능)")
    p.add_argument("--name", default="obstacles", help="데이터셋 이름 (기본: obstacles)")
    p.add_argument("--out", default=str(DEFAULT_OUT), help="저장 루트 (기본: training/datasets)")
    p.add_argument("--launch-file", default=str(DEFAULT_LAUNCH), help="장애물 배치가 정의된 launch 파일")
    p.add_argument("--odom-in-world", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                   help="odom 원점의 world 좌표. meta.json 의 odom_in_world(기본 항등)를 덮어쓴다")
    p.add_argument("--val-ratio", type=float, default=0.2, help="검증 구간 비율 (기본 0.2)")
    p.add_argument("--block-seconds", type=float, default=10.0, help="train/val 분할 단위 구간 길이(초)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--negatives", type=float, default=0.1, help="장애물 없는 배경 프레임 비율 (장애물 프레임 대비, 기본 0.1)")
    p.add_argument("--min-move", type=float, default=0.05, help="이전 채택 프레임 대비 최소 이동(m)")
    p.add_argument("--min-rot-deg", type=float, default=5.0, help="이전 채택 프레임 대비 최소 회전(deg)")
    p.add_argument("--max-range", type=float, default=3.0, help="이보다 먼 장애물은 라벨하지 않음(m)")
    p.add_argument("--near", type=float, default=0.1, help="카메라 최소 거리(m)")
    p.add_argument("--min-px", type=float, default=6.0, help="bbox 최소 변 길이(px)")
    p.add_argument("--min-area", type=float, default=100.0, help="bbox 최소 면적(px^2)")
    p.add_argument("--min-points", type=int, default=10, help="가려지지 않은 표면 점 최소 개수")
    p.add_argument("--min-inframe", type=float, default=0.5, help="화면 안에 있는 점의 최소 비율 (잘림 제한)")
    p.add_argument("--no-depth-check", action="store_true", help="depth 가려짐 판정을 끈다 (가려진 장애물도 라벨됨)")
    p.add_argument("--preview", type=int, default=30, help="확인용 라벨 오버레이 이미지 수")
    p.add_argument("--jpg-quality", type=int, default=95)
    return p.parse_args()


if __name__ == "__main__":
    main()
