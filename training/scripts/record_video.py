#!/usr/bin/env python3
"""RGB-D 주행 영상 녹화 노드 (YOLO 학습데이터 원본 촬영용).

로봇을 돌아다니게 하면서 한 번 길게 녹화해 두고, 장면 선별/자동 라벨링은
``build_dataset.py`` 가 나중에 처리한다. 이 스크립트는 "나중에 라벨을 자동으로 붙일 수 있도록"
영상과 함께 아래 정보를 같은 프레임 번호로 묶어 저장하는 역할만 한다.

구독 topic
    /camera/image_raw             (sensor_msgs/Image)       RGB      -> rgb.mp4
    /camera/depth/image_raw       (sensor_msgs/Image)       깊이     -> depth/*.png (가려짐 판정용)
    /camera/depth/camera_info     (sensor_msgs/CameraInfo)  내부 파라미터 -> meta.json
    /tf                           odom -> 카메라 frame      프레임별 카메라 자세 -> frames.jsonl

저장 구조 (<--out>/<--name>/)
    rgb.mp4          --fps 로 샘플링한 RGB 영상 (프레임 i = frames.jsonl 의 i번째 줄)
    depth/*.png      프레임 번호별 16-bit PNG, 단위 mm, 0 = 측정 불가
    frames.jsonl     프레임별 {frame, stamp, depth, depth_dt, pose{t,q}}   (pose = odom 기준 카메라 자세)
    meta.json        해상도, fps, camera_info, odom->world 변환(odom_in_world), 프레임 수

odom -> world 변환 (ODOM_IN_WORLD, 하드코딩)
    자동 라벨링은 Gazebo world 좌표의 장애물을 영상에 투영하므로 TF 의 odom 좌표가 world 좌표와
    어떤 관계인지 알아야 한다. custom_burger 의 diff drive 플러그인은 odometry_source 를 지정하지
    않아 기본값(WORLD)으로 동작하고, 이때 odom 좌표 = Gazebo world 좌표다 (로봇 소환 위치를 odom 원점으로
    쓰는 ENCODER 방식이 아니다). 실제 녹화에서도 첫 프레임 카메라 자세가 (-1.95, -0.47) 로, 소환 위치
    (-2.0, -0.5) 근처에서 시작하는 것으로 확인했다. 따라서 변환은 항등이다.
    odometry_source 를 ENCODER 로 바꾸면 이 상수를 소환 위치로 바꿔야 한다.

자세 필수
    카메라 자세(odom -> 카메라 frame TF)를 구할 수 없으면 녹화를 시작하지 않는다. 자세가 없는 영상은
    자동 라벨링에 쓸 수 없기 때문이다. 못 구하는 이유(/tf 없음, frame 없음, 트리 단절)를 로그로 알리고,
    --pose-timeout 초 안에 해결되지 않으면 종료한다. 녹화 도중 자세를 잃은 프레임은 저장하지 않고 건너뛴다.

조작
    Space   일시정지 / 재개
    q       종료 (Ctrl-C 도 가능)

사용 예
    source /opt/ros/humble/setup.bash
    python3 training/scripts/record_video.py --name house_run1
"""

from __future__ import annotations

import argparse
import json
import re
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

TRAINING_DIR = Path(__file__).resolve().parent.parent
DEFAULT_OUT = TRAINING_DIR / "datasets" / "recordings"
DEPTH_SCALE = 0.001  # PNG 한 단위 = 1 mm

# odom 프레임 원점의 world 좌표. diff drive 가 odometry_source=WORLD(기본값)라 odom == world (항등).
ODOM_IN_WORLD = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}


# --------------------------------------------------------------------------- #
# sensor_msgs/Image -> numpy (cv_bridge 의존성을 피하려고 직접 디코딩)
# --------------------------------------------------------------------------- #
def _to_array(msg: Image, dtype, channels: int) -> np.ndarray:
    itemsize = np.dtype(dtype).itemsize
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    row = row[:, : msg.width * channels * itemsize]
    arr = np.ascontiguousarray(row).view(dtype)
    return arr.reshape(msg.height, msg.width, channels) if channels > 1 else arr.reshape(msg.height, msg.width)


def rgb_msg_to_bgr(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    if enc == "rgb8":
        return cv2.cvtColor(_to_array(msg, np.uint8, 3), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return _to_array(msg, np.uint8, 3)
    if enc == "rgba8":
        return cv2.cvtColor(_to_array(msg, np.uint8, 4), cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(_to_array(msg, np.uint8, 4), cv2.COLOR_BGRA2BGR)
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(_to_array(msg, np.uint8, 1), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 RGB encoding: {msg.encoding}")


def depth_msg_to_mm(msg: Image) -> np.ndarray:
    """깊이 이미지를 uint16 mm 배열로 변환한다. NaN/inf/음수는 0(측정 불가)."""
    enc = msg.encoding.upper()
    if enc == "16UC1":  # RealSense 기본값: 이미 mm
        return _to_array(msg, np.uint16, 1)
    if enc == "32FC1":  # Gazebo 기본값: m
        meters = _to_array(msg, np.float32, 1)
        mm = np.nan_to_num(meters * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
        return np.clip(mm, 0, 65535).astype(np.uint16)
    raise ValueError(f"지원하지 않는 depth encoding: {msg.encoding}")


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# --------------------------------------------------------------------------- #
class RecorderNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("video_recorder")
        self.args = args
        self.use_depth = not args.no_depth

        self.root = Path(args.out) / args.name
        if (self.root / "frames.jsonl").exists() and not args.overwrite:
            raise SystemExit(f"{self.root} 에 이미 녹화가 있습니다. --name 을 바꾸거나 --overwrite 를 쓰세요.")
        (self.root / "depth").mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_depth: Optional[Image] = None
        self.camera_info: Optional[dict] = None
        self.writer: Optional[cv2.VideoWriter] = None
        self.frames_file = None
        self.video_name = ""
        self.frame_count = 0
        self.next_stamp: Optional[float] = None
        self.last_stamp: Optional[float] = None
        self.first_stamp: Optional[float] = None
        self.size: Optional[tuple[int, int]] = None
        self.paused = False
        self.no_pose = 0
        self.skipped_sync = 0
        self.last_pose_error = ""
        self._first_seen: dict[str, float] = {}
        self._last_report: dict[str, float] = {}

        self.create_subscription(Image, args.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, args.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self._on_camera_info, qos_profile_sensor_data)

        self.get_logger().info(f"저장 위치: {self.root}")
        self.get_logger().info(
            f"rgb={args.rgb_topic}, depth={args.depth_topic if self.use_depth else '(비활성)'}, "
            f"info={args.camera_info_topic}, tf={args.odom_frame}->이미지 frame, fps={args.fps}"
        )

    # ---- subscribers -------------------------------------------------------
    def _on_depth(self, msg: Image) -> None:
        self.latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_info is not None:
            return
        self.camera_info = {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "distortion_model": msg.distortion_model,
            "K": list(msg.k),
            "D": list(msg.d),
            "P": list(msg.p),
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5],
        }

    def _on_rgb(self, msg: Image) -> None:
        if self.paused:
            return
        missing = []
        if self.camera_info is None:
            missing.append(self.args.camera_info_topic)
        if self.use_depth and self.latest_depth is None:
            missing.append(self.args.depth_topic)
        if missing:
            if self._should_report("topics", interval=3.0, grace=2.0):
                self.get_logger().error(
                    f"다음 topic 이 수신되지 않아 녹화를 시작하지 않습니다: {', '.join(missing)}  "
                    f"(ros2 topic list 로 확인하세요)"
                )
            return

        stamp = stamp_sec(msg)
        if self.last_stamp is not None and stamp < self.last_stamp - 1.0:
            self.next_stamp = None  # 시뮬레이션 시간이 되감긴 경우
        self.last_stamp = stamp
        period = 1.0 / self.args.fps
        if self.next_stamp is not None and stamp < self.next_stamp - 1e-6:
            return

        depth_msg = None
        depth_dt = None
        if self.use_depth:
            depth_msg = self.latest_depth
            depth_dt = stamp_sec(depth_msg) - stamp
            if abs(depth_dt) > self.args.max_sync_diff:
                self.skipped_sync += 1
                return

        try:
            bgr = rgb_msg_to_bgr(msg)
            depth_mm = depth_msg_to_mm(depth_msg) if depth_msg is not None else None
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        # 자세를 구할 수 없으면 영상 파일을 만들기 전에 멈춘다 (녹화 시작 조건).
        frame_id = msg.header.frame_id
        pose = self._lookup_pose(frame_id, msg)
        if pose is None:
            self._handle_missing_pose(frame_id)
            return
        self._first_seen.pop("pose", None)

        if self.writer is None and not self._open_writer(bgr):
            raise SystemExit(1)
        if (bgr.shape[1], bgr.shape[0]) != self.size:
            self.get_logger().error("해상도가 중간에 바뀌었습니다. 녹화를 중단합니다.")
            raise SystemExit(1)

        self.writer.write(bgr)
        depth_rel = None
        if depth_mm is not None:
            depth_rel = f"depth/{self.frame_count:06d}.png"
            cv2.imwrite(str(self.root / depth_rel), depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        self.frames_file.write(
            json.dumps(
                {
                    "frame": self.frame_count,
                    "stamp": stamp,
                    "depth": depth_rel,
                    "depth_dt": depth_dt,
                    "pose": pose,
                }
            )
            + "\n"
        )
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.frame_count += 1
        self.next_stamp = stamp + period if self.next_stamp is None else max(self.next_stamp + period, stamp + 0.5 * period)
        if self.frame_count % (int(self.args.fps) * 5) == 0:
            self.get_logger().info(
                f"{self.frame_count} 프레임 ({self.frame_count / self.args.fps:.0f}s), 자세 없어 건너뜀 {self.no_pose}"
            )

    # ---- helpers -----------------------------------------------------------
    def _should_report(self, key: str, interval: float, grace: float = 0.0) -> bool:
        """같은 로그가 쏟아지지 않도록 key 별로 interval 초에 한 번만 True. grace 동안은 억제."""
        now = time.monotonic()
        first = self._first_seen.setdefault(key, now)
        if now - first < grace:
            return False
        if now - self._last_report.get(key, -1e9) < interval:
            return False
        self._last_report[key] = now
        return True

    def _lookup_pose(self, frame_id: str, msg: Image) -> Optional[dict]:
        """odom 기준 카메라 optical frame 의 자세. 영상 시각에 못 맞추면 최신 TF 로 대체."""
        for when in (Time.from_msg(msg.header.stamp), Time()):
            try:
                tr = self.tf_buffer.lookup_transform(self.args.odom_frame, frame_id, when).transform
            except TransformException as e:
                self.last_pose_error = str(e)
                continue
            return {
                "t": [tr.translation.x, tr.translation.y, tr.translation.z],
                "q": [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w],
            }
        return None

    def _describe_pose_failure(self, frame_id: str) -> str:
        """자세를 못 구하는 이유를 /tf topic 존재, frame 존재, 트리 연결 순으로 좁혀서 설명한다."""
        odom = self.args.odom_frame
        # 이 노드의 TransformListener 도 /tf 를 구독하므로 topic 목록이 아니라 "발행자 수"로 판단한다.
        publishers = [i for i in self.get_publishers_info_by_topic("/tf") if i.node_name != self.get_name()]
        if not publishers:
            return "자세 topic(/tf)이 없습니다. /tf 를 발행하는 노드가 없습니다 (Gazebo 로봇/robot_state_publisher 확인)."
        try:
            # 루트 프레임(예: odom)은 "자식" 줄에 나오지 않고 parent 로만 나오므로 둘 다 모은다.
            pairs = re.findall(r"^Frame (\S+) exists with parent (\S+?)\.?$", self.tf_buffer.all_frames_as_string(), re.M)
            frames = {name for pair in pairs for name in pair}
        except Exception:  # noqa: BLE001  진단용이므로 실패해도 녹화 로직에 영향 없음
            frames = set()
        known = ", ".join(sorted(frames)) or "(없음)"
        if odom not in frames:
            return f"/tf 에 '{odom}' 프레임이 없습니다. odom TF(diff drive)가 발행되는지 확인하세요. 수신된 프레임: {known}"
        if frame_id not in frames:
            return (
                f"/tf 에 카메라 프레임 '{frame_id}' 가 없습니다. robot_state_publisher / URDF 에 카메라 링크가 "
                f"있는지 확인하세요. 수신된 프레임: {known}"
            )
        return (
            f"'{odom}' 와 '{frame_id}' 는 /tf 에 있지만 서로 연결되어 있지 않습니다 (TF 트리가 끊김). "
            f"URDF 링크 이름(예: ${{namespace}} 자리표시자)과 diff drive 가 발행하는 프레임 이름이 같은지 확인하세요. "
            f"수신된 프레임: {known}"
        )

    def _handle_missing_pose(self, frame_id: str) -> None:
        self.no_pose += 1
        started = self.writer is not None
        if started:
            if self._should_report("pose_lost", interval=5.0):
                self.get_logger().warn(
                    f"카메라 자세를 잃어 해당 프레임을 저장하지 않습니다 (누적 {self.no_pose}). "
                    f"{self._describe_pose_failure(frame_id)}"
                )
            return
        if self._should_report("pose", interval=3.0, grace=2.0):
            self.get_logger().error(
                f"카메라 자세를 구할 수 없어 녹화를 시작하지 않습니다 ({self.args.odom_frame} -> {frame_id}). "
                f"{self._describe_pose_failure(frame_id)}"
            )
        waited = time.monotonic() - self._first_seen.setdefault("pose", time.monotonic())
        if self.args.pose_timeout > 0 and waited > self.args.pose_timeout:
            self.get_logger().error(f"{self.args.pose_timeout:.0f}초 안에 자세를 구하지 못해 종료합니다. 녹화된 프레임 없음.")
            raise SystemExit(1)

    def _open_writer(self, bgr: np.ndarray) -> bool:
        h, w = bgr.shape[:2]
        self.size = (w, h)
        candidates = [(self.args.codec, "mp4" if self.args.codec == "mp4v" else "avi"), ("MJPG", "avi")]
        for codec, ext in candidates:
            path = self.root / f"rgb.{ext}"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), self.args.fps, (w, h))
            if writer.isOpened():
                self.writer, self.video_name = writer, path.name
                break
        else:
            self.get_logger().error("VideoWriter 를 열 수 없습니다. (--codec 확인)")
            return False
        self.frames_file = (self.root / "frames.jsonl").open("w")
        self.get_logger().info(f"녹화 시작: {self.video_name} {w}x{h} @ {self.args.fps}fps")
        self._write_meta(final=False)
        return True

    def _write_meta(self, final: bool) -> None:
        meta = {
            "version": 1,
            "created": datetime.now().isoformat(timespec="seconds"),
            "video": self.video_name,
            "width": self.size[0],
            "height": self.size[1],
            "fps": self.args.fps,
            "rgb_topic": self.args.rgb_topic,
            "depth_topic": self.args.depth_topic if self.use_depth else None,
            "camera_info_topic": self.args.camera_info_topic,
            "odom_frame": self.args.odom_frame,
            "camera_frame": self.camera_info["frame_id"] if self.camera_info else None,
            "camera_info": self.camera_info,
            "depth_scale": DEPTH_SCALE,
            "depth_unit": "uint16 PNG, 1 unit = 1 mm, 0 = invalid",
            "odom_in_world": ODOM_IN_WORLD,
            "frames": self.frame_count,
            "frames_skipped_no_pose": self.no_pose,
            "complete": final,
        }
        (self.root / "meta.json").write_text(json.dumps(meta, indent=2))

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        if not self.paused:
            self.next_stamp = None
        self.get_logger().info("일시정지" if self.paused else "재개")

    def close(self) -> None:
        if self.writer is None:  # 녹화를 시작하지 못했으면 빈 폴더를 남기지 않는다.
            for d in (self.root / "depth", self.root):
                try:
                    d.rmdir()
                except OSError:
                    pass
        if self.writer is not None:
            self.writer.release()
            self.frames_file.close()
            self._write_meta(final=True)
        self.get_logger().info(
            f"종료. {self.frame_count} 프레임 저장 (자세 없어 건너뜀 {self.no_pose}, 동기화 실패로 건너뜀 {self.skipped_sync})"
        )


# --------------------------------------------------------------------------- #
def read_key() -> Optional[str]:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RGB-D 주행 영상 녹화 (ROS2)")
    p.add_argument("--name", default=datetime.now().strftime("rec_%Y%m%d_%H%M%S"), help="녹화 이름 (기본: 현재 시각)")
    p.add_argument("--out", default=str(DEFAULT_OUT), help="저장 루트 (기본: training/datasets/recordings)")
    p.add_argument("--fps", type=float, default=10.0, help="저장 프레임 레이트 (기본 10)")
    p.add_argument("--codec", default="mp4v", help="fourcc 코드 (기본 mp4v, 실패 시 MJPG)")
    p.add_argument("--rgb-topic", default="/camera/image_raw")
    p.add_argument("--depth-topic", default="/camera/depth/image_raw")
    p.add_argument("--camera-info-topic", default="/camera/depth/camera_info")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--pose-timeout", type=float, default=20.0,
                   help="카메라 자세를 구하지 못한 채 이 시간(초)이 지나면 종료 (기본 20, 0 이면 계속 대기)")
    p.add_argument("--no-depth", action="store_true", help="depth 를 저장하지 않음 (가려짐 판정 불가)")
    p.add_argument("--max-sync-diff", type=float, default=0.1, help="rgb-depth 허용 시간차(초)")
    p.add_argument("--overwrite", action="store_true", help="같은 이름의 녹화를 덮어쓴다")
    args, _ = p.parse_known_args()
    if args.fps <= 0:
        p.error("--fps 는 0보다 커야 합니다.")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RecorderNode(args)

    interactive = sys.stdin.isatty()
    old_term = None
    if interactive:
        old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info("Space: 일시정지/재개, q: 종료")
    else:
        node.get_logger().warn("TTY 가 아니라 키 입력을 받을 수 없습니다. Ctrl-C 로 종료하세요.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if interactive:
                key = read_key()
                if key == " ":
                    node.toggle_pause()
                elif key in ("q", "Q", "\x03"):
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if old_term is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
