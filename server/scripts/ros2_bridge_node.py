#!/usr/bin/env python3
"""ROS2 <-> FastAPI 서버 브릿지 노드.

로봇(또는 로봇과 같은 DDS 망에 있는 PC)에서 실행하며, ROS2 topic/TF로 들어오는
정보를 서버가 요구하는 형식(AGENTS.md 3장)으로 변환해 전달한다. 반대로 서버가
WebSocket으로 내려보내는 명령(stop / return_to_dock / ...)은 ROS2 서비스와
Nav2 Action으로 바꿔 실행한다.

ROS2 -> 서버
    TF  map -> base_footprint        -> telemetry.current_position / swept_path
    /coverage_status (String JSON)   -> telemetry.current_state (FSM 매핑)
    /battery_state (BatteryState)    -> telemetry.battery_level
        위 셋을 합쳐 ``ws://<server>/ws/robot`` 으로 telemetry 전송 (기본 2Hz)
    /explore_map (OccupancyGrid)     -> PNG 렌더링 후 ``POST /api/map``
        해상도/크기/원점(info.origin)/frame_id/시작 위치를 같은 요청에 실어 보낸다.
    /coverage_visited_map            -> 위 PNG 위에 청소 완료 영역을 초록색으로 덧칠

좌표 기준
    모든 좌표(위치, 궤적, 장애물, 목표)는 /explore_map 의 header.frame_id 좌표계 기준이다.
    /explore_map 은 map_threshold_node.py 가 /map 의 header/info 를 그대로 복사하고 값만
    -1/0/100으로 이진화한 지도라서 /map 과 격자 기하(frame, 원점, 해상도, 크기)가 같다.
    시작 위치(start_pose)는 TF가 처음 잡힌 위치(또는 dock_x/dock_y 파라미터)이며, 앱이 이
    점을 화면 기준점으로 쓴다. 로봇이 움직이기 전에 이 노드를 먼저 실행해야 한다.

서버 -> ROS2 (``/ws/robot`` 으로 수신하는 command 메시지)
    stop             -> <cleaner>/pause (Trigger) + 진행 중인 Nav2 goal 취소
    start_cleaning   -> coverage_cleaner 가 없으면 저장소 최상위에서 프로세스로 실행하고,
                        이미 떠 있으면 <cleaner>/resume (Trigger)
    start_mapping    -> 상태만 MAPPING으로 표시 (탐색 노드는 별도 실행)
    return_to_dock   -> pause + Nav2 NavigateToPose(dock)   [RETURNING]
    target_reclean   -> pause + Nav2 NavigateToPose(target) [TARGET_RECLEANING]

아직 포함되지 않은 것
    YOLO 장애물 감지 -> ``POST /api/obstacles`` (감지 topic이 정의되면 추가).

사용법 (ROS2 환경을 source 한 뒤)
    python3 server/scripts/ros2_bridge_node.py --ros-args \
        -p server_host:=192.168.0.10 -p server_port:=8000 \
        -p map_topic:=/explore_map -p use_sim_time:=true

의존성: rclpy, nav2_msgs, tf2_ros, numpy (ROS2 humble 설치에 포함) + websockets.
HTTP 업로드는 표준 라이브러리(urllib)만 사용한다.
"""

from __future__ import annotations

import asyncio
import json
import math
import queue
import struct
import subprocess
import sys
import threading
import time
import urllib.request
import uuid
import zlib
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import websockets
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# coverage_cleaner 의 state 문자열 -> 서버 FSM 상태 (schemas.RobotState)
_IDLE_CLEANER_STATES = {
    "waiting_for_map",
    "paused",
    "blocked",
    "complete_pass",
    "complete_reachable",
}
_OCCUPIED_THRESHOLD = 65

# 이 파일은 <repo>/server/scripts/ 에 있다. 청소 시작은 저장소 최상위에서 실행한다.
REPO_ROOT = Path(__file__).resolve().parents[2]
# README.md 의 실행 예시와 같은 옵션 (use_sim_time 은 이 노드의 값을 따라간다).
DEFAULT_CLEANER_ARGS = [
    "tf_timeout_sec:=1.5",
    "tracker_max_gap_sec:=1.5",
    "configure_goal_checker:=false",
    "check_endpoint:=false",
    "check_safe_position:=false",
    "reclean_unvisited:=false",
]


# ---------------------------------------------------------------------------
# 지도 -> PNG (외부 이미지 라이브러리 없이 numpy + zlib 만 사용)
# ---------------------------------------------------------------------------
def _png_chunk(tag: bytes, data: bytes) -> bytes:
    body = tag + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def encode_png_rgb(rgb: np.ndarray) -> bytes:
    """(H, W, 3) uint8 배열을 PNG 바이트로 인코딩."""
    height, width, _ = rgb.shape
    raw = np.zeros((height, width * 3 + 1), dtype=np.uint8)  # 각 행 맨 앞 filter byte = 0
    raw[:, 1:] = rgb.reshape(height, width * 3)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(raw.tobytes(), 6))
        + _png_chunk(b"IEND", b"")
    )


def render_map_png(map_msg: OccupancyGrid, visited_msg: Optional[OccupancyGrid]) -> bytes:
    """OccupancyGrid(+방문 영역)를 앱이 배경으로 쓸 PNG로 변환.

    이미지 위쪽이 map +y 방향이 되도록 상하 반전한다 (OccupancyGrid의 0행은 y 최소값).
    """
    h, w = map_msg.info.height, map_msg.info.width
    grid = np.asarray(map_msg.data, dtype=np.int16).reshape(h, w)
    rgb = np.empty((h, w, 3), dtype=np.uint8)
    rgb[:] = (205, 205, 205)  # unknown
    free = (grid >= 0) & (grid < _OCCUPIED_THRESHOLD)
    rgb[free] = (255, 255, 255)
    rgb[grid >= _OCCUPIED_THRESHOLD] = (0, 0, 0)

    if visited_msg is not None and (visited_msg.info.height, visited_msg.info.width) == (h, w):
        visited = np.asarray(visited_msg.data, dtype=np.int16).reshape(h, w) > 0
        rgb[visited & free] = (140, 215, 140)

    return encode_png_rgb(rgb[::-1])


def post_map_png(base_url: str, png: bytes, fields: dict) -> str:
    """``POST /api/map`` (multipart: 이미지 ``image`` + 메타데이터 form 필드). 서버가 돌려준 URL 반환."""
    boundary = uuid.uuid4().hex
    parts = []
    for name, value in fields.items():
        parts.append(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()
        )
    parts.append(
        (
            f"--{boundary}\r\n"
            'Content-Disposition: form-data; name="image"; filename="latest_map.png"\r\n'
            "Content-Type: image/png\r\n\r\n"
        ).encode()
        + png
        + f"\r\n--{boundary}--\r\n".encode()
    )
    request = urllib.request.Request(
        f"{base_url}/api/map",
        data=b"".join(parts),
        method="POST",
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
    )
    with urllib.request.urlopen(request, timeout=10) as response:  # noqa: S310 - 로컬망 서버
        return json.loads(response.read().decode()).get("map_image_url", "")


def map_upload_fields(map_msg: OccupancyGrid, start: Optional[tuple[float, float, float]]) -> dict:
    """OccupancyGrid.info 를 서버 ``POST /api/map`` 의 form 필드로 변환."""
    info = map_msg.info
    fields = {
        "frame_id": map_msg.header.frame_id or "map",
        "resolution": info.resolution,
        "width": info.width,
        "height": info.height,
        "origin_x": info.origin.position.x,
        "origin_y": info.origin.position.y,
    }
    if start is not None:
        fields.update(start_x=start[0], start_y=start[1], start_theta=start[2])
    return fields


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------
class ServerBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("server_bridge")

        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 8000)
        self.declare_parameter("map_topic", "/explore_map")
        self.declare_parameter("visited_topic", "/coverage_visited_map")
        self.declare_parameter("status_topic", "/coverage_status")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("telemetry_rate_hz", 2.0)
        self.declare_parameter("map_upload_period_sec", 5.0)
        self.declare_parameter("path_min_step_m", 0.05)
        self.declare_parameter("path_max_points", 500)
        self.declare_parameter("cleaner_node", "/coverage_cleaner")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        # NaN이면 TF가 처음 잡힌 위치(=시작 위치)를 도크로 사용한다.
        self.declare_parameter("dock_x", float("nan"))
        self.declare_parameter("dock_y", float("nan"))
        self.declare_parameter("dock_yaw", 0.0)
        # '청소 시작' 시 실행할 coverage_cleaner (저장소 최상위 기준 경로)
        self.declare_parameter("cleaner_script", "robot_ws/scripts/coverage_cleaner.py")
        self.declare_parameter("cleaner_args", DEFAULT_CLEANER_ARGS)

        p = self.get_parameter
        self.server_host = str(p("server_host").value)
        self.server_port = int(p("server_port").value)
        self.map_frame = str(p("map_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.telemetry_period = 1.0 / max(0.1, float(p("telemetry_rate_hz").value))
        self.map_upload_period = float(p("map_upload_period_sec").value)
        self.path_min_step = float(p("path_min_step_m").value)
        self.path_max_points = int(p("path_max_points").value)
        cleaner = str(p("cleaner_node").value).rstrip("/")
        self.cleaner_node = cleaner
        self.cleaner_script = str(p("cleaner_script").value)
        self.cleaner_args = [str(a) for a in p("cleaner_args").value]
        self.use_sim_time = bool(p("use_sim_time").value)
        self._cleaner_proc: Optional[subprocess.Popen] = None
        self.dock = [float(p("dock_x").value), float(p("dock_y").value), float(p("dock_yaw").value)]
        self._dock_auto = math.isnan(self.dock[0]) or math.isnan(self.dock[1])
        self._warned_origin_yaw = False

        # ROS 스레드와 서버 통신 스레드가 함께 쓰는 상태
        self._lock = threading.Lock()
        self._pose: Optional[tuple[float, float, float]] = None
        self._swept_path: list[tuple[float, float]] = []
        self._cleaner_state = "waiting_for_map"
        self._battery = 100
        self._state_override: Optional[str] = None  # 명령으로 강제된 FSM 상태
        self._map_msg: Optional[OccupancyGrid] = None
        self._visited_msg: Optional[OccupancyGrid] = None
        self._map_dirty = False
        self.commands: "queue.Queue[dict]" = queue.Queue()

        # --- 구독 ---------------------------------------------------------
        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        live = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )
        # 퍼블리셔가 transient-local 이든 volatile 이든 받을 수 있게 두 QoS로 모두 구독한다.
        for qos in (retained, live):
            self.create_subscription(OccupancyGrid, str(p("map_topic").value), self._on_map, qos)
            self.create_subscription(OccupancyGrid, str(p("visited_topic").value), self._on_visited, qos)
        self.create_subscription(String, str(p("status_topic").value), self._on_status, retained)
        self.create_subscription(BatteryState, str(p("battery_topic").value), self._on_battery, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self._update_pose)

        # --- 명령 실행용 클라이언트 -------------------------------------------
        self.pause_client = self.create_client(Trigger, f"{cleaner}/pause")
        self.resume_client = self.create_client(Trigger, f"{cleaner}/resume")
        self.nav_client = ActionClient(self, NavigateToPose, str(p("nav_action_name").value))
        self._nav_goal_handle = None
        self.create_timer(0.1, self._process_commands)

        self.get_logger().info(
            f"server_bridge 시작: server=http://{self.server_host}:{self.server_port}, "
            f"map={p('map_topic').value}, visited={p('visited_topic').value}"
        )

    # ------------------------------------------------------------------ 수신
    def _on_map(self, msg: OccupancyGrid) -> None:
        q = msg.info.origin.orientation
        origin_yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        with self._lock:
            frame = msg.header.frame_id or self.map_frame
            if frame != self.map_frame:
                # 좌표 기준은 /explore_map 의 frame 이다. 바뀌면 지금까지의 궤적/시작 위치는 무효.
                self.get_logger().info(f"좌표 기준 frame을 '{self.map_frame}' -> '{frame}' 로 변경 (/explore_map header)")
                self.map_frame = frame
                self._swept_path.clear()
                if self._dock_auto:
                    self.dock[0] = self.dock[1] = float("nan")
            if abs(origin_yaw) > 1e-3 and not self._warned_origin_yaw:
                self._warned_origin_yaw = True
                self.get_logger().warn("지도 원점에 회전이 있음: 앱 좌표 변환은 회전 없는 지도만 지원한다")
            self._map_msg = msg
            self._map_dirty = True

    def _on_visited(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._visited_msg = msg
            self._map_dirty = True

    def _on_status(self, msg: String) -> None:
        try:
            state = str(json.loads(msg.data).get("state", ""))
        except (ValueError, AttributeError):
            return
        with self._lock:
            self._cleaner_state = state

    def _on_battery(self, msg: BatteryState) -> None:
        pct = msg.percentage
        if math.isnan(pct):
            return
        level = pct * 100.0 if pct <= 1.0 else pct  # 0~1 또는 0~100 모두 허용
        with self._lock:
            self._battery = int(max(0, min(100, round(level))))

    def _update_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (t.x, t.y, yaw)
            if math.isnan(self.dock[0]) or math.isnan(self.dock[1]):
                self.dock[0], self.dock[1], self.dock[2] = t.x, t.y, yaw
                self.get_logger().info(f"도크 위치를 시작 위치로 설정: ({t.x:.2f}, {t.y:.2f})")
            last = self._swept_path[-1] if self._swept_path else None
            if last is None or math.hypot(t.x - last[0], t.y - last[1]) >= self.path_min_step:
                self._swept_path.append((t.x, t.y))
                if len(self._swept_path) > self.path_max_points:
                    # 오래된 구간은 절반으로 솎아내 전체 모양을 유지하면서 길이를 제한한다.
                    self._swept_path = self._swept_path[::2]

    # ------------------------------------------------- 서버 통신 스레드용 API
    def _fsm_state(self) -> str:
        if self._state_override is not None:
            return self._state_override
        return "IDLE" if self._cleaner_state in _IDLE_CLEANER_STATES else "CLEANING"

    def snapshot_telemetry(self) -> Optional[dict]:
        """TelemetryUpdate 스키마(server/app/schemas.py)에 맞는 dict. 위치를 모르면 None."""
        with self._lock:
            if self._pose is None:
                return None
            x, y, theta = self._pose
            return {
                "type": "telemetry",
                "current_state": self._fsm_state(),
                "current_position": {"x": x, "y": y, "theta": theta},
                "swept_path": [{"x": px, "y": py} for px, py in self._swept_path],
                "battery_level": self._battery,
            }

    def pop_map_upload(self) -> Optional[tuple[bytes, dict]]:
        """새 지도가 있으면 (PNG, 서버 form 필드)를 만들어 반환하고 dirty 플래그를 내린다."""
        with self._lock:
            if self._map_msg is None or not self._map_dirty:
                return None
            map_msg, visited_msg = self._map_msg, self._visited_msg
            start = None if math.isnan(self.dock[0]) or math.isnan(self.dock[1]) else tuple(self.dock)
            self._map_dirty = False
        return render_map_png(map_msg, visited_msg), map_upload_fields(map_msg, start)

    # ------------------------------------------------------------- 명령 실행
    def _set_override(self, state: Optional[str]) -> None:
        with self._lock:
            self._state_override = state

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f"{label} 서비스가 준비되지 않음 (coverage_cleaner 실행 중인지 확인)")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(f"{label}: {getattr(f.result(), 'message', f.exception())}")
        )

    def _cancel_nav(self) -> None:
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def _send_nav_goal(self, x: float, y: float, yaw: float, on_success_state: Optional[str]) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error("Nav2 NavigateToPose 서버를 찾을 수 없음")
            self._set_override(None)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        def on_goal(future) -> None:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn("Nav2가 목표를 거부함")
                self._set_override(None)
                return
            self._nav_goal_handle = handle
            handle.get_result_async().add_done_callback(on_result)

        def on_result(future) -> None:
            status = future.result().status
            self._nav_goal_handle = None
            if status == GoalStatus.STATUS_CANCELED:
                return  # 새 명령이 override를 이미 갱신했다.
            self.get_logger().info(f"Nav2 결과 status={status}")
            self._set_override(on_success_state)

        self.nav_client.send_goal_async(goal).add_done_callback(on_goal)

    def _start_cleaning(self) -> None:
        """coverage_cleaner 가 떠 있으면 resume, 없으면 저장소 최상위에서 실행한다."""
        node_name = self.cleaner_node.rsplit("/", 1)[-1]
        if self.resume_client.service_is_ready() or node_name in self.get_node_names():
            self._call_trigger(self.resume_client, "resume")  # 이미 실행 중(직접 켠 경우 포함)
            return
        if self._cleaner_proc is not None and self._cleaner_proc.poll() is None:
            self.get_logger().info("coverage_cleaner 가 시작되는 중입니다")
            return

        script = REPO_ROOT / self.cleaner_script
        if not script.is_file():
            self.get_logger().error(f"coverage_cleaner 스크립트를 찾을 수 없음: {script}")
            return
        command = [sys.executable, f"./{self.cleaner_script}", "--ros-args",
                   "-p", f"use_sim_time:={'true' if self.use_sim_time else 'false'}"]
        for arg in self.cleaner_args:
            command += ["-p", arg]
        self.get_logger().info(f"coverage_cleaner 실행 (cwd={REPO_ROOT}): {' '.join(command)}")
        try:
            self._cleaner_proc = subprocess.Popen(command, cwd=REPO_ROOT)  # noqa: S603 - 고정된 로컬 스크립트
        except OSError as exc:
            self.get_logger().error(f"coverage_cleaner 실행 실패: {exc}")
            return
        # 노드가 첫 상태를 내보내기 전에도 앱에는 청소 중으로 보이게 한다.
        with self._lock:
            self._cleaner_state = "starting"

    def _check_cleaner_process(self) -> None:
        """실행한 coverage_cleaner 가 곧바로 죽었다면 '청소 중' 표시를 풀고 원인을 알린다."""
        proc = self._cleaner_proc
        if proc is None or proc.poll() is None:
            return
        self._cleaner_proc = None
        self.get_logger().error(
            f"coverage_cleaner 프로세스가 종료됨 (exit code {proc.returncode}). "
            "위 출력(scipy 등 의존성, 인자 오류)을 확인하세요."
        )
        with self._lock:
            if self._cleaner_state == "starting":
                self._cleaner_state = "waiting_for_map"

    def stop_cleaner_process(self) -> None:
        """브릿지가 실행한 coverage_cleaner 를 브릿지 종료 시 함께 종료한다."""
        proc = self._cleaner_proc
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _process_commands(self) -> None:
        self._check_cleaner_process()
        while True:
            try:
                msg = self.commands.get_nowait()
            except queue.Empty:
                return
            command = msg.get("command")
            target = msg.get("target") or {}
            self.get_logger().info(f"서버 명령 수신: {command} {target or ''}")

            if command == "stop":
                self._cancel_nav()
                self._call_trigger(self.pause_client, "pause")
                self._set_override("IDLE")
            elif command == "start_cleaning":
                self._cancel_nav()
                self._set_override(None)
                self._start_cleaning()
            elif command == "start_mapping":
                self._set_override("MAPPING")
                self.get_logger().info("MAPPING 상태로 표시만 함 (탐색 노드는 별도 실행 필요)")
            elif command == "return_to_dock":
                self._cancel_nav()
                self._call_trigger(self.pause_client, "pause")
                self._set_override("RETURNING")
                self._send_nav_goal(self.dock[0], self.dock[1], self.dock[2], on_success_state="IDLE")
            elif command == "target_reclean":
                if "x" not in target or "y" not in target:
                    self.get_logger().warn("target_reclean 에 target 좌표가 없음")
                    continue
                self._cancel_nav()
                self._call_trigger(self.pause_client, "pause")
                self._set_override("TARGET_RECLEANING")
                # 도착 후에는 cleaner가 paused 이므로 IDLE로 보이게 override를 해제한다.
                self._send_nav_goal(float(target["x"]), float(target["y"]), 0.0, on_success_state=None)
            else:
                self.get_logger().warn(f"알 수 없는 명령: {command}")


# ---------------------------------------------------------------------------
# 서버 통신 (별도 스레드의 asyncio 루프)
# ---------------------------------------------------------------------------
class ServerLink(threading.Thread):
    def __init__(self, node: ServerBridgeNode) -> None:
        super().__init__(daemon=True, name="server-link")
        self.node = node
        self.base_url = f"http://{node.server_host}:{node.server_port}"
        self.ws_url = f"ws://{node.server_host}:{node.server_port}/ws/robot"

    def run(self) -> None:
        asyncio.run(self._main())

    async def _main(self) -> None:
        log = self.node.get_logger()
        upload_task = asyncio.create_task(self._map_upload_loop())
        try:
            while True:
                try:
                    log.info(f"서버 연결 시도: {self.ws_url}")
                    async with websockets.connect(self.ws_url) as ws:
                        log.info("서버 WebSocket 연결됨")
                        await asyncio.gather(self._send_loop(ws), self._receive_loop(ws))
                except Exception as exc:  # noqa: BLE001 - 끊기면 계속 재시도한다.
                    log.warn(f"서버 연결 끊김: {exc!r}; 3초 후 재시도")
                    await asyncio.sleep(3.0)
        finally:
            upload_task.cancel()

    async def _send_loop(self, ws) -> None:
        while True:
            telemetry = self.node.snapshot_telemetry()
            if telemetry is not None:
                await ws.send(json.dumps(telemetry))
            await asyncio.sleep(self.node.telemetry_period)

    async def _receive_loop(self, ws) -> None:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if msg.get("type") == "command":
                self.node.commands.put(msg)

    async def _map_upload_loop(self) -> None:
        loop = asyncio.get_running_loop()
        log = self.node.get_logger()
        while True:
            await asyncio.sleep(self.node.map_upload_period)
            try:
                upload = await loop.run_in_executor(None, self.node.pop_map_upload)
                if upload is not None:
                    png, fields = upload
                    url = await loop.run_in_executor(None, post_map_png, self.base_url, png, fields)
                    log.info(f"지도 업로드 완료: {url} ({len(png) // 1024} KiB)")
            except Exception as exc:  # noqa: BLE001
                log.warn(f"지도 업로드 실패: {exc!r}")
                with self.node._lock:  # 다음 주기에 다시 시도
                    self.node._map_dirty = True


def main() -> None:
    rclpy.init()
    node = ServerBridgeNode()
    ServerLink(node).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_cleaner_process()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
