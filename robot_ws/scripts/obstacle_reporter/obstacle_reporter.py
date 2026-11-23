#!/usr/bin/env python3
"""카메라에 장애물이 확실히 보이면 서버에 "어떤 장애물인지 + 사진"을 보고하는 노드.

흐름
    /camera/image_raw  --YOLO-->  같은 class 가 연속 N 프레임 "확실히" 보임
        --> 보고용 사진(장애물에 bounding box, 왼쪽 위에 class 이름) 을 만들어
        --> 서버 ``POST /api/obstacles`` (type=class, x, y, image)

서버는 이 사진을 ``<시간>_<class>.jpg`` 로 저장하고 앱에 알린다. 앱은 지도의 장애물 위치에 표시를
하고, 표시를 누르면 장애물 종류와 사진을 보여 준다.

사진은 장애물 하나에 한 장이다. 같은 class 는 한 번의 청소 안에서 한 번만 찍는다.
    * 사진을 찍기(=인코딩/전송) 전에 서버에 "이미 이 class 가 표시되어 있는지" 묻고, 있으면 찍지 않는다.
    * 새 청소를 시작하면 서버가 표시를 모두 내리므로 그때부터 다시 한 번 찍을 수 있다.
      (이 노드는 같은 장애물이 계속 보이면 ``repeat_sec`` 마다 서버에 확인만 한다.)
    * 서버도 같은 class 의 중복 등록을 막는다.

"찍는 조건" (모두 만족해야 한다) — 장애물 전체가 화면 가운데쯤에 보이면 보고한다
    * 신뢰도 >= conf (기본 0.3)
    * 상자가 화면 가장자리에서 border_px(기본 100px, 640x480 기준) 이상 안쪽에 있음 = 잘리지 않고 대충 화면 가운데에 옴
    * 상자 면적이 화면의 min_area_ratio 이상 (기본 0.03 = 640x480 에서 대략 96x96px, 0 = 확인 안 함)
    * 로봇(base_frame)과 장애물 중심 사이의 거리 <= max_distance (기본 0 = 확인 안 함)
    * 위 조건이 연속 min_frames 번의 추론에서 계속 만족 (기본 1)
    엄격하게 되돌리려면: -p conf:=0.6 -p min_frames:=3 -p min_area_ratio:=0.06 -p max_distance:=0.25

장애물 위치(x, y)
    카메라로 위치를 추정하지 않는다. coverage_cleaner 가 /coverage_status 의 ``obstacle_zones`` 로 알려 주는
    "스폰 위치 기준 map 좌표"(class 이름이 같다)를 쓴다. 로봇 위치는 TF(map_frame <- base_frame)이다.
    그래서 coverage_cleaner 와 TF 가 있어야 보고할 수 있다.

실행 (ROS 와 학습용 가상환경의 ultralytics 가 모두 필요하다)
    source /opt/ros/humble/setup.bash
    training/.venv/bin/python robot_ws/scripts/obstacle_reporter/obstacle_reporter.py \
        --ros-args -p use_sim_time:=true -p server_host:=127.0.0.1

파라미터는 declare_parameter 목록과 --ros-args -p 이름:=값 으로 바꾼다.
"""
from __future__ import annotations

import json
import queue
import sys
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reporter_logic import (Confirmer, Detection, annotate, encode_jpeg, encode_multipart,  # noqa: E402
                            image_to_bgr, is_clearly_visible, parse_zones, within_distance)

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WEIGHTS = str(REPO_ROOT / 'training' / 'weights' / 'obstacles_v1_best.pt')


class ObstacleReporter(Node):
    def __init__(self):
        super().__init__('obstacle_reporter')
        for name, value in dict(
            weights=DEFAULT_WEIGHTS,
            image_topic='/camera/image_raw',
            status_topic='/coverage_status',
            server_host='127.0.0.1',
            server_port=8000,
            conf=0.3,                # 이 신뢰도 이상의 상자면 발견으로 본다 (엄격하게: 0.6)
            min_frames=1,            # 연속으로 보여야 하는 추론 횟수 (엄격하게: 3)
            border_px=100.0,         # 상자가 화면 가장자리에서 이만큼(px) 떨어져 있어야 한다 = 잘리지 않고 대충 화면 가운데에 올 때만 (640x480 기준)
            min_area_ratio=0.0,      # 상자 면적 / 화면 면적 최소값. 0 이면 제한 없음 (원하면 0.03 = 640x480 에서 약 96x96px)
            infer_hz=5.0,            # 추론 주기(카메라가 더 빨라도 이 주기로만 본다)
            max_distance=0.0,        # 로봇-장애물 중심 거리(m)가 이 이하일 때만 찍는다. 0 이하면 거리 조건 없음 (엄격하게: 0.25)
            map_frame='map',         # 장애물 위치(obstacle_zones)가 속한 frame
            base_frame='base_footprint',   # 로봇 위치 기준 frame
            repeat_sec=10.0,         # 이미 찍은 class 를 서버에 다시 확인하는 최소 간격
            retry_sec=3.0,           # 서버 통신 실패 후 재시도 간격
            jpeg_quality=90,
            device='',               # '' 이면 ultralytics 가 자동 선택(GPU 우선)
        ).items():
            self.declare_parameter(name, value)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf, self.border_px = float(p('conf')), float(p('border_px'))
        self.min_area_ratio, self.max_distance = float(p('min_area_ratio')), float(p('max_distance'))
        self.map_frame, self.base_frame = str(p('map_frame')), str(p('base_frame'))
        self.infer_period = 1.0 / max(0.1, float(p('infer_hz')))
        self.repeat_sec, self.retry_sec = float(p('repeat_sec')), float(p('retry_sec'))
        self.jpeg_quality = int(p('jpeg_quality'))
        self.url = f"http://{p('server_host')}:{int(p('server_port'))}/api/obstacles"
        self.device = str(p('device')) or None

        from ultralytics import YOLO  # 가상환경에만 설치되어 있어 지연 import
        weights = str(p('weights'))
        if not Path(weights).is_file():
            raise SystemExit(f'weights 파일이 없습니다: {weights}')
        self.model = YOLO(weights)
        self.confirmer = Confirmer(int(p('min_frames')))
        self.zones: dict[str, tuple[float, float]] = {}
        self.last_infer = 0.0
        self.next_report: dict[str, float] = {}
        self.pending: set[str] = set()
        self.uploads: 'queue.Queue[tuple[str, float, float, object]]' = queue.Queue()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        threading.Thread(target=self._upload_loop, daemon=True, name='upload').start()

        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, str(p('status_topic')), self._on_status, retained)
        self.create_subscription(Image, str(p('image_topic')), self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"obstacle_reporter 시작: image={p('image_topic')}, server={self.url}, classes={self.model.names}, "
            f"conf>={self.conf}, {self.confirmer.min_frames}프레임 연속, "
            + (f"거리<={self.max_distance}m ({self.map_frame}<-{self.base_frame})" if self.max_distance > 0
               else "거리 조건 없음") + f", repeat={self.repeat_sec}s")

    def _on_status(self, msg):
        zones = parse_zones(msg.data)
        if zones != self.zones:
            self.zones = zones
            self.get_logger().info('장애물 위치(coverage_cleaner): ' + ', '.join(
                f'{c}=({x:.2f}, {y:.2f})' for c, (x, y) in zones.items()))

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self.last_infer < self.infer_period:
            return
        self.last_infer = now
        try:
            frame = image_to_bgr(bytes(msg.data), msg.encoding, msg.width, msg.height, msg.step)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return
        h, w = frame.shape[:2]
        result = self.model.predict(frame, conf=self.conf, verbose=False, device=self.device)[0]
        robot = self._robot_xy() if len(result.boxes) and self.max_distance > 0 else None
        visible = []
        for box in result.boxes:
            det = Detection(self.model.names[int(box.cls)], float(box.conf), *map(float, box.xyxy[0]))
            if (is_clearly_visible(det, w, h, self.conf, self.border_px, self.min_area_ratio)
                    and self._close_enough(det.cls, robot)):
                visible.append(det)
        for cls, det in self.confirmer.update(visible).items():
            self._report(cls, det, frame, now)

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform.translation
            return t.x, t.y
        except Exception as exc:  # noqa: BLE001 - TF 가 아직 없음
            self.get_logger().warn(f'로봇 위치 TF({self.map_frame}<-{self.base_frame})를 못 구함: {exc}',
                                   throttle_duration_sec=5.0)
            return None

    def _close_enough(self, cls, robot):
        """로봇이 장애물 중심에서 max_distance 이내인가(max_distance<=0 이면 거리는 보지 않는다).
        앱 지도에 표시할 장애물 위치를 모르면 False."""
        if cls not in self.zones:
            self.get_logger().warn(
                f'{cls} 가 보이지만 장애물 위치를 모릅니다: coverage_cleaner 가 '
                f'{self.get_parameter("status_topic").value} 로 obstacle_zones 를 알려 줘야 합니다',
                throttle_duration_sec=5.0)
            return False
        if self.max_distance <= 0:
            return True
        return robot is not None and within_distance(robot, self.zones[cls], self.max_distance)

    def _report(self, cls, det, frame, now):
        if now < self.next_report.get(cls, 0.0) or cls in self.pending:
            return
        x, y = self.zones[cls]
        # 사진은 이 순간의 화면이다. 서버에 이미 있으면(같은 청소에서 이미 찍음) 인코딩/전송하지 않는다.
        self.pending.add(cls)
        self.uploads.put((cls, x, y, annotate(frame, cls, det)))
        self.get_logger().info(f'장애물 사진 조건 충족: {cls} conf={det.conf:.2f} map=({x:.2f}, {y:.2f})',
                               throttle_duration_sec=10.0)

    def _already_photographed(self, cls) -> bool:
        """서버의 표시 중인 장애물 중에 이 class 가 있으면 True (이번 청소에서 이미 찍음)."""
        with urllib.request.urlopen(self.url, timeout=10) as response:  # noqa: S310 - 로컬망 서버
            return any(o.get('type') == cls for o in json.loads(response.read().decode()))

    def _upload_loop(self):
        while True:
            cls, x, y, image = self.uploads.get()
            delay = self.retry_sec
            try:
                if self._already_photographed(cls):
                    self.get_logger().info(f'{cls}: 이번 청소에서 이미 사진을 찍음 - 다시 찍지 않음',
                                           throttle_duration_sec=30.0)
                    self.next_report[cls] = time.monotonic() + self.repeat_sec
                    self.pending.discard(cls)
                    continue
                jpeg = encode_jpeg(image, self.jpeg_quality)
                body, content_type = encode_multipart(dict(type=cls, x=x, y=y), f'{cls}.jpg', jpeg)
                request = urllib.request.Request(self.url, data=body, method='POST',
                                                 headers={'Content-Type': content_type})
                with urllib.request.urlopen(request, timeout=10) as response:  # noqa: S310 - 로컬망 서버
                    created = response.status == 201
                self.get_logger().info(
                    f'사진 전송: {cls} {"-> 새 표시 생성" if created else "(서버에 이미 표시 중, 변경 없음)"}')
                delay = self.repeat_sec
            except Exception as exc:  # noqa: BLE001 - 어떤 실패든 스레드가 죽지 않게 한다
                self.get_logger().warn(f'서버 통신 실패({cls}): {exc!r}; {self.retry_sec:.0f}초 후 재시도')
            self.next_report[cls] = time.monotonic() + delay
            self.pending.discard(cls)


def main():
    rclpy.init()
    node = ObstacleReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
