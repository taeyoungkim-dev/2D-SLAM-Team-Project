#!/usr/bin/env python3
"""Standalone coverage planner/tracker for ROS 2 Humble + Nav2 RPP.

기준: visual_robot_cleaner_project_source_updated.md (2026-10-21).
입력: /explore_map (OccupancyGrid); TF frame은 메시지 header.frame_id 사용.
실행: python3 coverage_cleaner.py --ros-args -p use_sim_time:=true
미리보기: 위 명령에 -p send_to_nav2:=false 추가.
알고리즘 자체 검사: python3 coverage_cleaner.py --self-test (ROS 불필요).
의존성: numpy, scipy; 실행 시 rclpy, tf2_ros, nav2_msgs, nav_msgs,
        geometry_msgs, action_msgs, std_msgs, std_srvs, rcl_interfaces.

동작 기준
  * 현재 위치에서 안전하게 도달 가능한 known-free 영역을 최소 1회 덮는다.
    문/통로를 다시 지나는 것은 허용한다. 최단 경로/정확히 1회 방문 문제는 아니다.
  * 원형 footprint + clearance로 장애물/unknown/지도 밖을 팽창한다.
    coverage_width는 실제로 덮는 폭이며, 기본 0.15m는 설정값이지 실측값이 아니다.
  * 왕복 sweep + A* 연결 + 계획상 누락 셀 보충. 작은 구간도 버리지 않는다.
  * 계획한 경로가 아니라 실제 TF 이동으로 visited를 갱신한다. Nav2 성공만으로
    전체 완료를 선언하지 않는다. 실제 미방문 영역이 있으면 다시 계획한다.
  * 지도 원점이 정수 셀만큼 이동/확장하면 기록을 보존한다. 해상도/회전/frame
    변경 또는 비정수 셀 이동은 오기록을 피하기 위해 기록을 초기화한다.
  * 전역 위치 추정의 과거 궤적 자체가 바뀌는 SLAM loop closure는 visited를
    자동으로 보정할 수 없다. 이 경우 ~/reset 서비스를 사용한다.
  * FollowPath를 회전 지점마다 분리한다. 설정된 Nav2 goal checker의
    xy_goal_tolerance를 실행 시 최대 0.02m로 낮춘다(파일은 수정하지 않음).
    configure_goal_checker:=false이면 이 자동 설정을 끌 수 있다.
    다른 Nav2 목표 발행기(e-explore 포함)는 coverage 실행 전에 정지해야 한다.

출력
  /coverage_path              전체 계획 (Path, transient-local)
  /coverage_active_path       현재 FollowPath 구간 (Path)
  /coverage_visited_map        실제 방문한 free 셀
  /coverage_remaining_map      전체 known-free 중 미방문 셀 (접근 불가도 포함)
  /coverage_target_map         현재 도달 가능한 청소 대상 셀
  /coverage_unreachable_map    free이지만 현재 도달/청소 불가능한 미방문 셀
  /coverage_status            JSON String: 상태, 셀 수, 면적, 비율, 오류
지도 출력의 100은 해당 마스크를 뜻하며 장애물 costmap 입력으로 쓰지 않는다.

서비스 (노드 이름 기본 coverage_cleaner)
  ros2 service call /coverage_cleaner/pause std_srvs/srv/Trigger '{}'
  ros2 service call /coverage_cleaner/resume std_srvs/srv/Trigger '{}'
  ros2 service call /coverage_cleaner/reset std_srvs/srv/Trigger '{}'

주의: FollowPath 자체는 전역 재계획/BT recovery를 실행하지 않는다. 센서의
동적 장애물 회피는 Nav2 costmap/controller가 담당한다. 본 노드는 갱신 지도에서
경로가 막히면 취소 후 재계획하고, 반복 실패는 blocked 상태로 남긴다.
complete_reachable은 현재 known-free의 도달 가능 부분에 대한 판정이다.
미탐사 unknown/벽 주변의 청소 불가능한 띠/분리된 방까지 완료했다는 뜻이 아니다.
"""

from __future__ import annotations

import heapq
import json
import math
import sys
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label

Pixel = tuple[int, int]       # (row=y, column=x)
World = tuple[float, float]   # (x, y), meters
CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
NEIGHBORS = ((0, 1), (1, 0), (0, -1), (-1, 0),
             (1, 1), (1, -1), (-1, 1), (-1, -1))


class PlanningError(RuntimeError):
    pass


@dataclass(frozen=True)
class MapMeta:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    yaw: float = 0.0
    frame: str = 'map'

    def validate(self):
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0
                or not self.frame or not all(math.isfinite(v) for v in
                    (self.resolution, self.origin_x, self.origin_y, self.yaw))):
            raise ValueError('Invalid OccupancyGrid metadata')

    def local(self, x: float, y: float) -> World:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution

    def world(self, y: float, x: float) -> World:
        lx, ly = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly

    def pixel(self, x: float, y: float) -> Pixel:
        lx, ly = self.local(x, y)
        return math.floor(ly), math.floor(lx)


def inside(mask: np.ndarray, p: Pixel) -> bool:
    return 0 <= p[0] < mask.shape[0] and 0 <= p[1] < mask.shape[1] and bool(mask[p])


def disk(radius_cells: float) -> np.ndarray:
    """Use the real radius: rounding upward would claim unvisited floor as visited."""
    n = int(math.floor(radius_cells + 1e-10))
    yy, xx = np.ogrid[-n:n + 1, -n:n + 1]
    return xx * xx + yy * yy <= radius_cells * radius_cells + 1e-10


def stamp(mask: np.ndarray, p: Pixel, kernel: np.ndarray):
    r = kernel.shape[0] // 2
    y, x = p
    y0, y1 = max(0, y - r), min(mask.shape[0], y + r + 1)
    x0, x1 = max(0, x - r), min(mask.shape[1], x + r + 1)
    if y1 > y0 and x1 > x0:
        mask[y0:y1, x0:x1] |= kernel[y0 - y + r:y1 - y + r, x0 - x + r:x1 - x + r]


def traversable_map(free: np.ndarray, resolution: float, radius: float,
                    clearance: float) -> np.ndarray:
    """Conservative cell-centre clearance; map exterior is also occupied.

    Half a cell diagonal accounts for the occupied cell's area, instead of
    treating it as a zero-size point. Nav2 still validates the real footprint.
    """
    padded = np.pad(free, 1, constant_values=False)
    distance = distance_transform_edt(padded)[1:-1, 1:-1] * resolution
    return free & (distance > radius + clearance + resolution / math.sqrt(2.0))


def reachable_map(safe: np.ndarray, start: Pixel) -> np.ndarray:
    if not inside(safe, start):
        raise PlanningError('Robot is outside the safe map. Check TF, footprint and clearance.')
    labels, _ = label(safe, structure=CROSS)
    return labels == labels[start]


def line_is_safe(a: World, b: World, meta: MapMeta, safe: np.ndarray) -> bool:
    """Supercover traversal in continuous grid coordinates, including corner sides."""
    x0, y0 = meta.local(*a)
    x1, y1 = meta.local(*b)
    x, y, ex, ey = math.floor(x0), math.floor(y0), math.floor(x1), math.floor(y1)
    if not inside(safe, (y, x)) or not inside(safe, (ey, ex)):
        return False
    dx, dy = x1 - x0, y1 - y0
    sx = 1 if dx > 0 else -1 if dx < 0 else 0
    sy = 1 if dy > 0 else -1 if dy < 0 else 0
    # A line exactly along a grid boundary touches cells on both sides.
    boundary_x = sx == 0 and abs(x0 - round(x0)) < 1e-10
    boundary_y = sy == 0 and abs(y0 - round(y0)) < 1e-10

    def valid(cy, cx):
        return (inside(safe, (cy, cx))
                and (not boundary_x or inside(safe, (cy, cx - 1)))
                and (not boundary_y or inside(safe, (cy - 1, cx))))

    tx = ((x + 1 - x0) / dx if sx > 0 else (x - x0) / dx) if sx else math.inf
    ty = ((y + 1 - y0) / dy if sy > 0 else (y - y0) / dy) if sy else math.inf
    dtx, dty = (abs(1 / dx) if sx else math.inf), (abs(1 / dy) if sy else math.inf)
    if not valid(y, x):
        return False
    for _ in range(abs(ex - x) + abs(ey - y) + 2):
        if (x, y) == (ex, ey):
            return True
        if abs(tx - ty) < 1e-12:
            if not valid(y, x + sx) or not valid(y + sy, x):
                return False
            x, y, tx, ty = x + sx, y + sy, tx + dtx, ty + dty
        elif tx < ty:
            x, tx = x + sx, tx + dtx
        else:
            y, ty = y + sy, ty + dty
        if not valid(y, x):
            return False
    return False


def astar(safe: np.ndarray, start: Pixel, goal: Pixel, max_expansions: int = 500000) -> list[Pixel]:
    if not inside(safe, start) or not inside(safe, goal):
        raise PlanningError('A* endpoint outside traversable map')
    if start == goal:
        return [start]
    # Common sweep/transit case, without a heap search.
    if start[0] == goal[0] or start[1] == goal[1]:
        dy, dx = np.sign(goal[0] - start[0]), np.sign(goal[1] - start[1])
        path = [(start[0] + i * int(dy), start[1] + i * int(dx))
                for i in range(max(abs(goal[0] - start[0]), abs(goal[1] - start[1])) + 1)]
        if all(safe[p] for p in path):
            return path

    def h(p):
        dy, dx = abs(goal[0] - p[0]), abs(goal[1] - p[1])
        return max(dy, dx) + (math.sqrt(2) - 1) * min(dy, dx)

    heap = [(h(start), 0.0, start)]
    cost, parent, closed = {start: 0.0}, {}, set()
    while heap:
        _, g, p = heapq.heappop(heap)
        if p in closed:
            continue
        if p == goal:
            path = [p]
            while p in parent:
                p = parent[p]
                path.append(p)
            return path[::-1]
        closed.add(p)
        if len(closed) > max_expansions:
            raise PlanningError('A* search budget exceeded; no cell is silently skipped')
        for dy, dx in NEIGHBORS:
            q = p[0] + dy, p[1] + dx
            if not inside(safe, q) or q in closed:
                continue
            if dy and dx and (not safe[p[0] + dy, p[1]] or not safe[p[0], p[1] + dx]):
                continue
            ng = g + (math.sqrt(2) if dx and dy else 1.0)
            if ng < cost.get(q, math.inf):
                cost[q], parent[q] = ng, p
                heapq.heappush(heap, (ng + h(q), ng, q))
    raise PlanningError('No connected path; refusing to skip an unreachable waypoint')


def simplify(points: list[Pixel]) -> list[Pixel]:
    """Remove only forward-collinear points. A -> B -> A must retain B."""
    out = []
    for p in points:
        if out and p == out[-1]:
            continue
        while len(out) >= 2:
            a, b = out[-2:]
            u, v = (b[0] - a[0], b[1] - a[1]), (p[0] - b[0], p[1] - b[1])
            if u[0] * v[1] != u[1] * v[0] or u[0] * v[0] + u[1] * v[1] <= 0:
                break
            out.pop()
        out.append(p)
    return out


def intervals(row: np.ndarray):
    edges = np.diff(np.r_[False, row, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1))


@dataclass
class Plan:
    pixels: list[Pixel]
    target: np.ndarray
    predicted: np.ndarray
    reachable: np.ndarray


def plan_coverage(safe: np.ndarray, free: np.ndarray, meta: MapMeta, start: Pixel,
                  visited: np.ndarray, coverage_width: float = 0.15,
                  lane_spacing: float = 0.15, max_expansions: int = 500000) -> Plan:
    if min(coverage_width, lane_spacing, meta.resolution) <= 0:
        raise ValueError('coverage_width, lane_spacing and resolution must be positive')
    reachable = reachable_map(safe, start)
    kernel = disk(coverage_width / (2 * meta.resolution))
    target = free & binary_dilation(reachable, structure=kernel)
    predicted = visited.copy()
    route = [start]
    stamp(predicted, start, kernel)
    r = kernel.shape[0] // 2

    def append(goal):
        segment = astar(reachable, route[-1], goal, max_expansions)
        for p in segment[1:]:
            route.append(p)
            stamp(predicted, p, kernel)

    ys = np.flatnonzero(reachable.any(axis=1))
    step = max(1, int(math.floor(min(lane_spacing, coverage_width) / meta.resolution + 1e-9)))
    rows = list(range(int(ys[0]), int(ys[-1]) + 1, step))
    if rows[-1] != ys[-1]:
        rows.append(int(ys[-1]))
    if abs(start[0] - rows[-1]) < abs(start[0] - rows[0]):
        rows.reverse()
    left_to_right = start[1] <= np.flatnonzero(reachable[rows[0]]).mean()
    for y in rows:
        runs = intervals(reachable[y])
        if not left_to_right:
            runs.reverse()
        for lo, hi in runs:
            patch = np.s_[max(0, y - r):y + r + 1, max(0, lo - r):hi + r + 1]
            if not np.any(target[patch] & ~predicted[patch]):
                continue
            a, b = (int(lo), int(hi)) if left_to_right else (int(hi), int(lo))
            append((y, a))
            append((y, b))
        left_to_right = not left_to_right

    # A sweep alone misses short branches, concave boundaries and row remainders.
    while True:
        missing = np.argwhere(target & ~predicted)
        if not len(missing):
            break
        p = missing[np.argmin(np.sum((missing - np.asarray(route[-1])) ** 2, axis=1))]
        choices = []
        for dy, dx in np.argwhere(kernel) - r:
            q = int(p[0] + dy), int(p[1] + dx)
            if inside(reachable, q):
                choices.append(q)
        if not choices:
            raise PlanningError('A target has no reachable covering centre')
        goal = min(choices, key=lambda q: (q[0] - route[-1][0]) ** 2 + (q[1] - route[-1][1]) ** 2)
        append(goal)
        if not predicted[tuple(p)]:
            raise PlanningError('Coverage repair did not make progress')
    return Plan(simplify(route), target, predicted & free, reachable)


class CoverageTracker:
    def __init__(self, meta: MapMeta, width: float):
        self.meta, self.width = meta, width
        self.visited = np.zeros((meta.height, meta.width), dtype=bool)
        self.previous: Optional[tuple[World, float]] = None

    def rebase(self, meta: MapMeta) -> bool:
        """Preserve only an exact grid-to-grid correspondence; return whether preserved."""
        old = self.meta
        if old == meta:
            return True
        new = np.zeros((meta.height, meta.width), dtype=bool)
        ox, oy = old.local(meta.origin_x, meta.origin_y)
        compatible = (old.frame == meta.frame and abs(old.yaw - meta.yaw) < 1e-8
                      and abs(old.resolution - meta.resolution) < 1e-9
                      and abs(ox - round(ox)) < 1e-6 and abs(oy - round(oy)) < 1e-6)
        if compatible:
            dx, dy = round(ox), round(oy)
            nx0, ny0 = max(0, -dx), max(0, -dy)
            nx1, ny1 = min(meta.width, old.width - dx), min(meta.height, old.height - dy)
            if nx1 > nx0 and ny1 > ny0:
                new[ny0:ny1, nx0:nx1] = self.visited[ny0 + dy:ny1 + dy, nx0 + dx:nx1 + dx]
        self.meta, self.visited, self.previous = meta, new, None
        return compatible

    def stamp_segment(self, a: World, b: World, free: np.ndarray):
        """Cell centres within the real swept capsule, without rounding to robot cell."""
        ax, ay = self.meta.local(*a)
        bx, by = self.meta.local(*b)
        r = self.width / (2 * self.meta.resolution)
        x0, x1 = max(0, math.floor(min(ax, bx) - r)), min(self.meta.width, math.ceil(max(ax, bx) + r))
        y0, y1 = max(0, math.floor(min(ay, by) - r)), min(self.meta.height, math.ceil(max(ay, by) + r))
        if x1 <= x0 or y1 <= y0:
            return
        yy, xx = np.mgrid[y0:y1, x0:x1].astype(float)
        xx, yy = xx + 0.5, yy + 0.5
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        t = np.clip(((xx - ax) * vx + (yy - ay) * vy) / length2, 0, 1) if length2 else 0.0
        covered = (xx - ax - t * vx) ** 2 + (yy - ay - t * vy) ** 2 <= r * r + 1e-10
        self.visited[y0:y1, x0:x1] |= covered & free[y0:y1, x0:x1]

    def update(self, pose: World, stamp_sec: float, free: np.ndarray, safe: np.ndarray,
               max_speed: float = 0.4, max_gap: float = 0.5) -> bool:
        """False means a TF jump/time discontinuity was detected; do not bridge it."""
        previous = self.previous
        self.previous = pose, stamp_sec
        if not inside(safe, self.meta.pixel(*pose)):
            return False
        a = pose
        continuous = True
        if previous is not None:
            old, ts = previous
            dt = stamp_sec - ts
            dist = math.dist(pose, old)
            if dt < 0 or (dt == 0 and dist > 1e-6):
                continuous = False
            elif 0 < dt <= max_gap:
                if dist <= max_speed * dt + 0.01 and line_is_safe(old, pose, self.meta, safe):
                    a = old
                else:
                    continuous = False
            elif dt > max_gap:
                continuous = False
            # A long observation gap only stamps the current position and replans.
        self.stamp_segment(a, pose, free)
        return continuous


def split_legs(world: list[World], max_length: float) -> list[tuple[World, World]]:
    legs = []
    for a, b in zip(world, world[1:]):
        n = max(1, math.ceil(math.dist(a, b) / max_length))
        last = a
        for i in range(1, n + 1):
            point = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
            if math.dist(last, point) > 1e-9:
                legs.append((last, point))
            last = point
    return legs


def ros_node_class():
    """Delay ROS imports so --self-test also works on a non-ROS machine."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import OccupancyGrid, Path
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    class CoverageCleaner(Node):
        def __init__(self):
            super().__init__('coverage_cleaner')
            defaults = dict(
                map_topic='/explore_map', robot_base_frame='base_footprint',
                occupied_threshold=50, robot_radius=math.hypot(0.14, 0.14) / 2,
                coverage_clearance=0.02, lane_spacing=0.15, coverage_width=0.15,
                auto_plan=True, send_to_nav2=True, follow_path_action_name='/follow_path',
                controller_id='FollowPath', goal_checker_id='', controller_server='/controller_server',
                configure_goal_checker=True, goal_xy_tolerance=0.02, endpoint_tolerance=0.03,
                tracker_update_period_sec=0.1, tf_timeout_sec=1.0, tracker_max_speed=0.4,
                tracker_max_gap_sec=0.5, max_leg_length=2.0, path_resolution=0.025,
                retry_delay_sec=2.0, max_failures=3, max_no_progress_passes=3,
                max_astar_expansions=500000, goal_timeout_sec=120.0)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
                setattr(self, name, self.get_parameter(name).value)
            positive = ('robot_radius', 'lane_spacing', 'coverage_width', 'goal_xy_tolerance',
                        'endpoint_tolerance', 'tracker_update_period_sec', 'tf_timeout_sec',
                        'tracker_max_speed', 'tracker_max_gap_sec', 'max_leg_length',
                        'path_resolution', 'retry_delay_sec', 'goal_timeout_sec')
            if any(not math.isfinite(float(getattr(self, n))) or getattr(self, n) <= 0 for n in positive):
                raise ValueError('Distances and timeouts must be positive and finite')
            if (not math.isfinite(self.coverage_clearance) or self.coverage_clearance < 0
                    or not 1 <= self.occupied_threshold <= 100
                    or min(self.max_failures, self.max_no_progress_passes, self.max_astar_expansions) < 1
                    or self.endpoint_tolerance < self.goal_xy_tolerance):
                raise ValueError('Invalid threshold, clearance, retry limit or endpoint tolerance')
            if self.coverage_width > 2 * self.robot_radius:
                raise ValueError('coverage_width must fit inside the collision radius to avoid painting through walls')
            retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
            # Retained static maps and volatile threshold publishers both work.
            # Duplicate deliveries are filtered by geometry + classified contents.
            self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, retained)
            self.map_live_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, live)
            self.path_pub = self.create_publisher(Path, '/coverage_path', retained)
            self.active_pub = self.create_publisher(Path, '/coverage_active_path', retained)
            self.debug_pubs = {key: self.create_publisher(OccupancyGrid, '/coverage_' + key + '_map', retained)
                               for key in ('visited', 'remaining', 'target', 'unreachable')}
            self.status_pub = self.create_publisher(String, '/coverage_status', retained)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.action = ActionClient(self, FollowPath, self.follow_path_action_name)
            server = self.controller_server.rstrip('/')
            self.get_params = self.create_client(GetParameters, server + '/get_parameters')
            self.set_params = self.create_client(SetParameters, server + '/set_parameters')
            self.param_busy = False
            self.param_ready = not self.configure_goal_checker
            self.param_since = 0.0
            self.meta = self.info = self.free = self.safe = self.target = self.tracker = None
            self.map_version = 0
            self.enabled = bool(self.auto_plan)
            self.halted = False
            self.state, self.error = 'waiting_for_map', ''
            self.legs = deque()
            self.active_leg = None
            self.goal_handle = None
            self.goal_pending = False
            self.cancel_reason = ''
            self.cancel_sent = False
            self.action_since = 0.0
            self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='coverage-plan')
            self.plan_future = None
            self.plan_version = -1
            self.preview_version = -1
            self.retry_at = 0.0
            self.failures = self.no_progress_passes = 0
            self.pass_start_count = None
            self.last_pose = None
            self.last_debug = 0.0
            self.last_warning = {}
            self.pause_srv = self.create_service(Trigger, '~/pause', self.on_pause)
            self.resume_srv = self.create_service(Trigger, '~/resume', self.on_resume)
            self.reset_srv = self.create_service(Trigger, '~/reset', self.on_reset)
            self.timer = self.create_timer(self.tracker_update_period_sec, self.tick)
            self.get_logger().info(f'Coverage input={self.map_topic}; base={self.robot_base_frame}; '
                                   f'width={self.coverage_width:.3f}m; send_to_nav2={self.send_to_nav2}')

        @property
        def action_busy(self):
            return self.goal_pending or self.goal_handle is not None

        def warn(self, key, message):
            now = time.monotonic()
            if now - self.last_warning.get(key, -math.inf) >= 5.0:
                self.get_logger().warning(message)
                self.last_warning[key] = now

        def block(self, message):
            self.halted, self.error, self.state = True, message, 'blocked'
            self.legs.clear()
            self.cancel('blocked')
            self.get_logger().error(message)

        def on_map(self, msg):
            try:
                q = msg.info.origin.orientation
                norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if norm < 1e-8:
                    raise ValueError('Map origin quaternion has zero norm')
                qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
                if abs(qx) > 1e-6 or abs(qy) > 1e-6:
                    raise ValueError('Only planar map origins are supported')
                yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
                meta = MapMeta(msg.info.resolution, msg.info.origin.position.x,
                               msg.info.origin.position.y, msg.info.width, msg.info.height,
                               yaw, msg.header.frame_id)
                meta.validate()
                grid = np.asarray(msg.data, dtype=np.int16).reshape(meta.height, meta.width)
                if np.any((grid < -1) | (grid > 100)):
                    raise ValueError('OccupancyGrid values must be -1 or 0..100')
                free = (grid >= 0) & (grid < self.occupied_threshold)
            except (ValueError, TypeError) as exc:
                self.warn('bad_map', f'Ignoring invalid map: {exc}')
                return
            if self.meta == meta and np.array_equal(free, self.free):
                return
            old_meta = self.meta
            self.meta, self.info, self.free = meta, msg.info, free
            self.safe = traversable_map(free, meta.resolution, self.robot_radius, self.coverage_clearance)
            self.map_version += 1
            self.target = None
            if self.tracker is None:
                self.tracker = CoverageTracker(meta, self.coverage_width)
            elif not self.tracker.rebase(meta):
                self.warn('rebase', 'Grid geometry/frame changed incompatibly; visitation reset')
            self.tracker.visited &= free
            self.pass_start_count = None  # Different map denominators cannot be compared.
            if old_meta is not None and old_meta != meta:
                self.legs.clear()
                self.cancel('map_changed')
            elif self.active_leg and not line_is_safe(*self.active_leg, meta, self.safe):
                self.legs.clear()
                self.cancel('map_changed')
            elif any(not line_is_safe(a, b, meta, self.safe) for a, b in self.legs):
                # Current segment remains safe. Replan after its result.
                self.legs.clear()

        def robot_pose(self):
            if self.meta is None:
                return None
            try:
                tf = self.tf_buffer.lookup_transform(self.meta.frame, self.robot_base_frame, rclpy.time.Time())
                stamp_sec = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp_sec
                if age > self.tf_timeout_sec or age < -self.tf_timeout_sec:
                    self.warn('tf_age', f'TF age={age:.2f}s; check use_sim_time and clocks')
                    return None
                pose = (tf.transform.translation.x, tf.transform.translation.y)
                if not all(math.isfinite(v) for v in pose):
                    return None
                return pose, stamp_sec
            except TransformException as exc:
                self.warn('tf', f'Waiting for TF {self.meta.frame} -> {self.robot_base_frame}: {exc}')
                return None

        def refresh_target(self, pose):
            reachable = reachable_map(self.safe, self.meta.pixel(*pose))
            kernel = disk(self.coverage_width / (2 * self.meta.resolution))
            self.target = self.free & binary_dilation(reachable, structure=kernel)

        def tick(self):
            wall = time.monotonic()
            if self.meta is None:
                self.publish_status()
                return
            measured = self.robot_pose()
            if measured is None:
                self.tracker.previous = None
                self.legs.clear()
                self.plan_version = -1
                self.target = None
                self.cancel('tf_lost')
                if not self.halted:
                    self.state = 'waiting_for_tf'
                self.publish_status()
                return
            pose, stamp_sec = measured
            self.last_pose = pose
            if self.enabled and not self.halted and self.send_to_nav2:
                continuous = self.tracker.update(pose, stamp_sec, self.free, self.safe,
                                                 self.tracker_max_speed, self.tracker_max_gap_sec)
                if not continuous:
                    self.legs.clear()
                    self.plan_version = -1
                    self.target = None
                    self.cancel('localization_jump')
                    self.warn('jump', 'TF discontinuity or unsafe robot pose: replan without painting the gap')
            try:
                if self.target is None:
                    self.refresh_target(pose)
            except PlanningError as exc:
                self.cancel('unsafe_pose')
                self.legs.clear()
                self.state = 'waiting_for_safe_pose'
                self.warn('unsafe_pose', str(exc))
                self.publish_status()
                return
            if wall - self.last_debug >= 1.0:
                self.publish_debug()
                self.last_debug = wall
            if not self.enabled or self.halted:
                self.state = 'blocked' if self.halted else 'paused'
                return
            if self.action_busy:
                if wall - self.action_since > self.goal_timeout_sec:
                    self.cancel('timeout')
                    if wall - self.action_since > self.goal_timeout_sec + 10.0:
                        self.block('Nav2 did not terminate/cancel its goal; no replacement goal will be sent')
                return
            if wall < self.retry_at:
                return
            if self.send_to_nav2:
                if not self.action.server_is_ready():
                    self.state = 'waiting_for_nav2'
                    self.warn('nav2', f'Waiting for {self.follow_path_action_name}; will retry automatically')
                    return
                if not self.ensure_goal_checker():
                    return
            if self.plan_future is not None:
                if not self.plan_future.done():
                    return
                future, self.plan_future = self.plan_future, None
                if self.plan_version != self.map_version:
                    return  # A map change invalidates this snapshot.
                try:
                    plan = future.result()
                except Exception as exc:
                    self.block(f'Coverage planning failed: {exc}')
                    return
                self.accept_plan(plan, pose)
                if self.halted:
                    return
            if not self.send_to_nav2:
                if self.preview_version != self.map_version and self.plan_future is None:
                    self.start_plan(pose)
                return
            if self.legs:
                self.dispatch(pose)
                return
            remaining = int(np.count_nonzero(self.target & ~self.tracker.visited))
            if remaining == 0:
                if self.state != 'complete_reachable':
                    unreachable = np.count_nonzero(self.free & ~self.target & ~self.tracker.visited)
                    self.get_logger().info('Reachable coverage complete (measured TF); '
                                           f'{unreachable} known-free cells remain outside the reachable target')
                self.state = 'complete_reachable'
                self.error = ''
                self.publish_status()
                return
            done = int(np.count_nonzero(self.target & self.tracker.visited))
            if self.pass_start_count is not None:
                if done <= self.pass_start_count:
                    self.no_progress_passes += 1
                else:
                    self.no_progress_passes = 0
                    self.failures = 0
                self.pass_start_count = None
                if self.no_progress_passes >= self.max_no_progress_passes:
                    self.block('Unvisited cells remain after repeated passes. Check goal tolerance, '
                               'localization, controller tracking and coverage_width; use ~/resume after fixing.')
                    return
            self.start_plan(pose)

        def start_plan(self, pose):
            self.state = 'planning'
            self.plan_version = self.map_version
            self.plan_future = self.pool.submit(plan_coverage, self.safe.copy(), self.free.copy(), self.meta,
                                                self.meta.pixel(*pose), self.tracker.visited.copy(),
                                                self.coverage_width, self.lane_spacing, self.max_astar_expansions)

        def accept_plan(self, plan, pose):
            world = [self.meta.world(*p) for p in plan.pixels]
            if not line_is_safe(pose, world[0], self.meta, self.safe):
                self.block('Cannot safely connect the actual robot pose to the planned first cell')
                return
            if math.dist(pose, world[0]) > 1e-9:
                world.insert(0, pose)
            self.path_pub.publish(self.path_message(world))
            self.preview_version = self.map_version
            if not self.send_to_nav2:
                self.state = 'preview'
                return
            self.legs = deque(split_legs(world, self.max_leg_length))
            self.pass_start_count = int(np.count_nonzero(self.target & self.tracker.visited))
            self.state = 'executing'
            self.get_logger().info(f'Plan: {len(plan.pixels)} corners, {len(self.legs)} FollowPath segments; '
                                   f'remaining={np.count_nonzero(self.target & ~self.tracker.visited)} cells')

        def ensure_goal_checker(self):
            if self.param_ready:
                return True
            self.state = 'configuring_goal_checker'
            if self.param_busy:
                if time.monotonic() - self.param_since > 10:
                    self.block('Controller parameter service timed out')
                return False
            if not self.get_params.service_is_ready() or not self.set_params.service_is_ready():
                self.warn('params', 'Waiting for controller parameter services')
                return False
            self.param_busy, self.param_since = True, time.monotonic()
            request = GetParameters.Request()
            request.names = ['goal_checker_plugins']
            self.get_params.call_async(request).add_done_callback(self.checker_names)
            return False

        def checker_names(self, future):
            if self.halted:
                self.param_busy = False
                return
            try:
                names = list(future.result().values[0].string_array_value)
                if not self.goal_checker_id and len(names) == 1:
                    self.goal_checker_id = names[0]
                if self.goal_checker_id not in names:
                    raise ValueError(f'Set goal_checker_id to one of {names}')
                request = GetParameters.Request()
                request.names = [self.goal_checker_id + '.xy_goal_tolerance']
                self.get_params.call_async(request).add_done_callback(self.checker_tolerance)
            except Exception as exc:
                self.param_busy = False
                self.block(f'Cannot select Nav2 goal checker: {exc}')

        def checker_tolerance(self, future):
            if self.halted:
                self.param_busy = False
                return
            try:
                value = future.result().values[0]
                if value.type != ParameterType.PARAMETER_DOUBLE or value.double_value <= 0:
                    raise ValueError('Selected checker has no positive DOUBLE xy_goal_tolerance')
                if value.double_value <= self.goal_xy_tolerance:
                    self.param_ready, self.param_busy = True, False
                    return
                parameter = Parameter()
                parameter.name = self.goal_checker_id + '.xy_goal_tolerance'
                parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                  double_value=float(self.goal_xy_tolerance))
                request = SetParameters.Request()
                request.parameters = [parameter]
                self.set_params.call_async(request).add_done_callback(self.checker_configured)
            except Exception as exc:
                self.param_busy = False
                self.block(f'Cannot read goal tolerance: {exc}')

        def checker_configured(self, future):
            self.param_busy = False
            try:
                result = future.result().results
                if len(result) != 1 or not result[0].successful:
                    raise ValueError(result[0].reason if result else 'empty response')
                self.param_ready = True
                self.get_logger().info(f'{self.goal_checker_id}.xy_goal_tolerance set to '
                                       f'{self.goal_xy_tolerance:.3f}m for coverage (runtime only)')
            except Exception as exc:
                self.block(f'Cannot configure precise goal checking: {exc}')

        def path_message(self, world):
            msg = Path()
            msg.header.frame_id = self.meta.frame
            msg.header.stamp = self.get_clock().now().to_msg()
            dense = []
            spacing = min(self.path_resolution, self.meta.resolution / 2)
            for a, b in zip(world, world[1:]):
                n = max(1, math.ceil(math.dist(a, b) / spacing))
                dense.extend((a[0] + (b[0] - a[0]) * i / n,
                              a[1] + (b[1] - a[1]) * i / n) for i in range(n))
            if world:
                dense.append(world[-1])
            yaw = 0.0
            for i, (x, y) in enumerate(dense):
                if i + 1 < len(dense):
                    yaw = math.atan2(dense[i+1][1] - y, dense[i+1][0] - x)
                point = PoseStamped()
                point.header = msg.header
                point.pose.position.x, point.pose.position.y = float(x), float(y)
                point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                msg.poses.append(point)
            return msg

        def dispatch(self, pose):
            if self.action_busy or self.halted or not self.enabled:
                return
            _, goal = self.legs[0]
            if not line_is_safe(pose, goal, self.meta, self.safe):
                self.legs.clear()
                self.pass_start_count = None
                self.state = 'replanning'
                return
            if math.dist(pose, goal) < 1e-6:
                self.legs.popleft()
                return
            self.legs.popleft()
            self.active_leg = pose, goal
            path = self.path_message([pose, goal])
            self.active_pub.publish(path)
            request = FollowPath.Goal()
            request.path, request.controller_id = path, self.controller_id
            request.goal_checker_id = self.goal_checker_id
            # Reserve before send_goal_async to prevent overlapping pending goals.
            self.goal_pending, self.cancel_sent, self.cancel_reason = True, False, ''
            self.action_since = time.monotonic()
            self.state = 'executing'
            try:
                self.action.send_goal_async(request).add_done_callback(self.goal_response)
            except Exception as exc:
                # A transport exception cannot prove that Nav2 did not accept it.
                self.block(f'FollowPath send state is unknown; refusing to send another goal: {exc}')

        def goal_response(self, future):
            self.goal_pending = False
            try:
                handle = future.result()
            except Exception as exc:
                self.goal_pending = True
                self.block(f'FollowPath acceptance is unknown; no replacement will be sent: {exc}')
                return
            if not handle.accepted:
                self.active_leg = None
                if self.cancel_reason:
                    self.cancel_reason = ''
                    self.legs.clear()
                    return
                self.failed('Nav2 rejected FollowPath')
                return
            self.goal_handle = handle
            try:
                handle.get_result_async().add_done_callback(self.goal_result)
            except Exception as exc:
                self.block(f'Cannot observe the active goal result: {exc}')
                return
            if self.cancel_reason or not self.enabled or self.halted:
                self.cancel(self.cancel_reason or 'paused')

        def cancel(self, reason):
            if not self.action_busy:
                return
            self.cancel_reason = self.cancel_reason or reason
            if self.goal_handle is not None and not self.cancel_sent:
                self.cancel_sent = True
                try:
                    self.goal_handle.cancel_goal_async().add_done_callback(self.cancel_response)
                except Exception as exc:
                    self.block(f'Cancel transport failed; waiting for the original goal: {exc}')

        def cancel_response(self, future):
            try:
                result = future.result()
                if not result.goals_canceling and self.action_busy:
                    self.warn('cancel', 'Nav2 did not acknowledge cancellation; waiting for its terminal result')
            except Exception as exc:
                self.block(f'Cancellation response failed: {exc}')

        def goal_result(self, future):
            try:
                status = future.result().status
            except Exception as exc:
                self.block(f'Active goal terminal state is unknown; no replacement will be sent: {exc}')
                return
            leg, reason = self.active_leg, self.cancel_reason
            self.goal_handle, self.active_leg = None, None
            self.cancel_reason, self.cancel_sent = '', False
            self.active_pub.publish(self.path_message([]))
            if reason:
                self.legs.clear()
                if reason == 'timeout':
                    self.failed('FollowPath timed out')
                return
            measured = self.robot_pose()
            if status == GoalStatus.STATUS_SUCCEEDED and measured is not None and leg is not None:
                pose, stamp_sec = measured
                if math.dist(pose, leg[1]) > self.endpoint_tolerance:
                    self.failed('Nav2 reported success before reaching the segment endpoint. '
                                'Check goal_checker xy_goal_tolerance and TF.')
                    return
                self.tracker.update(pose, stamp_sec, self.free, self.safe,
                                    self.tracker_max_speed, self.tracker_max_gap_sec)
                self.error = ''
                self.state = 'checking_coverage'
                return
            self.failed(f'FollowPath ended with status={status}; visited cells remain recorded')

        def failed(self, message):
            self.legs.clear()
            self.pass_start_count = None
            self.failures += 1
            self.error = message
            if self.failures >= self.max_failures:
                self.block(message + ' Retry limit reached; fix the cause then call ~/resume.')
            else:
                self.state, self.retry_at = 'retry_wait', time.monotonic() + self.retry_delay_sec
                self.get_logger().warning(message + f' Retry {self.failures}/{self.max_failures}.')

        def on_pause(self, request, response):
            self.enabled = False
            self.legs.clear()
            self.plan_version = -1
            if self.tracker:
                self.tracker.previous = None
            self.cancel('paused')
            self.state = 'paused'
            response.success, response.message = True, 'Paused; waiting for active Nav2 cancellation if any'
            return response

        def on_resume(self, request, response):
            self.enabled, self.halted, self.error = True, False, ''
            self.failures = self.no_progress_passes = 0
            self.pass_start_count = None
            self.plan_version = -1
            self.param_busy = False
            self.target = None
            self.preview_version = -1
            self.legs.clear()
            self.cancel('resume_replan')
            response.success, response.message = True, 'Will replan remaining cells when map/TF/Nav2 are ready'
            return response

        def on_reset(self, request, response):
            self.on_pause(request, response)
            if self.tracker:
                self.tracker.visited.fill(False)
                self.tracker.previous = None
            self.map_version += 1  # Discard any in-flight plan using old visited cells.
            self.pass_start_count = None
            self.preview_version = -1
            self.halted, self.error = False, ''
            response.message = 'Visited cleared and paused; call ~/resume to start a new pass'
            return response

        def publish_status(self):
            status = dict(state=self.state, error=self.error, map_topic=self.map_topic,
                          map_version=self.map_version, navigation_active=self.action_busy)
            if self.target is not None:
                count = int(np.count_nonzero(self.target))
                done = int(np.count_nonzero(self.target & self.tracker.visited))
                unreachable = int(np.count_nonzero(self.free & ~self.target & ~self.tracker.visited))
                status.update(target_cells=count, visited_target_cells=done, remaining_target_cells=count-done,
                              unreachable_unvisited_cells=unreachable,
                              target_coverage_ratio=done/count if count else 0.0,
                              remaining_area_m2=(count-done)*self.meta.resolution**2)
            msg = String()
            msg.data = json.dumps(status, ensure_ascii=False)
            self.status_pub.publish(msg)

        def publish_debug(self):
            if self.target is None:
                return
            masks = dict(visited=self.tracker.visited & self.free,
                         remaining=self.free & ~self.tracker.visited, target=self.target,
                         unreachable=self.free & ~self.target & ~self.tracker.visited)
            for key, mask in masks.items():
                msg = OccupancyGrid()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id, msg.info = self.meta.frame, self.info
                msg.data = np.where(mask, 100, 0).astype(np.int8).ravel().tolist()
                self.debug_pubs[key].publish(msg)
            self.publish_status()

        def close(self):
            self.enabled = False
            self.cancel('shutdown')
            deadline = time.monotonic() + 3.0
            while self.action_busy and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.action_busy:
                self.get_logger().error('Nav2 cancellation not confirmed before exit')
            self.pool.shutdown(wait=False, cancel_futures=True)

    return CoverageCleaner


def self_test():
    """Deterministic regression tests; no ROS/robot is contacted."""
    import unittest

    class CoverageTests(unittest.TestCase):
        def assert_coverage(self, free, start, width=2.1, spacing=3.4):
            meta = MapMeta(1.0, -3.0, 2.0, free.shape[1], free.shape[0])
            plan = plan_coverage(free, free, meta, start, np.zeros_like(free), width, spacing)
            tracker = CoverageTracker(meta, width)
            world = [meta.world(*p) for p in plan.pixels]
            tracker.stamp_segment(world[0], world[0], free)
            for a, b in zip(world, world[1:]):
                self.assertTrue(line_is_safe(a, b, meta, free))
                tracker.stamp_segment(a, b, free)
            self.assertFalse(np.any(plan.target & ~tracker.visited))
            self.assertFalse(np.any(plan.target & ~plan.predicted))
            self.assertEqual(plan.pixels[0], start)
            return plan

        def test_rectangle_last_row(self):
            self.assert_coverage(np.ones((12, 19), bool), (5, 6))

        def test_one_cell_and_narrow_branch(self):
            self.assert_coverage(np.ones((1, 1), bool), (0, 0), 0.4)
            free = np.zeros((14, 20), bool)
            free[2:12, 2:12] = True
            free[6, 12:19] = True
            self.assert_coverage(free, (3, 3), 0.4, 8.0)

        def test_rooms_door_and_obstacle(self):
            free = np.ones((25, 35), bool)
            free[:, 17] = False
            free[10:14, 17] = True
            free[4:10, 5:10] = False
            self.assert_coverage(free, (3, 2))

        def test_disconnected_room_reported(self):
            free = np.ones((12, 20), bool)
            free[:, 9:12] = False
            plan = self.assert_coverage(free, (4, 3))
            self.assertFalse(plan.reachable[:, 12:].any())
            self.assertFalse(plan.target[:, 12:].any())
            self.assertGreater(np.count_nonzero(free & ~plan.target), 0)

        def test_diagonal_corner_forbidden(self):
            safe = np.eye(2, dtype=bool)
            with self.assertRaises(PlanningError):
                astar(safe, (0, 0), (1, 1))
            meta = MapMeta(1, 0, 0, 2, 2)
            self.assertFalse(line_is_safe((0.5, 0.5), (1.5, 1.5), meta, safe))
            self.assertEqual(np.count_nonzero(reachable_map(safe, (0, 0))), 1)

        def test_turnaround_not_deleted(self):
            self.assertEqual(simplify([(1, 1), (1, 5), (1, 3)]), [(1, 1), (1, 5), (1, 3)])
            self.assertEqual(simplify([(1, 1), (1, 3), (1, 5)]), [(1, 1), (1, 5)])

        def test_inflation_and_map_border(self):
            free = np.ones((30, 30), bool)
            free[12:15, 12:15] = False
            safe = traversable_map(free, 0.05, 0.099, 0.02)
            self.assertFalse(safe[0].any())
            self.assertFalse(safe[:, 0].any())
            self.assertFalse(safe[11, 12])
            self.assertTrue(safe[6, 6])

        def test_rotation_and_negative_origin(self):
            meta = MapMeta(0.05, -4.2, -7.3, 30, 40, 0.72, 'rotated_map')
            for p in ((0, 0), (18, 12), (39, 29)):
                self.assertEqual(meta.pixel(*meta.world(*p)), p)

        def test_visits_preserved_on_shift_and_resize(self):
            meta = MapMeta(0.1, 0, 0, 10, 10)
            tracker = CoverageTracker(meta, 0.15)
            tracker.visited[5, 4] = True
            bigger = MapMeta(0.1, -0.2, -0.3, 15, 18)
            self.assertTrue(tracker.rebase(bigger))
            self.assertTrue(tracker.visited[8, 6])
            self.assertTrue(tracker.rebase(MapMeta(0.1, -0.1, -0.3, 15, 18)))
            self.assertTrue(tracker.visited[8, 5])
            self.assertFalse(tracker.rebase(MapMeta(0.05, 0, 0, 30, 30)))
            self.assertFalse(tracker.visited.any())

        def test_interpolation_and_teleport(self):
            meta = MapMeta(0.01, 0, 0, 120, 30)
            free = np.ones((30, 120), bool)
            tracker = CoverageTracker(meta, 0.02)
            tracker.update((0.105, 0.155), 1.0, free, free)
            self.assertTrue(tracker.update((0.205, 0.155), 1.3, free, free))
            self.assertTrue(tracker.visited[15, 10:21].all())
            self.assertFalse(tracker.update((0.905, 0.155), 1.4, free, free))
            self.assertFalse(tracker.visited[15, 50])

        def test_true_coverage_width(self):
            meta = MapMeta(1, 0, 0, 9, 9)
            tracker = CoverageTracker(meta, 0.2)
            tracker.stamp_segment((4.5, 4.5), (4.5, 4.5), np.ones((9, 9), bool))
            self.assertEqual(np.count_nonzero(tracker.visited), 1)
            self.assertEqual(disk(0.1).shape, (1, 1))

        def test_start_is_not_silently_snapped(self):
            safe = np.ones((8, 8), bool)
            safe[0, 0] = False
            with self.assertRaises(PlanningError):
                self.assert_coverage(safe, (0, 0))

        def test_partial_coverage_and_random_maps(self):
            rng = np.random.default_rng(13)
            for _ in range(15):
                free = rng.random((18, 23)) > 0.18
                free[8, 11] = True
                self.assert_coverage(free, (8, 11), 2.1, 2.0)
            free = np.ones((13, 13), bool)
            visited = np.zeros_like(free)
            visited[:6] = True
            meta = MapMeta(1, 0, 0, 13, 13)
            plan = plan_coverage(free, free, meta, (7, 7), visited, 2.1, 2.0)
            self.assertFalse(np.any(plan.target & ~plan.predicted))

    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CoverageTests))
    return 0 if result.wasSuccessful() else 1


def main():
    if '--self-test' in sys.argv:
        return self_test()
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.executors import ExternalShutdownException
        node_class = ros_node_class()
    except ImportError as exc:
        print(f'ROS dependency missing: {exc}. Source /opt/ros/humble/setup.bash first.\n'
              'For algorithm-only checks: python3 coverage_cleaner.py --self-test', file=sys.stderr)
        return 2
    # Keep the ROS context alive during Ctrl+C long enough to request cancellation.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
