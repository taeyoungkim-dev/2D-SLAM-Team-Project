"""ROS2에 의존하지 않는 순수 NumPy 기반 지도 유틸리티.

이 모듈은 macOS 등 ROS2가 없는 환경에서도 일반 Python(numpy/scipy)만으로
테스트할 수 있어야 한다 (README_coverage.md 14장).

좌표계 규칙 (README_coverage.md 17장):
    - NumPy / OccupancyGrid 인덱스: ``[y, x]`` (row, col)
    - ROS world 좌표: ``[x, y]`` (meter)

이 규칙을 지키기 위해 이 파일의 모든 함수는 픽셀 좌표를 항상 ``(y, x)``
튜플/인자 순서로, world 좌표를 항상 ``(x, y)`` 순서로 주고받는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import binary_dilation, label

# ---------------------------------------------------------------------------
# Map classification 상수 (README_coverage.md 2장)
# ---------------------------------------------------------------------------
FREE = 0
OCCUPIED = 1
UNKNOWN = 2


@dataclass(frozen=True)
class MapMeta:
    """OccupancyGrid.info 에서 뽑아낸, ROS 메시지에 의존하지 않는 지도 메타데이터.

    Attributes:
        resolution: 미터/셀.
        origin_x, origin_y: OccupancyGrid.info.origin.position (map frame, meter).
            (map origin에 회전이 없다고 가정한다. Cartographer 기본 출력은
            origin.orientation이 identity이므로 이 프로젝트 범위에서는 안전한
            단순화이다. 회전이 있는 지도에는 이 유틸리티를 바로 쓸 수 없다.)
        width, height: 셀 단위 지도 크기 (OccupancyGrid.info.width/height).
    """

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


# ---------------------------------------------------------------------------
# OccupancyGrid <-> NumPy
# ---------------------------------------------------------------------------
def occupancy_grid_to_numpy(width: int, height: int, data: Sequence[int]) -> np.ndarray:
    """1차원 row-major OccupancyGrid.data 를 (height, width) int8 배열로 변환.

    입력: OccupancyGrid.data (1D, 길이 width*height, row-major, 각 값은
        -1(unknown) 또는 0~100(점유 확률)).
    출력: shape (height, width) int8 배열, 인덱스는 [y, x].
    """
    return np.array(data, dtype=np.int8).reshape((height, width))


def classify_map(grid: np.ndarray, occupied_threshold: int = 65) -> np.ndarray:
    """OccupancyGrid 값을 FREE/OCCUPIED/UNKNOWN 세 범주로 분류.

    입력: grid, shape (H, W) int8, 값 -1(unknown) 또는 0~100.
    출력: shape (H, W) uint8, 값은 FREE(0) / OCCUPIED(1) / UNKNOWN(2).

    threshold 미만은 free, threshold 이상은 occupied, -1은 unknown.
    (README_coverage.md 2장: threshold는 파라미터로 조정 가능해야 함 -> 이 함수의
    인자로 노출되어 있고, ROS 노드에서는 ROS parameter 값을 여기로 넘긴다.)
    """
    classified = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    classified[(grid >= 0) & (grid < occupied_threshold)] = FREE
    classified[grid >= occupied_threshold] = OCCUPIED
    return classified


def compute_cleanable_map(classified: np.ndarray) -> np.ndarray:
    """Unknown을 제외한 '청소 대상' 영역(cleanable_map)을 계산 (3장).

    입력: classify_map()의 출력.
    출력: shape (H, W) bool. True == 이 셀은 known-free (청소 후보).
    """
    return classified == FREE


# ---------------------------------------------------------------------------
# C-space (로봇 footprint + clearance 반영)
# ---------------------------------------------------------------------------
def make_disk_kernel(radius_cells: int) -> np.ndarray:
    """반지름 radius_cells(셀 단위)인 원형 구조 요소(structuring element) 생성.

    (README_coverage.md 4장: "가능하면 원형 approximation보다 실제 polygon
    footprint를 우선"하라고 했지만, 이 V1 구현은 원형 근사를 사용한다 -
    구현 복잡도를 낮추기 위한 명시적 선택이다. 자세한 이유는
    coverage_planner.compute_traversable_map()의 docstring 참고.)
    """
    if radius_cells <= 0:
        return np.array([[True]])
    y, x = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
    return (x * x + y * y) <= radius_cells * radius_cells


def inflate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """boolean mask를 radius_cells 만큼 원형으로 팽창(dilate)."""
    if radius_cells <= 0:
        return mask.copy()
    kernel = make_disk_kernel(radius_cells)
    return binary_dilation(mask, structure=kernel)


def compute_traversable_map(
    classified: np.ndarray,
    robot_radius_m: float,
    clearance_m: float,
    resolution: float,
) -> np.ndarray:
    """로봇 중심이 안전하게 지나갈 수 있는 C-space(traversable_map)를 계산.

    설계 선택 (README_coverage.md 3, 4, 13장):
        - OCCUPIED 셀뿐 아니라 UNKNOWN 셀도 "장애물처럼" 취급해서 함께
          inflate 한다. 13장이 "unknown 영역을 통과해서는 안 된다"고
          명시하므로, 로봇 중심이 unknown 경계에 너무 붙어서 지나가는 것도
          막기 위한 보수적인 선택이다.
        - inflation 반지름은 robot_radius_m + clearance_m 을 셀 단위로
          올림(ceil)하여 사용한다 (과소평가로 인한 충돌보다 과대평가로
          인한 영역 축소가 안전하다).

    입력:
        classified: classify_map() 출력, shape (H, W).
        robot_radius_m: 로봇 footprint의 원형 근사 반지름 (미터).
        clearance_m: 추가 안전 여유 (미터).
        resolution: 미터/셀.
    출력: shape (H, W) bool. True == 로봇 중심이 위치해도 안전한 known-free 셀.
    """
    obstacle_like = (classified == OCCUPIED) | (classified == UNKNOWN)
    radius_cells = int(math.ceil((robot_radius_m + clearance_m) / resolution))
    inflated_obstacle = inflate_mask(obstacle_like, radius_cells)
    free = classified == FREE
    return free & ~inflated_obstacle


# ---------------------------------------------------------------------------
# Connected components
# ---------------------------------------------------------------------------
def connected_components(mask: np.ndarray) -> Tuple[np.ndarray, int]:
    """8방향 연결 기준으로 mask의 connected component를 라벨링.

    입력: mask, shape (H, W) bool.
    출력: (labeled, num) - labeled: shape (H, W) int, 배경은 0, 컴포넌트는 1..num.
    """
    structure = np.ones((3, 3), dtype=int)
    labeled, num = label(mask, structure=structure)
    return labeled, num


# ---------------------------------------------------------------------------
# Pixel <-> World 변환
# ---------------------------------------------------------------------------
def pixel_to_world(y: int, x: int, meta: MapMeta) -> Tuple[float, float]:
    """픽셀 [y, x] (셀 인덱스) -> world (x_m, y_m) 미터. 셀 중심 기준.

    출력 순서는 world 좌표 규칙에 따라 (x, y) 이다.
    """
    world_x = meta.origin_x + (x + 0.5) * meta.resolution
    world_y = meta.origin_y + (y + 0.5) * meta.resolution
    return world_x, world_y


def world_to_pixel(world_x: float, world_y: float, meta: MapMeta) -> Tuple[int, int]:
    """world (x, y) 미터 -> 픽셀 [y, x] (셀 인덱스).

    pixel_to_world()와 셀 중심 기준으로 짝을 이루도록 floor를 사용한다
    (round-trip 오차가 resolution 이내가 되도록: Test 5).
    """
    x = int(math.floor((world_x - meta.origin_x) / meta.resolution))
    y = int(math.floor((world_y - meta.origin_y) / meta.resolution))
    return y, x


# ---------------------------------------------------------------------------
# 직선 시야(Line-of-sight) 검사 - Bresenham
# ---------------------------------------------------------------------------
def bresenham_line(y0: int, x0: int, y1: int, x1: int) -> List[Tuple[int, int]]:
    """정수 Bresenham 알고리즘. 시작점과 끝점을 포함한 픽셀 [y, x] 목록 반환."""
    points: List[Tuple[int, int]] = []
    dx = x1 - x0
    dy = y1 - y0
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return [(y0, x0)]
    for i in range(steps + 1):
        t = i / steps
        y = round(y0 + dy * t)
        x = round(x0 + dx * t)
        points.append((y, x))
    return points


def is_line_traversable(y0: int, x0: int, y1: int, x1: int, traversable: np.ndarray) -> bool:
    """(y0,x0) -> (y1,x1) 직선이 traversable 영역만 지나는지 검사.

    두 lane segment/waypoint를 "직선으로 그냥 연결"해도 되는지 판단할 때 사용한다
    (README_coverage.md 5, 13장: 장애물을 사이에 둔 free interval을 직선으로
    연결하면 안 됨 -> 이 함수로 사전 검증한다).
    """
    h, w = traversable.shape
    for y, x in bresenham_line(y0, x0, y1, x1):
        if not (0 <= y < h and 0 <= x < w):
            return False
        if not traversable[y, x]:
            return False
    return True


# ---------------------------------------------------------------------------
# 간단한 Grid A* (장애물로 분리된 영역/구간을 잇는 transit 경로용)
# ---------------------------------------------------------------------------
_NEIGHBORS_8 = (
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
)


def astar(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    traversable: np.ndarray,
    max_expansions: int = 200_000,
) -> Optional[List[Tuple[int, int]]]:
    """traversable 격자 위에서 start -> goal 8방향 A* 최단 경로 탐색.

    직선 연결(is_line_traversable)이 막힌 두 지점을 안전하게 잇기 위한
    "V3-lite" transit 경로 계산에 사용된다 (README_coverage.md 6장 V3,
    17장 "필요한 경우 A*를 간단히 구현"). Coverage lane 자체를 만드는 데는
    쓰이지 않는다 - 오직 이미 결정된 lane 사이를 연결할 때만 사용한다.

    입력: start, goal - 픽셀 [y, x]. 둘 다 traversable 이어야 한다(호출 전 확인).
    출력: [y,x] 픽셀 경로 (start, goal 포함) 또는 경로가 없으면 None.
    """
    import heapq

    h, w = traversable.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and traversable[start]):
        return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w and traversable[goal]):
        return None

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: List[Tuple[float, float, Tuple[int, int]]] = [(heuristic(start, goal), 0.0, start)]
    came_from: dict = {}
    g_score = {start: 0.0}
    closed: set = set()
    expansions = 0

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if expansions > max_expansions:
            return None

        cy, cx = current
        for dy, dx in _NEIGHBORS_8:
            ny, nx = cy + dy, cx + dx
            neighbor = (ny, nx)
            if not (0 <= ny < h and 0 <= nx < w):
                continue
            if not traversable[ny, nx]:
                continue
            if neighbor in closed:
                continue
            step_cost = math.hypot(dy, dx)
            tentative_g = g + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                heapq.heappush(open_heap, (tentative_g + heuristic(neighbor, goal), tentative_g, neighbor))

    return None


# ---------------------------------------------------------------------------
# Path simplification (직선 위의 불필요한 중간점 제거)
# ---------------------------------------------------------------------------
def simplify_path(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """직선(collinear) 위에 놓인 중간점을 제거한다 (README_coverage.md 5, 15장 Test4).

    입력/출력 모두 2-tuple 좌표 목록. 픽셀([y,x], int)이든 world([x,y], float)든
    동일하게 동작한다 (pixel<->world 변환이 회전 없는 affine이므로 직선성은
    보존된다 - MapMeta의 "회전 없음" 가정과 일치).
    """
    pts = list(points)
    if len(pts) <= 2:
        return pts

    simplified = [pts[0]]
    for i in range(1, len(pts) - 1):
        prev = simplified[-1]
        curr = pts[i]
        nxt = pts[i + 1]
        v1 = (curr[0] - prev[0], curr[1] - prev[1])
        v2 = (nxt[0] - curr[0], nxt[1] - curr[1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if abs(cross) > 1e-9:
            simplified.append(curr)
        # else: curr는 prev-nxt 직선 위 -> 버린다 (simplified[-1]은 그대로 prev 유지)
    simplified.append(pts[-1])
    return simplified


# ---------------------------------------------------------------------------
# Path validation
# ---------------------------------------------------------------------------
def validate_path(path_pixels: Sequence[Tuple[int, int]], traversable: np.ndarray) -> List[str]:
    """최종 경로가 traversable_map 밖으로 나가지 않는지 검증 (README_coverage.md 13장).

    입력: path_pixels - 픽셀 [y, x] 목록.
    출력: 위반 사항 설명 문자열 리스트. 빈 리스트면 valid.
    """
    violations: List[str] = []
    if not path_pixels:
        return violations

    h, w = traversable.shape
    for y, x in path_pixels:
        if not (0 <= y < h and 0 <= x < w):
            violations.append(f'point ({y},{x}) is outside map bounds')
        elif not traversable[y, x]:
            violations.append(f'point ({y},{x}) is not traversable')

    for (y0, x0), (y1, x1) in zip(path_pixels[:-1], path_pixels[1:]):
        if not is_line_traversable(y0, x0, y1, x1, traversable):
            violations.append(f'segment ({y0},{x0})-({y1},{x1}) crosses non-traversable cells')

    return violations


def robot_radius_from_footprint(footprint_size_m: Tuple[float, float]) -> float:
    """사각형 footprint(길이 x 폭)의 외접원 반지름을 계산.

    TurtleBot3 Burger 실제 URDF collision box는
    ``robot_ws/src/custom_burger/urdf/turtlebot3_burger.urdf`` 의
    ``<collision><geometry><box size="0.140 0.140 0.143"/>`` 이다
    (README_coverage.md 4장: "단순히 17.8cm를 robot radius로 쓰지 말 것" ->
    실제로는 0.140m x 0.140m 정사각형이다).

    이 함수는 회전에 관계없이 사각형 footprint 전체를 포함하는 외접원
    반지름을 반환한다 (원형 근사, 보수적/안전 방향). 실제 polygon
    footprint를 쓰는 대신 원형 근사를 선택한 이유는 이 파일 상단
    ``make_disk_kernel`` docstring 참고.
    """
    length, width = footprint_size_m
    return math.hypot(length, width) / 2.0


# TurtleBot3 Burger 기본 footprint 기반 기본 로봇 반지름 (원형 근사, 외접원).
# URDF box: 0.140m x 0.140m -> hypot(0.14,0.14)/2 ~= 0.099m
DEFAULT_BURGER_FOOTPRINT_M: Tuple[float, float] = (0.140, 0.140)
DEFAULT_ROBOT_RADIUS_M: float = robot_radius_from_footprint(DEFAULT_BURGER_FOOTPRINT_M)
