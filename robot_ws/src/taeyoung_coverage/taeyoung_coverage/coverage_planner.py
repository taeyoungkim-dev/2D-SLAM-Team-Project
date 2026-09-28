"""ROS2에 의존하지 않는 순수 Python Coverage Path Planner.

입력은 :mod:`map_utils` 가 만들어주는 ``traversable_map`` (NumPy bool 2D
배열)과 :class:`map_utils.MapMeta` 뿐이다. rclpy, nav_msgs 등 ROS 메시지는
이 파일 어디에서도 import 하지 않는다 (README_coverage.md 9, 14장).

알고리즘 개요 (V1 + V2 + V3-lite):
    1. ``traversable_map`` 을 connected component로 분리한다 (서로 다른
       청소 영역, 6장 V3).
    2. 각 영역을 행(row) 단위로 스캔한다. 한 행 안에서 장애물로 끊어진
       free interval은 자동으로 별개의 lane segment가 된다 (6장 V2 -
       "한 sweep line 안의 free-space interval을 구분").
    3. 지그재그(Boustrophedon) 순서로 lane segment들을 나열한다.
    4. 인접한 segment 끝점 사이를 먼저 직선 시야로 연결을 시도하고,
       막혀 있으면 A*로 우회 경로(transit)를 찾아 이어붙인다. 이 로직은
       영역 사이(V3)를 잇는 데도 동일하게 재사용된다.
    5. 직선 위 불필요한 중간점을 제거하고(path simplification),
       최종적으로 ``validate_path`` 로 검증한다.
    6. 픽셀 경로를 world 좌표로 변환하고, 다음 점을 바라보는 yaw를 계산한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import map_utils
from .map_utils import MapMeta


class CoveragePlanningError(RuntimeError):
    """계산된 coverage 경로가 validate_path 검증을 통과하지 못했을 때 발생."""


@dataclass(frozen=True)
class Waypoint:
    """world 좌표계의 한 경로점. x, y는 미터, yaw는 라디안."""

    x: float
    y: float
    yaw: float


PixelPoint = Tuple[int, int]  # (y, x)


# ---------------------------------------------------------------------------
# 1. 한 행(row) 안에서 free interval 찾기 (V2)
# ---------------------------------------------------------------------------
def row_intervals(region_row: np.ndarray, min_interval_cells: int) -> List[Tuple[int, int]]:
    """1D bool 배열(한 connected region의 한 행)에서 연속된 True 구간을 찾는다.

    입력: region_row, shape (W,) bool.
    출력: [(x_start, x_end), ...] (둘 다 포함, 오름차순 x). min_interval_cells
        보다 짧은 구간은 제외한다 (노이즈/너무 좁은 통로 제거).
    """
    intervals: List[Tuple[int, int]] = []
    width = region_row.shape[0]
    x = 0
    while x < width:
        if region_row[x]:
            x_start = x
            while x < width and region_row[x]:
                x += 1
            x_end = x - 1
            if x_end - x_start + 1 >= min_interval_cells:
                intervals.append((x_start, x_end))
        else:
            x += 1
    return intervals


# ---------------------------------------------------------------------------
# 2. 한 connected region에 대한 Boustrophedon lane 생성 (V1 + V2)
# ---------------------------------------------------------------------------
def plan_region_lanes(
    region_mask: np.ndarray,
    lane_step_rows: int,
    min_interval_cells: int,
) -> List[PixelPoint]:
    """한 connected region을 지그재그로 훑는 lane 끝점 목록을 만든다.

    lane_step_rows 간격으로 행을 골라, 각 행에서 (장애물로 끊긴) free
    interval마다 하나의 segment [시작점, 끝점]을 만든다. 홀수 번째 lane은
    왼쪽->오른쪽, 짝수 번째는 오른쪽->왼쪽으로 진행 방향을 번갈아 지그재그
    패턴을 만든다 (V1 기본 커버리지).

    같은 행에 장애물로 분리된 interval이 여러 개면 각각 별도 segment로
    나오므로, 그 사이는 이 함수가 아니라 :func:`connect_pixel_points` 가
    직선/A*로 이어준다 (장애물을 가로질러 직선으로 잇지 않기 위함, V2/5장).

    출력: [(y, x), ...] - lane segment들의 끝점만 순서대로 나열한 목록
        (아직 연결/검증 전 raw waypoint).
    """
    row_has_region = np.where(region_mask.any(axis=1))[0]
    if row_has_region.size == 0:
        return []

    y_min, y_max = int(row_has_region.min()), int(row_has_region.max())

    waypoints: List[PixelPoint] = []
    direction = 1  # 1: 왼쪽->오른쪽, -1: 오른쪽->왼쪽
    row = y_min
    while row <= y_max:
        intervals = row_intervals(region_mask[row], min_interval_cells)
        if intervals:
            ordered = intervals if direction == 1 else list(reversed(intervals))
            for x_start, x_end in ordered:
                if direction == 1:
                    waypoints.append((row, x_start))
                    waypoints.append((row, x_end))
                else:
                    waypoints.append((row, x_end))
                    waypoints.append((row, x_start))
            direction *= -1
        row += lane_step_rows

    return waypoints


# ---------------------------------------------------------------------------
# 3. 여러 region을 방문 순서로 정렬 (V3-lite)
# ---------------------------------------------------------------------------
def order_regions(
    region_masks: Sequence[np.ndarray],
    start_pixel: Optional[PixelPoint],
) -> List[np.ndarray]:
    """탐색할 region들을 방문 순서로 정렬한다.

    start_pixel이 주어지면 그 지점에 가장 가까운 region부터 시작하고,
    이후로는 항상 "마지막 region의 중심에서 가장 가까운 다음 region"을
    그리디하게 고른다. 완벽한 최적 순회(TSP)는 아니지만, 6장 V3가
    요구하는 "서로 나뉜 영역을 안전하게 연결"의 최소 구현으로 충분하다.
    """
    remaining = list(region_masks)
    if not remaining:
        return []

    def centroid(mask: np.ndarray) -> Tuple[float, float]:
        ys, xs = np.where(mask)
        return float(ys.mean()), float(xs.mean())

    if start_pixel is not None:
        def dist_to_start(mask: np.ndarray) -> float:
            ys, xs = np.where(mask)
            d2 = (ys - start_pixel[0]) ** 2 + (xs - start_pixel[1]) ** 2
            return float(d2.min())

        remaining.sort(key=dist_to_start)
    else:
        remaining.sort(key=lambda m: -int(np.count_nonzero(m)))

    ordered = [remaining.pop(0)]
    while remaining:
        last_centroid = centroid(ordered[-1])

        def dist_to_last(mask: np.ndarray) -> float:
            cy, cx = centroid(mask)
            return (cy - last_centroid[0]) ** 2 + (cx - last_centroid[1]) ** 2

        remaining.sort(key=dist_to_last)
        ordered.append(remaining.pop(0))

    return ordered


# ---------------------------------------------------------------------------
# 4. waypoint들을 안전하게 연결 (직선 우선, 막히면 A* transit)
# ---------------------------------------------------------------------------
def connect_pixel_points(
    points: Sequence[PixelPoint],
    traversable: np.ndarray,
    max_astar_expansions: int,
) -> List[PixelPoint]:
    """연속된 waypoint들을 traversable 영역만 지나도록 연결한다.

    각 인접 쌍에 대해:
        1) 직선(Bresenham)으로 바로 가도 안전하면 그대로 연결.
        2) 막혀 있으면 A*로 transit 경로를 찾아 끼워 넣는다.
        3) A*도 실패하면 (서로 완전히 격리된 영역) 그 연결은 포기하고
           다음 점으로 건너뛴다 - "장애물을 사이에 둔 free interval을
           직선으로 잇지 않는다"는 규칙을 지키기 위해, 안전한 경로가 없으면
           억지로 잇지 않는다 (5, 13, 16장).
    """
    if not points:
        return []

    result: List[PixelPoint] = [points[0]]
    for nxt in points[1:]:
        cur = result[-1]
        if cur == nxt:
            continue

        if map_utils.is_line_traversable(cur[0], cur[1], nxt[0], nxt[1], traversable):
            result.append(nxt)
            continue

        transit = map_utils.astar(cur, nxt, traversable, max_expansions=max_astar_expansions)
        if transit is None:
            # 안전하게 이을 방법이 없음 -> 이 점은 건너뛴다 (억지 연결 금지).
            continue
        result.extend(transit[1:])

    return result


# ---------------------------------------------------------------------------
# 5. 픽셀 경로 -> world Waypoint (yaw 포함)
# ---------------------------------------------------------------------------
def pixels_to_waypoints(pixels: Sequence[PixelPoint], meta: MapMeta) -> List[Waypoint]:
    """픽셀 [y,x] 경로를 world Waypoint(x,y,yaw) 목록으로 변환.

    각 점의 yaw는 "다음 점을 바라보는 방향"으로 계산한다 (README_coverage.md
    12장). 마지막 점은 이전 yaw를 그대로 유지한다.
    """
    import math

    world_points = [map_utils.pixel_to_world(y, x, meta) for y, x in pixels]
    waypoints: List[Waypoint] = []
    n = len(world_points)
    last_yaw = 0.0
    for i, (wx, wy) in enumerate(world_points):
        if i < n - 1:
            nx, ny = world_points[i + 1]
            last_yaw = math.atan2(ny - wy, nx - wx)
        waypoints.append(Waypoint(x=wx, y=wy, yaw=last_yaw))
    return waypoints


# ---------------------------------------------------------------------------
# 6. 전체 파이프라인
# ---------------------------------------------------------------------------
def plan_coverage_path(
    traversable: np.ndarray,
    meta: MapMeta,
    *,
    lane_spacing_m: float = 0.15,
    min_interval_cells: int = 2,
    min_region_cells: int = 20,
    start_world: Optional[Tuple[float, float]] = None,
    max_astar_expansions: int = 200_000,
) -> List[Waypoint]:
    """traversable_map으로부터 완전한 Coverage 경로를 계산한다.

    Args:
        traversable: map_utils.compute_traversable_map()의 출력, shape (H,W) bool.
        meta: 픽셀<->world 변환에 필요한 지도 메타데이터.
        lane_spacing_m: coverage lane 사이 간격 (미터). OccupancyGrid
            resolution과 다른, 로봇/청소 폭 기준의 별도 파라미터 (7장).
        min_interval_cells: 이보다 짧은 free interval은 (좁은 통로 등) 무시.
        min_region_cells: 이보다 작은 connected region은 노이즈로 무시.
        start_world: 로봇의 현재 world (x, y). 주어지면 가장 가까운 region부터
            방문 순서를 정한다. None이면 가장 큰 region부터 시작.
        max_astar_expansions: A* transit 탐색의 노드 확장 상한 (안전장치).

    Returns:
        world 좌표계 Waypoint 목록. 비어 있으면 청소 가능한 영역이 없다는 뜻.

    Raises:
        CoveragePlanningError: 최종 경로가 validate_path 검증에 실패한 경우
            (설계상 정상 흐름에서는 발생하지 않아야 하며, 발생 시 버그로 취급).
    """
    if lane_spacing_m <= 0:
        raise ValueError('lane_spacing_m must be positive')

    labeled, num_labels = map_utils.connected_components(traversable)
    region_masks = [labeled == i for i in range(1, num_labels + 1)]
    region_masks = [m for m in region_masks if int(np.count_nonzero(m)) >= min_region_cells]
    if not region_masks:
        return []

    start_pixel: Optional[PixelPoint] = None
    if start_world is not None:
        start_pixel = map_utils.world_to_pixel(start_world[0], start_world[1], meta)

    ordered_regions = order_regions(region_masks, start_pixel)

    lane_step_rows = max(1, round(lane_spacing_m / meta.resolution))

    raw_pixels: List[PixelPoint] = []
    for region_mask in ordered_regions:
        raw_pixels.extend(plan_region_lanes(region_mask, lane_step_rows, min_interval_cells))

    if len(raw_pixels) < 2:
        return []

    connected_pixels = connect_pixel_points(raw_pixels, traversable, max_astar_expansions)
    if len(connected_pixels) < 2:
        return []

    simplified_pixels = map_utils.simplify_path(connected_pixels)

    violations = map_utils.validate_path(simplified_pixels, traversable)
    if violations:
        raise CoveragePlanningError('; '.join(violations[:5]))

    return pixels_to_waypoints(simplified_pixels, meta)
