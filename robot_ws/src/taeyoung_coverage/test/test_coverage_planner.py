"""ROS2 없이 실행 가능한 coverage_planner 통합 테스트.

README_coverage.md 15장의 Test1~4에 대응한다.
가상의 OccupancyGrid를 NumPy로 직접 만들어서 검증한다.

실행: (패키지 루트 robot_ws/src/taeyoung_coverage/ 에서)
    python -m pytest test/test_coverage_planner.py -v
"""

import numpy as np

from taeyoung_coverage import coverage_planner, map_utils
from taeyoung_coverage.map_utils import MapMeta


def _make_meta(height: int, width: int, resolution: float = 0.05) -> MapMeta:
    return MapMeta(resolution=resolution, origin_x=0.0, origin_y=0.0, width=width, height=height)


# --- Test 1: 빈 직사각형 방 -> 정상 지그재그 커버리지 ------------------------
def test_v1_empty_rectangle_produces_zigzag_path():
    classified = np.full((40, 60), map_utils.FREE, dtype=np.uint8)
    meta = _make_meta(height=40, width=60)
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.05, clearance_m=0.0, resolution=meta.resolution
    )

    waypoints = coverage_planner.plan_coverage_path(
        traversable, meta, lane_spacing_m=0.15, min_region_cells=10
    )

    assert len(waypoints) >= 4
    # 모든 waypoint가 map 범위 안의 world 좌표여야 한다.
    max_x = meta.width * meta.resolution
    max_y = meta.height * meta.resolution
    for wp in waypoints:
        assert 0.0 <= wp.x <= max_x
        assert 0.0 <= wp.y <= max_y

    # 지그재그이므로 y(세로) 값이 여러 단계로 바뀌어야 한다 (한 줄로만 가면 안 됨).
    unique_ys = {round(wp.y, 3) for wp in waypoints}
    assert len(unique_ys) >= 2


# --- Test 2: 중앙에 장애물 -> 경로가 장애물을 통과하면 안 됨 -----------------
def test_v2_path_never_crosses_center_obstacle():
    height, width = 40, 60
    classified = np.full((height, width), map_utils.FREE, dtype=np.uint8)
    # 중앙에 사각형 장애물.
    classified[15:25, 25:35] = map_utils.OCCUPIED
    meta = _make_meta(height=height, width=width)

    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.05, clearance_m=0.0, resolution=meta.resolution
    )

    waypoints = coverage_planner.plan_coverage_path(
        traversable, meta, lane_spacing_m=0.15, min_region_cells=10
    )
    assert len(waypoints) > 0

    pixel_path = [map_utils.world_to_pixel(wp.x, wp.y, meta) for wp in waypoints]

    # 최종 검증: plan_coverage_path가 이미 validate_path를 내부에서 호출하지만,
    # 테스트에서도 독립적으로 다시 확인한다 (회귀 방지).
    violations = map_utils.validate_path(pixel_path, traversable)
    assert violations == []

    # 장애물 영역 자체는 traversable이 아니므로, 경로 점이 하나도 그 안에 없어야 한다.
    for y, x in pixel_path:
        assert not (15 <= y < 25 and 25 <= x < 35)


# --- Test 3: 좁은 통로 -> footprint+clearance보다 좁으면 traversable에서 제거 --
def test_v3_narrow_corridor_removed_from_traversable():
    height, width = 20, 20
    classified = np.full((height, width), map_utils.OCCUPIED, dtype=np.uint8)
    # 폭 1셀(0.05m)짜리 좁은 통로.
    classified[:, 10] = map_utils.FREE

    resolution = 0.05
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.10, clearance_m=0.02, resolution=resolution
    )

    # 로봇 반지름(0.10m) + clearance(0.02m) = 0.12m ~= 2.4셀 만큼 팽창되므로
    # 폭 1셀짜리 통로는 완전히 막혀야 한다.
    assert not traversable[:, 10].any()


# --- Test 4 (경로 단순화)는 map_utils 레벨에서 이미 검증되지만, 여기서는
#     plan_coverage_path 전체 파이프라인을 거친 결과에도 중복 직선점이
#     남지 않는지 확인한다. ---------------------------------------------------
def test_final_path_has_no_redundant_collinear_points():
    height, width = 20, 40
    classified = np.full((height, width), map_utils.FREE, dtype=np.uint8)
    meta = _make_meta(height=height, width=width)
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.05, clearance_m=0.0, resolution=meta.resolution
    )

    waypoints = coverage_planner.plan_coverage_path(
        traversable, meta, lane_spacing_m=0.2, min_region_cells=10
    )
    pixel_path = [map_utils.world_to_pixel(wp.x, wp.y, meta) for wp in waypoints]

    for (y0, x0), (y1, x1), (y2, x2) in zip(pixel_path[:-2], pixel_path[1:-1], pixel_path[2:]):
        v1 = (y1 - y0, x1 - x0)
        v2 = (y2 - y1, x2 - x1)
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        assert cross != 0, 'simplify_path 이후에도 직선상의 중복 점이 남아있음'


def test_empty_traversable_map_returns_empty_path():
    classified = np.full((10, 10), map_utils.OCCUPIED, dtype=np.uint8)
    meta = _make_meta(height=10, width=10)
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.05, clearance_m=0.0, resolution=meta.resolution
    )
    waypoints = coverage_planner.plan_coverage_path(traversable, meta)
    assert waypoints == []
