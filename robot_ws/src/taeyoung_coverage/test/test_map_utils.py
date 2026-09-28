"""ROS2 없이 실행 가능한 map_utils 단위 테스트.

실행: (패키지 루트 robot_ws/src/taeyoung_coverage/ 에서)
    python -m pytest test/test_map_utils.py -v
"""

import numpy as np

from taeyoung_coverage import map_utils
from taeyoung_coverage.map_utils import MapMeta


def test_occupancy_grid_to_numpy_shape_and_indexing():
    # 3 columns x 2 rows, row-major: row0=[0,1,2], row1=[3,4,5]
    data = [0, 1, 2, 3, 4, 5]
    grid = map_utils.occupancy_grid_to_numpy(width=3, height=2, data=data)
    assert grid.shape == (2, 3)
    assert grid[0, 0] == 0
    assert grid[0, 2] == 2
    assert grid[1, 0] == 3
    assert grid[1, 2] == 5


def test_classify_map_free_occupied_unknown():
    grid = np.array([[-1, 0, 64], [65, 100, 30]], dtype=np.int8)
    classified = map_utils.classify_map(grid, occupied_threshold=65)
    assert classified[0, 0] == map_utils.UNKNOWN
    assert classified[0, 1] == map_utils.FREE
    assert classified[0, 2] == map_utils.FREE  # 64 < 65 -> free
    assert classified[1, 0] == map_utils.OCCUPIED  # 65 >= 65 -> occupied
    assert classified[1, 1] == map_utils.OCCUPIED
    assert classified[1, 2] == map_utils.FREE


def test_compute_cleanable_map_excludes_unknown():
    classified = np.array(
        [
            [map_utils.FREE, map_utils.UNKNOWN],
            [map_utils.OCCUPIED, map_utils.FREE],
        ],
        dtype=np.uint8,
    )
    cleanable = map_utils.compute_cleanable_map(classified)
    assert cleanable.tolist() == [[True, False], [False, True]]


# --- Test 5 (README_coverage.md 15장): pixel <-> world round trip ---------
def test_pixel_world_round_trip():
    meta = MapMeta(resolution=0.05, origin_x=-1.0, origin_y=2.0, width=40, height=40)
    for y, x in [(0, 0), (5, 7), (39, 39), (20, 1)]:
        world_x, world_y = map_utils.pixel_to_world(y, x, meta)
        back_y, back_x = map_utils.world_to_pixel(world_x, world_y, meta)
        assert back_y == y
        assert back_x == x


def test_world_to_pixel_matches_origin():
    meta = MapMeta(resolution=0.1, origin_x=0.0, origin_y=0.0, width=10, height=10)
    # 셀 (0,0)의 중심은 world (0.05, 0.05)
    wx, wy = map_utils.pixel_to_world(0, 0, meta)
    assert wx == 0.05
    assert wy == 0.05


def test_compute_traversable_map_inflates_obstacles():
    classified = np.full((9, 9), map_utils.FREE, dtype=np.uint8)
    classified[4, 4] = map_utils.OCCUPIED
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.1, clearance_m=0.0, resolution=0.1
    )
    # 반지름 1셀(원형 커널)만큼 팽창하므로 (4,4) 및 상하좌우 인접 셀은 막힌다.
    assert not traversable[4, 4]
    assert not traversable[3, 4]
    assert not traversable[4, 3]
    # 원형 커널이므로 거리 sqrt(2)인 대각선 칸은 반지름 1 밖 -> 여전히 traversable.
    assert traversable[5, 5]
    # 충분히 먼 셀은 당연히 여전히 traversable.
    assert traversable[0, 0]


def test_compute_traversable_map_treats_unknown_as_obstacle():
    classified = np.full((5, 5), map_utils.FREE, dtype=np.uint8)
    classified[2, 2] = map_utils.UNKNOWN
    traversable = map_utils.compute_traversable_map(
        classified, robot_radius_m=0.0, clearance_m=0.0, resolution=0.1
    )
    assert not traversable[2, 2]


def test_connected_components_separates_regions():
    mask = np.zeros((5, 5), dtype=bool)
    mask[0:2, 0:2] = True
    mask[3:5, 3:5] = True
    labeled, num = map_utils.connected_components(mask)
    assert num == 2
    assert labeled[0, 0] == labeled[1, 1]
    assert labeled[0, 0] != labeled[4, 4]


def test_is_line_traversable_straight_and_blocked():
    traversable = np.ones((5, 5), dtype=bool)
    assert map_utils.is_line_traversable(0, 0, 4, 4, traversable)

    traversable[2, 2] = False
    assert not map_utils.is_line_traversable(0, 2, 4, 2, traversable)


def test_astar_finds_path_around_obstacle():
    traversable = np.ones((5, 5), dtype=bool)
    traversable[:, 2] = False
    traversable[4, 2] = True  # 맨 아래쪽에 구멍을 뚫어 우회로를 만든다

    path = map_utils.astar((0, 0), (0, 4), traversable)
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (0, 4)
    for y, x in path:
        assert traversable[y, x]


def test_astar_returns_none_when_unreachable():
    traversable = np.ones((5, 5), dtype=bool)
    traversable[:, 2] = False  # 완전히 막힌 벽 (구멍 없음)

    path = map_utils.astar((0, 0), (0, 4), traversable)
    assert path is None


# --- Test 4: path simplification -------------------------------------------
def test_simplify_path_removes_collinear_points():
    points = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    simplified = map_utils.simplify_path(points)
    assert simplified == [(0, 0), (4, 0)]


def test_simplify_path_keeps_turning_points():
    points = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    simplified = map_utils.simplify_path(points)
    assert simplified == [(0, 0), (2, 0), (2, 2)]


def test_validate_path_detects_out_of_bounds_and_blocked():
    traversable = np.ones((3, 3), dtype=bool)
    traversable[1, 1] = False

    violations = map_utils.validate_path([(0, 0), (10, 10)], traversable)
    assert any('outside map bounds' in v for v in violations)

    violations2 = map_utils.validate_path([(1, 1)], traversable)
    assert any('not traversable' in v for v in violations2)


def test_robot_radius_from_footprint_matches_burger_urdf():
    # robot_ws/src/custom_burger/urdf/turtlebot3_burger.urdf collision box: 0.140 x 0.140
    radius = map_utils.robot_radius_from_footprint((0.140, 0.140))
    assert abs(radius - 0.0990) < 1e-3
    assert abs(map_utils.DEFAULT_ROBOT_RADIUS_M - radius) < 1e-9
