"""알고리즘 회귀 테스트. ROS/로봇 없이 실행된다."""
from __future__ import annotations

import math
import unittest

import numpy as np

from coverage_planner import plan_coverage
from coverage_tracker import CoverageTracker
from coverage_types import MapMeta, PlanningError
from grid_ops import disk
from home_geometry import at_home, home_error, wrap_angle, yaw_from_quaternion
from line_safety import line_is_safe
from parameter_loader import default_parameters, validate_parameters
from path_search import astar, simplify
from traversability import reachable_map, traversable_map


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

    def test_default_parameter_files_are_valid(self):
        """parameters/feature_flags.py / thresholds.py / node_settings.py 기본값이 검증을 통과한다."""
        from types import SimpleNamespace
        params = SimpleNamespace(**default_parameters())
        validate_parameters(params)
        self.assertEqual(params.planning_extra_margin, 0.2)

    def test_home_geometry(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi / 2), -math.pi / 2)
        half = math.pi / 4
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, math.sin(half), math.cos(half)), math.pi / 2)
        home = (1.0, -2.0, 3.0)
        # 3.0rad 에서 -3.0rad 로: 가까운 쪽(약 +0.28rad)으로 돌아야 한다.
        distance, turn = home_error((1.0, -2.0), -3.0, home)
        self.assertAlmostEqual(distance, 0.0)
        self.assertAlmostEqual(turn, 6.0 - 2 * math.pi, places=6)
        self.assertTrue(at_home((1.05, -2.05), 2.95, home, 0.15, 0.1))
        self.assertFalse(at_home((1.3, -2.0), 3.0, home, 0.15, 0.1))   # 위치 초과
        self.assertFalse(at_home((1.0, -2.0), 2.8, home, 0.15, 0.1))   # yaw 초과

    def test_home_parameters_are_validated(self):
        from types import SimpleNamespace
        params = SimpleNamespace(**{**default_parameters(), 'home_x': 1.0})
        with self.assertRaises(ValueError):
            validate_parameters(params)
        params = SimpleNamespace(**{**default_parameters(), 'home_x': 1.0, 'home_y': 2.0})
        validate_parameters(params)


def run() -> int:
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(CoverageTests))
    return 0 if result.wasSuccessful() else 1
