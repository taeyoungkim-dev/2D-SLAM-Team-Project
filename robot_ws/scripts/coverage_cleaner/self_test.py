"""알고리즘 회귀 테스트. ROS/로봇 없이 실행된다."""
from __future__ import annotations

import math
import os
import tempfile
import unittest

import numpy as np

from coverage_planner import plan_coverage
from coverage_tracker import CoverageTracker
from coverage_types import MapMeta, PlanningError
from grid_ops import disk
from home_geometry import at_home, home_error, wrap_angle, yaw_from_quaternion
from line_safety import line_is_safe
from parameter_loader import default_parameters, validate_parameters
from parameters.thresholds import OBSTACLE_ZONE_RADII_M, PLANNING_EXTRA_MARGIN_M
from map_updater import MapUpdater
from obstacle_zones import Zone, zone_mask
from path_search import astar, simplify
from route_planner import plan_route
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
        self.assertEqual(params.planning_extra_margin, PLANNING_EXTRA_MARGIN_M)

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


class _FakeExecutor:
    def __init__(self):
        self.cancelled = []

    def cancel(self, reason):
        self.cancelled.append(reason)


class ObstacleZoneTests(unittest.TestCase):
    def test_spawn_file_positions_and_radii(self):
        from types import SimpleNamespace
        import obstacle_zones
        params = SimpleNamespace(**default_parameters())
        spawns = {sp.name: sp for sp in obstacle_zones.parse_spawn_file(params.obstacle_spawn_file)}
        self.assertEqual(set(spawns), {'power_cable', 'cardboard_box', 'urine_puddle'})
        for sp in spawns.values():                                        # 위치는 spawn 파일을 고치면 바뀌므로 값은 고정하지 않는다
            self.assertTrue(math.isfinite(sp.x) and math.isfinite(sp.y))
        radii = {n: obstacle_zones.zone_radius(n) for n in spawns}
        self.assertEqual(radii, {n: OBSTACLE_ZONE_RADII_M[n] for n in spawns})   # 표의 값을 그대로 쓴다

    def test_world_to_map_transform(self):
        import obstacle_zones
        sp = obstacle_zones.Spawn('x', 1.0, 0.0, 0.0)
        x, y = obstacle_zones.to_map(sp, 2.0, 3.0, math.pi / 2)   # world 의 +x 는 map 의 +y
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 4.0)

    def test_zones_are_created_once_and_never_updated(self):
        from types import SimpleNamespace
        import obstacle_zones
        params = SimpleNamespace(**default_parameters())
        notified, tf = [], [(1.0, 2.0, 0.0)]
        ctx = SimpleNamespace(
            params=params, state=SimpleNamespace(meta=MapMeta(0.05, -5.0, -5.0, 200, 200)),
            logger=SimpleNamespace(info=lambda m: None, error=print),
            warn=lambda k, m: None,
            map_updater=SimpleNamespace(on_zones_changed=lambda: notified.append(1)))
        looked_up = []

        def lookup(target, source):
            looked_up.append(1)
            if tf[0] is None:
                raise RuntimeError('no tf')
            return tf[0]

        inp = obstacle_zones.ObstacleZoneInput(ctx, lookup)
        self.assertEqual(inp.zones(), [])
        self.assertFalse(inp.done)
        tf[0] = None                                            # TF 가 없으면 구역 없이 기다린다
        inp.refresh()
        self.assertEqual(inp.zones(), [])
        self.assertEqual(notified, [])
        tf[0] = (1.0, 2.0, 0.0)
        inp.refresh()                                           # 처음 TF 가 되는 순간 한 번 만든다
        zones = {z.name: z for z in inp.zones()}
        self.assertAlmostEqual(zones['cardboard_box'].x, 1.58)
        self.assertAlmostEqual(zones['cardboard_box'].y, 1.44)
        self.assertEqual(zones['cardboard_box'].radius, OBSTACLE_ZONE_RADII_M['cardboard_box'])
        self.assertEqual(len(notified), 1)
        self.assertTrue(inp.done)
        calls = len(looked_up)
        tf[0] = (3.0, -1.0, 1.0)                                # 이후 TF 가 크게 보정돼도 갱신하지 않는다
        inp.refresh()
        inp.refresh()
        self.assertEqual(len(notified), 1)
        self.assertEqual(len(looked_up), calls)                 # TF 를 다시 보지도 않는다
        self.assertAlmostEqual({z.name: z for z in inp.zones()}['cardboard_box'].x, 1.58)

    def test_zone_mask_radius(self):
        meta = MapMeta(0.05, -2.0, -2.0, 80, 80, 0.5)
        mask = zone_mask(meta, [Zone('x', 0.3, 0.2, 0.25)])
        self.assertTrue(mask[meta.pixel(0.3, 0.2)])
        self.assertTrue(mask[meta.pixel(0.3 + 0.2, 0.2)])
        self.assertFalse(mask[meta.pixel(0.3 + 0.35, 0.2)])
        self.assertAlmostEqual(np.count_nonzero(mask) * 0.05 ** 2, math.pi * 0.25 ** 2, delta=0.03)

    def make_world(self):
        meta = MapMeta(0.05, 0.0, 0.0, 80, 80)
        free = np.ones((80, 80), bool)
        zone = zone_mask(meta, [Zone('x', 2.0, 2.0, 0.45)])
        radius, clearance = 0.1, 0.02
        safe = traversable_map(free, 0.05, radius, clearance)
        plan_safe = traversable_map(free & ~zone, 0.05, radius, clearance + 0.1)
        zone_stop = safe & ~traversable_map(free & ~zone, 0.05, radius, clearance)
        return meta, free, zone, safe, plan_safe, zone_stop

    def test_home_route_avoids_zone(self):
        meta, free, zone, safe, plan_safe, zone_stop = self.make_world()
        start, goal = meta.pixel(0.6, 2.0), meta.pixel(3.4, 2.0)
        route = plan_route(plan_safe, safe, safe & ~zone_stop, start, goal, 500000)
        world = [meta.world(*p) for p in route]
        self.assertGreater(len(world), 2)                  # 직선이 아니라 우회한다
        for a, b in zip(world, world[1:]):
            self.assertTrue(line_is_safe(a, b, meta, ~zone_stop))
        self.assertEqual(route[0], start)
        self.assertEqual(route[-1], goal)

    def test_route_leaves_zone_first(self):
        meta, free, zone, safe, plan_safe, zone_stop = self.make_world()
        start, goal = meta.pixel(2.0, 2.0), meta.pixel(3.4, 3.4)   # zone 한가운데에서 시작
        self.assertFalse(plan_safe[start])
        route = plan_route(plan_safe, safe, safe & ~zone_stop, start, goal, 500000)
        world = [meta.world(*p) for p in route]
        self.assertTrue(plan_safe[route[-1]])
        self.assertTrue(all(line_is_safe(a, b, meta, safe) for a, b in zip(world, world[1:])))

    def test_route_rejects_home_inside_zone(self):
        meta, free, zone, safe, plan_safe, zone_stop = self.make_world()
        with self.assertRaises(PlanningError):
            plan_route(plan_safe, safe, safe & ~zone_stop, meta.pixel(0.6, 2.0),
                       meta.pixel(2.0, 2.0), 500000)

    def test_map_updater_cancels_active_leg_through_zone(self):
        from types import SimpleNamespace
        import obstacle_zones
        from cleaner_state import CleanerState
        params = SimpleNamespace(**default_parameters())
        validate_parameters(params)
        state = CleanerState(params)
        warnings = []
        ctx = SimpleNamespace(
            params=params, state=state,
            logger=SimpleNamespace(warning=warnings.append, info=lambda m: None),
            warn=lambda k, m: None, executor=_FakeExecutor(),
            home=SimpleNamespace(on_zone_change=lambda: None),
            status=SimpleNamespace(publish_zone_map=lambda: None))
        ctx.area = __import__('planning_area').PlanningArea(ctx)
        spawn_file = tempfile.NamedTemporaryFile('w', suffix='.launch.py', delete=False)
        self.addCleanup(os.unlink, spawn_file.name)
        spawn_file.write('objects = (("urine_puddle", sdf, "2.0", "2.0", "0.0"),)\n')
        spawn_file.close()
        params.obstacle_spawn_file = spawn_file.name
        ctx.logger.error = print
        ctx.obstacles = obstacle_zones.ObstacleZoneInput(ctx, lambda target, source: (0.0, 0.0, 0.0))
        updater = MapUpdater(ctx)
        ctx.map_updater = updater
        meta = MapMeta(0.05, 0.0, 0.0, 80, 80)
        free = np.ones((80, 80), bool)
        updater._apply(meta, None, free, 'map_changed')
        self.assertIsNone(state.zone_stop)
        # 활성 경로가 (2, 2) 를 지난다.
        state.active_leg = ((0.6, 2.0), (3.4, 2.0))
        state.active_escape = False
        state.legs.append(((3.4, 2.0), (3.4, 3.0)))
        ctx.obstacles.refresh()                                  # 스폰 위치 -> zone -> on_zones_changed
        self.assertEqual(ctx.executor.cancelled, ['obstacle_zone'])
        self.assertFalse(state.legs)
        self.assertTrue(state.zone[meta.pixel(2.0, 2.0)])
        self.assertFalse(state.free[meta.pixel(2.0, 2.0)])
        self.assertTrue(state.safe[meta.pixel(2.0, 2.0)])      # 물리 안전 영역은 그대로
        self.assertFalse(state.plan_safe[meta.pixel(2.0, 2.0)])
        # 새 coverage 계획은 구역을 지나지 않는다.
        start = meta.pixel(0.6, 0.6)
        plan = plan_coverage(state.plan_safe, state.free, meta, start,
                             np.zeros_like(free), 0.25, 0.15)
        world = [meta.world(*p) for p in plan.pixels]
        for a, b in zip(world, world[1:]):
            self.assertFalse(ctx.area.leg_hits_zone(a, b))
        self.assertFalse(np.any(plan.target & state.zone))


def run() -> int:
    suite = unittest.TestSuite([
        unittest.defaultTestLoader.loadTestsFromTestCase(CoverageTests),
        unittest.defaultTestLoader.loadTestsFromTestCase(ObstacleZoneTests)])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1
