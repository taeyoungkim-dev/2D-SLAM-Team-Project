"""A* 경로 탐색과 경로 단순화 (ROS 불필요)."""
from __future__ import annotations

import heapq
import math

import numpy as np

from coverage_types import Pixel, PlanningError
from grid_ops import inside
from parameters.thresholds import THRESHOLD_DEFAULTS

NEIGHBORS = ((0, 1), (1, 0), (0, -1), (-1, 0),
             (1, 1), (1, -1), (-1, 1), (-1, -1))


def astar(safe: np.ndarray, start: Pixel, goal: Pixel,
          max_expansions: int = THRESHOLD_DEFAULTS['max_astar_expansions']) -> list[Pixel]:
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
