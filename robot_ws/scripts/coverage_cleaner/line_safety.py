"""두 월드 좌표 사이 직선이 safe 셀만 지나는지 검사 (ROS 불필요)."""
from __future__ import annotations

import math

import numpy as np

from coverage_types import MapMeta, World
from grid_ops import inside


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
