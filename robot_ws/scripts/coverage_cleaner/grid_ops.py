"""격자(numpy mask) 기본 연산 (ROS 불필요)."""
from __future__ import annotations

import math

import numpy as np

from coverage_types import Pixel


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


def intervals(row: np.ndarray):
    edges = np.diff(np.r_[False, row, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1))
