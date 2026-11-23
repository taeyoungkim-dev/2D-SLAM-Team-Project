"""free 지도 -> 주행 가능(safe) 지도, 시작점에서 도달 가능한 영역 계산 (ROS 불필요)."""
from __future__ import annotations

import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from coverage_types import Pixel, PlanningError
from grid_ops import inside

CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)


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
