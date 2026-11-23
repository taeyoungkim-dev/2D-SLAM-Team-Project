"""공용 타입, 예외, 지도 메타데이터 (ROS 불필요)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Pixel = tuple[int, int]       # (row=y, column=x)
World = tuple[float, float]   # (x, y), meters


class PlanningError(RuntimeError):
    pass


class PlanningMarginError(PlanningError):
    """Stop-safe pose, but insufficient clearance to start a coverage plan."""


@dataclass(frozen=True)
class MapMeta:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    yaw: float = 0.0
    frame: str = 'map'

    def validate(self):
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0
                or not self.frame or not all(math.isfinite(v) for v in
                    (self.resolution, self.origin_x, self.origin_y, self.yaw))):
            raise ValueError('Invalid OccupancyGrid metadata')

    def local(self, x: float, y: float) -> World:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy) / self.resolution, (-s * dx + c * dy) / self.resolution

    def world(self, y: float, x: float) -> World:
        lx, ly = (x + 0.5) * self.resolution, (y + 0.5) * self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly

    def pixel(self, x: float, y: float) -> Pixel:
        lx, ly = self.local(x, y)
        return math.floor(ly), math.floor(lx)


@dataclass
class Plan:
    pixels: list[Pixel]
    target: np.ndarray
    predicted: np.ndarray
    reachable: np.ndarray


@dataclass(frozen=True)
class TrackingDiagnostic:
    reason: str
    details: str
