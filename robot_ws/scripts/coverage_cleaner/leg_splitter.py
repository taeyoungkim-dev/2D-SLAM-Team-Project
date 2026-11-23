"""월드 경로를 FollowPath 한 구간(leg) 단위로 분할 (ROS 불필요)."""
from __future__ import annotations

import math

from coverage_types import World


def split_legs(world: list[World], max_length: float) -> list[tuple[World, World]]:
    legs = []
    for a, b in zip(world, world[1:]):
        n = max(1, math.ceil(math.dist(a, b) / max_length))
        last = a
        for i in range(1, n + 1):
            point = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
            if math.dist(last, point) > 1e-9:
                legs.append((last, point))
            last = point
    return legs
