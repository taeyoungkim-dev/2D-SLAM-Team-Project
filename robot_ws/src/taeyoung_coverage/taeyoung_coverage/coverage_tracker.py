"""로봇의 실제 이동 궤적을 기반으로 방문 영역(visited_map)을 추적한다.

README_coverage.md 8장: "로봇 중심의 한 cell만 visited 처리하지 말고, 실제
Coverage width를 고려하여 주변 영역도 visited 처리". 이 모듈도 ROS2에
의존하지 않는다 - :class:`CoverageTracker` 는 world (x, y) 좌표만 입력으로
받고, 실제 TF 조회는 ``coverage_node.py`` 가 담당한다.
"""

from __future__ import annotations

import numpy as np

from . import map_utils
from .map_utils import MapMeta


class CoverageTracker:
    """방문한 영역을 누적 기록하고, 남은(미청소) 영역을 계산한다."""

    def __init__(self, meta: MapMeta, coverage_width_m: float) -> None:
        """
        Args:
            meta: 현재 지도의 메타데이터.
            coverage_width_m: 로봇이 한 번 지나가면서 실제로 청소하는 폭(미터).
                이 값의 반경만큼 로봇 중심 주변 셀도 함께 visited 처리한다.
        """
        self.meta = meta
        self.coverage_width_m = coverage_width_m
        self.visited = np.zeros((meta.height, meta.width), dtype=bool)
        self._radius_cells = max(1, round((coverage_width_m / 2.0) / meta.resolution))
        self._kernel = map_utils.make_disk_kernel(self._radius_cells)

    def resize_if_needed(self, meta: MapMeta) -> None:
        """지도 크기가 바뀌면(재매핑 등) visited map을 새로 초기화한다."""
        if meta.width != self.meta.width or meta.height != self.meta.height:
            self.meta = meta
            self.visited = np.zeros((meta.height, meta.width), dtype=bool)
        else:
            self.meta = meta

    def update_from_world_pose(self, world_x: float, world_y: float) -> None:
        """world (x, y) 위치를 기준으로 coverage_width 반경 영역을 visited로 표시."""
        y, x = map_utils.world_to_pixel(world_x, world_y, self.meta)
        h, w = self.visited.shape
        if not (0 <= y < h and 0 <= x < w):
            return

        r = self._radius_cells
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        x0, x1 = max(0, x - r), min(w, x + r + 1)

        # 지도 경계에서 커널이 잘리는 경우, 커널에서도 동일한 만큼 잘라서 맞춘다.
        ky0 = y0 - (y - r)
        kx0 = x0 - (x - r)
        kernel_slice = self._kernel[ky0 : ky0 + (y1 - y0), kx0 : kx0 + (x1 - x0)]

        self.visited[y0:y1, x0:x1] |= kernel_slice

    def remaining_map(self, cleanable_map: np.ndarray) -> np.ndarray:
        """아직 방문하지 않은 청소 대상 영역 = cleanable_map & ~visited."""
        return cleanable_map & ~self.visited

    def coverage_ratio(self, cleanable_map: np.ndarray) -> float:
        """cleanable_map 중 실제로 방문한 비율 (0.0~1.0)."""
        total = int(np.count_nonzero(cleanable_map))
        if total == 0:
            return 1.0
        done = int(np.count_nonzero(cleanable_map & self.visited))
        return done / total
