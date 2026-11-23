"""실제 TF 이동으로 방문(visited) 셀을 기록하는 추적기 (ROS 불필요)."""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from coverage_types import MapMeta, TrackingDiagnostic, World
from grid_ops import inside
from line_safety import line_is_safe
from parameters.thresholds import (THRESHOLD_DEFAULTS, TRACKER_DISTANCE_SLACK_M,
                        TRACKER_SAME_STAMP_MOTION_LIMIT_M)


class CoverageTracker:
    def __init__(self, meta: MapMeta, width: float):
        self.meta, self.width = meta, width
        self.visited = np.zeros((meta.height, meta.width), dtype=bool)
        self.previous: Optional[tuple[World, float]] = None
        self.last_diagnostic: Optional[TrackingDiagnostic] = None

    def rebase(self, meta: MapMeta) -> bool:
        """Preserve only an exact grid-to-grid correspondence; return whether preserved."""
        old = self.meta
        if old == meta:
            return True
        new = np.zeros((meta.height, meta.width), dtype=bool)
        ox, oy = old.local(meta.origin_x, meta.origin_y)
        compatible = (old.frame == meta.frame and abs(old.yaw - meta.yaw) < 1e-8
                      and abs(old.resolution - meta.resolution) < 1e-9
                      and abs(ox - round(ox)) < 1e-6 and abs(oy - round(oy)) < 1e-6)
        if compatible:
            dx, dy = round(ox), round(oy)
            nx0, ny0 = max(0, -dx), max(0, -dy)
            nx1, ny1 = min(meta.width, old.width - dx), min(meta.height, old.height - dy)
            if nx1 > nx0 and ny1 > ny0:
                new[ny0:ny1, nx0:nx1] = self.visited[ny0 + dy:ny1 + dy, nx0 + dx:nx1 + dx]
        self.meta, self.visited, self.previous = meta, new, None
        self.last_diagnostic = None
        return compatible

    def stamp_segment(self, a: World, b: World, free: np.ndarray):
        """Cell centres within the real swept capsule, without rounding to robot cell."""
        ax, ay = self.meta.local(*a)
        bx, by = self.meta.local(*b)
        r = self.width / (2 * self.meta.resolution)
        x0, x1 = max(0, math.floor(min(ax, bx) - r)), min(self.meta.width, math.ceil(max(ax, bx) + r))
        y0, y1 = max(0, math.floor(min(ay, by) - r)), min(self.meta.height, math.ceil(max(ay, by) + r))
        if x1 <= x0 or y1 <= y0:
            return
        yy, xx = np.mgrid[y0:y1, x0:x1].astype(float)
        xx, yy = xx + 0.5, yy + 0.5
        vx, vy = bx - ax, by - ay
        length2 = vx * vx + vy * vy
        t = np.clip(((xx - ax) * vx + (yy - ay) * vy) / length2, 0, 1) if length2 else 0.0
        covered = (xx - ax - t * vx) ** 2 + (yy - ay - t * vy) ** 2 <= r * r + 1e-10
        self.visited[y0:y1, x0:x1] |= covered & free[y0:y1, x0:x1]

    def update(self, pose: World, stamp_sec: float, free: np.ndarray, safe: np.ndarray,
               max_speed: float = THRESHOLD_DEFAULTS['tracker_max_speed'],
               max_gap: float = THRESHOLD_DEFAULTS['tracker_max_gap_sec'],
               check_safe_position: bool = True) -> bool:
        """Keep the original bool contract; last_diagnostic explains any rejection."""
        self.last_diagnostic = None
        previous = self.previous
        self.previous = pose, stamp_sec
        pixel = self.meta.pixel(*pose)
        if check_safe_position and not inside(safe, pixel):
            in_bounds = 0 <= pixel[0] < safe.shape[0] and 0 <= pixel[1] < safe.shape[1]
            self.last_diagnostic = TrackingDiagnostic(
                'unsafe_pose',
                f'robot_xy=({pose[0]:.6f},{pose[1]:.6f})m; '
                f'grid_cell_yx={pixel}; in_bounds={in_bounds}; safe=False; '
                f'free={inside(free, pixel)}; map_shape_hw={safe.shape}; '
                f'current_stamp={stamp_sec:.9f}s')
            return False
        a = pose
        continuous = True
        if previous is not None:
            old, ts = previous
            dt = stamp_sec - ts
            dist = math.dist(pose, old)
            if dt < 0:
                continuous = False
                self.last_diagnostic = TrackingDiagnostic(
                    'time_reverse',
                    f'dt={dt:.9f}s < 0s; previous_stamp={ts:.9f}s; '
                    f'current_stamp={stamp_sec:.9f}s; distance={dist:.9f}m')
            elif dt == 0 and dist > TRACKER_SAME_STAMP_MOTION_LIMIT_M:
                continuous = False
                self.last_diagnostic = TrackingDiagnostic(
                    'same_stamp_motion',
                    f'dt={dt:.9f}s; distance={dist:.9f}m > '
                    f'limit={TRACKER_SAME_STAMP_MOTION_LIMIT_M:.9f}m; '
                    f'previous_stamp={ts:.9f}s; current_stamp={stamp_sec:.9f}s')
            elif 0 < dt <= max_gap:
                allowed = max_speed * dt + TRACKER_DISTANCE_SLACK_M
                if dist <= allowed:
                    if not check_safe_position or line_is_safe(old, pose, self.meta, safe):
                        a = old
                    else:
                        continuous = False
                        self.last_diagnostic = TrackingDiagnostic(
                            'unsafe_segment',
                            f'previous_xy=({old[0]:.6f},{old[1]:.6f})m; '
                            f'robot_xy=({pose[0]:.6f},{pose[1]:.6f})m; '
                            f'distance={dist:.9f}m; dt={dt:.9f}s; '
                            f'allowed_distance={allowed:.9f}m; segment_safe=False; '
                            f'previous_stamp={ts:.9f}s; current_stamp={stamp_sec:.9f}s')
                else:
                    continuous = False
                    self.last_diagnostic = TrackingDiagnostic(
                        'speed_limit',
                        f'distance={dist:.9f}m > allowed_distance={allowed:.9f}m; '
                        f'dt={dt:.9f}s; apparent_speed={dist / dt:.6f}m/s; '
                        f'max_speed={max_speed:.6f}m/s; '
                        f'distance_slack={TRACKER_DISTANCE_SLACK_M:.6f}m; '
                        f'previous_stamp={ts:.9f}s; current_stamp={stamp_sec:.9f}s')
            elif dt > max_gap:
                continuous = False
                self.last_diagnostic = TrackingDiagnostic(
                    'tf_gap',
                    f'dt={dt:.9f}s > max_gap={max_gap:.9f}s; '
                    f'previous_stamp={ts:.9f}s; current_stamp={stamp_sec:.9f}s; '
                    f'distance={dist:.9f}m')
            # A long observation gap only stamps the current position and replans.
        self.stamp_segment(a, pose, free)
        return continuous
