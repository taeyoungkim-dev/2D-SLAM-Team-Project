"""홈 복귀 판정용 순수 함수 (ROS 불필요, self_test 에서 검사한다)."""
from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """각도를 [-pi, pi] 로 정규화한다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def home_error(pose_xy, yaw: float, home) -> tuple[float, float]:
    """(위치 오차 m, 부호 있는 yaw 오차 rad). yaw 오차 = 돌아야 할 각도 (home - 현재)."""
    return math.dist(pose_xy, home[:2]), wrap_angle(home[2] - yaw)


def at_home(pose_xy, yaw: float, home, xy_tolerance: float, yaw_tolerance: float) -> bool:
    distance, yaw_error = home_error(pose_xy, yaw, home)
    return distance <= xy_tolerance and abs(yaw_error) <= yaw_tolerance
