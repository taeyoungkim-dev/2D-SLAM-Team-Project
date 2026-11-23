"""월드 좌표 리스트 -> nav_msgs/Path 변환."""
from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def build_path_message(world, meta, clock, path_resolution):
    msg = Path()
    msg.header.frame_id = meta.frame
    msg.header.stamp = clock.now().to_msg()
    dense = []
    spacing = min(path_resolution, meta.resolution / 2)
    for a, b in zip(world, world[1:]):
        n = max(1, math.ceil(math.dist(a, b) / spacing))
        dense.extend((a[0] + (b[0] - a[0]) * i / n,
                      a[1] + (b[1] - a[1]) * i / n) for i in range(n))
    if world:
        dense.append(world[-1])
    yaw = 0.0
    for i, (x, y) in enumerate(dense):
        if i + 1 < len(dense):
            yaw = math.atan2(dense[i+1][1] - y, dense[i+1][0] - x)
        point = PoseStamped()
        point.header = msg.header
        point.pose.position.x, point.pose.position.y = float(x), float(y)
        point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.poses.append(point)
    return msg
