"""OccupancyGrid 메시지 -> (MapMeta, free mask) 변환. ROS import 없이 메시지를 duck-typing 한다."""
from __future__ import annotations

import math

import numpy as np

from coverage_types import MapMeta


def parse_occupancy_grid(msg, occupied_threshold: int):
    """잘못된 지도면 ValueError/TypeError 를 던진다."""
    q = msg.info.origin.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-8:
        raise ValueError('Map origin quaternion has zero norm')
    qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    if abs(qx) > 1e-6 or abs(qy) > 1e-6:
        raise ValueError('Only planar map origins are supported')
    yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
    meta = MapMeta(msg.info.resolution, msg.info.origin.position.x,
                   msg.info.origin.position.y, msg.info.width, msg.info.height,
                   yaw, msg.header.frame_id)
    meta.validate()
    grid = np.asarray(msg.data, dtype=np.int16).reshape(meta.height, meta.width)
    if np.any((grid < -1) | (grid > 100)):
        raise ValueError('OccupancyGrid values must be -1 or 0..100')
    free = (grid >= 0) & (grid < occupied_threshold)
    return meta, free
