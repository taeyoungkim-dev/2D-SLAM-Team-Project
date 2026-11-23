#!/usr/bin/env python3
"""/coverage_obstacle_zone_map 이 제대로 만들어졌는지 숫자로 검사한다 (rviz2 없이도 된다).

실행: source /opt/ros/humble/setup.bash && python3 robot_ws/scripts/check_zone_map.py [--once]
  zone 지도(OccupancyGrid, 100=구역)의 연결 덩어리마다 중심/면적을 구해
  /coverage_status 의 obstacle_zones(중앙값 위치, 반지름)와 비교하고 PASS/FAIL 을 출력한다.
  기대 면적은 원 면적 pi*r^2 이고 5% 이내(격자 오차)면 PASS, 중심은 한 셀 + 1cm 이내면 PASS.
  지도 경계에 걸려 잘린 구역은 면적 비교를 건너뛴다.
"""
import json
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import label
from std_msgs.msg import String

RETAINED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Checker(Node):
    def __init__(self, once):
        super().__init__('check_zone_map')
        self.once, self.status, self.grid = once, None, None
        self.create_subscription(OccupancyGrid, '/coverage_obstacle_zone_map', self.on_grid, RETAINED)
        self.create_subscription(String, '/coverage_status', self.on_status, RETAINED)
        self.last_print = None
        self.create_timer(1.0, self.report)
        self.get_logger().info('/coverage_obstacle_zone_map 과 /coverage_status 를 기다리는 중...')

    def on_grid(self, msg):
        self.grid = msg

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def report(self):
        if self.grid is None or self.status is None:
            return
        g = self.grid
        info = g.info
        res, w, h = info.resolution, info.width, info.height
        data = np.asarray(g.data, dtype=np.int16).reshape(h, w)
        mask = data >= 100
        q = info.origin.orientation
        yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        c, s = math.cos(yaw), math.sin(yaw)

        def world(px, py):   # 셀 좌표(소수) -> map 좌표
            lx, ly = px * res, py * res
            return info.origin.position.x + c * lx - s * ly, info.origin.position.y + s * lx + c * ly

        labels, count = label(mask)
        comps = []
        for i in range(1, count + 1):
            ys, xs = np.nonzero(labels == i)
            cx, cy = world(xs.mean() + 0.5, ys.mean() + 0.5)
            touches = xs.min() == 0 or ys.min() == 0 or xs.max() == w - 1 or ys.max() == h - 1
            comps.append(dict(x=cx, y=cy, area=len(xs) * res * res, clipped=touches))
        zones = self.status.get('obstacle_zones', [])
        lines = [f"frame={g.header.frame_id} grid={w}x{h}@{res}m  구역 셀={int(mask.sum())}  "
                 f"덩어리={count}  status 의 zone={len(zones)}"]
        ok = count == len(zones) and bool(zones)
        used = set()
        for z in zones:
            best = min((i for i in range(len(comps)) if i not in used), default=None,
                       key=lambda i: math.dist((comps[i]['x'], comps[i]['y']), (z['x'], z['y'])))
            if best is None:
                lines.append(f"  FAIL {z['cls']}: 지도에 대응하는 덩어리가 없음")
                ok = False
                continue
            used.add(best)
            comp = comps[best]
            err = math.dist((comp['x'], comp['y']), (z['x'], z['y']))
            want = math.pi * z['radius'] ** 2
            # 격자 양자화: 반지름이 ±반 칸 이내면 통과(작은 구역은 5% 보다 넓게 허용)
            lo = math.pi * max(z['radius'] - res / 2, 0.0) ** 2
            hi = math.pi * (z['radius'] + res / 2) ** 2
            area_ok = comp['clipped'] or abs(comp['area'] - want) / want <= 0.05 or lo <= comp['area'] <= hi
            pos_ok = err <= res + 0.01 or comp['clipped']
            ok &= area_ok and pos_ok
            lines.append(f"  {'PASS' if area_ok and pos_ok else 'FAIL'} {z['cls']:14s} "
                         f"status=({z['x']:.2f},{z['y']:.2f}) r={z['radius']:.2f}  "
                         f"map 중심=({comp['x']:.2f},{comp['y']:.2f}) 오차={err * 100:.1f}cm  "
                         f"면적={comp['area']:.3f}m^2 (기대 {want:.3f})"
                         f"{' (경계에서 잘림)' if comp['clipped'] else ''}")
        if not zones:
            lines.append('  zone 이 아직 없음: coverage_cleaner 가 떠 있는지, /explore_map 이 오는지, '
                         'TF map<-odom 이 있는지 확인')
        text = '\n'.join(lines)
        if text != self.last_print:
            print(text, flush=True)
            self.last_print = text
        if self.once:
            print('결과:', 'PASS' if ok else 'FAIL', flush=True)
            raise SystemExit(0 if ok else 1)


def main():
    rclpy.init()
    node = Checker('--once' in sys.argv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = exc.code if isinstance(exc, SystemExit) else 0
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(code)


if __name__ == '__main__':
    main()
