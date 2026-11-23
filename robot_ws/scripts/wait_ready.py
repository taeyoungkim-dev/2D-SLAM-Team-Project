#!/usr/bin/env python3
"""조건이 충족될 때까지 기다렸다가 종료한다(통합 launch 의 실행 순서 제어용).

    --odom   로봇이 스폰되어 /odom 이 나올 때까지 (Gazebo + 로봇 준비)
    --nav2   Nav2 의 lifecycle manager 들이 모두 active 이고 TF map -> base_footprint 가 구해질 때까지
             (맵 서버, AMCL 위치 추정, 경로 추종이 모두 준비됨)

종료 코드: 0 = 준비됨, 1 = 제한 시간 초과(launch 가 전체를 종료한다).
    python3 wait_ready.py --odom --nav2 --timeout 300
"""
import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

LIFECYCLE_MANAGERS = ('/lifecycle_manager_localization', '/lifecycle_manager_navigation')


class Waiter(Node):
    def __init__(self, args):
        super().__init__('wait_ready')
        self.args = args
        self.odom_seen = False
        self.active = {name: False for name in LIFECYCLE_MANAGERS}
        self.pending = {}
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.lm_clients = {name: self.create_client(Trigger, name + '/is_active') for name in LIFECYCLE_MANAGERS}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_odom(self, _msg):
        self.odom_seen = True

    def _check_manager(self, name):
        if self.active[name] or name in self.pending or not self.lm_clients[name].service_is_ready():
            return
        future = self.lm_clients[name].call_async(Trigger.Request())
        self.pending[name] = future

        def done(f):
            self.pending.pop(name, None)
            if f.result() is not None and f.result().success:
                self.active[name] = True

        future.add_done_callback(done)

    def missing(self):
        """아직 충족되지 않은 조건의 설명 목록 (비어 있으면 준비 완료)."""
        out = []
        if self.args.odom and not self.odom_seen:
            out.append('/odom')
        if self.args.nav2:
            for name in LIFECYCLE_MANAGERS:
                self._check_manager(name)
                if not self.active[name]:
                    out.append(name + ' active')
            if not self.tf_buffer.can_transform('map', self.args.base_frame, rclpy.time.Time()):
                out.append(f'TF map->{self.args.base_frame}')
        return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom', action='store_true')
    parser.add_argument('--nav2', action='store_true')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=300.0)
    args, ros_args = parser.parse_known_args()
    if not (args.odom or args.nav2):
        parser.error('--odom 또는 --nav2 중 하나는 필요합니다')
    rclpy.init(args=ros_args)
    node = Waiter(args)
    start = last_log = time.monotonic()
    code = 1
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            missing = node.missing()
            if not missing:
                node.get_logger().info(f'준비 완료 ({time.monotonic() - start:.1f}s)')
                code = 0
                break
            if time.monotonic() - last_log > 10.0:
                last_log = time.monotonic()
                node.get_logger().info('기다리는 중: ' + ', '.join(missing))
        else:
            node.get_logger().error(f'{args.timeout:.0f}초 안에 준비되지 않음: ' + ', '.join(node.missing()))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
