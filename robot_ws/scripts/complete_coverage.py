#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener, TransformException
from scipy.ndimage import distance_transform_edt


# ============================================================
# TurtleBot3 Burger / coverage parameters
# ============================================================

ROBOT_RADIUS = 0.089       # Burger width 0.178 m / 2
SAFETY_MARGIN = 0.02       # 벽에서 추가 여유
CLEARANCE = ROBOT_RADIUS + SAFETY_MARGIN

LANE_SPACING = 0.15        # 청소 줄 사이 간격
POINT_SPACING = 0.25       # 같은 줄에서 waypoint 간격

FREE_THRESHOLD = 50        # 0~49 = free
MIN_SEGMENT_LENGTH = 0.15  # 너무 작은 조각은 무시


class SimpleCoverage(Node):

    def __init__(self):
        super().__init__('simple_coverage')

        # publisher setting
        qos = QoSProfile(
            #Queue size : 1
            depth=1,
            # Here to read
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )

        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path',
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.map_msg = None
        self.waypoints = []

        self.current_index = 0
        self.started = False
        self.goal_active = False

        # map, TF, Nav2가 준비됐는지 주기적으로 확인
        self.timer = self.create_timer(
            1.0,
            self.try_start
        )

        self.get_logger().info(
            'Simple coverage node started.'
        )

    # ========================================================
    # MAP
    # ========================================================

    def map_callback(self, msg):

        if self.map_msg is None:
            self.get_logger().info(
                f'Map received: {msg.info.width} x '
                f'{msg.info.height}, '
                f'resolution={msg.info.resolution}'
            )

        self.map_msg = msg

    # ========================================================
    # START
    # ========================================================

    def try_start(self):

        if self.started:
            return

        if self.map_msg is None:
            self.get_logger().info('Waiting for /map...')
            return

        if not self.nav_client.wait_for_server(
                timeout_sec=0.2):

            self.get_logger().info(
                'Waiting for Nav2...'
            )
            return

        robot = self.get_robot_position()

        if robot is None:
            self.get_logger().info(
                'Waiting for map -> base_link TF...'
            )
            return

        self.waypoints = \
            self.generate_coverage_waypoints(robot)

        if not self.waypoints:
            self.get_logger().error(
                'No coverage waypoints generated.'
            )
            return

        self.started = True

        self.get_logger().info(
            f'Coverage plan generated: '
            f'{len(self.waypoints)} waypoints'
        )

        self.publish_path()

        self.send_next_goal()

    # ========================================================
    # TF
    # ========================================================

    def get_robot_position(self):

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            return (
                t.transform.translation.x,
                t.transform.translation.y
            )

        except TransformException:
            return None

    # ========================================================
    # COVERAGE PATH GENERATION
    # ========================================================

    def generate_coverage_waypoints(
            self,
            robot_position):

        msg = self.map_msg

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(
            msg.data,
            dtype=np.int16
        ).reshape((height, width))

        # ----------------------------------------------------
        # free 영역
        # unknown(-1)은 free가 아님
        # ----------------------------------------------------

        free = (
            (grid >= 0) &
            (grid < FREE_THRESHOLD)
        )

        # ----------------------------------------------------
        # 벽 + unknown에서 떨어진 거리 계산
        #
        # distance_transform_edt:
        # free cell → 가장 가까운 non-free cell 거리
        # ----------------------------------------------------

        distance_pixels = \
            distance_transform_edt(free)

        distance_m = distance_pixels * res

        safe = (
            free &
            (distance_m >= CLEARANCE)
        )

        safe_count = np.count_nonzero(safe)

        self.get_logger().info(
            f'Safe free cells: {safe_count}'
        )

        if safe_count == 0:
            return []

        lane_px = max(
            1,
            int(round(LANE_SPACING / res))
        )

        point_px = max(
            1,
            int(round(POINT_SPACING / res))
        )

        min_segment_px = max(
            1,
            int(round(
                MIN_SEGMENT_LENGTH / res
            ))
        )

        ys, xs = np.where(safe)

        min_y = int(np.min(ys))
        max_y = int(np.max(ys))

        # 여러 시작 방향을 만들어보고
        # 현재 로봇에 가장 가까운 첫 goal을 가진
        # 경로를 선택
        candidates = []

        candidates.append(
            self.build_raster_path(
                safe,
                min_y,
                max_y,
                lane_px,
                point_px,
                min_segment_px,
                reverse_rows=False,
                first_direction_right=True
            )
        )

        candidates.append(
            self.build_raster_path(
                safe,
                min_y,
                max_y,
                lane_px,
                point_px,
                min_segment_px,
                reverse_rows=False,
                first_direction_right=False
            )
        )

        candidates.append(
            self.build_raster_path(
                safe,
                min_y,
                max_y,
                lane_px,
                point_px,
                min_segment_px,
                reverse_rows=True,
                first_direction_right=True
            )
        )

        candidates.append(
            self.build_raster_path(
                safe,
                min_y,
                max_y,
                lane_px,
                point_px,
                min_segment_px,
                reverse_rows=True,
                first_direction_right=False
            )
        )

        candidates = [
            p for p in candidates
            if len(p) > 0
        ]

        if not candidates:
            return []

        robot_x, robot_y = robot_position

        def first_distance(path):

            py, px = path[0]

            wx = origin_x + \
                (px + 0.5) * res

            wy = origin_y + \
                (py + 0.5) * res

            return math.hypot(
                wx - robot_x,
                wy - robot_y
            )

        pixel_path = min(
            candidates,
            key=first_distance
        )

        # ----------------------------------------------------
        # pixel → map 좌표
        # ----------------------------------------------------

        world_path = []

        for py, px in pixel_path:

            wx = origin_x + \
                (px + 0.5) * res

            wy = origin_y + \
                (py + 0.5) * res

            world_path.append(
                (wx, wy)
            )

        return world_path

    # ========================================================
    # RASTER / LAWNMOWER
    # ========================================================

    def build_raster_path(
            self,
            safe,
            min_y,
            max_y,
            lane_px,
            point_px,
            min_segment_px,
            reverse_rows=False,
            first_direction_right=True):

        rows = list(
            range(
                min_y,
                max_y + 1,
                lane_px
            )
        )

        if reverse_rows:
            rows.reverse()

        path = []

        direction_right = \
            first_direction_right

        width = safe.shape[1]

        for y in rows:

            row = safe[y]

            # -----------------------------
            # 이 row에서 연속된 free 구간 찾기
            # -----------------------------

            segments = []

            x = 0

            while x < width:

                while x < width and \
                        not row[x]:
                    x += 1

                if x >= width:
                    break

                start = x

                while x < width and row[x]:
                    x += 1

                end = x - 1

                if (
                    end - start + 1
                    >= min_segment_px
                ):
                    segments.append(
                        (start, end)
                    )

            if not segments:
                continue

            # 지그재그
            if direction_right:

                for start, end in segments:

                    xs = list(
                        range(
                            start,
                            end + 1,
                            point_px
                        )
                    )

                    if not xs or \
                            xs[-1] != end:
                        xs.append(end)

                    for px in xs:
                        path.append(
                            (y, px)
                        )

            else:

                for start, end in \
                        reversed(segments):

                    xs = list(
                        range(
                            end,
                            start - 1,
                            -point_px
                        )
                    )

                    if not xs or \
                            xs[-1] != start:
                        xs.append(start)

                    for px in xs:
                        path.append(
                            (y, px)
                        )

            direction_right = \
                not direction_right

        return path

    # ========================================================
    # RVIZ PATH
    # ========================================================

    def publish_path(self):

        msg = Path()

        msg.header.frame_id = 'map'
        msg.header.stamp = \
            self.get_clock().now().to_msg()

        for i, (x, y) in \
                enumerate(self.waypoints):

            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y

            # 다음 waypoint 방향을 향하도록 설정
            if i < len(self.waypoints) - 1:

                nx, ny = \
                    self.waypoints[i + 1]

                yaw = math.atan2(
                    ny - y,
                    nx - x
                )

                pose.pose.orientation.z = \
                    math.sin(yaw / 2.0)

                pose.pose.orientation.w = \
                    math.cos(yaw / 2.0)

            else:
                pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

    # ========================================================
    # NAV2
    # ========================================================

    def send_next_goal(self):

        if self.goal_active:
            return

        if self.current_index >= \
                len(self.waypoints):

            self.get_logger().info(
                '============================'
            )
            self.get_logger().info(
                'Coverage finished.'
            )
            self.get_logger().info(
                '============================'
            )

            return

        x, y = \
            self.waypoints[self.current_index]

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'

        goal.pose.header.stamp = \
            self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # 다음 waypoint 방향
        if self.current_index < \
                len(self.waypoints) - 1:

            nx, ny = \
                self.waypoints[
                    self.current_index + 1
                ]

            yaw = math.atan2(
                ny - y,
                nx - x
            )

            goal.pose.pose.orientation.z = \
                math.sin(yaw / 2.0)

            goal.pose.pose.orientation.w = \
                math.cos(yaw / 2.0)

        else:

            goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Goal '
            f'{self.current_index + 1}/'
            f'{len(self.waypoints)} '
            f'-> ({x:.2f}, {y:.2f})'
        )

        self.goal_active = True

        future = \
            self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
            self,
            future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Goal rejected -> skip'
            )

            self.goal_active = False
            self.current_index += 1

            self.send_next_goal()
            return

        result_future = \
            goal_handle.get_result_async()

        result_future.add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(
            self,
            future):

        status = future.result().status

        if status == \
                GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                'Goal reached.'
            )

        else:

            self.get_logger().warn(
                f'Goal failed '
                f'(status={status}) -> skip'
            )

        self.goal_active = False

        self.current_index += 1

        self.send_next_goal()


def main(args=None):

    rclpy.init(args=args)

    node = SimpleCoverage()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()