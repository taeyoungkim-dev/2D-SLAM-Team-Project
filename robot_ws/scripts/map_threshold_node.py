#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapThresholdNode(Node):
    def __init__(self, source_topic, output_topic, free_max, occupied_min):
        super().__init__('map_threshold_node')

        if not (0 <= free_max < occupied_min <= 100):
            raise ValueError(
                f'Invalid thresholds: require 0 <= free_max < occupied_min <= 100, '
                f'got free_max={free_max}, occupied_min={occupied_min}'
            )

        self.source_topic = source_topic
        self.output_topic = output_topic
        self.free_max = free_max
        self.occupied_min = occupied_min
        self.msg_count = 0

        # Volatile subscriber is compatible with both volatile and transient-local
        # map publishers, and we only need current/future /map messages.
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            #durability=DurabilityPolicy.VOLATILE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Keep the latest converted map available for late subscribers.
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            pub_qos,
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.source_topic,
            self.map_callback,
            sub_qos,
        )

        self.get_logger().info(
            f'Converting {self.source_topic} -> {self.output_topic} | '
            f'free: 0..{self.free_max} -> 0, '
            f'unknown: {self.free_max + 1}..{self.occupied_min - 1} and -1 -> -1, '
            f'occupied: {self.occupied_min}..100 -> 100'
        )

    def map_callback(self, msg: OccupancyGrid):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info

        converted = []
        unknown_count = 0
        free_count = 0
        occupied_count = 0

        for value in msg.data:
            if value < 0:
                new_value = -1
                unknown_count += 1
            elif value <= self.free_max:
                new_value = 0
                free_count += 1
            elif value >= self.occupied_min:
                new_value = 100
                occupied_count += 1
            else:
                # Optional uncertainty band between free_max and occupied_min.
                new_value = -1
                unknown_count += 1

            converted.append(new_value)

        out.data = converted
        self.publisher.publish(out)

        self.msg_count += 1
        if self.msg_count == 1 or self.msg_count % 10 == 0:
            self.get_logger().info(
                f'Published map #{self.msg_count}: '
                f'free={free_count}, occupied={occupied_count}, '
                f'unknown={unknown_count}, total={len(converted)}'
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert a probabilistic OccupancyGrid into a binary map for explore_lite.'
    )
    parser.add_argument(
        '--source',
        default='/map',
        help='Input OccupancyGrid topic (default: /map)',
    )
    parser.add_argument(
        '--output',
        default='/explore_map',
        help='Output OccupancyGrid topic (default: /explore_map)',
    )
    parser.add_argument(
        '--free-max',
        type=int,
        default=49,
        help='Values <= this become FREE(0). Default: 49',
    )
    parser.add_argument(
        '--occupied-min',
        type=int,
        default=50,
        help='Values >= this become OCCUPIED(100). Default: 50',
    )

    args, ros_args = parser.parse_known_args()
    return args, ros_args


def main():
    args, ros_args = parse_args()

    rclpy.init(args=ros_args)
    node = MapThresholdNode(
        source_topic=args.source,
        output_topic=args.output,
        free_max=args.free_max,
        occupied_min=args.occupied_min,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
