#!/usr/bin/env python3
"""Summarize Nav2 rosbag topics by second; write nav2_timeline.csv.
Usage: python3 analyze_nav2_bag.py nav2_debug
Run in a Python environment with ROS 2 sourced.
"""
import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path

import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = {'/cmd_vel', '/cmd_vel_nav', '/evaluation', '/odom'}


def new_bucket():
    return {'nav': None, 'cmd': None, 'odom_first': None,
            'odom_last': None, 'evaluation': None}


def evaluate(msg):
    all_trajectories = msg.twists
    legal = [item for item in all_trajectories if item.total >= 0]
    rejected = Counter()
    for item in all_trajectories:
        if item.total < 0:
            name = item.scores[-1].name if item.scores else 'unknown'
            rejected[name] += 1

    best_v, best_w = None, None
    if 0 <= msg.best_index < len(all_trajectories):
        best = all_trajectories[msg.best_index]
        if best.total >= 0:
            best_v = float(best.traj.velocity.x)
            best_w = float(best.traj.velocity.theta)

    top_rejection = rejected.most_common(1)[0][0] if rejected else ''
    return (len(legal), len(all_trajectories) - len(legal),
            top_rejection, best_v, best_w)


def format_float(value):
    return '' if value is None else f'{value:.3f}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='directory containing metadata.yaml')
    args = parser.parse_args()
    bag = Path(args.bag)
    metadata_file = bag / 'metadata.yaml'
    if not metadata_file.is_file():
        parser.error(f'metadata.yaml not found in {bag}')

    metadata = yaml.safe_load(metadata_file.read_text())['rosbag2_bagfile_information']
    storage_id = metadata['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    type_names = {t.name: t.type for t in reader.get_all_topics_and_types()}
    types = {topic: get_message(type_names[topic])
             for topic in TOPICS if topic in type_names}
    print('Recorded topics:', ', '.join(sorted(types)))
    if '/odom' not in types:
        print('Warning: no /odom; movement cannot be estimated.')
    if '/evaluation' not in types:
        print('Warning: no /evaluation; critic failures cannot be estimated.')

    buckets = defaultdict(new_bucket)
    first_ns = None
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if first_ns is None:
            first_ns = timestamp_ns
        if topic not in types:
            continue
        sec = int((timestamp_ns - first_ns) / 1_000_000_000)
        bucket = buckets[sec]
        msg = deserialize_message(raw, types[topic])
        if topic in ('/cmd_vel', '/cmd_vel_nav'):
            key = 'nav' if topic == '/cmd_vel_nav' else 'cmd'
            bucket[key] = (float(msg.linear.x), float(msg.angular.z))
        elif topic == '/odom':
            xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
            if bucket['odom_first'] is None:
                bucket['odom_first'] = xy
            bucket['odom_last'] = xy
        elif topic == '/evaluation':
            bucket['evaluation'] = evaluate(msg)
    del reader

    if not buckets:
        print('No matching messages in bag.')
        return

    rows = []
    for sec in sorted(buckets):
        b = buckets[sec]
        xy_a, xy_b = b['odom_first'], b['odom_last']
        moved = math.dist(xy_a, xy_b) if xy_a and xy_b else None
        nav, cmd, ev = b['nav'], b['cmd'], b['evaluation']
        rows.append({
            'sec': sec,
            'odom_moved_m': format_float(moved),
            'nav_vx': format_float(nav[0]) if nav else '',
            'nav_wz': format_float(nav[1]) if nav else '',
            'cmd_vx': format_float(cmd[0]) if cmd else '',
            'cmd_wz': format_float(cmd[1]) if cmd else '',
            'legal_trajectories': ev[0] if ev else '',
            'illegal_trajectories': ev[1] if ev else '',
            'top_rejection_critic': ev[2] if ev else '',
            'best_traj_vx': format_float(ev[3]) if ev else '',
            'best_traj_wz': format_float(ev[4]) if ev else '',
        })

    output = 'nav2_timeline.csv'
    with open(output, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'Saved {len(rows)} seconds to {output}')

    # Tentative first stall after robot has already moved. This is a hint,
    # not a definite failure diagnosis; turning in place also matches.
    moved_once = False
    stopped = 0
    first_stall_sec = None
    for row in rows:
        dist_text = row['odom_moved_m']
        if dist_text == '':
            stopped = 0
            continue
        dist = float(dist_text)
        if dist >= 0.020:
            moved_once = True
            stopped = 0
        elif moved_once and dist < 0.005:
            stopped += 1
            if stopped >= 3:
                first_stall_sec = row['sec'] - 2
                break
        else:
            stopped = 0
    if first_stall_sec is None:
        print('No 3-second low-displacement interval detected; inspect CSV manually.')
    else:
        print(f'Candidate first stall: about {first_stall_sec}s after bag start.')
        print('Rows from 5s before to 8s after this interval:')
        w = csv.DictWriter(__import__('sys').stdout, fieldnames=list(rows[0]))
        w.writeheader()
        for row in rows:
            if first_stall_sec - 5 <= row['sec'] <= first_stall_sec + 8:
                w.writerow(row)
    print('Blank cells mean no message received that second (not zero velocity).')


if __name__ == '__main__':
    main()

