#!/usr/bin/env python3
"""Inspect DWB trajectory scores around a suspected Nav2 stall.

Usage: python3 inspect_dwb_scores.py nav2_debug 19 20 23 30 40 70
Run in a ROS 2 environment with rosbag2_py, rclpy and dwb_msgs installed.
"""
import argparse
from pathlib import Path
from collections import Counter
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def fmt(traj):
    v = traj.traj.velocity
    return f'vx={v.x:+.3f} m/s, wz={v.theta:+.3f} rad/s, score={traj.total:.3f}'


def breakdown(traj):
    return {s.name: (float(s.raw_score), float(s.scale), float(s.raw_score * s.scale)) for s in traj.scores}


def main():
    p = argparse.ArgumentParser(description='Compare chosen DWB trajectory with best legal forward option.')
    p.add_argument('bag', help='Directory containing rosbag metadata.yaml')
    p.add_argument('seconds', nargs='*', type=int, default=[19, 20, 23, 30, 40, 70])
    p.add_argument('--forward-threshold', type=float, default=0.05)
    args = p.parse_args()
    md = yaml.safe_load((Path(args.bag)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=md['storage_identifier']),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types={a.name:a.type for a in reader.get_all_topics_and_types()}
    if '/evaluation' not in types:
        raise SystemExit('No /evaluation in this bag')
    msg_type=get_message(types['/evaluation'])
    evaluations={}
    start_ns=None
    targets=set(args.seconds)
    while reader.has_next():
        topic,raw,t=reader.read_next()
        if start_ns is None:
            start_ns=t
        if topic=='/evaluation':
            sec=int((t-start_ns)/1_000_000_000)
            if sec in targets:
                evaluations[sec]=deserialize_message(raw,msg_type)
    for sec in sorted(targets):
        msg=evaluations.get(sec)
        print('\n'+'='*65+f'\nTime {sec}s after bag start')
        if msg is None:
            print('No /evaluation message that second');continue
        tr=msg.twists
        valid=[x for x in tr if x.total>=0]
        invalid=[x for x in tr if x.total<0]
        reasons=Counter(x.scores[-1].name if x.scores else 'unknown' for x in invalid)
        forward=[x for x in valid if x.traj.velocity.x>args.forward_threshold]
        if not tr or msg.best_index>=len(tr):
            print('Invalid or missing best_index');continue
        best=tr[msg.best_index]
        print(f'Legal {len(valid)}/{len(tr)}, legal forward vx>{args.forward_threshold}: {len(forward)}, rejected: {dict(reasons)}')
        print('CHOSEN:',fmt(best))
        competitor=min(forward,key=lambda x:x.total) if forward else None
        if competitor is not None:
            print('BEST LEGAL FORWARD:',fmt(competitor))
        else:
            print('No legal forward trajectory exceeds threshold')
        a,b=breakdown(best),breakdown(competitor) if competitor is not None else {}
        print(f'{"Critic":22s} {"Chosen weighted":>16s} {"Forward weighted":>16s} {"Forward - chosen":>18s}')
        for name in dict.fromkeys([*a.keys(),*b.keys()]):
            av=a.get(name,(0,0,float('nan')))[2]
            bv=b.get(name,(0,0,float('nan')))[2]
            print(f'{name:22s} {av:16.3f} {bv:16.3f} {bv-av:18.3f}')
        print('Note: with short-circuit trajectory scoring, some critics are not evaluated for worse candidates.')


if __name__=='__main__':
    main()

