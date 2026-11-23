"""TF에서 로봇 위치(pose, stamp_sec)를 읽는 기능.

get() 은 성공하면 ((x, y), stamp_sec), 실패(TF 없음/오래됨/미래/비정상)하면 None 을
돌려주며 실패 사유는 ctx.state.last_tf_diagnostic 에 남긴다.
성공하면 같은 TF 의 yaw 도 ctx.state.last_yaw 에 저장한다(홈 복귀에서 사용).
"""
from __future__ import annotations

import math

from rclpy.time import Time
from tf2_ros import TransformException

from home_geometry import yaw_from_quaternion


class RobotPoseProvider:
    def __init__(self, ctx, tf_buffer):
        self.ctx = ctx
        self.tf_buffer = tf_buffer

    def get(self):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        s.last_tf_diagnostic = ''
        if s.meta is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(s.meta.frame, p.robot_base_frame, Time())
            stamp_sec = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now_sec = ctx.clock.now().nanoseconds * 1e-9
            age = now_sec - stamp_sec
            if age > p.tf_timeout_sec or age < -p.tf_timeout_sec:
                reason = 'tf_stale' if age > p.tf_timeout_sec else 'tf_future'
                s.last_tf_diagnostic = (
                    f'reason={reason}; age={age:.9f}s; '
                    f'allowed_age=[{-p.tf_timeout_sec:.6f},{p.tf_timeout_sec:.6f}]s; '
                    f'now={now_sec:.9f}s; tf_stamp={stamp_sec:.9f}s; '
                    f'frames={s.meta.frame}->{p.robot_base_frame}')
                ctx.warn(reason, 'TF rejected: ' + s.last_tf_diagnostic +
                         '; check use_sim_time and clocks')
                return None
            pose = (tf.transform.translation.x, tf.transform.translation.y)
            if not all(math.isfinite(v) for v in pose):
                s.last_tf_diagnostic = f'reason=nonfinite_pose; robot_xy={pose}'
                ctx.warn('nonfinite_pose', 'TF rejected: ' + s.last_tf_diagnostic)
                return None
            q = tf.transform.rotation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if not math.isfinite(yaw):
                s.last_tf_diagnostic = f'reason=nonfinite_yaw; yaw={yaw}'
                ctx.warn('nonfinite_yaw', 'TF rejected: ' + s.last_tf_diagnostic)
                return None
            s.last_yaw = yaw
            return pose, stamp_sec
        except TransformException as exc:
            s.last_tf_diagnostic = (
                f'reason=tf_lookup_failed; frames={s.meta.frame}->{p.robot_base_frame}; '
                f'error={exc}')
            ctx.warn('tf', f'Waiting for TF {s.meta.frame} -> {p.robot_base_frame}: {exc}')
            return None
