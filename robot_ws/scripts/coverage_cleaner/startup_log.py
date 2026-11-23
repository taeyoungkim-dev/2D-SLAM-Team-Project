"""노드 시작 시 설정 요약 로그."""
from __future__ import annotations

from parameters.thresholds import TRACKER_DISTANCE_SLACK_M, TRACKER_SAME_STAMP_MOTION_LIMIT_M


def log_startup(logger, params):
    p = params
    logger.info(f'Coverage input={p.map_topic}; base={p.robot_base_frame}; '
                f'width={p.coverage_width:.3f}m; send_to_nav2={p.send_to_nav2}')
    logger.info(
        f'Checks: configure_goal_checker={p.configure_goal_checker}; '
        f'check_endpoint={p.check_endpoint}; '
        f'check_safe_position={p.check_safe_position}; '
        f'goal_xy_tolerance={p.goal_xy_tolerance:.3f}m; '
        f'endpoint_tolerance={p.endpoint_tolerance:.3f}m; '
        'successful segments reset failures to 0')
    logger.info(
        f'Completion: reclean_unvisited={p.reclean_unvisited}; '
        'false stops after a normally completed path; recovery replanning remains enabled')
    logger.info(
        f'Return home: switch={p.return_home_topic}; on_complete={p.return_home_on_complete}; '
        f'home_pose_topic={p.home_pose_topic}; xy_tolerance={p.home_xy_tolerance:.3f}m; '
        f'yaw_tolerance={p.home_yaw_tolerance:.3f}rad; '
        f'nav={p.navigate_action_name}; spin={p.spin_action_name}')
    logger.info(
        f'Obstacle avoidance={p.obstacle_avoidance}; spawn_file={p.obstacle_spawn_file}; '
        f'frame={p.obstacle_frame}; '
        'zone map=/coverage_obstacle_zone_map')
    logger.info(
        f'Clearance: stop={p.coverage_clearance:.3f}m; '
        f'planning={p.coverage_clearance + p.planning_extra_margin:.3f}m '
        f'(extra={p.planning_extra_margin:.3f}m); '
        'both also include robot radius and grid correction')
    logger.info(
        f'TF diagnostic limits: tf_timeout_sec={p.tf_timeout_sec:.6f}s; '
        f'tracker_max_gap_sec={p.tracker_max_gap_sec:.6f}s; '
        f'tracker_max_speed={p.tracker_max_speed:.6f}m/s; '
        f'distance_slack={TRACKER_DISTANCE_SLACK_M:.6f}m; '
        f'same_stamp_motion_limit={TRACKER_SAME_STAMP_MOTION_LIMIT_M:.9f}m')
