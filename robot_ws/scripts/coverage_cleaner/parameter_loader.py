"""ROS 파라미터 선언/로드와 검증.

기본값은 feature_flags.py / thresholds.py / node_settings.py (parameters/ 폴더)에서 가져온다.
검증 함수는 ROS 없이도 호출 가능하다.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

from parameters.feature_flags import FEATURE_FLAG_DEFAULTS
from parameters.node_settings import NODE_SETTING_DEFAULTS
from parameters.thresholds import PLANNING_EXTRA_MARGIN_M, THRESHOLD_DEFAULTS

POSITIVE_PARAMETERS = (
    'robot_radius', 'lane_spacing', 'coverage_width', 'goal_xy_tolerance',
    'endpoint_tolerance', 'tracker_update_period_sec', 'tf_timeout_sec',
    'tracker_max_speed', 'tracker_max_gap_sec', 'max_leg_length',
    'path_resolution', 'retry_delay_sec', 'goal_timeout_sec',
    'home_xy_tolerance', 'home_yaw_tolerance', 'home_nav_timeout_sec', 'home_spin_timeout_sec')


def default_parameters() -> dict:
    """세 개의 설정 파일의 기본값을 하나의 dict로 합친다."""
    return {**NODE_SETTING_DEFAULTS, **FEATURE_FLAG_DEFAULTS, **THRESHOLD_DEFAULTS}


def validate_parameters(params: SimpleNamespace) -> None:
    """값 검증. 통과하면 params.planning_extra_margin 을 채운다."""
    if any(not math.isfinite(float(getattr(params, n))) or getattr(params, n) <= 0
           for n in POSITIVE_PARAMETERS):
        raise ValueError('Distances and timeouts must be positive and finite')
    if (not math.isfinite(params.coverage_clearance) or params.coverage_clearance < 0
            or not 1 <= params.occupied_threshold <= 100
            or min(params.max_failures, params.max_no_progress_passes,
                   params.max_astar_expansions) < 1
            or (params.configure_goal_checker and params.check_endpoint
                and params.endpoint_tolerance < params.goal_xy_tolerance)):
        raise ValueError('Invalid threshold, clearance, retry limit or endpoint tolerance')
    if math.isnan(params.home_x) != math.isnan(params.home_y):
        raise ValueError('home_x and home_y must be given together')
    if (not math.isnan(params.home_x)
            and not all(math.isfinite(v) for v in (params.home_x, params.home_y, params.home_yaw))):
        raise ValueError('home_x, home_y and home_yaw must be finite')
    params.planning_extra_margin = float(PLANNING_EXTRA_MARGIN_M)
    if not math.isfinite(params.planning_extra_margin) or params.planning_extra_margin < 0:
        raise ValueError('PLANNING_EXTRA_MARGIN_M must be finite and non-negative')


def load_parameters(node) -> SimpleNamespace:
    """node.declare_parameter 로 선언하고 (CLI 덮어쓰기 반영) 검증된 params 를 돌려준다."""
    values = {}
    for name, value in default_parameters().items():
        node.declare_parameter(name, value)
        values[name] = node.get_parameter(name).value
    params = SimpleNamespace(**values)
    validate_parameters(params)
    return params
