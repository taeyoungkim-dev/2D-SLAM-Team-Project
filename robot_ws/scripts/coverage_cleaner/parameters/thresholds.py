"""임계값/허용오차/제한값 파라미터 모음 (단위: m, s, m/s).

ROS 파라미터의 기본값이며 `--ros-args -p 이름:=값` 으로 덮어쓸 수 있다.
파일 수정 후에는 노드를 재시작한다.
"""
import math

# 계획 경로의 최소 벽 거리 = 정지 기준 거리 + 아래 추가 여유.
# 현재 시험값 0.2m = 20cm. 실제 추종/위치 추정 오차에 맞춰 조절한다.
#PLANNING_EXTRA_MARGIN_M = 0.2
PLANNING_EXTRA_MARGIN_M = 0.1

THRESHOLD_DEFAULTS = dict(
    # --- 지도 / 형상 ---
    occupied_threshold=50,                      # OccupancyGrid 점유 판정 기준(1..100)
    robot_radius=math.hypot(0.14, 0.14) / 2,    # 원형 footprint 반지름
    coverage_clearance=0.02,                    # 정지 판정 추가 여유
    lane_spacing=0.25,                          # sweep 줄 간격
    #edited by tky
    #coverage_width=0.15
    coverage_width=0.25,                        # 실제로 덮는 폭(설정값이지 실측값 아님)
    # --- 목표/끝점 허용오차 ---
    goal_xy_tolerance=0.05,                     # Nav2 goal checker xy_goal_tolerance 상한
    endpoint_tolerance=0.1,                     # Nav2 성공 후 끝점 허용 거리
    # --- TF / 추적 ---
    tracker_update_period_sec=0.1,              # tick 주기
    tf_timeout_sec=1.0,                         # TF age 허용값
    tracker_max_speed=0.4,                      # 추적 연속성 판정 최대 속도
    tracker_max_gap_sec=0.5,                    # 추적 관측 간격 허용값
    # --- 경로 ---
    max_leg_length=2.0,                         # FollowPath 한 구간 최대 길이
    path_resolution=0.025,                      # 발행 Path 점 간격
    # --- 재시도 / 제한 ---
    retry_delay_sec=2.0,
    max_failures=3,                             # 연속 실패 한도
    max_no_progress_passes=3,
    max_astar_expansions=500000,
    goal_timeout_sec=120.0,
    # --- 홈 복귀 ---
    home_max_failures=8,                        # 복귀 시도 연속 실패 한도(실패할 때마다 자체 경로 <-> NavigateToPose 를 번갈아 쓴다)
    home_xy_tolerance=0.1,                      # 복귀 완료로 인정하는 위치 오차(m)
    home_yaw_tolerance=0.4,                     # 복귀 완료로 인정하는 yaw 오차(rad, 약 22.9도)
    home_nav_timeout_sec=300.0,                 # NavigateToPose 한 번의 제한 시간
    home_spin_timeout_sec=20.0,                 # 제자리 yaw 보정(Spin) 한 번의 제한 시간
)

# ---- 파라미터로 노출되지 않는 내부 상수 (기존 코드의 하드코딩 값 그대로) ----
TRACKER_DISTANCE_SLACK_M = 0.01             # 속도 한도 판정 시 거리 여유
TRACKER_SAME_STAMP_MOTION_LIMIT_M = 1e-6    # 같은 stamp에서 허용되는 이동량
WARN_THROTTLE_SEC = 5.0                     # 같은 경고 키의 최소 출력 간격
DEBUG_PUBLISH_PERIOD_SEC = 1.0              # 디버그 지도 발행 주기
CANCEL_GRACE_SEC = 10.0                     # goal_timeout 이후 취소 미확인 시 block까지 유예
PARAM_SERVICE_TIMEOUT_SEC = 10.0            # controller 파라미터 서비스 응답 한도
SHUTDOWN_CANCEL_WAIT_SEC = 3.0              # 종료 시 Nav2 취소 확인 대기
HOME_MAX_SPIN_TRIES = 5                     # 복귀 후 yaw 보정 시도 한도
OBSTACLE_REFRESH_PERIOD_SEC = 1.0           # 구역을 만들기 위한 map<-odom TF 확인 주기(만든 뒤 중단)

# 장애물 별 청소 불가 구역 반지름(m). 스폰 위치를 중심으로 하는 원의 반지름이며 그대로 쓴다.
# 로봇 반지름/여유는 traversable_map 이 따로 더한다.
OBSTACLE_ZONE_RADII_M = dict(
    power_cable=0.05,
    cardboard_box=0.05,
    urine_puddle=0.05,
)
OBSTACLE_ZONE_DEFAULT_RADIUS_M = 0.25       # 위 표에 없는 모델 이름
