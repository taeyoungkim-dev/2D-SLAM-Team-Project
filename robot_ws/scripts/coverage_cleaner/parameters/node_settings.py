"""토픽/프레임/액션/컨트롤러 이름 등 ROS 연결 설정.

기능 on/off(feature_flags.py)나 임계값(thresholds.py)이 아닌 이름 계열 파라미터와
출력 토픽 이름을 모았다. 파라미터는 `--ros-args -p 이름:=값` 으로 덮어쓸 수 있다.
"""

from pathlib import Path

NODE_NAME = 'coverage_cleaner'

# parameters/ -> coverage_cleaner/ -> scripts/ -> robot_ws/ -> 저장소 루트
_REPO_ROOT = Path(__file__).resolve().parents[4]

NODE_SETTING_DEFAULTS = dict(
    map_topic='/explore_map',
    robot_base_frame='base_footprint',
    follow_path_action_name='/follow_path',
    controller_id='FollowPath',
    goal_checker_id='',
    controller_server='/controller_server',
    # --- 홈 복귀(Return Home) ---
    # 스위치 topic (std_msgs/Bool). True 가 오면 청소를 멈추고 홈 위치/yaw 로 복귀한다.
    return_home_topic='/return_home',
    # 홈 위치 입력 topic (geometry_msgs/PoseStamped, transient-local). 서버 브릿지가 도크 위치를 알려준다.
    home_pose_topic='/home_pose',
    navigate_action_name='/navigate_to_pose',
    spin_action_name='/spin',
    # 홈 위치 직접 지정(map frame). NaN 이면 첫 TF 위치 또는 /home_pose 를 쓴다.
    # 값을 주면 /home_pose 는 무시한다. home_x, home_y 는 함께 주어야 한다.
    home_x=float('nan'),
    home_y=float('nan'),
    home_yaw=0.0,
    # --- 장애물 회피 ---
    # Gazebo 스폰 launch 파일. 여기 적힌 장애물 위치(world 좌표)를 청소 불가 구역의 중심으로 쓴다.
    obstacle_spawn_file=str(_REPO_ROOT / 'assets' / 'objects' / 'spawn_household_obstacles.launch.py'),
    # 스폰 좌표가 속한 frame. diff drive 가 odometry_source=WORLD 라 odom == Gazebo world 이다.
    obstacle_frame='odom',
)

# 출력 토픽
COVERAGE_PATH_TOPIC = '/coverage_path'
COVERAGE_ACTIVE_PATH_TOPIC = '/coverage_active_path'
COVERAGE_STATUS_TOPIC = '/coverage_status'
# /coverage_<key>_map. obstacle_zone(=/coverage_obstacle_zone_map)은 장애물 때문에 청소할 수 없는
# 구역(맵 기준, 100=구역)이며 zone 이 바뀌면 즉시 발행된다.
DEBUG_MAP_KEYS = ('visited', 'remaining', 'target', 'unreachable', 'obstacle_zone')
