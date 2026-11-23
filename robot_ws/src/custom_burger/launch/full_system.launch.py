"""시뮬레이션 전체를 한 번에 실행한다 (터미널 하나, Ctrl-C 한 번으로 전체 종료).

    cd ~/workspace/2D-SLAM-Team-Project/robot_ws && source install/setup.zsh && cd ..
    ros2 launch robot_ws/src/custom_burger/launch/full_system.launch.py

실행 순서 (고정 지연이 아니라 "준비됨"을 확인하고 다음 단계로 간다)
    0. Gazebo + 로봇(my_turtlebot3_world), 서버(uvicorn)
    1. /odom 이 나올 때까지 대기 (robot_ws/scripts/wait_ready.py --odom)
    2. 장애물 스폰, Nav2(+rviz), map_threshold_node
       - AMCL 초기 위치는 rviz 의 2D Pose Estimate 로 직접 찍는다(자동 설정 없음).
    3. Nav2 lifecycle active + TF map->base_footprint 가 될 때까지 대기 (wait_ready.py --nav2)
    4. coverage_cleaner, obstacle_reporter, ros2_bridge_node
       - cleaner 는 기본적으로 대기 상태로 뜬다(auto_plan:=false). 앱의 [청소 시작] 이 청소를 시작한다.

launch 인자 (이름:=값)
    gui:=false            Gazebo 창을 띄우지 않는다            rviz:=false          rviz 를 띄우지 않는다
    auto_plan:=true       cleaner 가 준비되면 바로 청소 시작   server:=false       서버/브릿지를 실행하지 않는다
    reporter:=false       obstacle_reporter 를 실행하지 않는다  public_host:=<IP>    앱이 사진을 받을 서버 주소(기본: 이 PC 의 LAN IP)
    map:=<yaml>  params_file:=<yaml>  server_port:=8000
"""
import os
import socket
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 이 파일: <repo>/robot_ws/src/custom_burger/launch/full_system.launch.py
REPO = Path(os.environ.get('SLAM_REPO_ROOT') or Path(__file__).resolve().parents[4])
SCRIPTS = REPO / 'robot_ws' / 'scripts'
SIM_TIME = ['-p', 'use_sim_time:=true']


def lan_ip() -> str:
    """이 PC 의 LAN IP (앱이 서버 사진 주소로 접속할 때 쓴다). 못 구하면 127.0.0.1."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))  # 실제로 패킷을 보내지 않는다
            return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'


def process(name, cmd, **kwargs):
    kwargs.setdefault('cwd', str(REPO))
    return ExecuteProcess(cmd=[str(c) if isinstance(c, Path) else c for c in cmd], name=name,
                          output='screen', **kwargs)


def after(target, actions, what):
    """target 프로세스가 성공(0)으로 끝나면 actions 를 시작하고, 실패하면 전체를 종료한다."""
    def on_exit(event, context):
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f'[full_system] {what} 준비 실패(종료 코드 {event.returncode}): 전체를 종료합니다'),
                EmitEvent(event=Shutdown(reason=f'{what} not ready'))]
    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=on_exit))


def generate_launch_description():
    port = LaunchConfiguration('server_port')
    server_on = IfCondition(LaunchConfiguration('server'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(get_package_share_directory('custom_burger') + '/launch/my_turtlebot3_world.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    # FastAPI 서버. 앱이 받는 사진 주소는 SERVER_PUBLIC_BASE_URL 로 만들어진다.
    server = process(
        'server', [REPO / 'server' / '.venv' / 'bin' / 'uvicorn', 'app.main:app', '--host', '0.0.0.0',
                   '--port', port],
        cwd=str(REPO / 'server'), condition=server_on,
        additional_env={'SERVER_PUBLIC_BASE_URL': ['http://', LaunchConfiguration('public_host'), ':', port]})

    wait_odom = process('wait_odom', ['python3', SCRIPTS / 'wait_ready.py', '--odom', '--timeout', '300'])

    spawn_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(REPO / 'assets' / 'objects' / 'spawn_household_obstacles.launch.py')))
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'bringup_launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'true',
                          'params_file': LaunchConfiguration('params_file')}.items())
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', str(Path(get_package_share_directory('turtlebot3_navigation2'))
                                     / 'rviz' / 'tb3_navigation2.rviz')],
                parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))
    map_threshold = process('map_threshold_node', ['python3', SCRIPTS / 'map_threshold_node.py'])

    wait_nav2 = process('wait_nav2', ['python3', SCRIPTS / 'wait_ready.py', '--nav2', '--timeout', '300'])

    cleaner = process('coverage_cleaner', [
        'python3', SCRIPTS / 'coverage_cleaner' / 'coverage_cleaner.py', '--ros-args', *SIM_TIME,
        '-p', 'tf_timeout_sec:=1.5', '-p', 'tracker_max_gap_sec:=1.5', '-p', 'configure_goal_checker:=false',
        '-p', 'check_endpoint:=false', '-p', 'check_safe_position:=false', '-p', 'reclean_unvisited:=false',
        '-p', ['auto_plan:=', LaunchConfiguration('auto_plan')]])
    reporter = process('obstacle_reporter', [
        REPO / 'training' / '.venv' / 'bin' / 'python', SCRIPTS / 'obstacle_reporter' / 'obstacle_reporter.py',
        '--ros-args', *SIM_TIME, '-p', 'server_host:=127.0.0.1', '-p', ['server_port:=', port]],
        condition=IfCondition(LaunchConfiguration('reporter')))
    bridge = process('ros2_bridge', [
        'python3', REPO / 'server' / 'scripts' / 'ros2_bridge_node.py', '--ros-args', *SIM_TIME,
        '-p', 'server_host:=127.0.0.1', '-p', ['server_port:=', port]], condition=server_on)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 창'),
        DeclareLaunchArgument('rviz', default_value='true', description='rviz 창'),
        DeclareLaunchArgument('server', default_value='true', description='서버와 ros2_bridge_node 실행'),
        DeclareLaunchArgument('reporter', default_value='true', description='obstacle_reporter 실행'),
        DeclareLaunchArgument('auto_plan', default_value='false',
                              description='true 면 cleaner 가 준비되면 바로 청소 시작(false 면 앱의 [청소 시작] 대기)'),
        DeclareLaunchArgument('public_host', default_value=lan_ip(), description='앱이 서버에 접속하는 주소'),
        DeclareLaunchArgument('server_port', default_value='8000'),
        DeclareLaunchArgument('map', default_value=str(REPO / 'maps' / 'world_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(REPO / 'custom_burger_cam_rpp.yaml')),
        gazebo, server, wait_odom,
        after(wait_odom, [spawn_obstacles, nav2, rviz, map_threshold, wait_nav2], 'Gazebo 로봇(/odom)'),
        after(wait_nav2, [cleaner, reporter, bridge], 'Nav2'),
    ])
