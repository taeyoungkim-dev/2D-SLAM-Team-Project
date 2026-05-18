import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    custom_burger_dir = get_package_share_directory('custom_burger')

    # ---------------------------------------------------------
    # [파트 A] 배경 띄우기 : turtlebot3_world.world 빌려오기
    # ---------------------------------------------------------
    # [변경됨] 'turtlebot3_house.world' -> 'turtlebot3_world.world' 로 변경
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # ---------------------------------------------------------
    # [파트 B] 내 로봇 띄우기 (동일함)
    # ---------------------------------------------------------
    urdf_file = os.path.join(custom_burger_dir, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    sdf_file = os.path.join(custom_burger_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_file,
            # [변경됨] turtlebot3_world 맵에 맞게 시작 위치 변경 (원래 맵의 기본 스폰 위치)
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.0'
        ],
        output='screen',
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd
    ])