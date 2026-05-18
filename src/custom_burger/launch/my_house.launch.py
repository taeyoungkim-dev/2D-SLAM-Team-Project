import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 탐색 (ROS 2 전화번호부)
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    custom_burger_dir = get_package_share_directory('custom_burger')

    # ---------------------------------------------------------
    # [파트 A] 배경(World) 띄우기 : 원본 시스템 폴더에서 House 월드 빌려오기
    # ---------------------------------------------------------
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_house.world')
    
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
    # [파트 B] 내 로봇 띄우기 : 커스텀 패키지의 URDF와 SDF 사용하기
    # ---------------------------------------------------------
    # URDF (로봇 뼈대 도면)
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

    # SDF (카메라 센서가 추가된 물리 모델 스폰)
    sdf_file = os.path.join(custom_burger_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_file,
            '-x', '-2.0',
            '-y', '1.0',
            '-z', '0.0'
        ],
        output='screen',
    )

    # 2. 선언한 노드들을 모아서 최종 실행 명령 반환
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd
    ])