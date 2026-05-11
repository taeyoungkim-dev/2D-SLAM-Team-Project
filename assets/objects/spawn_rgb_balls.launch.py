import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 현재 폴더의 절대 경로를 가져옵니다.
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 빨간 공 노드
    spawn_red = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ball_red', '-file', os.path.join(base_dir, 'red_ball.sdf'), '-x', '2.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    # 초록 공 노드
    spawn_green = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ball_green', '-file', os.path.join(base_dir, 'green_ball.sdf'), '-x', '2.0', '-y', '1.0', '-z', '1.0'],
        output='screen'
    )

    # 파란 공 노드
    spawn_blue = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ball_blue', '-file', os.path.join(base_dir, 'blue_ball.sdf'), '-x', '2.0', '-y', '-1.0', '-z', '1.0'],
        output='screen'
    )

    # 가벽(문 막이) 노드
    spawn_blocker = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'blocker_1', '-file', os.path.join(base_dir, 'door_blocker.sdf'), '-x', '2.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        spawn_red,
        spawn_green,
        spawn_blue,
        spawn_blocker
    ])