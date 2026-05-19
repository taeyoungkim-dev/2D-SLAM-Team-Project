import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RTAB-Map 공식 런치 파일 경로 가져오기
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')

    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
        ),
        launch_arguments={
            'args': '-d',  # -d: 실행할 때마다 이전 지도 데이터를 초기화하고 새로 그림
            'frame_id': 'base_footprint',  # 로봇의 기준 좌표계
            'subscribe_depth': 'true',     # 뎁스 카메라 사용 ON
            'subscribe_rgb': 'true',       # 컬러 카메라 사용 ON
            'rgb_topic': '/camera/image_raw',        # Gazebo에서 나오는 컬러 토픽
            'depth_topic': '/camera/depth/image_raw',# Gazebo에서 나오는 뎁스 토픽
            'camera_info_topic': '/camera/camera_info',
            'odom_topic': '/odom',         # 바퀴(오도메트리) 위치 정보
            'approx_sync': 'true',         # 카메라와 바퀴 센서 간의 시간 오차 허용 (시뮬레이션 필수)
            'rviz': 'true',                # 3D 맵을 볼 수 있도록 미리 세팅된 RViz 켜기
            'rtabmap_viz': 'false',        # RTAB-Map 자체 UI는 무거우므로 끄기
        }.items()
    )

    return LaunchDescription([
        rtabmap_cmd
    ])
