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
            'use_sim_time': 'true',
            'qos': '2',
            # [필수 수정] args에 2D Grid 생성을 위한 파라미터 명시적 추가
            'rtabmap_args': '--delete_db_on_start --Grid/Sensor 1 --Grid/RangeMax 3.5 --Grid/RayTracing true', 
            'frame_id': 'base_footprint',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            
            # [필수 추가] LiDAR 기반 2D Map 생성을 위해 scan topic subscribe 활성화
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            
            # [필수 추가] 불안정한 Visual Odometry 끄기 (Extrapolation Error 방지)
            'visual_odometry': 'false', 
            
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'odom_topic': '/odom',
            
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',
            
            'rviz': 'true',
            'rtabmap_viz': 'false',
        }.items()
    )

    return LaunchDescription([
        rtabmap_cmd
    ])