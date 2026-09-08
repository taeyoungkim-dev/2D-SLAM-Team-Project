# 상황발생
`ros2 launch custom_burger rtabmap_3d.launch.py`를 실행했더니 다음과 같은 에러 로그가 떴다.

```zsh
ros2 launch custom_burger rtabmap_3d.launch.py

[INFO] [launch]: All log files can be found below /root/.ros/log/2026-05-24-13-07-02-735973-MSKIm-1830

[INFO] [launch]: Default logging verbosity is set to INFO

[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions when trying to load file of format [py]:

 - PackageNotFoundError: "package 'rtabmap_launch' not found, searching: ['/root/ros2_slam_ws/2D-SLAM-Team-Project/install/taeyoung_explorer', '/root/ros2_slam_ws/2D-SLAM-Team-Project/install/custom_burger', '/opt/ros/humble']"

 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

# 원인: `rtabmap_launch` Package Not Found Error
- Dependency 누락
출력된 로그의 PackageNotFoundError는 시스템이 rtabmap_launch라는 Package를 찾지 못했다는 명확한 Exception이다. rtabmap_3d.launch.py 내부에서 get_package_share_directory('rtabmap_launch')를 Call하고 있지만, 현재 Docker Container 내부의 기본 ROS 경로(/opt/ros/humble)에 RTAB-Map 관련 Binary package들이 Install 되어 있지 않기 때문에 발생한 문제이다.

# 해결:
Unbuntu의 APT package manager를 통해 ROS 2 Humble 버전용 RTAB-Map 패키지들을 System-wide로 설치해준다.

```zsh
sudo apt update
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap-launch ros-humble-rtabmap
```

![결과](../assets/images/result_solving_rtabmap_launch_PackageNotFoundError.png)
