ros2_ws에서
# 1. 셋업 파일 적용
source install/setup.bash

# 2. 라이다 노드 실행
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# esp32 node
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
토픽 확인
ros2 topic echo /joint_states 

# imu 센서 node
ros2 run mobile_imu_driver imu_udp_node

# rover 물리정보 node
ros2 launch rover_description rsp.launch.py 

# 실제 실행 명령
ros2 launch rover_bringup bringup.launch.py  

#조종 명령어
ros2 run teleop_twist_keyboard teleop_twist_keyboard


빠른 실행 방법
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
ros2 launch rover_bringup bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
