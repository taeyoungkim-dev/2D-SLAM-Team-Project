### 문제점 1
Explore Lite(자율 탐색)를 실행하기 위해서는 로봇의 주행을 담당하는 Navigation2(Nav2)가 정상적으로 실행되어 작동하고 있어야 한다. 그러나 터틀봇3에서 제공하는 기본 waffle_pi.yaml 파라미터 파일을 사용하여 Nav2를 실행하면, [planner_server] 노드에서 Failed to create global planner (FATAL) 에러가 발생하며 Nav2 엔진 전체가 비정상 종료된다. 이로 인해 Explore Lite가 Nav2 서버와 연결되지 못하고 대기(Waiting to connect...) 상태에 빠지며 로봇이 움직이지 않는다.
---
### 원인
ROS 2 패키지 버전 업데이트(Humble 버전 이상)에 따른 플러그인 명명 규칙(Syntax) 변경으로 인한 문법 충돌 문제이다.
최신 ROS 2의 Nav2 엔진은 플러그인 이름을 지정할 때 슬래시(/)를 사용하는 것을 표준으로 삼고 있다. 그러나 시스템에 기본 설치된 터틀봇3의 파라미터 파일(/opt/ros/humble/.../waffle_pi.yaml)은 과거 구버전의 C++ 네임스페이스 방식인 더블 콜론(::) 문법을 여전히 사용하고 있다. (예: nav2_navfn_planner::NavfnPlanner)
이 때문에 최신 Nav2 엔진이 해당 이름의 플러그인 클래스를 시스템에서 찾지 못해 구동을 포기하고 죽어버린 것이다.
---
### 해결
읽기 전용인 시스템 기본 파라미터 파일을 로컬 작업 공간(Workspace)으로 복사해 온 뒤, 문제가 되는 오타 문법(::)을 최신 표준(/)으로 직접 수정한 '커스텀 파라미터 파일'을 만들어 Nav2 실행 시 인자로 넘겨준다.

1. 파라미터 파일 로컬 복사 (custom_waffle_pi.yaml 생성)

```Bash
cd ~/workspace/2D-SLAM-Team-Project
cp /opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml ./custom_waffle_pi.yaml
```
2. 문법 수정

모든 ::을 \로 바꾸기
```zsh
sed -i 's/nav2_behaviors::/nav2_behaviors\//g' custom_waffle_pi.yaml
```

3. 수정된 커스텀 파라미터 파일로 Nav2 재실행
기본 파일 경로 대신, 수정한 ./custom_waffle_pi.yaml 파일 경로를 지정하여 Nav2 엔진을 켠다.

```Bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./custom_waffle_pi.yaml
```


### 문제점 2

```zsh
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

위를 실행시 아래의 에러가 발생하며 로봇이 제자리에서 이상하게 동작함.

```zsh
[ERROR] ... Robot out of costmap bounds, cannot search for frontiers
```
현재의 /map 안에 로봇이 위치해있지 않아 나타나는 에러라고 함. 따라서 로봇을 wasd로 움직여서 주변 /map을 조금 그려줌

```zsh
[explore-1] [WARN] [1778213678.750202368] [explore_node]: No frontiers found, stopping. 
```
다음으로는 위의 에러가 발생함

### 원인

벽이 아닌 경계를 표시하는 /explore/frontiers/marker_array 가 rviz2에서 표시되지 않음

![Error : Can not load marker_array_missing_1.img](/assets/images/marker_array_missing_1.png)
![Error : Can not load marker_array_missing_2.img](/assets/images/marker_array_missing_2.png)

topic에 아예 /explore/frontiers/marker_array 가 없는 것을 확인
```zsh
 ros2 topic list
/behavior_server/transition_event
/behavior_tree_log
/bond
/bt_navigator/transition_event
/camera/camera_info
/camera/image_raw
/clicked_point
/clock
/cmd_vel
/cmd_vel_nav
/cmd_vel_teleop
/constraint_list
/controller_server/transition_event
/cost_cloud
/diagnostics
/evaluation
/explore/resume
/global_costmap/clearing_endpoints
/global_costmap/costmap
/global_costmap/costmap_raw
/global_costmap/costmap_updates
/global_costmap/footprint
/global_costmap/global_costmap/transition_event
/global_costmap/published_footprint
/global_costmap/voxel_grid
/goal_pose
/imu
/initialpose
/joint_states
/landmark_poses_list
/local_costmap/clearing_endpoints
/local_costmap/costmap
/local_costmap/costmap_raw
/local_costmap/costmap_updates
/local_costmap/footprint
/local_costmap/local_costmap/transition_event
/local_costmap/published_footprint
/local_costmap/voxel_grid
/local_plan
/map
/map_updates
/marker
/move_base_simple/goal
/odom
/parameter_events
/performance_metrics
/plan
/plan_smoothed
/planner_server/transition_event
/preempt_teleop
/received_global_plan
/robot_description
/rosout
/scan
/scan_matched_points2
/smoother_server/transition_event
/speed_limit
/submap_list
/tf
/tf_static
/trajectory_node_list
/transformed_global_plan
/waypoint_follower/transition_event
```

적절한 빈 공간이 없어서 탐색이 종료됐다고 파악.
default 세팅은 min_frontier_size:=0.75
너무 큼
min_frontier_size:=0.2로 줄이기


### 해결

```zsh
ros2 launch explore_lite explore.launch.py use_sim_time:=true robot_base_frame:=base_link min_frontier_size:=0.2
```