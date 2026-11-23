# 2D-SLAM-Team-Project
2D SLAM Project

> **처음 clone 했다면 [SETUP.md](SETUP.md)(환경 구축 가이드: robot_ws / server / app)부터 보세요.**

# TurtleBot3 2D SLAM in Gazebo (ROS 2 Humble)

본 프로젝트는 ROS 2 Humble 환경에서 TurtleBot3 (burger)를 활용하여 Gazebo 가상 환경에서 2D SLAM(Cartographer)을 수행하고 지도를 생성하는 시뮬레이션 환경 구축 가이드입니다.

## 🛠 Prerequisites (사전 요구 사항)
* **OS:** Ubuntu 22.04 LTS
* **ROS 2:** Humble Hawksbill
* **Shell:** Zsh
* **Simulator:** Gazebo 11 (Classic)

---

## 📦 1. Installation & Setup (의존성 패키지 설치)

### 1.1 충돌 방지를 위한 Gazebo 환경 초기화
신형 Gazebo(Ignition) 패키지와 구형 Gazebo 11 간의 충돌을 방지하기 위해 기존 관련 패키지를 정리하고 설치합니다.

```zsh
# Gazebo 11 및 ROS 2 브릿지 설치
sudo apt update
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs
```

### 1.2 TurtleBot3 및 SLAM 필수 패키지 설치
로봇 구동과 매핑에 필요한 공식 패키지들을 설치합니다.

```zsh
sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo
```

### 1.3 Gazebo 모델 데이터 수동 다운로드 (초기 로딩 지연 방지)
Gazebo 첫 실행 시 모델 다운로드로 인한 응답 없음(Timeout) 현상을 방지하기 위해 미리 모델 데이터를 클론합니다.


```zsh
#2.1 명령을 한번 실행해야 .gazebo 파일이 생김
cd ~/.gazebo
rm -rf models
git clone https://github.com/osrf/gazebo_models.git models
```

### 1.4 환경 변수 설정 (`~/.zshrc`)
TurtleBot3 Burger 모델을 기본으로 사용하도록 `zsh` 환경에 등록합니다.

자신의 환경이 bash인지 zsh인지 체크해야함!
```zsh
echo "export TURTLEBOT3_MODEL=burger" >> ~/.zshrc
source ~/.zshrc
```

### 1.5 taeyoung_explorer 패키지 설치
```zsh
cd ~/2D-SLAM-Team-Project
colcon build --symlink-install
```

### 1.6 custom_burger 패키지 빌드 (⚠️ 필수)
> **반드시 `custom_burger` 를 빌드해야 합니다.** 이 패키지에는 RealSense 깊이 카메라가 달린 커스텀 TurtleBot3 모델(`/camera/depth/image_raw`, `/camera/depth/camera_info` 등)이 들어 있습니다.
> 빌드하지 않으면 `ros2 launch custom_burger ...` 가 `package 'custom_burger' not found` 오류로 실패하고, 기본 `turtlebot3_gazebo` 의 `turtlebot3_world.launch.py` 로 띄운 로봇에는 깊이 카메라가 없어 `training/` 의 영상 녹화와 자동 라벨링이 동작하지 않습니다.

```zsh
cd ~/2D-SLAM-Team-Project/robot_ws
colcon build --symlink-install --packages-select custom_burger
source install/setup.zsh   # 새 터미널을 열 때마다 필요 (bash 는 setup.bash)
```

빌드 확인: `ros2 pkg list | grep custom_burger` 에 `custom_burger` 가 나와야 합니다.

## 🚀 2. How to Run (실행 방법)

### [권장] 한 번에 실행 (`full_system.launch.py`) — 터미널 1개
Gazebo(world 맵) → 장애물 스폰 → Nav2(rviz 에서 2D Pose Estimate 수동) → `map_threshold_node` → `coverage_cleaner` → `obstacle_reporter` → 서버(uvicorn) → `ros2_bridge_node` 를 순서대로 실행한다. 고정 지연이 아니라 `/odom` 수신, Nav2 lifecycle active + TF `map→base_footprint` 를 확인한 뒤 다음 단계로 넘어가며, 준비가 안 되면(5분 초과) 전체를 종료한다. `Ctrl-C` 한 번이면 전부 종료된다.

```bash
cd ~/workspace/2D-SLAM-Team-Project
source robot_ws/install/setup.zsh          # (bash 면 setup.bash)
export TURTLEBOT3_MODEL=burger             # ~/.zshrc 에 이미 있으면 생략
ros2 launch robot_ws/src/custom_burger/launch/full_system.launch.py
# 창 없이: gui:=false rviz:=false  /  앱 없이 바로 청소: auto_plan:=true  /  서버·브릿지 제외: server:=false
# 앱이 사진을 받을 주소를 직접 지정: public_host:=192.168.x.x (기본은 이 PC 의 LAN IP 자동 감지)
```

- 청소는 기본적으로 **앱의 [청소 시작]** 으로 시작한다(cleaner 의 `auto_plan` 기본값이 false 라 대기 상태로 뜬다. 앱 없이 바로 청소하려면 `auto_plan:=true`).
- 초기 위치는 자동이 아니다. rviz 가 뜨면 **2D Pose Estimate** 로 로봇 위치를 직접 찍는다(TF `map→base_footprint` 가 생겨야 다음 단계(cleaner, reporter, bridge)가 시작되므로 `rviz:=true` 로 실행하고 5분 안에 찍는다).
- 사전 준비: `colcon build` 완료, `server/.venv`(서버 의존성), `training/.venv`(ultralytics, reporter 용), 시스템 파이썬에 `pip3 install --user "websockets>=12.0"`(ros2_bridge_node 용).
- 아래 Terminal 1~5 는 각 단계를 따로 실행/디버깅할 때의 수동 방식이다.

성공적인 자율 탐색 및 2D SLAM 시뮬레이션을 위해 총 5개의 터미널 창을 순서대로 실행합니다.
*(모든 명령어는 워크스페이스 최상단인 `2D-SLAM-Team-Project` 폴더 안에서 실행한다고 가정합니다.)*

### [Terminal 1] Gazebo 시뮬레이션 실행 (환경 구성)
원하는 맵을 선택하여 가상 환경과 로봇을 소환합니다. (기본값: House 맵)
맵을 불러오고 반드시 로봇을 원하는 위치에 위치시키고 Terminal 2를 실행해야 합니다.

> `ros2 launch custom_burger ...` 를 쓰려면 먼저 **[1.6 custom_burger 빌드](#16-custom_burger-패키지-빌드--필수)** 를 끝내고, 터미널마다 `source robot_ws/install/setup.zsh` 를 해야 합니다.

```zsh
export TURTLEBOT3_MODEL=burger

# 1. House Map (SLAM 테스트 권장)
ros2 launch custom_burger my_house.launch.py
#ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
  x_pose:=-6.7 \
  y_pose:=-3.16

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


# 2. World Map (기본 장애물 맵)
# ros2 launch custom_burger my_turtlebot3_world.launch.py

# 3. Empty World (장애물 없는 빈 맵)
# ros2 launch custom_burger empty_world.launch.py
```

### [Terminal 2] 커스텀 오브젝트 및 가벽 소환
색이 다른 공 3개와 맵의 열린 입구를 막을 `door_blocker`를 한 번에 소환합니다. 
공과 벽의 위치는 `assets/objects/spawn_rgb_balls.launch.py`에서 수정하거나 가제보 환경에서 마우스로 수동으로 옮길 수 있습니다.

```zsh
ros2 launch ./assets/objects/spawn_rgb_balls.launch.py
```

### [Terminal 3] 2D SLAM (Cartographer) 및 RViz2 실행
로봇의 LiDAR 센서 데이터를 시각화하고 지도를 그리기 시작합니다. 가상 환경 동기화를 위해 반드시 `use_sim_time:=true` 파라미터를 추가해야 합니다.

```zsh
export TURTLEBOT3_MODEL=burger
#Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
#RTAB-MAP
ros2 launch custom_burger rtabmap_3d.launch.py use_sim_time:=true
```
> **Tip:** RViz2 창이 열리면 왼쪽 하단 `Add` 버튼을 눌러 `By topic` 탭에서 `/camera/image_raw`를 추가하면 로봇의 1인칭 카메라 뷰를 실시간으로 확인할 수 있습니다.

### [Terminal 4] Navigation2 실행 (자율주행 엔진)
자율 탐색 시 장애물을 회피하며 주행하기 위해 Nav2 엔진을 켭니다. ROS 2 버전 호환성을 위해 수정된 커스텀 파라미터 파일(`custom_burger.yaml`)을 사용합니다.

```zsh
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./custom_burger.yaml

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=./maps/map.yaml \
  params_file:=./custom_burger.yaml
```

### [Terminal 5] 자율 탐색 (taeyoung explorer) 또는 수동 조종

taeyoung_explorer 실행
```zsh
source install/setup.zsh
ros2 run taeyoung_explorer explorer_node
```

**옵션 B: 수동 조종 (Teleop)**
키보드 방향키(W, A, S, D, X)를 이용해 로봇을 직접 조종하고 싶을 때 사용합니다. (이 경우 Terminal 4와 Terminal 5의 옵션 A는 실행하지 않아도 됩니다.)
```zsh
ros2 run turtlebot3_teleop teleop_keyboard
```

### [선택] 강제 홈 복귀 스위치 (`/return_home`)
`coverage_cleaner`는 `/return_home` (`std_msgs/Bool`)에 `true`가 오면 청소를 멈추고 **시작 위치와 yaw**(오차 안)로 돌아옵니다.
스위치가 켜지는 경우는 두 가지이며 둘 다 같은 topic을 씁니다.

1. 청소 완료(`complete_pass` / `complete_reachable`): `coverage_cleaner`가 직접 `true`를 발행 (`return_home_on_complete:=false`로 끌 수 있음)
2. 앱의 중지/복귀 버튼: 서버 `return_to_dock` 명령 -> `ros2_bridge_node.py`가 `true`를 발행

```zsh
# 수동으로 켜기 / 복귀 중단(false: 복귀 중이면 중단 후 일시정지, 재개는 ~/resume)
# 스위치는 "복귀 진행 중"을 뜻하며, 복귀 완료/최종 실패/~/pause·~/resume 시 cleaner가 스스로 false로 내립니다.
ros2 topic pub --once /return_home std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /return_home std_msgs/msg/Bool '{data: false}'

# 진행 확인: state 가 returning_home -> returned_home (실패 시 return_failed)
ros2 topic echo /coverage_status --field data
```
- 홈 위치 우선순위: `-p home_x:=0.0 -p home_y:=0.0 -p home_yaw:=0.0`(실수로 입력) > `/home_pose`(브릿지가 도크 위치 발행) > cleaner가 처음 받은 TF 위치
- 완료 기준(실측 TF): `home_xy_tolerance`(기본 0.1m), `home_yaw_tolerance`(기본 0.4rad). `NavigateToPose`로 이동한 뒤 yaw가 어긋나면 Nav2 `/spin`으로 보정합니다.
- Terminal 4의 Nav2(`bt_navigator`, `behavior_server`)가 떠 있어야 합니다. `~/pause`, `~/resume`, `~/reset`은 복귀 중에도 우선합니다.
- 장애물 구역이 있으면 `NavigateToPose` 대신 구역을 뺀 자체 경로(A\*)를 `FollowPath`로 따라갑니다(아래 "장애물 탐지" 참고).

### [선택] 청소 불가 구역 회피 (`assets/objects/spawn_household_obstacles.launch.py` 기반)
카메라는 쓰지 않습니다. `coverage_cleaner`가 장애물 스폰 launch 파일의 위치를 읽어 그 위치에 원형 "청소 불가 구역"(`/coverage_obstacle_zone_map`)을 만들고, 청소/복귀 경로에서 피합니다.

```text
spawn_household_obstacles.launch.py 의 (이름, x, y)  +  TF(map<-odom)  -> map 좌표 -> 반지름 r 인 원 -> /coverage_obstacle_zone_map
```
- 별도 노드는 필요 없습니다. Gazebo 에 장애물을 스폰한 뒤 `coverage_cleaner` 를 평소처럼 실행하면 됩니다. 스폰 좌표는 Gazebo world 좌표이고 diff drive 가 `odometry_source=WORLD` 라서 odom == world 이므로, TF `map<-odom`(Cartographer)으로 map 좌표로 바꿉니다. 구역은 지도와 TF 가 처음 준비됐을 때 **한 번만** 만들고, 이후 TF 가 보정되어도 다시 갱신하지 않습니다(위치 고정). 지도 격자가 바뀌면 같은 좌표의 구역을 새 격자에 다시 그려 발행할 뿐입니다.
- 구역 반지름(m)은 `coverage_cleaner/parameters/thresholds.py` 의 `OBSTACLE_ZONE_RADII_M` 에서 장애물 이름별로 정합니다(표에 없는 이름은 `OBSTACLE_ZONE_DEFAULT_RADIUS_M`). 이 값이 그대로 구역 원의 반지름이고, 로봇 반지름/여유는 자동으로 더해집니다.
- 스폰 위치를 바꾸면 launch 파일만 고치면 됩니다(`-p obstacle_spawn_file:=<경로>`, `-p obstacle_frame:=odom` 으로 변경 가능).
- 구역 지도 확인: `rviz2 -d robot_ws/rviz/obstacle_zone_check.rviz` (Fixed Frame `map`, `/coverage_obstacle_zone_map` 은 Durability **Transient Local** 이어야 보입니다). 숫자 검사는 `python3 robot_ws/scripts/check_zone_map.py --once` (구역 지도의 중심/면적을 `/coverage_status` 의 `obstacle_zones` 와 비교해 PASS/FAIL).
- 청소 중 활성 경로(`/coverage_active_path`)가 구역에 걸리면 즉시 멈추고, 구역을 뺀 새 경로로 다시 계획합니다. 끄려면 `-p obstacle_avoidance:=false`.

### [선택] 장애물 발견 보고 (`robot_ws/scripts/obstacle_reporter`)
카메라 화면 가운데쯤에 장애물 전체가 보이면 서버에 "어떤 장애물인지 + 사진"을 보냅니다. 서버는 사진을 `<시간>_<class>.jpg` 로 저장하고, 앱은 지도의 장애물 위치에 표시를 합니다(표시를 누르면 종류와 사진이 보입니다).

```zsh
# 서버(uvicorn)와 coverage_cleaner 를 띄운 뒤. ROS 와 학습용 가상환경(training/.venv, ultralytics)이 모두 필요하다.
source /opt/ros/humble/setup.bash
training/.venv/bin/python robot_ws/scripts/obstacle_reporter/obstacle_reporter.py --ros-args -p use_sim_time:=true -p server_host:=<서버IP>
```
- 사진에는 장애물에 **bounding box(초록 테두리)** 가 그려지고 왼쪽 위에 class 이름이 적힙니다.
- 사진은 **장애물 전체가 화면 가운데쯤에 보일 때만**(상자가 화면 가장자리에서 `border_px`=100px(640x480 기준) 이상 안쪽에 있어 잘리지도 않고 가장자리에 치우치지도 않을 때) 찍습니다. 더 가운데만 원하면 `-p border_px:=140`, 덜 엄격하게는 `-p border_px:=30` 처럼 바꿉니다. 그 외 조건은 느슨합니다: 신뢰도 `conf` 0.3 이상이 1번(`min_frames`) 잡히면 되고, 로봇-장애물 거리 `max_distance` 는 기본 0 = 확인 안 함. 상자가 너무 작으면(멀리 있으면) 찍지 않습니다: 상자 면적이 화면의 `min_area_ratio` 이상이어야 하고 기본 0.03(640x480 에서 약 96x96px)입니다. 더 가까이서만 찍으려면 `-p min_area_ratio:=0.06`, 크기 조건을 끄려면 `-p min_area_ratio:=0`. 엄격하게 하려면 `-p conf:=0.6 -p min_frames:=3 -p min_area_ratio:=0.06 -p max_distance:=0.25`.
- 장애물 위치(와 거리 계산의 장애물 중심)는 카메라가 아니라 `coverage_cleaner` 의 `/coverage_status` `obstacle_zones`(스폰 위치), 로봇 위치는 TF `map<-base_footprint` 입니다. 그래서 `coverage_cleaner` 가 떠 있어야 합니다.
- 사진은 장애물 하나에 한 장입니다. 같은 class 는 한 번의 청소 안에서 한 번만 찍고 표시도 한 번만 만듭니다(사진을 찍기 전에 서버에 이미 있는지 확인). 복귀하거나 중간에 종료해도 남고, **앱에서 새 청소를 시작하면** 표시와 `/coverage_visited_map` 이 초기화되어 다시 한 번 찍을 수 있습니다(서버 + `ros2_bridge_node.py`).
- 로직 검사: `python3 robot_ws/scripts/obstacle_reporter/self_test.py` (ROS/YOLO 불필요).

## 💾 3. Save the Map (지도 저장하기)

맵 스캔이 충분히 완료되었다면, 새로운 터미널을 열고 생성된 지도를 저장합니다. (`map`이라는 이름의 `.yaml`과 `.pgm` 파일이 생성됩니다.)

```zsh
ros2 run nav2_map_server map_saver_cli -f ./map
```

## 📡 4. m-explore-ros2 (참고용 Frontier Exploration 패키지)

자체 구현한 `taeyoung_explorer`와 비교/참고하기 위해 [`robo-friends/m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)를 `src/m-explore-ros2`에 clone하고, `explore_lite`(frontier 탐색), `explore_lite_msgs`, `multirobot_map_merge`(다중 로봇 맵 병합) 패키지를 빌드해두었습니다.

### 4.1 Clone (이미 되어 있음)
```zsh
cd ~/2D-SLAM-Team-Project/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### 4.2 Build
```zsh
cd ~/2D-SLAM-Team-Project/robot_ws
colcon build --symlink-install --packages-select explore_lite_msgs explore_lite multirobot_map_merge
source install/setup.zsh
```
### 4.3 파라미터 변경
/home/taeyoung/workspace/2D-SLAM-Team-Project/robot_ws/src/m-explore-ros2/explore/config/params.yaml
의 값을 변경해야함

/**:
  ros__parameters:
    robot_base_frame: base_link
    return_to_init: true
    #edited by tyk
    #costmap_topic: map
    costmap_topic: explore_map
    #costmap_updates_topic: map_updates
    costmap_updates_topic: explore_map_updates
    visualize: true
    planner_frequency: 0.15
    progress_timeout: 30.0
    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    #min_frontier_size: 0.75
    min_frontier_size: 0.05


### 4.4 실행 (Terminal 5 대신 사용 가능)
Gazebo(Terminal 1~2) + Cartographer(Terminal 3) + Nav2(Terminal 4)를 켠 상태에서, `taeyoung_explorer` 대신 `explore_lite`로 자율 탐색을 해보고 싶다면:
```zsh
python3 ./robot_ws/scripts/map_threshold_node.py

ros2 launch explore_lite explore.launch.py
```
RViz2에서 `Add → By topic → /explore/frontiers`를 추가하면 탐색 중인 frontier(마커)를 확인할 수 있습니다.



### my instructions
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
  x_pose:=-6.7 \
  y_pose:=-3.16

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

cd ~/workspace/2D-SLAM-Team-Project/robot_ws && source install/setup.zsh
ros2 launch custom_burger my_turtlebot3_world.launch.py

ros2 launch ./assets/objects/spawn_rgb_balls.launch.py

ros2 launch ./assets/objects/spawn_household_obstacles.launch.py

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./custom_burger_cam_rpp.yaml

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=./maps/world_map.yaml \
  params_file:=./custom_burger_cam_rpp.yaml
```

ros2 run turtlebot3_teleop teleop_keyboard

python3 ./robot_ws/scripts/map_threshold_node.py

# 기능 on/off · 임계값 · 토픽 이름의 기본값은 robot_ws/scripts/coverage_cleaner/parameters/
# (feature_flags.py, thresholds.py, node_settings.py) 에서 수정하거나 --ros-args -p 로 덮어쓴다.
python3 ./robot_ws/scripts/coverage_cleaner/coverage_cleaner.py

cd ~/2D-SLAM-Team-Project/robot_ws
source install/setup.zsh
ros2 launch explore_lite explore.launch.py

ros2 run nav2_map_server map_saver_cli -f ./map

ros2 bag record -o nav2_debug \
  /cmd_vel \
  /cmd_vel_nav \
  /evaluation \
  /odom


ros2 bag info nav2_debug

python3 analyze_nav2_bag.py nav2_debug


python3 ./robot_ws/scripts/coverage_cleaner/coverage_cleaner.py --ros-args \
  -p use_sim_time:=true \
  -p send_to_nav2:=false \
  -p tf_timeout_sec:=1.5 \
  -p tracker_max_gap_sec:=1.5


python3 ./robot_ws/scripts/coverage_cleaner/coverage_cleaner.py --ros-args \
  -p use_sim_time:=true \
  -p tf_timeout_sec:=1.5 \
  -p tracker_max_gap_sec:=1.5

python3 ./robot_ws/scripts/coverage_cleaner/coverage_cleaner.py --ros-args \
  -p use_sim_time:=true \
  -p tf_timeout_sec:=1.5 \
  -p tracker_max_gap_sec:=1.5 \
  -p configure_goal_checker:=false \
  -p check_endpoint:=false \
  -p check_safe_position:=false \
  -p reclean_unvisited:=false

training/.venv/bin/python robot_ws/scripts/obstacle_reporter/obstacle_reporter.py --ros-args -p use_sim_time:=true

### For real robot

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=./custom_burger.yaml

python3 ./robot_ws/scripts/map_threshold_node.py

ros2 launch explore_lite explore.launch.py

ros2 run nav2_map_server map_saver_cli -f ./maps/my_map



robot_ws

cd ~/workspace/2D-SLAM-Team-Project/robot_ws && source install/setup.zsh
ros2 launch custom_burger my_turtlebot3_world.launch.py

ros2 launch ./assets/objects/spawn_household_obstacles.launch.py

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=./maps/world_map.yaml \
  params_file:=./custom_burger_cam_rpp.yaml

python3 ./robot_ws/scripts/map_threshold_node.py

python3 ./robot_ws/scripts/coverage_cleaner/coverage_cleaner.py --ros-args \
  -p use_sim_time:=true \
  -p tf_timeout_sec:=1.5 \
  -p tracker_max_gap_sec:=1.5 \
  -p configure_goal_checker:=false \
  -p check_endpoint:=false \
  -p check_safe_position:=false \
  -p reclean_unvisited:=false

서버가 먼저 켜져 있어야 함
cd ~/workspace/2D-SLAM-Team-Project
source robot_ws/install/setup.zsh
training/.venv/bin/python robot_ws/scripts/obstacle_reporter/obstacle_reporter.py --ros-args -p use_sim_time:=true


server

cd ./server
source .venv/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

cd ./server
source /opt/ros/humble/setup.zsh
python3 ./scripts/ros2_bridge_node.py --ros-args \
  -p server_host:=127.0.0.1 -p server_port:=8000 -p use_sim_time:=true


app

cd app
flutter run --android-skip-build-dependency-validation
