# 2D-SLAM-Team-Project
2D SLAM Project

# TurtleBot3 2D SLAM in Gazebo (ROS 2 Humble)

본 프로젝트는 ROS 2 Humble 환경에서 TurtleBot3 (Waffle Pi)를 활용하여 Gazebo 가상 환경에서 2D SLAM(Cartographer)을 수행하고 지도를 생성하는 시뮬레이션 환경 구축 가이드입니다.

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
# 신형 Gazebo 패키지 및 찌꺼기 제거
# 신형 Gazebo가 깔려있지 않다면 스킵해도 됨.
sudo apt remove -y gz-tools2 "^ignition-*" "^libignition-*" "^libgz-*"
sudo apt autoremove -y
sudo apt update --fix-missing
sudo apt --fix-broken install

# Gazebo 11 및 ROS 2 브릿지 설치
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs

#자율탐색 ros2 패키지
mkdir -p src
cd src
git clone https://github.com/robo-friends/m-explore-ros2.git

#자신의 Directory 위치로
cd ~/workspace/2D-SLAM-Team-Project
colcon build
```



### 1.2 TurtleBot3 및 SLAM 필수 패키지 설치
로봇 구동과 매핑에 필요한 공식 패키지들을 설치합니다.

```zsh
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
```

### 1.3 Gazebo 모델 데이터 수동 다운로드 (초기 로딩 지연 방지)
Gazebo 첫 실행 시 모델 다운로드로 인한 응답 없음(Timeout) 현상을 방지하기 위해 미리 모델 데이터를 클론합니다.

```zsh
cd ~/.gazebo
rm -rf models
git clone https://github.com/osrf/gazebo_models.git models
```

### 1.4 환경 변수 설정 (`~/.zshrc`)
TurtleBot3 Waffle Pi 모델을 기본으로 사용하도록 `zsh` 환경에 등록합니다.

```zsh
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.zshrc
source ~/.zshrc
```

### 1.5 custom_waffle_pi.yaml 생성
1. 파라미터 파일 로컬 복사 (custom_waffle_pi.yaml 생성)

```zsh
cd ~/workspace/2D-SLAM-Team-Project
cp /opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml ./custom_waffle_pi.yaml
```
2. 문법 수정

```zsh
sed -i 's/nav2_navfn_planner::NavfnPlanner/nav2_navfn_planner\/NavfnPlanner/g' custom_waffle_pi.yaml
sed -i 's/nav2_behaviors::/nav2_behaviors\//g' custom_waffle_pi.yaml
```

## 🚀 2. How to Run (실행 방법)

성공적인 자율 탐색 및 2D SLAM 시뮬레이션을 위해 총 5개의 터미널 창을 순서대로 실행합니다.
*(모든 명령어는 워크스페이스 최상단인 `2D-SLAM-Team-Project` 폴더 안에서 실행한다고 가정합니다.)*

### [Terminal 1] Gazebo 시뮬레이션 실행 (환경 구성)
원하는 맵을 선택하여 가상 환경과 로봇을 소환합니다. (기본값: House 맵)

```zsh
export TURTLEBOT3_MODEL=waffle_pi

# 1. House Map (SLAM 테스트 권장)
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# 2. World Map (기본 장애물 맵)
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 3. Empty World (장애물 없는 빈 맵)
# ros2 launch turtlebot3_gazebo empty_world.launch.py
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
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
> **Tip:** RViz2 창이 열리면 왼쪽 하단 `Add` 버튼을 눌러 `By topic` 탭에서 `/camera/image_raw`를 추가하면 로봇의 1인칭 카메라 뷰를 실시간으로 확인할 수 있습니다.

### [Terminal 4] Navigation2 실행 (자율주행 엔진)
자율 탐색 시 장애물을 회피하며 주행하기 위해 Nav2 엔진을 켭니다. ROS 2 버전 호환성을 위해 수정된 커스텀 파라미터 파일(`custom_waffle_pi.yaml`)을 사용합니다.

```zsh
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./custom_waffle_pi.yaml
```

### [Terminal 5] 자율 탐색 (Explore Lite) 또는 수동 조종
**옵션 A: 자율 탐색 시작 (권장)**
로봇이 미지의 영역(Frontier)을 스스로 찾아다니며 맵을 100% 완성합니다.
```zsh
source install/setup.zsh
ros2 launch explore_lite explore.launch.py use_sim_time:=true robot_base_frame:=base_link min_frontier_size:=0.2
```

**옵션 B: 수동 조종 (Teleop)**
키보드 방향키(W, A, S, D, X)를 이용해 로봇을 직접 조종하고 싶을 때 사용합니다. (이 경우 Terminal 4와 Terminal 5의 옵션 A는 실행하지 않아도 됩니다.)
```zsh
ros2 run turtlebot3_teleop teleop_keyboard
```
## 💾 3. Save the Map (지도 저장하기)

맵 스캔이 충분히 완료되었다면, 새로운 터미널을 열고 생성된 지도를 저장합니다. (`map`이라는 이름의 `.yaml`과 `.pgm` 파일이 생성됩니다.)

```zsh
ros2 run nav2_map_server map_saver_cli -f ~/map
```