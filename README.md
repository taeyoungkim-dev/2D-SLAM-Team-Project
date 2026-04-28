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

---

## 🚀 2. How to Run (실행 방법)

성공적인 2D SLAM 시뮬레이션을 위해 총 3개의 터미널 창을 사용합니다.

### [Terminal 1] Gazebo 시뮬레이션 실행
원하는 맵을 선택하여 가상 환경과 로봇을 소환합니다. (기본값: House 맵)

```zsh
# 1. House Map (SLAM 테스트 권장)
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# 2. World Map (기본 장애물 맵)
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 3. Empty World (장애물 없는 빈 맵)
# ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### [Terminal 2] 2D SLAM (Cartographer) 및 RViz2 실행
로봇의 LiDAR 센서 데이터를 시각화하고 지도를 그리기 시작합니다. 가상 환경 동기화를 위해 반드시 `use_sim_time:=true` 파라미터를 추가해야 합니다.

```zsh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
> **Tip:** RViz2 창이 열리면 왼쪽 하단 `Add` 버튼을 눌러 `By topic` 탭에서 `/camera/image_raw`를 추가하면 로봇의 1인칭 카메라 뷰를 실시간으로 확인할 수 있습니다.

### [Terminal 3] Teleop (키보드 원격 조종)
방향키(W, A, S, D, X)를 이용해 로봇을 이동시키며 맵 구석구석을 스캔합니다.

```zsh
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 💾 3. Save the Map (지도 저장하기)

맵 스캔이 충분히 완료되었다면, 새로운 터미널을 열고 생성된 지도를 저장합니다. (`map`이라는 이름의 `.yaml`과 `.pgm` 파일이 생성됩니다.)

```zsh
ros2 run nav2_map_server map_saver_cli -f ~/map
```