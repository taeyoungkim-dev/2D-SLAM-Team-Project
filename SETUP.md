# 환경 구축 가이드 (SETUP)

이 저장소를 **처음 clone 한 사람**이 `robot_ws`(ROS 2 / Gazebo), `server`(FastAPI), `app`(Flutter)을 돌릴 수 있게 만드는 순서를 정리한 문서입니다.
(`training/` 학습 환경은 다루지 않습니다. 단, 장애물 인식 노드가 쓰는 YOLO 실행 환경은 [4장](#4-장애물-인식-노드-obstacle_reporter-yolo)에 있습니다.)

> 이 문서는 개발에 쓴 PC(Ubuntu 22.04)의 실제 설치 상태를 기준으로 썼습니다. 새 PC에서 처음부터 다시 돌려 검증한 것은 아니므로, 막히면 [8장](#8-자주-나는-문제)을 먼저 보세요.

## 0. 전체 구조와 가상환경을 저장소에 올리지 않은 이유

```text
[TurtleBot3 + Gazebo (ROS 2 Humble)] <-- ros2_bridge_node --> [FastAPI 서버] <-- REST/WebSocket --> [Flutter 앱]
        robot_ws/                          server/                                    app/
```

| 구성 | 위치 | 어디서 도는가 | 환경 |
|---|---|---|---|
| 로봇/시뮬레이션 | `robot_ws/` | Ubuntu 22.04 PC | **ROS 2 Humble(apt)** + 시스템 Python 3.10 |
| 서버 | `server/` | 같은 PC(또는 같은 LAN 의 PC) | Python 3.10 가상환경 `server/.venv` |
| 장애물 인식 | `robot_ws/scripts/obstacle_reporter/` | 시뮬레이션 PC | 가상환경 `training/.venv`(ultralytics) + ROS |
| 앱 | `app/` | Android 에뮬레이터/기기 | Flutter SDK + Android SDK |

- **가상환경(`.venv`)은 GitHub 에 올리지 않습니다.** 절대경로가 박혀 있어 다른 PC에서 동작하지 않고, `training/.venv` 는 PyTorch 때문에 약 6GB 입니다. 아래 명령 몇 줄로 다시 만들 수 있으므로 `.gitignore` 의 `.venv/` 설정을 그대로 둡니다.
- ROS 2 와 Gazebo 는 apt 로 시스템에 설치합니다(저장소에 포함할 수 없음).
- 이 문서는 **Ubuntu 22.04 + ROS 2 Humble + Gazebo 11(Classic)** 를 전제로 합니다. 쉘은 zsh 기준이고 bash 면 `setup.zsh` 를 `setup.bash` 로 바꾸세요.

## 1. 공통 준비

```bash
sudo apt update
sudo apt install -y git curl unzip build-essential python3-pip python3-venv
git clone <이 저장소 URL> ~/workspace/2D-SLAM-Team-Project
cd ~/workspace/2D-SLAM-Team-Project
```

- 이후 명령은 특별한 말이 없으면 **저장소 최상위**에서 실행한다고 가정합니다. 위치가 다르면 `SLAM_REPO_ROOT` 환경변수로 저장소 경로를 알려 줄 수 있습니다(통합 launch 용).
- 모든 장치(로봇 PC, 앱이 도는 폰)는 **같은 로컬 Wi-Fi/LAN** 에 있어야 합니다. 인터넷은 설치할 때만 필요하고 실행 중에는 필요 없습니다.

---

## 2. robot_ws (ROS 2 + Gazebo + Nav2)

### 2.1 ROS 2 Humble 설치
공식 문서 순서대로 설치합니다: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc     # bash 면 ~/.bashrc 와 setup.bash
```

### 2.2 Gazebo 와 TurtleBot3, Nav2, Cartographer

```bash
sudo apt install -y \
  gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-msgs \
  ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 \
  ros-humble-tf2-ros-py ros-humble-rviz2
```

- `turtlebot3_navigation2`(rviz 설정)와 `nav2_bringup` 은 `full_system.launch.py` 가 직접 사용합니다.
- 신형 Gazebo(Ignition/gz)와 Gazebo 11 이 섞이면 충돌하니, `gazebo`(Classic 11)만 설치하세요.

### 2.3 로봇 모델 환경변수

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.zshrc
source ~/.zshrc
```

### 2.4 Gazebo 모델 미리 받기 (첫 실행 멈춤 방지)
Gazebo 를 **한 번 실행**해서 `~/.gazebo` 가 생긴 뒤에 합니다.

```bash
cd ~/.gazebo && rm -rf models && git clone https://github.com/osrf/gazebo_models.git models
```

### 2.5 청소/브릿지 노드가 쓰는 Python 패키지
`coverage_cleaner`, `map_threshold_node`, `ros2_bridge_node` 는 **시스템 Python(3.10)** 으로 실행됩니다(가상환경 아님).

```bash
sudo apt install -y python3-numpy python3-scipy python3-opencv       # numpy 1.21, scipy 1.8 (opencv 는 reporter 자체 테스트용)
pip3 install --user "websockets>=12.0"                               # ros2_bridge_node 가 서버에 WebSocket 으로 연결
```

> `pip3 install --user numpy` 같은 업그레이드는 하지 마세요. ROS 2 Humble 의 Python 패키지가 apt 의 numpy 1.21 기준으로 빌드되어 있습니다.

### 2.6 워크스페이스 빌드

```bash
cd robot_ws
colcon build --symlink-install --packages-select custom_burger
source install/setup.zsh        # 새 터미널마다 필요 (bash 면 setup.bash)
ros2 pkg list | grep custom_burger     # custom_burger 가 나와야 한다
```

`custom_burger` 는 깊이 카메라가 달린 커스텀 TurtleBot3 모델과 launch 파일(`full_system.launch.py`, `my_turtlebot3_world.launch.py`, `my_house.launch.py`)을 담고 있어 **반드시 빌드**해야 합니다.

### 2.7 (선택) 자율 탐색 패키지 `m-explore-ros2`
지도를 새로 만들 때(frontier 탐색)만 필요합니다. **저장해 둔 지도(`maps/world_map.yaml`)로 청소하는 데모에는 필요 없습니다.**
`robot_ws/src/m-explore-ros2` 는 외부 저장소라 clone 하면 빈 폴더이므로 직접 받아야 합니다.

```bash
cd robot_ws/src && rm -rf m-explore-ros2
git clone https://github.com/robo-friends/m-explore-ros2.git
cd .. && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select explore_lite_msgs explore_lite multirobot_map_merge
```

`explore/config/params.yaml` 에서 `costmap_topic: explore_map`, `costmap_updates_topic: explore_map_updates` 로 바꿔야 합니다(자세한 값은 루트 `README.md` 4.3).

### 2.8 이 저장소에 이미 들어 있는 것
지도(`maps/`), Nav2 파라미터(`custom_burger_cam_rpp.yaml` 등), 장애물 모델/스폰 파일(`assets/objects/`)은 저장소에 포함되어 있어 따로 받을 것이 없습니다.

---

## 3. server (FastAPI)

```bash
cd server
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt      # fastapi, uvicorn[standard], python-multipart, aiosqlite, websockets
deactivate
```

- 검증된 버전: Python 3.10, fastapi 0.162, uvicorn 0.54, aiosqlite 0.22, python-multipart 0.0.32, websockets 16.1.
- **통합 launch(`full_system.launch.py`)는 `server/.venv/bin/uvicorn` 을 직접 실행하므로 가상환경 위치/이름을 바꾸면 안 됩니다.**
- 직접 실행할 때(필요하면):

```bash
cd server && source .venv/bin/activate
cp .env.example .env     # SERVER_PUBLIC_BASE_URL 을 이 PC 의 LAN IP 로 수정 (ip addr 로 확인)
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

- 방화벽을 쓴다면 앱이 접속할 수 있게 열어 둡니다: `sudo ufw allow 8000/tcp`
- 동작 확인: 브라우저에서 `http://<서버IP>:8000/docs`, 또는 `curl http://127.0.0.1:8000/api/health`
- 서버 테스트(선택): 가상환경에서 `pip install pytest httpx` 후 `cd server && pytest -q` (TestClient 가 다른 패키지를 요구하면 오류 메시지의 안내대로 설치).

---

## 4. 장애물 인식 노드 (obstacle_reporter, YOLO)

카메라 영상에서 장애물을 찾아 서버로 보고하는 노드이고, `full_system.launch.py` 가 `training/.venv/bin/python` 으로 실행합니다. (`reporter:=false` 로 launch 하면 이 장은 건너뛸 수 있습니다.)

```bash
python3 -m venv training/.venv
source training/.venv/bin/activate
pip install ultralytics            # torch, torchvision, opencv-python, numpy 가 함께 설치됨 (용량이 큼)
deactivate
```

- 검증된 버전: ultralytics 8.4, torch 2.14, numpy 2.2. GPU 가 없으면 CPU 로 동작합니다(`device` 파라미터 기본 자동 선택).
- ROS(`rclpy`)는 가상환경에 설치하지 않고, **`source /opt/ros/humble/setup.zsh` 후 실행**하면 가상환경 Python 이 시스템 ROS 패키지를 찾습니다.
- **모델 가중치는 저장소에 포함되어 있습니다**: `training/weights/obstacles_v1_best.pt`(약 5MB, `training/.gitignore` 에서 이 파일만 예외로 풀어 둠). clone 하면 따로 받을 것이 없습니다. (사전학습 `yolo11n.pt` 는 학습할 때만 쓰므로 제외 상태입니다.)
- 단독 실행: `training/.venv/bin/python robot_ws/scripts/obstacle_reporter/obstacle_reporter.py --ros-args -p use_sim_time:=true`
- 자체 테스트(ROS/모델 불필요): `python3 robot_ws/scripts/obstacle_reporter/self_test.py`

---

## 5. app (Flutter, Android)

### 5.1 Flutter SDK 설치
Flutter 를 `snap` 이나 압축 파일이 아니라 **git clone 으로** 설치합니다(그래야 `flutter` 가 "not a clone of the GitHub project" 오류를 내지 않습니다).

```bash
sudo apt install -y git curl unzip xz-utils zip libglu1-mesa
mkdir -p ~/development && cd ~/development
git clone https://github.com/flutter/flutter.git -b stable
echo 'export PATH="$HOME/development/flutter/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
flutter --version
```

- 검증된 버전: Flutter 3.50.1 (stable), Dart 3.14. 이 앱의 요구 조건은 `pubspec.yaml` 의 `sdk: ">=3.3.0 <4.0.0"` 입니다.

### 5.2 Android 개발 환경

1. **Android Studio** 를 설치합니다(<https://developer.android.com/studio>). 처음 실행하면 Android SDK 가 `~/Android/Sdk` 에 설치됩니다.
2. Android Studio → *More Actions → SDK Manager* 에서 다음을 설치합니다.
   - SDK Platform: Android 36 이상(검증: 35, 36, 37), SDK Build-Tools 36
   - **SDK Tools 탭 → "Android SDK Command-line Tools (latest)"** ← 빠뜨리기 쉽습니다. 없으면 `flutter doctor` 가 `cmdline-tools component is missing` 을 냅니다.
   - Android Emulator, Platform-Tools
3. 환경변수와 라이선스:

```bash
echo 'export ANDROID_HOME="$HOME/Android/Sdk"' >> ~/.zshrc
source ~/.zshrc
flutter config --android-sdk "$ANDROID_HOME"
flutter doctor --android-licenses      # 전부 y
flutter doctor -v                      # Flutter / Android toolchain 이 ✓ 인지 확인
```

4. **JDK 17 이상**이 필요합니다(Gradle 9 / Java 17 타깃). Android Studio 에 JDK(JBR)가 들어 있어 보통 자동으로 쓰입니다. `flutter doctor -v` 의 Java 항목이 실패하거나 `java -version` 이 11 이면: `sudo apt install openjdk-17-jdk` 후 `flutter config --jdk-dir /usr/lib/jvm/java-17-openjdk-amd64`.
5. 에뮬레이터: Android Studio → *Device Manager → Create Device* 로 만들고 실행합니다(Google Play 이미지 권장). 실제 기기는 개발자 옵션의 USB 디버깅을 켜고 연결합니다.

> `flutter doctor` 의 *Linux toolchain* 항목(clang, GTK)은 Linux 데스크톱 앱용이라 **무시해도 됩니다**(이 프로젝트는 Android 대상).

### 5.3 앱 의존성 설치와 실행

```bash
cd app
flutter pub get          # provider, http, web_socket_channel, shared_preferences, intl, cupertino_icons
flutter devices          # 에뮬레이터/기기가 보여야 한다
flutter run              # 기기가 여러 개면 -d <기기 id>
```

- `app/android/` 는 저장소에 포함되어 있습니다. **`flutter create .` 로 덮어쓰지 마세요.** `AndroidManifest.xml` 에 `INTERNET` 권한과 `usesCleartextTraffic="true"`(로컬 서버와 http/ws 통신) 를 직접 넣었고, 이게 없으면 앱이 서버에 연결하지 못합니다. (`app/android` 가 없는 상태라면 `flutter create --platforms=android .` 후 이 두 항목을 `<application>` / `<manifest>` 에 추가하세요.)
- 첫 빌드는 Gradle/의존성 다운로드 때문에 몇 분 걸립니다.
- 빌드 중 *"Android Gradle Plugin version 9.1.0 ... is below the recommended minimum"* 같은 검증 오류가 나면:

```bash
flutter run --android-skip-build-dependency-validation
flutter build apk --debug --android-skip-build-dependency-validation     # APK 만 만들 때
```

### 5.4 서버 주소
- **에뮬레이터**: 앱 설정(⚙️)의 서버 IP 에 **무엇을 넣어도** 호스트 PC 의 서버(`10.0.2.2:8000`)에 자동으로 연결됩니다.
- **실제 폰**: 폰과 서버 PC 가 같은 Wi-Fi 에 있어야 하고, 설정에 서버 PC 의 LAN IP 와 포트 8000 을 넣습니다(`ip addr` 로 확인). IP 를 비우거나 틀리게 넣으면 같은 네트워크에서 서버를 찾아봅니다.
- 앱 단위 테스트: `cd app && flutter test` (`flutter analyze` 는 deprecated 안내 info 만 나옵니다).

---

## 6. 한 번에 실행하기 (환경 구축이 끝난 뒤)

```bash
cd ~/workspace/2D-SLAM-Team-Project
source robot_ws/install/setup.zsh
export TURTLEBOT3_MODEL=burger
ros2 launch robot_ws/src/custom_burger/launch/full_system.launch.py
```

1. Gazebo 와 rviz 가 뜨면 rviz 에서 **2D Pose Estimate** 로 로봇 위치를 직접 찍습니다(5분 안에).
2. 서버(uvicorn)와 `ros2_bridge_node` 가 자동으로 시작됩니다. 서버 주소는 launch 가 이 PC 의 LAN IP 를 자동 감지합니다(직접 지정: `public_host:=192.168.x.x`).
3. 앱을 실행하고 **청소 시작**을 누릅니다.

옵션: `gui:=false rviz:=false`(창 없이), `reporter:=false`(장애물 인식 제외), `server:=false`(서버/브릿지 제외), `auto_plan:=true`(앱 없이 바로 청소). 각 단계를 따로 실행하는 방법은 루트 `README.md` 를 보세요.

## 7. 환경이 맞는지 확인하는 명령 모음

```bash
ros2 pkg list | grep -E "custom_burger|turtlebot3_gazebo|nav2_bringup|turtlebot3_navigation2"   # 4개 모두
python3 -c "import numpy, scipy, cv2, websockets; print('system python OK')"
server/.venv/bin/python -c "import fastapi, uvicorn, aiosqlite; print('server OK')"
training/.venv/bin/python -c "import ultralytics; print('yolo OK')"        # reporter 를 쓸 때만
ls training/weights/obstacles_v1_best.pt                                   # reporter 를 쓸 때만
python3 robot_ws/scripts/coverage_cleaner/coverage_cleaner.py --self-test  # 청소 로직 자체 테스트 (24개)
cd app && flutter doctor -v && flutter test
```

## 8. 자주 나는 문제

| 증상 | 원인 / 해결 |
|---|---|
| `package 'custom_burger' not found` | `robot_ws` 를 빌드하지 않았거나 `source robot_ws/install/setup.zsh` 를 안 함 (2.6) |
| `ModuleNotFoundError: websockets` (ros2_bridge_node) | 시스템 Python 에 설치 필요: `pip3 install --user "websockets>=12.0"` (2.5) |
| `ModuleNotFoundError: scipy` (cleaner) 또는 `cv2` (reporter self_test) | `sudo apt install python3-scipy python3-opencv` (2.5) |
| `weights 파일이 없습니다` (obstacle_reporter) | `training/weights/obstacles_v1_best.pt` 가 없음: 커밋/푸시에서 빠졌는지 확인 (4장) |
| `server/.venv/bin/uvicorn` 없음으로 launch 실패 | 3장의 가상환경을 `server/.venv` 로 만들지 않음 |
| Gazebo 첫 실행에서 멈춤/Timeout | 2.4 의 Gazebo 모델 미리 받기 |
| `flutter` 가 "not a clone of the GitHub project" | Flutter 를 clone 이 아닌 방식으로 설치함 → 5.1 대로 다시 설치 |
| `cmdline-tools component is missing` | SDK Manager → SDK Tools → Command-line Tools 설치 (5.2) |
| Android 빌드가 AGP 버전 검증으로 실패 | `--android-skip-build-dependency-validation` 추가 (5.3) |
| 앱이 서버에 연결 안 됨 (실제 폰) | 같은 Wi-Fi 인지, 서버가 `--host 0.0.0.0` 인지, 방화벽 8000 포트, 폰에서 `http://<서버IP>:8000/docs` 접속 확인 |
| 앱이 서버에 연결 안 됨 (에뮬레이터) | `app/android` 의 `AndroidManifest.xml` 에 `usesCleartextTraffic` 이 있는지 확인 (5.3) |
| rviz 에서 로봇이 안 보이거나 cleaner 가 시작 안 함 | 2D Pose Estimate 를 찍지 않음. TF `map→base_footprint` 가 있어야 다음 단계가 시작됨 |
