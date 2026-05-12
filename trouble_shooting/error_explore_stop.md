### 문제점
![Error : Can not load p1.img](/assets/images/error_stop_1.png)
![Error : Can not load p2.img](/assets/images/error_stop_2.png)

* **증상:** 터틀봇(TurtleBot3 Waffle Pi)이 `explore_lite` 패키지를 이용한 자율 탐사 중, 미탐사 구역(Frontier)이 문 너머에 존재함에도 불구하고 좁은 방문턱을 넘지 못하고 탐사를 조기 종료하는 현상 발생.
* **에러 로그:** `Failed to create path` (Nav2) 및 `No frontiers found, stopping.` (explore_lite)

### 원인
현재 로봇이 문을 통과하지 못하는 원인은 하드웨어 제약, 지도 격자 해상도(Resolution), 그리고 패키지 간의 데이터 덮어쓰기 문제가 복합적으로 작용한 결과임.

**1. 물리적 제약과 코스트맵(Costmap) 해상도의 충돌**
* 가제보(Gazebo) 하우스 맵의 방문 폭은 약 35cm이며, 와플 파이의 폭(Footprint)은 28cm로 물리적인 여유 공간은 약 7cm(양옆 3.5cm)에 불과함.
* 하지만 ROS 2 코스트맵의 기본 격자 해상도(Resolution)는 `0.05m(5cm)`임.
* 5cm 단위의 투박한 픽셀로 지도를 인식하기 때문에, 28cm짜리 로봇 히트박스가 35cm의 문을 통과할 때 미세한 축 틀어짐만 발생해도 벽 픽셀과 충돌하는 것으로 계산됨 (물리적 여유 공간이 해상도보다 작아서 통과 불가 판정).

**2. Cartographer의 해상도 덮어쓰기 (Override)**
* 위 1번 문제를 해결하기 위해 Nav2 파라미터(`custom_waffle_pi.yaml`)에서 해상도를 `0.02m`로 정밀하게 낮추었으나 적용되지 않음.
* 지도를 최초로 생성하여 배포하는 Cartographer 패키지 내부 설정 파일(`.lua`)에 해상도가 `0.05m`로 하드코딩되어 있기 때문임.
* 결과적으로 Nav2가 Cartographer의 5cm 단위 맵 데이터를 그대로 덮어쓰게 되어, 여전히 좁은 문을 5cm 블록으로 인식함.

**3. Nav2와 explore_lite의 상호작용에 의한 탐사 강제 종료**
* 라이다(LiDAR) 센서는 문 너머의 빈 공간을 인식하여 다음 방 깊은 곳에 '프론티어(Frontier)'를 정상적으로 생성함.
* 하지만 Nav2(경로 계획기)가 위 1, 2번의 이유로 "해당 프론티어로 향하는 문을 통과할 수 없다(Failed to create path)"고 판단함.
* `explore_lite` 노드는 해당 프론티어를 '접근 불가(Unreachable) 구역'으로 간주하여 탐색 목록에서 영구 삭제(블랙리스트 처리)해버림.
* 남은 프론티어가 없다고 판단한 `explore_lite`는 최종적으로 탐사를 중단함.
---
해결방법

로봇의 사이즈를 작게 설정하여 nav2가 갈 수 있는 길이라고 선택하게 만들기

custom_waffle_pi.yaml
```
# footprint: "[[-0.149, -0.149], [0.149, -0.149], [0.149, 0.149], [-0.149, 0.149]]"
robot_radius: 0.02
inflation_radius: 0.05
```
---
문제 발생

좁은 방 입구를 못넘고 있음
---
원인 분석

![Error : Can not load map.png](/assets/images/error_map.png)
![Error : Can not load error_costmap.png](/assets/images/error_costmap.png)
위 이미지를 보면 충분히 frontier를 만들 수 있는 선분이 있음을 확인할 수 있다.

원인 추정
1. Frontier이 어떠한 이유로 만들어지지 않아서 탐사가 되지 않고 있다.
or
2. Frontier는 정확히 만들어졌지만 문 틈 사이가 좁아서 탐사할 수 없는 공간이라고 파악되어 탐사를 중단했다.

둘 중 어느것이 원인인지 밝혀야 한다.

/scripts/frontier_logger.py를 통해 아무런 frontier_log가 나오지 않는 것을 확인. 하지만 로봇은 방 안에서 움직이고 있으므로 explore_lite 내부에서 frontier topic을 의도적으로 숨긴 것으로 추정됨.

정보 검색 중 explore_lite github에서 tb3 burger로 잘 동작하는 것을 확인.
따라서 모델을 burger로 변경해서 실험해보려고 함.

---
모델을 burger로 변경하는 과정


/param 안에 /humble 이라는 폴더가 있는 것을 발견. 그 안에 humble에 맞는 정상적인 .yaml 파일을 발견

nav2 실행 명령어를 수정하여 burger 기준으로 param을 줄 수 있도록 수정
```zsh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true params_file:=/opt/ros/humble/share/turtlebot3_navigation2/param/humble/burger.yaml
```

zshrc 파일을 수정
```
export TURTLEBOT3_MODEL=burger
```

---
문제 발생

로봇를 burger로 교체하여도 방 입구를 지나갈 수 없음.

---
원인 분석

![Error : Can not load error_map_2.png](/assets/images/error_map_2.png)

위 이미지를 보면 돌 수 있는 벽의 안전구역 margin이 보라색으로 크게 펼쳐져 있는 것을 볼 수 있음. 로봇이 지나갈 공간이 부족함. 따라서 margin을 줄이는 작업이 필요함.

---
해결 방법

/opt/ros/humble/share/turtlebot3_navigation2/param/humble/burger.yaml을 복사하여 custom_burger.yaml을 생성.

custom_burger.yaml를
```
inflation_radius: 0.2
```
로 수정.

nav2 실행시
```zsh
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./custom_burger.yaml
```
로 실행

---
문제 발생

![Error : Can not load error_map_3.png](/assets/images/error_map_3.png)
Margin은 확실히 줄었지만 아직도 빈 공간 탐사를 진행하지 않고 탐사를 끝내버림.


### 해결
(추후 적용될 해결 방안 작성 예정)