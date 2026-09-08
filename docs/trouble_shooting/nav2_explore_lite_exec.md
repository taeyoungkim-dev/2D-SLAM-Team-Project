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

적절한 빈 공간이 없어서 탐색이 종료됐다고 파악.
default 세팅은 min_frontier_size:=0.75
너무 큼
min_frontier_size:=0.2로 줄이기

```zsh
ros2 launch explore_lite explore.launch.py use_sim_time:=true robot_base_frame:=base_link min_frontier_size:=0.2
```

시도했음에도
```zsh
[explore-1] [WARN] [1778213678.750202368] [explore_node]: No frontiers found, stopping. 
```
같은 에러 발생

---

원인 재분석

현재 사용하는 라이브러리
- turtlebot3
- Nav2
- explore_lite

nav2에서 costmap에 margin을 크게 붙여서 explore_lite에서 min_frontier_size를 작게 설정해도 frontier를 찾을 수 없었다고 가정

따라서 nav2의 margin을 작게 수정

해결 방법

custom_waffle_pi.yaml 에서
inflation_radius: 0.5
의 값들을
inflation_radius: 0.1
로 수정

에러 발생
```zsh
[explore-1] [WARN] [1778478746.985216281] [explore_node]: [FrontierSearch] Could not find nearby clear cell to start search
[explore-1] [WARN] [1778478746.992391230] [explore_node]: No frontiers found, stopping.
```
---

원인 재분석
'Could not find nearby clear cell to start search'라는 log는 로봇의 바로 주변 범위
map이 그려지지 않아서 발생하는 문제라고 가정

로봇을 수동으로 조금씩 움직여서 로봇 발 밑의 map을 채워넣기

해결방법

nav2를 실행하고 나서 수동 조종으로 map 조금 채워넣고 explore_lite 실행

에러발생
```zsh
[explore-1] [WARN] [1778493446.874897806] [explore_node]: [FrontierSearch] Could not find nearby clear cell to start search
[explore-1] [WARN] [1778493446.877085259] [explore_node]: No frontiers found, stopping.
```
---

원인 재분석

현재 explore_lite는 /map 토픽을 기준으로 frontier를 찾고 있다.
/map은 ros2 cartographer에서 확률 적으로 값을 부여받음.
하지만 ros1에서는 0,1 binary 값으로 /map이 생성되었음.
explore_lite는 binary map 방식으로 frontier를 만들기 때문에
확률적 int값을 가지고 있는 /map을 쓰지 않고 /costmap을 써서 해결할 수 있을 것이다.

해결방법

```zsh
ros2 launch explore_lite explore.launch.py use_sim_time:=true robot_base_frame:=base_link min_frontier_size:=0.2 costmap_topic:=/global_costmap/costmap
```
/global_costmap/costmap 파라미터 옵션 추가

에러발생
```zsh
[explore-1] [WARN] [1778494821.637382660] [explore_node]: [FrontierSearch] Could not find nearby clear cell to start search
[explore-1] [WARN] [1778494821.640963177] [explore_node]: No frontiers found, stopping.
```
---

원인

```zsh
[explore_node]: Waiting for costmap to become available, topic: map
```
위 log를 보면 아직도 /map을 참고해서 frontier를 생성하는 것을 알 수 있음.
따라서 costmap_topic:=/global_costmap/costmap 명령어가 적용되지 않을 것을 알 수 있음.
애초에 그런 파라미터를 만들어 놓지 않은듯

해결방법

```zsh
ros2 run explore_lite explore --ros-args -p use_sim_time:=true -p robot_base_frame:=base_link -p min_frontier_size:=0.2 -p costmap_topic:=/global_costmap/costmap
```
ros2 강제 파라미터 주입 명령어 --ros-args -p를 사용해서 costmap 파라미터 강제 주입

문제 발생

터틀봇이 탐사를 하긴 하지만 벽에 박으면서 탐사를 함.

---

원인 분석

이전에 변경한 custom_waffle_pi.yaml 파일의 inflation_radius와 min_frontier_size 파리미터 변경으로 탐사가 불가능한 좁은 곳도 탐사하려고 해서 문제가 발생하는 것으로 추측

해결방법

이전에 변경한 custom_waffle_pi.yaml 파일의 inflation_radius를 0.5로 되돌리고
min_frontier_size:=0.2 파라미터를 빼기
```zsh
ros2 run explore_lite explore --ros-args -p use_sim_time:=true -p robot_base_frame:=base_link -p min_frontier_size:=0.5 -p costmap_topic:=/global_costmap/costmap
```
custom_waffle_pi.yaml에서
```
inflation_radius: 0.25
```
로 수정


### 해결

custom_waffle_pi.yaml에서
```
inflation_radius: 0.25
```
로 수정

and

explore_lite 실행 명령어 수정
```zsh
ros2 run explore_lite explore --ros-args -p use_sim_time:=true -p robot_base_frame:=base_link -p min_frontier_size:=0.5 -p costmap_topic:=/global_costmap/costmap
```