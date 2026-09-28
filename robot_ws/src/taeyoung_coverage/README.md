# taeyoung_coverage

Cartographer가 생성한 `/map` (`nav_msgs/OccupancyGrid`)을 입력으로 받아,
로봇청소기처럼 자유 공간을 빠짐없이 주행하는 **Complete Coverage Path**를
계산하고, Nav2 Controller Server의 `FollowPath` Action으로 실행시키는
ROS 2 Python 패키지. 상세 요구사항은 [`README_coverage`](./README_coverage)
참고.

```text
Cartographer
    ↓
/map (nav_msgs/OccupancyGrid)
    ↓
map_utils.py        (지도 분류, C-space 팽창, A*, 좌표 변환 — ROS 비의존)
    ↓
coverage_planner.py  (Boustrophedon 지그재그 경로 생성 — ROS 비의존)
    ↓
coverage_node.py     (ROS2 wiring: /map 구독, TF, Path publish, FollowPath 호출)
    ↓
nav_msgs/Path → nav2_msgs/action/FollowPath → Nav2 Controller Server → /cmd_vel
```

## 1. 패키지 구조

```text
taeyoung_coverage/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md                  (이 파일)
├── README_coverage            (원본 요구사항 스펙)
├── resource/taeyoung_coverage
├── taeyoung_coverage/
│   ├── __init__.py
│   ├── map_utils.py            # ROS 비의존, pure NumPy/SciPy
│   ├── coverage_planner.py     # ROS 비의존, pure Python
│   ├── coverage_tracker.py     # ROS 비의존, pure Python (visited/remaining)
│   └── coverage_node.py        # ROS 2 노드 (rclpy 필요)
└── test/
    ├── test_map_utils.py        # ROS 없이 실행 가능
    └── test_coverage_planner.py # ROS 없이 실행 가능
```

## 2. 코드 재사용 vs 신규 구현

기존 `taeyoung_explorer/explorer_node.py`에서 재사용한 패턴:

- `/map` 구독 QoS (`RELIABLE` + `TRANSIENT_LOCAL`)
- `OccupancyGrid.data → np.array(..., dtype=np.int8).reshape((height, width))`
- `scipy.ndimage.label` 기반 connected component
- `tf2_ros.Buffer`/`TransformListener`로 `map → base_link` 위치 조회
- `ActionClient` + `goal_response_callback`/`get_result_callback` 콜백 패턴
  (다만 `NavigateToPose` 대신 `FollowPath` 사용, `status==4` 매직넘버 대신
  `action_msgs.msg.GoalStatus.STATUS_SUCCEEDED` 사용)

재사용하지 **않은** 것: frontier 탐색 알고리즘 자체(용도가 다름).

신규로 만든 파일: `map_utils.py`, `coverage_planner.py`, `coverage_tracker.py`,
`coverage_node.py`, `test/test_map_utils.py`, `test/test_coverage_planner.py`.

## 3. 핵심 알고리즘 설명

### 3.1 지도 분류 및 C-space (`map_utils.py`)

1. `classify_map`: OccupancyGrid 값(-1/0~100)을 `FREE`/`OCCUPIED`/`UNKNOWN` 3
   클래스로 분류 (`occupied_threshold` 파라미터, 기본 65).
2. `compute_traversable_map`: `OCCUPIED`와 `UNKNOWN`을 모두 장애물로 취급하여
   (README_coverage 16장 규칙 5: "unknown을 free로 간주하지 말 것") 원형
   커널로 `robot_radius + coverage_clearance` 만큼 팽창시킨 뒤, 그 팽창 영역과
   겹치지 않는 free 셀만 `traversable=True`로 남긴다. → C-space.
3. `robot_radius_from_footprint`: `custom_burger/urdf/turtlebot3_burger.urdf`의
   실제 collision box(`0.140 x 0.140 m`)의 외접원 반지름
   (`hypot(0.140,0.140)/2 ≈ 0.099 m`)을 기본값으로 사용 — 스펙이 명시적으로
   금지한 "naive 17.8cm" 대신 실제 URDF 치수 기반.

### 3.2 좌표계 규칙

- NumPy / OccupancyGrid 인덱스: **`[y, x]`** (row, col)
- ROS world 좌표: **`[x, y]`** (미터)
- `pixel_to_world(y, x, meta)` ↔ `world_to_pixel(world_x, world_y, meta)`는
  셀 중심(cell-center) 기준으로 정확히 왕복 변환되도록 설계 (Test 5로 검증).

### 3.3 Boustrophedon 커버리지 (`coverage_planner.py`)

1. `connected_components`로 traversable 영역을 방(room) 단위로 분리하고,
   너무 작은 영역(`min_region_cells`)은 무시한다.
2. `order_regions`: 로봇 시작 위치(TF로 조회, 없으면 가장 큰 영역)에서 가까운
   영역부터 그리디하게 방문 순서를 정한다.
3. `plan_region_lanes`: 각 영역에서 `lane_spacing_m`(지도 resolution과는 독립적인
   별도 파라미터) 간격으로 행(row)을 샘플링하고, 각 행에서 `row_intervals`로
   **연속된 free 구간**을 모두 찾는다. 장애물이 한 행을 두 개로 나누면 자동으로
   두 개의 별도 구간(=두 개의 lane segment)이 생긴다 (V2: 장애물 인식).
   왼→오, 오→왼을 lane마다 번갈아 지그재그를 만든다.
4. `connect_pixel_points`: 연속된 waypoint 사이를 먼저 **직선 시야(line-of-sight,
   Bresenham)** 로 연결 시도하고, 막히면 **A\*** 로 우회 경로를 찾는다. 두 방법
   모두 실패하면 (README_coverage 16장 규칙 6 금지: "떨어진 free interval을
   직선으로 연결하기") 그 연결을 **포기**하여 안전하지 않은 직선 연결을
   만들지 않는다.
5. `simplify_path`: 일직선 상의 중복 경유점을 cross-product 판정으로 제거하여
   Nav2에는 세그먼트의 끝점/꺾이는 점만 전달한다 (규칙 1: "모든 cell을 waypoint로
   보내지 않는다").
6. `validate_path`: 최종 pixel 경로가 지도 범위 내에 있고 모든 점·구간이
   traversable한지 검증, 위반 시 `CoveragePlanningError` 발생 → Nav2로 전달 안 함.
7. `pixels_to_waypoints`: 각 waypoint의 `yaw`를 다음 점을 향하는 `atan2`로
   계산하여 회전 방향까지 포함한 `nav_msgs/Path`를 만들 수 있게 한다.

### 3.4 Visited/Remaining 추적 (`coverage_tracker.py`)

- 로봇이 지나간 위치(TF)를 중심으로 `coverage_width`(로봇이 한 번에 실제로
  청소하는 폭) 반경의 원형 영역을 `visited_map`에 매 주기 누적.
- `remaining_map = cleanable_map & ~visited_map` 형태로 미청소 영역을 계산
  (README_coverage 8장). 디버그용 `/coverage_visited_map`,
  `/coverage_remaining_map` OccupancyGrid 토픽으로 publish.
- V1~V2 범위에서는 이 정보를 재계획에 아직 사용하지 않음(스펙에서 "한 번에
  완성할 필요는 없다"고 명시한 부분 — 4장 "다음 개선 단계" 참고).

### 3.5 Nav2 연동 (`coverage_node.py`)

- `NavigateToPose`가 **아니라 `nav2_msgs/action/FollowPath`** 사용
  (규칙 3: "Nav2 global planner에게 coverage path 생성을 맡기지 않는다").
  `FollowPath.Goal.path`에 우리가 만든 `nav_msgs/Path`를 그대로 넣고,
  `controller_id`는 기본값(빈 문자열 → nav2 params의 default controller)을 사용.
- `/map` 콜백에서 `classify_map → compute_traversable_map → plan_coverage_path`를
  한 번 실행해 경로를 만들고(`auto_plan` 파라미터로 자동/수동 전환 가능),
  `/coverage_path`에 publish한 뒤 `send_to_nav2=True`이면 즉시 FollowPath 목표로
  전송한다.

## 4. ROS 2 Dependency 목록 (`package.xml`)

- `rclpy`
- `nav_msgs` (`OccupancyGrid`, `Path`)
- `geometry_msgs` (`PoseStamped`)
- `nav2_msgs` (`FollowPath` action)
- `action_msgs` (`GoalStatus`)
- `tf2_ros` (`Buffer`, `TransformListener`, `TransformException`)
- `python3-numpy`
- `python3-scipy`

## 5. Ubuntu 빌드 명령

```bash
cd ~/2D-SLAM-Team-Project/robot_ws
# (최초 1회) 의존 패키지 설치가 필요하면:
# rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select taeyoung_coverage --symlink-install
source install/setup.bash
```

## 6. 실행 명령

```bash
# 0. (별도 터미널) TurtleBot3 + Cartographer + Nav2가 이미 실행 중이어야 함.
#    예: ros2 launch turtlebot3_cartographer cartographer.launch.py
#        ros2 launch turtlebot3_navigation2 navigation2.launch.py

# 1. taeyoung_coverage 노드 실행 (기본 파라미터로 자동 계획 + Nav2 전송)
ros2 run taeyoung_coverage taeyoung_coverage_node

# 2. 파라미터를 조정해서 실행하고 싶다면:
ros2 run taeyoung_coverage taeyoung_coverage_node --ros-args \
  -p lane_spacing:=0.2 \
  -p coverage_clearance:=0.03 \
  -p send_to_nav2:=false   # 먼저 /coverage_path만 확인하고 싶을 때
```

주요 파라미터 (`declare_parameter` 기본값):

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `occupied_threshold` | 65 | 이 값 이상이면 OCCUPIED로 분류 |
| `robot_radius` | ≈0.099 m | URDF footprint 외접원 반지름 |
| `coverage_clearance` | 0.02 m | 추가 안전 여유 |
| `lane_spacing` | 0.15 m | 지그재그 라인 간격 (map resolution과 독립) |
| `coverage_width` | 0.15 m | 한 번 지나가며 실제로 청소하는 폭 (visited 추적용) |
| `min_interval_cells` | 2 | 이보다 짧은 free 구간은 무시 |
| `min_region_cells` | 20 | 이보다 작은 영역은 무시 |
| `auto_plan` | true | `/map` 수신 시 자동으로 1회 경로 계산 |
| `send_to_nav2` | true | 계산된 경로를 즉시 FollowPath로 전송할지 여부 |
| `follow_path_action_name` | `follow_path` | Nav2 Controller Server의 action 이름 |

## 7. 테스트 명령

ROS 2 없이도 실행 가능한 순수 Python 테스트 (`map_utils.py`, `coverage_planner.py`):

```bash
cd ~/2D-SLAM-Team-Project/robot_ws/src/taeyoung_coverage
python3 -m pip install numpy scipy pytest   # 최초 1회
python3 -m pytest test/test_map_utils.py test/test_coverage_planner.py -v
```

현재 20개 테스트 모두 통과 확인됨 (Test 1~5 대응):

- Test 1 `test_v1_empty_rectangle_produces_zigzag_path` — 빈 방 지그재그
- Test 2 `test_v2_path_never_crosses_center_obstacle` — 중앙 장애물 회피
- Test 3 `test_v3_narrow_corridor_removed_from_traversable` — 좁은 통로 제외
- Test 4 `test_simplify_path_*`, `test_final_path_has_no_redundant_collinear_points` — 경로 단순화
- Test 5 `test_pixel_world_round_trip` — pixel↔world 왕복 변환

colcon 통합 테스트 (Ubuntu ROS 2 환경, ament_flake8/pep257 포함):

```bash
cd ~/2D-SLAM-Team-Project/robot_ws
colcon test --packages-select taeyoung_coverage
colcon test-result --verbose
```

## 8. RViz에서 경로 확인하는 방법

1. RViz2 실행: `rviz2` (또는 Nav2 launch가 이미 띄운 RViz 사용)
2. 좌측 하단 `Add` → `By topic` → `/coverage_path` → `Path` 선택
   (Fixed Frame은 `map`).
3. 디버그용 방문/미방문 영역을 보고 싶으면 같은 방식으로
   `/coverage_visited_map`, `/coverage_remaining_map`을 `Map` 디스플레이로 추가.
4. `send_to_nav2:=false`로 노드를 실행하면 실제 로봇을 움직이지 않고 경로
   모양만 먼저 검증할 수 있다.

## 9. 현재 구현의 한계와 다음 개선 단계

- **V1~V2까지만 구현**: 여러 개의 분리된 영역이 있을 때 `order_regions`는
  그리디 최근접 방식으로만 순서를 정한다 — 방 사이 최적 순회(TSP 근사) 미구현.
- **visited/remaining 기반 재계획 없음**: `coverage_tracker.py`가 방문 영역을
  추적하고 디버그 토픽으로 publish하지만, 아직 "커버리지가 끝났는데 remaining이
  남으면 재계획"하는 루프는 붙어 있지 않다 (스펙 8장에서 "한 번에 완성할 필요는
  없다"고 명시한 부분).
- **Circular footprint approximation**: 실제 polygon footprint 대신 URDF
  collision box의 외접원 반지름을 사용하는 보수적 근사이다(코드/문서에 명시).
  로봇이 사각형에 가까우므로 대각선 방향 여유가 다소 과도하게 보수적일 수 있다.
- **동적 재계획(re-planning) 없음**: 주행 중 새로운 장애물이 발견되어도
  (예: YOLO EVADING 로직) 이 패키지는 아직 이를 받아 경로를 다시 계산하지
  않는다. `taeyoung_coverage_node`에 별도 서비스/토픽을 추가해 EVADING 이후
  재계획을 트리거하는 것이 다음 단계다.
- **Controller 튜닝 미검증**: `FollowPath`가 요구하는 실제 controller
  plugin(`nav2_regulated_pure_pursuit_controller` 등)과의 실측 튜닝은 실제
  로봇/Gazebo 환경에서 별도로 검증이 필요하다.
- **다중 층/회전된 map 미지원**: `MapMeta`는 map이 회전되지 않았다고 가정한다
  (Cartographer 기본 출력과 동일하지만, 회전된 origin이 있는 경우는 미지원).
