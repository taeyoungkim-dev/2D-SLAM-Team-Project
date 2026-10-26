# Local Backend Server (FastAPI)

`AGENTS.md` 의 **1.2 Local Backend Server Agent** 구현체입니다.
TurtleBot3(ROS2) 로봇 에이전트와 Flutter 앱 에이전트 사이에서 **API Gateway
+ State Store** 역할을 하며, 외부 인터넷 없이 **로컬 Wi-Fi(LAN)** 안에서만
동작하는 것을 전제로 설계되었습니다.

```
[TurtleBot3 (ROS2)] <--WebSocket/HTTP--> [FastAPI Server] <--REST/WebSocket--> [Flutter App]
```

## 1. 디렉터리 구조

```
server/
├── app/
│   ├── main.py            # FastAPI 앱 생성, 라우터/미들웨어/정적 파일 등록
│   ├── config.py          # 환경변수 기반 설정 (호스트, 포트, 경로 등)
│   ├── schemas.py         # Pydantic 스키마 (AGENTS.md 3장 JSON 스키마 구현)
│   ├── database.py        # SQLite 영속 저장소 (obstacles, history)
│   ├── state_store.py     # 메모리 State Store (실시간 telemetry, FSM 상태)
│   ├── ws_manager.py      # WebSocket 연결 관리자 (robot 그룹 / app 그룹)
│   ├── static/
│   │   ├── images/        # 장애물 사진 저장 위치 (HTTP로 서빙: /images/*)
│   │   └── maps/          # 최신 지도 이미지 저장 위치 (/maps/*)
│   └── routers/
│       ├── telemetry.py   # REQ-APP-01: 실시간 위치/궤적/배터리
│       ├── obstacles.py   # REQ-ROB-02, REQ-APP-02/03: 장애물 로그 + 재청소
│       ├── commands.py    # REQ-SYS-01: 정지/복귀, 청소/매핑 시작 등 명령
│       ├── history.py     # REQ-APP-04 (P1): 청소 기록
│       ├── maps.py        # 지도 이미지 업로드/조회
│       └── ws.py          # WebSocket 엔드포인트 (/ws/robot, /ws/app)
├── scripts/
│   ├── ros2_bridge_node.py   # ROS2 topic/TF <-> 서버 브릿지 노드 (실제 로봇/시뮬레이션용)
│   └── mock_robot_client.py  # 실제 로봇 없이 서버/앱을 테스트하기 위한 모의 클라이언트
├── tests/
│   └── test_api.py        # REST API 스모크 테스트 (pytest)
├── data/                  # SQLite DB 파일 저장 위치 (app.db)
├── requirements.txt
├── .env.example
└── pytest.ini
```

## 2. 설치 및 실행

```bash
cd server
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt

cp .env.example .env             # 실제 LAN IP로 SERVER_PUBLIC_BASE_URL 수정
export $(grep -v '^#' .env | xargs)   # 또는 직접 export

uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

실행 후 브라우저에서 `http://<서버IP>:8000/docs` 로 접속하면 Swagger UI로
모든 API를 바로 테스트할 수 있습니다.

### 2.1 ROS2 브릿지 노드 실행 (`scripts/ros2_bridge_node.py`)

ROS2 topic/TF 를 서버로 올리고 서버 명령을 ROS2 로 전달하는 노드입니다.
서버를 먼저 켜고, **로봇이 움직이기 전에** 별도 터미널에서 실행합니다.
`rclpy` 가 필요하므로 서버용 `.venv` 가 아니라 ROS 를 source 한 셸의 시스템 Python 으로 실행합니다.

```bash
pip3 install --user "websockets>=12.0"   # 최초 1회 (requirements.txt 와 같은 버전)

source /opt/ros/humble/setup.bash
python3 server/scripts/ros2_bridge_node.py --ros-args \
  -p server_host:=127.0.0.1 -p server_port:=8000 -p use_sim_time:=true
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `server_host`, `server_port` | `127.0.0.1`, `8000` | 서버 주소 (서버가 다른 PC 면 LAN IP) |
| `use_sim_time` | `false` | Gazebo 시뮬레이션이면 `true` |
| `map_topic` | `/explore_map` | 지도 PNG 와 좌표 기준 frame 으로 쓰는 topic |
| `dock_x`, `dock_y`, `dock_yaw` | 시작 위치 | `return_to_dock` 목표. 생략하면 TF 첫 위치 |
| `cleaner_script`, `cleaner_args` | `robot_ws/scripts/coverage_cleaner.py`, 루트 `README.md` 의 옵션 | 앱의 **청소 시작**을 눌렀을 때 실행할 스크립트와 `-p` 옵션 |

**청소 시작 버튼**: 앱에서 누르면 브릿지가 저장소 최상위(`2D-SLAM-Team-Project/`)에서
`python3 ./robot_ws/scripts/coverage_cleaner.py --ros-args -p use_sim_time:=<브릿지 값> -p tf_timeout_sec:=1.5 ...`
를 실행합니다(`use_sim_time` 은 브릿지에 준 값을 따라갑니다). 이미 `coverage_cleaner` 노드가 떠 있으면
새로 실행하지 않고 `resume` 만 호출하며, 브릿지를 종료하면 브릿지가 실행한 청소 노드도 함께 종료됩니다.
앱이 아니라 로봇/호스트 PC 쪽 브릿지가 실행하므로 `scipy` 등 청소 노드 의존성이 그 PC 에 설치되어 있어야 합니다.

나머지 파라미터(`visited_topic`, `status_topic`, `battery_topic`, `base_frame`, `telemetry_rate_hz`,
`map_upload_period_sec`, `cleaner_node`)는 스크립트 상단 docstring 과 `declare_parameter` 를 참고하세요.

정상이면 브릿지 로그에 `서버 WebSocket 연결됨`, `지도 업로드 완료` 가 나오고
`curl http://127.0.0.1:8000/api/map/latest` 가 200 을 돌려줍니다. 404 면 브릿지가 아직 지도를 올리지 않은 것이므로
`ros2 topic info /explore_map -v` 로 구독자가 있는지 확인하세요.

## 3. 실제 로봇 없이 테스트하기 (Mock Robot)

로봇(ROS2) 쪽 브릿지 노드가 아직 준비되지 않았을 때, 아래 스크립트로
`/ws/robot` 에 가짜 로봇을 붙여 앱 개발/서버 테스트를 진행할 수 있습니다.

```bash
# 서버를 먼저 켜둔 상태에서, 새 터미널에서:
python scripts/mock_robot_client.py --host 127.0.0.1 --port 8000 --obstacle-every 20
```

- 0.5초마다 원을 그리며 이동하는 가짜 telemetry를 전송합니다.
- `--obstacle-every N` 스텝마다 가짜 장애물을 하나씩 `POST /api/obstacles` 로 등록합니다.
- 서버가 내려보내는 명령(`target_reclean`, `stop` 등)을 콘솔에 출력합니다.

## 4. 단위 테스트

```bash
cd server
source .venv/bin/activate
pytest -q
```

## 5. API 개요

FSM 상태값은 `AGENTS.md` 2장과 동일합니다:
`IDLE | MAPPING | CLEANING | EVADING | TARGET_RECLEANING | RETURNING`

| Method & Path | 설명 | 호출 주체 |
|---|---|---|
| `GET /api/health` | 서버/로봇 연결 상태 확인 | 디버깅 |
| `GET /api/telemetry` | 현재 로봇 telemetry 조회 (폴링용) | App |
| `POST /api/telemetry` | telemetry 갱신 (WS 대체용 REST 경로) | Robot |
| `POST /api/obstacles` | 장애물 감지 로그 등록 (multipart, 이미지 포함) | Robot |
| `GET /api/obstacles` | 장애물 목록 조회 (`?status=uncleared` 등 필터) | App |
| `GET /api/obstacles/{id}` | 장애물 상세 조회 | App |
| `POST /api/obstacles/{id}/resolve` | `{"action": "cleared" \| "recleaning"}` | App |
| `POST /api/commands` | `{"command": "start_cleaning" \| "start_mapping" \| "stop" \| "return_to_dock" \| "target_reclean", "target"?: {x,y}}` | App |
| `POST /api/commands/emergency_stop` | 즉시 `RETURNING` 상태로 전환 (REQ-SYS-01) | App |
| `POST /api/history` | 청소 완료 기록 저장 | Server 내부/Robot |
| `GET /api/history` | 청소 기록 목록 조회 | App |
| `POST /api/map` | 지도 PNG + 메타데이터 업로드 (multipart, 아래 5.3) | Robot |
| `GET /api/map/latest` | 최신 지도 URL + 해상도/원점/시작 위치 조회 | App |
| `WS /ws/robot` | 로봇 <-> 서버 실시간 채널 (telemetry 업로드, 명령 수신) | Robot |
| `WS /ws/app` | 서버 -> 앱 실시간 브로드캐스트 (telemetry, 장애물 알림) | App |
| `GET /images/{file}` | 장애물 사진 정적 서빙 | App |
| `GET /maps/{file}` | 지도 이미지 정적 서빙 (`map_<version>.png`, 최근 4개만 보관) | App |

### 5.1 `/ws/robot` 메시지 포맷

로봇 -> 서버 (telemetry, `AGENTS.md` 3.2 스키마와 동일):
```json
{
  "type": "telemetry",
  "current_state": "CLEANING",
  "current_position": {"x": 2.10, "y": 1.15, "theta": 0.5},
  "swept_path": [{"x": 0.0, "y": 0.0}],
  "battery_level": 85
}
```

서버 -> 로봇 (명령):
```json
{
  "type": "command",
  "command": "target_reclean",
  "target": {"x": 1.25, "y": -0.5},
  "obstacle_id": "obs_001",
  "issued_at": "2026-09-15T06:43:18.140264+00:00"
}
```

### 5.2 `/ws/app` 브로드캐스트 포맷

```json
{ "type": "telemetry", "payload": { "...RobotTelemetry..." } }
{ "type": "obstacle_alert", "payload": { "...ObstacleLog..." } }
{ "type": "obstacle_update", "payload": { "...ObstacleLog..." } }
{ "type": "map_update", "payload": { "...MapInfo (5.3)..." } }
{ "type": "robot_connected", "payload": {} }
{ "type": "robot_disconnected", "payload": {} }
```

### 5.3 지도 이미지와 좌표 기준

모든 좌표(telemetry, 장애물, `target_reclean` 목표)는 ROS `/explore_map` 의
`header.frame_id` (보통 `map`) 좌표계 기준이다. `/explore_map` 은
`map_threshold_node.py` 가 `/map` 의 header/info 를 그대로 복사하고 값만 이진화한 것이라
`/map` 과 frame·원점·해상도·크기가 같다.

`POST /api/map` (multipart) — 이미지와 메타데이터를 한 요청으로 보내 항상 한 쌍으로 저장한다.

| 필드 | 설명 |
|---|---|
| `image` | 지도 PNG (위쪽이 +y, 1픽셀 = `resolution` m) |
| `resolution`, `width`, `height` | `OccupancyGrid.info` 의 해상도(m/px)와 크기 |
| `origin_x`, `origin_y` | PNG 왼쪽 아래 모서리의 map 좌표 (`info.origin.position`) |
| `frame_id` | 기본 `map` |
| `start_x`, `start_y`, `start_theta` | 로봇이 처음 움직이기 시작한 위치(도크). 앱의 화면 기준점 |

`GET /api/map/latest` 와 `map_update` payload:

```json
{
  "frame_id": "map", "resolution": 0.05, "width": 384, "height": 512,
  "origin": {"x": -9.6, "y": -12.8},
  "start_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
  "map_image_url": "http://192.168.x.x:8000/maps/map_000012.png",
  "version": 12, "updated_at": "2026-10-27T01:20:00Z"
}
```

픽셀 변환: `px = (x - origin.x) / resolution`, `py = height - (y - origin.y) / resolution`.
지도가 자라면 `origin`/`width`/`height` 가 바뀌므로 항상 같은 응답의 값을 쓴다.

## 6. 로봇(ROS2) 쪽 연동 가이드

이 저장소의 `robot_ws` 는 ROS2 노드만 포함하므로, 실제 연동을 위해서는
별도의 "브릿지 노드"가 필요합니다. 브릿지 노드는:

1. `/odom`, `/global_costmap/costmap`, 배터리 토픽 등을 구독해
   `RobotTelemetry` 형태로 변환한 뒤 `/ws/robot` 으로 0.5~1Hz 전송.
2. YOLOv8 노드가 장애물을 감지하면 이미지 + map 좌표를
   `POST /api/obstacles` 로 전송.
3. `/ws/robot` 으로 들어오는 `command` 메시지를 받아
   `NavigateToPose` 액션 호출 등으로 변환.

`scripts/ros2_bridge_node.py` 가 이 브릿지의 구현입니다(telemetry, 지도 업로드, 명령 수신).
`scripts/mock_robot_client.py` 는 ROS 없이 1, 2번 흐름을 흉내 내는 테스트용입니다.
