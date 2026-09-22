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
| `POST /api/map` | 최신 지도 이미지 업로드 (multipart) | Robot |
| `GET /api/map/latest` | 최신 지도 이미지 URL 조회 | App |
| `WS /ws/robot` | 로봇 <-> 서버 실시간 채널 (telemetry 업로드, 명령 수신) | Robot |
| `WS /ws/app` | 서버 -> 앱 실시간 브로드캐스트 (telemetry, 장애물 알림) | App |
| `GET /images/{file}` | 장애물 사진 정적 서빙 | App |
| `GET /maps/{file}` | 지도 이미지 정적 서빙 | App |

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
{ "type": "robot_connected", "payload": {} }
{ "type": "robot_disconnected", "payload": {} }
```

## 6. 로봇(ROS2) 쪽 연동 가이드

이 저장소의 `robot_ws` 는 ROS2 노드만 포함하므로, 실제 연동을 위해서는
별도의 "브릿지 노드"가 필요합니다. 브릿지 노드는:

1. `/odom`, `/global_costmap/costmap`, 배터리 토픽 등을 구독해
   `RobotTelemetry` 형태로 변환한 뒤 `/ws/robot` 으로 0.5~1Hz 전송.
2. YOLOv8 노드가 장애물을 감지하면 이미지 + map 좌표를
   `POST /api/obstacles` 로 전송.
3. `/ws/robot` 으로 들어오는 `command` 메시지를 받아
   `NavigateToPose` 액션 호출 등으로 변환.

`scripts/mock_robot_client.py` 가 위 1, 2번 흐름의 참고 예시입니다.
