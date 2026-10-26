"""API 데이터 스키마 (Pydantic).

AGENTS.md 3장(DATA_INTERFACE)에 정의된 JSON 스키마를 그대로 구현한다.
"""

from __future__ import annotations

from datetime import datetime, timezone
from enum import Enum
from typing import Literal, Optional

from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# 2. Finite State Machine
# ---------------------------------------------------------------------------
class RobotState(str, Enum):
    IDLE = "IDLE"
    MAPPING = "MAPPING"
    CLEANING = "CLEANING"
    EVADING = "EVADING"
    TARGET_RECLEANING = "TARGET_RECLEANING"
    RETURNING = "RETURNING"


# ---------------------------------------------------------------------------
# 공용 좌표/포즈
# ---------------------------------------------------------------------------
class MapCoordinate(BaseModel):
    x: float
    y: float


class Pose2D(MapCoordinate):
    theta: float = 0.0


# ---------------------------------------------------------------------------
# 지도 메타데이터 (지도 PNG를 월드 좌표와 맞추기 위한 정보)
# ---------------------------------------------------------------------------
class MapMeta(BaseModel):
    """로봇이 ``/explore_map`` (OccupancyGrid) 에서 읽어 지도 PNG와 함께 올리는 값.

    텔레메트리/장애물/목표 좌표는 모두 ``frame_id`` 좌표계(보통 ``map``) 기준이다.
    PNG는 위쪽이 +y가 되도록 상하 반전되어 있고 1픽셀 = ``resolution`` m 이다.

        px = (x - origin.x) / resolution
        py = height - (y - origin.y) / resolution
    """

    frame_id: str = "map"
    resolution: float = Field(gt=0, description="m/pixel")
    width: int = Field(gt=0, description="PNG 가로 픽셀 수")
    height: int = Field(gt=0, description="PNG 세로 픽셀 수")
    origin: MapCoordinate = Field(description="PNG 왼쪽 아래 모서리의 frame_id 좌표(m)")
    start_pose: Pose2D = Field(
        default_factory=lambda: Pose2D(x=0.0, y=0.0, theta=0.0),
        description="로봇이 처음 움직이기 시작한 위치(도크). 앱이 이 점을 화면 기준점으로 쓴다.",
    )


class MapInfo(MapMeta):
    """``GET /api/map/latest`` 응답 / ``map_update`` WebSocket payload."""

    map_image_url: str
    version: int = 1
    updated_at: datetime


# ---------------------------------------------------------------------------
# 3.1 Obstacle Log Schema
# ---------------------------------------------------------------------------
ObstacleStatus = Literal["uncleared", "cleared", "recleaning"]


class ObstacleLog(BaseModel):
    obstacle_id: str
    type: str
    map_coordinate: MapCoordinate
    image_url: str
    status: ObstacleStatus = "uncleared"
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class ObstacleCreateForm(BaseModel):
    """멀티파트 POST /api/obstacles 에서 이미지와 함께 전달되는 메타데이터."""

    obstacle_id: str
    type: str
    x: float
    y: float


class ObstacleResolveRequest(BaseModel):
    action: Literal["cleared", "recleaning"]


# ---------------------------------------------------------------------------
# 3.2 Robot Telemetry Schema
# ---------------------------------------------------------------------------
class RobotTelemetry(BaseModel):
    current_state: RobotState
    current_position: Pose2D
    swept_path: list[MapCoordinate] = Field(default_factory=list)
    battery_level: int = 100
    updated_at: Optional[datetime] = None


class TelemetryUpdate(BaseModel):
    """로봇 -> 서버로 전송되는 텔레메트리 갱신 페이로드."""

    current_state: RobotState
    current_position: Pose2D
    swept_path: list[MapCoordinate] = Field(default_factory=list)
    battery_level: int = 100


# ---------------------------------------------------------------------------
# 명령 (App -> Server -> Robot)
# ---------------------------------------------------------------------------
CommandName = Literal[
    "start_cleaning",
    "start_mapping",
    "stop",
    "return_to_dock",
    "target_reclean",
]


class CommandRequest(BaseModel):
    command: CommandName
    target: Optional[MapCoordinate] = None
    obstacle_id: Optional[str] = None


class CommandMessage(BaseModel):
    """서버 -> 로봇으로 WebSocket을 통해 전달되는 명령 메시지."""

    type: Literal["command"] = "command"
    command: CommandName
    target: Optional[MapCoordinate] = None
    obstacle_id: Optional[str] = None
    issued_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


# ---------------------------------------------------------------------------
# P1: History Logging
# ---------------------------------------------------------------------------
class HistoryRecord(BaseModel):
    id: Optional[int] = None
    map_image_url: Optional[str] = None
    duration_sec: int
    obstacles_found: int = 0
    started_at: datetime
    finished_at: datetime
    note: Optional[str] = None


class HistoryCreateRequest(BaseModel):
    map_image_url: Optional[str] = None
    duration_sec: int
    obstacles_found: int = 0
    started_at: datetime
    finished_at: datetime
    note: Optional[str] = None


# ---------------------------------------------------------------------------
# WebSocket 브로드캐스트 envelope (서버 -> 앱)
# ---------------------------------------------------------------------------
class WSEventType(str, Enum):
    TELEMETRY = "telemetry"
    OBSTACLE_ALERT = "obstacle_alert"
    OBSTACLE_UPDATE = "obstacle_update"
    MAP_UPDATE = "map_update"
    ROBOT_CONNECTED = "robot_connected"
    ROBOT_DISCONNECTED = "robot_disconnected"


class WSEvent(BaseModel):
    type: WSEventType
    payload: dict
