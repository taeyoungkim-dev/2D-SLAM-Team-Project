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
    ROBOT_CONNECTED = "robot_connected"
    ROBOT_DISCONNECTED = "robot_disconnected"


class WSEvent(BaseModel):
    type: WSEventType
    payload: dict
