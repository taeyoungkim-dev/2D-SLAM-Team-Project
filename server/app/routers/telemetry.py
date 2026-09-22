"""REQ-APP-01: Real-time Telemetry.

로봇의 실시간 위치/청소 궤적/배터리 상태를 다룬다. WebSocket이 주 전송
경로이지만, 네트워크 사정에 따라 REST Polling(GET) / Push(POST) 도 함께
지원한다 (AGENTS.md: "서버에서 맵/상태 폴링").
"""

from __future__ import annotations

from fastapi import APIRouter

from ..schemas import RobotTelemetry, TelemetryUpdate, WSEvent, WSEventType
from ..state_store import state_store
from ..ws_manager import manager

router = APIRouter(prefix="/api/telemetry", tags=["telemetry"])


@router.get("", response_model=RobotTelemetry)
async def get_telemetry() -> RobotTelemetry:
    """앱이 폴링 방식으로 현재 로봇 상태를 가져올 때 사용."""
    return await state_store.get_telemetry()


@router.post("", response_model=RobotTelemetry)
async def push_telemetry(update: TelemetryUpdate) -> RobotTelemetry:
    """로봇이 WebSocket 대신 REST로 상태를 밀어넣을 때 사용 (백업 경로).

    갱신 후 연결된 모든 앱 클라이언트에게 실시간으로 브로드캐스트한다.
    """
    telemetry = await state_store.update_telemetry(
        current_state=update.current_state,
        current_position=update.current_position,
        swept_path=update.swept_path,
        battery_level=update.battery_level,
    )
    event = WSEvent(type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json"))
    await manager.broadcast_to_apps(event.model_dump(mode="json"))
    return telemetry
