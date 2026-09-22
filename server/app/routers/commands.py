"""REQ-SYS-01 / REQ-APP-03: 앱에서 로봇으로 내려가는 명령.

앱은 로봇과 직접 통신하지 않는다. 항상 서버를 거쳐 WebSocket(``/ws/robot``)
으로 명령이 전달된다. 서버는 명령을 받는 즉시 FSM 상태도 선제적으로
갱신하여 앱이 지연 없이 상태 변화를 볼 수 있게 한다.
"""

from __future__ import annotations

from datetime import datetime, timezone

from fastapi import APIRouter, HTTPException

from ..schemas import CommandMessage, CommandRequest, RobotState, WSEvent, WSEventType
from ..state_store import state_store
from ..ws_manager import manager

router = APIRouter(prefix="/api/commands", tags=["commands"])

# 명령 -> 서버가 즉시 반영할 FSM 상태 매핑
_COMMAND_TO_STATE = {
    "start_cleaning": RobotState.CLEANING,
    "start_mapping": RobotState.MAPPING,
    "stop": RobotState.IDLE,
    "return_to_dock": RobotState.RETURNING,
    "target_reclean": RobotState.TARGET_RECLEANING,
}


async def _dispatch(request: CommandRequest) -> dict:
    message = CommandMessage(
        command=request.command,
        target=request.target,
        obstacle_id=request.obstacle_id,
        issued_at=datetime.now(timezone.utc),
    )
    sent_to_robot = await manager.send_command_to_robot(message.model_dump(mode="json"))

    next_state = _COMMAND_TO_STATE.get(request.command)
    telemetry = None
    if next_state is not None:
        telemetry = await state_store.set_state(next_state)
        await manager.broadcast_to_apps(
            WSEvent(
                type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json")
            ).model_dump(mode="json")
        )

    return {
        "command": request.command,
        "delivered_to_robot": sent_to_robot,
        "current_state": telemetry.current_state if telemetry else None,
    }


@router.post("")
async def send_command(request: CommandRequest) -> dict:
    """범용 명령 엔드포인트.

    ``command`` 값: start_cleaning | start_mapping | stop | return_to_dock | target_reclean
    """
    if request.command == "target_reclean" and request.target is None:
        raise HTTPException(status_code=422, detail="target_reclean requires 'target' coordinate")
    return await _dispatch(request)


@router.post("/emergency_stop")
async def emergency_stop() -> dict:
    """REQ-SYS-01: 앱의 '정지/복귀' 버튼. 즉시 RETURNING 상태로 전환."""
    return await _dispatch(CommandRequest(command="return_to_dock"))
