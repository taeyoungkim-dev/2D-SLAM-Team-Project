"""WebSocket 엔드포인트.

- ``/ws/robot``: 로봇(ROS2 브릿지 노드)이 연결한다. 로봇 -> 서버로
  telemetry JSON을 계속 전송하고, 서버 -> 로봇으로는 명령
  (:mod:`..ws_manager` 를 통해) 을 내려보낸다.
- ``/ws/app``: Flutter 앱이 연결한다. 서버는 telemetry 갱신 / 장애물
  알림을 실시간으로 브로드캐스트한다 (REQ-APP-01, REQ-APP-02).
"""

from __future__ import annotations

import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..schemas import TelemetryUpdate, WSEvent, WSEventType
from ..state_store import state_store
from ..ws_manager import manager

logger = logging.getLogger("server.ws.router")

router = APIRouter(tags=["websocket"])


@router.websocket("/ws/robot")
async def robot_ws(websocket: WebSocket) -> None:
    await manager.connect_robot(websocket)
    await state_store.set_robot_connected(True)
    await manager.broadcast_to_apps(
        WSEvent(type=WSEventType.ROBOT_CONNECTED, payload={}).model_dump(mode="json")
    )
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type", "telemetry")
            if msg_type == "telemetry":
                try:
                    update = TelemetryUpdate.model_validate(data)
                except Exception:  # noqa: BLE001
                    logger.warning("invalid telemetry payload from robot: %s", data)
                    continue
                telemetry = await state_store.update_telemetry(
                    current_state=update.current_state,
                    current_position=update.current_position,
                    swept_path=update.swept_path,
                    battery_level=update.battery_level,
                )
                await manager.broadcast_to_apps(
                    WSEvent(
                        type=WSEventType.TELEMETRY,
                        payload=telemetry.model_dump(mode="json"),
                    ).model_dump(mode="json")
                )
            else:
                logger.info("unhandled message type from robot: %s", msg_type)
    except WebSocketDisconnect:
        pass
    finally:
        await manager.disconnect_robot(websocket)
        if manager.robot_count() == 0:
            await state_store.set_robot_connected(False)
            await manager.broadcast_to_apps(
                WSEvent(type=WSEventType.ROBOT_DISCONNECTED, payload={}).model_dump(
                    mode="json"
                )
            )


@router.websocket("/ws/app")
async def app_ws(websocket: WebSocket) -> None:
    await manager.connect_app(websocket)
    # 접속 즉시 최신 상태 스냅샷을 보내 화면을 바로 채울 수 있게 한다.
    telemetry = await state_store.get_telemetry()
    await websocket.send_json(
        WSEvent(type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json")).model_dump(
            mode="json"
        )
    )
    try:
        while True:
            # 앱 -> 서버 방향으로는 별도 명령이 없다 (REST로 처리).
            # ping/keep-alive 목적의 메시지만 수신하고 무시한다.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await manager.disconnect_app(websocket)
