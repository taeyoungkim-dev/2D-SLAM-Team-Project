"""REQ-ROB-02 / REQ-APP-02 / REQ-APP-03: 장애물 로깅 및 인터랙티브 재청소.

흐름:
1. 로봇(YOLOv8)이 장애물을 감지하면 이미지와 좌표를 ``POST /api/obstacles``
   로 전송한다. 서버는 이미지를 저장하고 ``image_url`` 을 만들어 DB에 기록,
   FSM을 ``EVADING`` 으로 전환하고 앱에 실시간 알림을 브로드캐스트한다.
2. 앱은 지도 위 '장애물 핀'을 클릭해 사진을 확인하고 [치웠음] 또는
   [재청소] 버튼을 누른다 -> ``POST /api/obstacles/{id}/resolve``.
   - ``cleared``: 상태만 종료 처리.
   - ``recleaning``: 로봇에게 해당 좌표로 이동하라는 명령을 WebSocket으로
     전달하고 FSM을 ``TARGET_RECLEANING`` 으로 전환한다.
"""

from __future__ import annotations

import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, File, Form, HTTPException, UploadFile

from .. import database
from ..config import settings
from ..schemas import (
    CommandMessage,
    MapCoordinate,
    ObstacleLog,
    ObstacleResolveRequest,
    RobotState,
    WSEvent,
    WSEventType,
)
from ..state_store import state_store
from ..ws_manager import manager

router = APIRouter(prefix="/api/obstacles", tags=["obstacles"])


def _row_to_obstacle_log(row: dict) -> ObstacleLog:
    return ObstacleLog(
        obstacle_id=row["obstacle_id"],
        type=row["type"],
        map_coordinate=MapCoordinate(x=row["x"], y=row["y"]),
        image_url=row["image_url"],
        status=row["status"],
        created_at=row["created_at"],
        updated_at=row["updated_at"],
    )


@router.post("", response_model=ObstacleLog)
async def create_obstacle(
    obstacle_id: str = Form(...),
    type: str = Form(...),  # noqa: A002 - AGENTS.md 스키마의 필드명을 그대로 사용
    x: float = Form(...),
    y: float = Form(...),
    image: Optional[UploadFile] = File(None),
) -> ObstacleLog:
    """로봇 -> 서버: 장애물 감지 로그 등록 (이미지 멀티파트 업로드)."""

    settings.ensure_directories()

    if image is not None:
        suffix = Path(image.filename or "").suffix or ".jpg"
        dest_path = settings.IMAGES_DIR / f"{obstacle_id}{suffix}"
        with dest_path.open("wb") as buffer:
            shutil.copyfileobj(image.file, buffer)
        image_url = f"{settings.PUBLIC_BASE_URL}/images/{dest_path.name}"
    else:
        # 이미지 없이도 좌표/타입만으로 로그를 남길 수 있게 허용 (테스트/시뮬레이션 용)
        image_url = f"{settings.PUBLIC_BASE_URL}/images/placeholder.jpg"

    row = await database.insert_obstacle(obstacle_id, type, x, y, image_url)
    obstacle = _row_to_obstacle_log(row)

    # 로봇이 장애물을 회피 중임을 FSM에 반영
    telemetry = await state_store.set_state(RobotState.EVADING)

    # REQ-APP-02: 앱에 실시간 알림 (장애물 알림 + 최신 상태)
    await manager.broadcast_to_apps(
        WSEvent(
            type=WSEventType.OBSTACLE_ALERT, payload=obstacle.model_dump(mode="json")
        ).model_dump(mode="json")
    )
    await manager.broadcast_to_apps(
        WSEvent(
            type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json")
        ).model_dump(mode="json")
    )

    return obstacle


@router.get("", response_model=list[ObstacleLog])
async def list_obstacles(status: Optional[str] = None) -> list[ObstacleLog]:
    rows = await database.list_obstacles(status=status)
    return [_row_to_obstacle_log(row) for row in rows]


@router.get("/{obstacle_id}", response_model=ObstacleLog)
async def get_obstacle(obstacle_id: str) -> ObstacleLog:
    row = await database.get_obstacle(obstacle_id)
    if row is None:
        raise HTTPException(status_code=404, detail="obstacle not found")
    return _row_to_obstacle_log(row)


@router.post("/{obstacle_id}/resolve", response_model=ObstacleLog)
async def resolve_obstacle(obstacle_id: str, body: ObstacleResolveRequest) -> ObstacleLog:
    """앱 -> 서버: 사용자가 [치웠음] 또는 [재청소]를 선택했을 때 호출."""

    row = await database.get_obstacle(obstacle_id)
    if row is None:
        raise HTTPException(status_code=404, detail="obstacle not found")

    if body.action == "cleared":
        row = await database.update_obstacle_status(obstacle_id, "cleared")
        # 남아있는 미청소 장애물이 없다면 CLEANING으로 복귀시켜준다.
        remaining = await database.count_uncleared_obstacles()
        next_state = RobotState.CLEANING if remaining == 0 else RobotState.EVADING
        telemetry = await state_store.set_state(next_state)
        await manager.broadcast_to_apps(
            WSEvent(
                type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json")
            ).model_dump(mode="json")
        )
    else:  # "recleaning"
        row = await database.update_obstacle_status(obstacle_id, "recleaning")
        command = CommandMessage(
            command="target_reclean",
            target=MapCoordinate(x=row["x"], y=row["y"]),
            obstacle_id=obstacle_id,
            issued_at=datetime.now(timezone.utc),
        )
        await manager.send_command_to_robot(command.model_dump(mode="json"))
        telemetry = await state_store.set_state(RobotState.TARGET_RECLEANING)
        await manager.broadcast_to_apps(
            WSEvent(
                type=WSEventType.TELEMETRY, payload=telemetry.model_dump(mode="json")
            ).model_dump(mode="json")
        )

    obstacle = _row_to_obstacle_log(row)  # type: ignore[arg-type]
    await manager.broadcast_to_apps(
        WSEvent(
            type=WSEventType.OBSTACLE_UPDATE, payload=obstacle.model_dump(mode="json")
        ).model_dump(mode="json")
    )
    return obstacle
