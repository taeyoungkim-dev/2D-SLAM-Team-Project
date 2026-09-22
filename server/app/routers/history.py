"""REQ-APP-04: History Logging (P1).

청소가 끝날 때마다(로봇이 RETURNING/IDLE로 전환되는 시점) 맵 이미지, 소요
시간, 발견된 장애물 개수를 기록하고 앱에서 리스트로 조회할 수 있게 한다.
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException

from .. import database
from ..schemas import HistoryCreateRequest, HistoryRecord

router = APIRouter(prefix="/api/history", tags=["history"])


def _row_to_history(row: dict) -> HistoryRecord:
    return HistoryRecord(
        id=row["id"],
        map_image_url=row["map_image_url"],
        duration_sec=row["duration_sec"],
        obstacles_found=row["obstacles_found"],
        started_at=row["started_at"],
        finished_at=row["finished_at"],
        note=row["note"],
    )


@router.post("", response_model=HistoryRecord)
async def create_history(body: HistoryCreateRequest) -> HistoryRecord:
    row = await database.insert_history(
        {
            "map_image_url": body.map_image_url,
            "duration_sec": body.duration_sec,
            "obstacles_found": body.obstacles_found,
            "started_at": body.started_at.isoformat(),
            "finished_at": body.finished_at.isoformat(),
            "note": body.note,
        }
    )
    return _row_to_history(row)


@router.get("", response_model=list[HistoryRecord])
async def list_history() -> list[HistoryRecord]:
    rows = await database.list_history()
    return [_row_to_history(row) for row in rows]


@router.get("/{history_id}", response_model=HistoryRecord)
async def get_history(history_id: int) -> HistoryRecord:
    row = await database.get_history(history_id)
    if row is None:
        raise HTTPException(status_code=404, detail="history record not found")
    return _row_to_history(row)
