"""SQLite 기반 영속 저장소.

- obstacles: 장애물 로그 (REQ-ROB-02 / REQ-APP-03). ``active=0`` 은 이전 청소의 기록(앱 지도에서는 내려가지만
  시간/class/사진은 남는다).
- history: 청소 완료 기록 (REQ-APP-04, P1)

데모 규모의 프로젝트이므로 무거운 ORM 대신 `aiosqlite`를 사용해 FastAPI의
비동기 이벤트 루프를 막지 않으면서도 단순한 구조를 유지한다.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Any, AsyncIterator, Optional

import aiosqlite

from .config import settings

_SCHEMA = """
CREATE TABLE IF NOT EXISTS obstacles (
    obstacle_id TEXT PRIMARY KEY,
    type TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    image_url TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'uncleared',
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL,
    active INTEGER NOT NULL DEFAULT 1
);

CREATE TABLE IF NOT EXISTS history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_image_url TEXT,
    duration_sec INTEGER NOT NULL,
    obstacles_found INTEGER NOT NULL DEFAULT 0,
    started_at TEXT NOT NULL,
    finished_at TEXT NOT NULL,
    note TEXT
);
"""


async def init_db() -> None:
    settings.ensure_directories()
    async with aiosqlite.connect(settings.DB_PATH) as db:
        await db.executescript(_SCHEMA)
        # 예전 DB(active 컬럼 없음) 마이그레이션
        cursor = await db.execute("PRAGMA table_info(obstacles)")
        columns = {row[1] for row in await cursor.fetchall()}
        if "active" not in columns:
            await db.execute("ALTER TABLE obstacles ADD COLUMN active INTEGER NOT NULL DEFAULT 1")
        await db.commit()


@asynccontextmanager
async def get_db() -> AsyncIterator[aiosqlite.Connection]:
    db = await aiosqlite.connect(settings.DB_PATH)
    db.row_factory = aiosqlite.Row
    try:
        yield db
    finally:
        await db.close()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# Obstacles
# ---------------------------------------------------------------------------
async def insert_obstacle(
    obstacle_id: str, type_: str, x: float, y: float, image_url: str
) -> dict[str, Any]:
    now = _now()
    async with get_db() as db:
        await db.execute(
            """
            INSERT INTO obstacles (obstacle_id, type, x, y, image_url, status, created_at, updated_at, active)
            VALUES (?, ?, ?, ?, ?, 'uncleared', ?, ?, 1)
            ON CONFLICT(obstacle_id) DO UPDATE SET
                type=excluded.type, x=excluded.x, y=excluded.y,
                image_url=excluded.image_url, updated_at=excluded.updated_at, active=1
            """,
            (obstacle_id, type_, x, y, image_url, now, now),
        )
        await db.commit()
    return await get_obstacle(obstacle_id)  # type: ignore[return-value]


async def get_obstacle(obstacle_id: str) -> Optional[dict[str, Any]]:
    async with get_db() as db:
        cursor = await db.execute(
            "SELECT * FROM obstacles WHERE obstacle_id = ?", (obstacle_id,)
        )
        row = await cursor.fetchone()
        return dict(row) if row else None


async def find_active_obstacle_by_type(type_: str) -> Optional[dict[str, Any]]:
    """이번 청소에서 이미 발견해 표시 중인 같은 class 의 장애물(상태 무관)."""
    async with get_db() as db:
        cursor = await db.execute(
            "SELECT * FROM obstacles WHERE type = ? AND active = 1 ORDER BY created_at DESC LIMIT 1",
            (type_,),
        )
        row = await cursor.fetchone()
        return dict(row) if row else None


async def deactivate_all_obstacles() -> int:
    """새 청소 시작: 지도 표시를 모두 내린다(시간/class/사진 기록은 DB 와 이미지 폴더에 남는다)."""
    async with get_db() as db:
        cursor = await db.execute("UPDATE obstacles SET active = 0 WHERE active = 1")
        await db.commit()
        return cursor.rowcount


async def list_obstacles(
    status: Optional[str] = None, include_archived: bool = False
) -> list[dict[str, Any]]:
    where, args = [], []
    if not include_archived:
        where.append("active = 1")
    if status:
        where.append("status = ?")
        args.append(status)
    clause = f" WHERE {' AND '.join(where)}" if where else ""
    async with get_db() as db:
        cursor = await db.execute(
            f"SELECT * FROM obstacles{clause} ORDER BY created_at DESC", args
        )
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]


async def update_obstacle_status(obstacle_id: str, status: str) -> Optional[dict[str, Any]]:
    async with get_db() as db:
        await db.execute(
            "UPDATE obstacles SET status = ?, updated_at = ? WHERE obstacle_id = ?",
            (status, _now(), obstacle_id),
        )
        await db.commit()
    return await get_obstacle(obstacle_id)


async def count_uncleared_obstacles() -> int:
    async with get_db() as db:
        cursor = await db.execute(
            "SELECT COUNT(*) as c FROM obstacles WHERE status != 'cleared' AND active = 1"
        )
        row = await cursor.fetchone()
        return int(row["c"]) if row else 0


# ---------------------------------------------------------------------------
# History
# ---------------------------------------------------------------------------
async def insert_history(record: dict[str, Any]) -> dict[str, Any]:
    async with get_db() as db:
        cursor = await db.execute(
            """
            INSERT INTO history (map_image_url, duration_sec, obstacles_found, started_at, finished_at, note)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                record.get("map_image_url"),
                record["duration_sec"],
                record.get("obstacles_found", 0),
                record["started_at"],
                record["finished_at"],
                record.get("note"),
            ),
        )
        await db.commit()
        new_id = cursor.lastrowid
    return await get_history(new_id)  # type: ignore[return-value]


async def get_history(history_id: int) -> Optional[dict[str, Any]]:
    async with get_db() as db:
        cursor = await db.execute("SELECT * FROM history WHERE id = ?", (history_id,))
        row = await cursor.fetchone()
        return dict(row) if row else None


async def list_history() -> list[dict[str, Any]]:
    async with get_db() as db:
        cursor = await db.execute("SELECT * FROM history ORDER BY finished_at DESC")
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]
