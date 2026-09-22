"""서버가 들고 있는 '현재 상태'에 대한 단일 진실 공급원(Single Source of Truth).

로봇 <-> 서버 <-> 앱 사이에서 오가는 최신 Telemetry 를 메모리에 보관한다.
DB(SQLite)는 영속 데이터(장애물 로그, 히스토리)를 다루고, 이 모듈은
자주 갱신되는 실시간 상태(로봇 위치, FSM 상태, 배터리 등)를 다룬다.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from typing import Optional

from .schemas import MapCoordinate, Pose2D, RobotState, RobotTelemetry


class StateStore:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._telemetry = RobotTelemetry(
            current_state=RobotState.IDLE,
            current_position=Pose2D(x=0.0, y=0.0, theta=0.0),
            swept_path=[],
            battery_level=100,
            updated_at=datetime.now(timezone.utc),
        )
        self._robot_connected = False
        self._robot_last_seen: Optional[datetime] = None

    async def get_telemetry(self) -> RobotTelemetry:
        async with self._lock:
            return self._telemetry.model_copy(deep=True)

    async def update_telemetry(
        self,
        current_state: RobotState,
        current_position: Pose2D,
        swept_path: list[MapCoordinate],
        battery_level: int,
    ) -> RobotTelemetry:
        async with self._lock:
            self._telemetry = RobotTelemetry(
                current_state=current_state,
                current_position=current_position,
                swept_path=swept_path,
                battery_level=battery_level,
                updated_at=datetime.now(timezone.utc),
            )
            self._robot_last_seen = self._telemetry.updated_at
            return self._telemetry.model_copy(deep=True)

    async def set_state(self, state: RobotState) -> RobotTelemetry:
        """DB 갱신 없이 FSM 상태만 서버에서 즉시 바꿔야 할 때 사용
        (예: 앱의 정지/복귀 명령, 장애물 감지에 따른 EVADING 진입)."""
        async with self._lock:
            current = self._telemetry
            self._telemetry = current.model_copy(update={"current_state": state, "updated_at": datetime.now(timezone.utc)})
            return self._telemetry.model_copy(deep=True)

    async def set_robot_connected(self, connected: bool) -> None:
        async with self._lock:
            self._robot_connected = connected
            if connected:
                self._robot_last_seen = datetime.now(timezone.utc)

    async def is_robot_connected(self) -> bool:
        async with self._lock:
            return self._robot_connected


state_store = StateStore()
