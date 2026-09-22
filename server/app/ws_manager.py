"""WebSocket 연결 관리자.

두 그룹의 클라이언트를 관리한다.

- ``robot``: TurtleBot3(ROS2) 쪽에서 붙는 연결. 서버가 이 연결로 명령
  (start/stop/return/target_reclean 등)을 내려보낸다. 보통 1개만 존재.
- ``app``: Flutter 앱에서 붙는 연결. 서버가 telemetry/장애물 알림을
  실시간으로 브로드캐스트한다. 여러 개 존재할 수 있다.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import WebSocket

logger = logging.getLogger("server.ws")


class ConnectionManager:
    def __init__(self) -> None:
        self._robot_sockets: set[WebSocket] = set()
        self._app_sockets: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    # -- robot ---------------------------------------------------------
    async def connect_robot(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._robot_sockets.add(websocket)
        logger.info("robot connected (%d total)", len(self._robot_sockets))

    async def disconnect_robot(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._robot_sockets.discard(websocket)
        logger.info("robot disconnected (%d total)", len(self._robot_sockets))

    async def send_command_to_robot(self, message: dict[str, Any]) -> bool:
        """현재 연결된 모든 robot 소켓에 명령을 전송한다.

        Returns:
            bool: 최소 1개 이상의 로봇 소켓에 전송했는지 여부.
        """
        async with self._lock:
            targets = list(self._robot_sockets)
        sent = False
        for socket in targets:
            try:
                await socket.send_json(message)
                sent = True
            except Exception:  # noqa: BLE001 - 연결이 끊긴 소켓은 무시
                logger.warning("failed to send command to a robot socket", exc_info=True)
        return sent

    # -- app -------------------------------------------------------------
    async def connect_app(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._app_sockets.add(websocket)
        logger.info("app client connected (%d total)", len(self._app_sockets))

    async def disconnect_app(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._app_sockets.discard(websocket)
        logger.info("app client disconnected (%d total)", len(self._app_sockets))

    async def broadcast_to_apps(self, message: dict[str, Any]) -> None:
        async with self._lock:
            targets = list(self._app_sockets)
        stale: list[WebSocket] = []
        for socket in targets:
            try:
                await socket.send_json(message)
            except Exception:  # noqa: BLE001
                stale.append(socket)
        if stale:
            async with self._lock:
                for socket in stale:
                    self._app_sockets.discard(socket)

    def robot_count(self) -> int:
        return len(self._robot_sockets)

    def app_count(self) -> int:
        return len(self._app_sockets)


manager = ConnectionManager()
