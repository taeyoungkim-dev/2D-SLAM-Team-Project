"""FastAPI 진입점.

Local Backend Server Agent (AGENTS.md 1.2) 구현체.
Robot Agent(ROS2) <-> Server <-> Mobile App Agent(Flutter) 사이의
API Gateway 및 State Store 역할을 한다.

실행:
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
"""

from __future__ import annotations

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .config import settings
from .database import init_db
from .routers import commands, history, maps, obstacles, telemetry, ws


@asynccontextmanager
async def lifespan(app: FastAPI):
    settings.ensure_directories()
    await init_db()
    yield


app = FastAPI(
    title="2D SLAM Robot Vacuum - Local Server",
    description=(
        "TurtleBot3 로봇청소기와 Flutter 앱 사이의 API Gateway / State Store. "
        "외부 인터넷 없이 로컬 Wi-Fi(LAN)에서만 동작하는 것을 전제로 한다."
    ),
    version="0.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ALLOW_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 이미지/지도 정적 파일 서빙 (AGENTS.md: "HTTP (Image serving)")
settings.ensure_directories()
app.mount("/images", StaticFiles(directory=settings.IMAGES_DIR), name="images")
app.mount("/maps", StaticFiles(directory=settings.MAPS_DIR), name="maps")

app.include_router(telemetry.router)
app.include_router(obstacles.router)
app.include_router(commands.router)
app.include_router(history.router)
app.include_router(maps.router)
app.include_router(ws.router)


@app.get("/", tags=["health"])
async def root() -> dict:
    return {"service": "2d-slam-robot-vacuum-server", "status": "ok"}


@app.get("/api/health", tags=["health"])
async def health() -> dict:
    from .state_store import state_store
    from .ws_manager import manager

    return {
        "status": "ok",
        "robot_connected": await state_store.is_robot_connected(),
        "robot_sockets": manager.robot_count(),
        "app_sockets": manager.app_count(),
    }
