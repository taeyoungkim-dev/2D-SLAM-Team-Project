"""맵 이미지 업로드/조회.

Cartographer/RTAB-Map 이 저장한 최신 2D 지도 이미지를 로봇(또는 로봇 쪽
브릿지 스크립트)이 서버로 업로드하고, 앱은 이를 HTTP로 내려받아 배경으로
깔고 그 위에 실시간 위치/궤적/장애물 핀을 그린다 (REQ-APP-01, REQ-APP-04).
"""

from __future__ import annotations

import shutil
from pathlib import Path

from fastapi import APIRouter, File, HTTPException, UploadFile

from ..config import settings

router = APIRouter(prefix="/api/map", tags=["map"])

_LATEST_MAP_NAME = "latest_map.png"


@router.post("")
async def upload_map(image: UploadFile = File(...)) -> dict:
    settings.ensure_directories()
    suffix = Path(image.filename or "").suffix or ".png"
    dest_path = settings.MAPS_DIR / f"latest_map{suffix}"
    with dest_path.open("wb") as buffer:
        shutil.copyfileobj(image.file, buffer)
    image_url = f"{settings.PUBLIC_BASE_URL}/maps/{dest_path.name}"
    return {"map_image_url": image_url}


@router.get("/latest")
async def get_latest_map() -> dict:
    settings.ensure_directories()
    candidates = sorted(settings.MAPS_DIR.glob("latest_map.*"))
    if not candidates:
        raise HTTPException(status_code=404, detail="no map uploaded yet")
    image_url = f"{settings.PUBLIC_BASE_URL}/maps/{candidates[0].name}"
    return {"map_image_url": image_url}
