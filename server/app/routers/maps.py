"""맵 이미지 + 메타데이터 업로드/조회.

로봇 쪽 브릿지(``scripts/ros2_bridge_node.py``)가 ``/explore_map`` 을 PNG로 렌더링해
해상도/크기/원점/시작 위치와 **같은 요청**으로 올린다. 이미지와 메타데이터가 서로 다른
시점의 값이 되면 앱에서 핀이 어긋나므로 항상 한 쌍으로 저장한다.

저장 위치 (``settings.MAPS_DIR``)
    map_000012.png   업로드 version 별 지도 이미지 (한 번 쓰면 덮어쓰지 않는다)
    latest_map.json  최신 이미지의 MapInfo (해상도, 원점, 시작 위치, version ...)

이미지를 항상 새 파일 이름으로 저장하는 이유: 같은 이름(latest_map.png)을 교체하면, 앱이
그 파일을 내려받는 도중에 교체가 일어났을 때 StaticFiles 가 stat 한 크기와 실제로 읽은 파일
크기가 달라져 "Response content longer than Content-Length" 로 응답이 끊기고, 앱에서 지도가
잠시 사라졌다. 오래된 이미지는 최근 몇 개만 남기고 지운다 (내려받는 중인 클라이언트 보호).

앱은 ``GET /api/map/latest`` 로 현재 값을 읽고, 이후에는 WebSocket ``map_update``
이벤트(payload = MapInfo)로 갱신 알림을 받는다 (REQ-APP-01, REQ-APP-04).
"""

from __future__ import annotations

import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, File, Form, HTTPException, UploadFile

from ..config import settings
from ..schemas import MapCoordinate, MapInfo, MapMeta, Pose2D, WSEvent, WSEventType
from ..ws_manager import manager

router = APIRouter(prefix="/api/map", tags=["map"])

_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}
_META_NAME = "latest_map.json"
_VERSIONED_RE = re.compile(r"^map_(\d+)\.(png|jpg|jpeg)$", re.IGNORECASE)
_KEEP_OLD_IMAGES = 3  # 최신 이미지 외에 남겨둘 이전 버전 수


def _versioned_images() -> list[tuple[int, Path]]:
    """``map_<version>.<ext>`` 이미지를 version 오름차순으로."""
    found = []
    for path in settings.MAPS_DIR.iterdir():
        match = _VERSIONED_RE.match(path.name)
        if match:
            found.append((int(match.group(1)), path))
    return sorted(found)


def _latest_image() -> Optional[Path]:
    versioned = _versioned_images()
    if versioned:
        return versioned[-1][1]
    # 버전 이름을 쓰기 전에 올라온 latest_map.* 도 읽을 수 있게 한다.
    for path in sorted(settings.MAPS_DIR.glob("latest_map.*")):
        if path.suffix.lower() in _IMAGE_SUFFIXES:
            return path
    return None


def _read_meta() -> Optional[MapInfo]:
    meta_path = settings.MAPS_DIR / _META_NAME
    if not meta_path.exists():
        return None
    try:
        return MapInfo.model_validate_json(meta_path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def _atomic_write(dest: Path, data: bytes) -> None:
    """읽는 쪽이 쓰다 만 파일을 보지 않도록 임시 파일에 쓴 뒤 교체한다."""
    tmp = dest.with_name(dest.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, dest)


@router.post("", response_model=MapInfo)
async def upload_map(
    image: UploadFile = File(...),
    resolution: float = Form(..., gt=0, description="m/pixel (OccupancyGrid.info.resolution)"),
    width: int = Form(..., gt=0),
    height: int = Form(..., gt=0),
    origin_x: float = Form(..., description="OccupancyGrid.info.origin.position.x"),
    origin_y: float = Form(..., description="OccupancyGrid.info.origin.position.y"),
    frame_id: str = Form("map"),
    start_x: float = Form(0.0),
    start_y: float = Form(0.0),
    start_theta: float = Form(0.0),
) -> MapInfo:
    settings.ensure_directories()

    suffix = Path(image.filename or "").suffix.lower()
    if suffix not in _IMAGE_SUFFIXES:
        suffix = ".png"

    previous = _read_meta()
    existing = _versioned_images()
    version = max(previous.version if previous else 0, existing[-1][0] if existing else 0) + 1
    dest_path = settings.MAPS_DIR / f"map_{version:06d}{suffix}"
    _atomic_write(dest_path, await image.read())

    meta = MapMeta(
        frame_id=frame_id,
        resolution=resolution,
        width=width,
        height=height,
        origin=MapCoordinate(x=origin_x, y=origin_y),
        start_pose=Pose2D(x=start_x, y=start_y, theta=start_theta),
    )
    info = MapInfo(
        **meta.model_dump(),
        map_image_url=f"{settings.PUBLIC_BASE_URL}/maps/{dest_path.name}",
        version=version,
        updated_at=datetime.now(timezone.utc),
    )
    _atomic_write(settings.MAPS_DIR / _META_NAME, info.model_dump_json().encode("utf-8"))

    # 최신 이미지 + 직전 몇 개만 남기고 정리한다.
    for _, old in _versioned_images()[: -(_KEEP_OLD_IMAGES + 1)]:
        old.unlink(missing_ok=True)

    await manager.broadcast_to_apps(
        WSEvent(type=WSEventType.MAP_UPDATE, payload=info.model_dump(mode="json")).model_dump(mode="json")
    )
    return info


@router.get("/latest")
async def get_latest_map() -> dict:
    """최신 지도 URL과 메타데이터. 메타데이터 없이 올라온 옛 지도는 URL만 돌려준다."""
    settings.ensure_directories()
    image_path = _latest_image()
    if image_path is None:
        raise HTTPException(status_code=404, detail="no map uploaded yet")
    info = _read_meta()
    if info is None:
        return {"map_image_url": f"{settings.PUBLIC_BASE_URL}/maps/{image_path.name}"}
    return info.model_dump(mode="json")
