"""서버 전역 설정.

로컬 Wi-Fi망(LAN) 안에서만 동작하는 것을 전제로 하므로 외부 서비스(예: Firebase) 연동은
하지 않는다. 모든 값은 환경변수로 오버라이드할 수 있게 하여, 로봇/서버/앱이 서로 다른
호스트에서 실행되어도 IP만 바꿔주면 되도록 구성한다.
"""

from __future__ import annotations

import os
from pathlib import Path


class Settings:
    """환경변수 기반 설정 값 모음 (AGENTS.md 3장의 스키마를 그대로 따름)."""

    # --- 서버 네트워크 설정 ---
    HOST: str = os.getenv("SERVER_HOST", "0.0.0.0")
    PORT: int = int(os.getenv("SERVER_PORT", "8000"))

    # 앱/로봇이 이미지 URL을 만들 때 사용할 서버의 LAN IP.
    # 예: 192.168.0.10 (데모 환경에 맞게 .env 또는 환경변수로 지정)
    PUBLIC_BASE_URL: str = os.getenv("SERVER_PUBLIC_BASE_URL", "http://192.168.0.10:8000")

    # --- 경로 설정 ---
    BASE_DIR: Path = Path(__file__).resolve().parent
    STATIC_DIR: Path = BASE_DIR / "static"
    IMAGES_DIR: Path = STATIC_DIR / "images"
    MAPS_DIR: Path = STATIC_DIR / "maps"

    DATA_DIR: Path = BASE_DIR.parent / "data"
    DB_PATH: Path = Path(os.getenv("SERVER_DB_PATH", str(DATA_DIR / "app.db")))

    # --- 도메인 설정 ---
    # 로봇이 일정 시간 이상 telemetry를 보내지 않으면 연결이 끊긴 것으로 간주.
    ROBOT_TIMEOUT_SEC: float = float(os.getenv("ROBOT_TIMEOUT_SEC", "5.0"))

    # CORS: 데모 환경에서는 로컬망 내 임의의 클라이언트(앱, 브라우저 테스트 등)를 허용.
    CORS_ALLOW_ORIGINS: list[str] = ["*"]

    def ensure_directories(self) -> None:
        for path in (self.STATIC_DIR, self.IMAGES_DIR, self.MAPS_DIR, self.DATA_DIR):
            path.mkdir(parents=True, exist_ok=True)


settings = Settings()
