"""핵심 REST API에 대한 스모크 테스트.

실행: (server/ 디렉토리에서)
    pytest -q
"""

from __future__ import annotations

import os
import tempfile

import pytest
from fastapi.testclient import TestClient


@pytest.fixture()
def client():
    tmpdir = tempfile.mkdtemp()
    os.environ["SERVER_DB_PATH"] = os.path.join(tmpdir, "test.db")

    from app.config import settings
    from app.main import app  # 환경변수 설정 후 import 해야 DB 경로가 반영됨

    # 앱 모듈은 한 번만 import 되므로, 테스트마다 새 DB 를 쓰도록 경로를 직접 바꾼다.
    settings.DB_PATH = type(settings.DB_PATH)(tmpdir) / "test.db"

    with TestClient(app) as test_client:
        yield test_client


def test_health(client: TestClient) -> None:
    resp = client.get("/api/health")
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "ok"
    assert body["robot_connected"] is False


def test_telemetry_get_default(client: TestClient) -> None:
    resp = client.get("/api/telemetry")
    assert resp.status_code == 200
    body = resp.json()
    assert body["current_state"] == "IDLE"
    assert body["battery_level"] == 100


def test_telemetry_push_and_get(client: TestClient) -> None:
    payload = {
        "current_state": "CLEANING",
        "current_position": {"x": 1.0, "y": 2.0, "theta": 0.5},
        "swept_path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
        "battery_level": 87,
    }
    resp = client.post("/api/telemetry", json=payload)
    assert resp.status_code == 200
    assert resp.json()["current_state"] == "CLEANING"

    resp2 = client.get("/api/telemetry")
    assert resp2.json()["battery_level"] == 87


def test_obstacle_lifecycle(client: TestClient) -> None:
    create_resp = client.post(
        "/api/obstacles",
        data={"obstacle_id": "obs_001", "type": "towel", "x": 1.25, "y": -0.5},
    )
    assert create_resp.status_code == 200
    obstacle = create_resp.json()
    assert obstacle["status"] == "uncleared"
    assert obstacle["image_url"].endswith("/images/placeholder.jpg")

    # FSM should now be EVADING
    telemetry = client.get("/api/telemetry").json()
    assert telemetry["current_state"] == "EVADING"

    list_resp = client.get("/api/obstacles")
    assert len(list_resp.json()) == 1

    resolve_resp = client.post(
        "/api/obstacles/obs_001/resolve", json={"action": "recleaning"}
    )
    assert resolve_resp.status_code == 200
    assert resolve_resp.json()["status"] == "recleaning"

    telemetry2 = client.get("/api/telemetry").json()
    assert telemetry2["current_state"] == "TARGET_RECLEANING"

    resolve_resp2 = client.post(
        "/api/obstacles/obs_001/resolve", json={"action": "cleared"}
    )
    assert resolve_resp2.json()["status"] == "cleared"


def test_commands_emergency_stop(client: TestClient) -> None:
    resp = client.post("/api/commands/emergency_stop")
    assert resp.status_code == 200
    body = resp.json()
    assert body["current_state"] == "RETURNING"

    telemetry = client.get("/api/telemetry").json()
    assert telemetry["current_state"] == "RETURNING"


def test_commands_target_reclean_requires_target(client: TestClient) -> None:
    resp = client.post("/api/commands", json={"command": "target_reclean"})
    assert resp.status_code == 422


def test_map_upload_with_meta(client: TestClient, tmp_path, monkeypatch) -> None:
    from app.config import settings

    monkeypatch.setattr(settings, "MAPS_DIR", tmp_path)

    assert client.get("/api/map/latest").status_code == 404

    fields = {
        "resolution": "0.05",
        "width": "384",
        "height": "512",
        "origin_x": "-9.6",
        "origin_y": "-12.8",
        "frame_id": "map",
        "start_x": "1.5",
        "start_y": "-0.5",
        "start_theta": "0.25",
    }
    files = {"image": ("latest_map.png", b"\x89PNG-fake", "image/png")}
    with client.websocket_connect("/ws/app") as app_ws:
        app_ws.receive_json()  # 접속 직후 telemetry 스냅샷
        resp = client.post("/api/map", data=fields, files=files)
        assert resp.status_code == 200
        event = app_ws.receive_json()
    assert event["type"] == "map_update"
    assert event["payload"]["version"] == 1

    latest = client.get("/api/map/latest").json()
    assert latest["map_image_url"].endswith("/maps/map_000001.png")
    assert latest["resolution"] == 0.05
    assert (latest["width"], latest["height"]) == (384, 512)
    assert latest["origin"] == {"x": -9.6, "y": -12.8}
    assert latest["start_pose"] == {"x": 1.5, "y": -0.5, "theta": 0.25}
    assert latest["frame_id"] == "map"

    again = client.post("/api/map", data=fields, files=files).json()
    assert again["version"] == 2
    assert again["map_image_url"].endswith("/maps/map_000002.png")
    # 이전 이미지를 덮어쓰지 않는다 (내려받는 도중 교체되어 응답이 끊기는 문제 방지).
    assert (tmp_path / "map_000001.png").exists()


def test_map_old_images_are_pruned(client: TestClient, tmp_path, monkeypatch) -> None:
    from app.config import settings

    monkeypatch.setattr(settings, "MAPS_DIR", tmp_path)
    fields = {"resolution": "0.05", "width": "4", "height": "4", "origin_x": "0", "origin_y": "0"}
    for _ in range(8):
        assert client.post("/api/map", data=fields, files={"image": ("m.png", b"x", "image/png")}).status_code == 200
    names = sorted(p.name for p in tmp_path.glob("map_*.png"))
    assert names == [f"map_{v:06d}.png" for v in (5, 6, 7, 8)]
    assert client.get("/api/map/latest").json()["version"] == 8


def test_map_upload_requires_meta(client: TestClient, tmp_path, monkeypatch) -> None:
    from app.config import settings

    monkeypatch.setattr(settings, "MAPS_DIR", tmp_path)
    resp = client.post("/api/map", files={"image": ("m.png", b"x", "image/png")})
    assert resp.status_code == 422


def test_history_crud(client: TestClient) -> None:
    payload = {
        "map_image_url": None,
        "duration_sec": 120,
        "obstacles_found": 2,
        "started_at": "2026-09-15T10:00:00",
        "finished_at": "2026-09-15T10:02:00",
        "note": "test run",
    }
    create_resp = client.post("/api/history", json=payload)
    assert create_resp.status_code == 200
    record = create_resp.json()
    assert record["duration_sec"] == 120

    list_resp = client.get("/api/history")
    assert len(list_resp.json()) == 1


def _report(client: TestClient, cls: str, x: float, y: float):
    return client.post(
        "/api/obstacles",
        data={"type": cls, "x": x, "y": y},
        files={"image": ("frame.jpg", b"\xff\xd8fake-jpeg", "image/jpeg")},
    )


def test_camera_report_saved_with_time_and_class(client: TestClient, tmp_path, monkeypatch) -> None:
    import re

    from app.config import settings

    monkeypatch.setattr(settings, "IMAGES_DIR", tmp_path)
    state_before = client.get("/api/telemetry").json()["current_state"]
    resp = _report(client, "power_cable", -0.24, -1.92)
    assert resp.status_code == 201
    body = resp.json()
    assert body["type"] == "power_cable"
    assert body["map_coordinate"] == {"x": -0.24, "y": -1.92}
    assert body["status"] == "uncleared"
    assert body["created_at"]
    # 사진은 <시간>_<class>.jpg 로 저장된다.
    saved = [p.name for p in tmp_path.iterdir()]
    assert len(saved) == 1
    assert re.fullmatch(r"\d{8}_\d{6}_power_cable\.jpg", saved[0])
    assert body["image_url"].endswith(f"/images/{saved[0]}")
    assert (tmp_path / saved[0]).read_bytes() == b"\xff\xd8fake-jpeg"
    # 카메라 발견 보고는 FSM 을 EVADING 으로 바꾸지 않는다.
    assert client.get("/api/telemetry").json()["current_state"] == state_before


def test_camera_report_one_marker_per_obstacle(client: TestClient, tmp_path, monkeypatch) -> None:
    from app.config import settings

    monkeypatch.setattr(settings, "IMAGES_DIR", tmp_path)
    first = _report(client, "cardboard_box", 1.08, -0.56)
    again = _report(client, "cardboard_box", 1.08, -0.56)      # 같은 장애물을 계속 보고 있다
    other = _report(client, "urine_puddle", 1.10, -1.83)
    assert (first.status_code, again.status_code, other.status_code) == (201, 200, 201)
    assert again.json()["obstacle_id"] == first.json()["obstacle_id"]
    assert len(list(tmp_path.iterdir())) == 2                  # 중복 보고의 사진은 저장하지 않는다
    listed = client.get("/api/obstacles").json()
    assert sorted(o["type"] for o in listed) == ["cardboard_box", "urine_puddle"]


def test_markers_stay_until_next_cleaning_starts(client: TestClient, tmp_path, monkeypatch) -> None:
    from app.config import settings

    monkeypatch.setattr(settings, "IMAGES_DIR", tmp_path)
    _report(client, "power_cable", -0.24, -1.92)
    # 중간에 종료/복귀해도 표시는 남는다.
    for call in (
        lambda: client.post("/api/commands", json={"command": "stop"}),
        lambda: client.post("/api/commands/emergency_stop"),
        lambda: client.post("/api/commands", json={"command": "start_mapping"}),
    ):
        assert call().status_code == 200
        assert len(client.get("/api/obstacles").json()) == 1

    # 다시 청소를 시작하면 표시가 초기화되고 앱에 알려진다.
    with client.websocket_connect("/ws/app") as app_ws:
        app_ws.receive_json()  # 접속 직후 telemetry 스냅샷
        assert client.post("/api/commands", json={"command": "start_cleaning"}).status_code == 200
        types = []
        while "obstacles_reset" not in types:
            types.append(app_ws.receive_json()["type"])
    assert client.get("/api/obstacles").json() == []
    # 사진/시간/class 기록은 남는다.
    archived = client.get("/api/obstacles", params={"include_archived": "true"}).json()
    assert [o["type"] for o in archived] == ["power_cable"]
    assert len(list(tmp_path.iterdir())) == 1

    # 새 청소에서는 같은 장애물을 다시 발견하면 새 표시가 만들어진다.
    again = _report(client, "power_cable", -0.24, -1.92)
    assert again.status_code == 201
    assert again.json()["obstacle_id"] != archived[0]["obstacle_id"]
    assert len(client.get("/api/obstacles").json()) == 1


def test_map_version_never_goes_back(client: TestClient, tmp_path, monkeypatch) -> None:
    """지도 파일을 지워도 version 은 이어진다(앱은 version 이 바뀔 때 새 지도를 받는다)."""
    from app.config import settings

    monkeypatch.setattr(settings, "MAPS_DIR", tmp_path)
    fields = {"resolution": "0.05", "width": "4", "height": "4", "origin_x": "0", "origin_y": "0"}
    files = {"image": ("m.png", b"x", "image/png")}
    assert client.post("/api/map", data=fields, files=files).json()["version"] == 1
    for path in tmp_path.glob("map_0*.png"):
        path.unlink()
    (tmp_path / "latest_map.json").unlink()
    assert client.post("/api/map", data=fields, files=files).json()["version"] == 2
