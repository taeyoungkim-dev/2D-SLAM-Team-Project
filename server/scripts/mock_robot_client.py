#!/usr/bin/env python3
"""로봇 없이 서버/앱을 개발·테스트하기 위한 모의(mock) 로봇 클라이언트.

실제 TurtleBot3/ROS2 브릿지 노드가 준비되기 전까지, 이 스크립트가
``/ws/robot`` 에 붙어서 가짜 telemetry를 흘려주고, 주기적으로 가짜
장애물을 하나씩 올려준다. 서버가 내려보내는 명령(stop/return/target_reclean
등)도 수신해서 콘솔에 출력한다.

사용법:
    python scripts/mock_robot_client.py --host 127.0.0.1 --port 8000

의존성: websockets (requirements.txt 에 포함됨)
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import math
import urllib.parse
import urllib.request

import websockets


def build_telemetry(step: int) -> dict:
    angle = step * 0.1
    x = 2.0 * math.cos(angle)
    y = 2.0 * math.sin(angle)
    swept_path = [
        {"x": 2.0 * math.cos(a), "y": 2.0 * math.sin(a)}
        for a in [angle - i * 0.1 for i in range(min(step, 20))]
    ]
    battery = max(0, 100 - step // 5)
    return {
        "type": "telemetry",
        "current_state": "CLEANING",
        "current_position": {"x": x, "y": y, "theta": angle % (2 * math.pi)},
        "swept_path": swept_path,
        "battery_level": battery,
    }


def post_fake_obstacle(base_url: str, obstacle_id: str) -> None:
    data = urllib.parse.urlencode(
        {
            "obstacle_id": obstacle_id,
            "type": "towel",
            "x": 1.25,
            "y": -0.5,
        }
    ).encode()
    req = urllib.request.Request(
        f"{base_url}/api/obstacles",
        data=data,
        method="POST",
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    with urllib.request.urlopen(req) as resp:  # noqa: S310 - 로컬 데모 스크립트
        print("[mock-robot] obstacle posted:", resp.read().decode())


async def run(host: str, port: int, obstacle_every: int) -> None:
    ws_url = f"ws://{host}:{port}/ws/robot"
    base_url = f"http://{host}:{port}"
    print(f"[mock-robot] connecting to {ws_url}")

    async with websockets.connect(ws_url) as websocket:

        async def receiver() -> None:
            async for raw in websocket:
                print("[mock-robot] <- command:", raw)

        recv_task = asyncio.create_task(receiver())

        counter = itertools.count(1)
        try:
            for step in counter:
                telemetry = build_telemetry(step)
                await websocket.send(json.dumps(telemetry))

                if obstacle_every > 0 and step % obstacle_every == 0:
                    try:
                        post_fake_obstacle(base_url, f"obs_{step:04d}")
                    except Exception as exc:  # noqa: BLE001
                        print("[mock-robot] failed to post obstacle:", exc)

                await asyncio.sleep(0.5)
        finally:
            recv_task.cancel()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--obstacle-every",
        type=int,
        default=20,
        help="N번째 telemetry마다 가짜 장애물을 하나 등록 (0이면 비활성화)",
    )
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.obstacle_every))


if __name__ == "__main__":
    main()
