"""장애물 스폰 위치 -> 청소 불가 구역(Zone).

카메라는 쓰지 않는다. 장애물 위치는 Gazebo 스폰 launch 파일에서 읽는다.
  params.obstacle_spawn_file   assets/objects/spawn_household_obstacles.launch.py
    그 안의 ``objects = ((이름, sdf, x, y, yaw), ...)`` 를 읽어 이름/x/y 를 얻는다(world 좌표).
  반지름 = thresholds.OBSTACLE_ZONE_RADII_M[이름] (장애물마다 다름. 그대로 구역 반지름으로 쓴다)
    로봇 반지름/여유는 기존 traversable_map 이 free 에서 zone 을 뺀 뒤 자동으로 더한다.

좌표: 스폰 좌표는 Gazebo world 좌표이고, custom_burger 의 diff drive 는 odometry_source=WORLD 라서
  odom frame 이 곧 world frame 이다. 그래서 TF(map <- obstacle_frame, 기본 odom)로 map 좌표로 바꾼다.
  구역은 한 번만 만든다: 지도와 TF 가 처음 모두 준비됐을 때의 map 좌표로 고정하고 이후 다시 갱신하지 않는다
  (Cartographer 가 map->odom 을 나중에 보정해도 구역 위치는 그대로). TF 가 없으면 생길 때까지 구역 없이 동작한다.
  지도 격자가 바뀌면(지도 갱신) 같은 좌표의 구역을 새 격자에 다시 그릴 뿐이다.

앞부분(parse_spawn_file, zone_mask, ObstacleZoneInput 의 변환/비교)은 ROS 없이 동작한다.
"""
from __future__ import annotations

import ast
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from coverage_types import MapMeta
from parameters.thresholds import OBSTACLE_ZONE_DEFAULT_RADIUS_M, OBSTACLE_ZONE_RADII_M


@dataclass(frozen=True)
class Zone:
    name: str
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Spawn:
    name: str
    x: float
    y: float
    yaw: float


def parse_spawn_file(path) -> list[Spawn]:
    """launch 파일의 ``objects = ((이름, sdf, x, y, yaw), ...)`` 에서 스폰 위치를 읽는다 (파일은 실행하지 않음)."""
    tree = ast.parse(Path(path).read_text())
    entries = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'objects' for t in node.targets):
            entries = node.value
            break
    if not isinstance(entries, (ast.Tuple, ast.List)):
        raise ValueError(f"{path} 에서 'objects = (...)' 를 찾지 못했습니다")
    spawns = []
    for entry in entries.elts:
        if not isinstance(entry, ast.Tuple) or len(entry.elts) < 5:
            continue
        name = ast.literal_eval(entry.elts[0])
        x, y, yaw = (float(ast.literal_eval(e)) for e in entry.elts[-3:])
        spawns.append(Spawn(str(name), x, y, yaw))
    if not spawns:
        raise ValueError(f'{path} 에 장애물이 없습니다')
    return spawns


def zone_radius(name: str) -> float:
    return OBSTACLE_ZONE_RADII_M.get(name, OBSTACLE_ZONE_DEFAULT_RADIUS_M)


def to_map(spawn: Spawn, tx: float, ty: float, yaw: float) -> tuple[float, float]:
    """world(=obstacle_frame) 좌표를 map 좌표로. (tx, ty, yaw) = map <- obstacle_frame 변환."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * spawn.x - s * spawn.y, ty + s * spawn.x + c * spawn.y


def zone_mask(meta: MapMeta, zones) -> np.ndarray:
    """zone(원) 안에 셀 중심이 들어오는 셀을 True 로 표시한다(맵 회전과 무관)."""
    mask = np.zeros((meta.height, meta.width), dtype=bool)
    for z in zones:
        lx, ly = meta.local(z.x, z.y)
        r = z.radius / meta.resolution
        x0, x1 = max(0, math.floor(lx - r) - 1), min(meta.width, math.ceil(lx + r) + 2)
        y0, y1 = max(0, math.floor(ly - r) - 1), min(meta.height, math.ceil(ly + r) + 2)
        if x1 <= x0 or y1 <= y0:
            continue
        yy, xx = np.mgrid[y0:y1, x0:x1].astype(float)
        mask[y0:y1, x0:x1] |= (xx + 0.5 - lx) ** 2 + (yy + 0.5 - ly) ** 2 <= r * r
        cx, cy = math.floor(lx), math.floor(ly)
        if 0 <= cx < meta.width and 0 <= cy < meta.height:
            mask[cy, cx] = True
    return mask


class ObstacleZoneInput:
    """스폰 위치를 읽어 두었다가, TF 로 map 좌표를 구해 zone 을 한 번 만들고 MapUpdater 에 알린다.

    lookup(target_frame, source_frame) -> (x, y, yaw) 는 cleaner_node 가 TF 로 구현해 넘긴다.
    TF 를 못 구하면 예외를 던지면 된다(구해질 때까지 zone 없이 기다린다).
    """

    def __init__(self, ctx, lookup=None):
        p = ctx.params
        self.ctx, self.lookup = ctx, lookup
        self.enabled = bool(p.obstacle_avoidance)
        self.spawns: list[Spawn] = []
        self.applied: dict[str, tuple[float, float]] = {}
        if not self.enabled:
            return
        try:
            self.spawns = parse_spawn_file(p.obstacle_spawn_file)
        except (OSError, ValueError, SyntaxError) as exc:
            self.enabled = False
            ctx.logger.error(f'장애물 스폰 파일을 읽지 못해 장애물 구역을 쓰지 않습니다: '
                             f'{p.obstacle_spawn_file}: {exc}')
            return
        for sp in self.spawns:
            ctx.logger.info(f'Obstacle {sp.name}: spawn=({sp.x:.3f}, {sp.y:.3f}) in {p.obstacle_frame!r}; '
                            f'zone radius={zone_radius(sp.name):.3f}m')

    @property
    def done(self) -> bool:
        """구역이 한 번 만들어졌거나(이후 고정), 쓰지 않으면 True."""
        return not self.enabled or len(self.applied) == len(self.spawns)

    def zones(self) -> list[Zone]:
        if not self.enabled:
            return []
        return [Zone(sp.name, *self.applied[sp.name], zone_radius(sp.name))
                for sp in self.spawns if sp.name in self.applied]

    def mask(self, meta: MapMeta):
        zones = self.zones()
        return zone_mask(meta, zones) if zones else None

    def refresh(self):
        """주기 호출(1Hz): map 과 TF 가 처음 준비되면 zone 위치를 한 번 계산해 고정한다. 이후에는 아무것도 하지 않는다."""
        ctx = self.ctx
        meta = ctx.state.meta
        if self.done or meta is None or self.lookup is None:
            return
        frame = ctx.params.obstacle_frame
        try:
            tx, ty, yaw = (0.0, 0.0, 0.0) if frame == meta.frame else self.lookup(meta.frame, frame)
        except Exception as exc:  # noqa: BLE001  TF 예외 종류와 무관하게 "아직 못 구함"
            ctx.warn('obstacle_tf', f'Waiting for TF {meta.frame} <- {frame} to place the obstacle zones: {exc}')
            return
        for sp in self.spawns:
            self.applied[sp.name] = to_map(sp, tx, ty, yaw)
        for z in self.zones():
            ctx.logger.info(f'Obstacle zone {z.name} fixed: map x={z.x:.3f} y={z.y:.3f} radius={z.radius:.3f}m')
        ctx.map_updater.on_zones_changed()
