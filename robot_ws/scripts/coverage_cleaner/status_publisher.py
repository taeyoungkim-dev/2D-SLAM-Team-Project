"""/coverage_status(JSON) 와 디버그 OccupancyGrid 지도 발행."""
from __future__ import annotations

import json

import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class StatusPublisher:
    def __init__(self, ctx, status_pub, debug_pubs):
        self.ctx = ctx
        self.status_pub = status_pub
        self.debug_pubs = debug_pubs

    def publish_status(self):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        status = dict(state=s.state, error=s.error, map_topic=p.map_topic,
                      map_version=s.map_version, navigation_active=s.action_busy,
                      planning_extra_margin_m=p.planning_extra_margin,
                      failures=s.failures,
                      reclean_unvisited=p.reclean_unvisited,
                      single_pass_finished=s.single_pass_finished,
                      configure_goal_checker=p.configure_goal_checker,
                      check_endpoint=p.check_endpoint,
                      check_safe_position=p.check_safe_position)
        status.update(home_mode=s.home_mode, return_phase=ctx.home.phase)
        status['obstacle_zones'] = [
            dict(cls=z.name, x=z.x, y=z.y, radius=z.radius)
            for z in ctx.obstacles.zones()]
        if s.home is not None:
            status['home'] = dict(x=s.home[0], y=s.home[1], yaw=s.home[2])
        if s.target is not None:
            count = int(np.count_nonzero(s.target))
            done = int(np.count_nonzero(s.target & s.tracker.visited))
            unreachable = int(np.count_nonzero(s.free & ~s.target & ~s.tracker.visited))
            status.update(target_cells=count, visited_target_cells=done, remaining_target_cells=count-done,
                          unreachable_unvisited_cells=unreachable,
                          target_coverage_ratio=done/count if count else 0.0,
                          remaining_area_m2=(count-done)*s.meta.resolution**2)
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_mask(self, key, mask):
        ctx = self.ctx
        s = ctx.state
        msg = OccupancyGrid()
        msg.header.stamp = ctx.clock.now().to_msg()
        msg.header.frame_id, msg.info = s.meta.frame, s.info
        msg.data = np.where(mask, 100, 0).astype(np.int8).ravel().tolist()
        self.debug_pubs[key].publish(msg)

    def _zone_mask(self):
        s = self.ctx.state
        return s.zone if s.zone is not None else np.zeros((s.meta.height, s.meta.width), bool)

    def publish_zone_map(self):
        """장애물 때문에 청소할 수 없는 구역(맵 기준)을 발행한다."""
        if self.ctx.state.meta is not None:
            self._publish_mask('obstacle_zone', self._zone_mask())
        self.publish_status()

    def publish_debug(self):
        ctx = self.ctx
        s = ctx.state
        if s.target is None:
            return
        masks = dict(visited=s.tracker.visited & s.free,
                     remaining=s.free & ~s.tracker.visited, target=s.target,
                     unreachable=s.free & ~s.target & ~s.tracker.visited)
        masks['obstacle_zone'] = self._zone_mask()
        for key, mask in masks.items():
            self._publish_mask(key, mask)
        self.publish_status()
