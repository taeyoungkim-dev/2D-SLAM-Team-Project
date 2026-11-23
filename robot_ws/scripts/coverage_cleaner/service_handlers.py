"""~/pause, ~/resume, ~/reset (std_srvs/Trigger) 처리. pause/resume/reset 은 홈 복귀도 중단한다."""
from __future__ import annotations


class ServiceHandlers:
    def __init__(self, ctx):
        self.ctx = ctx

    def on_pause(self, request, response):
        ctx = self.ctx
        s = ctx.state
        ctx.home.abort('paused')  # 홈 복귀 중이면 그것도 멈춘다.
        ctx.pause('paused')
        s.state = 'paused'
        response.success, response.message = True, 'Paused; waiting for active Nav2 cancellation if any'
        return response

    def on_resume(self, request, response):
        ctx = self.ctx
        s = ctx.state
        ctx.home.abort('resume')  # 복귀를 중단하고 청소를 다시 계획한다.
        s.single_pass_finished = False
        s.enabled, s.halted, s.error = True, False, ''
        s.failures = s.no_progress_passes = 0
        s.pass_start_count = None
        s.plan_version = -1
        s.param_busy = False
        s.target = None
        s.preview_version = -1
        s.legs.clear()
        s.pass_in_progress = False
        ctx.executor.cancel('resume_replan')
        response.success, response.message = True, 'Will replan remaining cells when map/TF/Nav2 are ready'
        return response

    def on_reset(self, request, response):
        ctx = self.ctx
        s = ctx.state
        s.single_pass_finished = False
        self.on_pause(request, response)
        if s.tracker:
            s.tracker.visited.fill(False)
            s.tracker.previous = None
        s.map_version += 1  # Discard any in-flight plan using old visited cells.
        s.pass_start_count = None
        s.preview_version = -1
        s.halted, s.error = False, ''
        response.message = 'Visited cleared and paused; call ~/resume to start a new pass'
        return response
