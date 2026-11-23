"""Nav2 controller_server 의 goal checker xy_goal_tolerance 를 실행 시 낮추는 기능.

ensure() 가 True 를 돌려줄 때까지 tick 은 청소를 시작하지 않는다.
configure_goal_checker=false 이면 state.param_ready 가 처음부터 True 라 아무 것도 하지 않는다.
"""
from __future__ import annotations

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from parameters.thresholds import PARAM_SERVICE_TIMEOUT_SEC


class GoalCheckerConfigurator:
    def __init__(self, ctx, get_params, set_params):
        self.ctx = ctx
        self.get_params = get_params
        self.set_params = set_params

    def ensure(self):
        ctx = self.ctx
        s = ctx.state
        if s.param_ready:
            return True
        s.state = 'configuring_goal_checker'
        if s.param_busy:
            if time.monotonic() - s.param_since > PARAM_SERVICE_TIMEOUT_SEC:
                ctx.block('Controller parameter service timed out')
            return False
        if not self.get_params.service_is_ready() or not self.set_params.service_is_ready():
            ctx.warn('params', 'Waiting for controller parameter services')
            return False
        s.param_busy, s.param_since = True, time.monotonic()
        request = GetParameters.Request()
        request.names = ['goal_checker_plugins']
        self.get_params.call_async(request).add_done_callback(self.checker_names)
        return False

    def checker_names(self, future):
        ctx = self.ctx
        s = ctx.state
        if s.halted:
            s.param_busy = False
            return
        try:
            names = list(future.result().values[0].string_array_value)
            if not s.goal_checker_id and len(names) == 1:
                s.goal_checker_id = names[0]
            if s.goal_checker_id not in names:
                raise ValueError(f'Set goal_checker_id to one of {names}')
            request = GetParameters.Request()
            request.names = [s.goal_checker_id + '.xy_goal_tolerance']
            self.get_params.call_async(request).add_done_callback(self.checker_tolerance)
        except Exception as exc:
            s.param_busy = False
            ctx.block(f'Cannot select Nav2 goal checker: {exc}')

    def checker_tolerance(self, future):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        if s.halted:
            s.param_busy = False
            return
        try:
            value = future.result().values[0]
            if value.type != ParameterType.PARAMETER_DOUBLE or value.double_value <= 0:
                raise ValueError('Selected checker has no positive DOUBLE xy_goal_tolerance')
            if value.double_value <= p.goal_xy_tolerance:
                s.param_ready, s.param_busy = True, False
                return
            parameter = Parameter()
            parameter.name = s.goal_checker_id + '.xy_goal_tolerance'
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                             double_value=float(p.goal_xy_tolerance))
            request = SetParameters.Request()
            request.parameters = [parameter]
            self.set_params.call_async(request).add_done_callback(self.checker_configured)
        except Exception as exc:
            s.param_busy = False
            ctx.block(f'Cannot read goal tolerance: {exc}')

    def checker_configured(self, future):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        s.param_busy = False
        try:
            result = future.result().results
            if len(result) != 1 or not result[0].successful:
                raise ValueError(result[0].reason if result else 'empty response')
            s.param_ready = True
            ctx.logger.info(f'{s.goal_checker_id}.xy_goal_tolerance set to '
                            f'{p.goal_xy_tolerance:.3f}m for coverage (runtime only)')
        except Exception as exc:
            ctx.block(f'Cannot configure precise goal checking: {exc}')
