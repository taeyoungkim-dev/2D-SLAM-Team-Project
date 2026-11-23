"""CoverageTracker 가 TF 를 거부했을 때 사유를 경고로 출력."""
from __future__ import annotations


def log_tracking_diagnostic(ctx, source):
    s, p = ctx.state, ctx.params
    issue = s.tracker.last_diagnostic
    if issue is not None:
        # Throttle each cause separately so one failure cannot hide another.
        ctx.warn('tracker_' + issue.reason,
                 f'Tracker rejected TF: reason={issue.reason}; {issue.details}; '
                 f'source={source}; frames={s.meta.frame}->{p.robot_base_frame}; '
                 'gap not marked visited')
