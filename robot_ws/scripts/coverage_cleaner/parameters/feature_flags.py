"""기능 켜기/끄기(on/off) 파라미터 모음.

여기 값은 ROS 파라미터의 기본값이다. 파일을 고친 뒤 노드를 재시작하거나,
실행 시 `--ros-args -p 이름:=false` 로 파일 수정 없이 덮어쓸 수 있다.
"""

# True면 시작 즉시 자동 계획/청소를 활성화한다. False면 ~/resume 을 불러야 시작한다.
AUTO_PLAN = True

# False면 Nav2(FollowPath)로 보내지 않고 경로 미리보기(/coverage_path)만 발행한다.
SEND_TO_NAV2 = True

# True: 경로 정상 완료 후 실제 미방문 셀을 다시 계획해 추가 청소한다.
# False: 1회 경로 완료 후 정지하고 방문 기록은 유지한다(오류/취소 복구 재계획은 유지).
RECLEAN_UNVISITED = True

# True: 실행 시 Nav2 goal checker의 xy_goal_tolerance를 goal_xy_tolerance 이하로 낮춘다.
# False여도 기존 Nav2 설정값을 복구하지는 않는다.
CONFIGURE_GOAL_CHECKER = True

# True: Nav2 성공 후 끝점 거리(endpoint_tolerance) 검사를 수행한다.
CHECK_ENDPOINT = True

# True: footprint/여유 기반 현재 위치 및 연결 경로 안전 검사를 수행한다.
CHECK_SAFE_POSITION = True

# True: 청소가 끝나면(complete_pass / complete_reachable) 스스로 복귀 스위치를 켠다.
# False여도 /return_home 에 True 를 보내면 복귀한다(앱의 중지 버튼 경로).
RETURN_HOME_ON_COMPLETE = True


FEATURE_FLAG_DEFAULTS = dict(
    auto_plan=AUTO_PLAN,
    send_to_nav2=SEND_TO_NAV2,
    reclean_unvisited=RECLEAN_UNVISITED,
    configure_goal_checker=CONFIGURE_GOAL_CHECKER,
    check_endpoint=CHECK_ENDPOINT,
    check_safe_position=CHECK_SAFE_POSITION,
    return_home_on_complete=RETURN_HOME_ON_COMPLETE,
)
