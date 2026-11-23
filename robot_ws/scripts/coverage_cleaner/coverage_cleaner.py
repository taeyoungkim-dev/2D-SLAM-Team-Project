#!/usr/bin/env python3
"""Standalone coverage planner/tracker for ROS 2 Humble + Nav2 RPP.

변경 (2026-10-26)
  * reclean_unvisited(기본 true): 경로 정상 완료 후 미방문 셀 추가 청소 여부.
    false이면 1회 경로 완료 후 정지하며 방문 기록은 유지한다(자동 복귀 없음).
    오류/취소에 따른 복구 재계획은 유지하며, ~/resume으로 다시 시작할 수 있다.
  * 독립 스위치: configure_goal_checker / check_endpoint / check_safe_position.
    모두 기본 true. --ros-args -p 이름:=false 로 끈다(변경 후 노드 재시작).
  * 끝점 검사 실패 로그에 goal_xy, robot_xy, error, tolerance, TF stamp를 출력.
  * 구간 성공 인정 시 failures를 0으로 초기화: max_failures는 연속 실패 한도.
  * check_endpoint=false: Nav2 성공 후 끝점 거리 검사를 생략한다.
  * check_safe_position=false: footprint/여유 기반 현재 위치·연결 경로 검사를 끈다.
    계획 영역 밖에서는 known-free 경로로 계획 영역에 연결한다. 청소 계획 자체의
    장애물 여유, known-free 경로 조건, TF 시간/연속성 검사, Nav2 검사는 유지한다.
  * configure_goal_checker=false는 기존 Nav2 설정값을 복구하지 않는다.

기준: visual_robot_cleaner_project_source_updated.md (2026-10-21).
입력: /explore_map (OccupancyGrid); TF frame은 메시지 header.frame_id 사용.
실행: python3 coverage_cleaner.py --ros-args -p use_sim_time:=true
미리보기: 위 명령에 -p send_to_nav2:=false 추가.
알고리즘 자체 검사: python3 coverage_cleaner.py --self-test (ROS 불필요).
TF 진단: 실패 사유별로 측정값/기준값과 Nav2 취소 요청 이유를 출력한다.
  tf_timeout_sec는 TF age 허용값, tracker_max_gap_sec는 관측 간격 허용값이다.
장애물 회피: assets/objects/spawn_household_obstacles.launch.py 의 스폰 위치(odom=world -> map TF 변환)에
  SDF 형상 기반 반지름의 원을 "청소 불가 구역"으로 만들어(/coverage_obstacle_zone_map) free 에서 뺀다. 활성 경로가 구역에
  걸리면 멈추고 구역을 뺀 경로로 다시 계획하며, 구역 안에 있으면 먼저 빠져나온다.
  홈 복귀는 구역이 있으면 구역을 뺀 자체 A* 경로를 FollowPath 로 따라간다(없으면 NavigateToPose).
의존성: numpy, scipy; 실행 시 rclpy, tf2_ros, nav2_msgs, nav_msgs,
        geometry_msgs, action_msgs, std_msgs, std_srvs, rcl_interfaces.

동작 기준
  * 현재 위치에서 안전하게 도달 가능한 known-free 영역을 최소 1회 덮는다.
    문/통로를 다시 지나는 것은 허용한다. 최단 경로/정확히 1회 방문 문제는 아니다.
  * 정지 판정: 원형 footprint + coverage_clearance + 격자 보정.
    경로 계획: 정지 판정 거리 + PLANNING_EXTRA_MARGIN_M(아래 상수).
    여유 때문에 계획할 수 없는 벽 주변/좁은 통로는 청소 완료로 기록하지 않는다.
    check_safe_position=true일 때 시작 위치가 계획 영역 밖이면 대기한다.
    coverage_width는 실제로 덮는 폭이며, 기본 0.15m는 설정값이지 실측값이 아니다.
  * 왕복 sweep + A* 연결 + 계획상 누락 셀 보충. 작은 구간도 버리지 않는다.
  * 계획한 경로가 아니라 실제 TF 이동으로 visited를 갱신한다. Nav2 성공만으로
    전체 완료를 선언하지 않는다. reclean_unvisited=true이면 실제 미방문 영역을
    다시 계획한다. false의 complete_pass는 경로 완료이며 전 셀 청소 완료가 아니다.
  * 지도 원점이 정수 셀만큼 이동/확장하면 기록을 보존한다. 해상도/회전/frame
    변경 또는 비정수 셀 이동은 오기록을 피하기 위해 기록을 초기화한다.
  * 전역 위치 추정의 과거 궤적 자체가 바뀌는 SLAM loop closure는 visited를
    자동으로 보정할 수 없다. 이 경우 ~/reset 서비스를 사용한다.
  * FollowPath를 회전 지점마다 분리한다. 설정된 Nav2 goal checker의
    xy_goal_tolerance를 실행 시 goal_xy_tolerance 이하로 낮춘다(기본 0.02m).
    configure_goal_checker:=false이면 이 자동 설정을 끌 수 있다.
    다른 Nav2 목표 발행기(e-explore 포함)는 coverage 실행 전에 정지해야 한다.

출력
  /coverage_path              전체 계획 (Path, transient-local)
  /coverage_active_path       현재 FollowPath 구간 (Path)
  /coverage_visited_map        실제 방문한 free 셀
  /coverage_remaining_map      전체 known-free 중 미방문 셀 (접근 불가도 포함)
  /coverage_target_map         현재 도달 가능한 청소 대상 셀
  /coverage_unreachable_map    free이지만 현재 도달/청소 불가능한 미방문 셀
  /coverage_status            JSON String: 상태, 셀 수, 면적, 비율, 오류, 홈 위치/복귀 단계
입력(스위치)
  /return_home                std_msgs/Bool   True: 청소 중지 후 홈 복귀
  /home_pose                  geometry_msgs/PoseStamped (transient-local) 홈 위치 지정
지도 출력의 100은 해당 마스크를 뜻하며 장애물 costmap 입력으로 쓰지 않는다.

서비스 (노드 이름 기본 coverage_cleaner)
  ros2 service call /coverage_cleaner/pause std_srvs/srv/Trigger '{}'
  ros2 service call /coverage_cleaner/resume std_srvs/srv/Trigger '{}'
  ros2 service call /coverage_cleaner/reset std_srvs/srv/Trigger '{}'

강제 홈 복귀 (스위치 topic)
  ros2 topic pub --once /return_home std_msgs/msg/Bool '{data: true}'
  True 가 오면 청소를 멈추고 시작 위치/yaw 로 돌아온다(NavigateToPose + 제자리 Spin 보정).
  켜지는 경우: (1) 청소 완료 시 이 노드가 직접 (return_home_on_complete),
               (2) 앱 중지 버튼 -> 서버 -> ros2_bridge_node.py 가 같은 topic 에 True 발행.
  False 는 복귀 중단(일시정지 유지). 복귀 중에도 ~/pause, ~/resume, ~/reset 이 우선한다.
  홈 위치: home_x/home_y/home_yaw 파라미터 > /home_pose(PoseStamped) > 첫 TF 위치.
  /coverage_status 의 state 는 returning_home -> returned_home | return_failed 로 바뀐다.
  완료 기준은 실측 TF: home_xy_tolerance(m), home_yaw_tolerance(rad).

주의: FollowPath 자체는 전역 재계획/BT recovery를 실행하지 않는다. 센서의
동적 장애물 회피는 Nav2 costmap/controller가 담당한다. 본 노드는 갱신 지도에서
경로가 막히면 취소 후 재계획하고, 반복 실패는 blocked 상태로 남긴다.
complete_reachable은 현재 known-free의 도달 가능 부분에 대한 판정이다.
미탐사 unknown/벽 주변의 청소 불가능한 띠/분리된 방까지 완료했다는 뜻이 아니다.

모듈 구성 (이 디렉터리, 모두 같은 위치)
  설정 파일(여기만 고치면 됨)
    parameters/feature_flags.py   기능 켜기/끄기 파라미터
    parameters/thresholds.py      임계값/허용오차/제한값 (PLANNING_EXTRA_MARGIN_M 포함)
    parameters/node_settings.py   토픽/프레임/액션 이름
  알고리즘(ROS 불필요, 단독 import/테스트 가능)
    coverage_types / grid_ops / traversability / line_safety / path_search /
    coverage_planner / coverage_tracker / leg_splitter / map_input / home_geometry
  노드 기능(ROS 필요)
    robot_pose / map_updater / planning_area / plan_manager / goal_checker /
    nav2_executor / pass_completion / status_publisher / service_handlers /
    tracking_diagnostics / path_message / tick_controller / home_return
  연결
    cleaner_node.py    모든 기능을 연결하는 ROS 노드
    coverage_cleaner.py  실행 진입점(이 파일)
  테스트: self_test.py
"""
from __future__ import annotations

import sys


def main():
    if '--self-test' in sys.argv:
        from self_test import run
        return run()
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.executors import ExternalShutdownException
        from cleaner_node import CoverageCleaner
    except ImportError as exc:
        print(f'ROS dependency missing: {exc}. Source /opt/ros/humble/setup.bash first.\n'
              'For algorithm-only checks: python3 coverage_cleaner.py --self-test', file=sys.stderr)
        return 2
    # Keep the ROS context alive during Ctrl+C long enough to request cancellation.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CoverageCleaner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
