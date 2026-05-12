[ taeyoung_explorer 개발 요구사항 명세서 ]

1. 입력 데이터 (Subscription)
- 토픽 이름: /global_costmap/costmap (또는 상황에 따라 /map)
- 메시지 타입: nav_msgs/msg/OccupancyGrid
- 목적: 지도의 0(빈 공간), 100(벽), -1(미지 영역) 데이터를 2D Numpy 행렬로 변환하여 분석.

2. 핵심 로직 (Frontier Detection)
- 알고리즘: 0(Free)과 -1(Unknown)이 맞닿은 경계 픽셀 추출.
- 필터링 조건: 
  * 너무 작은 경계선은 노이즈로 간주하고 제거 (min_frontier_size 적용).
  * Nav2가 갈 수 없다고 실패를 반환한 곳(블랙리스트) 제외.
- 목표 선정: 현재 로봇 위치(TF: base_link)에서 가장 가까운 프론티어 군집의 중심점(Center) 계산.

3. 출력 명령 (Action Client)
- 액션 이름: /navigate_to_pose
- 액션 타입: nav2_msgs/action/NavigateToPose
- 방식: 단순 Publish가 아닌 Action Client 방식을 사용하여, 로봇이 이동 중인지, 도착 완료했는지, 경로 생성에 실패했는지(튕겼는지) 피드백을 받아 다음 행동(루프)을 결정.