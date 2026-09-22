# Mobile App (Flutter) - User App

`AGENTS.md` 의 **1.3 Mobile App Agent** 구현체입니다.
Local Backend Server(FastAPI, `../server`)와 REST/WebSocket으로 통신하여
로봇의 실시간 위치/궤적을 2D 캔버스에 표시하고, 장애물 알림 및 재청소
요청 등 사용자 상호작용(HRI)을 처리합니다.

이 디렉터리에는 `lib/`, `pubspec.yaml`, `test/` 등 **Flutter 소스**만 포함되어
있습니다. `android/`, `ios/` 등 플랫폼별 러너 프로젝트는 아래 안내대로
로컬에서 `flutter create .` 로 직접 생성해서 사용하세요 (팀원마다 Flutter
버전이 달라 플랫폼 산출물이 계속 충돌하는 것을 방지하기 위해 커밋하지 않습니다).

## 1. 디렉터리 구조

```
app/
├── lib/
│   ├── main.dart                    # 앱 진입점, Provider 초기화
│   ├── core/
│   │   ├── app_config.dart          # 서버 IP/포트 설정 (SharedPreferences 저장)
│   │   └── fsm_state.dart           # RobotState enum + 라벨/아이콘/색상 (REQ-APP-05)
│   ├── models/                      # 서버 JSON 스키마 <-> Dart 모델
│   │   ├── coordinate.dart          # Coordinate, Pose2D
│   │   ├── robot_telemetry.dart     # RobotTelemetry (AGENTS.md 3.2)
│   │   ├── obstacle_log.dart        # ObstacleLog (AGENTS.md 3.1)
│   │   ├── history_record.dart      # 청소 기록 (REQ-APP-04)
│   │   └── ws_event.dart            # WebSocket envelope
│   ├── services/
│   │   ├── api_client.dart          # REST 클라이언트 (http)
│   │   └── robot_socket_service.dart# /ws/app 실시간 클라이언트 (자동 재접속)
│   ├── state/
│   │   └── robot_controller.dart    # 앱 전역 상태 (ChangeNotifier)
│   ├── screens/
│   │   ├── home_screen.dart         # 메인 대시보드 (지도 + 명령 패널)
│   │   ├── history_screen.dart      # 청소 기록 리스트 (REQ-APP-04)
│   │   └── settings_screen.dart     # 서버 IP/포트 설정
│   └── widgets/
│       ├── status_app_bar.dart      # 상단 FSM 상태 표시 (REQ-APP-05)
│       ├── map_canvas.dart          # 2D 캔버스: 위치/궤적/장애물 핀 (REQ-APP-01)
│       ├── obstacle_marker_sheet.dart # 장애물 상세 + 치웠음/재청소 (REQ-APP-03)
│       └── control_panel.dart       # 청소/매핑 시작, 정지/복귀 (REQ-SYS-01)
├── test/
│   └── widget_test.dart
├── pubspec.yaml
└── analysis_options.yaml
```

## 2. 실행 방법

```bash
cd app
flutter create .          # android/ios 등 플랫폼 러너를 처음 한 번 생성
flutter pub get
flutter run                # 연결된 기기/에뮬레이터에서 실행
```

앱 실행 후 우측 상단 ⚙️ 아이콘 → **서버 설정** 화면에서 Local Backend
Server가 실행 중인 PC의 LAN IP와 포트(기본 `8000`)를 입력하세요.
(`server/README.md` 의 `SERVER_PUBLIC_BASE_URL` 과 동일한 IP)

### 정적 분석 / 테스트

```bash
flutter analyze
flutter test
```

## 3. 화면 흐름

1. **홈 화면**: 상단 상태바(FSM 아이콘+텍스트, 배터리, 서버 연결 표시) +
   중앙 2D 지도 캔버스 + 하단 명령 버튼(청소 시작/매핑 시작/정지·복귀).
2. 로봇이 장애물을 감지해 `EVADING` 상태로 바뀌면, 서버가 `/ws/app` 으로
   `obstacle_alert` 이벤트를 브로드캐스트 → 화면 하단에 스낵바 팝업
   (REQ-APP-02).
3. 지도 위 빨간 장애물 핀을 탭하면 사진과 좌표가 담긴 바텀시트가 열리고,
   [치웠음] / [재청소] 버튼으로 서버에 결과를 전송 (REQ-APP-03).
   - [재청소] 선택 시 서버가 로봇에게 `target_reclean` 명령을 내려보내고
     로봇 상태가 `TARGET_RECLEANING` 으로 바뀌는 것을 앱이 실시간으로 반영.
4. 상단 시계 아이콘으로 **청소 기록** 화면 이동 (REQ-APP-04, P1).

## 4. 서버 연동 방식

- **REST (`ApiClient`)**: 최초 진입 시 telemetry/장애물 목록을 가져오고,
  버튼 클릭 시 명령/재청소 요청을 보낸다.
- **WebSocket (`RobotSocketService`)**: `/ws/app` 에 연결해 서버가 실시간으로
  보내주는 telemetry 갱신, 장애물 알림을 수신한다. 연결이 끊기면 3초
  간격으로 자동 재접속한다.
- 서버 IP를 설정 화면에서 바꾸면 `AppConfig` 가 변경을 통지하고,
  `RobotController` 가 이를 감지해 WebSocket을 재접속하고 REST 상태를
  다시 불러온다.

## 5. 로봇/서버 없이 UI만 확인하고 싶을 때

`server/scripts/mock_robot_client.py` 로 가짜 로봇을 띄운 상태에서 서버를
실행해두면, 실제 하드웨어 없이도 앱에서 실시간 이동/장애물 알림/재청소
흐름을 전부 확인할 수 있습니다. (`server/README.md` 3장 참고)
