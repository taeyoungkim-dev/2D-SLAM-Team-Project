# 로봇청소기 플랫폼 - Mobile App

시각 지능 기반 인터랙티브 로봇청소기 플랫폼의 **Flutter 앱**입니다.
Local Backend Server(FastAPI, `../server`)와 REST/WebSocket으로 통신하며
로봇의 실시간 위치·궤적·배터리 상태를 2D 지도에 표시하고, 장애물 알림을
받아 [치웠음]/[재청소]로 응답하거나 정지/복귀 명령을 보낼 수 있습니다.

## 실행에 필요한 것

- **Flutter SDK** (3.x, Dart 포함) - https://docs.flutter.dev/get-started/install
- 실행할 타겟 하나:
  - Chrome/Edge 등 **웹 브라우저** (`flutter run -d web-server`, 가장 간단)
  - **Android 에뮬레이터** 또는 실제 **Android 기기** (USB 디버깅)
  - **Windows 데스크톱** (Visual Studio C++ 빌드 도구 필요)
- (선택, 실데이터 확인용) **Local Backend Server** 실행 중이어야 함 → `../server/README.md` 참고
  - 서버 없이 켜도 앱 UI 자체는 뜨며, 연결 안내 화면만 대신 표시됩니다.

## 빠른 실행

```bash
cd app
flutter create .      # 플랫폼(android/ios/web 등) 폴더 최초 1회 생성
flutter pub get
flutter run -d web-server --web-hostname=0.0.0.0 --web-port=8080
```

앱 안 설정(⚙️) 화면에서 서버 IP/포트를 입력하면 됩니다.

더 자세한 내용은 `README.md` 를 참고하세요.
