# AGENTS.md: System Agents and State Specification

This document defines the roles, states (FSM), and communication specifications of each system agent participating in the **"Visual Intelligence-based Interactive Robot Vacuum Platform"** project[cite: 3].

## 1. System Agents
To ensure demo stability, the platform consists of 3 main agents operating on a local Wi-Fi network (Local Area Network) without an external internet connection[cite: 3].

### 1.1. Robot Agent (TurtleBot3)
* **OS / Framework**: Ubuntu, ROS2 humble[cite: 3].
* **Core Features**: Autonomous driving using Nav2 and Cartographer, Object detection via YOLOv8 (2D Camera)[cite: 3].
* **Role**: Performs autonomous driving, detects objects, transforms map coordinates, and acts as the physical robotic agent[cite: 3].

### 1.2. Local Backend Server Agent (Local Server)
* **OS / Framework**: Ubuntu (Host PC), FastAPI (Python)[cite: 3].
* **Communication Protocol**: HTTP (Image serving), WebSocket (Real-time data communication)[cite: 3].
* **Role**: Acts as an API Gateway and State Store between the robot (ROS2) and the mobile app[cite: 3].

### 1.3. Mobile App Agent (User App)
* **OS / Framework**: Android, Flutter (Dart)[cite: 3].
* **Role**: Provides real-time UI/UX to the user, issues commands to the server, and polls the map and status from the server[cite: 3].

---

## 2. Finite State Machine (FSM)
The robot agent always maintains one of the following states, which is shared in real-time with the mobile app via the server[cite: 3].

* `IDLE`: Standby state. Waiting for command reception[cite: 3].
* `MAPPING`: Creating a 2D map via autonomous exploration (Frontier Exploration) or manual control[cite: 3].
* `CLEANING`: Normal operation state, driving along the cleaning path based on Nav2[cite: 3].
* `EVADING`: State where vision (YOLOv8) detects obstacles in the LiDAR blind spot, updates the Keep-Out Zone in the Nav2 Costmap, and generates a detour path[cite: 3].
* `TARGET_RECLEANING`: State where the robot moves to the 'uncleaned area' coordinates specified by the user to perform partial cleaning[cite: 3].
* `RETURNING`: Returning to the starting position (dock) after receiving an emergency stop command or completing the cleaning[cite: 3].

---

## 3. Data Interface
JSON data communication schemas defined to exchange states and commands between agents[cite: 3].

### 3.1. Obstacle Log Schema
Data structure used when the robot evades an obstacle and logs the uncleaned coordinates[cite: 3].

    {
      "obstacle_id": "obs_001",
      "type": "towel",
      "map_coordinate": {"x": 1.25, "y": -0.50},
      "image_url": "http://192.168.x.x:8000/images/obs_001.jpg",
      "status": "uncleared"
    }

> **Note**: When the user reports via the app that the obstacle has been cleared, the `status` changes to "cleared"[cite: 3].

### 3.2. Robot Telemetry Schema
Data structure to render the robot's current state, real-time position, and swept path on the app[cite: 3].

    {
      "current_state": "CLEANING",
      "current_position": {"x": 2.10, "y": 1.15, "theta": 0.5},
      "swept_path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
      "battery_level": 85
    }


# **[AI Context] 시각 지능 기반 인터랙티브 로봇청소기 플랫폼**

## **1. SYSTEM_OVERVIEW (시스템 메타 정보)**
*   **Project_Name:** 시각 지능 기반 인터랙티브 로봇청소기 플랫폼
*   **Core_Value:** 라이다 사각지대의 동적 장애물(전선, 배설물 등)을 2D 카메라로 회피 ➡️ 미청소 좌표 로깅 ➡️ 앱을 통한 사용자 상호작용(HRI) ➡️ 타겟 재청소 수행.
*   **Network_Constraint:** **[중요]** 데모 시연의 안정성을 위해 모든 시스템은 외부 인터넷 연결 없이 **로컬 Wi-Fi 망 내부 통신(Local Area Network)**으로만 동작해야 함. (파이어베이스 배제)

## **2. SYSTEM_ARCHITECTURE (기술 스택 및 통신 구조)**
```text
[TurtleBot3 (ROS2)] <--(ROS Bridge/DDS)--> [Local Server (FastAPI)] <--(REST/WebSocket)--> [App (Flutter)]
- OS: Ubuntu (ROS2 Foxy)                   - OS: Ubuntu (Host PC)                      - OS: Android
- Navigation: Nav2 / Cartographer          - Framework: FastAPI (Python)               - Framework: Flutter (Dart)
- Vision: YOLOv8 (2D Camera)               - Role: State Store, API Gateway            - Role: UI/UX, User Command
- Role: 자율주행, 객체 인식, 좌표 변환        - Protocol: HTTP (Image), WebSocket (Data)  - State: 서버에서 맵/상태 폴링(Polling)
```

## **3. FINITE_STATE_MACHINE (로봇 및 시스템 상태 정의)**
AI 코드 생성을 위한 시스템 상태(State) 정의입니다. 로봇은 항상 아래 상태 중 하나를 가집니다.
*   `IDLE`: 대기 상태. 명령 수신 대기.
*   `MAPPING`: 자율 탐색(Frontier Exploration) 또는 수동 조종을 통한 2D 맵 생성 중.
*   `CLEANING`: Nav2 기반으로 청소 경로를 주행 중 (정상 상태).
*   `EVADING`: 비전(YOLO)이 장애물을 감지하여 Nav2 Costmap에 진입 금지 구역을 업데이트하고 우회 경로를 생성하는 중.
*   `TARGET_RECLEANING`: 사용자가 지정한 '미청소 구역' 좌표로 이동하여 부분 청소 중.
*   `RETURNING`: 긴급 종료 또는 청소 완료 후 시작 위치(도크)로 복귀 중.

## **4. DATA_INTERFACE (API 및 데이터 구조 명세)**
서버와 앱 간의 통신 데이터(JSON) 구조 정의입니다. 향후 코드 생성 시 이 스키마를 따릅니다.

*   **Obstacle_Log_Schema (장애물 기록 데이터)**
    ```json
    {
      "obstacle_id": "obs_001",
      "type": "towel",
      "map_coordinate": {"x": 1.25, "y": -0.50},
      "image_url": "http://192.168.x.x:8000/images/obs_001.jpg",
      "status": "uncleared" // 치워지면 "cleared"로 변경
    }
    ```
*   **Robot_Telemetry_Schema (로봇 실시간 상태 데이터)**
    ```json
    {
      "current_state": "CLEANING",
      "current_position": {"x": 2.10, "y": 1.15, "theta": 0.5},
      "swept_path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}, ...],
      "battery_level": 85
    }
    ```

## **5. REQUIREMENTS_BACKLOG (우선순위 기반 요구사항 명세)**

### **P0 (Critical: 데모 시연을 위한 필수 기능)**
*   **[REQ-ROB-01] SLAM & Navigation:** 로봇은 Cartographer로 맵을 생성하고, Nav2를 통해 주어진 목표점(Goal)으로 주행할 수 있다. (기존 구현된 자율 탐색 활용)
*   **[REQ-ROB-02] Vision Avoidance:** YOLOv8로 장애물 인식 시, 해당 좌표를 FastAPI 서버로 POST 하고, 로봇은 해당 좌표를 우회한다.
*   **[REQ-APP-01] Real-time Telemetry:** 플러터 앱은 FastAPI 서버로부터 `Robot_Telemetry_Schema`를 수신하여 로봇의 위치와 청소 궤적(Swept Path)을 2D 캔버스에 그린다.
*   **[REQ-APP-02] Obstacle Alert:** 로봇이 `EVADING` 상태 진입 시, 앱은 화면 하단에 팝업(Dialog/Snackbar)을 띄우고 장애물 정보를 표시한다.
*   **[REQ-APP-03] Interactive Re-cleaning:** 앱 맵 상의 '장애물 핀' 클릭 ➡️ 사진 확인 ➡️ [치웠음/재청소] 클릭 ➡️ FastAPI로 해당 `obstacle_id`와 좌표 전송 ➡️ 로봇이 해당 좌표로 Nav2 주행(`TARGET_RECLEANING` 상태).
*   **[REQ-SYS-01] Emergency Control:** 앱에서 '정지/복귀' 버튼 클릭 시 즉시 로봇 상태가 `RETURNING` 또는 `IDLE`로 변경된다.

### **P1 (High: 완성도 및 사용자 경험 향상)**
*   **[REQ-APP-04] History Logging:** 청소 완료 시마다 맵 이미지, 소요 시간, 장애물 발견 내역을 DB(SQLite/JSON)에 저장하고, 앱에서 리스트 형태로 조회할 수 있다.
*   **[REQ-APP-05] Status Bar UI:** 앱 상단에 현재 로봇의 상태(FSM 기준)를 직관적인 텍스트와 아이콘으로 상시 표시한다.

### **P2 (Nice-to-have: 추가 개발 목표)**
*   **[REQ-APP-06] Scheduling:** 특정 시간에 API를 호출하여 청소를 시작하는 예약 기능.
*   **[REQ-ROB-04] Active Pushing:** 가벼운 장애물(종이컵, 양말 등) 인식 시 회피하지 않고, 전면 푸셔(Pusher)를 이용해 특정 모음 구역으로 밀어서 치우는 물리적 상호작용 기능.
