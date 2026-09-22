import 'dart:async';

import 'package:flutter/foundation.dart';

import '../core/app_config.dart';
import '../models/coordinate.dart';
import '../models/history_record.dart';
import '../models/obstacle_log.dart';
import '../models/robot_telemetry.dart';
import '../models/ws_event.dart';
import '../services/api_client.dart';
import '../services/robot_socket_service.dart';

/// 앱 전역 상태를 관리하는 단일 컨트롤러.
///
/// - 서버로부터 받은 최신 [RobotTelemetry] 를 보관 (REQ-APP-01)
/// - 장애물 로그 목록을 보관하고, 새 알림이 오면 [obstacleAlerts] 로 흘려보냄 (REQ-APP-02)
/// - 사용자의 명령(청소 시작/정지/재청소 등)을 서버로 전달 (REQ-SYS-01, REQ-APP-03)
class RobotController extends ChangeNotifier {
  final AppConfig config;
  final ApiClient api;
  final RobotSocketService socket;

  RobotTelemetry telemetry = RobotTelemetry.initial();
  final Map<String, ObstacleLog> _obstacles = {};
  List<HistoryRecord> history = const [];

  bool wsConnected = false;
  bool robotConnected = false;
  String? lastErrorMessage;

  StreamSubscription<WsEvent>? _eventSub;
  StreamSubscription<bool>? _connSub;

  final _alertController = StreamController<ObstacleLog>.broadcast();

  RobotController({required this.config})
      : api = ApiClient(config),
        socket = RobotSocketService(config) {
    config.addListener(_onConfigChanged);
  }

  List<ObstacleLog> get obstacles => _obstacles.values.toList();

  /// REQ-APP-02: 새 장애물 알림(팝업/스낵바 트리거용) 스트림.
  Stream<ObstacleLog> get obstacleAlerts => _alertController.stream;

  Future<void> init() async {
    await _refreshInitialState();
    if (_eventSub == null) {
      _eventSub = socket.events.listen(_onWsEvent);
      _connSub = socket.connectionStream.listen((connected) {
        wsConnected = connected;
        notifyListeners();
      });
      socket.connect();
    }
  }

  Future<void> _refreshInitialState() async {
    try {
      telemetry = await api.getTelemetry();
      final list = await api.getObstacles();
      _obstacles
        ..clear()
        ..addEntries(list.map((o) => MapEntry(o.obstacleId, o)));
      lastErrorMessage = null;
    } catch (e) {
      lastErrorMessage = '서버에 연결할 수 없습니다: $e';
    }
    notifyListeners();
  }

  Future<void> refreshHistory() async {
    try {
      history = await api.getHistory();
      notifyListeners();
    } catch (e) {
      lastErrorMessage = '히스토리를 불러오지 못했습니다: $e';
      notifyListeners();
    }
  }

  void _onWsEvent(WsEvent event) {
    switch (event.type) {
      case 'telemetry':
        telemetry = RobotTelemetry.fromJson(event.payload);
        notifyListeners();
        break;
      case 'obstacle_alert':
        final obstacle = ObstacleLog.fromJson(event.payload);
        _obstacles[obstacle.obstacleId] = obstacle;
        _alertController.add(obstacle);
        notifyListeners();
        break;
      case 'obstacle_update':
        final obstacle = ObstacleLog.fromJson(event.payload);
        _obstacles[obstacle.obstacleId] = obstacle;
        notifyListeners();
        break;
      case 'robot_connected':
        robotConnected = true;
        notifyListeners();
        break;
      case 'robot_disconnected':
        robotConnected = false;
        notifyListeners();
        break;
      default:
        break;
    }
  }

  void _onConfigChanged() {
    socket.reconnect();
    unawaited(_refreshInitialState());
  }

  // -- 사용자 명령 (REQ-SYS-01) -------------------------------------------
  Future<void> startCleaning() => _runCommand(() => api.sendCommand('start_cleaning'));

  Future<void> startMapping() => _runCommand(() => api.sendCommand('start_mapping'));

  Future<void> stop() => _runCommand(() => api.sendCommand('stop'));

  Future<void> emergencyReturn() => _runCommand(api.emergencyStop);

  // -- 장애물 상호작용 (REQ-APP-03) ---------------------------------------
  Future<void> markObstacleCleared(String obstacleId) =>
      _runCommand(() => _resolve(obstacleId, 'cleared'));

  Future<void> requestRecleaning(String obstacleId) =>
      _runCommand(() => _resolve(obstacleId, 'recleaning'));

  Future<void> _resolve(String obstacleId, String action) async {
    final updated = await api.resolveObstacle(obstacleId, action);
    _obstacles[obstacleId] = updated;
  }

  Future<void> _runCommand(Future<void> Function() action) async {
    try {
      await action();
      lastErrorMessage = null;
    } catch (e) {
      lastErrorMessage = '명령 전송에 실패했습니다: $e';
    }
    notifyListeners();
  }

  /// 지도 위 특정 좌표(예: 앱에서 사용자가 직접 지정한 미청소 구역)로 이동.
  Future<void> targetReclean(Coordinate target, {String? obstacleId}) {
    return _runCommand(
      () => api.sendCommand(
        'target_reclean',
        target: {'x': target.x, 'y': target.y},
        obstacleId: obstacleId,
      ),
    );
  }

  @override
  void dispose() {
    config.removeListener(_onConfigChanged);
    _eventSub?.cancel();
    _connSub?.cancel();
    _alertController.close();
    socket.dispose();
    super.dispose();
  }
}
