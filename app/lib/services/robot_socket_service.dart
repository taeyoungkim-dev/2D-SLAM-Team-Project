import 'dart:async';
import 'dart:convert';

import 'package:web_socket_channel/web_socket_channel.dart';

import '../core/app_config.dart';
import '../models/ws_event.dart';

/// `/ws/app` 에 연결해 서버가 브로드캐스트하는 telemetry/장애물 이벤트를
/// 실시간으로 수신한다 (REQ-APP-01, REQ-APP-02).
///
/// 연결이 끊기면 지수 백오프 없이 고정 간격(3초)으로 재접속을 시도한다.
/// 데모(로컬 LAN)환경에서는 재접속 지연을 짧게 유지하는 것이 더 유리하다.
class RobotSocketService {
  final AppConfig config;

  final _eventController = StreamController<WsEvent>.broadcast();
  final _connectionController = StreamController<bool>.broadcast();

  WebSocketChannel? _channel;
  StreamSubscription<dynamic>? _subscription;
  Timer? _reconnectTimer;
  bool _disposed = false;
  bool _connected = false;

  RobotSocketService(this.config);

  Stream<WsEvent> get events => _eventController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;
  bool get isConnected => _connected;

  void connect() {
    _disposed = false;
    _openChannel();
  }

  void _openChannel() {
    if (_disposed) return;
    _reconnectTimer?.cancel();

    try {
      final channel = WebSocketChannel.connect(Uri.parse(config.wsAppUrl));
      _channel = channel;
      _subscription = channel.stream.listen(
        _onMessage,
        onError: (_) => _handleDisconnect(),
        onDone: _handleDisconnect,
        cancelOnError: true,
      );
      _setConnected(true);
    } catch (_) {
      _handleDisconnect();
    }
  }

  void _onMessage(dynamic raw) {
    try {
      final decoded = jsonDecode(raw as String) as Map<String, dynamic>;
      _eventController.add(WsEvent.fromJson(decoded));
    } catch (_) {
      // 서버가 예상치 못한 형식을 보냈을 경우 조용히 무시한다.
    }
  }

  void _handleDisconnect() {
    _setConnected(false);
    _subscription?.cancel();
    _subscription = null;
    _channel = null;
    if (!_disposed) {
      _reconnectTimer?.cancel();
      _reconnectTimer = Timer(const Duration(seconds: 3), _openChannel);
    }
  }

  void _setConnected(bool value) {
    if (_connected == value) return;
    _connected = value;
    _connectionController.add(value);
  }

  /// 접속 정보(서버 IP)가 바뀌었을 때 강제로 재접속한다.
  void reconnect() {
    _subscription?.cancel();
    _channel?.sink.close();
    _openChannel();
  }

  void dispose() {
    _disposed = true;
    _reconnectTimer?.cancel();
    _subscription?.cancel();
    _channel?.sink.close();
    _eventController.close();
    _connectionController.close();
  }
}
