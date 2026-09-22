import 'dart:convert';

import 'package:http/http.dart' as http;

import '../core/app_config.dart';
import '../models/history_record.dart';
import '../models/obstacle_log.dart';
import '../models/robot_telemetry.dart';

/// Local Backend Server(FastAPI)와의 REST 통신을 담당한다.
/// (`server/app/routers/*.py` 의 엔드포인트와 1:1 대응)
class ApiClient {
  final AppConfig config;
  final http.Client _http;

  ApiClient(this.config, {http.Client? httpClient})
      : _http = httpClient ?? http.Client();

  Uri _uri(String path, [Map<String, String>? query]) {
    return Uri.parse('${config.httpBaseUrl}$path')
        .replace(queryParameters: query);
  }

  Map<String, dynamic> _decode(http.Response response) {
    if (response.statusCode < 200 || response.statusCode >= 300) {
      throw ApiException(response.statusCode, response.body);
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  List<dynamic> _decodeList(http.Response response) {
    if (response.statusCode < 200 || response.statusCode >= 300) {
      throw ApiException(response.statusCode, response.body);
    }
    return jsonDecode(response.body) as List<dynamic>;
  }

  // -- Telemetry (REQ-APP-01) ------------------------------------------
  Future<RobotTelemetry> getTelemetry() async {
    final res = await _http.get(_uri('/api/telemetry'));
    return RobotTelemetry.fromJson(_decode(res));
  }

  // -- Obstacles (REQ-ROB-02 / REQ-APP-02 / REQ-APP-03) -----------------
  Future<List<ObstacleLog>> getObstacles({String? status}) async {
    final res = await _http.get(
      _uri('/api/obstacles', status != null ? {'status': status} : null),
    );
    return _decodeList(res)
        .map((e) => ObstacleLog.fromJson(e as Map<String, dynamic>))
        .toList();
  }

  Future<ObstacleLog> resolveObstacle(String obstacleId, String action) async {
    final res = await _http.post(
      _uri('/api/obstacles/$obstacleId/resolve'),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'action': action}),
    );
    return ObstacleLog.fromJson(_decode(res));
  }

  // -- Commands (REQ-SYS-01) -------------------------------------------
  Future<void> sendCommand(
    String command, {
    Map<String, double>? target,
    String? obstacleId,
  }) async {
    final res = await _http.post(
      _uri('/api/commands'),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'command': command,
        if (target != null) 'target': target,
        if (obstacleId != null) 'obstacle_id': obstacleId,
      }),
    );
    if (res.statusCode < 200 || res.statusCode >= 300) {
      throw ApiException(res.statusCode, res.body);
    }
  }

  Future<void> emergencyStop() async {
    final res = await _http.post(_uri('/api/commands/emergency_stop'));
    if (res.statusCode < 200 || res.statusCode >= 300) {
      throw ApiException(res.statusCode, res.body);
    }
  }

  // -- History (REQ-APP-04, P1) -----------------------------------------
  Future<List<HistoryRecord>> getHistory() async {
    final res = await _http.get(_uri('/api/history'));
    return _decodeList(res)
        .map((e) => HistoryRecord.fromJson(e as Map<String, dynamic>))
        .toList();
  }

  // -- Health -------------------------------------------------------------
  Future<bool> ping() async {
    try {
      final res = await _http
          .get(_uri('/api/health'))
          .timeout(const Duration(seconds: 3));
      return res.statusCode == 200;
    } catch (_) {
      return false;
    }
  }
}

class ApiException implements Exception {
  final int statusCode;
  final String body;

  ApiException(this.statusCode, this.body);

  @override
  String toString() => 'ApiException($statusCode): $body';
}
