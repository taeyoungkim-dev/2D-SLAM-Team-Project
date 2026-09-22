import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// Local Backend Server(FastAPI)의 접속 정보를 보관/관리한다.
///
/// AGENTS.md 의 네트워크 제약("외부 인터넷 없이 로컬 Wi-Fi 망 내부 통신")에
/// 따라, 서버 IP는 데모 환경마다 달라질 수 있으므로 앱 내 설정 화면에서
/// 바꿀 수 있게 하고 [SharedPreferences] 에 저장해 재실행 시에도 유지한다.
class AppConfig extends ChangeNotifier {
  static const _prefsHostKey = 'server_host';
  static const _prefsPortKey = 'server_port';

  String _host = '192.168.0.10';
  int _port = 8000;

  String get host => _host;
  int get port => _port;

  String get httpBaseUrl => 'http://$_host:$_port';
  String get wsAppUrl => 'ws://$_host:$_port/ws/app';

  Future<void> load() async {
    final prefs = await SharedPreferences.getInstance();
    _host = prefs.getString(_prefsHostKey) ?? _host;
    _port = prefs.getInt(_prefsPortKey) ?? _port;
    notifyListeners();
  }

  Future<void> update({required String host, required int port}) async {
    _host = host;
    _port = port;
    notifyListeners();

    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_prefsHostKey, host);
    await prefs.setInt(_prefsPortKey, port);
  }
}
