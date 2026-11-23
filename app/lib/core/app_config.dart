import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../services/server_locator.dart';

/// Local Backend Server(FastAPI)의 접속 정보를 보관/관리한다.
///
/// AGENTS.md 의 네트워크 제약("외부 인터넷 없이 로컬 Wi-Fi 망 내부 통신")에
/// 따라 서버 IP 는 데모 환경마다 달라질 수 있다. 사용자가 설정 화면에 입력한 값([host])은
/// "먼저 시도해 볼 주소"일 뿐이고, 비어 있거나 틀려도 [resolve] 가 서버를 찾아
/// 실제로 접속할 주소([activeHost])를 정한다. [SharedPreferences] 에는 입력값을 저장한다.
class AppConfig extends ChangeNotifier {
  AppConfig({ServerLocator? locator}) : _locator = locator ?? ServerLocator();

  static const _prefsHostKey = 'server_host';
  static const _prefsPortKey = 'server_port';

  final ServerLocator _locator;

  String _host = '';
  int _port = 8000;
  String? _foundHost;
  Future<bool>? _resolving;

  /// 설정 화면에서 값을 저장할 때마다 올라간다(서버 찾기를 다시 시작해야 하는 신호).
  int revision = 0;

  /// 사용자가 입력한 값(비어 있을 수 있다).
  String get host => _host;
  int get port => _port;

  /// 실제 접속에 쓰는 주소: 찾은 서버가 있으면 그것, 없으면 입력값(없으면 에뮬레이터용 10.0.2.2).
  String get activeHost => _foundHost ?? (_host.isNotEmpty ? _host : '10.0.2.2');

  String get httpBaseUrl => 'http://$activeHost:$_port';
  String get wsAppUrl => 'ws://$activeHost:$_port/ws/app';

  Future<void> load() async {
    final prefs = await SharedPreferences.getInstance();
    _host = prefs.getString(_prefsHostKey) ?? _host;
    _port = prefs.getInt(_prefsPortKey) ?? _port;
    notifyListeners();
  }

  Future<void> update({required String host, required int port}) async {
    _host = host;
    _port = port;
    _foundHost = null;
    revision++;
    _locator.forgetLastScan();
    notifyListeners();

    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_prefsHostKey, host);
    await prefs.setInt(_prefsPortKey, port);
  }

  /// 서버를 찾아 [activeHost] 를 정한다. 동시에 여러 번 불러도 한 번만 실행한다.
  /// 서버를 찾았으면 true. (이미 알던 주소와 같으면 리스너에 알리지 않는다.)
  Future<bool> resolve({bool force = false}) {
    if (force) _locator.forgetLastScan();
    return _resolving ??= _resolveOnce().whenComplete(() => _resolving = null);
  }

  Future<bool> _resolveOnce() async {
    final found = await _locator.find(
      preferred: [if (_foundHost != null) _foundHost!, _host],
      port: _port,
    );
    if (found == null) return false;
    if (found != _foundHost) {
      _foundHost = found;
      notifyListeners();
    }
    return true;
  }
}
