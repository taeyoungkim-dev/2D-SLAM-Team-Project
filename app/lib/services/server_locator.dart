import 'dart:async';
import 'dart:io';

import 'package:http/http.dart' as http;

/// 서버 주소를 사용자가 정확히 입력하지 않아도 서버를 찾아 준다.
///
/// 후보를 순서대로 `GET /api/health` 로 확인해 처음 응답한 호스트를 쓴다.
///   1. 지난번에 찾은 주소, 사용자가 설정에 입력한 값(틀려도 상관없다)
///   2. 안드로이드 에뮬레이터에서 본 PC 주소 `10.0.2.2`
///   3. 기기의 Wi-Fi/네트워크 IP 와 같은 /24 대역 전체 (실제 폰: 같은 공유기의 PC)
class ServerLocator {
  ServerLocator({http.Client? httpClient, this.scanInterval = const Duration(seconds: 20)})
      : _http = httpClient ?? http.Client();

  final http.Client _http;

  /// 같은 대역 전체 스캔은 비싸므로 이 간격 안에서는 다시 하지 않는다.
  final Duration scanInterval;
  DateTime? _lastScan;

  static const _quickTimeout = Duration(milliseconds: 700);
  static const _scanTimeout = Duration(milliseconds: 500);
  static const _scanBatch = 64;

  /// [preferred] 의 각 값은 비어 있거나 잘못된 값이어도 된다(먼저 확인할 후보). 못 찾으면 null.
  Future<String?> find({required List<String> preferred, required int port}) async {
    final quick = <String>{
      for (final raw in preferred)
        if (_cleanHost(raw) != null) _cleanHost(raw)!,
      '10.0.2.2',
    };
    for (final host in quick) {
      if (await _isServer(host, port, _quickTimeout)) return host;
    }

    final now = DateTime.now();
    if (_lastScan != null && now.difference(_lastScan!) < scanInterval) return null;
    _lastScan = now;
    return _scanLocalSubnets(port, skip: quick);
  }

  /// 같은 입력으로 바로 다시 전체 스캔하도록 제한 시간을 지운다(설정 변경, [연결 테스트] 용).
  void forgetLastScan() => _lastScan = null;

  /// 공백/스킴/경로가 섞인 입력에서 호스트만 꺼낸다. 쓸 수 없으면 null.
  static String? _cleanHost(String raw) {
    var text = raw.trim();
    if (text.isEmpty) return null;
    if (!text.contains('://')) text = 'http://$text';
    try {
      final host = Uri.parse(text).host;
      return host.isEmpty ? null : host;
    } catch (_) {
      return null;
    }
  }

  Future<bool> _isServer(String host, int port, Duration timeout) async {
    try {
      final res = await _http.get(Uri.http('$host:$port', '/api/health')).timeout(timeout);
      return res.statusCode == 200;
    } catch (_) {
      return false;
    }
  }

  Future<String?> _scanLocalSubnets(int port, {required Set<String> skip}) async {
    final prefixes = <String>{};
    try {
      final interfaces = await NetworkInterface.list(type: InternetAddressType.IPv4);
      for (final iface in interfaces) {
        for (final addr in iface.addresses) {
          final parts = addr.address.split('.');
          if (parts.length == 4 && !addr.isLoopback) prefixes.add('${parts[0]}.${parts[1]}.${parts[2]}');
        }
      }
    } catch (_) {
      return null;
    }
    final hosts = <String>[
      for (final prefix in prefixes)
        for (var i = 1; i <= 254; i++) '$prefix.$i',
    ]..removeWhere(skip.contains);

    for (var start = 0; start < hosts.length; start += _scanBatch) {
      final batch = hosts.skip(start).take(_scanBatch).toList();
      final found = await Future.wait(batch.map((h) async => await _isServer(h, port, _scanTimeout) ? h : null));
      for (final host in found) {
        if (host != null) return host;
      }
    }
    return null;
  }
}
