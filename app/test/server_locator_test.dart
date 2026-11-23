import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';

import 'package:robot_vacuum_app/services/server_locator.dart';

/// [alive] 에 있는 호스트만 /api/health 에 200 으로 응답하는 가짜 HTTP 클라이언트.
ServerLocator locatorWith(Set<String> alive, {List<String>? asked}) {
  return ServerLocator(
    httpClient: MockClient((http.Request request) async {
      asked?.add(request.url.host);
      return http.Response(
        alive.contains(request.url.host) ? '{"status":"ok"}' : 'no',
        alive.contains(request.url.host) ? 200 : 404,
      );
    }),
  );
}

void main() {
  test('입력한 IP 가 맞으면 그 주소를 쓴다', () async {
    final locator = locatorWith({'192.168.0.2'});
    expect(await locator.find(preferred: ['192.168.0.2'], port: 8000), '192.168.0.2');
  });

  test('IP 를 틀리게 입력해도 에뮬레이터 주소 10.0.2.2 로 연결된다', () async {
    final locator = locatorWith({'10.0.2.2'});
    expect(await locator.find(preferred: ['아무거나 !!'], port: 8000), '10.0.2.2');
    expect(await locator.find(preferred: ['1.2.3.4'], port: 8000), '10.0.2.2');
    expect(await locator.find(preferred: [''], port: 8000), '10.0.2.2');
  });

  test('http:// 나 포트가 붙은 입력에서도 호스트만 쓴다', () async {
    final asked = <String>[];
    final locator = locatorWith({'192.168.0.2'}, asked: asked);
    expect(await locator.find(preferred: ['http://192.168.0.2:8000/'], port: 8000), '192.168.0.2');
    expect(asked.first, '192.168.0.2');
  });

  test('서버가 어디에도 없으면 null', () async {
    final locator = locatorWith({});
    expect(await locator.find(preferred: ['1.2.3.4'], port: 8000), isNull);
  });
}
