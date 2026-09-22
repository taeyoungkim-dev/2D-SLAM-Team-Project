import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:robot_vacuum_app/core/app_config.dart';
import 'package:robot_vacuum_app/main.dart';

void main() {
  testWidgets('HomeScreen shows the status app bar', (tester) async {
    final config = AppConfig();
    await tester.pumpWidget(RobotVacuumApp(config: config));

    // 초기 렌더링 시 로딩/에러 상태와 무관하게 앱 바 타이틀 영역이 존재해야 한다.
    expect(find.byType(AppBar), findsOneWidget);
    expect(find.byIcon(Icons.settings), findsOneWidget);
    expect(find.byIcon(Icons.history), findsOneWidget);
  });
}
