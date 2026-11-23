import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'core/app_config.dart';
import 'screens/home_screen.dart';
import 'state/robot_controller.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  final config = AppConfig();
  await config.load();

  runApp(RobotVacuumApp(config: config));
}

/// 시각 지능 기반 인터랙티브 로봇청소기 플랫폼 - Mobile App Agent(Flutter) 진입점.
class RobotVacuumApp extends StatelessWidget {
  const RobotVacuumApp({super.key, required this.config});

  final AppConfig config;

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider<AppConfig>.value(value: config),
        ChangeNotifierProvider<RobotController>(
          create: (_) => RobotController(config: config)..init(),
        ),
      ],
      child: MaterialApp(
        title: '로봇청소기 플랫폼',
        debugShowCheckedModeBanner: false,
        theme: ThemeData(
          colorSchemeSeed: Colors.teal,
          useMaterial3: true,
        ),
        home: const HomeScreen(),
      ),
    );
  }
}
