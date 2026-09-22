import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/obstacle_log.dart';
import '../state/robot_controller.dart';
import '../widgets/control_panel.dart';
import '../widgets/map_canvas.dart';
import '../widgets/obstacle_marker_sheet.dart';
import '../widgets/status_app_bar.dart';
import 'history_screen.dart';
import 'settings_screen.dart';

/// 메인 대시보드: 상태 바 + 2D 지도 + 명령 패널.
class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  StreamSubscription<ObstacleLog>? _alertSub;

  @override
  void initState() {
    super.initState();
    final controller = context.read<RobotController>();
    // REQ-APP-02: EVADING 진입(장애물 알림) 시 팝업 표시.
    _alertSub = controller.obstacleAlerts.listen(_showObstacleAlert);
  }

  void _showObstacleAlert(ObstacleLog obstacle) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).clearSnackBars();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        duration: const Duration(seconds: 6),
        content: Text('장애물 발견 (${obstacle.type}) - 지도에서 확인해주세요.'),
        action: SnackBarAction(
          label: '확인',
          onPressed: () => showObstacleDetailSheet(context, obstacle),
        ),
      ),
    );
  }

  @override
  void dispose() {
    _alertSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: StatusAppBar(
        onHistoryTap: () => Navigator.of(context).push(
          MaterialPageRoute(builder: (_) => const HistoryScreen()),
        ),
        onSettingsTap: () => Navigator.of(context).push(
          MaterialPageRoute(builder: (_) => const SettingsScreen()),
        ),
      ),
      body: Consumer<RobotController>(
        builder: (context, controller, _) {
          if (controller.lastErrorMessage != null) {
            return Center(
              child: Padding(
                padding: const EdgeInsets.all(24),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Icon(Icons.cloud_off, size: 48, color: Colors.grey),
                    const SizedBox(height: 12),
                    Text(
                      controller.lastErrorMessage!,
                      textAlign: TextAlign.center,
                    ),
                    const SizedBox(height: 12),
                    FilledButton(
                      onPressed: () => controller.init(),
                      child: const Text('다시 시도'),
                    ),
                  ],
                ),
              ),
            );
          }

          return MapCanvas(
            telemetry: controller.telemetry,
            obstacles: controller.obstacles,
            onObstacleTap: (obstacle) =>
                showObstacleDetailSheet(context, obstacle),
          );
        },
      ),
      bottomNavigationBar: const ControlPanel(),
    );
  }
}
