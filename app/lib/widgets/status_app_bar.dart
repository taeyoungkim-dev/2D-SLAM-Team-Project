import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../core/fsm_state.dart';
import '../state/robot_controller.dart';

/// REQ-APP-05: 앱 상단에 현재 로봇 상태(FSM)를 아이콘+텍스트로 상시 표시.
class StatusAppBar extends StatelessWidget implements PreferredSizeWidget {
  const StatusAppBar({super.key, this.onSettingsTap, this.onHistoryTap});

  final VoidCallback? onSettingsTap;
  final VoidCallback? onHistoryTap;

  @override
  Size get preferredSize => const Size.fromHeight(kToolbarHeight);

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotController>(
      builder: (context, controller, _) {
        final state = controller.telemetry.currentState;
        final battery = controller.telemetry.batteryLevel;

        return AppBar(
          title: Row(
            children: [
              Icon(state.icon, color: state.color),
              const SizedBox(width: 8),
              Text(state.label),
            ],
          ),
          actions: [
            _ConnectionDot(connected: controller.wsConnected),
            const SizedBox(width: 4),
            _BatteryIndicator(level: battery),
            const SizedBox(width: 8),
            IconButton(
              icon: const Icon(Icons.history),
              tooltip: '청소 기록',
              onPressed: onHistoryTap,
            ),
            IconButton(
              icon: const Icon(Icons.settings),
              tooltip: '서버 설정',
              onPressed: onSettingsTap,
            ),
          ],
        );
      },
    );
  }
}

class _ConnectionDot extends StatelessWidget {
  const _ConnectionDot({required this.connected});

  final bool connected;

  @override
  Widget build(BuildContext context) {
    return Tooltip(
      message: connected ? '서버 연결됨' : '서버 연결 끊김',
      child: Container(
        width: 10,
        height: 10,
        decoration: BoxDecoration(
          color: connected ? Colors.greenAccent : Colors.redAccent,
          shape: BoxShape.circle,
        ),
      ),
    );
  }
}

class _BatteryIndicator extends StatelessWidget {
  const _BatteryIndicator({required this.level});

  final int level;

  IconData get _icon {
    if (level >= 90) return Icons.battery_full;
    if (level >= 60) return Icons.battery_5_bar;
    if (level >= 30) return Icons.battery_3_bar;
    if (level > 0) return Icons.battery_1_bar;
    return Icons.battery_alert;
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(_icon, size: 20),
        const SizedBox(width: 2),
        Text('$level%'),
      ],
    );
  }
}
