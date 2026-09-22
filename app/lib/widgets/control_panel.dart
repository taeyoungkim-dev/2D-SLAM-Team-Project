import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../core/fsm_state.dart';
import '../state/robot_controller.dart';

/// REQ-SYS-01 + 기본 청소/매핑 시작 제어.
/// 화면 하단에 항상 떠 있는 명령 버튼 모음.
class ControlPanel extends StatelessWidget {
  const ControlPanel({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotController>(
      builder: (context, controller, _) {
        final state = controller.telemetry.currentState;
        final isIdle = state == RobotState.idle;

        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          decoration: BoxDecoration(
            color: Theme.of(context).colorScheme.surface,
            boxShadow: const [
              BoxShadow(color: Colors.black12, blurRadius: 8, offset: Offset(0, -2)),
            ],
          ),
          child: SafeArea(
            top: false,
            child: Row(
              children: [
                Expanded(
                  child: FilledButton.icon(
                    icon: const Icon(Icons.cleaning_services),
                    label: const Text('청소 시작'),
                    onPressed: isIdle ? controller.startCleaning : null,
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: OutlinedButton.icon(
                    icon: const Icon(Icons.map_outlined),
                    label: const Text('매핑 시작'),
                    onPressed: isIdle ? controller.startMapping : null,
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: FilledButton.tonalIcon(
                    style: FilledButton.styleFrom(
                      backgroundColor: Colors.red.withValues(alpha: 0.12),
                      foregroundColor: Colors.red,
                    ),
                    icon: const Icon(Icons.stop_circle_outlined),
                    label: const Text('정지/복귀'),
                    onPressed: controller.emergencyReturn,
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
