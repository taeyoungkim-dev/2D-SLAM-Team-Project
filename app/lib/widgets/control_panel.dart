import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../core/fsm_state.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';

/// REQ-SYS-01 + 기본 청소/매핑 시작 제어.
/// 지도 위에 떠 있는 Frosted glass 명령 패널.
class ControlPanel extends StatelessWidget {
  const ControlPanel({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotController>(
      builder: (context, controller, _) {
        final state = controller.telemetry.currentState;
        final isIdle = state == RobotState.idle;

        return SafeArea(
          top: false,
          child: Padding(
            padding: const EdgeInsets.fromLTRB(16, 0, 16, 12),
            child: SpringEntrance(
              fade: false,
              offset: const Offset(0, 48),
              child: GlassPanel(
                radius: 28,
                padding: const EdgeInsets.all(10),
                child: Row(
                  children: [
                    Expanded(
                      child: AppButton(
                        icon: CupertinoIcons.sparkles,
                        label: '청소 시작',
                        onPressed: isIdle ? controller.startCleaning : null,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: AppButton(
                        variant: AppButtonVariant.outlined,
                        icon: CupertinoIcons.map,
                        label: '매핑 시작',
                        onPressed: isIdle ? controller.startMapping : null,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: AppButton(
                        variant: AppButtonVariant.tinted,
                        icon: CupertinoIcons.stop_circle,
                        label: '정지/복귀',
                        onPressed: controller.emergencyReturn,
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}
