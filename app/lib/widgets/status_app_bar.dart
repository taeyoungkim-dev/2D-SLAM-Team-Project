import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import '../core/fsm_state.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';

/// REQ-APP-05: 앱 상단에 현재 로봇 상태(FSM)를 아이콘+텍스트로 상시 표시.
///
/// 지도 위에 떠 있는 Frosted glass 바. 상태가 바뀌면 칩이 스프링으로 교체된다.
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
          automaticallyImplyLeading: false,
          backgroundColor: Colors.transparent,
          surfaceTintColor: Colors.transparent,
          elevation: 0,
          scrolledUnderElevation: 0,
          systemOverlayStyle: SystemUiOverlayStyle.dark,
          flexibleSpace: const GlassBar(),
          titleSpacing: 16,
          title: Align(
            alignment: Alignment.centerLeft,
            child: AnimatedSwitcher(
              duration: AppSprings.bouncy.duration,
              transitionBuilder: springSwitcherTransition,
              layoutBuilder: (current, previous) => Stack(
                alignment: Alignment.centerLeft,
                children: [...previous, if (current != null) current],
              ),
              child: _StatePill(key: ValueKey(state), state: state),
            ),
          ),
          actions: [
            _ConnectionDot(connected: controller.wsConnected),
            const SizedBox(width: 10),
            _BatteryIndicator(level: battery),
            const SizedBox(width: 10),
            AppIconButton(
              icon: CupertinoIcons.clock,
              tooltip: '청소 기록',
              onTap: onHistoryTap,
            ),
            const SizedBox(width: 8),
            AppIconButton(
              icon: CupertinoIcons.gear,
              tooltip: '서버 설정',
              onTap: onSettingsTap,
            ),
            const SizedBox(width: 16),
          ],
        );
      },
    );
  }
}

/// 현재 FSM 상태 칩. 대기=옅은 회색, 동작 중=짙은 회색, 장애물 회피=검정(고대비 강조).
class _StatePill extends StatelessWidget {
  const _StatePill({super.key, required this.state});

  final RobotState state;

  @override
  Widget build(BuildContext context) {
    final Color bg;
    final Color fg;
    switch (state) {
      case RobotState.idle:
        bg = AppColors.fillSubtle;
        fg = AppColors.inkSecondary;
      case RobotState.evading:
        bg = AppColors.ink;
        fg = Colors.white;
      case RobotState.mapping:
      case RobotState.cleaning:
      case RobotState.targetRecleaning:
      case RobotState.returning:
        bg = AppColors.inkSecondary;
        fg = Colors.white;
    }

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
      decoration: ShapeDecoration(
        color: bg,
        shape: squircle(14),
        shadows: state == RobotState.idle ? const [] : AppShadows.soft,
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(robotStateIcon(state), size: 16, color: fg),
          const SizedBox(width: 6),
          Text(
            state.label,
            style: TextStyle(
              color: fg,
              fontSize: 14,
              fontWeight: FontWeight.w600,
              letterSpacing: -0.2,
            ),
          ),
        ],
      ),
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
      child: AnimatedContainer(
        duration: AppSprings.decorationDuration,
        curve: AppSprings.decorationCurve,
        width: 10,
        height: 10,
        decoration: BoxDecoration(
          // 연결됨 = 꽉 찬 검정 점, 끊김 = 속이 빈 회색 링.
          color: connected ? AppColors.ink : Colors.transparent,
          border: Border.all(
            color: connected ? AppColors.ink : AppColors.inkTertiary,
            width: 1.6,
          ),
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
    if (level >= 90) return CupertinoIcons.battery_full;
    if (level >= 60) return CupertinoIcons.battery_75_percent;
    if (level >= 30) return CupertinoIcons.battery_25_percent;
    return CupertinoIcons.battery_empty;
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        AnimatedSwitcher(
          duration: const Duration(milliseconds: 300),
          transitionBuilder: springSwitcherTransition,
          child: Icon(
            _icon,
            key: ValueKey(_icon),
            size: 22,
            color: AppColors.ink,
          ),
        ),
        const SizedBox(width: 3),
        // 배터리 숫자가 바뀔 때 부드럽게 굴러가는 카운터.
        TweenAnimationBuilder<double>(
          tween: Tween<double>(end: level.toDouble()),
          duration: const Duration(milliseconds: 500),
          curve: Curves.easeOutCubic,
          builder: (context, value, _) => Text(
            '${value.round()}%',
            style: const TextStyle(
              color: AppColors.ink,
              fontSize: 13,
              fontWeight: FontWeight.w600,
              fontFeatures: [FontFeature.tabularFigures()],
            ),
          ),
        ),
      ],
    );
  }
}
