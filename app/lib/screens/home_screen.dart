import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/obstacle_log.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';
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
    showAppSnackBar(
      context,
      duration: const Duration(seconds: 6),
      child: Builder(
        builder: (snackContext) => GlassPanel(
          dark: true,
          radius: 24,
          padding: const EdgeInsets.fromLTRB(16, 12, 12, 12),
          child: Row(
            children: [
              const Icon(
                CupertinoIcons.exclamationmark_triangle_fill,
                size: 20,
                color: Colors.white,
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  '장애물 발견 (${obstacle.type}) - 지도에서 확인해주세요.',
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                    height: 1.3,
                    letterSpacing: -0.2,
                  ),
                ),
              ),
              const SizedBox(width: 8),
              _AlertConfirmButton(
                onTap: () {
                  ScaffoldMessenger.of(snackContext).hideCurrentSnackBar();
                  showObstacleDetailSheet(context, obstacle);
                },
              ),
            ],
          ),
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
      // 지도가 유리 상태 바/컨트롤 패널 뒤까지 펼쳐져 Frosted glass 효과가 보이게 한다.
      extendBody: true,
      extendBodyBehindAppBar: true,
      appBar: StatusAppBar(
        onHistoryTap: () => Navigator.of(context).push(
          SpringPageRoute<void>(builder: (_) => const HistoryScreen()),
        ),
        onSettingsTap: () => Navigator.of(context).push(
          SpringPageRoute<void>(builder: (_) => const SettingsScreen()),
        ),
      ),
      body: Consumer<RobotController>(
        builder: (context, controller, _) {
          final Widget content;
          if (controller.lastErrorMessage != null) {
            content = Center(
              key: const ValueKey('error'),
              child: Padding(
                padding: const EdgeInsets.all(24),
                child: SpringEntrance(
                  child: AppCard(
                    radius: 28,
                    padding: const EdgeInsets.all(24),
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        const Icon(
                          CupertinoIcons.wifi_slash,
                          size: 44,
                          color: AppColors.inkTertiary,
                        ),
                        const SizedBox(height: 14),
                        Text(
                          controller.lastErrorMessage!,
                          textAlign: TextAlign.center,
                          style: AppText.body,
                        ),
                        const SizedBox(height: 18),
                        SizedBox(
                          width: 160,
                          child: AppButton(
                            icon: CupertinoIcons.arrow_clockwise,
                            label: '다시 시도',
                            onPressed: () => controller.init(),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            );
          } else {
            content = MapCanvas(
              key: const ValueKey('map'),
              telemetry: controller.telemetry,
              mapInfo: controller.displayedMapInfo,
              mapImageBytes: controller.mapImageBytes,
              obstacles: controller.obstacles,
              // extendBody 로 하단 패널 높이가 body 의 하단 padding 으로 전달된다.
              bottomInset: MediaQuery.paddingOf(context).bottom,
              onObstacleTap: (obstacle) =>
                  showObstacleDetailSheet(context, obstacle),
            );
          }

          // 지도 <-> 오류 화면 전환을 부드럽게.
          return AnimatedSwitcher(
            duration: const Duration(milliseconds: 350),
            switchInCurve: Curves.easeOut,
            switchOutCurve: Curves.easeIn,
            child: content,
          );
        },
      ),
      bottomNavigationBar: const ControlPanel(),
    );
  }
}

/// 알림 팝업의 [확인] 버튼 (흰색 Squircle, 스프링 눌림).
class _AlertConfirmButton extends StatelessWidget {
  const _AlertConfirmButton({required this.onTap});

  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return SpringPressable(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 9),
        decoration: ShapeDecoration(
          color: Colors.white,
          shape: squircle(13),
        ),
        child: const Text(
          '확인',
          style: TextStyle(
            color: AppColors.ink,
            fontSize: 13,
            fontWeight: FontWeight.w700,
          ),
        ),
      ),
    );
  }
}
