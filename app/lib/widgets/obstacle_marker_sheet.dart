import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/obstacle_log.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';

/// REQ-APP-03: 지도 위 장애물 핀을 클릭하면 뜨는 상세 시트.
/// 사진을 보여주고 [치웠음] / [재청소] 버튼으로 서버에 결과를 전송한다.
Future<void> showObstacleDetailSheet(
  BuildContext context,
  ObstacleLog obstacle,
) {
  return showModalBottomSheet<void>(
    context: context,
    isScrollControlled: true,
    backgroundColor: Colors.transparent,
    elevation: 0,
    barrierColor: const Color(0x33000000),
    sheetAnimationStyle: AnimationStyle(
      curve: AppSprings.smooth,
      duration: AppSprings.smooth.duration,
      reverseCurve: Curves.easeInCubic,
      reverseDuration: const Duration(milliseconds: 260),
    ),
    builder: (context) => ObstacleDetailSheet(obstacle: obstacle),
  );
}

class ObstacleDetailSheet extends StatefulWidget {
  const ObstacleDetailSheet({super.key, required this.obstacle});

  final ObstacleLog obstacle;

  @override
  State<ObstacleDetailSheet> createState() => _ObstacleDetailSheetState();
}

class _ObstacleDetailSheetState extends State<ObstacleDetailSheet> {
  bool _submitting = false;

  Future<void> _handleAction(Future<void> Function() action) async {
    setState(() => _submitting = true);
    await action();
    setState(() => _submitting = false);
    if (mounted) Navigator.of(context).pop();
  }

  @override
  Widget build(BuildContext context) {
    final obstacle = widget.obstacle;
    final controller = context.read<RobotController>();

    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
        child: GlassPanel(
          radius: 34,
          blur: 28,
          padding: const EdgeInsets.all(18),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              // 그랩 핸들
              Center(
                child: Container(
                  width: 36,
                  height: 5,
                  margin: const EdgeInsets.only(bottom: 14),
                  decoration: ShapeDecoration(
                    color: AppColors.hairline,
                    shape: squircle(3),
                  ),
                ),
              ),
              Row(
                children: [
                  Container(
                    width: 36,
                    height: 36,
                    decoration: ShapeDecoration(
                      color: AppColors.ink,
                      shape: squircle(12),
                    ),
                    child: const Icon(
                      CupertinoIcons.exclamationmark_triangle_fill,
                      size: 18,
                      color: Colors.white,
                    ),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Text(
                      '장애물 발견: ${obstacle.type}',
                      style: AppText.title,
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Text(
                '좌표: (${obstacle.mapCoordinate.x.toStringAsFixed(2)}, '
                '${obstacle.mapCoordinate.y.toStringAsFixed(2)})  '
                '상태: ${obstacle.status}',
                style: AppText.callout,
              ),
              const SizedBox(height: 14),
              DecoratedBox(
                decoration: ShapeDecoration(
                  shape: squircle(22),
                  shadows: AppShadows.soft,
                ),
                child: ClipPath(
                  clipper: ShapeBorderClipper(shape: squircle(22)),
                  clipBehavior: Clip.antiAlias,
                  child: AspectRatio(
                    aspectRatio: 4 / 3,
                    child: Image.network(
                      obstacle.imageUrl,
                      fit: BoxFit.cover,
                      frameBuilder: (context, child, frame, wasSync) {
                        if (wasSync) return child;
                        return AnimatedOpacity(
                          opacity: frame == null ? 0 : 1,
                          duration: const Duration(milliseconds: 400),
                          curve: Curves.easeOut,
                          child: child,
                        );
                      },
                      errorBuilder: (context, error, stackTrace) => Container(
                        color: AppColors.fillSubtle,
                        alignment: Alignment.center,
                        child: const Icon(
                          CupertinoIcons.photo_fill,
                          size: 44,
                          color: AppColors.inkTertiary,
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 16),
              Row(
                children: [
                  Expanded(
                    child: AppButton(
                      variant: AppButtonVariant.outlined,
                      icon: CupertinoIcons.checkmark_circle,
                      label: '치웠음',
                      onPressed: _submitting
                          ? null
                          : () => _handleAction(
                                () => controller
                                    .markObstacleCleared(obstacle.obstacleId),
                              ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: AppButton(
                      icon: CupertinoIcons.sparkles,
                      label: '재청소',
                      onPressed: _submitting
                          ? null
                          : () => _handleAction(
                                () => controller
                                    .requestRecleaning(obstacle.obstacleId),
                              ),
                    ),
                  ),
                ],
              ),
              AnimatedSize(
                duration: const Duration(milliseconds: 350),
                curve: Curves.easeOutCubic,
                alignment: Alignment.topCenter,
                child: _submitting
                    ? const Padding(
                        padding: EdgeInsets.only(top: 14),
                        child: Center(child: CupertinoActivityIndicator()),
                      )
                    : const SizedBox(width: double.infinity),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
