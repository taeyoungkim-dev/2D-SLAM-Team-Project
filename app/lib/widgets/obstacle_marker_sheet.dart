import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/obstacle_log.dart';
import '../state/robot_controller.dart';

/// REQ-APP-03: 지도 위 장애물 핀을 클릭하면 뜨는 상세 시트.
/// 사진을 보여주고 [치웠음] / [재청소] 버튼으로 서버에 결과를 전송한다.
Future<void> showObstacleDetailSheet(
  BuildContext context,
  ObstacleLog obstacle,
) {
  return showModalBottomSheet<void>(
    context: context,
    isScrollControlled: true,
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
        padding: const EdgeInsets.all(16),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.warning_amber_rounded, color: Colors.orange),
                const SizedBox(width: 8),
                Expanded(
                  child: Text(
                    '장애물 발견: ${obstacle.type}',
                    style: Theme.of(context).textTheme.titleLarge,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 4),
            Text(
              '좌표: (${obstacle.mapCoordinate.x.toStringAsFixed(2)}, '
              '${obstacle.mapCoordinate.y.toStringAsFixed(2)})  '
              '상태: ${obstacle.status}',
              style: Theme.of(context).textTheme.bodySmall,
            ),
            const SizedBox(height: 12),
            ClipRRect(
              borderRadius: BorderRadius.circular(8),
              child: AspectRatio(
                aspectRatio: 4 / 3,
                child: Image.network(
                  obstacle.imageUrl,
                  fit: BoxFit.cover,
                  errorBuilder: (context, error, stackTrace) => Container(
                    color: Colors.black12,
                    alignment: Alignment.center,
                    child: const Icon(Icons.broken_image, size: 48),
                  ),
                ),
              ),
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: OutlinedButton.icon(
                    icon: const Icon(Icons.check_circle_outline),
                    label: const Text('치웠음'),
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
                  child: FilledButton.icon(
                    icon: const Icon(Icons.cleaning_services),
                    label: const Text('재청소'),
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
            if (_submitting)
              const Padding(
                padding: EdgeInsets.only(top: 12),
                child: LinearProgressIndicator(),
              ),
          ],
        ),
      ),
    );
  }
}
