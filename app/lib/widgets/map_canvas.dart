import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../models/coordinate.dart';
import '../models/obstacle_log.dart';
import '../models/robot_telemetry.dart';

/// REQ-APP-01: 로봇의 위치와 청소 궤적(Swept Path)을 2D 캔버스에 그린다.
/// REQ-APP-02/03: 장애물 핀을 지도 위에 표시하고, 탭하면 상세 정보를 보여준다.
///
/// 좌표계는 ROS 맵 좌표(미터, x: 오른쪽+, y: 위쪽+)를 그대로 쓰고, 화면
/// 중앙을 원점으로 두어 로봇이 어느 방향으로 가든 화면 안에서 보이게 한다.
class MapCanvas extends StatelessWidget {
  const MapCanvas({
    super.key,
    required this.telemetry,
    required this.obstacles,
    this.onObstacleTap,
    this.pixelsPerMeter = 40.0,
  });

  final RobotTelemetry telemetry;
  final List<ObstacleLog> obstacles;
  final ValueChanged<ObstacleLog>? onObstacleTap;
  final double pixelsPerMeter;

  Offset _worldToScreen(Size size, Coordinate coord) {
    final center = Offset(size.width / 2, size.height / 2);
    return Offset(
      center.dx + coord.x * pixelsPerMeter,
      center.dy - coord.y * pixelsPerMeter, // y축 반전 (화면은 아래로 증가)
    );
  }

  void _handleTapUp(TapUpDetails details, Size size) {
    if (onObstacleTap == null) return;
    const hitRadius = 22.0;
    ObstacleLog? nearest;
    double bestDistance = double.infinity;
    for (final obstacle in obstacles) {
      final screenPos = _worldToScreen(size, obstacle.mapCoordinate);
      final distance = (screenPos - details.localPosition).distance;
      if (distance <= hitRadius && distance < bestDistance) {
        nearest = obstacle;
        bestDistance = distance;
      }
    }
    if (nearest != null) {
      onObstacleTap!(nearest);
    }
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        return GestureDetector(
          onTapUp: (details) => _handleTapUp(details, size),
          child: CustomPaint(
            size: size,
            painter: _MapPainter(
              telemetry: telemetry,
              obstacles: obstacles,
              pixelsPerMeter: pixelsPerMeter,
              worldToScreen: _worldToScreen,
            ),
          ),
        );
      },
    );
  }
}

class _MapPainter extends CustomPainter {
  _MapPainter({
    required this.telemetry,
    required this.obstacles,
    required this.pixelsPerMeter,
    required this.worldToScreen,
  });

  final RobotTelemetry telemetry;
  final List<ObstacleLog> obstacles;
  final double pixelsPerMeter;
  final Offset Function(Size size, Coordinate coord) worldToScreen;

  @override
  void paint(Canvas canvas, Size size) {
    _drawBackground(canvas, size);
    _drawGrid(canvas, size);
    _drawSweptPath(canvas, size);
    _drawObstacles(canvas, size);
    _drawRobot(canvas, size);
  }

  void _drawBackground(Canvas canvas, Size size) {
    final paint = Paint()..color = const Color(0xFFF5F5F5);
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), paint);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.withValues(alpha: 0.25)
      ..strokeWidth = 1;
    const gridMeters = 1.0;
    final step = gridMeters * pixelsPerMeter;

    final center = Offset(size.width / 2, size.height / 2);
    for (double x = center.dx % step; x < size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = center.dy % step; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }

    final axisPaint = Paint()
      ..color = Colors.grey.withValues(alpha: 0.6)
      ..strokeWidth = 1.5;
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), axisPaint);
    canvas.drawLine(Offset(center.dx, 0), Offset(center.dx, size.height), axisPaint);
  }

  void _drawSweptPath(Canvas canvas, Size size) {
    if (telemetry.sweptPath.length < 2) return;
    final paint = Paint()
      ..color = Colors.blueAccent
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    final path = Path();
    final first = worldToScreen(size, telemetry.sweptPath.first);
    path.moveTo(first.dx, first.dy);
    for (final coord in telemetry.sweptPath.skip(1)) {
      final point = worldToScreen(size, coord);
      path.lineTo(point.dx, point.dy);
    }
    canvas.drawPath(path, paint);
  }

  void _drawObstacles(Canvas canvas, Size size) {
    for (final obstacle in obstacles) {
      final center = worldToScreen(size, obstacle.mapCoordinate);
      final color = switch (obstacle.status) {
        'cleared' => Colors.green,
        'recleaning' => Colors.purple,
        _ => Colors.redAccent,
      };
      final fill = Paint()..color = color;
      final border = Paint()
        ..color = Colors.white
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2;
      canvas.drawCircle(center, 10, fill);
      canvas.drawCircle(center, 10, border);

      final textPainter = TextPainter(
        text: const TextSpan(
          text: '!',
          style: TextStyle(
            color: Colors.white,
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      textPainter.paint(
        canvas,
        center - Offset(textPainter.width / 2, textPainter.height / 2),
      );
    }
  }

  void _drawRobot(Canvas canvas, Size size) {
    final center = worldToScreen(size, telemetry.currentPosition);
    final bodyPaint = Paint()..color = Colors.deepOrange;
    canvas.drawCircle(center, 12, bodyPaint);

    // 로봇의 진행 방향(theta)을 화살표로 표시.
    final theta = telemetry.currentPosition.theta;
    final direction = Offset(
      center.dx + 18 * math.cos(theta),
      center.dy - 18 * math.sin(theta),
    );
    final headingPaint = Paint()
      ..color = Colors.deepOrange
      ..strokeWidth = 3;
    canvas.drawLine(center, direction, headingPaint);
  }

  @override
  bool shouldRepaint(covariant _MapPainter oldDelegate) {
    return oldDelegate.telemetry != telemetry ||
        oldDelegate.obstacles != obstacles;
  }
}
