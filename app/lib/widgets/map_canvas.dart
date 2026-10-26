import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

import '../models/coordinate.dart';
import '../models/map_info.dart';
import '../models/obstacle_log.dart';
import '../models/robot_telemetry.dart';

/// REQ-APP-01: 로봇의 위치와 청소 궤적(Swept Path)을 2D 캔버스에 그린다.
/// REQ-APP-02/03: 장애물 핀을 지도 위에 표시하고, 탭하면 상세 정보를 보여준다.
///
/// 좌표는 ROS 맵 좌표(미터, x: 오른쪽+, y: 위쪽+)를 그대로 쓴다(서버가 `/explore_map`
/// 의 frame 기준으로 보낸다). 기본 화면 중앙에는 [mapInfo]의 `start_pose`(로봇이 처음
/// 움직이기 시작한 위치)를 둔다. 지도가 없으면 맵 좌표 (0, 0)이 중앙이다.
///
/// 지도 이미지는 서버 PNG([mapImageBytes])를 [MapInfo]의 해상도/원점으로 배치한다:
/// 왼쪽 위 모서리 = (origin.x, origin.y + height * resolution).
/// [mapInfo]와 [mapImageBytes]는 반드시 같은 버전의 쌍이어야 한다.
///
/// 확대/축소: 두 손가락 핀치, 마우스 휠, 오른쪽 아래 +/- 버튼. 한 손가락 드래그로 이동하고
/// 가운데 버튼으로 처음 보기(배율 1배, 시작 위치 중앙)로 돌아간다.
class MapCanvas extends StatefulWidget {
  const MapCanvas({
    super.key,
    required this.telemetry,
    required this.obstacles,
    this.mapInfo,
    this.mapImageBytes,
    this.onObstacleTap,
    this.pixelsPerMeter = 40.0,
  });

  final RobotTelemetry telemetry;
  final List<ObstacleLog> obstacles;
  final MapInfo? mapInfo;
  final Uint8List? mapImageBytes;
  final ValueChanged<ObstacleLog>? onObstacleTap;

  /// 배율 1배일 때 1m 가 차지하는 화면 픽셀 수.
  final double pixelsPerMeter;

  @override
  State<MapCanvas> createState() => _MapCanvasState();
}

class _MapCanvasState extends State<MapCanvas> {
  static const double _minZoom = 0.25;
  static const double _maxZoom = 8.0;
  static const double _buttonZoomFactor = 1.4;

  double _zoom = 1.0;

  /// 화면 중앙(= 기준점)에서 화면상으로 얼마나 밀렸는지 (픽셀).
  Offset _pan = Offset.zero;
  Size _size = Size.zero;

  // 제스처 시작 시점의 값. 핀치 중심(focal point) 아래의 월드 점을 고정하는 데 쓴다.
  double _gestureStartZoom = 1.0;
  Offset _gestureWorldAtFocal = Offset.zero; // 중앙 기준, 배율 1배 픽셀 단위

  double get _ppm => widget.pixelsPerMeter * _zoom;

  Coordinate get _anchor =>
      widget.mapInfo?.startPose ?? const Coordinate(x: 0, y: 0);

  Offset _worldToScreen(Size size, Coordinate coord) {
    final center = Offset(size.width / 2, size.height / 2) + _pan;
    final anchor = _anchor;
    return Offset(
      center.dx + (coord.x - anchor.x) * _ppm,
      center.dy - (coord.y - anchor.y) * _ppm, // y축 반전 (화면은 아래로 증가)
    );
  }

  // -- 확대/축소/이동 -----------------------------------------------------
  double _clampZoom(double zoom) => zoom.clamp(_minZoom, _maxZoom).toDouble();

  void _handleScaleStart(ScaleStartDetails details) {
    _gestureStartZoom = _zoom;
    final center = Offset(_size.width / 2, _size.height / 2);
    _gestureWorldAtFocal =
        (details.localFocalPoint - center - _pan) / _gestureStartZoom;
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    final center = Offset(_size.width / 2, _size.height / 2);
    final zoom = _clampZoom(_gestureStartZoom * details.scale);
    setState(() {
      _zoom = zoom;
      // 손가락 아래의 지도 점이 계속 손가락 아래에 있도록 이동량을 다시 계산한다.
      _pan = details.localFocalPoint - center - _gestureWorldAtFocal * zoom;
    });
  }

  /// 화면 중앙을 기준으로 배율을 [factor]배 한다.
  void _zoomBy(double factor) {
    final zoom = _clampZoom(_zoom * factor);
    setState(() {
      _pan = _pan * (zoom / _zoom);
      _zoom = zoom;
    });
  }

  void _zoomAt(Offset focal, double factor) {
    final center = Offset(_size.width / 2, _size.height / 2);
    final zoom = _clampZoom(_zoom * factor);
    final worldAtFocal = (focal - center - _pan) / _zoom;
    setState(() {
      _zoom = zoom;
      _pan = focal - center - worldAtFocal * zoom;
    });
  }

  void _resetView() {
    setState(() {
      _zoom = 1.0;
      _pan = Offset.zero;
    });
  }

  void _handlePointerSignal(PointerSignalEvent event) {
    if (event is PointerScrollEvent) {
      // 위로 굴리면 확대.
      _zoomAt(event.localPosition, event.scrollDelta.dy < 0 ? 1.15 : 1 / 1.15);
    }
  }

  // -- 장애물 탭 ------------------------------------------------------------
  void _handleTapUp(TapUpDetails details) {
    final onTap = widget.onObstacleTap;
    if (onTap == null) return;
    const hitRadius = 22.0;
    ObstacleLog? nearest;
    double bestDistance = double.infinity;
    for (final obstacle in widget.obstacles) {
      final screenPos = _worldToScreen(_size, obstacle.mapCoordinate);
      final distance = (screenPos - details.localPosition).distance;
      if (distance <= hitRadius && distance < bestDistance) {
        nearest = obstacle;
        bestDistance = distance;
      }
    }
    if (nearest != null) {
      onTap(nearest);
    }
  }

  /// 서버 지도 PNG를 월드 좌표에 맞춰 배치한다.
  Widget _buildMapImage(Size size) {
    final info = widget.mapInfo;
    final bytes = widget.mapImageBytes;
    if (info == null || bytes == null) return const SizedBox.shrink();

    final topLeft = _worldToScreen(
      size,
      Coordinate(x: info.origin.x, y: info.origin.y + info.heightMeters),
    );
    return Positioned(
      left: topLeft.dx,
      top: topLeft.dy,
      width: info.widthMeters * _ppm,
      height: info.heightMeters * _ppm,
      child: Image.memory(
        bytes,
        fit: BoxFit.fill,
        filterQuality: FilterQuality.none, // 격자 셀 경계를 흐리지 않는다.
        gaplessPlayback: true, // 새 지도로 바뀌는 순간에도 이전 지도를 유지한다.
        errorBuilder: (_, __, ___) => const SizedBox.shrink(),
      ),
    );
  }

  Widget _buildZoomControls() {
    final percent = (_zoom * 100).round();
    return Positioned(
      right: 12,
      bottom: 12,
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          FloatingActionButton.small(
            heroTag: null,
            tooltip: '확대',
            onPressed: _zoom < _maxZoom ? () => _zoomBy(_buttonZoomFactor) : null,
            child: const Icon(Icons.add),
          ),
          const SizedBox(height: 8),
          FloatingActionButton.small(
            heroTag: null,
            tooltip: '처음 보기 ($percent%)',
            onPressed: _resetView,
            child: const Icon(Icons.center_focus_strong),
          ),
          const SizedBox(height: 8),
          FloatingActionButton.small(
            heroTag: null,
            tooltip: '축소',
            onPressed:
                _zoom > _minZoom ? () => _zoomBy(1 / _buttonZoomFactor) : null,
            child: const Icon(Icons.remove),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        _size = size;
        final painter = _MapPainter(
          telemetry: widget.telemetry,
          obstacles: widget.obstacles,
          pixelsPerMeter: _ppm,
          pan: _pan,
          worldToScreen: _worldToScreen,
          anchor: _anchor,
        );
        return Stack(
          children: [
            Positioned.fill(
              child: Listener(
                onPointerSignal: _handlePointerSignal,
                child: GestureDetector(
                  onTapUp: _handleTapUp,
                  onScaleStart: _handleScaleStart,
                  onScaleUpdate: _handleScaleUpdate,
                  child: SizedBox.fromSize(
                    size: size,
                    child: CustomPaint(
                      painter: _BackgroundPainter(),
                      // 배경 -> 지도 이미지(child) -> 격자/궤적/장애물/로봇 순으로 그린다.
                      child: Stack(
                        fit: StackFit.expand,
                        clipBehavior: Clip.hardEdge,
                        children: [_buildMapImage(size)],
                      ),
                      foregroundPainter: painter,
                    ),
                  ),
                ),
              ),
            ),
            _buildZoomControls(),
          ],
        );
      },
    );
  }
}

class _BackgroundPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = const Color(0xFFF5F5F5),
    );
  }

  @override
  bool shouldRepaint(covariant _BackgroundPainter oldDelegate) => false;
}

class _MapPainter extends CustomPainter {
  _MapPainter({
    required this.telemetry,
    required this.obstacles,
    required this.pixelsPerMeter,
    required this.pan,
    required this.worldToScreen,
    required this.anchor,
  });

  final RobotTelemetry telemetry;
  final List<ObstacleLog> obstacles;
  final double pixelsPerMeter;
  final Offset pan;
  final Offset Function(Size size, Coordinate coord) worldToScreen;

  /// 배율 1배일 때 화면 중앙에 놓이는 기준점 (지도의 start_pose).
  final Coordinate anchor;

  @override
  void paint(Canvas canvas, Size size) {
    _drawGrid(canvas, size);
    _drawStartPoint(canvas, size);
    _drawSweptPath(canvas, size);
    _drawObstacles(canvas, size);
    _drawRobot(canvas, size);
  }

  void _drawGrid(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.withValues(alpha: 0.25)
      ..strokeWidth = 1;
    // 많이 축소하면 1m 격자가 너무 촘촘해지므로 간격을 키운다.
    var gridMeters = 1.0;
    while (gridMeters * pixelsPerMeter < 16) {
      gridMeters *= 2;
    }
    final step = gridMeters * pixelsPerMeter;

    // 격자와 축은 기준점(start_pose)을 지난다.
    final origin = Offset(size.width / 2, size.height / 2) + pan;
    for (double x = origin.dx % step; x < size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = origin.dy % step; y < size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }

    final axisPaint = Paint()
      ..color = Colors.grey.withValues(alpha: 0.6)
      ..strokeWidth = 1.5;
    canvas.drawLine(Offset(0, origin.dy), Offset(size.width, origin.dy), axisPaint);
    canvas.drawLine(Offset(origin.dx, 0), Offset(origin.dx, size.height), axisPaint);
  }

  void _drawStartPoint(Canvas canvas, Size size) {
    final point = worldToScreen(size, anchor);
    final paint = Paint()
      ..color = Colors.blueGrey
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    canvas.drawCircle(point, 7, paint);
    canvas.drawCircle(point, 2, Paint()..color = Colors.blueGrey);
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
        oldDelegate.obstacles != obstacles ||
        oldDelegate.pixelsPerMeter != pixelsPerMeter ||
        oldDelegate.pan != pan ||
        oldDelegate.anchor.x != anchor.x ||
        oldDelegate.anchor.y != anchor.y;
  }
}
