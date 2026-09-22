import '../core/fsm_state.dart';
import 'coordinate.dart';

/// AGENTS.md 3.2 Robot Telemetry Schema.
class RobotTelemetry {
  final RobotState currentState;
  final Pose2D currentPosition;
  final List<Coordinate> sweptPath;
  final int batteryLevel;
  final DateTime? updatedAt;

  const RobotTelemetry({
    required this.currentState,
    required this.currentPosition,
    required this.sweptPath,
    required this.batteryLevel,
    this.updatedAt,
  });

  factory RobotTelemetry.initial() {
    return const RobotTelemetry(
      currentState: RobotState.idle,
      currentPosition: Pose2D(x: 0, y: 0, theta: 0),
      sweptPath: [],
      batteryLevel: 100,
    );
  }

  factory RobotTelemetry.fromJson(Map<String, dynamic> json) {
    final rawPath = (json['swept_path'] as List<dynamic>?) ?? const [];
    return RobotTelemetry(
      currentState: robotStateFromApi(json['current_state'] as String),
      currentPosition:
          Pose2D.fromJson(json['current_position'] as Map<String, dynamic>),
      sweptPath: rawPath
          .map((e) => Coordinate.fromJson(e as Map<String, dynamic>))
          .toList(),
      batteryLevel: (json['battery_level'] as num?)?.toInt() ?? 0,
      updatedAt: json['updated_at'] != null
          ? DateTime.tryParse(json['updated_at'] as String)
          : null,
    );
  }
}
