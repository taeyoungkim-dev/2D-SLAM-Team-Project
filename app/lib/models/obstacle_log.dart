import 'coordinate.dart';

/// AGENTS.md 3.1 Obstacle Log Schema.
class ObstacleLog {
  final String obstacleId;
  final String type;
  final Coordinate mapCoordinate;
  final String imageUrl;
  final String status; // uncleared | cleared | recleaning
  final DateTime? createdAt;
  final DateTime? updatedAt;

  const ObstacleLog({
    required this.obstacleId,
    required this.type,
    required this.mapCoordinate,
    required this.imageUrl,
    required this.status,
    this.createdAt,
    this.updatedAt,
  });

  bool get isCleared => status == 'cleared';

  factory ObstacleLog.fromJson(Map<String, dynamic> json) {
    return ObstacleLog(
      obstacleId: json['obstacle_id'] as String,
      type: json['type'] as String,
      mapCoordinate:
          Coordinate.fromJson(json['map_coordinate'] as Map<String, dynamic>),
      imageUrl: json['image_url'] as String,
      status: json['status'] as String,
      createdAt: json['created_at'] != null
          ? DateTime.tryParse(json['created_at'] as String)
          : null,
      updatedAt: json['updated_at'] != null
          ? DateTime.tryParse(json['updated_at'] as String)
          : null,
    );
  }

  ObstacleLog copyWith({String? status}) {
    return ObstacleLog(
      obstacleId: obstacleId,
      type: type,
      mapCoordinate: mapCoordinate,
      imageUrl: imageUrl,
      status: status ?? this.status,
      createdAt: createdAt,
      updatedAt: updatedAt,
    );
  }
}
