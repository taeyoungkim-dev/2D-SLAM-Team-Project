import 'coordinate.dart';

/// 장애물 class(학습 모델의 이름) -> 앱에 표시하는 이름.
const Map<String, String> kObstacleNamesKo = {
  'power_cable': '전선',
  'cardboard_box': '박스',
  'urine_puddle': '반려견 소변',
};

String obstacleDisplayName(String type) => kObstacleNamesKo[type] ?? type;

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

  /// 앱에 보여 주는 장애물 이름(한국어). 모르는 class 는 원래 이름을 그대로 쓴다.
  String get displayName => obstacleDisplayName(type);

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
