/// AGENTS.md 의 `map_coordinate` / `swept_path` 항목에 대응하는 단순 좌표.
class Coordinate {
  final double x;
  final double y;

  const Coordinate({required this.x, required this.y});

  factory Coordinate.fromJson(Map<String, dynamic> json) {
    return Coordinate(
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
    );
  }

  Map<String, dynamic> toJson() => {'x': x, 'y': y};

  @override
  String toString() => 'Coordinate(x: $x, y: $y)';
}

/// `current_position` 처럼 방향(theta)이 포함된 2D 포즈.
class Pose2D extends Coordinate {
  final double theta;

  const Pose2D({required super.x, required super.y, this.theta = 0.0});

  factory Pose2D.fromJson(Map<String, dynamic> json) {
    return Pose2D(
      x: (json['x'] as num).toDouble(),
      y: (json['y'] as num).toDouble(),
      theta: (json['theta'] as num?)?.toDouble() ?? 0.0,
    );
  }

  @override
  Map<String, dynamic> toJson() => {'x': x, 'y': y, 'theta': theta};
}
