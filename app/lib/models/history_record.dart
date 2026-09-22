/// REQ-APP-04 (P1): 청소 완료 기록.
class HistoryRecord {
  final int id;
  final String? mapImageUrl;
  final int durationSec;
  final int obstaclesFound;
  final DateTime startedAt;
  final DateTime finishedAt;
  final String? note;

  const HistoryRecord({
    required this.id,
    required this.durationSec,
    required this.obstaclesFound,
    required this.startedAt,
    required this.finishedAt,
    this.mapImageUrl,
    this.note,
  });

  factory HistoryRecord.fromJson(Map<String, dynamic> json) {
    return HistoryRecord(
      id: (json['id'] as num).toInt(),
      mapImageUrl: json['map_image_url'] as String?,
      durationSec: (json['duration_sec'] as num).toInt(),
      obstaclesFound: (json['obstacles_found'] as num?)?.toInt() ?? 0,
      startedAt: DateTime.parse(json['started_at'] as String),
      finishedAt: DateTime.parse(json['finished_at'] as String),
      note: json['note'] as String?,
    );
  }

  String get durationLabel {
    final minutes = durationSec ~/ 60;
    final seconds = durationSec % 60;
    return '$minutes분 $seconds초';
  }
}
