import 'coordinate.dart';

/// 서버 `GET /api/map/latest` 응답 / WebSocket `map_update` payload.
///
/// 지도 PNG를 월드 좌표(ROS `/explore_map` 의 frame, 보통 `map`)와 맞추기 위한 정보다.
/// PNG는 위쪽이 +y 이고 1픽셀 = [resolution] m 이다.
///
///     px = (x - origin.x) / resolution
///     py = height - (y - origin.y) / resolution
class MapInfo {
  final String frameId;
  final double resolution;
  final int width;
  final int height;

  /// PNG 왼쪽 아래 모서리의 월드 좌표(m).
  final Coordinate origin;

  /// 로봇이 처음 움직이기 시작한 위치(도크). 화면의 기준점으로 쓴다.
  final Pose2D startPose;

  final String mapImageUrl;
  final int version;

  const MapInfo({
    required this.frameId,
    required this.resolution,
    required this.width,
    required this.height,
    required this.origin,
    required this.startPose,
    required this.mapImageUrl,
    required this.version,
  });

  /// 지도 한 변의 실제 크기(m).
  double get widthMeters => width * resolution;
  double get heightMeters => height * resolution;

  /// 서버가 만든 URL의 호스트는 `SERVER_PUBLIC_BASE_URL` 설정값이라 앱에 설정된 주소와
  /// 다를 수 있다. 경로만 취해 앱이 실제로 접속 중인 서버로 요청하고, `version`을 붙여
  /// 새 지도가 올라오면 캐시된 이전 이미지를 쓰지 않게 한다.
  String imageUrlFor(String httpBaseUrl) {
    final path = Uri.parse(mapImageUrl).path;
    return '$httpBaseUrl$path?v=$version';
  }

  /// 메타데이터(해상도/원점 등)가 없는 응답이면 null.
  static MapInfo? tryParse(Map<String, dynamic> json) {
    if (json['resolution'] == null ||
        json['width'] == null ||
        json['height'] == null ||
        json['origin'] == null) {
      return null;
    }
    return MapInfo(
      frameId: json['frame_id'] as String? ?? 'map',
      resolution: (json['resolution'] as num).toDouble(),
      width: (json['width'] as num).toInt(),
      height: (json['height'] as num).toInt(),
      origin: Coordinate.fromJson(json['origin'] as Map<String, dynamic>),
      startPose: json['start_pose'] != null
          ? Pose2D.fromJson(json['start_pose'] as Map<String, dynamic>)
          : const Pose2D(x: 0, y: 0),
      mapImageUrl: json['map_image_url'] as String,
      version: (json['version'] as num?)?.toInt() ?? 1,
    );
  }
}
