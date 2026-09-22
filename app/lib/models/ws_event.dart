/// 서버(`/ws/app`)가 브로드캐스트하는 WebSocket 이벤트 envelope.
///
/// `{"type": "telemetry" | "obstacle_alert" | "obstacle_update" |
///   "robot_connected" | "robot_disconnected", "payload": {...}}`
class WsEvent {
  final String type;
  final Map<String, dynamic> payload;

  const WsEvent({required this.type, required this.payload});

  factory WsEvent.fromJson(Map<String, dynamic> json) {
    return WsEvent(
      type: json['type'] as String,
      payload: (json['payload'] as Map<String, dynamic>?) ?? const {},
    );
  }
}
