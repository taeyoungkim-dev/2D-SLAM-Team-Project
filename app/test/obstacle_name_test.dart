import 'package:flutter_test/flutter_test.dart';
import 'package:robot_vacuum_app/models/obstacle_log.dart';

void main() {
  test('장애물 class 는 한국어 이름으로 보인다', () {
    expect(obstacleDisplayName('power_cable'), '전선');
    expect(obstacleDisplayName('cardboard_box'), '박스');
    expect(obstacleDisplayName('urine_puddle'), '반려견 소변');
    expect(obstacleDisplayName('unknown_thing'), 'unknown_thing');
  });
}
