import 'package:flutter/material.dart';

/// AGENTS.md 2장(FINITE_STATE_MACHINE)에 정의된 로봇 상태.
///
/// 서버는 이 값을 문자열(`IDLE`, `MAPPING`, ...)로 그대로 주고받으므로,
/// [RobotState.fromApi] / [RobotStateX.apiValue] 로 변환한다.
enum RobotState {
  idle,
  mapping,
  cleaning,
  evading,
  targetRecleaning,
  returning,
}

RobotState robotStateFromApi(String value) {
  switch (value) {
    case 'IDLE':
      return RobotState.idle;
    case 'MAPPING':
      return RobotState.mapping;
    case 'CLEANING':
      return RobotState.cleaning;
    case 'EVADING':
      return RobotState.evading;
    case 'TARGET_RECLEANING':
      return RobotState.targetRecleaning;
    case 'RETURNING':
      return RobotState.returning;
    default:
      return RobotState.idle;
  }
}

/// REQ-APP-05: 상단 상태 바에 표시할 아이콘/텍스트/색상.
extension RobotStateX on RobotState {
  String get apiValue {
    switch (this) {
      case RobotState.idle:
        return 'IDLE';
      case RobotState.mapping:
        return 'MAPPING';
      case RobotState.cleaning:
        return 'CLEANING';
      case RobotState.evading:
        return 'EVADING';
      case RobotState.targetRecleaning:
        return 'TARGET_RECLEANING';
      case RobotState.returning:
        return 'RETURNING';
    }
  }

  String get label {
    switch (this) {
      case RobotState.idle:
        return '대기 중';
      case RobotState.mapping:
        return '지도 생성 중';
      case RobotState.cleaning:
        return '청소 중';
      case RobotState.evading:
        return '장애물 회피 중';
      case RobotState.targetRecleaning:
        return '재청소 이동 중';
      case RobotState.returning:
        return '도크로 복귀 중';
    }
  }

  IconData get icon {
    switch (this) {
      case RobotState.idle:
        return Icons.pause_circle_outline;
      case RobotState.mapping:
        return Icons.map_outlined;
      case RobotState.cleaning:
        return Icons.cleaning_services;
      case RobotState.evading:
        return Icons.warning_amber_rounded;
      case RobotState.targetRecleaning:
        return Icons.my_location;
      case RobotState.returning:
        return Icons.home_outlined;
    }
  }

  Color get color {
    switch (this) {
      case RobotState.idle:
        return Colors.grey;
      case RobotState.mapping:
        return Colors.blueAccent;
      case RobotState.cleaning:
        return Colors.green;
      case RobotState.evading:
        return Colors.orange;
      case RobotState.targetRecleaning:
        return Colors.purple;
      case RobotState.returning:
        return Colors.blueGrey;
    }
  }
}
