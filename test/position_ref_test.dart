import 'package:flutter_test/flutter_test.dart';
import 'package:robot_controller/models/position_ref.dart';

/// "R:" -- the position you set on the map, as gps_node reports it
/// (publish_posref). The JSON below is what the robot sends.
void main() {
  test('nothing set: no summary, the map shows nothing extra', () {
    final s = PositionRefStatus.parse('{"stage":"OFF","using":null,'
        '"secs_left":null,"lidar":false,"note":""}')!;
    expect(s.off, isTrue);
    expect(s.summary, isNull);
  });

  test('after the first tap: waiting for the facing tap', () {
    final s = PositionRefStatus.parse('{"stage":"FACE","using":null,'
        '"secs_left":null,"lidar":false,"note":"robot position SET '
        '(GPS was 9 m off) -- now tap a point it is facing"}')!;
    expect(s.waitingForFacing, isTrue);
    expect(s.active, isFalse);
    expect(s.summary, contains('tap where it faces'));
    expect(s.note, contains('9 m off'));
  });

  test('tracked by lidar', () {
    final s = PositionRefStatus.parse('{"stage":"ON","using":"SLAM",'
        '"secs_left":null,"lidar":true,"note":""}')!;
    expect(s.active, isTrue);
    expect(s.lidar, isTrue);
    expect(s.summary, contains('lidar'));
  });

  test('corrected GPS says how long it has left', () {
    final s = PositionRefStatus.parse('{"stage":"ON","using":"GPS",'
        '"secs_left":42,"lidar":true,"note":""}')!;
    expect(s.using, 'GPS');
    expect(s.secsLeft, 42);
    expect(s.summary, 'Your position, corrected GPS (42s left)');
  });

  test('garbage is ignored, not thrown', () {
    expect(PositionRefStatus.parse('not json'), isNull);
    expect(PositionRefStatus.parse('[1,2]'), isNull);
    expect(PositionRefStatus.parse(''), isNull);
  });
}
