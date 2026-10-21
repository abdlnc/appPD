import 'dart:convert';

/// What the robot reports on "R:" -- a position you set on the map, and
/// what is carrying it along as the robot moves (gps_node.py, YOU SET
/// WHERE IT IS).
///
/// Setting it is two taps: where the robot really is, then a point it is
/// facing. The robot then tracks its movement by lidar (SLAM) while the
/// lidar can see walls and houses, and by GPS corrected by the error it had
/// when the lidar cannot -- for [secsLeft] more seconds, after which it is
/// dropped and plain GPS takes over again.
class PositionRefStatus {
  /// OFF (plain GPS), FACE (position set, waiting for the facing tap),
  /// ON (set and being tracked).
  final String stage;

  /// What is tracking it right now: 'SLAM', 'GPS' (corrected), or null.
  final String? using;

  /// Seconds the corrected GPS is still trusted (only while [using] is GPS).
  final int? secsLeft;

  /// Whether lidar tracking was lined up at the taps. False means SLAM was
  /// not running then, and it is corrected GPS only.
  final bool lidar;

  /// The robot's last message about it, e.g. "GPS was 9 m off".
  final String note;

  const PositionRefStatus({
    required this.stage,
    this.using,
    this.secsLeft,
    this.lidar = false,
    this.note = '',
  });

  bool get active => stage == 'ON';
  bool get waitingForFacing => stage == 'FACE';
  bool get off => stage == 'OFF';

  /// One line for the map, or null when nothing is set.
  String? get summary {
    if (waitingForFacing) return 'Position set -- now tap where it faces';
    if (!active) return null;
    if (using == 'SLAM') return 'Your position, tracked by lidar';
    if (using == 'GPS') {
      final left = secsLeft != null ? ' (${secsLeft}s left)' : '';
      return 'Your position, corrected GPS$left';
    }
    return 'Your position';
  }

  static PositionRefStatus? parse(String json) {
    try {
      final d = jsonDecode(json);
      if (d is! Map) return null;
      return PositionRefStatus(
        stage: (d['stage'] as String?) ?? 'OFF',
        using: d['using'] as String?,
        secsLeft: (d['secs_left'] as num?)?.toInt(),
        lidar: d['lidar'] == true,
        note: (d['note'] as String?) ?? '',
      );
    } catch (_) {
      return null;
    }
  }
}
