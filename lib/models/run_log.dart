import 'dart:convert';

import 'geo_image.dart' show formatCoord, metersBetween;

/// One recorded run: from the moment driving started to the moment it stopped.
///
/// AUTO runs are recorded automatically, bracketed by the START/STOP AUTO
/// button. A MANUAL run is recorded only when the operator asks for it, from
/// the record button that appears with the developer joystick -- manual
/// driving has no natural start and end the app can detect, so the person
/// marks them.
///
/// GPS is optional at both ends and recorded independently: a run that
/// started indoors and finished outside keeps the end position rather than
/// throwing both away.
class RunLog {
  final String mode; // 'AUTO' | 'MANUAL'
  final DateTime start;
  final DateTime end;
  final double? startLat;
  final double? startLon;
  final double? endLat;
  final double? endLon;

  /// Ground distance covered during the run, summed from the GPS fixes as
  /// they arrived. Null when the run had no usable GPS.
  ///
  /// Unlike [displacementM] this follows the route, so a run that returns to
  /// its start still reports the ground it covered.
  final double? distanceM;

  const RunLog({
    required this.mode,
    required this.start,
    required this.end,
    this.startLat,
    this.startLon,
    this.endLat,
    this.endLon,
    this.distanceM,
  });

  Duration get duration => end.difference(start);

  bool get hasDistance => distanceM != null;

  /// "42.3 m" below a kilometre, "1.24 km" above -- metres stop being
  /// readable once a run covers a field.
  String get distanceLabel {
    final d = distanceM;
    if (d == null) return 'no GPS';
    return d >= 1000 ? '${(d / 1000).toStringAsFixed(2)} km'
                     : '${d.toStringAsFixed(1)} m';
  }

  bool get hasStartFix => startLat != null && startLon != null;
  bool get hasEndFix => endLat != null && endLon != null;

  /// Straight-line distance between where the run began and where it ended,
  /// or null unless BOTH ends have a fix.
  ///
  /// Deliberately not called "distance travelled": the robot's actual path is
  /// longer than this whenever it turned, and this says nothing about the
  /// route between the two points. A run that returns to its start reads 0.
  double? get displacementM => (hasStartFix && hasEndFix)
      ? metersBetween(startLat!, startLon!, endLat!, endLon!)
      : null;

  /// "1h 04m 12s", "4m 12s", "12s" -- the leading units are dropped when
  /// they would be zero, so a short run is not padded out with "0h 00m".
  String get durationLabel {
    final s = duration.inSeconds;
    final h = s ~/ 3600, m = (s % 3600) ~/ 60, sec = s % 60;
    if (h > 0) {
      return '${h}h ${m.toString().padLeft(2, '0')}m '
          '${sec.toString().padLeft(2, '0')}s';
    }
    if (m > 0) return '${m}m ${sec.toString().padLeft(2, '0')}s';
    return '${sec}s';
  }

  String coordLabel(double? lat, double? lon) =>
      (lat == null || lon == null) ? 'no GPS' : '${formatCoord(lat)}, ${formatCoord(lon)}';

  String get startCoordLabel => coordLabel(startLat, startLon);
  String get endCoordLabel => coordLabel(endLat, endLon);

  Map<String, dynamic> toJson() => {
        'mode': mode,
        'start': start.millisecondsSinceEpoch,
        'end': end.millisecondsSinceEpoch,
        if (startLat != null) 'slat': startLat,
        if (startLon != null) 'slon': startLon,
        if (endLat != null) 'elat': endLat,
        if (endLon != null) 'elon': endLon,
        if (distanceM != null) 'dist': distanceM,
      };

  static RunLog? fromJson(Map<String, dynamic> j) {
    final mode = j['mode'];
    final s = j['start'], e = j['end'];
    if (mode is! String || s is! int || e is! int) return null;
    double? d(String k) => (j[k] as num?)?.toDouble();
    return RunLog(
      mode: mode,
      start: DateTime.fromMillisecondsSinceEpoch(s),
      end: DateTime.fromMillisecondsSinceEpoch(e),
      startLat: d('slat'),
      startLon: d('slon'),
      endLat: d('elat'),
      endLon: d('elon'),
      distanceM: d('dist'),
    );
  }

  static String encodeList(List<RunLog> runs) =>
      jsonEncode(runs.map((r) => r.toJson()).toList());

  /// Decodes a stored list, skipping anything malformed rather than throwing.
  /// A corrupted entry costs one log line, not the whole history.
  static List<RunLog> decodeList(String? raw) {
    if (raw == null || raw.isEmpty) return [];
    try {
      final data = jsonDecode(raw);
      if (data is! List) return [];
      return [
        for (final e in data)
          if (e is Map<String, dynamic>)
            if (fromJson(e) case final r?) r,
      ];
    } catch (_) {
      return [];
    }
  }
}
