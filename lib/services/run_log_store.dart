import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../models/run_log.dart';

/// Keeps the run history, and the run currently in progress.
///
/// Both live on the phone rather than the robot. The app is what knows when
/// AUTO was pressed and when the operator started a manual recording, and the
/// history outlives any one connection to the Pi.
///
/// The IN-PROGRESS run is persisted too, not just the finished ones. A run can
/// last an hour, and Android is free to kill a backgrounded app at any point in
/// it; without this, the whole run would vanish. On the next launch the run is
/// picked back up where it left off.
class RunLogStore extends ChangeNotifier {
  static const _kRuns = 'runlog.runs';
  static const _kActiveMode = 'runlog.active.mode';
  static const _kActiveStart = 'runlog.active.start';
  static const _kActiveLat = 'runlog.active.lat';
  static const _kActiveLon = 'runlog.active.lon';

  /// How many runs are kept. Old ones fall off the end rather than growing
  /// without limit on a device that may never be cleared.
  static const int maxRuns = 200;

  List<RunLog> _runs = [];
  List<RunLog> get runs => List.unmodifiable(_runs);

  String? _activeMode; // 'AUTO' | 'MANUAL' | null
  DateTime? _activeStart;
  double? _activeLat, _activeLon;

  String? get activeMode => _activeMode;
  DateTime? get activeStart => _activeStart;
  bool get isRecording => _activeMode != null;

  /// How long the current run has been going, or null when none is.
  Duration? get elapsed =>
      _activeStart == null ? null : DateTime.now().difference(_activeStart!);

  Future<void> load() async {
    final p = await SharedPreferences.getInstance();
    _runs = RunLog.decodeList(p.getString(_kRuns));
    _activeMode = p.getString(_kActiveMode);
    final startMs = p.getInt(_kActiveStart);
    _activeStart =
        startMs == null ? null : DateTime.fromMillisecondsSinceEpoch(startMs);
    _activeLat = p.getDouble(_kActiveLat);
    _activeLon = p.getDouble(_kActiveLon);
    // A mode without a start time (or the reverse) is a half-written state
    // from a kill at exactly the wrong moment; treat it as no run.
    if (_activeMode == null || _activeStart == null) {
      _activeMode = null;
      _activeStart = null;
    }
    notifyListeners();
  }

  /// Begin recording. Ignored when a run is already in progress, so a double
  /// tap cannot silently discard the first one's start time.
  Future<void> start(String mode, {double? lat, double? lon}) async {
    if (_activeMode != null) return;
    _activeMode = mode;
    _activeStart = DateTime.now();
    _activeLat = lat;
    _activeLon = lon;
    final p = await SharedPreferences.getInstance();
    await p.setString(_kActiveMode, mode);
    await p.setInt(_kActiveStart, _activeStart!.millisecondsSinceEpoch);
    if (lat != null) await p.setDouble(_kActiveLat, lat);
    if (lon != null) await p.setDouble(_kActiveLon, lon);
    notifyListeners();
  }

  /// Finish the current run and file it. Returns the entry, or null if
  /// nothing was being recorded.
  Future<RunLog?> stop({double? lat, double? lon}) async {
    if (_activeMode == null || _activeStart == null) return null;
    final run = RunLog(
      mode: _activeMode!,
      start: _activeStart!,
      end: DateTime.now(),
      startLat: _activeLat,
      startLon: _activeLon,
      endLat: lat,
      endLon: lon,
    );
    _runs = [run, ..._runs];
    if (_runs.length > maxRuns) _runs = _runs.sublist(0, maxRuns);
    _activeMode = null;
    _activeStart = null;
    _activeLat = null;
    _activeLon = null;
    final p = await SharedPreferences.getInstance();
    await p.setString(_kRuns, RunLog.encodeList(_runs));
    await p.remove(_kActiveMode);
    await p.remove(_kActiveStart);
    await p.remove(_kActiveLat);
    await p.remove(_kActiveLon);
    notifyListeners();
    return run;
  }

  Future<void> clear() async {
    _runs = [];
    final p = await SharedPreferences.getInstance();
    await p.setString(_kRuns, RunLog.encodeList(_runs));
    notifyListeners();
  }
}
