import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../models/geo_image.dart' show metersBetween;
import '../models/run_log.dart';
import 'robot_socket.dart';

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
  static const _kActiveDist = 'runlog.active.dist';

  /// Movement below this between two fixes is treated as GPS noise and not
  /// added. A stationary receiver wanders by a metre or two, and over an hour
  /// that noise alone would invent hundreds of metres of "distance".
  static const double minStepM = 1.0;

  /// A jump further than this between consecutive fixes is a GPS spike, not
  /// the robot. At any speed this machine drives, it cannot cover 40m between
  /// two updates.
  static const double maxJumpM = 40.0;

  /// How many runs are kept. Old ones fall off the end rather than growing
  /// without limit on a device that may never be cleared.
  static const int maxRuns = 200;

  List<RunLog> _runs = [];
  List<RunLog> get runs => List.unmodifiable(_runs);

  String? _activeMode; // 'AUTO' | 'MANUAL' | null
  DateTime? _activeStart;
  double? _activeLat, _activeLon;

  // Distance accumulated so far in the active run, and the last fix it was
  // measured from. Null distance means no usable GPS has arrived yet, which
  // is what distinguishes "didn't move" from "couldn't tell".
  double? _activeDist;
  double? _lastLat, _lastLon;

  RobotSocket? _socket;

  String? get activeMode => _activeMode;
  DateTime? get activeStart => _activeStart;
  bool get isRecording => _activeMode != null;

  /// How long the current run has been going, or null when none is.
  Duration? get elapsed =>
      _activeStart == null ? null : DateTime.now().difference(_activeStart!);

  /// Follow a socket's GPS so distance accrues wherever the operator is in
  /// the app -- the control screen does not have to be on top for a run to
  /// keep measuring. Idempotent: re-binding the same socket does nothing.
  void bind(RobotSocket socket) {
    if (identical(_socket, socket)) return;
    _socket?.removeListener(_onSocket);
    _socket = socket;
    _socket!.addListener(_onSocket);
  }

  void _onSocket() {
    final s = _socket;
    if (s == null || _activeMode == null) return;
    if (!s.hasGps || s.robotLat == null || s.robotLon == null) return;
    noteFix(s.robotLat!, s.robotLon!);
  }

  /// Add one GPS fix to the active run's distance.
  ///
  /// Public so it can be tested without a socket; the filtering is the part
  /// worth testing, since it decides what counts as the robot moving.
  void noteFix(double lat, double lon) {
    if (_activeMode == null) return;
    if (_lastLat == null || _lastLon == null) {
      // First usable fix of the run: a starting point, no distance yet.
      _lastLat = lat;
      _lastLon = lon;
      _activeDist ??= 0.0;
      _activeLat ??= lat;
      _activeLon ??= lon;
      return;
    }
    final step = metersBetween(_lastLat!, _lastLon!, lat, lon);
    if (step < minStepM || step > maxJumpM) return;
    _activeDist = (_activeDist ?? 0.0) + step;
    _lastLat = lat;
    _lastLon = lon;
    _persistActive();
  }

  Future<void> _persistActive() async {
    // Read the value BEFORE awaiting. Reading it after would race with
    // stop(), which clears it: a fix arriving as the operator ends a run
    // would then dereference null.
    final d = _activeDist;
    if (d == null) return;
    final p = await SharedPreferences.getInstance();
    await p.setDouble(_kActiveDist, d);
  }

  @override
  void dispose() {
    _socket?.removeListener(_onSocket);
    super.dispose();
  }

  Future<void> load() async {
    final p = await SharedPreferences.getInstance();
    _runs = RunLog.decodeList(p.getString(_kRuns));
    _activeMode = p.getString(_kActiveMode);
    final startMs = p.getInt(_kActiveStart);
    _activeStart =
        startMs == null ? null : DateTime.fromMillisecondsSinceEpoch(startMs);
    _activeLat = p.getDouble(_kActiveLat);
    _activeLon = p.getDouble(_kActiveLon);
    _activeDist = p.getDouble(_kActiveDist);
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
    _activeDist = (lat != null && lon != null) ? 0.0 : null;
    _lastLat = lat;
    _lastLon = lon;
    final p = await SharedPreferences.getInstance();
    await p.setString(_kActiveMode, mode);
    await p.setInt(_kActiveStart, _activeStart!.millisecondsSinceEpoch);
    if (lat != null) await p.setDouble(_kActiveLat, lat);
    if (lon != null) await p.setDouble(_kActiveLon, lon);
    if (_activeDist != null) await p.setDouble(_kActiveDist, _activeDist!);
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
      distanceM: _activeDist,
    );
    _runs = [run, ..._runs];
    if (_runs.length > maxRuns) _runs = _runs.sublist(0, maxRuns);
    _activeMode = null;
    _activeStart = null;
    _activeLat = null;
    _activeLon = null;
    _activeDist = null;
    _lastLat = null;
    _lastLon = null;
    final p = await SharedPreferences.getInstance();
    await p.setString(_kRuns, RunLog.encodeList(_runs));
    await p.remove(_kActiveMode);
    await p.remove(_kActiveStart);
    await p.remove(_kActiveLat);
    await p.remove(_kActiveLon);
    await p.remove(_kActiveDist);
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
