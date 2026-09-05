import 'dart:async'; // Timer + StreamSubscription (auto-reconnect)
import 'dart:convert'; // Added for Base64 decoding
import 'dart:math';
import 'dart:typed_data'; // Added for Uint8List
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter/foundation.dart';

/// A GPS location where the robot's Lidar reported a DANGER-level obstacle.
class ObstaclePin {
  final double lat;
  final double lon;
  final String zone; // front / front_l / front_r / left / right / back
  final double distanceM;
  final DateTime time;
  ObstaclePin({
    required this.lat,
    required this.lon,
    required this.zone,
    required this.distanceM,
    required this.time,
  });
}

class RobotSocket extends ChangeNotifier {
  WebSocketChannel? _channel;
  StreamSubscription? _sub; // so we can detach cleanly before reconnecting
  bool _isConnected = false;
  String _currentStatus = "Disconnected";

  // ---- auto-reconnect ----
  String? _lastIp; // last IP we were asked to connect to
  Timer? _reconnectTimer;
  bool _disposed = false;

  // LiDAR points
  List<Point<double>> _lidarPoints = [];
  List<Point<double>> get lidarPoints => _lidarPoints;

  // Camera Image Data
  Uint8List? _cameraImage;
  Uint8List? get cameraImage => _cameraImage;

  bool get isConnected => _isConnected;
  String get currentStatus => _currentStatus;
  String? get hostIp => _lastIp; // Pi IP, for building HTTP URLs (e.g. gallery)

  // ---- Obstacle warning (robot Lidar -> /obstacle -> "O:" over WebSocket) ----
  String _obstacleLevel = "CLEAR"; // CLEAR / CAUTION / DANGER
  String _obstacleZone = "";
  double _obstacleDist = 0.0;
  String get obstacleLevel => _obstacleLevel;
  String get obstacleZone => _obstacleZone;
  double get obstacleDist => _obstacleDist;

  /// True when the robot reports it is boxed in -- forward, reverse AND
  /// both turn directions all blocked, so its avoidance maneuver has no
  /// move left to make. Sent as an optional 4th field on the obstacle
  /// message ("DANGER|zone|dist|BLOCKED"), so older 3-field messages
  /// still parse fine.
  bool _pathBlocked = false;
  bool get pathBlocked => _pathBlocked;

  /// True when connected but the GPS has no fix right now. Uses the fix
  /// flag rather than lat/lon, because lat/lon keep their LAST known
  /// value after a fix is lost -- so checking those would wrongly report
  /// "have GPS" while the signal is actually gone.
  bool get noGpsSignal => _isConnected && _gpsFix < 1;

  // ---- Robot GPS position (gps_node -> /gps -> "G:" over WebSocket) ----
  double? _robotLat;
  double? _robotLon;
  double _robotHeading = 0.0; // degrees, compass bearing
  bool _hasHeading = false;
  int _gpsFix = 0;
  int _gpsSats = 0;
  int _gpsSatsView = 0; // satellites IN VIEW while still acquiring (no fix)
  double? get robotLat => _robotLat;
  double? get robotLon => _robotLon;
  double get robotHeading => _robotHeading;
  bool get hasHeading => _hasHeading;
  int get gpsFix => _gpsFix;
  int get gpsSats => _gpsSats;
  int get gpsSatsView => _gpsSatsView;
  bool get hasGps => _robotLat != null && _robotLon != null;

  // ---- SLAM map save ("SAVEMAP" -> "MAPSAVED:<name>" / "MAPERR:<reason>") ----
  bool _isSavingMap = false;
  String? _mapSaveResult; // saved map's name, on success
  String? _mapSaveError; // failure reason, on error
  bool get isSavingMap => _isSavingMap;
  String? get mapSaveResult => _mapSaveResult;
  String? get mapSaveError => _mapSaveError;

  // ---- SLAM map reset ("RESETMAP" -> "MAPRESET:ok" / "MAPRESETERR:<reason>")
  // Wipes the robot's LIVE mapped area (restarts slam_toolbox on the Pi).
  // Already-saved snapshots in the gallery's MAPS tab are NOT touched. ----
  bool _isResettingMap = false;
  bool _mapResetOk = false; // true once a reset succeeded (one-shot flag)
  String? _mapResetError; // failure reason, on error
  bool get isResettingMap => _isResettingMap;
  bool get mapResetOk => _mapResetOk;
  String? get mapResetError => _mapResetError;

  // ---- Obstacle pins (GPS location logged each time a DANGER obstacle is seen) ----
  final List<ObstaclePin> _obstaclePins = [];
  List<ObstaclePin> get obstaclePins => _obstaclePins;
  DateTime? _lastPinTime;
  static const Duration _pinCooldown = Duration(seconds: 4);
  static const int _maxPins = 300;

  void connect(String ipAddress) {
    _lastIp = ipAddress;
    _reconnectTimer?.cancel(); // stop any pending retry
    _sub?.cancel(); // detach old listener so its onDone can't clobber state
    try {
      _channel?.sink.close();
    } catch (_) {}

    try {
      final uri = Uri.parse('ws://$ipAddress:8765');
      _channel = WebSocketChannel.connect(uri);
      _isConnected = true;
      _currentStatus = "Connected";
      notifyListeners();

      _sub = _channel!.stream.listen(
        (message) => _handleMessage(message),
        onError: (error) => _onDropped(),
        onDone: () => _onDropped(),
        cancelOnError: true,
      );
    } catch (e) {
      _onDropped();
    }
  }

  /// Link dropped (error / closed / connect failed). Mark offline and schedule
  /// an automatic reconnect to the last IP.
  void _onDropped() {
    _isConnected = false;
    _currentStatus = "Disconnected";
    _obstacleLevel = "CLEAR";
    notifyListeners();
    _scheduleReconnect();
  }

  void _scheduleReconnect() {
    if (_disposed || _lastIp == null) return;
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 2), () {
      if (!_isConnected && !_disposed && _lastIp != null) {
        _currentStatus = "Reconnecting\u2026";
        notifyListeners();
        connect(_lastIp!);
      }
    });
  }

  /// Manual disconnect (stops auto-reconnect). Call from a "Disconnect" button
  /// if you ever add one; otherwise the app just keeps the link alive.
  void disconnect() {
    _lastIp = null;
    _reconnectTimer?.cancel();
    _sub?.cancel();
    try {
      _channel?.sink.close();
    } catch (_) {}
    _isConnected = false;
    _currentStatus = "Disconnected";
    notifyListeners();
  }

  @override
  void dispose() {
    _disposed = true;
    _reconnectTimer?.cancel();
    _sub?.cancel();
    try {
      _channel?.sink.close();
    } catch (_) {}
    super.dispose();
  }

  void _handleMessage(dynamic message) {
    if (message is String) {
      // 1. Camera Frame (C:)
      if (message.startsWith("C:")) {
        String base64String = message.substring(2);
        _cameraImage = base64Decode(base64String);
        notifyListeners();
      }
      // 2. Lidar Data (L:)
      else if (message.startsWith("L:")) {
        _parseLidarData(message.substring(2));
      }
      // 3. Obstacle Warning (O:)
      else if (message.startsWith("O:")) {
        _parseObstacle(message.substring(2));
      }
      // 4. GPS Position (G:)
      else if (message.startsWith("G:")) {
        _parseGps(message.substring(2));
      }
      // 5. SLAM map save result (MAPSAVED: / MAPERR:)
      else if (message.startsWith("MAPSAVED:")) {
        _isSavingMap = false;
        _mapSaveResult = message.substring("MAPSAVED:".length);
        _mapSaveError = null;
        notifyListeners();
      } else if (message.startsWith("MAPERR:")) {
        _isSavingMap = false;
        _mapSaveError = message.substring("MAPERR:".length);
        _mapSaveResult = null;
        notifyListeners();
      }
      // 6. SLAM map reset result (MAPRESET: / MAPRESETERR:)
      else if (message.startsWith("MAPRESET:")) {
        _isResettingMap = false;
        _mapResetOk = true;
        _mapResetError = null;
        _lidarPoints = []; // the live radar view is stale now too
        notifyListeners();
      } else if (message.startsWith("MAPRESETERR:")) {
        _isResettingMap = false;
        _mapResetError = message.substring("MAPRESETERR:".length);
        _mapResetOk = false;
        notifyListeners();
      }
    }
  }

  void _parseGps(String data) {
    // FIX:        "lat,lon,heading,speed_kmh,fix,sats"
    // ACQUIRING:  ",,,0.0,0,<sats_in_view>"   (empty lat/lon, no fix yet)
    List<String> p = data.split(',');
    if (p.length < 2) return;
    final lat = double.tryParse(p[0]);
    final lon = double.tryParse(p[1]);

    if (lat != null && lon != null) {
      // valid fix -> real position
      _robotLat = lat;
      _robotLon = lon;
      if (p.length >= 3 && p[2].trim().isNotEmpty) {
        final h = double.tryParse(p[2]);
        if (h != null) {
          _robotHeading = h;
          _hasHeading = true;
        }
      }
      if (p.length >= 5) _gpsFix = int.tryParse(p[4]) ?? _gpsFix;
      if (p.length >= 6) _gpsSats = int.tryParse(p[5]) ?? _gpsSats;
    } else {
      // acquiring (no fix yet) -> just track satellites in view for the UI
      _gpsFix = 0;
      if (p.length >= 6) _gpsSatsView = int.tryParse(p[5]) ?? _gpsSatsView;
    }
    notifyListeners();
  }

  void _parseObstacle(String data) {
    final String previousLevel = _obstacleLevel;
    List<String> parts = data.split('|');
    String level = parts[0].trim().toUpperCase();
    if (level == "CLEAR" || parts.length < 3) {
      _obstacleLevel = "CLEAR";
      _obstacleZone = "";
      _obstacleDist = 0.0;
      _pathBlocked = false;
    } else {
      _obstacleLevel = level;
      _obstacleZone = parts[1].trim();
      _obstacleDist = double.tryParse(parts[2].trim()) ?? 0.0;
      // optional 4th field: "BLOCKED" = no escape route at all
      _pathBlocked =
          parts.length >= 4 && parts[3].trim().toUpperCase() == "BLOCKED";
    }
    if (_obstacleLevel == "DANGER" && hasGps) {
      _maybeAddObstaclePin(previousLevel);
    }
    notifyListeners();
  }

  /// Drop a pin at the robot's current GPS position for a DANGER obstacle.
  /// Fires immediately on the CLEAR/CAUTION -> DANGER edge, and again on a
  /// cooldown while the robot lingers in DANGER, so a long stop near an
  /// obstacle doesn't spam one pin per Lidar scan.
  void _maybeAddObstaclePin(String previousLevel) {
    final now = DateTime.now();
    final risingEdge = previousLevel != "DANGER";
    final cooldownElapsed =
        _lastPinTime == null || now.difference(_lastPinTime!) >= _pinCooldown;
    if (!risingEdge && !cooldownElapsed) return;
    _lastPinTime = now;
    _obstaclePins.add(ObstaclePin(
      lat: _robotLat!,
      lon: _robotLon!,
      zone: _obstacleZone,
      distanceM: _obstacleDist,
      time: now,
    ));
    if (_obstaclePins.length > _maxPins) {
      _obstaclePins.removeRange(0, _obstaclePins.length - _maxPins);
    }
  }

  void clearObstaclePins() {
    _obstaclePins.clear();
    notifyListeners();
  }

  void _parseLidarData(String data) {
    List<Point<double>> newPoints = [];
    List<String> pairs = data.split(';');
    for (var pair in pairs) {
      var parts = pair.split(',');
      if (parts.length == 2) {
        try {
          double angle = double.parse(parts[0]);
          double dist = double.parse(parts[1]);
          newPoints.add(Point(angle, dist));
        } catch (e) {}
      }
    }
    _lidarPoints = newPoints;
    notifyListeners();
  }

  void sendCommand(String command) {
    if (_channel != null && _isConnected) {
      _channel!.sink.add(command);
    }
  }

  // ---- Waypoint commands (for GPS navigation) ----
  void sendWaypoint(double lat, double lon) {
    sendCommand("WP:${lat.toStringAsFixed(7)},${lon.toStringAsFixed(7)}");
  }

  void clearWaypoint() {
    sendCommand("WP:CLEAR");
  }

  // ---- SLAM map save (see _handleMessage for the MAPSAVED:/MAPERR: reply) ----
  void saveSlamMap() {
    _isSavingMap = true;
    _mapSaveResult = null;
    _mapSaveError = null;
    notifyListeners();
    sendCommand("SAVEMAP");
  }

  /// Call once the result has been shown (e.g. in a SnackBar) so it doesn't
  /// get shown again on the next rebuild.
  void clearMapSaveResult() {
    _mapSaveResult = null;
    _mapSaveError = null;
    notifyListeners();
  }

  // ---- SLAM map reset (see _handleMessage for the MAPRESET:/MAPRESETERR: reply)
  /// Wipes the robot's live mapped area and starts mapping over from scratch.
  /// Does NOT delete any snapshots already saved to the gallery's MAPS tab.
  void resetSlamMap() {
    _isResettingMap = true;
    _mapResetOk = false;
    _mapResetError = null;
    notifyListeners();
    sendCommand("RESETMAP");
  }

  /// Call once the reset result has been shown, so it doesn't re-show.
  void clearMapResetResult() {
    _mapResetOk = false;
    _mapResetError = null;
    notifyListeners();
  }
}