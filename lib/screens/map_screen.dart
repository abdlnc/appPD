import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';

/// GPS map (2.1 visualization) + interactive waypoint navigation (2.2).
/// - GO starts navigation and shows a live NAVIGATING state + STOP button.
/// - The map FOLLOWS the robot so the arrow always stays on the real GPS.
class MapScreen extends StatefulWidget {
  const MapScreen({super.key});
  @override
  State<MapScreen> createState() => _MapScreenState();
}

class _MapScreenState extends State<MapScreen> {
  final MapController _map = MapController();
  LatLng? _waypoint;
  bool _navigating = false; // GO pressed -> actively driving to waypoint
  bool _arrived = false; // reached the waypoint
  bool _follow = true; // map auto-centers on the robot
  bool _satellite = false;
  bool _firstFix = false;

  // --- Traversed-path trail ("dinaanan ng robot") ---
  final List<LatLng> _trail = []; // breadcrumb of past robot GPS positions
  LatLng? _smoothed; // EMA-smoothed position (tames GPS jitter)
  bool _showTrail = true; // toggle the trail layer on/off
  bool _showObstacles = true; // toggle obstacle pins on/off
  RobotSocket? _socket; // cached for the trail listener
  static const double _trailMinStepM = 3.0; // min meters between trail points
  static const int _trailMaxPoints = 5000; // cap so the list can't grow forever
  static const double _trailMaxJumpM = 40.0; // ignore isolated GPS spikes

  static const double _arrivalM = 3.0;
  static const LatLng _fallback = LatLng(14.7481338, 121.0616677);

  static const String _streetUrl =
      'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
  static const String _satUrl =
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}';

  void _recenter(LatLng robot) {
    setState(() => _follow = true);
    try {
      _map.move(robot, _map.camera.zoom);
    } catch (_) {
      _map.move(robot, 18);
    }
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final s = context.read<RobotSocket>();
    if (!identical(_socket, s)) {
      _socket?.removeListener(_onTrailUpdate);
      _socket = s;
      _socket!.addListener(_onTrailUpdate);
    }
  }

  @override
  void dispose() {
    _socket?.removeListener(_onTrailUpdate);
    super.dispose();
  }

  /// Append the robot's current GPS to the trail once it has moved enough.
  /// Smooths jitter (weak GPS fix wanders a few meters) so the line stays clean.
  void _onTrailUpdate() {
    if (!mounted) return;
    final s = _socket;
    if (s == null || !s.hasGps) return;
    final raw = LatLng(s.robotLat!, s.robotLon!);

    // glitch guard: ignore an isolated wild jump (GPS spike)
    if (_smoothed != null) {
      final jump = const Distance().as(LengthUnit.Meter, _smoothed!, raw);
      if (jump > _trailMaxJumpM) return;
    }

    // light exponential smoothing to tame jitter from a weak fix
    if (_smoothed == null) {
      _smoothed = raw;
    } else {
      const a = 0.35; // lower = smoother (more lag), higher = snappier
      _smoothed = LatLng(
        _smoothed!.latitude * (1 - a) + raw.latitude * a,
        _smoothed!.longitude * (1 - a) + raw.longitude * a,
      );
    }
    final p = _smoothed!;

    if (_trail.isEmpty) {
      setState(() => _trail.add(p));
      return;
    }
    final d = const Distance().as(LengthUnit.Meter, _trail.last, p);
    if (d >= _trailMinStepM) {
      setState(() {
        _trail.add(p);
        if (_trail.length > _trailMaxPoints) {
          _trail.removeRange(0, _trail.length - _trailMaxPoints);
        }
      });
    }
  }

  void _clearTrail() => setState(() {
        _trail.clear();
        _smoothed = null;
      });

  @override
  Widget build(BuildContext context) {
    final socket = context.read<RobotSocket>();
    context.select<RobotSocket, String>((s) =>
        '${s.isConnected},${s.robotLat},${s.robotLon},${s.robotHeading},${s.gpsFix},${s.gpsSats},${s.gpsSatsView},${s.obstaclePins.length}');

    final LatLng? robot =
        socket.hasGps ? LatLng(socket.robotLat!, socket.robotLon!) : null;

    // center once on first fix, then keep following while _follow is on
    if (robot != null && !_firstFix) {
      _firstFix = true;
      WidgetsBinding.instance.addPostFrameCallback((_) {
        try {
          _map.move(robot, 18);
        } catch (_) {}
      });
    } else if (robot != null && _follow) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        try {
          _map.move(robot, _map.camera.zoom);
        } catch (_) {}
      });
    }

    double? distM;
    if (robot != null && _waypoint != null) {
      distM = const Distance().as(LengthUnit.Meter, robot, _waypoint!);
    }
    final distLabel = distM == null
        ? ""
        : (distM >= 1000
            ? "${(distM / 1000).toStringAsFixed(2)} km"
            : "${distM.toStringAsFixed(0)} m");

    // auto-detect arrival
    if (_navigating && distM != null && distM <= _arrivalM) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (mounted && _navigating) {
          setState(() {
            _navigating = false;
            _arrived = true;
          });
        }
      });
    }

    final markers = <Marker>[];
    if (robot != null) {
      markers.add(Marker(
        point: robot,
        width: 46,
        height: 46,
        child: Transform.rotate(
          angle: socket.robotHeading * math.pi / 180.0,
          child: const Icon(Icons.navigation,
              color: Color(0xFF8BC34A), size: 40),
        ),
      ));
    }
    if (_waypoint != null) {
      markers.add(Marker(
        point: _waypoint!,
        width: 46,
        height: 46,
        child: const Icon(Icons.place, color: Color(0xFFE53935), size: 42),
      ));
    }
    if (_showObstacles) {
      for (final pin in socket.obstaclePins) {
        markers.add(Marker(
          point: LatLng(pin.lat, pin.lon),
          width: 30,
          height: 30,
          child: GestureDetector(
            onTap: () => _showObstacleInfo(pin),
            child: Container(
              decoration: BoxDecoration(
                color: const Color(0xFFE53935).withValues(alpha: 0.92),
                shape: BoxShape.circle,
                border: Border.all(color: Colors.white, width: 1.5),
              ),
              child: const Icon(Icons.warning_amber_rounded,
                  color: Colors.white, size: 16),
            ),
          ),
        ));
      }
    }

    return Scaffold(
      backgroundColor: AppColors.bg,
      appBar: AppBar(
        backgroundColor: AppColors.surface,
        foregroundColor: AppColors.textHi,
        title: Text("GPS MAP",
            style: GoogleFonts.rajdhani(
                fontWeight: FontWeight.w700, letterSpacing: 1.5)),
        actions: [
          IconButton(
            icon: Icon(_satellite ? Icons.map : Icons.satellite_alt),
            tooltip: _satellite ? "Street view" : "Satellite view",
            onPressed: () => setState(() => _satellite = !_satellite),
          ),
          IconButton(
            icon: Icon(Icons.route, color: _showTrail ? null : Colors.white38),
            tooltip: _showTrail
                ? "Hide path trail (${_trail.length})"
                : "Show path trail",
            onPressed: () => setState(() => _showTrail = !_showTrail),
          ),
          IconButton(
            icon: Icon(Icons.warning_amber_rounded,
                color: _showObstacles
                    ? const Color(0xFFE53935)
                    : Colors.white38),
            tooltip: _showObstacles
                ? "Hide obstacle pins (${socket.obstaclePins.length})"
                : "Show obstacle pins",
            onPressed: () => setState(() => _showObstacles = !_showObstacles),
          ),
          IconButton(
            icon: const Icon(Icons.layers_clear),
            tooltip: "Clear trail & obstacle pins",
            onPressed: (_trail.isEmpty && socket.obstaclePins.isEmpty)
                ? null
                : () {
                    _clearTrail();
                    socket.clearObstaclePins();
                  },
          ),
          IconButton(
            icon: Icon(_follow ? Icons.my_location : Icons.location_searching),
            tooltip: "Follow robot",
            onPressed: robot == null ? null : () => _recenter(robot),
          ),
        ],
      ),
      body: Stack(
        children: [
          FlutterMap(
            mapController: _map,
            options: MapOptions(
              initialCenter: robot ?? _fallback,
              initialZoom: 18,
              onTap: (tapPos, point) => setState(() {
                _waypoint = point;
                _arrived = false;
                _navigating = false;
              }),
              onPositionChanged: (camera, hasGesture) {
                if (hasGesture && _follow) {
                  setState(() => _follow = false); // user panned -> stop follow
                }
              },
            ),
            children: [
              TileLayer(
                urlTemplate: _satellite ? _satUrl : _streetUrl,
                userAgentPackageName: 'com.example.robot_controller',
              ),
              if (_showTrail && _trail.length >= 2)
                PolylineLayer(polylines: [
                  Polyline(
                    points: _trail,
                    strokeWidth: 3,
                    color: const Color(0xFF22D3EE), // cyan = dinaanan (trail)
                  ),
                ]),
              if (robot != null && _waypoint != null)
                PolylineLayer(polylines: [
                  Polyline(
                    points: [robot, _waypoint!],
                    strokeWidth: 3,
                    color: _navigating
                        ? const Color(0xFF8BC34A)
                        : const Color(0xFFFFB300),
                  ),
                ]),
              MarkerLayer(markers: markers),
            ],
          ),
          Positioned(top: 10, left: 10, right: 10, child: _gpsCard(socket)),
          Positioned(
            bottom: 0,
            left: 0,
            right: 0,
            child: _bottomPanel(socket, robot, distLabel),
          ),
        ],
      ),
    );
  }

  void _showObstacleInfo(ObstaclePin pin) {
    final t = pin.time;
    final ts = "${t.hour.toString().padLeft(2, '0')}:"
        "${t.minute.toString().padLeft(2, '0')}:"
        "${t.second.toString().padLeft(2, '0')}";
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(SnackBar(
        behavior: SnackBarBehavior.floating,
        backgroundColor: const Color(0xFFE53935),
        content: Text(
          "DANGER • ${pin.zone.toUpperCase()} • "
          "${pin.distanceM.toStringAsFixed(2)}m @ $ts",
          style: GoogleFonts.rajdhani(
              color: Colors.white, fontWeight: FontWeight.w700),
        ),
      ));
  }

  Widget _gpsCard(RobotSocket socket) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        color: (_satellite ? Colors.black : AppColors.surface)
            .withValues(alpha: 0.82),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: AppColors.border),
      ),
      child: socket.hasGps
          ? Text(
              "LAT ${socket.robotLat!.toStringAsFixed(6)}   "
              "LON ${socket.robotLon!.toStringAsFixed(6)}\n"
              "HDG ${socket.hasHeading ? '${socket.robotHeading.toStringAsFixed(0)}\u00B0' : '\u2014'}"
              "   FIX ${socket.gpsFix}   SAT ${socket.gpsSats}"
              "${_trail.isNotEmpty ? '   \u2022   TRAIL ${_trail.length}' : ''}",
              style: GoogleFonts.rajdhani(
                  color: _satellite ? Colors.white : AppColors.textHi,
                  fontWeight: FontWeight.w600,
                  height: 1.4),
            )
          // Connection-aware status: distinguishes "no link" from
          // "linked, but GPS has no sky lock yet" (shows live sats-in-view).
          : Text(
              socket.isConnected
                  ? (socket.gpsSatsView > 0
                      ? "Connected \u2022 acquiring GPS\u2026 ${socket.gpsSatsView} sat(s) in view (needs 4+ for a fix)"
                      : "Connected \u2022 acquiring GPS\u2026 (needs open sky)")
                  : "Not connected \u2014 set the Pi IP in the Connection screen",
              style: GoogleFonts.rajdhani(
                  color: _satellite ? Colors.white70 : AppColors.textLo)),
    );
  }

  Widget _bottomPanel(RobotSocket socket, LatLng? robot, String distLabel) {
    const lime = Color(0xFF8BC34A);
    Widget content;

    if (_arrived) {
      content = _row(
        title: "ARRIVED \u2713",
        titleColor: lime,
        subtitle: "Destination reached",
        actions: [_clearBtn(socket)],
      );
    } else if (_navigating) {
      content = _row(
        title: "\u25B6 NAVIGATING",
        titleColor: lime,
        subtitle:
            distLabel.isNotEmpty ? "$distLabel to go" : "Driving to waypoint\u2026",
        actions: [_stopBtn(socket)],
      );
    } else if (_waypoint != null) {
      content = _row(
        title: "WAYPOINT",
        titleColor: AppColors.textLo,
        subtitle:
            "${_waypoint!.latitude.toStringAsFixed(6)}, ${_waypoint!.longitude.toStringAsFixed(6)}"
            "${distLabel.isNotEmpty ? '   \u2022   $distLabel' : ''}",
        actions: [
          _clearBtn(socket),
          const SizedBox(width: 6),
          _goBtn(socket, robot),
        ],
      );
    } else {
      content = _row(
        title: "WAYPOINT",
        titleColor: AppColors.textLo,
        subtitle: "Tap the map to set a destination",
        actions: [_goBtn(socket, null)],
      );
    }

    return Container(
      padding: const EdgeInsets.fromLTRB(16, 14, 16, 8),
      decoration: BoxDecoration(
        color: AppColors.bg.withValues(alpha: 0.96),
        borderRadius: const BorderRadius.vertical(top: Radius.circular(20)),
        border: Border(
            top: BorderSide(
                color: (_navigating ? lime : AppColors.lime)
                    .withValues(alpha: 0.5),
                width: _navigating ? 2 : 1)),
      ),
      child: SafeArea(top: false, child: content),
    );
  }

  Widget _row({
    required String title,
    required Color titleColor,
    required String subtitle,
    required List<Widget> actions,
  }) {
    return Row(
      children: [
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(title,
                  style: GoogleFonts.rajdhani(
                      color: titleColor,
                      fontSize: 12,
                      letterSpacing: 1.5,
                      fontWeight: FontWeight.w700)),
              const SizedBox(height: 2),
              Text(subtitle,
                  style: GoogleFonts.rajdhani(
                      color: AppColors.textHi,
                      fontWeight: FontWeight.w700,
                      fontSize: 15)),
            ],
          ),
        ),
        ...actions,
      ],
    );
  }

  Widget _goBtn(RobotSocket socket, LatLng? robot) {
    final enabled = robot != null && _waypoint != null;
    return ElevatedButton.icon(
      onPressed: enabled
          ? () {
              setState(() {
                _navigating = true;
                _arrived = false;
                _follow = true;
              });
              socket.sendWaypoint(_waypoint!.latitude, _waypoint!.longitude);
              try {
                _map.move(robot, _map.camera.zoom);
              } catch (_) {}
            }
          : null,
      style: ElevatedButton.styleFrom(
        backgroundColor: AppColors.lime,
        foregroundColor: AppColors.bg,
        disabledBackgroundColor: AppColors.surfaceHi,
        padding: const EdgeInsets.symmetric(horizontal: 18, vertical: 12),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      icon: const Icon(Icons.navigation, size: 18),
      label: Text("GO",
          style: GoogleFonts.rajdhani(
              fontWeight: FontWeight.w800, letterSpacing: 1)),
    );
  }

  Widget _stopBtn(RobotSocket socket) {
    return ElevatedButton.icon(
      onPressed: () {
        setState(() => _navigating = false);
        socket.clearWaypoint(); // -> robot mode STOP
      },
      style: ElevatedButton.styleFrom(
        backgroundColor: const Color(0xFFE53935),
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 22, vertical: 12),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      icon: const Icon(Icons.stop, size: 20),
      label: Text("STOP",
          style: GoogleFonts.rajdhani(
              fontWeight: FontWeight.w800, letterSpacing: 1)),
    );
  }

  Widget _clearBtn(RobotSocket socket) {
    return TextButton(
      onPressed: () {
        setState(() {
          _waypoint = null;
          _navigating = false;
          _arrived = false;
        });
        socket.clearWaypoint();
      },
      child: Text("CLEAR",
          style: GoogleFonts.rajdhani(
              color: AppColors.textLo, fontWeight: FontWeight.w700)),
    );
  }
}