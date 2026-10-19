import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../models/field_boundary.dart';
import '../models/geo_image.dart';
import '../services/gallery_api.dart';
import '../services/robot_socket.dart';
import '../services/run_log_store.dart';
import '../theme/app_theme.dart';

/// GPS map (2.1 visualization) + interactive waypoint navigation (2.2).
/// - GO starts navigation and shows a live NAVIGATING state + STOP button.
/// - The map FOLLOWS the robot so the arrow always stays on the real GPS.
class MapScreen extends StatefulWidget {
  /// Optional: open centred on one spot (used by the gallery's
  /// "show where this was taken" action).
  final LatLng? focus;
  final String? focusLabel;
  const MapScreen({super.key, this.focus, this.focusLabel});
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

  // ---- field boundary editing (see models/field_boundary.dart) ----
  // While editing, a tap adds a CORNER instead of setting a waypoint. The
  // draft is only a preview until SAVE: the robot validates it again and
  // reports back on "Z:", and what the map then draws is what it stored.
  bool _editingFence = false;
  final List<LatLng> _draft = [];
  double _inset = kDefaultInsetM;
  RobotSocket? _socket; // cached for the trail listener
  static const double _trailMinStepM = 3.0; // min meters between trail points
  static const int _trailMaxPoints = 5000; // cap so the list can't grow forever
  static const double _trailMaxJumpM = 40.0; // ignore isolated GPS spikes

  // --- Geotagged photo pins (objective: map captured images to lat/lon) ---
  // Loaded over HTTP from gallery_server.py, NOT over the robot's WebSocket:
  // these are files on disk, so they are fetched on demand rather than
  // streamed while driving.
  PhotoIndex _photoIndex = PhotoIndex.empty;
  List<PhotoCluster> _photoPins = [];
  bool _showPhotos = true;
  bool _photosLoading = false;
  String? _photoError;
  bool _photosRequested = false;
  // Photos within this distance share one pin. The robot saves a soil image
  // every 60s, so a session leaves hundreds; one marker each would bury the
  // map, and while it is stationary they all land on the same spot anyway.
  static const double _photoClusterM = 2.0;

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
  void initState() {
    super.initState();
    if (widget.focus != null) {
      // Opened to show ONE photo's location: don't let the first GPS fix or
      // the follow-the-robot behaviour yank the camera away from it.
      _firstFix = true;
      _follow = false;
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
    if (!_photosRequested && s.hostIp != null) {
      _photosRequested = true;
      _loadPhotos();
    }
  }

  /// Fetch the photo list and turn it into map pins.
  ///
  /// Deliberately manual (on open + refresh button) rather than polled: the
  /// list only changes when the robot saves a photo, and polling would eat
  /// bandwidth shared with the live camera feed.
  Future<void> _loadPhotos() async {
    final ip = _socket?.hostIp;
    if (ip == null || _photosLoading) return;
    setState(() {
      _photosLoading = true;
      _photoError = null;
    });
    try {
      final idx = await fetchPhotoIndex(ip);
      if (!mounted) return;
      setState(() {
        _photoIndex = idx;
        _photoPins = clusterPhotos(
          idx.photos.map((p) => p.geo).toList(),
          radiusM: _photoClusterM,
        );
        _photosLoading = false;
      });
    } catch (e) {
      if (!mounted) return;
      // Pins are an extra layer: if the gallery server is unreachable the map
      // keeps working on live WebSocket data alone.
      setState(() {
        _photoError = e.toString();
        _photosLoading = false;
      });
    }
  }

  /// Which kind a photo came from, so its thumbnail can be fetched.
  String _kindOf(GeoImage g) => _photoIndex.photos
      .firstWhere((p) => p.geo.name == g.name,
          orElse: () => GalleryPhoto(g, g.kind == 'soil' ? 'soil' : 'captures'))
      .kind;

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

    if (_showPhotos) {
      for (final pin in _photoPins) {
        markers.add(Marker(
          point: LatLng(pin.lat, pin.lon),
          width: 34,
          height: 34,
          child: GestureDetector(
            onTap: () => _showPhotoPin(pin),
            child: _PhotoPinIcon(cluster: pin),
          ),
        ));
      }
    }
    if (widget.focus != null) {
      markers.add(Marker(
        point: widget.focus!,
        width: 46,
        height: 46,
        child: const Icon(Icons.my_location,
            color: Color(0xFF22D3EE), size: 40),
      ));
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
            icon: Icon(Icons.fence,
                color: _editingFence
                    ? const Color(0xFFFFB300)
                    : (socket.boundary?.active == true
                        ? AppColors.lime
                        : null)),
            tooltip: _editingFence
                ? "Stop editing the boundary"
                : "Edit the field boundary",
            onPressed: () => setState(() {
              if (!_editingFence) {
                // start from what the robot has, so editing is editing
                _draft
                  ..clear()
                  ..addAll(socket.boundary?.points ?? const []);
                _inset = socket.boundary?.insetM ?? kDefaultInsetM;
              }
              _editingFence = !_editingFence;
            }),
          ),
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
                ? "Hide Lidar obstacle pins (${socket.obstaclePins.length})"
                : "Show Lidar obstacle pins",
            onPressed: () => setState(() => _showObstacles = !_showObstacles),
          ),
          IconButton(
            icon: _photosLoading
                ? const SizedBox(
                    width: 18,
                    height: 18,
                    child: CircularProgressIndicator(
                        strokeWidth: 2, color: AppColors.lime),
                  )
                : Icon(Icons.photo_camera_outlined,
                    color: _photoError != null
                        ? const Color(0xFFFFB300)
                        : (_showPhotos ? AppColors.lime : Colors.white38)),
            tooltip: _photoError != null
                ? "Obstacle pins unavailable - tap to retry"
                : (_showPhotos
                    ? "Hide obstacle photos (${_photoPins.length})"
                    : "Show obstacle photos"),
            onPressed: () {
              if (_photoError != null || _photoPins.isEmpty) {
                _loadPhotos();
              } else {
                setState(() => _showPhotos = !_showPhotos);
              }
            },
          ),
          IconButton(
            icon: const Icon(Icons.layers_clear),
            tooltip: "Clear trail & Lidar pins",
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
              initialCenter: widget.focus ?? robot ?? _fallback,
              initialZoom: 18,
              onTap: (tapPos, point) => setState(() {
                if (_editingFence) {
                  _draft.add(point); // a corner, not a destination
                  return;
                }
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
              ..._boundaryLayers(socket),
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
          // Photos that exist but carry no position. Stated explicitly:
          // if they just silently vanished from the map it would look like
          // a bug rather than the honest "there was no fix at that moment".
          if (_showPhotos && _photoIndex.noFixCount > 0)
            Positioned(
              top: 96,
              right: 10,
              child: Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: AppColors.surface.withValues(alpha: 0.88),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.white12),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Icon(Icons.location_disabled,
                        size: 13, color: Color(0xFFFFB300)),
                    const SizedBox(width: 5),
                    Text(
                      "${_photoIndex.noFixCount} obstacle photo"
                      "${_photoIndex.noFixCount == 1 ? '' : 's'} without GPS",
                      style: GoogleFonts.rajdhani(
                          color: AppColors.textLo,
                          fontSize: 12,
                          fontWeight: FontWeight.w600),
                    ),
                  ],
                ),
              ),
            ),
          Positioned(
            bottom: 0,
            left: 0,
            right: 0,
            child: _editingFence
                ? _fencePanel(socket, robot)
                : Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      if (socket.boundary?.active == true)
                        _coverCard(socket, robot),
                      _bottomPanel(socket, robot, distLabel),
                    ],
                  ),
          ),
        ],
      ),
    );
  }

  /// Tapping a photo pin: what was photographed here, and when.
  void _showPhotoPin(PhotoCluster cluster) {
    final ip = _socket?.hostIp;
    if (ip == null) return;
    showModalBottomSheet<void>(
      context: context,
      backgroundColor: AppColors.surface,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (_) {
        final imgs = cluster.images;
        return SafeArea(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(16, 14, 16, 18),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    const Icon(Icons.warning_amber_rounded,
                        color: Color(0xFFFFB300), size: 18),
                    const SizedBox(width: 6),
                    Expanded(
                      child: Text(
                        cluster.cover.coordLabel,
                        style: GoogleFonts.rajdhani(
                          color: AppColors.textHi,
                          fontWeight: FontWeight.w700,
                          fontSize: 16,
                        ),
                      ),
                    ),
                    Text(
                      imgs.length == 1
                          ? "1 obstacle"
                          : "${imgs.length} obstacles",
                      style: GoogleFonts.rajdhani(color: AppColors.textLo),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                SizedBox(
                  height: 108,
                  child: ListView.separated(
                    scrollDirection: Axis.horizontal,
                    itemCount: imgs.length,
                    separatorBuilder: (_, __) => const SizedBox(width: 8),
                    itemBuilder: (_, i) {
                      final g = imgs[i];
                      return Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          ClipRRect(
                            borderRadius: BorderRadius.circular(8),
                            child: Image.network(
                              galleryImgUrl(ip, _kindOf(g), g.name),
                              width: 128,
                              height: 84,
                              fit: BoxFit.cover,
                              errorBuilder: (_, __, ___) => Container(
                                width: 128,
                                height: 84,
                                color: AppColors.surfaceHi,
                                child: const Icon(Icons.broken_image,
                                    color: AppColors.textLo),
                              ),
                            ),
                          ),
                          const SizedBox(height: 3),
                          Text(
                            "${g.time.hour.toString().padLeft(2, '0')}:"
                            "${g.time.minute.toString().padLeft(2, '0')}:"
                            "${g.time.second.toString().padLeft(2, '0')}"
                            "${g.kind == 'soil' ? '  soil' : ''}",
                            style: GoogleFonts.rajdhani(
                                color: AppColors.textLo, fontSize: 12),
                          ),
                        ],
                      );
                    },
                  ),
                ),
              ],
            ),
          ),
        );
      },
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
              "LAT ${formatCoord(socket.robotLat!)}   "
              "LON ${formatCoord(socket.robotLon!)}\n"
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

  // ---------------------------------------------------------------- boundary
  static const _fenceColor = Color(0xFF8BC34A);
  static const _stopColor = Color(0xFFFFB300);
  static const _laneColor = Color(0xFF22D3EE);

  /// The boundary, its stop line and COVER's lanes -- or, while editing,
  /// the draft and a live preview of where its stop line would fall.
  List<Widget> _boundaryLayers(RobotSocket socket) {
    if (_editingFence) {
      final check = checkBoundary(_draft, _inset);
      return [
        if (_draft.length >= 2)
          PolylineLayer(polylines: [
            Polyline(
              points: [..._draft, if (_draft.length >= 3) _draft.first],
              strokeWidth: 3,
              color: check.ok ? _fenceColor : const Color(0xFFE53935),
            ),
          ]),
        if (check.ok)
          PolylineLayer(polylines: [
            Polyline(
              points: [...check.stopLine, check.stopLine.first],
              strokeWidth: 2,
              color: _stopColor,
              pattern: StrokePattern.dashed(segments: const [8, 6]),
            ),
          ]),
        MarkerLayer(markers: [
          for (var i = 0; i < _draft.length; i++)
            Marker(
              point: _draft[i],
              width: 22,
              height: 22,
              child: Container(
                alignment: Alignment.center,
                decoration: BoxDecoration(
                  color: _stopColor,
                  shape: BoxShape.circle,
                  border: Border.all(color: Colors.black54),
                ),
                child: Text("${i + 1}",
                    style: GoogleFonts.rajdhani(
                        color: Colors.black,
                        fontSize: 12,
                        fontWeight: FontWeight.w800)),
              ),
            ),
        ]),
      ];
    }
    final b = socket.boundary;
    if (b == null || !b.active || b.points.length < 3) return const [];
    final lanes = socket.coverPlan?.lanes ?? const <List<LatLng>>[];
    return [
      PolygonLayer(polygons: [
        Polygon(
          points: b.points,
          color: _fenceColor.withValues(alpha: 0.10),
          borderColor: _fenceColor,
          borderStrokeWidth: 3,
        ),
      ]),
      if (b.stopLine.length >= 3)
        PolylineLayer(polylines: [
          Polyline(
            points: [...b.stopLine, b.stopLine.first],
            strokeWidth: 2,
            color: _stopColor,
            pattern: StrokePattern.dashed(segments: const [8, 6]),
          ),
        ]),
      if (lanes.isNotEmpty)
        PolylineLayer(polylines: [
          for (final ln in lanes)
            if (ln.length >= 2)
              Polyline(
                points: ln,
                strokeWidth: 2,
                color: _laneColor.withValues(alpha: 0.7),
              ),
        ]),
    ];
  }

  /// Bottom panel while editing the boundary.
  Widget _fencePanel(RobotSocket socket, LatLng? robot) {
    final check = checkBoundary(_draft, _inset);
    final online = socket.isConnected;
    final String status;
    final Color statusColor;
    if (_draft.isEmpty) {
      status = "Tap the map to add corners, or stand the robot at a corner "
          "and use ROBOT HERE";
      statusColor = AppColors.textLo;
    } else if (check.ok) {
      status = "${check.corners.length} corners, "
          "${(check.areaM2 / 10000).toStringAsFixed(2)} ha -- "
          "stop line ${_inset.toStringAsFixed(1)} m inside";
      statusColor = _fenceColor;
    } else {
      status = check.reason;
      statusColor = const Color(0xFFFFB300);
    }
    Widget btn(String label, IconData icon, VoidCallback? onTap,
        {Color? color}) {
      return TextButton.icon(
        onPressed: onTap,
        style: TextButton.styleFrom(
            foregroundColor: color ?? AppColors.textHi,
            padding: const EdgeInsets.symmetric(horizontal: 8)),
        icon: Icon(icon, size: 18),
        label: Text(label,
            style: GoogleFonts.rajdhani(fontWeight: FontWeight.w700)),
      );
    }

    return Container(
      padding: const EdgeInsets.fromLTRB(14, 12, 14, 14),
      decoration: BoxDecoration(
        color: AppColors.surface.withValues(alpha: 0.96),
        border: const Border(top: BorderSide(color: _stopColor, width: 2)),
      ),
      child: SafeArea(
        top: false,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.fence, color: _stopColor, size: 18),
                const SizedBox(width: 6),
                Text("EDITING BOUNDARY",
                    style: GoogleFonts.rajdhani(
                        color: _stopColor,
                        fontWeight: FontWeight.w800,
                        letterSpacing: 1.2)),
                const Spacer(),
                Text("stop line",
                    style: GoogleFonts.rajdhani(
                        color: AppColors.textLo, fontSize: 12)),
                IconButton(
                  visualDensity: VisualDensity.compact,
                  icon: const Icon(Icons.remove, size: 18),
                  onPressed: _inset > 1.0
                      ? () => setState(() => _inset -= 0.5)
                      : null,
                ),
                Text("${_inset.toStringAsFixed(1)} m",
                    style: GoogleFonts.rajdhani(
                        color: AppColors.textHi, fontWeight: FontWeight.w700)),
                IconButton(
                  visualDensity: VisualDensity.compact,
                  icon: const Icon(Icons.add, size: 18),
                  onPressed: _inset < 15.0
                      ? () => setState(() => _inset += 0.5)
                      : null,
                ),
              ],
            ),
            Text(status,
                style: GoogleFonts.rajdhani(color: statusColor, fontSize: 13)),
            const SizedBox(height: 6),
            Wrap(
              spacing: 2,
              children: [
                // the same receiver the robot navigates with: more accurate
                // for the robot than a tap on a map that may be offset
                btn("ROBOT HERE", Icons.my_location,
                    robot == null ? null : () => setState(() => _draft.add(robot))),
                btn("UNDO", Icons.undo,
                    _draft.isEmpty ? null : () => setState(() => _draft.removeLast())),
                btn("CLEAR", Icons.delete_outline,
                    _draft.isEmpty ? null : () => setState(() => _draft.clear())),
                if (_draft.isEmpty && socket.boundary?.points.isNotEmpty == true)
                  btn("REMOVE BOUNDARY", Icons.layers_clear, !online
                      ? null
                      : () {
                          socket.clearFence();
                          setState(() => _editingFence = false);
                        }, color: const Color(0xFFE53935))
                else
                  btn("SAVE", Icons.check, !(check.ok && online)
                      ? null
                      : () {
                          socket.sendFence(check.corners, _inset);
                          setState(() => _editingFence = false);
                        }, color: _fenceColor),
              ],
            ),
          ],
        ),
      ),
    );
  }

  /// COVER mode: start/stop, and how far through the field it is.
  Widget _coverCard(RobotSocket socket, LatLng? robot) {
    final st = socket.coverStatus;
    final plan = socket.coverPlan;
    final running = st?.active == true;
    final runs = context.read<RunLogStore>();
    String line;
    if (plan != null && plan.error.isNotEmpty && plan.lanes.isEmpty) {
      line = "COVER can't plan this field: ${plan.error}";
    } else if (st == null || plan == null) {
      line = "COVER: waiting for the plan";
    } else if (st.done) {
      final where = st.note.contains('back at the start')
          ? ', back at the start'
          : st.note.contains('could not get back')
              ? ' -- could not get back to the start'
              : '';
      line = "COVER complete -- ~${st.coverage.toStringAsFixed(0)}% covered, "
          "${st.overlap.toStringAsFixed(0)}% driven twice$where";
    } else if (running && st.returning) {
      line = "Field done -- returning to start"
          "${st.homeM != null ? ' (${st.homeM!.toStringAsFixed(0)} m to go)' : ''}";
    } else if (running) {
      line = "${st.phase == 'FILL' ? 'Filling gaps' : 'Lane ${st.lane} of ${st.lanes}'}"
          "  \u2022  ~${st.coverage.toStringAsFixed(0)}% covered";
    } else {
      final mins = plan.summary['minutes'];
      line = "${plan.lanes.length} lanes"
          "${mins != null ? ', about $mins min' : ''}"
          "${st.coverage > 0 ? '  \u2022  ~${st.coverage.toStringAsFixed(0)}% done so far' : ''}";
    }
    final canStart = socket.isConnected && plan != null &&
        plan.lanes.isNotEmpty && !running && st?.done != true;
    return Container(
      margin: const EdgeInsets.fromLTRB(10, 0, 10, 8),
      padding: const EdgeInsets.fromLTRB(12, 8, 8, 8),
      decoration: BoxDecoration(
        color: AppColors.surface.withValues(alpha: 0.94),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: running ? _laneColor : AppColors.border),
      ),
      child: Row(
        children: [
          Icon(Icons.grid_on, size: 18, color: running ? _laneColor : AppColors.textLo),
          const SizedBox(width: 8),
          Expanded(
            child: Text(line,
                style: GoogleFonts.rajdhani(
                    color: AppColors.textHi,
                    fontSize: 13,
                    fontWeight: FontWeight.w600)),
          ),
          if (!running && (st?.coverage ?? 0) > 0)
            TextButton(
              onPressed: socket.isConnected ? socket.resetCover : null,
              child: Text("RESET",
                  style: GoogleFonts.rajdhani(
                      color: AppColors.textLo, fontWeight: FontWeight.w700)),
            ),
          if (running)
            ElevatedButton(
              onPressed: () {
                socket.sendCommand("MODE:STOP");
                if (runs.activeMode == 'COVER') {
                  runs.stop(
                      lat: socket.hasGps ? socket.robotLat : null,
                      lon: socket.hasGps ? socket.robotLon : null);
                }
              },
              style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFFE53935),
                  foregroundColor: Colors.white),
              child: Text("STOP",
                  style: GoogleFonts.rajdhani(fontWeight: FontWeight.w800)),
            )
          else
            ElevatedButton(
              onPressed: !canStart
                  ? null
                  : () {
                      socket.startCover();
                      // a COVER run is logged like AUTO; that also turns
                      // image capture on (RunLogStore -> CAPTURE:ON)
                      runs.start('COVER',
                          lat: socket.hasGps ? socket.robotLat : null,
                          lon: socket.hasGps ? socket.robotLon : null);
                    },
              style: ElevatedButton.styleFrom(
                  backgroundColor: _laneColor, foregroundColor: Colors.black),
              child: Text("START COVER",
                  style: GoogleFonts.rajdhani(fontWeight: FontWeight.w800)),
            ),
        ],
      ),
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
            "${formatCoord(_waypoint!.latitude)}, ${formatCoord(_waypoint!.longitude)}"
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

/// Map marker for the obstacles found at one spot.
///
/// Amber with a camera icon, deliberately not the red warning used by the
/// Lidar's own DANGER pins: both mark obstacles, but one is a photographed
/// finding you can open and the other is a live range reading.
class _PhotoPinIcon extends StatelessWidget {
  final PhotoCluster cluster;
  const _PhotoPinIcon({required this.cluster});

  @override
  Widget build(BuildContext context) {
    const color = Color(0xFFFFB300);
    return Stack(
      clipBehavior: Clip.none,
      children: [
        Container(
          decoration: BoxDecoration(
            color: color.withValues(alpha: 0.92),
            shape: BoxShape.circle,
            border: Border.all(color: Colors.white, width: 1.5),
          ),
          child: const Icon(Icons.photo_camera,
              color: Color(0xFF1A1200), size: 17),
        ),
        if (cluster.count > 1)
          Positioned(
            right: -3,
            top: -3,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 1),
              decoration: BoxDecoration(
                color: Colors.black87,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.white24),
              ),
              child: Text(
                cluster.count > 99 ? "99+" : "${cluster.count}",
                style: GoogleFonts.rajdhani(
                  color: Colors.white,
                  fontSize: 9,
                  fontWeight: FontWeight.w700,
                ),
              ),
            ),
          ),
      ],
    );
  }
}
