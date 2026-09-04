import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';
import '../widgets/lidar_painter.dart';

/// A saved snapshot of the live Lidar radar view. Kept in memory for the
/// current app session only (not written to device storage).
class LidarSnapshot {
  final Uint8List png;
  final DateTime time;
  final int pointCount;
  LidarSnapshot(
      {required this.png, required this.time, required this.pointCount});
}

/// Live Lidar point-cloud ("radar") view, with a button to save the current
/// scan as an image and a gallery of the snapshots saved this session.
class LidarMapScreen extends StatefulWidget {
  const LidarMapScreen({super.key});
  @override
  State<LidarMapScreen> createState() => _LidarMapScreenState();
}

class _LidarMapScreenState extends State<LidarMapScreen> {
  final GlobalKey _paintKey = GlobalKey();
  final List<LidarSnapshot> _snapshots = [];
  bool _saving = false;

  // ---- SLAM map save (server-side -- ~/save_slam_map.sh on the Pi via the
  // "SAVEMAP" WebSocket command, distinct from the point-cloud screenshot
  // above which is purely client-side/session-only) ----
  /// Destructive -- confirm before wiping the robot's mapped area.
  Future<void> _confirmResetMap(RobotSocket socket) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: AppColors.surface,
        title: Text("RESET MAPPED AREA?",
            style: GoogleFonts.rajdhani(
                color: AppColors.textHi,
                fontWeight: FontWeight.w700,
                letterSpacing: 1)),
        content: Text(
          "This wipes the robot's current SLAM map and starts mapping over "
          "from scratch.\n\nSnapshots you already saved (Gallery → MAPS) are "
          "NOT deleted.",
          style: GoogleFonts.rajdhani(color: AppColors.textLo, fontSize: 14),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: Text("CANCEL",
                style: GoogleFonts.rajdhani(
                    color: AppColors.textLo, fontWeight: FontWeight.w700)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            child: Text("RESET",
                style: GoogleFonts.rajdhani(
                    color: AppColors.offline, fontWeight: FontWeight.w700)),
          ),
        ],
      ),
    );
    if (confirmed == true) socket.resetSlamMap();
  }

  void _showSlamSnackBar(bool ok, String text) {
    if (!mounted) return;
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(SnackBar(
        behavior: SnackBarBehavior.floating,
        backgroundColor: ok ? AppColors.lime : AppColors.offline,
        content: Text(text,
            style: GoogleFonts.rajdhani(
                color: ok ? AppColors.onLime : Colors.white,
                fontWeight: FontWeight.w700)),
      ));
  }

  Future<void> _save(int pointCount) async {
    if (_saving) return;
    setState(() => _saving = true);
    try {
      final boundary = _paintKey.currentContext?.findRenderObject()
          as RenderRepaintBoundary?;
      if (boundary == null) return;
      final image = await boundary.toImage(pixelRatio: 2.0);
      final byteData = await image.toByteData(format: ui.ImageByteFormat.png);
      if (byteData == null) return;
      setState(() {
        _snapshots.insert(
          0,
          LidarSnapshot(
            png: byteData.buffer.asUint8List(),
            time: DateTime.now(),
            pointCount: pointCount,
          ),
        );
      });
      if (!mounted) return;
      ScaffoldMessenger.of(context)
        ..hideCurrentSnackBar()
        ..showSnackBar(SnackBar(
          behavior: SnackBarBehavior.floating,
          backgroundColor: AppColors.lime,
          content: Text("Saved ($pointCount points)",
              style: GoogleFonts.rajdhani(
                  color: AppColors.onLime, fontWeight: FontWeight.w700)),
        ));
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context)
        ..hideCurrentSnackBar()
        ..showSnackBar(SnackBar(
          behavior: SnackBarBehavior.floating,
          backgroundColor: AppColors.offline,
          content: Text("Couldn't save snapshot: $e",
              style: GoogleFonts.rajdhani(color: Colors.white)),
        ));
    } finally {
      if (mounted) setState(() => _saving = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.bg,
      appBar: AppBar(
        backgroundColor: AppColors.surface,
        foregroundColor: AppColors.textHi,
        title: Text("LIDAR MAP",
            style: GoogleFonts.rajdhani(
                fontWeight: FontWeight.w700, letterSpacing: 1.5)),
      ),
      body: Consumer<RobotSocket>(
        builder: (context, socket, _) {
          final points = socket.lidarPoints;
          if (socket.mapSaveResult != null || socket.mapSaveError != null) {
            final ok = socket.mapSaveResult != null;
            final text = ok
                ? "SLAM map saved as \"${socket.mapSaveResult}\" — view it in Gallery → MAPS"
                : "Couldn't save SLAM map: ${socket.mapSaveError}";
            WidgetsBinding.instance.addPostFrameCallback((_) {
              _showSlamSnackBar(ok, text);
              socket.clearMapSaveResult();
            });
          }
          if (socket.mapResetOk || socket.mapResetError != null) {
            final ok = socket.mapResetOk;
            final text = ok
                ? "Mapped area reset — the robot is mapping from scratch now"
                : "Couldn't reset the map: ${socket.mapResetError}";
            WidgetsBinding.instance.addPostFrameCallback((_) {
              _showSlamSnackBar(ok, text);
              socket.clearMapResetResult();
            });
          }
          return ListView(
            padding: const EdgeInsets.all(16),
            children: [
              AspectRatio(
                aspectRatio: 1,
                child: RepaintBoundary(
                  key: _paintKey,
                  child: Container(
                    decoration: BoxDecoration(
                      color: AppColors.surfaceHi,
                      borderRadius: BorderRadius.circular(16),
                      border: Border.all(color: AppColors.border),
                    ),
                    child: ClipRRect(
                      borderRadius: BorderRadius.circular(16),
                      child: Stack(
                        children: [
                          Positioned.fill(
                            child: CustomPaint(painter: LidarPainter(points)),
                          ),
                          if (points.isEmpty)
                            Center(
                              child: Padding(
                                padding: const EdgeInsets.all(24),
                                child: Text(
                                  socket.isConnected
                                      ? "No Lidar data yet — waiting for the robot's scan feed"
                                      : "Not connected — set the Pi IP in the Connection screen",
                                  textAlign: TextAlign.center,
                                  style: GoogleFonts.rajdhani(
                                      color: AppColors.textLo, fontSize: 14),
                                ),
                              ),
                            ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 10),
              Text(
                "${points.length} point(s) live",
                style: GoogleFonts.rajdhani(
                    color: AppColors.textLo, fontSize: 13),
              ),
              const SizedBox(height: 14),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: (points.isEmpty || _saving)
                      ? null
                      : () => _save(points.length),
                  icon: const Icon(Icons.save_alt),
                  label: Text(_saving ? "SAVING…" : "SAVE CURRENT MAP"),
                ),
              ),
              const SizedBox(height: 10),
              SizedBox(
                width: double.infinity,
                child: OutlinedButton.icon(
                  onPressed: (!socket.isConnected || socket.isSavingMap)
                      ? null
                      : () => socket.saveSlamMap(),
                  icon: socket.isSavingMap
                      ? const SizedBox(
                          width: 16,
                          height: 16,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.map_outlined),
                  label: Text(
                      socket.isSavingMap ? "SAVING SLAM MAP…" : "SAVE SLAM MAP SNAPSHOT"),
                ),
              ),
              Padding(
                padding: const EdgeInsets.only(top: 6),
                child: Text(
                  "Saves the actual SLAM-built map from the robot (slam_toolbox) "
                  "into its gallery. Different from the radar screenshot above.",
                  style: GoogleFonts.rajdhani(color: AppColors.textLo, fontSize: 12),
                ),
              ),
              const SizedBox(height: 10),
              SizedBox(
                width: double.infinity,
                child: OutlinedButton.icon(
                  onPressed: (!socket.isConnected || socket.isResettingMap)
                      ? null
                      : () => _confirmResetMap(socket),
                  style: OutlinedButton.styleFrom(
                    foregroundColor: AppColors.offline,
                    side: const BorderSide(color: AppColors.offline),
                  ),
                  icon: socket.isResettingMap
                      ? const SizedBox(
                          width: 16,
                          height: 16,
                          child: CircularProgressIndicator(strokeWidth: 2),
                        )
                      : const Icon(Icons.delete_sweep_outlined),
                  label: Text(socket.isResettingMap
                      ? "RESETTING…"
                      : "RESET MAPPED AREA"),
                ),
              ),
              Padding(
                padding: const EdgeInsets.only(top: 6),
                child: Text(
                  "Wipes the robot's current SLAM map and starts mapping over "
                  "from scratch. Saved snapshots are not deleted.",
                  style: GoogleFonts.rajdhani(color: AppColors.textLo, fontSize: 12),
                ),
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Text(
                    "SAVED THIS SESSION",
                    style: GoogleFonts.rajdhani(
                        color: AppColors.textLo,
                        fontSize: 12,
                        letterSpacing: 1.5,
                        fontWeight: FontWeight.w700),
                  ),
                  const Spacer(),
                  if (_snapshots.isNotEmpty)
                    TextButton(
                      onPressed: () => setState(() => _snapshots.clear()),
                      child: Text("CLEAR",
                          style: GoogleFonts.rajdhani(
                              color: AppColors.textLo,
                              fontWeight: FontWeight.w700)),
                    ),
                ],
              ),
              const SizedBox(height: 8),
              if (_snapshots.isEmpty)
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 20),
                  child: Text(
                    "No snapshots saved yet. Saved maps only last for this "
                    "app session.",
                    style: GoogleFonts.rajdhani(color: AppColors.textLo),
                  ),
                )
              else
                GridView.builder(
                  shrinkWrap: true,
                  physics: const NeverScrollableScrollPhysics(),
                  gridDelegate:
                      const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 3,
                    crossAxisSpacing: 6,
                    mainAxisSpacing: 6,
                  ),
                  itemCount: _snapshots.length,
                  itemBuilder: (context, i) {
                    final snap = _snapshots[i];
                    return GestureDetector(
                      onTap: () => Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (_) => _SnapshotViewer(
                            snapshots: _snapshots,
                            startIndex: i,
                          ),
                        ),
                      ),
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(8),
                        child: Image.memory(snap.png, fit: BoxFit.cover),
                      ),
                    );
                  },
                ),
            ],
          );
        },
      ),
    );
  }
}

class _SnapshotViewer extends StatefulWidget {
  final List<LidarSnapshot> snapshots;
  final int startIndex;
  const _SnapshotViewer({required this.snapshots, required this.startIndex});

  @override
  State<_SnapshotViewer> createState() => _SnapshotViewerState();
}

class _SnapshotViewerState extends State<_SnapshotViewer> {
  late final PageController _page;
  late int _index;

  @override
  void initState() {
    super.initState();
    _index = widget.startIndex;
    _page = PageController(initialPage: _index);
  }

  @override
  void dispose() {
    _page.dispose();
    super.dispose();
  }

  String _fmtTime(DateTime t) =>
      "${t.year}-${t.month.toString().padLeft(2, '0')}-${t.day.toString().padLeft(2, '0')} "
      "${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}:"
      "${t.second.toString().padLeft(2, '0')}";

  @override
  Widget build(BuildContext context) {
    final snap = widget.snapshots[_index];
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        foregroundColor: Colors.white,
        title: Text("${snap.pointCount} points",
            style: GoogleFonts.rajdhani(fontSize: 15)),
      ),
      body: Column(
        children: [
          Expanded(
            child: PageView.builder(
              controller: _page,
              itemCount: widget.snapshots.length,
              onPageChanged: (i) => setState(() => _index = i),
              itemBuilder: (context, i) => InteractiveViewer(
                minScale: 1,
                maxScale: 5,
                child: Center(
                  child: Image.memory(widget.snapshots[i].png),
                ),
              ),
            ),
          ),
          Container(
            width: double.infinity,
            padding: const EdgeInsets.symmetric(vertical: 10),
            color: const Color(0xFF111111),
            child: Text(
              "${_index + 1} / ${widget.snapshots.length}   •   ${_fmtTime(snap.time)}",
              textAlign: TextAlign.center,
              style: GoogleFonts.rajdhani(color: Colors.white70),
            ),
          ),
        ],
      ),
    );
  }
}
