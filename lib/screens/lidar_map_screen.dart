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
