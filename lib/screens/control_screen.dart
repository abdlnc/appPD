import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import '../services/dev_settings.dart';
import '../services/run_log_store.dart';
import 'logs_screen.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';
import '../widgets/lidar_painter.dart';
import '../widgets/obstacle_warning.dart';
import '../widgets/detection_overlay.dart';
import 'map_screen.dart';
import 'gallery_screen.dart';
import 'lidar_map_screen.dart';

class ControlScreen extends StatefulWidget {
  const ControlScreen({super.key});
  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  bool isAutoMode = false;

  // --- Hidden developer settings (see DevSettings) ---
  bool _showJoystick = false; // hidden unless enabled in developer settings
  int _devTaps = 0;
  DateTime? _lastDevTap;
  static const int _devTapsNeeded = 7;
  static const Duration _devTapWindow = Duration(milliseconds: 1500);

  @override
  void initState() {
    super.initState();
    DevSettings.loadShowJoystick().then((v) {
      if (mounted) setState(() => _showJoystick = v);
    });
  }

  /// Start or stop a recorded run, stamping the robot's position at each end.
  ///
  /// The fix is read at the moment of the call rather than held from earlier:
  /// a run that began indoors and ended outside keeps a real end point, and
  /// one with no fix at all still records its duration.
  Future<void> _record(bool start, String mode, RobotSocket socket) async {
    final store = context.read<RunLogStore>();
    final lat = socket.hasGps ? socket.robotLat : null;
    final lon = socket.hasGps ? socket.robotLon : null;
    if (start) {
      await store.start(mode, lat: lat, lon: lon);
    } else {
      final run = await store.stop(lat: lat, lon: lon);
      if (run != null && mounted) {
        ScaffoldMessenger.of(context)
          ..hideCurrentSnackBar()
          ..showSnackBar(SnackBar(
            behavior: SnackBarBehavior.floating,
            content: Text("${run.mode} run logged - ${run.durationLabel}",
                style: GoogleFonts.rajdhani(fontWeight: FontWeight.w600)),
          ));
      }
    }
    if (mounted) setState(() {});
  }

  /// 7 quick taps on the mode pill open the developer settings. Taps more
  /// than [_devTapWindow] apart start the count again, so ordinary
  /// accidental taps never add up to anything.
  void _onModePillTap(RobotSocket socket) {
    final now = DateTime.now();
    if (_lastDevTap == null || now.difference(_lastDevTap!) > _devTapWindow) {
      _devTaps = 0;
    }
    _lastDevTap = now;
    _devTaps++;

    final left = _devTapsNeeded - _devTaps;
    if (left <= 0) {
      _devTaps = 0;
      ScaffoldMessenger.of(context).hideCurrentSnackBar();
      _openDevSettings(socket);
      return;
    }
    // Only start counting down once it is clearly deliberate.
    if (_devTaps >= 3) {
      ScaffoldMessenger.of(context)
        ..hideCurrentSnackBar()
        ..showSnackBar(
          SnackBar(
            behavior: SnackBarBehavior.floating,
            duration: const Duration(milliseconds: 900),
            margin: const EdgeInsets.fromLTRB(16, 0, 16, 16),
            content: Text(
              "$left more tap${left == 1 ? '' : 's'} to developer settings",
              style: GoogleFonts.rajdhani(fontWeight: FontWeight.w600),
            ),
          ),
        );
    }
  }

  Future<void> _setShowJoystick(bool value, RobotSocket socket) async {
    setState(() => _showJoystick = value);
    await DevSettings.saveShowJoystick(value);
    // Hiding the joystick must not leave a stale drive command behind: the
    // Pi keeps re-sending the LAST joystick position while in MANUAL. Zero
    // it -- but only in manual, because any "x,y" message also switches the
    // robot INTO manual, which must never happen to a robot running AUTO.
    if (!value && !isAutoMode) {
      socket.sendCommand("0,0");
    }
  }

  void _openDevSettings(RobotSocket socket) {
    showModalBottomSheet<void>(
      context: context,
      backgroundColor: AppColors.surface,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setSheet) => SafeArea(
          child: Padding(
            padding: const EdgeInsets.fromLTRB(20, 16, 20, 20),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    const Icon(
                      Icons.developer_mode,
                      color: AppColors.dev,
                      size: 20,
                    ),
                    const SizedBox(width: 8),
                    Text(
                      "DEVELOPER SETTINGS",
                      style: GoogleFonts.rajdhani(
                        color: AppColors.textHi,
                        fontWeight: FontWeight.w700,
                        fontSize: 16,
                        letterSpacing: 1.5,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 4),
                Text(
                  "For bench testing and calibration.",
                  style: GoogleFonts.rajdhani(color: AppColors.textLo),
                ),
                const SizedBox(height: 8),
                SwitchListTile(
                  contentPadding: EdgeInsets.zero,
                  // Dev hue, so even the switch reads as "not a user setting"
                  thumbColor: const WidgetStatePropertyAll(AppColors.dev),
                  trackColor: WidgetStateProperty.resolveWith((s) =>
                      s.contains(WidgetState.selected)
                          ? AppColors.devDim.withValues(alpha: 0.5)
                          : AppColors.surfaceHi),
                  value: _showJoystick,
                  onChanged: (v) {
                    _setShowJoystick(v, socket);
                    setSheet(() {});
                  },
                  title: Text(
                    "Manual joystick",
                    style: GoogleFonts.rajdhani(
                      color: AppColors.textHi,
                      fontWeight: FontWeight.w700,
                      fontSize: 15,
                    ),
                  ),
                  subtitle: Text(
                    "Show the on-screen joystick for driving by hand. "
                    "Hidden by default so the robot runs in AUTO / NAV.",
                    style: GoogleFonts.rajdhani(color: AppColors.textLo),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  void _logEvent(String action) {
    try {
      FirebaseFirestore.instance.collection('robot_logs').add({
        'action': action,
        'timestamp': FieldValue.serverTimestamp(),
        'device': 'AGV Controller',
      });
    } catch (e) {
      debugPrint("Logging skipped: $e");
    }
  }

  void _toggleMode(RobotSocket socket) {
    setState(() => isAutoMode = !isAutoMode);
    if (isAutoMode) {
      socket.sendCommand("MODE:AUTO");
      _logEvent("Switched to AUTO Mode");
      _record(true, 'AUTO', socket);
    } else {
      socket.sendCommand("MODE:MANUAL");
      _logEvent("Switched to MANUAL Mode");
      // Only close a run that AUTO opened. A manual recording running at the
      // same time belongs to the operator and is theirs to stop.
      if (context.read<RunLogStore>().activeMode == 'AUTO') {
        _record(false, 'AUTO', socket);
      }
    }
  }

  void _emergencyStop(RobotSocket socket) {
    setState(() => isAutoMode = false);
    socket.sendCommand("MODE:STOP");
    _logEvent("EMERGENCY STOP");
    // The robot has stopped driving, so an AUTO run has ended whatever the
    // mode button says.
    if (context.read<RunLogStore>().activeMode == 'AUTO') {
      _record(false, 'AUTO', socket);
    }
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(
        SnackBar(
          behavior: SnackBarBehavior.floating,
          backgroundColor: const Color(0xFFE53935),
          duration: const Duration(seconds: 2),
          margin: const EdgeInsets.fromLTRB(16, 0, 16, 16),
          shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12)),
          content: Row(
            children: [
              const Icon(Icons.stop_circle, color: Colors.white, size: 22),
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  "EMERGENCY STOP — robot halted",
                  style: GoogleFonts.rajdhani(
                    color: Colors.white,
                    fontWeight: FontWeight.w700,
                    fontSize: 15,
                    letterSpacing: 0.5,
                  ),
                ),
              ),
            ],
          ),
        ),
      );
  }

  /// Consistent circular action button for the top bar.
  Widget _circleBtn({
    required IconData icon,
    required Color iconColor,
    required Color bg,
    required Color borderColor,
    VoidCallback? onTap,
    String? tooltip,
  }) {
    Widget btn = Material(
      color: bg,
      shape: CircleBorder(side: BorderSide(color: borderColor)),
      child: InkWell(
        customBorder: const CircleBorder(),
        onTap: onTap,
        child: SizedBox(
          width: 42,
          height: 42,
          child: Center(child: Icon(icon, color: iconColor, size: 20)),
        ),
      ),
    );
    if (tooltip != null) btn = Tooltip(message: tooltip, child: btn);
    return btn;
  }

  @override
  Widget build(BuildContext context) {
    final socket = Provider.of<RobotSocket>(context);
    final runs = context.watch<RunLogStore>();
    final Color accent = isAutoMode ? AppColors.amber : AppColors.lime;
    final bool hasCamera = socket.cameraImage != null;
    final bool online = socket.isConnected;

    return Scaffold(
      body: Stack(
        children: [
          // LAYER 1: Camera feed (fills screen)
          Positioned.fill(
            child: hasCamera
                ? Image.memory(
                    socket.cameraImage!,
                    gaplessPlayback: true,
                    fit: BoxFit.cover,
                  )
                : Container(
                    color: Colors.black,
                    child: Center(
                      child: Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          const Icon(Icons.videocam_off,
                              color: AppColors.textLo, size: 40),
                          const SizedBox(height: 8),
                          Text(
                            "NO CAMERA FEED",
                            style: GoogleFonts.rajdhani(
                                color: AppColors.textLo, letterSpacing: 2),
                          ),
                        ],
                      ),
                    ),
                  ),
          ),
          // LAYER 1.5: AI detection boxes over the live camera. Deliberately
          // only when there IS a camera feed -- boxes over the "no feed"
          // placeholder would be meaningless.
          if (hasCamera)
            Positioned.fill(
              child: DetectionOverlay(
                boxes: socket.detectionBoxes,
                frameAspect: socket.detectionFrameAspect,
                age: socket.detectionAge,
              ),
            ),
          // LAYER 2: Lidar radar overlay -- only over the empty/no-camera state
          if (!hasCamera)
            Positioned.fill(
              child: IgnorePointer(
                child: CustomPaint(painter: LidarPainter(socket.lidarPoints)),
              ),
            ),
          // LAYER 3: Modern floating top bar
          SafeArea(
            child: Padding(
              padding: const EdgeInsets.fromLTRB(16, 12, 16, 0),
              child: Row(
                children: [
                  // Mode pill (left) -- shrinks gracefully, never overflows.
                  // Also the hidden developer-settings unlock: 7 quick taps.
                  Flexible(
                    child: GestureDetector(
                      behavior: HitTestBehavior.opaque,
                      onTap: () => _onModePillTap(socket),
                      child: Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 14, vertical: 11),
                        decoration: BoxDecoration(
                          // MANUAL is filled violet, AUTO keeps the dark
                          // surface with an amber edge. A solid block of a
                          // colour used nowhere else marks the whole control
                          // as developer territory, not just its icon.
                          color: isAutoMode
                              ? AppColors.surface.withValues(alpha: 0.85)
                              : AppColors.devDim,
                          borderRadius: BorderRadius.circular(40),
                          border: Border.all(
                              color: isAutoMode
                                  ? accent.withValues(alpha: 0.6)
                                  : AppColors.dev),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(
                              isAutoMode
                                  ? Icons.smart_toy
                                  : Icons.sports_esports,
                              // White on the violet fill; amber on the dark.
                              color: isAutoMode ? accent : Colors.white,
                              size: 18,
                            ),
                            // Only AUTO is labelled. MANUAL is the developer
                            // path, so it stays an unlabelled violet marker
                            // rather than reading as a mode on offer.
                            if (isAutoMode) ...[
                              const SizedBox(width: 8),
                              Flexible(
                                child: Text(
                                  "AUTO",
                                  overflow: TextOverflow.ellipsis,
                                  style: GoogleFonts.rajdhani(
                                    color: accent,
                                    fontWeight: FontWeight.w700,
                                    fontSize: 15,
                                    letterSpacing: 1.5,
                                  ),
                                ),
                              ),
                            ],
                          ],
                        ),
                      ),
                    ),
                  ),
                  const Spacer(),
                  // Right action cluster -- consistent circular buttons
                  // Status, not a button (no onTap). Deliberately NOT green:
                  // green marks the icons the operator can actually press.
                  _circleBtn(
                    icon: online ? Icons.wifi : Icons.wifi_off,
                    iconColor: online ? AppColors.textHi : AppColors.offline,
                    bg: AppColors.surface.withValues(alpha: 0.85),
                    borderColor: AppColors.border,
                    tooltip: online ? "Connected" : "Disconnected",
                  ),
                  const SizedBox(width: 8),
                  _circleBtn(
                    icon: Icons.map,
                    iconColor: AppColors.lime,
                    bg: AppColors.surface.withValues(alpha: 0.85),
                    borderColor: AppColors.border,
                    tooltip: "GPS Map",
                    onTap: () => Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => const MapScreen()),
                    ),
                  ),
                  const SizedBox(width: 8),
                  _circleBtn(
                    icon: Icons.radar,
                    iconColor: AppColors.lime,
                    bg: AppColors.surface.withValues(alpha: 0.85),
                    borderColor: AppColors.border,
                    tooltip: "Lidar Map",
                    onTap: () => Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => const LidarMapScreen()),
                    ),
                  ),
                  const SizedBox(width: 8),
                  _circleBtn(
                    icon: Icons.timer_outlined,
                    iconColor: AppColors.lime,
                    bg: AppColors.surface.withValues(alpha: 0.85),
                    borderColor: AppColors.border,
                    tooltip: "Run logs",
                    onTap: () => Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => const LogsScreen()),
                    ),
                  ),
                  const SizedBox(width: 8),
                  _circleBtn(
                    icon: Icons.photo_library,
                    iconColor: AppColors.lime,
                    bg: AppColors.surface.withValues(alpha: 0.85),
                    borderColor: AppColors.border,
                    tooltip: "Saved Images",
                    onTap: () => Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => const GalleryScreen()),
                    ),
                  ),
                  const SizedBox(width: 8),
                  _circleBtn(
                    icon: Icons.stop,
                    iconColor: Colors.white,
                    bg: const Color(0xFFE53935),
                    borderColor: const Color(0xFFE53935),
                    tooltip: "Emergency Stop",
                    onTap: () => _emergencyStop(socket),
                  ),
                ],
              ),
            ),
          ),
          // LAYER 3.5: Obstacle warning banner (auto-hides when CLEAR)
          const SafeArea(
            child: Padding(
              padding: EdgeInsets.only(top: 66),
              child: Align(
                alignment: Alignment.topCenter,
                child: ObstacleWarning(),
              ),
            ),
          ),
          // LAYER 4: Bottom control panel
          Align(
            alignment: Alignment.bottomCenter,
            child: Container(
              width: double.infinity,
              padding: const EdgeInsets.fromLTRB(20, 18, 20, 8),
              decoration: BoxDecoration(
                color: AppColors.bg.withValues(alpha: 0.92),
                borderRadius:
                    const BorderRadius.vertical(top: Radius.circular(28)),
                border: Border(
                  top: BorderSide(
                      color: accent.withValues(alpha: 0.45), width: 1.5),
                ),
              ),
              child: SafeArea(
                top: false,
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Text(
                            "SYSTEM STATUS",
                            style: GoogleFonts.rajdhani(
                              color: AppColors.textLo,
                              fontSize: 12,
                              letterSpacing: 1.5,
                            ),
                          ),
                          const SizedBox(height: 2),
                          Text(
                            socket.currentStatus,
                            style: GoogleFonts.rajdhani(
                              color: AppColors.textHi,
                              fontWeight: FontWeight.w700,
                              fontSize: 18,
                            ),
                          ),
                          const SizedBox(height: 4),
                          Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Icon(Icons.satellite_alt,
                                  size: 13,
                                  color: socket.hasGps
                                      ? AppColors.lime
                                      : AppColors.textLo),
                              const SizedBox(width: 5),
                              Text(
                                socket.hasGps
                                    ? "GPS \u00B7 FIX ${socket.gpsFix} \u00B7 ${socket.gpsSats} SAT"
                                    : "GPS \u00B7 NO FIX",
                                style: GoogleFonts.rajdhani(
                                  color: socket.hasGps
                                      ? AppColors.textHi
                                      : AppColors.textLo,
                                  fontSize: 13,
                                  fontWeight: FontWeight.w600,
                                  letterSpacing: 0.5,
                                ),
                              ),
                            ],
                          ),
                          const SizedBox(height: 14),
                          ElevatedButton.icon(
                            onPressed: () => _toggleMode(socket),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: isAutoMode
                                  ? AppColors.amber
                                  : AppColors.surfaceHi,
                              foregroundColor: isAutoMode
                                  ? AppColors.onAmber
                                  : AppColors.textHi,
                              padding: const EdgeInsets.symmetric(
                                  horizontal: 18, vertical: 12),
                              shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(12)),
                            ),
                            icon: Icon(
                              isAutoMode
                                  ? Icons.stop_circle
                                  : Icons.play_circle,
                              size: 20,
                            ),
                            label: Text(
                              isAutoMode ? "STOP AUTO" : "START AUTO",
                              style: GoogleFonts.rajdhani(
                                fontWeight: FontWeight.w700,
                                letterSpacing: 1,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    // Manual joystick: hidden unless turned on in the
                    // developer settings (7 taps on the mode pill).
                    if (_showJoystick) ...[
                      const SizedBox(width: 12),
                      // Manual driving has no start or stop the app can
                      // detect, so a recorded manual run is marked by hand.
                      // Violet, like everything else that appears only
                      // because a developer setting is on.
                      Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          _circleBtn(
                            icon: runs.activeMode == 'MANUAL'
                                ? Icons.stop_circle
                                : Icons.fiber_manual_record,
                            iconColor: runs.activeMode == 'MANUAL'
                                ? AppColors.dev
                                : AppColors.textLo,
                            bg: AppColors.surface.withValues(alpha: 0.85),
                            borderColor: runs.activeMode == 'MANUAL'
                                ? AppColors.dev
                                : AppColors.border,
                            tooltip: runs.activeMode == 'MANUAL'
                                ? "Stop recording this run"
                                : (runs.isRecording
                                    ? "An AUTO run is already recording"
                                    : "Record a manual run"),
                            // Disabled while AUTO is recording: two runs at
                            // once would make both durations meaningless.
                            onTap: (runs.isRecording &&
                                    runs.activeMode != 'MANUAL')
                                ? null
                                : () => _record(
                                    runs.activeMode != 'MANUAL',
                                    'MANUAL',
                                    socket),
                          ),
                          const SizedBox(height: 4),
                          Text(
                            runs.activeMode == 'MANUAL' ? "REC" : "LOG",
                            style: GoogleFonts.rajdhani(
                              color: runs.activeMode == 'MANUAL'
                                  ? AppColors.dev
                                  : AppColors.textLo,
                              fontSize: 10,
                              fontWeight: FontWeight.w700,
                              letterSpacing: 1,
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(width: 16),
                      Opacity(
                        opacity: isAutoMode ? 0.3 : 1.0,
                        child: Container(
                          padding: const EdgeInsets.all(10),
                          decoration: BoxDecoration(
                            color: AppColors.surface.withValues(alpha: 0.6),
                            shape: BoxShape.circle,
                            // Violet ring: this control is only on screen
                            // because a developer setting turned it on.
                            border: Border.all(
                                color: AppColors.devDim, width: 1.5),
                          ),
                          child: SizedBox(
                            width: 130,
                            height: 130,
                            child: Joystick(
                              mode: JoystickMode.all,
                              listener: (details) {
                                if (!isAutoMode) {
                                  socket.sendCommand(
                                      "${details.x},${details.y}");
                                }
                              },
                            ),
                          ),
                        ),
                      ),
                    ],
                  ],
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}