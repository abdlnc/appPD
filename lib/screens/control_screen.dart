import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';
import '../widgets/lidar_painter.dart';
import '../widgets/obstacle_warning.dart';
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
    } else {
      socket.sendCommand("MODE:MANUAL");
      _logEvent("Switched to MANUAL Mode");
    }
  }

  void _emergencyStop(RobotSocket socket) {
    setState(() => isAutoMode = false);
    socket.sendCommand("MODE:STOP");
    _logEvent("EMERGENCY STOP");
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
                  // Mode pill (left) -- shrinks gracefully, never overflows
                  Flexible(
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 14, vertical: 11),
                      decoration: BoxDecoration(
                        color: AppColors.surface.withValues(alpha: 0.85),
                        borderRadius: BorderRadius.circular(40),
                        border:
                            Border.all(color: accent.withValues(alpha: 0.6)),
                      ),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Icon(
                            isAutoMode
                                ? Icons.smart_toy
                                : Icons.sports_esports,
                            color: accent,
                            size: 18,
                          ),
                          const SizedBox(width: 8),
                          Flexible(
                            child: Text(
                              isAutoMode ? "AUTO" : "MANUAL",
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
                      ),
                    ),
                  ),
                  const Spacer(),
                  // Right action cluster -- consistent circular buttons
                  _circleBtn(
                    icon: online ? Icons.wifi : Icons.wifi_off,
                    iconColor: online ? AppColors.online : AppColors.offline,
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
                    const SizedBox(width: 16),
                    Opacity(
                      opacity: isAutoMode ? 0.3 : 1.0,
                      child: Container(
                        padding: const EdgeInsets.all(10),
                        decoration: BoxDecoration(
                          color: AppColors.surface.withValues(alpha: 0.6),
                          shape: BoxShape.circle,
                          border: Border.all(color: AppColors.border),
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
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}