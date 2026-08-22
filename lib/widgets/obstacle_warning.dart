import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../services/robot_socket.dart';

/// Floating banner showing the robot's obstacle status (from the Lidar).
/// - Hidden when CLEAR or disconnected
/// - Amber for CAUTION, Red for DANGER
/// - Shows the direction (zone) and distance of the closest obstacle
class ObstacleWarning extends StatelessWidget {
  const ObstacleWarning({super.key});

  // Robot zone code -> readable direction
  String _zoneLabel(String zone) {
    switch (zone) {
      case 'front':
        return 'FRONT';
      case 'front_l':
        return 'FRONT-LEFT';
      case 'front_r':
        return 'FRONT-RIGHT';
      case 'left':
        return 'LEFT';
      case 'right':
        return 'RIGHT';
      case 'back':
        return 'REAR';
      default:
        return zone.toUpperCase();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotSocket>(
      builder: (context, socket, child) {
        final String level = socket.obstacleLevel;

        // Nothing to show when the path is clear or robot is offline.
        if (level == 'CLEAR' || !socket.isConnected) {
          return const SizedBox.shrink();
        }

        final bool danger = level == 'DANGER';
        final Color c =
            danger ? const Color(0xFFE53935) : const Color(0xFFFFB300);
        final IconData icon =
            danger ? Icons.dangerous : Icons.warning_amber_rounded;
        final String dir = _zoneLabel(socket.obstacleZone);
        final String dist = socket.obstacleDist > 0
            ? ' • ${socket.obstacleDist.toStringAsFixed(2)}m'
            : '';

        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
          decoration: BoxDecoration(
            color: c.withValues(alpha: 0.94),
            borderRadius: BorderRadius.circular(14),
            boxShadow: [
              BoxShadow(
                color: c.withValues(alpha: 0.55),
                blurRadius: 18,
                spreadRadius: 1,
              ),
            ],
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, color: Colors.white, size: 22),
              const SizedBox(width: 10),
              Text(
                danger ? 'DANGER' : 'CAUTION',
                style: GoogleFonts.rajdhani(
                  color: Colors.white,
                  fontWeight: FontWeight.w800,
                  fontSize: 18,
                  letterSpacing: 1.5,
                ),
              ),
              const SizedBox(width: 8),
              Text(
                '$dir$dist',
                style: GoogleFonts.rajdhani(
                  color: Colors.white,
                  fontWeight: FontWeight.w600,
                  fontSize: 15,
                  letterSpacing: 0.5,
                ),
              ),
            ],
          ),
        );
      },
    );
  }
}