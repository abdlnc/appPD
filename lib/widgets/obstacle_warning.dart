import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../services/robot_socket.dart';

/// Floating banners for conditions the driver needs to know about.
/// Stacked top-centre, most severe first, each auto-hiding on its own:
///   * NO PATH        - robot boxed in; avoidance has no move left to make
///   * DANGER/CAUTION - closest Lidar obstacle
///   * NO GPS SIGNAL  - no fix, so NAV mode cannot drive at all
/// All hidden while disconnected.
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

  Widget _banner({
    required Color color,
    required IconData icon,
    required String title,
    required String detail,
  }) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.94),
        borderRadius: BorderRadius.circular(14),
        boxShadow: [
          BoxShadow(
            color: color.withValues(alpha: 0.55),
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
            title,
            style: GoogleFonts.rajdhani(
              color: Colors.white,
              fontWeight: FontWeight.w800,
              fontSize: 18,
              letterSpacing: 1.5,
            ),
          ),
          if (detail.isNotEmpty) ...[
            const SizedBox(width: 8),
            Flexible(
              child: Text(
                detail,
                style: GoogleFonts.rajdhani(
                  color: Colors.white,
                  fontWeight: FontWeight.w600,
                  fontSize: 15,
                  letterSpacing: 0.5,
                ),
              ),
            ),
          ],
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotSocket>(
      builder: (context, socket, child) {
        // Nothing is meaningful while offline.
        if (!socket.isConnected) return const SizedBox.shrink();

        final banners = <Widget>[];

        // 1. NO PATH -- most severe. Supersedes the DANGER banner rather than
        // stacking with it: being boxed in always implies a DANGER-range
        // reading, so showing both would just be the same news twice.
        if (socket.pathBlocked) {
          banners.add(_banner(
            color: const Color(0xFFC62828), // deeper red than DANGER
            icon: Icons.block,
            title: 'NO PATH',
            detail: 'boxed in - no way forward, back or around',
          ));
        } else {
          final level = socket.obstacleLevel;
          if (level == 'DANGER' || level == 'CAUTION') {
            final danger = level == 'DANGER';
            final dist = socket.obstacleDist > 0
                ? ' - ${socket.obstacleDist.toStringAsFixed(2)}m'
                : '';
            banners.add(_banner(
              color: danger ? const Color(0xFFE53935) : const Color(0xFFFFB300),
              icon: danger ? Icons.dangerous : Icons.warning_amber_rounded,
              title: danger ? 'DANGER' : 'CAUTION',
              detail: '${_zoneLabel(socket.obstacleZone)}$dist',
            ));
          }
        }

        // 2. NO GPS SIGNAL -- independent of any obstacle state. Blue-grey on
        // purpose: this is not a collision hazard, and colouring it red/amber
        // would blend it in with the ones that are.
        if (socket.noGpsSignal) {
          final sats = socket.gpsSatsView > 0
              ? '${socket.gpsSatsView} sats in view - NAV unavailable'
              : 'searching - NAV unavailable';
          banners.add(_banner(
            color: const Color(0xFF546E7A),
            icon: Icons.satellite_alt,
            title: 'NO GPS SIGNAL',
            detail: sats,
          ));
        }

        if (banners.isEmpty) return const SizedBox.shrink();

        return Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            for (int i = 0; i < banners.length; i++) ...[
              if (i > 0) const SizedBox(height: 8),
              banners[i],
            ],
          ],
        );
      },
    );
  }
}
