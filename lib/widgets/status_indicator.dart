import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';

class StatusIndicator extends StatelessWidget {
  const StatusIndicator({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotSocket>(
      builder: (context, socket, child) {
        final bool online = socket.isConnected;
        final Color c = online ? AppColors.online : AppColors.offline;
        final String label = online ? "ONLINE" : "OFFLINE";
        final IconData icon = online ? Icons.wifi : Icons.wifi_off;

        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
          decoration: BoxDecoration(
            color: AppColors.surface.withValues(alpha: 0.85),
            border: Border.all(color: c.withValues(alpha: 0.7)),
            borderRadius: BorderRadius.circular(14),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(icon, color: c, size: 14),
              const SizedBox(width: 8),
              Text(
                label,
                style: GoogleFonts.rajdhani(
                  color: c,
                  fontWeight: FontWeight.w700,
                  fontSize: 13,
                  letterSpacing: 1.5,
                ),
              ),
            ],
          ),
        );
      },
    );
  }
}