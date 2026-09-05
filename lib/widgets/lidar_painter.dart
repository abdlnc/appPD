import 'dart:math';
import 'package:flutter/material.dart';
import '../theme/app_theme.dart';

class LidarPainter extends CustomPainter {
  final List<Point<double>> points;

  LidarPainter(this.points);

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    const maxRange = 3000.0; // 3000mm = 3 meters
    final scale = (size.width / 2) / maxRange;

    // Radar rings (subtle lime)
    final ringPaint = Paint()
      ..color = AppColors.lime.withValues(alpha: 0.15)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.0;
    canvas.drawCircle(center, 1000 * scale, ringPaint); // 1m
    canvas.drawCircle(center, 2000 * scale, ringPaint); // 2m
    canvas.drawCircle(center, 3000 * scale, ringPaint); // 3m

    // Robot center (amber)
    canvas.drawCircle(center, 5.0, Paint()..color = AppColors.amber);

    // Detected points (lime)
    final pointPaint = Paint()
      ..color = AppColors.lime
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.round;

    for (var point in points) {
      // Polar (angle/dist) -> Cartesian. Robot convention (see the zone map
      // in control_node.py): 0deg = FRONT, 90 = LEFT, 180 = BACK, 270 = RIGHT
      // -- angles increase counter-clockwise (standard ROS REP-103).
      // Flutter's canvas has +y pointing DOWN, which flips the handedness, so
      // dx must be NEGATED -- otherwise left and right come out mirrored
      // (front/back still look right, which is why this hid for so long).
      double radians = (point.x - 90) * (pi / 180.0);
      double dist = point.y;
      if (dist > 0 && dist < maxRange) {
        double r = dist * scale;
        double dx = center.dx - r * cos(radians); // negated: CCW -> y-down canvas
        double dy = center.dy + r * sin(radians);
        canvas.drawCircle(Offset(dx, dy), 2.5, pointPaint);
      }
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}