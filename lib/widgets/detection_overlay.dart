import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

/// Draws the AI's detection boxes over the live camera view.
///
/// The camera Image uses BoxFit.cover, which scales the frame to FILL the
/// widget and centre-crops the overflow. Box corners arrive normalised 0-1
/// against the source frame, so they must go through that same scale+crop
/// transform -- mapping them straight onto the widget rect would put every
/// box in the wrong place whenever the widget's aspect differs from the
/// camera's (i.e. basically always, on a phone screen).
///
/// The boxes are NOT live: inference takes 1.1-1.7s and runs about every 3s,
/// so they always trail the video. That is why the age is drawn on screen --
/// the alternative (silently showing second-old boxes as if current) would be
/// worse than not showing them at all.
class DetectionOverlay extends StatelessWidget {
  final List<List<double>> boxes; // each: [x1, y1, x2, y2, conf] normalised
  final double frameAspect; // source frame w/h
  final Duration? age;

  const DetectionOverlay({
    super.key,
    required this.boxes,
    required this.frameAspect,
    this.age,
  });

  @override
  Widget build(BuildContext context) {
    if (boxes.isEmpty) return const SizedBox.shrink();
    return IgnorePointer(
      child: CustomPaint(
        painter: _BoxPainter(boxes, frameAspect, age),
        size: Size.infinite,
      ),
    );
  }
}

class _BoxPainter extends CustomPainter {
  final List<List<double>> boxes;
  final double frameAspect;
  final Duration? age;

  _BoxPainter(this.boxes, this.frameAspect, this.age);

  @override
  void paint(Canvas canvas, Size size) {
    if (size.width <= 0 || size.height <= 0) return;

    // Replicate BoxFit.cover: scale so the frame fills the widget, then the
    // overflow is cropped evenly on both sides / top and bottom.
    final widgetAspect = size.width / size.height;
    double drawW, drawH;
    if (widgetAspect > frameAspect) {
      drawW = size.width; // width-limited: crop top/bottom
      drawH = size.width / frameAspect;
    } else {
      drawH = size.height; // height-limited: crop left/right
      drawW = size.height * frameAspect;
    }
    final dx = (size.width - drawW) / 2.0; // negative = cropped
    final dy = (size.height - drawH) / 2.0;

    // Fade as the boxes age, so stale ones visibly lose confidence.
    final ageS = (age?.inMilliseconds ?? 0) / 1000.0;
    final fade = (1.0 - (ageS / 6.0)).clamp(0.35, 1.0);
    final color = Color.lerp(
        const Color(0xFF6FD24A), const Color(0xFFF5B301), (ageS / 6.0).clamp(0.0, 1.0))!;

    final stroke = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.5
      ..color = color.withValues(alpha: fade);

    for (final b in boxes) {
      final rect = Rect.fromLTRB(
        dx + b[0] * drawW,
        dy + b[1] * drawH,
        dx + b[2] * drawW,
        dy + b[3] * drawH,
      );
      canvas.drawRRect(
          RRect.fromRectAndRadius(rect, const Radius.circular(4)), stroke);

      final label = "Obstacle ${b[4].toStringAsFixed(2)}";
      final tp = TextPainter(
        text: TextSpan(
          text: label,
          style: GoogleFonts.rajdhani(
            color: Colors.white.withValues(alpha: fade),
            fontSize: 13,
            fontWeight: FontWeight.w700,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      final lw = tp.width + 10, lh = tp.height + 4;
      // keep the label on screen when the box runs off the top
      final ly = rect.top - lh >= 0 ? rect.top - lh : rect.top;
      canvas.drawRect(
        Rect.fromLTWH(rect.left, ly, lw, lh),
        Paint()..color = color.withValues(alpha: fade * 0.85),
      );
      tp.paint(canvas, Offset(rect.left + 5, ly + 2));
    }

    // Age chip -- these boxes are seconds behind the video, say so.
    if (age != null) {
      final txt = "AI ${ageS.toStringAsFixed(1)}s ago";
      final tp = TextPainter(
        text: TextSpan(
          text: txt,
          style: GoogleFonts.rajdhani(
            color: Colors.white,
            fontSize: 12,
            fontWeight: FontWeight.w600,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      const pad = 6.0;
      final r = Rect.fromLTWH(
          8, size.height - tp.height - 2 * pad - 8, tp.width + 2 * pad, tp.height + 2 * pad);
      canvas.drawRRect(
        RRect.fromRectAndRadius(r, const Radius.circular(6)),
        Paint()..color = Colors.black.withValues(alpha: 0.55),
      );
      tp.paint(canvas, Offset(r.left + pad, r.top + pad));
    }
  }

  @override
  bool shouldRepaint(covariant _BoxPainter old) =>
      old.boxes != boxes || old.age != age || old.frameAspect != frameAspect;
}
