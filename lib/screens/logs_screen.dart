import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

import '../models/run_log.dart';
import '../services/run_log_store.dart';
import '../theme/app_theme.dart';

/// How long each run lasted, and where it began and ended.
class LogsScreen extends StatelessWidget {
  const LogsScreen({super.key});

  static String _clock(DateTime t) =>
      '${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}';

  static String _day(DateTime t) {
    const m = [
      'Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
      'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec',
    ];
    return '${t.day} ${m[t.month - 1]} ${t.year}';
  }

  @override
  Widget build(BuildContext context) {
    final store = context.watch<RunLogStore>();
    final runs = store.runs;

    return Scaffold(
      backgroundColor: AppColors.bg,
      appBar: AppBar(
        backgroundColor: AppColors.surface,
        foregroundColor: AppColors.textHi,
        title: Text("RUN LOGS",
            style: GoogleFonts.rajdhani(
                fontWeight: FontWeight.w700, letterSpacing: 1.5)),
        actions: [
          IconButton(
            icon: const Icon(Icons.delete_sweep_outlined),
            tooltip: "Clear all logs",
            onPressed: runs.isEmpty
                ? null
                : () async {
                    final ok = await showDialog<bool>(
                      context: context,
                      builder: (ctx) => AlertDialog(
                        backgroundColor: AppColors.surface,
                        title: Text("Clear every run log?",
                            style: GoogleFonts.rajdhani(
                                color: AppColors.textHi,
                                fontWeight: FontWeight.w700)),
                        content: Text(
                          "${runs.length} run${runs.length == 1 ? '' : 's'} "
                          "will be removed. This cannot be undone.",
                          style:
                              GoogleFonts.rajdhani(color: AppColors.textLo),
                        ),
                        actions: [
                          TextButton(
                            onPressed: () => Navigator.pop(ctx, false),
                            child: Text("CANCEL",
                                style: GoogleFonts.rajdhani(
                                    color: AppColors.textLo)),
                          ),
                          TextButton(
                            onPressed: () => Navigator.pop(ctx, true),
                            child: Text("CLEAR",
                                style: GoogleFonts.rajdhani(
                                    color: const Color(0xFFE53935),
                                    fontWeight: FontWeight.w700)),
                          ),
                        ],
                      ),
                    );
                    if (ok == true) await store.clear();
                  },
          ),
        ],
      ),
      body: runs.isEmpty && !store.isRecording
          ? Center(
              child: Padding(
                padding: const EdgeInsets.all(28),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Icon(Icons.timer_outlined,
                        color: AppColors.textLo, size: 40),
                    const SizedBox(height: 10),
                    Text("No runs recorded yet",
                        style: GoogleFonts.rajdhani(
                            color: AppColors.textHi,
                            fontSize: 16,
                            fontWeight: FontWeight.w700)),
                    const SizedBox(height: 6),
                    Text(
                      "An AUTO run is timed from START AUTO until it stops. "
                      "Manual runs are recorded from the button beside the "
                      "joystick.",
                      textAlign: TextAlign.center,
                      style: GoogleFonts.rajdhani(color: AppColors.textLo),
                    ),
                  ],
                ),
              ),
            )
          : ListView.builder(
              padding: const EdgeInsets.fromLTRB(12, 12, 12, 24),
              itemCount: runs.length + (store.isRecording ? 1 : 0),
              itemBuilder: (context, i) {
                if (store.isRecording && i == 0) return _activeCard(store);
                final run = runs[store.isRecording ? i - 1 : i];
                return _runCard(run);
              },
            ),
    );
  }

  /// The run in progress, shown at the top so it is obvious something is
  /// still being timed. Its duration is deliberately not live here -- the
  /// control screen carries the ticking clock.
  Widget _activeCard(RunLogStore store) {
    final auto = store.activeMode == 'AUTO';
    final start = store.activeStart!;
    return Container(
      margin: const EdgeInsets.only(bottom: 10),
      padding: const EdgeInsets.fromLTRB(14, 12, 14, 12),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(10),
        border: Border.all(
            color: auto ? AppColors.amber : AppColors.dev, width: 1.5),
      ),
      child: Row(
        children: [
          Icon(auto ? Icons.smart_toy : Icons.sports_esports,
              color: auto ? AppColors.amber : AppColors.dev, size: 18),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              "${store.activeMode} run in progress - started ${_clock(start)}",
              style: GoogleFonts.rajdhani(
                  color: AppColors.textHi, fontWeight: FontWeight.w700),
            ),
          ),
        ],
      ),
    );
  }

  Widget _runCard(RunLog run) {
    final auto = run.mode == 'AUTO';
    final accent = auto ? AppColors.amber : AppColors.dev;
    final disp = run.displacementM;
    return Container(
      margin: const EdgeInsets.only(bottom: 10),
      padding: const EdgeInsets.fromLTRB(14, 12, 14, 12),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: AppColors.border),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 7, vertical: 2),
                decoration: BoxDecoration(
                  color: accent.withValues(alpha: 0.15),
                  borderRadius: BorderRadius.circular(4),
                  border: Border.all(color: accent.withValues(alpha: 0.5)),
                ),
                child: Text(run.mode,
                    style: GoogleFonts.rajdhani(
                        color: accent,
                        fontSize: 11,
                        fontWeight: FontWeight.w700,
                        letterSpacing: 1)),
              ),
              const SizedBox(width: 8),
              Text(run.durationLabel,
                  style: GoogleFonts.rajdhani(
                      color: AppColors.textHi,
                      fontSize: 18,
                      fontWeight: FontWeight.w700)),
              const Spacer(),
              Text("${_day(run.start)}  ${_clock(run.start)}-${_clock(run.end)}",
                  style: GoogleFonts.rajdhani(
                      color: AppColors.textLo, fontSize: 12)),
            ],
          ),
          const SizedBox(height: 8),
          _point(Icons.trip_origin, "from", run.startCoordLabel,
              run.hasStartFix),
          const SizedBox(height: 3),
          _point(Icons.place, "to", run.endCoordLabel, run.hasEndFix),
          if (disp != null) ...[
            const SizedBox(height: 5),
            Text(
              disp < 1
                  ? "finished where it started"
                  : "${disp.toStringAsFixed(1)} m between start and end "
                      "(straight line, not distance driven)",
              style: GoogleFonts.rajdhani(
                  color: AppColors.textLo, fontSize: 12),
            ),
          ],
        ],
      ),
    );
  }

  Widget _point(IconData icon, String label, String value, bool hasFix) {
    return Row(
      children: [
        Icon(icon,
            size: 13, color: hasFix ? AppColors.lime : AppColors.textLo),
        const SizedBox(width: 6),
        SizedBox(
          width: 34,
          child: Text(label,
              style:
                  GoogleFonts.rajdhani(color: AppColors.textLo, fontSize: 12)),
        ),
        Expanded(
          child: Text(value,
              style: GoogleFonts.rajdhani(
                  color: hasFix ? AppColors.textHi : AppColors.textLo,
                  fontSize: 13,
                  fontWeight: hasFix ? FontWeight.w600 : FontWeight.w400)),
        ),
      ],
    );
  }
}
