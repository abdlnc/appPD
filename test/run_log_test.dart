import 'package:flutter_test/flutter_test.dart';
import 'package:robot_controller/models/run_log.dart';

RunLog run({
  required int seconds,
  String mode = 'AUTO',
  double? slat,
  double? slon,
  double? elat,
  double? elon,
}) {
  final t = DateTime(2026, 10, 4, 9, 0, 0);
  return RunLog(
    mode: mode,
    start: t,
    end: t.add(Duration(seconds: seconds)),
    startLat: slat,
    startLon: slon,
    endLat: elat,
    endLon: elon,
  );
}

void main() {
  group('duration', () {
    test('seconds only, below a minute', () {
      expect(run(seconds: 42).durationLabel, '42s');
      expect(run(seconds: 0).durationLabel, '0s');
    });

    test('minutes and seconds', () {
      expect(run(seconds: 252).durationLabel, '4m 12s');
      expect(run(seconds: 60).durationLabel, '1m 00s');
    });

    test('hours, with padded minutes and seconds', () {
      expect(run(seconds: 3852).durationLabel, '1h 04m 12s');
      expect(run(seconds: 7200).durationLabel, '2h 00m 00s');
    });

    test('a run spanning midnight is still measured correctly', () {
      final r = RunLog(
        mode: 'AUTO',
        start: DateTime(2026, 10, 4, 23, 58, 0),
        end: DateTime(2026, 10, 5, 0, 3, 0),
      );
      expect(r.duration, const Duration(minutes: 5));
      expect(r.durationLabel, '5m 00s');
    });
  });

  group('GPS at each end', () {
    test('both ends recorded', () {
      final r = run(
          seconds: 60,
          slat: 14.632479,
          slon: 121.090634,
          elat: 14.633,
          elon: 121.0907);
      expect(r.hasStartFix, isTrue);
      expect(r.hasEndFix, isTrue);
      expect(r.startCoordLabel, '14.632, 121.091');
    });

    test('no fix at either end still logs the run', () {
      final r = run(seconds: 60);
      expect(r.hasStartFix, isFalse);
      expect(r.hasEndFix, isFalse);
      expect(r.startCoordLabel, 'no GPS');
      expect(r.endCoordLabel, 'no GPS');
      expect(r.durationLabel, '1m 00s');
    });

    test('started indoors, finished outside: the end point is kept', () {
      final r = run(seconds: 60, elat: 14.6330, elon: 121.0907);
      expect(r.hasStartFix, isFalse);
      expect(r.hasEndFix, isTrue);
      expect(r.endCoordLabel, '14.633, 121.091');
    });

    test('displacement needs BOTH ends', () {
      expect(run(seconds: 60, slat: 14.6, slon: 121.0).displacementM, isNull);
      expect(run(seconds: 60, elat: 14.6, elon: 121.0).displacementM, isNull);
      expect(run(seconds: 60).displacementM, isNull);
    });

    test('displacement is the straight line between the ends', () {
      // 0.001 degrees of latitude is about 110 m
      final r = run(
          seconds: 60, slat: 14.6320, slon: 121.0900, elat: 14.6330, elon: 121.0900);
      expect(r.displacementM!, closeTo(110.5, 2));
    });

    test('a run that returns to its start reads ~0', () {
      final r = run(
          seconds: 60, slat: 14.632, slon: 121.09, elat: 14.632, elon: 121.09);
      expect(r.displacementM!, closeTo(0, 0.01));
    });
  });

  group('storage', () {
    test('survives a round trip through JSON', () {
      final r = run(
          seconds: 125,
          mode: 'MANUAL',
          slat: 14.632479,
          slon: 121.090634,
          elat: -33.8688,
          elon: 151.2093);
      final back = RunLog.decodeList(RunLog.encodeList([r])).single;
      expect(back.mode, 'MANUAL');
      expect(back.start, r.start);
      expect(back.end, r.end);
      expect(back.startLat, closeTo(14.632479, 1e-9));
      expect(back.endLon, closeTo(151.2093, 1e-9));
      expect(back.durationLabel, '2m 05s');
    });

    test('a run with no GPS round-trips as no GPS, not as zeroes', () {
      final back = RunLog.decodeList(RunLog.encodeList([run(seconds: 10)])).single;
      expect(back.startLat, isNull);
      expect(back.hasStartFix, isFalse);
    });

    test('order is preserved', () {
      final list = [run(seconds: 1), run(seconds: 2), run(seconds: 3)];
      final back = RunLog.decodeList(RunLog.encodeList(list));
      expect(back.map((r) => r.duration.inSeconds).toList(), [1, 2, 3]);
    });

    test('corrupt storage costs the bad entries, not the whole history', () {
      const raw = '[{"mode":"AUTO","start":1,"end":2},'
          '{"mode":"AUTO"},'          // missing timestamps
          '{"start":5,"end":6},'      // missing mode
          '"not an object",'
          '{"mode":"MANUAL","start":10,"end":20}]';
      final back = RunLog.decodeList(raw);
      expect(back.length, 2);
      expect(back.map((r) => r.mode).toList(), ['AUTO', 'MANUAL']);
    });

    test('empty, null and malformed input give an empty list', () {
      expect(RunLog.decodeList(null), isEmpty);
      expect(RunLog.decodeList(''), isEmpty);
      expect(RunLog.decodeList('not json'), isEmpty);
      expect(RunLog.decodeList('{"not":"a list"}'), isEmpty);
    });
  });
}
