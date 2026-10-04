import 'package:flutter_test/flutter_test.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:robot_controller/services/run_log_store.dart';

/// ~1.11 m per 0.00001 degrees of latitude, which makes the steps below easy
/// to reason about against minStepM (1.0 m) and maxJumpM (40 m).
const double lat0 = 14.63200;
const double lon0 = 121.09000;
double latPlusMetres(double m) => lat0 + (m / 110540.0);

void main() {
  setUp(() => SharedPreferences.setMockInitialValues({}));

  Future<RunLogStore> started({double? lat = lat0, double? lon = lon0}) async {
    final s = RunLogStore();
    await s.load();
    await s.start('AUTO', lat: lat, lon: lon);
    return s;
  }

  test('a straight run sums its steps', () async {
    final s = await started();
    s.noteFix(latPlusMetres(5), lon0);
    s.noteFix(latPlusMetres(10), lon0);
    s.noteFix(latPlusMetres(15), lon0);
    final run = await s.stop(lat: latPlusMetres(15), lon: lon0);
    expect(run!.distanceM!, closeTo(15, 0.5));
  });

  test('an out-and-back run counts the ground, not the displacement',
      () async {
    final s = await started();
    s.noteFix(latPlusMetres(20), lon0);
    s.noteFix(lat0, lon0); // back to the start
    final run = await s.stop(lat: lat0, lon: lon0);
    expect(run!.distanceM!, closeTo(40, 1));
    expect(run.displacementM!, closeTo(0, 0.5)); // same place it began
  });

  test('jitter below the minimum step adds nothing', () async {
    final s = await started();
    // A stationary receiver wandering ~0.3 m, forty times over.
    for (var i = 0; i < 40; i++) {
      s.noteFix(latPlusMetres(i.isEven ? 0.3 : 0.0), lon0);
    }
    final run = await s.stop(lat: lat0, lon: lon0);
    expect(run!.distanceM, 0.0); // measured, and it did not move
  });

  test('a GPS spike is rejected instead of adding a false kilometre',
      () async {
    final s = await started();
    s.noteFix(latPlusMetres(5), lon0);
    s.noteFix(latPlusMetres(5000), lon0); // 5 km jump: impossible
    s.noteFix(latPlusMetres(10), lon0);
    final run = await s.stop(lat: latPlusMetres(10), lon: lon0);
    expect(run!.distanceM!, closeTo(10, 1));
  });

  test('a run with no GPS at all reports unknown, not zero', () async {
    final s = await started(lat: null, lon: null);
    final run = await s.stop();
    expect(run!.distanceM, isNull);
    expect(run.hasDistance, isFalse);
  });

  test('GPS arriving mid-run starts measuring from there', () async {
    final s = await started(lat: null, lon: null);
    s.noteFix(lat0, lon0); // first fix: a starting point, no distance yet
    s.noteFix(latPlusMetres(8), lon0);
    final run = await s.stop(lat: latPlusMetres(8), lon: lon0);
    expect(run!.distanceM!, closeTo(8, 0.5));
  });

  test('fixes outside a run are ignored', () async {
    final s = RunLogStore();
    await s.load();
    s.noteFix(lat0, lon0);
    s.noteFix(latPlusMetres(100), lon0);
    expect(s.isRecording, isFalse);
    expect(await s.stop(), isNull); // nothing to stop, nothing logged
  });

  test('a second start does not discard the run in progress', () async {
    final s = await started();
    final firstStart = s.activeStart;
    await s.start('MANUAL', lat: lat0, lon: lon0);
    expect(s.activeMode, 'AUTO');
    expect(s.activeStart, firstStart);
  });

  test('an in-progress run and its distance survive a restart', () async {
    final s = await started();
    s.noteFix(latPlusMetres(12), lon0);
    // A fresh store reading the same storage, as after the app is killed.
    final revived = RunLogStore();
    await revived.load();
    expect(revived.isRecording, isTrue);
    expect(revived.activeMode, 'AUTO');
    final run = await revived.stop(lat: latPlusMetres(12), lon: lon0);
    expect(run!.distanceM!, closeTo(12, 1));
  });

  test('finished runs are kept newest first', () async {
    final s = await started();
    await s.stop(lat: lat0, lon: lon0);
    await s.start('MANUAL', lat: lat0, lon: lon0);
    await s.stop(lat: lat0, lon: lon0);
    expect(s.runs.length, 2);
    expect(s.runs.first.mode, 'MANUAL');
  });
}
