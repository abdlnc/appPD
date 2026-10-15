import 'package:flutter_test/flutter_test.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:robot_controller/services/run_log_store.dart';
import 'package:robot_controller/services/robot_socket.dart';

/// Records what would have been sent to the robot, with a connection that
/// the test switches on and off by hand.
class FakeSocket extends RobotSocket {
  bool connected = false;
  final sent = <String>[];
  @override
  bool get isConnected => connected;
  @override
  void sendCommand(String command) => sent.add(command);
  void setConnected(bool v) {
    connected = v;
    notifyListeners();
  }
}

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

  group('deleting one run', () {
    Future<RunLogStore> withThree() async {
      final s = RunLogStore();
      await s.load();
      for (final m in ['AUTO', 'MANUAL', 'AUTO']) {
        await s.start(m, lat: lat0, lon: lon0);
        await s.stop(lat: lat0, lon: lon0);
      }
      return s;  // newest first: AUTO, MANUAL, AUTO
    }

    test('removes exactly that run and reports where it was', () async {
      final s = await withThree();
      final middle = s.runs[1];
      final at = await s.remove(middle);
      expect(at, 1);
      expect(s.runs.length, 2);
      expect(s.runs.any((r) => identical(r, middle)), isFalse);
    });

    test('the deletion is saved, not just hidden', () async {
      final s = await withThree();
      await s.remove(s.runs[0]);
      final reloaded = RunLogStore();
      await reloaded.load();
      expect(reloaded.runs.length, 2);
    });

    test('of two identical-looking runs, only the chosen one goes', () async {
      // Same mode, same position, both started this second -- equal in
      // every field. Deleting by value could take the wrong one.
      final s = await withThree();
      final keep = s.runs[0], drop = s.runs[2];
      await s.remove(drop);
      expect(s.runs.any((r) => identical(r, keep)), isTrue);
      expect(s.runs.any((r) => identical(r, drop)), isFalse);
    });

    test('undo puts it back in the same place', () async {
      final s = await withThree();
      final middle = s.runs[1];
      final at = await s.remove(middle);
      await s.restore(middle, at);
      expect(s.runs.length, 3);
      expect(identical(s.runs[1], middle), isTrue);
    });

    test('undo pressed twice does not duplicate the run', () async {
      final s = await withThree();
      final r = s.runs[0];
      final at = await s.remove(r);
      await s.restore(r, at);
      await s.restore(r, at);
      expect(s.runs.length, 3);
    });

    test('undo survives the history changing in between', () async {
      final s = await withThree();
      final last = s.runs[2];
      final at = await s.remove(last);         // was at index 2
      await s.remove(s.runs[0]);               // history shrinks to 1
      await s.restore(last, at);               // 2 is now out of range
      expect(s.runs.length, 2);
      expect(identical(s.runs.last, last), isTrue);
    });

    test('removing a run that is not there changes nothing', () async {
      final s = await withThree();
      final r = s.runs[0];
      await s.remove(r);
      expect(await s.remove(r), -1);
      expect(s.runs.length, 2);
    });
  });

  group('image capture follows the run', () {
    Future<(RunLogStore, FakeSocket)> bound({bool online = true}) async {
      final s = RunLogStore();
      await s.load();
      final sock = FakeSocket()..connected = online;
      s.bind(sock);
      sock.sent.clear();   // ignore the state sent on binding
      return (s, sock);
    }

    test('starting a run turns capture on', () async {
      final (s, sock) = await bound();
      await s.start('AUTO', lat: lat0, lon: lon0);
      expect(sock.sent, ['CAPTURE:ON']);
    });

    test('a manual recording turns it on too', () async {
      final (s, sock) = await bound();
      await s.start('MANUAL');
      expect(sock.sent, ['CAPTURE:ON']);
    });

    test('ending the run turns it off', () async {
      final (s, sock) = await bound();
      await s.start('AUTO');
      await s.stop();
      expect(sock.sent, ['CAPTURE:ON', 'CAPTURE:OFF']);
    });

    test('a second start while recording sends nothing new', () async {
      final (s, sock) = await bound();
      await s.start('MANUAL');
      await s.start('AUTO');   // ignored: a run is already going
      expect(sock.sent, ['CAPTURE:ON']);
    });

    test('nothing is sent while offline', () async {
      final (s, sock) = await bound(online: false);
      await s.start('AUTO');
      expect(sock.sent, isEmpty);
    });

    test('reconnecting mid-run turns capture back on', () async {
      // The Pi forces capture off when the phone drops, and loses it
      // entirely if it restarts. The run is still going, so re-assert it.
      final (s, sock) = await bound();
      await s.start('AUTO');
      sock.setConnected(false);
      sock.sent.clear();
      sock.setConnected(true);
      expect(sock.sent, ['CAPTURE:ON']);
    });

    test('reconnecting with no run going keeps capture off', () async {
      final (_, sock) = await bound(online: false);
      sock.setConnected(true);
      expect(sock.sent, ['CAPTURE:OFF']);
    });

    test('a run restored after the app was killed resumes capture', () async {
      final first = RunLogStore();
      await first.load();
      await first.start('MANUAL');
      // app killed and reopened: a fresh store reads the persisted run
      final revived = RunLogStore();
      final sock = FakeSocket()..connected = true;
      revived.bind(sock);
      sock.sent.clear();
      await revived.load();
      expect(revived.isRecording, isTrue);
      expect(sock.sent, ['CAPTURE:ON']);
    });
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
