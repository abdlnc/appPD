import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:robot_controller/main.dart';
import 'package:robot_controller/services/robot_socket.dart';
import 'package:robot_controller/services/run_log_store.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// 2026-11-06: the provider wiring called RunLogStore.load() on EVERY socket
/// update (a cascade-precedence slip in main.dart), and each load re-sent
/// CAPTURE -- the soil camera ended up taking a photo every 1-2 seconds.
/// This builds the app's real providers and checks load() runs once, however
/// often the socket changes.
void main() {
  setUp(() => SharedPreferences.setMockInitialValues({}));

  testWidgets('RunLogStore.load() runs once, not on every socket update',
      (t) async {
    late RobotSocket socket;
    late RunLogStore store;
    await t.pumpWidget(MultiProvider(
      providers: appProviders(),
      child: Builder(builder: (context) {
        socket = context.read<RobotSocket>();
        store = context.watch<RunLogStore>();
        return const SizedBox();
      }),
    ));
    await t.pump();
    for (var i = 0; i < 50; i++) {
      socket.notifyListeners(); // what every camera frame / scan / fix does
      await t.pump();
    }
    expect(store.debugLoadCount, 1);
  });
}
