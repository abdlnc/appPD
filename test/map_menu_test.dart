import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';
import 'package:robot_controller/screens/map_screen.dart';
import 'package:robot_controller/services/robot_socket.dart';
import 'package:robot_controller/services/run_log_store.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// The GPS map's app bar on a SMALL phone: it used to carry eight icons.
/// Now it is Follow + one tools-and-layers menu -- this checks it fits
/// (no overflow) at 360 x 640 and that the menu holds every former button.
void main() {
  setUp(() {
    SharedPreferences.setMockInitialValues({});
    GoogleFonts.config.allowRuntimeFetching = false;
  });

  Future<void> pumpMap(WidgetTester t) async {
    t.view.physicalSize = const Size(360, 640);
    t.view.devicePixelRatio = 1.0;
    addTearDown(t.view.reset);
    await t.pumpWidget(MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => RobotSocket()),
        ChangeNotifierProvider(create: (_) => RunLogStore()),
      ],
      child: const MaterialApp(home: MapScreen()),
    ));
    await t.pump(const Duration(milliseconds: 100));
  }

  testWidgets('app bar fits a 360 px phone: Follow + one menu',
      (t) async {
    await pumpMap(t);
    expect(find.byTooltip('Map tools and layers'), findsOneWidget);
    expect(find.byTooltip('Follow robot'), findsOneWidget);
    // the old separate buttons are gone from the bar
    expect(find.byTooltip('Satellite view'), findsNothing);
    expect(find.byTooltip('Edit the field boundary'), findsNothing);
  });

  testWidgets('the menu holds every former button, and toggles work',
      (t) async {
    await pumpMap(t);
    await t.tap(find.byTooltip('Map tools and layers'));
    await t.pumpAndSettle();
    for (final label in [
      'Set robot position',
      'Edit field boundary',
      'Satellite view',
      'Path trail',
      'Lidar obstacle pins',
      'Obstacle photos',
      'Clear trail & Lidar pins',
    ]) {
      expect(find.text(label), findsOneWidget, reason: label);
    }
    // satellite -> the next time the menu offers Street view
    await t.tap(find.text('Satellite view'));
    await t.pumpAndSettle();
    await t.tap(find.byTooltip('Map tools and layers'));
    await t.pumpAndSettle();
    expect(find.text('Street view'), findsOneWidget);
    // boundary editing: the menu now offers to stop it
    await t.tap(find.text('Edit field boundary'));
    await t.pumpAndSettle();
    expect(find.text('EDITING BOUNDARY'), findsOneWidget);
    await t.tap(find.byTooltip('Map tools and layers'));
    await t.pumpAndSettle();
    expect(find.text('Stop editing boundary'), findsOneWidget);
    // set position: offered (greyed out while not connected to the robot)
    expect(find.text('Set robot position'), findsOneWidget);
  });
}
