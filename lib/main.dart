import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:provider/single_child_widget.dart';
import 'package:firebase_core/firebase_core.dart';
import 'theme/app_theme.dart';
import 'screens/connection_screen.dart';
import 'services/robot_socket.dart';
import 'services/run_log_store.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // Firebase is OPTIONAL (only used for event logging).
  // Wrapped in try/catch so a missing/misconfigured google-services.json
  // never crashes the app on launch. Control still works without it.
  try {
    await Firebase.initializeApp();
  } catch (e) {
    debugPrint('Firebase init skipped (logging disabled): $e');
  }

  runApp(MultiProvider(providers: appProviders(), child: const MyApp()));
}

/// The app's providers. A function (not inline in main) so a test can build
/// exactly this wiring -- see test/app_providers_test.dart.
List<SingleChildWidget> appProviders() => [
  ChangeNotifierProvider(create: (_) => RobotSocket()),
  // Run history, and any run still in progress from a previous launch.
  // Bound to the socket so an active run keeps measuring distance no
  // matter which screen is open.
  ChangeNotifierProxyProvider<RobotSocket, RunLogStore>(
    create: (_) => RunLogStore()..load(),
    // The parentheses matter. `store ?? RunLogStore()..load()` parses
    // as `(store ?? RunLogStore())..load()` -- the cascade binds looser
    // than `??` -- so load() ran on EVERY socket update, many times a
    // second. Each load re-read the run log and re-sent CAPTURE, and
    // reads caught a run half-saved, so it sent ON/OFF/ON... 10-20
    // times a second; every ON made the soil camera take a photo
    // (2026-11-06: one every 1-2 s). load() belongs to creation only.
    update: (_, socket, store) =>
        (store ?? (RunLogStore()..load()))..bind(socket),
  ),
];

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AGV Controller',
      debugShowCheckedModeBanner: false,
      theme: AppTheme.dark,
      home: const ConnectionScreen(),
    );
  }
}
