import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:firebase_core/firebase_core.dart';
import 'theme/app_theme.dart';
import 'screens/connection_screen.dart';
import 'services/robot_socket.dart';

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

  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => RobotSocket()),
      ],
      child: const MyApp(),
    ),
  );
}

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