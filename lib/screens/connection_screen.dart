import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_fonts/google_fonts.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';
import 'control_screen.dart';

class ConnectionScreen extends StatefulWidget {
  const ConnectionScreen({super.key});

  @override
  State<ConnectionScreen> createState() => _ConnectionScreenState();
}

class _ConnectionScreenState extends State<ConnectionScreen> {
  final TextEditingController _ipController = TextEditingController();

  @override
  void initState() {
    super.initState();
    // Change this to your Pi's IP (run `hostname -I` on the Pi).
    _ipController.text = "192.168.1.X";
  }

  @override
  void dispose() {
    _ipController.dispose();
    super.dispose();
  }

  void _connect(BuildContext context) {
    final socketService = Provider.of<RobotSocket>(context, listen: false);
    socketService.connect(_ipController.text.trim());
    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const ControlScreen()),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Center(
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(28.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // Logo badge
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: AppColors.surface,
                    borderRadius: BorderRadius.circular(28),
                    border: Border.all(color: AppColors.border),
                  ),
                  child: ClipRRect(
                    borderRadius: BorderRadius.circular(18),
                    child: Image.asset(
                      'assets/img/logo.png',
                      width: 100,
                      height: 100,
                      errorBuilder: (_, __, ___) => const Icon(
                        Icons.agriculture,
                        size: 72,
                        color: AppColors.lime,
                      ),
                    ),
                  ),
                ),
                const SizedBox(height: 22),
                Text(
                  "AGV",
                  style: GoogleFonts.rajdhani(
                    fontSize: 44,
                    fontWeight: FontWeight.w700,
                    letterSpacing: 6,
                    color: AppColors.textHi,
                  ),
                ),
                Text(
                  "AUTONOMOUS GROUND VEHICLE",
                  style: GoogleFonts.rajdhani(
                    fontSize: 12,
                    fontWeight: FontWeight.w600,
                    letterSpacing: 2.5,
                    color: AppColors.textLo,
                  ),
                ),
                const SizedBox(height: 40),
                TextField(
                  controller: _ipController,
                  decoration: const InputDecoration(
                    labelText: "Raspberry Pi IP Address",
                    prefixIcon: Icon(Icons.lan_outlined, color: AppColors.textLo),
                  ),
                  style: const TextStyle(
                    color: AppColors.textHi,
                    letterSpacing: 1,
                  ),
                  keyboardType: TextInputType.text, // IPs contain dots
                ),
                const SizedBox(height: 24),
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton.icon(
                    onPressed: () => _connect(context),
                    icon: const Icon(Icons.power_settings_new),
                    label: const Text("CONNECT"),
                  ),
                ),
                const SizedBox(height: 14),
                Text(
                  "Make sure your phone and the Pi are on the same Wi-Fi.",
                  textAlign: TextAlign.center,
                  style: GoogleFonts.rajdhani(
                    fontSize: 13,
                    color: AppColors.textLo,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}