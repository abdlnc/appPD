import 'package:shared_preferences/shared_preferences.dart';

/// Hidden developer settings, stored on the phone so they survive restarts.
///
/// Opened by tapping the AUTO/MANUAL mode pill on the control screen 7 times
/// in quick succession -- the same trick Android uses for its own developer
/// options. Deliberately undiscoverable: these are for bench testing and
/// calibration, not for someone operating the robot in the field.
class DevSettings {
  static const String _kShowJoystick = 'dev.show_joystick';

  /// Whether the manual-drive joystick is shown. Off by default: in normal
  /// use the robot is driven in AUTO or NAV, and a joystick on screen invites
  /// a stray thumb to take manual control.
  static Future<bool> loadShowJoystick() async {
    final p = await SharedPreferences.getInstance();
    return p.getBool(_kShowJoystick) ?? false;
  }

  static Future<void> saveShowJoystick(bool value) async {
    final p = await SharedPreferences.getInstance();
    await p.setBool(_kShowJoystick, value);
  }

}
