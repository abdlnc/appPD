import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

/// Agri-tech dark palette for the AGV controller.
/// One place to tweak all the colors of the app.
class AppColors {
  static const bg = Color(0xFF0C1A10); // deep green-black (scaffold)
  static const surface = Color(0xFF15271B); // panels / cards
  static const surfaceHi = Color(0xFF1E3526); // elevated surface / inputs
  static const lime = Color(0xFF6FD24A); // primary accent (manual mode)
  static const limeDim = Color(0xFF4E9E33);
  static const amber = Color(0xFFF5B301); // secondary accent (auto mode)
  static const textHi = Color(0xFFEAF7E1); // primary text
  static const textLo = Color(0xFF8FB89B); // muted text
  static const online = Color(0xFF5AB552); // connected
  static const offline = Color(0xFFE5533D); // disconnected (warm, not harsh)
  static const border = Color(0xFF2A4632);

  // Dark text to place ON the lime/amber accent buttons
  static const onLime = Color(0xFF06210C);
  static const onAmber = Color(0xFF2A1B00);
}

class AppTheme {
  static ThemeData get dark {
    final base = ThemeData(brightness: Brightness.dark, useMaterial3: true);

    const scheme = ColorScheme.dark(
      primary: AppColors.lime,
      onPrimary: AppColors.onLime,
      secondary: AppColors.amber,
      onSecondary: AppColors.onAmber,
      surface: AppColors.surface,
      onSurface: AppColors.textHi,
      error: AppColors.offline,
    );

    return base.copyWith(
      scaffoldBackgroundColor: AppColors.bg,
      colorScheme: scheme,
      // Rajdhani = techy but readable ("maangas" pero hindi puro monospace)
      textTheme: GoogleFonts.rajdhaniTextTheme(base.textTheme).apply(
        bodyColor: AppColors.textHi,
        displayColor: AppColors.textHi,
      ),
      appBarTheme: const AppBarTheme(
        backgroundColor: AppColors.surface,
        foregroundColor: AppColors.textHi,
        elevation: 0,
        centerTitle: false,
      ),
      inputDecorationTheme: InputDecorationTheme(
        filled: true,
        fillColor: AppColors.surfaceHi,
        labelStyle: const TextStyle(color: AppColors.textLo),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(14),
          borderSide: const BorderSide(color: AppColors.border),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(14),
          borderSide: const BorderSide(color: AppColors.lime, width: 2),
        ),
      ),
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(
          backgroundColor: AppColors.lime,
          foregroundColor: AppColors.onLime,
          padding: const EdgeInsets.symmetric(horizontal: 40, vertical: 16),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(14),
          ),
          textStyle: GoogleFonts.rajdhani(
            fontWeight: FontWeight.w700,
            fontSize: 18,
            letterSpacing: 1.2,
          ),
        ),
      ),
    );
  }
}