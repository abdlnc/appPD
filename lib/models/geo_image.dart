import 'dart:math' as math;

/// Metadata carried in the FILENAME of an image saved by the robot.
///
/// The Pi encodes the capture time and the GPS position of every photo into
/// its filename (see ai_detector_node.py / soil_camera_node.py), because the
/// files are served by a plain read-only HTTP server that has no metadata
/// endpoint -- the name is the only channel there is. The four shapes:
///
///   capture_20260906_134003_lat14.632479_lon121.090634.jpg
///   capture_20260906_135211_lat14.632454_lon121.090589_detected.jpg
///   capture_20260920_173737_nogps.jpg
///   soil_20260920_174146_nogps.jpg
///   capture_20260927_190210_nogps_s20260927-1858.jpg      (session-tagged)
///
/// The `_s<YYYYmmdd-HHMM>` tag is the SESSION: one run of the robot, stamped
/// when the capturing node started. Photos saved before that tag existed have
/// no session in the name, so [sessionOf] falls back to grouping by time gap.
///
/// "_nogps" means there was NO fix at that moment. The robot deliberately
/// writes that instead of the last known position -- a photo tagged with a
/// stale position is worse than one honestly marked as untagged -- so
/// [lat]/[lon] stay null here and such an image is never placed on the map.
///
/// Parsing is kept in this file, free of Flutter and of any network code, so
/// it can be unit-tested on its own (see test/geo_image_test.dart).
class GeoImage {
  /// Original filename, exactly as served.
  final String name;

  /// "capture" (front camera / AI) or "soil" (downward camera).
  final String kind;

  /// Capture time, from the filename stamp.
  ///
  /// Deliberately NOT the file's mtime: mtime changes as soon as anyone
  /// copies, backs up or syncs the folder, whereas this is the moment the
  /// robot actually took the photo.
  final DateTime time;

  /// Position at capture time, or null when there was no fix.
  final double? lat;
  final double? lon;

  /// True for the AI's annotated copy (`_detected.jpg`), which is the same
  /// moment as its raw `capture_...jpg` twin.
  final bool annotated;

  /// Session tag from the filename ("20260927-1858"), or null on the older
  /// photos saved before sessions were recorded.
  final String? session;

  const GeoImage({
    required this.name,
    required this.kind,
    required this.time,
    required this.lat,
    required this.lon,
    required this.annotated,
    this.session,
  });

  bool get hasFix => lat != null && lon != null;

  /// Identifies the capture EVENT rather than the file, so a raw capture and
  /// its annotated twin collapse to one thing on the map instead of two pins
  /// sitting exactly on top of each other.
  String get eventKey => '$kind|${time.millisecondsSinceEpoch}|$lat|$lon';

  /// e.g. "14.632, 121.091" -- see [formatCoord].
  String get coordLabel =>
      hasFix ? '${formatCoord(lat!)}, ${formatCoord(lon!)}' : 'no GPS fix';

  static final RegExp _re = RegExp(
    r'^(capture|soil)_(\d{8})_(\d{6})_'
    r'(?:nogps|lat(-?\d+(?:\.\d+)?)_lon(-?\d+(?:\.\d+)?))'
    r'(?:_s(\d{8}-\d{4}))?'
    r'(_detected)?\.(?:jpg|jpeg|png)$',
    caseSensitive: false,
  );

  /// Returns null for anything that isn't one of the robot's image names --
  /// SLAM maps (map_*.png), stray files, or a future naming scheme. Callers
  /// treat null as "no coordinates known", never as an error.
  static GeoImage? parse(String name) {
    final m = _re.firstMatch(name.trim());
    if (m == null) return null;

    final d = m.group(2)!; // yyyyMMdd
    final t = m.group(3)!; // HHmmss
    final time = DateTime(
      int.parse(d.substring(0, 4)),
      int.parse(d.substring(4, 6)),
      int.parse(d.substring(6, 8)),
      int.parse(t.substring(0, 2)),
      int.parse(t.substring(2, 4)),
      int.parse(t.substring(4, 6)),
    );

    final latS = m.group(4);
    final lonS = m.group(5);
    double? lat = latS == null ? null : double.tryParse(latS);
    double? lon = lonS == null ? null : double.tryParse(lonS);

    // Out-of-range values mean a corrupt name, not a real place. Drop BOTH so
    // the image is treated as untagged rather than pinned somewhere wrong.
    if (lat != null && (lat.abs() > 90 || lon == null || lon.abs() > 180)) {
      lat = null;
      lon = null;
    }

    return GeoImage(
      name: name.trim(),
      kind: m.group(1)!.toLowerCase(),
      time: time,
      lat: lat,
      lon: lon,
      session: m.group(6),
      annotated: m.group(7) != null,
    );
  }
}

/// Distance in metres between two lat/lon pairs (equirectangular
/// approximation -- accurate to well under a metre at the few-metre scale
/// this is used for, and cheap enough to run over a thousand images).
double metersBetween(double lat1, double lon1, double lat2, double lon2) {
  const r = 6371000.0;
  final x = _rad(lon2 - lon1) * math.cos(_rad((lat1 + lat2) / 2.0));
  final y = _rad(lat2 - lat1);
  return r * math.sqrt(x * x + y * y);
}

double _rad(double deg) => deg * math.pi / 180.0;

/// Decimal places used when a coordinate is SHOWN.
///
/// Display only. What is stored keeps its full precision: filenames carry 6
/// decimals, and a waypoint is sent to the robot with 7, because navigation
/// works on the number itself rather than on what the screen says.
const int kCoordDecimals = 3;

/// A coordinate as it appears on screen, rounded to the nearest
/// [kCoordDecimals] places (121.090634 shows as 121.091).
String formatCoord(double value) => value.toStringAsFixed(kCoordDecimals);

/// A group of photos taken close enough together to share one map pin.
class PhotoCluster {
  final double lat;
  final double lon;
  final List<GeoImage> images; // newest first

  const PhotoCluster({
    required this.lat,
    required this.lon,
    required this.images,
  });

  GeoImage get cover => images.first;
  int get count => images.length;
}

/// Collapses nearby photos into single pins.
///
/// Necessary, not cosmetic: the robot saves a soil photo every 60s, so a
/// session leaves hundreds of images. Rendering one marker each would bury
/// the map under overlapping icons and make it unreadable -- and while it
/// sits still (or has no fix drift) every one of them lands on the same spot.
///
/// Images with no fix are skipped entirely; count those separately and tell
/// the user, rather than letting them vanish silently.
List<PhotoCluster> clusterPhotos(List<GeoImage> images,
    {double radiusM = 2.0}) {
  final fixed = images.where((i) => i.hasFix).toList()
    ..sort((a, b) => b.time.compareTo(a.time)); // newest first

  final clusters = <_MutableCluster>[];
  for (final img in fixed) {
    _MutableCluster? hit;
    for (final c in clusters) {
      if (metersBetween(c.lat, c.lon, img.lat!, img.lon!) <= radiusM) {
        hit = c;
        break;
      }
    }
    if (hit == null) {
      clusters.add(_MutableCluster(img.lat!, img.lon!, [img]));
    } else {
      hit.images.add(img);
    }
  }
  return clusters
      .map((c) => PhotoCluster(lat: c.lat, lon: c.lon, images: c.images))
      .toList();
}

class _MutableCluster {
  final double lat;
  final double lon;
  final List<GeoImage> images;
  _MutableCluster(this.lat, this.lon, this.images);
}

/// One run of the robot: the photos saved between switching it on and off.
class PhotoSession {
  /// Session tag ("20260927-1858") for tagged photos, or a synthetic
  /// `gap:<epoch>` key for older untagged ones.
  final String key;

  /// True when this session was inferred from time gaps rather than read from
  /// the filenames — worth showing differently, since its boundaries are a
  /// guess about photos saved before sessions were recorded.
  final bool inferred;

  final List<GeoImage> images; // newest first
  const PhotoSession(
      {required this.key, required this.inferred, required this.images});

  DateTime get start => images.last.time;
  DateTime get end => images.first.time;
  int get count => images.length;
  int get withFix => images.where((i) => i.hasFix).length;

  /// "27 Sep, 18:58" — the session's own start, which is what an operator
  /// remembers it by.
  String get label {
    const months = [
      'Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
      'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec',
    ];
    final t = start;
    return '${t.day} ${months[t.month - 1]}, '
        '${t.hour.toString().padLeft(2, '0')}:'
        '${t.minute.toString().padLeft(2, '0')}';
  }
}

/// Photos more than this far apart belong to different sessions, when the
/// filenames carry no session tag. The robot saves a soil photo every 60s and
/// an obstacle photo at most every 15s while running, so a gap this long means
/// it was switched off in between.
const Duration kSessionGap = Duration(minutes: 20);

/// Groups photos into sessions, newest session first.
///
/// Prefers the session tag in the filename. Untagged photos (saved before the
/// tag existed) are grouped by time gap instead, so old runs still appear as
/// separate sessions rather than one undifferentiated pile. The two kinds are
/// never mixed into one session: a tagged photo and an untagged one are from
/// different software versions, so a shared session would be a guess.
List<PhotoSession> groupBySession(List<GeoImage> images,
    {Duration gap = kSessionGap}) {
  final sorted = [...images]..sort((a, b) => b.time.compareTo(a.time));

  final tagged = <String, List<GeoImage>>{};
  final untagged = <GeoImage>[];
  for (final img in sorted) {
    if (img.session != null) {
      tagged.putIfAbsent(img.session!, () => []).add(img);
    } else {
      untagged.add(img);
    }
  }

  final out = <PhotoSession>[
    for (final e in tagged.entries)
      PhotoSession(key: e.key, inferred: false, images: e.value),
  ];

  // Walk the untagged ones newest-first, breaking a session whenever the step
  // back in time exceeds the gap.
  var run = <GeoImage>[];
  void flush() {
    if (run.isEmpty) return;
    out.add(PhotoSession(
        key: 'gap:${run.first.time.millisecondsSinceEpoch}',
        inferred: true,
        images: List.of(run)));
    run = [];
  }

  for (final img in untagged) {
    if (run.isNotEmpty && run.last.time.difference(img.time).abs() > gap) {
      flush();
    }
    run.add(img);
  }
  flush();

  out.sort((a, b) => b.end.compareTo(a.end)); // newest session first
  return out;
}
