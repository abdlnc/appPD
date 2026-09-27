import 'package:flutter_test/flutter_test.dart';
import 'package:robot_controller/models/geo_image.dart';

void main() {
  sessionTests();
  group('GeoImage.parse', () {
    test('GPS-tagged capture', () {
      final g = GeoImage.parse(
          'capture_20260906_134003_lat14.632479_lon121.090634.jpg')!;
      expect(g.kind, 'capture');
      expect(g.hasFix, isTrue);
      expect(g.lat, closeTo(14.632479, 1e-9));
      expect(g.lon, closeTo(121.090634, 1e-9));
      expect(g.annotated, isFalse);
      expect(g.time, DateTime(2026, 9, 6, 13, 40, 3));
    });

    test('annotated twin parses the same position + flags itself', () {
      final g = GeoImage.parse(
          'capture_20260906_135211_lat14.632454_lon121.090589_detected.jpg')!;
      expect(g.annotated, isTrue);
      expect(g.hasFix, isTrue);
      expect(g.lat, closeTo(14.632454, 1e-9));
    });

    test('soil image', () {
      final g = GeoImage.parse('soil_20260920_174146_nogps.jpg')!;
      expect(g.kind, 'soil');
      expect(g.hasFix, isFalse);
      expect(g.time, DateTime(2026, 9, 20, 17, 41, 46));
    });

    test('_nogps gives NULL coords, never 0,0', () {
      final g = GeoImage.parse('capture_20260920_173737_nogps.jpg')!;
      expect(g.lat, isNull);
      expect(g.lon, isNull);
      expect(g.hasFix, isFalse);
      expect(g.coordLabel, 'no GPS fix');
    });

    test('southern / western hemisphere (negative values)', () {
      final g = GeoImage.parse(
          'capture_20260906_134003_lat-33.868800_lon-151.209300.jpg')!;
      expect(g.lat, closeTo(-33.8688, 1e-9));
      expect(g.lon, closeTo(-151.2093, 1e-9));
    });

    test('out-of-range coords are dropped, not pinned', () {
      final g = GeoImage.parse(
          'capture_20260906_134003_lat914.632479_lon121.090634.jpg')!;
      expect(g.hasFix, isFalse);
    });

    test('non-robot filenames return null', () {
      expect(GeoImage.parse('map_20260918_155940.png'), isNull);
      expect(GeoImage.parse('IMG_1234.jpg'), isNull);
      expect(GeoImage.parse(''), isNull);
      expect(GeoImage.parse('capture_2026_lat1_lon2.jpg'), isNull);
    });

    test('coordLabel matches the Pi\'s 6-decimal precision', () {
      final g = GeoImage.parse(
          'capture_20260906_134003_lat14.632479_lon121.090634.jpg')!;
      expect(g.coordLabel, '14.632479, 121.090634');
    });
  });

  group('clusterPhotos', () {
    GeoImage at(double lat, double lon, int sec) => GeoImage.parse(
        'soil_20260920_1741${sec.toString().padLeft(2, '0')}'
        '_lat${lat.toStringAsFixed(6)}_lon${lon.toStringAsFixed(6)}.jpg')!;

    test('photos on the same spot collapse to one pin', () {
      final pins = clusterPhotos([
        at(14.632479, 121.090634, 1),
        at(14.632479, 121.090634, 2),
        at(14.632480, 121.090635, 3), // ~0.15 m away
      ]);
      expect(pins.length, 1);
      expect(pins.first.count, 3);
    });

    test('photos far apart stay separate', () {
      final pins = clusterPhotos([
        at(14.632479, 121.090634, 1),
        at(14.633479, 121.090634, 2), // ~111 m north
      ]);
      expect(pins.length, 2);
    });

    test('no-fix images are excluded from pins entirely', () {
      final pins = clusterPhotos([
        GeoImage.parse('soil_20260920_174146_nogps.jpg')!,
        at(14.632479, 121.090634, 1),
      ]);
      expect(pins.length, 1);
      expect(pins.first.count, 1);
    });

    test('cover image is the newest in the cluster', () {
      final pins = clusterPhotos([
        at(14.632479, 121.090634, 10),
        at(14.632479, 121.090634, 55),
        at(14.632479, 121.090634, 30),
      ]);
      expect(pins.first.cover.time.second, 55);
    });

    test('empty input is handled', () {
      expect(clusterPhotos([]), isEmpty);
    });
  });

  group('metersBetween', () {
    test('~111 km per degree of latitude', () {
      final d = metersBetween(14.0, 121.0, 15.0, 121.0);
      expect(d, closeTo(111195, 500));
    });

    test('zero distance for identical points', () {
      expect(metersBetween(14.6, 121.0, 14.6, 121.0), closeTo(0, 1e-6));
    });
  });
}

// ---------------------------------------------------------------- sessions
void sessionTests() {
  group('session tag', () {
    test('parses the session out of the filename', () {
      final g = GeoImage.parse(
          'capture_20260927_190210_nogps_s20260927-1858.jpg')!;
      expect(g.session, '20260927-1858');
      expect(g.hasFix, isFalse);
      expect(g.time, DateTime(2026, 9, 27, 19, 2, 10));
    });

    test('session + GPS + annotated all together', () {
      final g = GeoImage.parse('capture_20260927_190210'
          '_lat14.207668_lon121.382832_s20260927-1858_detected.jpg')!;
      expect(g.session, '20260927-1858');
      expect(g.annotated, isTrue);
      expect(g.lat, closeTo(14.207668, 1e-9));
    });

    test('soil image with a session', () {
      final g = GeoImage.parse('soil_20260927_190300_nogps_s20260927-1858.jpg')!;
      expect(g.kind, 'soil');
      expect(g.session, '20260927-1858');
    });

    test('older untagged photos still parse, with a null session', () {
      final g = GeoImage.parse(
          'capture_20260906_134003_lat14.632479_lon121.090634.jpg')!;
      expect(g.session, isNull);
    });

    test('a malformed session tag is not accepted as one', () {
      // Wrong shape -> the whole name fails to match, rather than being read
      // as an untagged photo with junk in it.
      expect(GeoImage.parse('capture_20260927_190210_nogps_s2026.jpg'), isNull);
    });
  });

  group('groupBySession', () {
    GeoImage tagged(String session, int hh, int mm) => GeoImage.parse(
        'capture_20260927_${hh.toString().padLeft(2, '0')}'
        '${mm.toString().padLeft(2, '0')}00_nogps_s$session.jpg')!;
    GeoImage untagged(int day, int hh, int mm) => GeoImage.parse(
        'soil_202609${day.toString().padLeft(2, '0')}_'
        '${hh.toString().padLeft(2, '0')}${mm.toString().padLeft(2, '0')}'
        '00_nogps.jpg')!;

    test('tagged photos group by their tag, not by time', () {
      final s = groupBySession([
        tagged('20260927-1800', 18, 5),
        tagged('20260927-1800', 18, 40), // 35 min later, same run
        tagged('20260927-1900', 19, 2),
      ]);
      expect(s.length, 2);
      expect(s.first.key, '20260927-1900'); // newest first
      expect(s.firstWhere((x) => x.key == '20260927-1800').count, 2);
      expect(s.every((x) => x.inferred), isFalse);
    });

    test('untagged photos split on a gap longer than kSessionGap', () {
      final s = groupBySession([
        untagged(20, 10, 0),
        untagged(20, 10, 1), // 1 min later: same run
        untagged(20, 14, 0), // hours later: new run
      ]);
      expect(s.length, 2);
      expect(s.every((x) => x.inferred), isTrue);
      expect(s.map((x) => x.count).toList()..sort(), [1, 2]);
    });

    test('tagged and untagged are never merged into one session', () {
      final s = groupBySession([
        tagged('20260927-1900', 19, 0),
        untagged(27, 19, 1), // one minute apart, but no tag
      ]);
      expect(s.length, 2);
      expect(s.where((x) => x.inferred).length, 1);
    });

    test('session start/end and label come from the photos', () {
      final s = groupBySession([
        tagged('20260927-1800', 18, 30),
        tagged('20260927-1800', 18, 5),
      ]).single;
      expect(s.start, DateTime(2026, 9, 27, 18, 5));
      expect(s.end, DateTime(2026, 9, 27, 18, 30));
      expect(s.label, '27 Sep, 18:05');
    });

    test('counts how many of a session have GPS', () {
      final s = groupBySession([
        GeoImage.parse('capture_20260927_190210_lat14.2_lon121.3'
            '_s20260927-1858.jpg')!,
        tagged('20260927-1858', 19, 3),
      ]).single;
      expect(s.count, 2);
      expect(s.withFix, 1);
    });

    test('empty input gives no sessions', () {
      expect(groupBySession([]), isEmpty);
    });
  });
}
