import 'package:flutter_test/flutter_test.dart';
import 'package:latlong2/latlong.dart';
import 'package:robot_controller/models/field_boundary.dart';

/// The app previews a boundary with the SAME checks the robot applies
/// (agv_control/field_geometry.py). The PARITY group pins that: its expected
/// numbers were produced by the robot's Python code, so if either side
/// changes without the other, these fail -- rather than the preview saying
/// "OK" and the robot then refusing the boundary.

const sq = [XY(0, 0), XY(40, 0), XY(40, 40), XY(0, 40)]; // 40 x 40 m
const lShape = [
  XY(0, 0), XY(60, 0), XY(60, 20), XY(20, 20), XY(20, 60), XY(0, 60),
];

List<LatLng> ll(List<List<double>> v) => [for (final p in v) LatLng(p[0], p[1])];

void main() {
  group('geometry', () {
    test('area: CCW positive, CW negative', () {
      expect(polygonArea(sq), closeTo(1600, 1e-9));
      expect(polygonArea(sq.reversed.toList()), closeTo(-1600, 1e-9));
    });

    test('a figure-of-eight is not simple', () {
      expect(isSimple(sq), isTrue);
      expect(isSimple(lShape), isTrue);
      expect(isSimple(const [XY(0, 0), XY(10, 10), XY(10, 0), XY(0, 10)]),
          isFalse);
    });

    test('point in polygon, including a concave notch', () {
      expect(pointInPolygon(const XY(20, 20), sq), isTrue);
      expect(pointInPolygon(const XY(50, 20), sq), isFalse);
      expect(pointInPolygon(const XY(10, 50), lShape), isTrue);
      expect(pointInPolygon(const XY(40, 40), lShape), isFalse); // the notch
    });

    test('a 40 m square inset by 3 m is 34 x 34', () {
      final ins = insetPolygon(sq, 3)!;
      expect(polygonArea(ins).abs(), closeTo(34 * 34, 1e-6));
      for (final v in ins) {
        expect(distToBoundary(v, sq), closeTo(3, 1e-6));
      }
    });

    test('a concave L insets cleanly; a needle refuses rather than spike', () {
      expect(insetPolygon(lShape, 3), isNotNull);
      expect(insetPolygon(const [XY(0, 0), XY(100, 0), XY(0, 3)], 2), isNull);
      expect(insetPolygon(const [XY(0, 0), XY(100, 0), XY(100, 4), XY(0, 4)], 3),
          isNull);
    });
  });

  group('checkBoundary (what SAVE allows)', () {
    final fr = LocalFrame(14.6320, 121.0900);
    List<LatLng> m(List<XY> xy) => xy.map(fr.toLL).toList();

    test('a 40 m square is fine', () {
      final c = checkBoundary(m(sq), 3);
      expect(c.ok, isTrue, reason: c.reason);
      expect(c.areaM2, closeTo(1600, 1));
      expect(c.stopLine.length, 4);
    });

    test('two corners are not a field', () {
      expect(checkBoundary(m(sq.sublist(0, 2)), 3).ok, isFalse);
    });

    test('a 5 m square is refused as too small', () {
      final c = checkBoundary(
          m(const [XY(0, 0), XY(5, 0), XY(5, 5), XY(0, 5)]), 1);
      expect(c.ok, isFalse);
      expect(c.reason, contains('below'));
    });

    test('repeated taps collapse; tapping the first corner just closes it', () {
      final taps = m([...sq, sq.last, const XY(0, 40.1), sq.first]);
      final c = checkBoundary(taps, 3);
      expect(c.ok, isTrue);
      expect(c.corners.length, 4);
    });

    test('a wider stop line can make a field unusable', () {
      final strip = m(const [XY(0, 0), XY(100, 0), XY(100, 12), XY(0, 12)]);
      expect(checkBoundary(strip, 3).ok, isTrue);
      expect(checkBoundary(strip, 6).ok, isFalse);
    });
  });

  group('PARITY with the robot (numbers from field_geometry.py)', () {
    const pentagon = [
      [14.632, 121.09],
      [14.631955034, 121.090557679],
      [14.632269796, 121.090697099],
      [14.632494627, 121.090325313],
      [14.632314763, 121.089953527],
    ];

    test('same area', () {
      final c = checkBoundary(ll(pentagon), 3);
      expect(c.ok, isTrue);
      expect(c.areaM2, closeTo(3375.0, 0.5));
    });

    test('same stop line', () {
      final c = checkBoundary(ll(pentagon), 3);
      expect(c.stopLine.first.latitude, closeTo(14.632025101, 1e-8));
      expect(c.stopLine.first.longitude, closeTo(121.090024461, 1e-8));
      final fr = LocalFrame(14.632, 121.09);
      expect(polygonArea(c.stopLine.map(fr.toXY).toList()).abs(),
          closeTo(2731.306, 0.5));
    });

    test('same verdict on crossing edges', () {
      const bowtie = [
        [14.632, 121.09],
        [14.632359729, 121.090371786],
        [14.632, 121.090371786],
        [14.632359729, 121.09],
      ];
      final c = checkBoundary(ll(bowtie), 3);
      expect(c.ok, isFalse);
      expect(c.reason, 'edges cross each other');
    });

    test('same verdict on too narrow for the stop line', () {
      const thin = [
        [14.632, 121.09],
        [14.632, 121.090929466],
        [14.632035973, 121.090929466],
        [14.632035973, 121.09],
      ];
      expect(checkBoundary(ll(thin), 3).ok, isFalse);
    });
  });

  group('what the robot reports', () {
    test('Z: an active boundary', () {
      final b = FieldBoundary.parse('{"points":[[14.632,121.09],[14.6321,121.09],'
          '[14.6321,121.0901]],"inset_m":3.0,"active":true,"error":"",'
          '"stop_line":[[14.63203,121.09003]],"area_m2":120.5}')!;
      expect(b.active, isTrue);
      expect(b.points.length, 3);
      expect(b.insetM, 3.0);
      expect(b.areaM2, 120.5);
    });

    test('Z: an inactive draft keeps its reason', () {
      final b = FieldBoundary.parse('{"points":[[14.632,121.09]],"inset_m":3,'
          '"active":false,"error":"needs at least 3 distinct corners",'
          '"stop_line":[]}')!;
      expect(b.active, isFalse);
      expect(b.error, contains('3 distinct'));
    });

    test('P: lanes and summary', () {
      final p = CoverPlan.parse('{"lanes":[[[14.632,121.09],[14.632,121.091]]],'
          '"turns":[],"summary":{"lanes":1,"minutes":3.2}}')!;
      expect(p.lanes.single.length, 2);
      expect(p.summary['minutes'], 3.2);
    });

    test('V: progress', () {
      final v = CoverStatus.parse('{"phase":"LANE","lane":2,"lanes":6,'
          '"coverage":31.5,"overlap":2.0,"active":true}')!;
      expect(v.lane, 2);
      expect(v.lanes, 6);
      expect(v.coverage, 31.5);
      expect(v.active, isTrue);
      expect(v.done, isFalse);
    });

    test('V: driving back to the start', () {
      final v = CoverStatus.parse('{"phase":"RETURN","lane":5,"lanes":5,'
          '"coverage":88.0,"overlap":9.0,"active":true,"home_m":23.4,'
          '"note":""}')!;
      expect(v.returning, isTrue);
      expect(v.done, isFalse);
      expect(v.homeM, 23.4);
    });

    test('V: finished, with how it ended', () {
      final v = CoverStatus.parse('{"phase":"DONE","lane":5,"lanes":5,'
          '"coverage":88.0,"overlap":9.0,"active":true,"home_m":null,'
          '"note":"coverage complete, back at the start"}')!;
      expect(v.done, isTrue);
      expect(v.homeM, isNull);
      expect(v.note, contains('back at the start'));
    });

    test('V: an older robot without the new fields still parses', () {
      final v = CoverStatus.parse('{"phase":"DONE","lane":5,"lanes":5,'
          '"coverage":88.0,"overlap":9.0,"active":false}')!;
      expect(v.homeM, isNull);
      expect(v.note, '');
    });

    test('garbage is ignored, not thrown', () {
      expect(FieldBoundary.parse('not json'), isNull);
      expect(CoverPlan.parse('[1,2]'), isNull);
      expect(CoverStatus.parse(''), isNull);
    });
  });
}
