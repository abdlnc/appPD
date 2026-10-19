import 'dart:convert';
import 'dart:math' as math;

import 'package:latlong2/latlong.dart';

/// The field boundary, its stop line, and COVER mode's plan and progress.
///
/// The ROBOT owns the boundary (fence_node.py validates and enforces it);
/// this file only (a) previews a boundary while it is being drawn, with the
/// SAME checks the robot will apply, so the operator sees "edges cross" or
/// "too narrow for the stop line" before pressing save rather than after,
/// and (b) parses what the robot reports back.
///
/// The geometry is a direct port of agv_control/field_geometry.py. Keep the
/// two in step: if they disagree, the preview says yes and the robot says no.
/// Pure Dart, no Flutter, so it is unit-tested on its own
/// (test/field_boundary_test.dart).

// ---- validation limits: MUST match field_geometry.py ----
const int kMinCorners = 3;
const int kMaxCorners = 64;
const double kMinAreaM2 = 100.0;
const double kMaxAreaM2 = 1000000.0;
const double kMinInsetAreaM2 = 25.0;
const double kDupPointM = 0.5;
const double kDefaultInsetM = 3.0;
const double _rEarth = 6371000.0;

/// A point in flat local metres: x = east, y = north.
class XY {
  final double x, y;
  const XY(this.x, this.y);
  @override
  String toString() => '(${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})';
}

/// Flat metres around an anchor. Same equirectangular frame as the Pi.
class LocalFrame {
  final double lat0, lon0;
  final double _kx, _ky;
  LocalFrame(this.lat0, this.lon0)
      : _kx = _rad(1.0) * _rEarth * math.cos(_rad(lat0)),
        _ky = _rad(1.0) * _rEarth;
  XY toXY(LatLng p) => XY((p.longitude - lon0) * _kx, (p.latitude - lat0) * _ky);
  LatLng toLL(XY p) => LatLng(lat0 + p.y / _ky, lon0 + p.x / _kx);
}

double _rad(double d) => d * math.pi / 180.0;

double _metres(LatLng a, LatLng b) {
  final x = _rad(b.longitude - a.longitude) *
      math.cos(_rad((a.latitude + b.latitude) / 2));
  final y = _rad(b.latitude - a.latitude);
  return _rEarth * math.sqrt(x * x + y * y);
}

/// Signed area, shoelace. Positive = counter-clockwise.
double polygonArea(List<XY> p) {
  var a = 0.0;
  for (var i = 0; i < p.length; i++) {
    final q = p[(i + 1) % p.length];
    a += p[i].x * q.y - q.x * p[i].y;
  }
  return a / 2.0;
}

int _orient(XY a, XY b, XY c) {
  final v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (v.abs() < 1e-12) return 0;
  return v > 0 ? 1 : -1;
}

bool _onSeg(XY a, XY b, XY p) =>
    math.min(a.x, b.x) - 1e-12 <= p.x &&
    p.x <= math.max(a.x, b.x) + 1e-12 &&
    math.min(a.y, b.y) - 1e-12 <= p.y &&
    p.y <= math.max(a.y, b.y) + 1e-12;

bool segmentsIntersect(XY p1, XY p2, XY p3, XY p4) {
  final o1 = _orient(p1, p2, p3), o2 = _orient(p1, p2, p4);
  final o3 = _orient(p3, p4, p1), o4 = _orient(p3, p4, p2);
  if (o1 != o2 && o3 != o4) return true;
  if (o1 == 0 && _onSeg(p1, p2, p3)) return true;
  if (o2 == 0 && _onSeg(p1, p2, p4)) return true;
  if (o3 == 0 && _onSeg(p3, p4, p1)) return true;
  if (o4 == 0 && _onSeg(p3, p4, p2)) return true;
  return false;
}

/// No edge crosses or touches another, apart from neighbours.
bool isSimple(List<XY> p) {
  final n = p.length;
  if (n < 3) return false;
  for (var i = 0; i < n; i++) {
    for (var j = i + 1; j < n; j++) {
      if ((j + 1) % n == i || (i + 1) % n == j) continue;
      if (segmentsIntersect(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) {
        return false;
      }
    }
  }
  return true;
}

/// Even-odd ray cast; either winding.
bool pointInPolygon(XY pt, List<XY> poly) {
  var inside = false;
  for (var i = 0; i < poly.length; i++) {
    final a = poly[i], b = poly[(i + 1) % poly.length];
    if ((a.y > pt.y) != (b.y > pt.y)) {
      final xc = a.x + (pt.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (xc > pt.x) inside = !inside;
    }
  }
  return inside;
}

double _distToSeg(XY p, XY a, XY b) {
  final dx = b.x - a.x, dy = b.y - a.y;
  final l2 = dx * dx + dy * dy;
  var t = l2 == 0 ? 0.0 : ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
  t = t.clamp(0.0, 1.0);
  final cx = a.x + t * dx, cy = a.y + t * dy;
  return math.sqrt((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy));
}

double distToBoundary(XY p, List<XY> poly) {
  var best = double.infinity;
  for (var i = 0; i < poly.length; i++) {
    best = math.min(best, _distToSeg(p, poly[i], poly[(i + 1) % poly.length]));
  }
  return best;
}

/// The stop line: [poly] moved inward by [d], or null if that does not give
/// a clean polygon. Checked, not trusted -- see field_geometry.inset_polygon.
List<XY>? insetPolygon(List<XY> poly, double d) {
  final q = polygonArea(poly) >= 0 ? poly : poly.reversed.toList();
  final n = q.length;
  final starts = <XY>[], dirs = <XY>[];
  for (var i = 0; i < n; i++) {
    final a = q[i], b = q[(i + 1) % n];
    final ex = b.x - a.x, ey = b.y - a.y;
    final l = math.sqrt(ex * ex + ey * ey);
    if (l < 1e-9) return null;
    starts.add(XY(a.x + (-ey / l) * d, a.y + (ex / l) * d));
    dirs.add(XY(ex, ey));
  }
  final out = <XY>[];
  for (var i = 0; i < n; i++) {
    final j = (i - 1 + n) % n;
    final p1 = starts[j], d1 = dirs[j], p2 = starts[i], d2 = dirs[i];
    final den = d1.x * d2.y - d1.y * d2.x;
    if (den.abs() < 1e-12) {
      out.add(p2);
    } else {
      final t = ((p2.x - p1.x) * d2.y - (p2.y - p1.y) * d2.x) / den;
      out.add(XY(p1.x + t * d1.x, p1.y + t * d1.y));
    }
  }
  if (out.length < 3 || !isSimple(out) || polygonArea(out) <= 0) return null;
  for (final v in out) {
    if (!pointInPolygon(v, q) || distToBoundary(v, q) < d - 1e-6) return null;
  }
  return out;
}

/// The result of checking a boundary the way the robot will.
class BoundaryCheck {
  final bool ok;
  final String reason;
  final double areaM2;
  final List<LatLng> corners; // repeated taps removed
  final List<LatLng> stopLine;
  const BoundaryCheck(
      {required this.ok,
      required this.reason,
      this.areaM2 = 0,
      this.corners = const [],
      this.stopLine = const []});
}

/// Check a boundary exactly as fence_node will (validate_boundary).
BoundaryCheck checkBoundary(List<LatLng> raw, double insetM) {
  if (insetM < 0) {
    return const BoundaryCheck(ok: false, reason: "inset must not be negative");
  }
  final pts = <LatLng>[];
  for (final p in raw) {
    if (pts.isNotEmpty && _metres(pts.last, p) < kDupPointM) continue;
    pts.add(p);
  }
  if (pts.length > 1 && _metres(pts.first, pts.last) < kDupPointM) {
    pts.removeLast(); // closed it by tapping the first corner again
  }
  if (pts.length < kMinCorners) {
    return BoundaryCheck(
        ok: false,
        reason: "needs at least $kMinCorners distinct corners",
        corners: pts);
  }
  if (pts.length > kMaxCorners) {
    return BoundaryCheck(
        ok: false, reason: "more than $kMaxCorners corners", corners: pts);
  }
  final fr = LocalFrame(pts.first.latitude, pts.first.longitude);
  final xy = pts.map(fr.toXY).toList();
  if (!isSimple(xy)) {
    return BoundaryCheck(ok: false, reason: "edges cross each other", corners: pts);
  }
  final area = polygonArea(xy).abs();
  if (area < kMinAreaM2) {
    return BoundaryCheck(
        ok: false,
        reason: "area ${area.toStringAsFixed(0)} m2 is below "
            "${kMinAreaM2.toStringAsFixed(0)} m2",
        areaM2: area,
        corners: pts);
  }
  if (area > kMaxAreaM2) {
    return BoundaryCheck(
        ok: false,
        reason: "area ${area.toStringAsFixed(0)} m2 is too large",
        areaM2: area,
        corners: pts);
  }
  final ins = insetPolygon(xy, insetM);
  if (ins == null || polygonArea(ins).abs() < kMinInsetAreaM2) {
    return BoundaryCheck(
        ok: false,
        reason: "nothing usable is left ${insetM.toStringAsFixed(1)} m inside "
            "-- too narrow for that margin",
        areaM2: area,
        corners: pts);
  }
  return BoundaryCheck(
      ok: true,
      reason: "",
      areaM2: area,
      corners: pts,
      stopLine: ins.map(fr.toLL).toList());
}

List<LatLng> _lls(dynamic v) => [
      for (final p in (v as List? ?? const []))
        if (p is List && p.length >= 2)
          LatLng((p[0] as num).toDouble(), (p[1] as num).toDouble()),
    ];

/// What the robot reports on "Z:" -- its stored boundary.
class FieldBoundary {
  final List<LatLng> points;
  final double insetM;
  final bool active;
  final String error;
  final List<LatLng> stopLine;
  final double? areaM2;
  const FieldBoundary(
      {required this.points,
      required this.insetM,
      required this.active,
      required this.error,
      required this.stopLine,
      this.areaM2});

  static FieldBoundary? parse(String json) {
    try {
      final d = jsonDecode(json);
      if (d is! Map) return null;
      return FieldBoundary(
        points: _lls(d['points']),
        insetM: (d['inset_m'] as num?)?.toDouble() ?? kDefaultInsetM,
        active: d['active'] == true,
        error: (d['error'] as String?) ?? '',
        stopLine: _lls(d['stop_line']),
        areaM2: (d['area_m2'] as num?)?.toDouble(),
      );
    } catch (_) {
      return null;
    }
  }
}

/// What the robot reports on "P:" -- COVER's planned lanes.
class CoverPlan {
  final List<List<LatLng>> lanes;
  final Map<String, dynamic> summary;
  final String error;
  const CoverPlan(
      {required this.lanes, required this.summary, required this.error});

  static CoverPlan? parse(String json) {
    try {
      final d = jsonDecode(json);
      if (d is! Map) return null;
      return CoverPlan(
        lanes: [for (final ln in (d['lanes'] as List? ?? const [])) _lls(ln)],
        summary: Map<String, dynamic>.from(d['summary'] as Map? ?? const {}),
        error: (d['error'] as String?) ?? '',
      );
    } catch (_) {
      return null;
    }
  }
}

/// What the robot reports on "V:" -- COVER's progress.
class CoverStatus {
  final String phase; // ACQUIRE LANE TURN FILL DONE NOPLAN
  final int lane, lanes;
  final double coverage, overlap;
  final bool active;
  final String error;
  const CoverStatus(
      {required this.phase,
      required this.lane,
      required this.lanes,
      required this.coverage,
      required this.overlap,
      required this.active,
      required this.error});

  bool get done => phase == 'DONE';

  static CoverStatus? parse(String json) {
    try {
      final d = jsonDecode(json);
      if (d is! Map) return null;
      return CoverStatus(
        phase: (d['phase'] as String?) ?? '',
        lane: (d['lane'] as num?)?.toInt() ?? 0,
        lanes: (d['lanes'] as num?)?.toInt() ?? 0,
        coverage: (d['coverage'] as num?)?.toDouble() ?? 0,
        overlap: (d['overlap'] as num?)?.toDouble() ?? 0,
        active: d['active'] == true,
        error: (d['error'] as String?) ?? '',
      );
    } catch (_) {
      return null;
    }
  }
}
