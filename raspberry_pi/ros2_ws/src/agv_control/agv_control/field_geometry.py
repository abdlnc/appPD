#!/usr/bin/env python3
"""
Field geometry for the boundary fence and COVER mode. Pure Python.

No ROS, no numpy, no network: everything here is a function of its inputs,
so it can be tested on its own (see ~/field_geometry_test.py) and shared by
fence_node, coverage_node, control_node, nav_node, path_recorder and the two
camera nodes without any of them having to agree on anything else.

COORDINATES. Positions arrive as lat/lon. Everything geometric happens in a
LocalFrame: flat metres around an anchor point, x = east, y = north. At
field scale (hundreds of metres) the flat-earth error is millimetres --
measured on the app side against haversine at this latitude -- which is
nothing next to the receiver's +/-2.5m.

WHAT THE FENCE ACTUALLY TESTS. "Inside the stop line" is computed EXACTLY,
as: inside the boundary polygon AND at least inset_m from every edge. That
is correct for any simple polygon, convex or not. The inset polygon itself
(inset_polygon) is only used to DRAW the stop line and to PLAN lanes. If it
cannot be built for an awkward shape, planning refuses -- but the fence
keeps working, because enforcement never depended on it.
"""

import math

R_EARTH = 6371000.0

# ---- boundary validation ----
MIN_POINTS = 3
MAX_POINTS = 64            # a hand-drawn field, not a survey
MIN_AREA_M2 = 100.0        # 10m x 10m: smaller is a mis-tap, not a field
MAX_AREA_M2 = 1_000_000.0  # 1 km^2: larger is a mis-tap on the far side of town
MIN_INSET_AREA_M2 = 25.0   # what is left inside the stop line must be drivable
DUP_POINT_M = 0.5          # consecutive corners closer than this are one corner


# =========================================================== local frame
class LocalFrame:
    """Flat metres around (lat0, lon0): x = east, y = north."""

    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self._kx = math.radians(1.0) * R_EARTH * math.cos(math.radians(lat0))
        self._ky = math.radians(1.0) * R_EARTH

    def to_xy(self, lat, lon):
        return ((lon - self.lon0) * self._kx, (lat - self.lat0) * self._ky)

    def to_ll(self, x, y):
        return (self.lat0 + y / self._ky, self.lon0 + x / self._kx)


def meters_between(lat1, lon1, lat2, lon2):
    """Same equirectangular formula the app uses, so the two agree."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return R_EARTH * math.hypot(x, y)


def bearing_xy(dx, dy):
    """Compass bearing (0 = north, 90 = east) of a local-frame vector."""
    return math.degrees(math.atan2(dx, dy)) % 360.0


def norm180(a):
    return (a + 180.0) % 360.0 - 180.0


# =========================================================== polygons
def polygon_area(poly):
    """Signed area, shoelace. Positive = counter-clockwise."""
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def ccw(poly):
    """The same polygon, counter-clockwise."""
    return list(poly) if polygon_area(poly) >= 0 else list(reversed(poly))


def _orient(a, b, c):
    v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)


def _on_segment(a, b, p):
    return (min(a[0], b[0]) - 1e-12 <= p[0] <= max(a[0], b[0]) + 1e-12 and
            min(a[1], b[1]) - 1e-12 <= p[1] <= max(a[1], b[1]) + 1e-12)


def segments_intersect(p1, p2, p3, p4):
    """True if segment p1p2 touches segment p3p4 at all, including at an
    endpoint or along a shared collinear stretch."""
    o1, o2 = _orient(p1, p2, p3), _orient(p1, p2, p4)
    o3, o4 = _orient(p3, p4, p1), _orient(p3, p4, p2)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, p2, p3):
        return True
    if o2 == 0 and _on_segment(p1, p2, p4):
        return True
    if o3 == 0 and _on_segment(p3, p4, p1):
        return True
    if o4 == 0 and _on_segment(p3, p4, p2):
        return True
    return False


def is_simple(poly):
    """No edge crosses or touches another, apart from neighbours sharing a
    corner. A figure-of-eight boundary has no well-defined inside."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue                      # neighbours share a corner
            b1, b2 = poly[j], poly[(j + 1) % n]
            if segments_intersect(a1, a2, b1, b2):
                return False
    return True


def point_in_polygon(p, poly):
    """Even-odd ray cast. Works for any simple polygon, either winding."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if xc > x:
                inside = not inside
    return inside


def closest_on_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return a
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return (ax + t * dx, ay + t * dy)


def nearest_edge(p, poly):
    """(distance, edge_index, closest_point) to the polygon's boundary."""
    best = (math.inf, -1, None)
    n = len(poly)
    for i in range(n):
        c = closest_on_segment(p, poly[i], poly[(i + 1) % n])
        d = math.hypot(p[0] - c[0], p[1] - c[1])
        if d < best[0]:
            best = (d, i, c)
    return best


def dist_to_boundary(p, poly):
    return nearest_edge(p, poly)[0]


# =========================================================== the fence
FENCE_IN, FENCE_LINE, FENCE_OUT = "IN", "LINE", "OUT"


def fence_status(p, poly, inset_m):
    """IN (clear of the stop line), LINE (inside the boundary but within
    inset_m of it), or OUT (outside the boundary). Exact for any simple
    polygon -- see the module note."""
    if not point_in_polygon(p, poly):
        return FENCE_OUT
    return FENCE_LINE if dist_to_boundary(p, poly) < inset_m else FENCE_IN


def inward_bearing(p, poly, inset_poly=None):
    """Compass bearing pointing back INTO the field from p.

    Given the stop line (inset_poly) and p is past it: toward the NEAREST
    POINT ON THE STOP LINE -- the shortest way back to where the robot may
    drive. At a corner that points diagonally into the field, which is
    right. (The inward normal of the nearest edge, used before, flips
    between the two edges at a corner; in simulation that sent the robot
    reversing along one edge and out through the other.)

    Without inset_poly, or with p already inside it: inside the boundary,
    the inward normal of the nearest edge; outside, toward the nearest
    point on the boundary."""
    if inset_poly is not None and not point_in_polygon(p, inset_poly):
        d, i, c = nearest_edge(p, inset_poly)
        if d > 1e-6:
            return bearing_xy(c[0] - p[0], c[1] - p[1])
    d, i, c = nearest_edge(p, poly)
    if not point_in_polygon(p, poly) and d > 1e-6:
        return bearing_xy(c[0] - p[0], c[1] - p[1])
    q = ccw(poly)
    d, i, c = nearest_edge(p, q)
    a, b = q[i], q[(i + 1) % len(q)]
    ex, ey = b[0] - a[0], b[1] - a[1]
    return bearing_xy(-ey, ex)          # left normal of a CCW edge = inward


def past_line_depth(p, poly, inset_m):
    """How far past the stop line p is, in metres: 0 at the line, inset_m at
    the boundary, more outside. Negative when inside the stop line. One
    number that only ever grows as the robot goes the wrong way -- which is
    what the "never get deeper" guard watches."""
    d = dist_to_boundary(p, poly)
    return inset_m - d if point_in_polygon(p, poly) else inset_m + d


FORWARD_IN_DEG = 60.0


def allowed_direction(facing_deg, inward_deg):
    """At the stop line: +1 if driving FORWARD heads back in, -1 if
    REVERSING does.

    Forward only when the robot CONFIDENTLY faces inward -- within
    FORWARD_IN_DEG, not merely within 90. Near 90 degrees (side-on, which is
    how a robot meets the line mid-turn) the heading from GPS positions is
    at its least reliable, and a wrong "forward" drives it deeper. Reverse
    is the spec's default and the right answer for the usual case, arriving
    head-first. The caller decides what to do when the facing is unknown."""
    return (+1 if abs(norm180(inward_deg - facing_deg)) < FORWARD_IN_DEG
            else -1)


def _line_intersect(p1, d1, p2, d2):
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-12:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def inset_polygon(poly, d):
    """The stop line: poly moved inward by d metres, or None if that does
    not produce a clean polygon (a narrow neck, a sharp spike).

    Each edge is shifted inward and neighbouring shifted edges are
    intersected. The result is then CHECKED rather than trusted: it must be
    simple, keep its winding, and every corner must sit inside the original
    at least d from its edges (minus a hair). Miter spikes at sharp corners
    and collapsed necks fail that, and planning refuses rather than plan
    lanes along a line that is not where the fence is."""
    q = ccw(poly)
    n = len(q)
    lines = []
    for i in range(n):
        a, b = q[i], q[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey)
        if L < 1e-9:
            return None
        nx, ny = -ey / L, ex / L                  # inward for CCW
        lines.append(((a[0] + nx * d, a[1] + ny * d), (ex, ey)))
    out = []
    for i in range(n):
        p_prev, d_prev = lines[i - 1]
        p_cur, d_cur = lines[i]
        x = _line_intersect(p_prev, d_prev, p_cur, d_cur)
        out.append(x if x is not None else p_cur)
    if len(out) < 3 or not is_simple(out) or polygon_area(out) <= 0:
        return None
    for v in out:
        if not point_in_polygon(v, q) or dist_to_boundary(v, q) < d - 1e-6:
            return None
    return out


def validate_boundary(latlon, inset_m):
    """Check a boundary as the app sent it. Returns (ok, reason, cleaned)
    where cleaned is the corner list with repeated taps removed."""
    if inset_m < 0:
        return False, "inset must not be negative", None
    pts = []
    for la, lo in latlon:
        if not (-90 <= la <= 90 and -180 <= lo <= 180):
            return False, f"corner out of range: {la}, {lo}", None
        if pts and meters_between(pts[-1][0], pts[-1][1], la, lo) < DUP_POINT_M:
            continue
        pts.append((la, lo))
    if len(pts) > 1 and meters_between(pts[0][0], pts[0][1],
                                       pts[-1][0], pts[-1][1]) < DUP_POINT_M:
        pts.pop()                                 # closed it by tapping the first
    if len(pts) < MIN_POINTS:
        return False, f"needs at least {MIN_POINTS} distinct corners", None
    if len(pts) > MAX_POINTS:
        return False, f"more than {MAX_POINTS} corners", None
    fr = LocalFrame(*pts[0])
    xy = [fr.to_xy(*p) for p in pts]
    if not is_simple(xy):
        return False, "edges cross each other", None
    area = abs(polygon_area(xy))
    if area < MIN_AREA_M2:
        return False, f"area {area:.0f} m2 is below {MIN_AREA_M2:.0f} m2", None
    if area > MAX_AREA_M2:
        return False, f"area {area:.0f} m2 is above {MAX_AREA_M2:.0f} m2", None
    ins = inset_polygon(xy, inset_m)
    if ins is None or abs(polygon_area(ins)) < MIN_INSET_AREA_M2:
        return False, (f"nothing usable is left {inset_m:.1f} m inside the "
                       f"boundary -- the field is too narrow for that "
                       f"margin"), None
    return True, "", pts


# =========================================================== lanes
def _convex_hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and _orient(h[-2], h[-1], p) <= 0:
                h.pop()
            h.append(p)
        return h
    lower, upper = half(pts), half(reversed(pts))
    return lower[:-1] + upper[:-1]


def _rot(p, ang):
    c, s = math.cos(ang), math.sin(ang)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def long_axis_angle(poly):
    """Angle (radians, from +x/east) of the field's LONG axis, from the
    minimum-area bounding rectangle. Lanes run along it so there are as
    few turns as possible -- turning is the slow, imprecise part."""
    hull = _convex_hull(poly)
    best = None
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        r = [_rot(p, -ang) for p in hull]
        w = max(p[0] for p in r) - min(p[0] for p in r)
        h = max(p[1] for p in r) - min(p[1] for p in r)
        if best is None or w * h < best[0]:
            best = (w * h, ang if w >= h else ang + math.pi / 2)
    return best[1] if best else 0.0


def lawnmower_lanes(inset_poly, spacing, angle, end_margin=0.0,
                    min_len=2.0, side_margin=0.0):
    """Back-and-forth lanes across the inset area, along `angle`.

    Lanes are evenly spread so the real spacing is <= `spacing` (never
    wider than asked), half a spacing in from each side -- or side_margin,
    if that is more. Each lane is cut
    back by `end_margin` at both ends to leave room to turn INSIDE the stop
    line. A concave field can give one lane line several separate pieces;
    each becomes its own lane. Directions alternate, so consecutive lanes
    end and start on the same side.

    Returns (lanes, real_spacing), each lane a ((x, y), (x, y)) pair."""
    r = [_rot(p, -angle) for p in inset_poly]
    ymin = min(p[1] for p in r)
    ymax = max(p[1] for p in r)
    height = ymax - ymin
    n = max(1, math.ceil(height / spacing))
    real = height / n
    # outermost lanes at least side_margin in: if centring would put them
    # closer, spread the lanes over the narrower band instead
    edge = max(real / 2.0, side_margin)
    if n > 1:
        span = max(0.0, height - 2 * edge)
        n = max(1, math.ceil(span / spacing) + 1)
        real = span / (n - 1) if n > 1 else 0.0
    lanes = []
    flip = False
    for k in range(n):
        y = (ymin + edge + k * real) if n > 1 else (ymin + height / 2.0)
        xs = []
        m = len(r)
        for i in range(m):
            (x1, y1), (x2, y2) = r[i], r[(i + 1) % m]
            if (y1 > y) != (y2 > y):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        pieces = []
        for j in range(0, len(xs) - 1, 2):
            a, b = xs[j] + end_margin, xs[j + 1] - end_margin
            if b - a >= min_len:
                pieces.append((a, b))
        if flip:
            pieces = [(b, a) for a, b in reversed(pieces)]
        for a, b in pieces:
            lanes.append((_rot((a, y), angle), _rot((b, y), angle)))
        if pieces:
            flip = not flip
    return lanes, real


def turn_waypoints(prev_start, prev_end, next_start, turn_diameter):
    """Waypoints from the end of one lane to the start of the next.

    Wide enough (lane spacing >= the turning diameter): straight to the next
    lane's start -- turning at it on full lock already traces a U no wider
    than the spacing.

    Too tight: a robot that cannot pivot cannot U-turn inside its own
    turning circle. So it swings OUT first, away from the next lane, over
    the top, and down on the far side of it: a "bulb" or loop-out turn.
    The bulb is (diameter - spacing)/2 wider than the lanes on each side
    and reaches a turning RADIUS beyond the further lane end -- so lanes
    must be trimmed by at least that (end_margin) for it to stay inside
    the stop line.

    The lane just finished gives "forward"; "across" is the part of the
    gap to the next lane that is square to it. Lanes of unequal length
    (an angled field edge) are handled by measuring forward from whichever
    end reaches further."""
    fx, fy = prev_end[0] - prev_start[0], prev_end[1] - prev_start[1]
    L = math.hypot(fx, fy) or 1.0
    fx, fy = fx / L, fy / L
    vx, vy = next_start[0] - prev_end[0], next_start[1] - prev_end[1]
    along = vx * fx + vy * fy
    px, py = vx - along * fx, vy - along * fy
    spacing = math.hypot(px, py)
    if spacing >= turn_diameter or spacing < 1e-6:
        return [next_start]
    ux, uy = px / spacing, py / spacing           # across, toward next lane
    r = turn_diameter / 2.0
    w = (turn_diameter - spacing) / 2.0
    base = max(0.0, along)                        # beyond the further end
    ex, ey = prev_end

    def at(fwd, side):
        return (ex + fx * fwd + ux * side, ey + fy * fwd + uy * side)
    return [
        at(base + r * 0.5, -w),                   # out, away from next lane
        at(base + r, spacing / 2.0),              # over the top
        at(base + r * 0.5, spacing + w),          # down past the next lane
        next_start,
    ]


# =========================================================== following
class TrackHeading:
    """Heading from GPS POSITIONS a few metres apart, not from the
    receiver's instantaneous course reading.

    At walking pace the NEO-M8N's course field wanders by tens of degrees
    from one fix to the next, and is meaningless when nearly stationary.
    A bearing taken across MIN_BASELINE metres of actual travel averages
    that out. The cost is lag: at 0.6 m/s a 3 m baseline is ~5 s old,
    which is why the follower looks well ahead rather than at its feet.

    Direction-aware: positions logged while REVERSING give the opposite of
    where the robot faces, so the track restarts whenever the direction of
    travel flips, and a reversing track is turned round before use."""

    # max_age only bounds memory -- heading() already walks back from the
    # newest point and stops at the first one far enough away, so older
    # points never affect the answer once the robot has covered the
    # baseline. It must be long, though: it was 20 s, and creeping at
    # 0.18 m/s covers only 3.6 m in 20 s -- less than a 5 m baseline -- so a
    # slow robot could NEVER get a heading. In simulation it crept straight,
    # blind, until it reached the far side of the field.
    def __init__(self, min_baseline=3.0, max_age=120.0):
        self.min_baseline = min_baseline
        self.max_age = max_age
        self.pts = []            # (t, x, y)
        self.direction = 0       # +1 forward, -1 reverse, 0 unknown

    def reset(self):
        self.pts = []

    def feed(self, t, x, y, direction):
        if direction != 0 and direction != self.direction:
            self.pts = []                       # travel flipped: start again
            self.direction = direction
        self.pts.append((t, x, y))
        while self.pts and t - self.pts[0][0] > self.max_age:
            self.pts.pop(0)

    def heading(self):
        """Compass bearing the robot FACES, or None if it has not yet moved
        far enough in one direction to tell."""
        if len(self.pts) < 2 or self.direction == 0:
            return None
        t1, x1, y1 = self.pts[-1]
        for t0, x0, y0 in reversed(self.pts[:-1]):
            if math.hypot(x1 - x0, y1 - y0) >= self.min_baseline:
                h = bearing_xy(x1 - x0, y1 - y0)
                return h if self.direction > 0 else (h + 180.0) % 360.0
        return None


def lookahead_point(a, b, p, lookahead):
    """Point on the line through lane a->b, `lookahead` metres beyond p's
    projection onto it. Steering at this, rather than at the lane END, is
    what pulls the robot back ONTO the lane line after a detour instead of
    cutting a diagonal to the far end."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    s = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy
    s = s + lookahead
    return (a[0] + ux * s, a[1] + uy * s), s / L


def progress_along(a, b, p):
    """0 at a, 1 at b, for p projected onto the lane line."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy or 1.0
    return ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2


def cross_track(a, b, p):
    """Signed distance from the lane line: + = left of a->b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy) or 1.0
    return (dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / L


def steer_toward(p, heading_deg, target, full_deg):
    """-1..+1 steering, + = LEFT, the /cmd_vel convention. Proportional to
    the heading error -- nav_node's law. Kept for reference and tests; the
    lane follower uses pursuit_steer, see there for why."""
    want = bearing_xy(target[0] - p[0], target[1] - p[1])
    err = norm180(want - heading_deg)            # + = target to the RIGHT
    return max(-1.0, min(1.0, -err / full_deg))


def pursuit_steer(p, heading_deg, target, turn_radius):
    """Pure-pursuit steering, -1..+1, + = LEFT.

    The arc that reaches `target` from `p`, starting along heading_deg, has
    curvature 2*sin(a)/L (a = bearing error, L = distance). Full lock is
    curvature 1/turn_radius, so the command is that arc's curvature times
    the turning radius.

    Why not steer_toward (full lock at a fixed heading error)? Because the
    heading here comes from GPS POSITIONS a few metres apart, so it lags --
    it describes where the robot pointed a metre or two ago. A high,
    distance-blind gain on a lagging heading oscillates: in simulation it
    weaved +/-2 m along every lane even with a perfect GPS. Pure pursuit's
    gain falls with lookahead, which is exactly what keeps a lagged loop
    stable, and rises for a close target (a turn), where it is needed."""
    L = math.hypot(target[0] - p[0], target[1] - p[1])
    if L < 1e-6:
        return 0.0
    want = bearing_xy(target[0] - p[0], target[1] - p[1])
    a = math.radians(norm180(want - heading_deg))  # + = target to the RIGHT
    curv = 2.0 * math.sin(a) / L                     # + = curve RIGHT
    return max(-1.0, min(1.0, -curv * turn_radius))


# =========================================================== coverage grid
class CoverageGrid:
    """Which parts of the field have been driven, in fixed-size cells.

    A lane stands for a STRIP of the field `swath` metres wide, not just the
    0.6m the body physically rolls over: lanes are spaced for the GPS's
    +/-2.5m, not for the tyres, so each one is taken to sample the ground on
    either side of it. A cell counts as covered when its centre comes within
    swath/2 of the driven path. Coverage % is therefore "of the field, at
    lane resolution" -- and with +/-2.5m GPS against 3m cells it is an
    estimate, good to about a cell.

    OVERLAP is ground the robot DROVE OVER on more than one pass: cells its
    path went through (centre within half a cell) from two different lanes.
    Deliberately not "two lanes' strips touched it": neighbouring strips
    meet along the line between lanes, so every cell there would count,
    from the geometry alone -- an early version reported 56% overlap on a
    run that never drove the same ground twice."""

    def __init__(self, poly, cell, swath):
        self.cell = cell
        self.swath = swath
        self.poly = poly
        self.driven = {}                     # (i, j) -> set(lane ids), core
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        self.x0, self.y0 = min(xs), min(ys)
        nx = int(math.ceil((max(xs) - self.x0) / cell))
        ny = int(math.ceil((max(ys) - self.y0) / cell))
        self.cells = {}                      # (i, j) -> set(lane ids)
        for i in range(nx):
            for j in range(ny):
                c = self.centre(i, j)
                if point_in_polygon(c, poly):
                    self.cells[(i, j)] = set()

    def centre(self, i, j):
        return (self.x0 + (i + 0.5) * self.cell,
                self.y0 + (j + 0.5) * self.cell)

    def mark_segment(self, p, q, lane_id):
        r = self.swath / 2.0
        lo_i = int(math.floor((min(p[0], q[0]) - r - self.x0) / self.cell))
        hi_i = int(math.floor((max(p[0], q[0]) + r - self.x0) / self.cell))
        lo_j = int(math.floor((min(p[1], q[1]) - r - self.y0) / self.cell))
        hi_j = int(math.floor((max(p[1], q[1]) + r - self.y0) / self.cell))
        for i in range(lo_i, hi_i + 1):
            for j in range(lo_j, hi_j + 1):
                ids = self.cells.get((i, j))
                if ids is None:
                    continue
                c = self.centre(i, j)
                cp = closest_on_segment(c, p, q)
                d = math.hypot(c[0] - cp[0], c[1] - cp[1])
                if d <= r:
                    ids.add(lane_id)
                if d <= self.cell / 2.0:
                    self.driven.setdefault((i, j), set()).add(lane_id)

    def coverage_pct(self):
        if not self.cells:
            return 0.0
        done = sum(1 for v in self.cells.values() if v)
        return 100.0 * done / len(self.cells)

    def overlap_pct(self):
        """Of the ground the robot DROVE over, the share it drove over more
        than once. (Divided by driven cells, not covered ones: a covered
        strip is wider than the track, so dividing by it would make driving
        everything twice read about 50% instead of 100%.)"""
        driven = [v for v in self.driven.values() if v]
        if not driven:
            return 0.0
        return 100.0 * sum(1 for v in driven if len(v) > 1) / len(driven)

    def missed(self):
        return [self.centre(i, j) for (i, j), v in self.cells.items() if not v]

    def to_dict(self):
        return {"cell": self.cell, "swath": self.swath,
                "covered": [[i, j, sorted(v)] for (i, j), v in
                            self.cells.items() if v],
                "driven": [[i, j, sorted(v)] for (i, j), v in
                           self.driven.items() if v]}

    def load_dict(self, d):
        if d.get("cell") != self.cell:
            return
        for i, j, ids in d.get("covered", []):
            if (i, j) in self.cells:
                self.cells[(i, j)] = set(ids)
        for i, j, ids in d.get("driven", []):
            self.driven[(i, j)] = set(ids)


def nearest_neighbour_route(start, pts):
    """Visit every point, always going to the closest unvisited one next.
    Not optimal, but simple, predictable, and good enough for the handful
    of gaps a fill-in pass has to pick up."""
    route, left, cur = [], list(pts), start
    while left:
        k = min(range(len(left)),
                key=lambda i: math.hypot(left[i][0] - cur[0],
                                         left[i][1] - cur[1]))
        cur = left.pop(k)
        route.append(cur)
    return route


# =========================================================== photo dedup
def dhash_bits(rows):
    """Difference hash from an 8-row x 9-column grayscale grid (the caller
    resizes the image -- that is the only part needing OpenCV). Each bit is
    "is this pixel brighter than its right-hand neighbour", so the hash
    describes the picture's structure rather than its exposure: the same
    spot photographed twice hashes nearly the same even if the sun went in.
    Returns a 64-bit int."""
    h = 0
    for r in range(8):
        row = rows[r]
        for c in range(8):
            h = (h << 1) | (1 if row[c] > row[c + 1] else 0)
    return h


def hamming(a, b):
    return bin(a ^ b).count("1")


SAVE, SKIP, REPLACE = "save", "skip", "replace"


def dedup_decision(new, recent, radius_m, hash_max):
    """Decide what to do with a new detection photo.

    new / each of recent: dict with lat, lon (None when no fix), conf, hash.
    Returns (SAVE, None), (SKIP, why) or (REPLACE, the_recent_entry).

    Two independent ways a photo can be "the same one again":
      * POSITION -- within radius_m of an earlier photo. Only when both have
        a fix; a photo without one cannot be placed and is never matched
        this way.
      * PICTURE -- dHash within hash_max bits of a recent photo. This is
        what catches the cases position cannot: no fix at all, or the robot
        standing still (where GPS drift can wander a photo just past the
        radius while it shows exactly the same thing).
    On a match, the more confident photo wins: the new one REPLACES the old
    pair if it is more confident, else it is SKIPPED."""
    match, why = None, ""
    if new.get("lat") is not None:
        best = None
        for r in recent:
            if r.get("lat") is None:
                continue
            d = meters_between(new["lat"], new["lon"], r["lat"], r["lon"])
            if d <= radius_m and (best is None or d < best[0]):
                best = (d, r)
        if best:
            match, why = best[1], f"{best[0]:.1f}m from an earlier photo"
    if match is None and new.get("hash") is not None:
        best = None
        for r in recent:
            if r.get("hash") is None:
                continue
            h = hamming(new["hash"], r["hash"])
            if h <= hash_max and (best is None or h < best[0]):
                best = (h, r)
        if best:
            match, why = best[1], f"looks the same as an earlier photo " \
                                  f"({best[0]} bits apart)"
    if match is None:
        return SAVE, None
    if new.get("conf", 0.0) > match.get("conf", 0.0):
        return REPLACE, match
    return SKIP, why
