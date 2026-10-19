#!/usr/bin/env python3
"""
Unit tests for agv_control.field_geometry -- the boundary fence, lanes,
coverage grid and photo de-duplication maths.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/field_geometry_test.py

No ROS graph, no robot, no GPS: pure geometry on made-up fields. The fence
cases are the ones that matter most -- a wrong IN where it should say LINE
or OUT is the robot driving off the field.
"""
import math
import sys

try:
    from agv_control import field_geometry as fg
except ImportError:
    sys.exit("cannot import agv_control.field_geometry -- "
             "source ~/ros2_ws/install/setup.bash first")

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


SQ = [(0, 0), (40, 0), (40, 40), (0, 40)]                  # 40 x 40 m
L_SHAPE = [(0, 0), (60, 0), (60, 20), (20, 20), (20, 60), (0, 60)]

print("-- local frame")
fr = fg.LocalFrame(14.6320, 121.0900)
x, y = fr.to_xy(14.6329, 121.0911)
la, lo = fr.to_ll(x, y)
chk("lat/lon -> x/y -> lat/lon round-trips", close(la, 14.6329, 1e-9)
    and close(lo, 121.0911, 1e-9))
chk("frame distance agrees with meters_between",
    close(math.hypot(x, y), fg.meters_between(14.6320, 121.0900,
                                              14.6329, 121.0911), 0.05),
    f"{math.hypot(x, y):.2f} m")
chk("x is east, y is north", fr.to_xy(14.6320, 121.0901)[0] > 0
    and fr.to_xy(14.6321, 121.0900)[1] > 0)

print("\n-- polygons")
chk("CCW square has positive area", close(fg.polygon_area(SQ), 1600))
chk("CW square has negative area", close(fg.polygon_area(SQ[::-1]), -1600))
chk("ccw() normalises winding", fg.polygon_area(fg.ccw(SQ[::-1])) > 0)
chk("square is simple", fg.is_simple(SQ))
chk("L-shape is simple", fg.is_simple(L_SHAPE))
chk("figure-of-eight is not simple",
    not fg.is_simple([(0, 0), (10, 10), (10, 0), (0, 10)]))
chk("two corners only is not a polygon", not fg.is_simple([(0, 0), (1, 1)]))

print("\n-- point in polygon")
chk("centre of square is inside", fg.point_in_polygon((20, 20), SQ))
chk("outside square is outside", not fg.point_in_polygon((50, 20), SQ))
chk("winding does not matter", fg.point_in_polygon((20, 20), SQ[::-1]))
chk("inside the L's arm", fg.point_in_polygon((10, 50), L_SHAPE))
chk("the L's notch is OUTSIDE", not fg.point_in_polygon((40, 40), L_SHAPE))

print("\n-- the fence: IN / LINE / OUT  (40 m square, 3 m stop line)")
chk("middle of the field is IN", fg.fence_status((20, 20), SQ, 3) == "IN")
chk("1 m from an edge is LINE", fg.fence_status((20, 1), SQ, 3) == "LINE")
chk("2.9 m from an edge is LINE", fg.fence_status((20, 2.9), SQ, 3) == "LINE")
chk("3.1 m from an edge is IN", fg.fence_status((20, 3.1), SQ, 3) == "IN")
chk("past the edge is OUT", fg.fence_status((20, -1), SQ, 3) == "OUT")
chk("in the L's notch is OUT", fg.fence_status((40, 40), L_SHAPE, 3) == "OUT")
chk("(22,22) is in the L's notch, so OUT", fg.fence_status((22, 22), L_SHAPE, 3) == "OUT")
chk("near the L's INNER corner is LINE (concave, exact)",
    fg.fence_status((18, 18), L_SHAPE, 3) == "LINE",
    f"{fg.dist_to_boundary((18, 18), L_SHAPE):.2f} m from the corner")
chk("inset 0 = only OUT outside", fg.fence_status((20, 0.1), SQ, 0) == "IN")

print("\n-- which way is back in")
def bdiff(a, b):
    return abs(fg.norm180(a - b))
chk("near the north edge, back in is south",
    bdiff(fg.inward_bearing((20, 39), SQ), 180) < 1e-6)
chk("near the east edge, back in is west",
    bdiff(fg.inward_bearing((39, 20), SQ), 270) < 1e-6)
chk("outside east, back in is west",
    bdiff(fg.inward_bearing((45, 20), SQ), 270) < 1e-6)
chk("winding does not flip it",
    bdiff(fg.inward_bearing((20, 39), SQ[::-1]), 180) < 1e-6)

print("\n-- inset (the stop line, for drawing and planning)")
ins = fg.inset_polygon(SQ, 3)
chk("square inset by 3 is 34 x 34", ins is not None
    and close(abs(fg.polygon_area(ins)), 34 * 34, 1e-6))
chk("every inset corner is 3 m inside",
    ins and all(close(fg.dist_to_boundary(v, SQ), 3, 1e-6) for v in ins))
insL = fg.inset_polygon(L_SHAPE, 3)
chk("concave L insets cleanly", insL is not None and fg.is_simple(insL))
thin = [(0, 0), (100, 0), (100, 4), (0, 4)]
chk("a 4 m-wide strip cannot take a 3 m inset", fg.inset_polygon(thin, 3) is None)
spike = [(0, 0), (100, 0), (0, 3)]
chk("a needle triangle refuses rather than spike",
    fg.inset_polygon(spike, 2) is None)

print("\n-- boundary validation")
def ll(xy):
    return [fr.to_ll(*p) for p in xy]
okv, why, pts = fg.validate_boundary(ll(SQ), 3)
chk("a 40 m square is accepted", okv, why)
okv, why, _ = fg.validate_boundary(ll(SQ[:2]), 3)
chk("two corners are refused", not okv, why)
okv, why, _ = fg.validate_boundary(ll([(0, 0), (40, 40), (40, 0), (0, 40)]), 3)
chk("crossing edges are refused", not okv, why)
okv, why, _ = fg.validate_boundary(ll([(0, 0), (5, 0), (5, 5), (0, 5)]), 1)
chk("a 5 m square is refused as too small", not okv, why)
okv, why, _ = fg.validate_boundary(ll(thin), 3)
chk("too narrow for the stop line is refused", not okv, why)
okv, why, pts = fg.validate_boundary(ll(SQ + [SQ[3], (0, 40.1)]), 3)
chk("repeated taps collapse to one corner", okv and len(pts) == 4,
    f"{len(pts) if pts else 0} corners")
okv, why, pts = fg.validate_boundary(ll(SQ + [SQ[0]]), 3)
chk("tapping the first corner again just closes it", okv and len(pts) == 4)
okv, why, _ = fg.validate_boundary([(91, 0), (0, 0), (0, 1)], 3)
chk("an impossible latitude is refused", not okv, why)

print("\n-- lanes")
RECT = [(0, 0), (120, 0), (120, 40), (0, 40)]
ang = fg.long_axis_angle(RECT)
chk("long axis of a 120x40 field is east-west",
    close(abs(math.cos(ang)), 1.0, 1e-6), f"{math.degrees(ang):.1f} deg")
rot30 = [fg._rot(p, math.radians(30)) for p in RECT]
chk("and follows the field when it is rotated 30 deg",
    bdiff(math.degrees(fg.long_axis_angle(rot30)) % 180, 30) < 1e-6)
insR = fg.inset_polygon(RECT, 3)            # 114 x 34
lanes, real = fg.lawnmower_lanes(insR, 6.0, ang)
chk("34 m wide at <= 6 m spacing needs 6 lanes", len(lanes) == 6,
    f"{len(lanes)} lanes, {real:.2f} m apart")
chk("real spacing never wider than asked", real <= 6.0 + 1e-9)
d0 = (lanes[0][1][0] - lanes[0][0][0])
d1 = (lanes[1][1][0] - lanes[1][0][0])
chk("consecutive lanes run opposite ways", d0 * d1 < 0)
chk("every lane end is inside the stop line",
    all(fg.fence_status(p, RECT, 3) != "OUT" and
        fg.dist_to_boundary(p, RECT) >= 3 - 1e-6
        for ln in lanes for p in ln))
lanes_m, _ = fg.lawnmower_lanes(insR, 6.0, ang, end_margin=5.0)
chk("end_margin trims each end", close(
    math.dist(*lanes_m[0]), math.dist(*lanes[0]) - 10.0, 1e-6))
U = [(0, 0), (90, 0), (90, 60), (60, 60), (60, 20), (30, 20), (30, 60), (0, 60)]
insU = fg.inset_polygon(U, 3)
lanesU, _ = fg.lawnmower_lanes(insU, 6.0, math.pi / 2)
left_arm = [ln for ln in lanesU if max(ln[0][0], ln[1][0]) < 30]
right_arm = [ln for ln in lanesU if min(ln[0][0], ln[1][0]) > 60]
chk("a U-shaped field gets lanes in BOTH arms",
    insU is not None and left_arm and right_arm,
    f"{len(left_arm)} left, {len(right_arm)} right, {len(lanesU)} total")
chk("no U lane crosses the gap",
    all(fg.point_in_polygon(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), U)
        for a, b in lanesU))

print("\n-- turns")
wp = fg.turn_waypoints((0, 0), (50, 0), (50, 8), turn_diameter=6.0)
chk("spacing >= turning diameter: straight to the next lane", wp == [(50, 8)])
wp = fg.turn_waypoints((0, 0), (50, 0), (50, 4), turn_diameter=8.0)
chk("too tight: a 4-point loop-out", len(wp) == 4 and wp[-1] == (50, 4))
ys = [p[1] for p in wp[:3]]
chk("the loop swings out by (D - s)/2 on BOTH sides",
    close(min(ys), -2.0) and close(max(ys), 6.0), f"y {min(ys)} .. {max(ys)}")
chk("and goes a turning radius past the lane end",
    close(max(p[0] for p in wp[:3]), 54.0), f"{max(p[0] for p in wp[:3])}")
wp = fg.turn_waypoints((60, 0), (10, 0), (10, 4), turn_diameter=8.0)
chk("works heading the other way too", max(p[0] for p in wp[:3]) < 10)
wp = fg.turn_waypoints((0, 0), (50, 0), (53, 4), turn_diameter=8.0)
chk("unequal lane ends: bulb starts past the FURTHER end",
    min(p[0] for p in wp[:3]) >= 53 - 1e-9)

print("\n-- heading from positions")
th = fg.TrackHeading(min_baseline=3.0)
th.feed(0, 0, 0, +1)
th.feed(1, 0, 1, +1)
chk("1 m of travel is not enough", th.heading() is None)
th.feed(2, 0, 3.5, +1)
chk("3.5 m north = heading 0", close(th.heading(), 0.0, 1e-6))
th.feed(3, 0, 2.5, -1)
chk("reversing restarts the track", th.heading() is None)
th.feed(4, 0, 1.0, -1)
th.feed(5, 0, -1.0, -1)
chk("backing south while facing north reads 0",
    close(th.heading() % 360, 0.0, 1e-6), f"{th.heading()}")
slow = fg.TrackHeading(min_baseline=5.0)
for k in range(41):                       # 0.18 m/s for 40 s = 7.2 m east
    slow.feed(float(k), 0.18 * k, 0.0, +1)
chk("a slow creep still gets a heading (default max_age)",
    slow.heading() is not None and close(slow.heading(), 90.0, 1e-6),
    f"{slow.heading()}")
th2 = fg.TrackHeading(min_baseline=3.0, max_age=5.0)
th2.feed(0, 0, 0, +1)
th2.feed(10, 0, 3.5, +1)
chk("positions older than max_age are forgotten", th2.heading() is None)

print("\n-- following a lane")
pt, frac = fg.lookahead_point((0, 0), (100, 0), (10, -4), 8.0)
chk("lookahead is ON the lane line, ahead of the robot",
    close(pt[1], 0.0) and close(pt[0], 18.0))
chk("progress along a lane", close(fg.progress_along((0, 0), (100, 0),
                                                      (25, 7)), 0.25))
chk("cross-track: left of the lane is +",
    fg.cross_track((0, 0), (100, 0), (10, 2)) > 0)
s = fg.steer_toward((0, 0), 0.0, (-5, 10), 45.0)
chk("target ahead-left steers LEFT (+)", s > 0, f"{s:.2f}")
s = fg.steer_toward((0, 0), 0.0, (5, 10), 45.0)
chk("target ahead-right steers RIGHT (-)", s < 0, f"{s:.2f}")
s = fg.steer_toward((0, 0), 0.0, (100, -1), 45.0)
chk("target way off to the side is full lock", close(abs(s), 1.0))

print("\n-- coverage grid  (3 m cells, 6 m swath)")
g = fg.CoverageGrid(insR, 3.0, 6.0)
chk("only cells inside the stop line count", len(g.cells) > 0 and all(
    fg.point_in_polygon(g.centre(i, j), insR) for i, j in g.cells))
for k, (a, b) in enumerate(lanes):
    g.mark_segment(a, b, k)
cov = g.coverage_pct()
chk("driving every lane covers nearly all of it", cov > 90, f"{cov:.1f}%")
chk("plain lanes overlap little", g.overlap_pct() < 25,
    f"{g.overlap_pct():.1f}% overlap")
g2 = fg.CoverageGrid(insR, 3.0, 6.0)
g2.mark_segment(lanes[0][0], lanes[0][1], 0)
g2.mark_segment(lanes[0][0], lanes[0][1], 1)       # same lane driven twice
chk("driving a lane twice is 100% overlap", close(g2.overlap_pct(), 100.0))
g3 = fg.CoverageGrid(insR, 3.0, 6.0)
g3.mark_segment(lanes[0][0], lanes[0][1], 0)
chk("missed() lists what is not yet covered",
    len(g3.missed()) == len(g3.cells) - sum(1 for v in g3.cells.values() if v))
g4 = fg.CoverageGrid(insR, 3.0, 6.0)
g4.load_dict(g3.to_dict())
chk("progress survives a save and reload",
    close(g4.coverage_pct(), g3.coverage_pct()))

print("\n-- fill-in route")
r = fg.nearest_neighbour_route((0, 0), [(10, 0), (1, 0), (5, 0)])
chk("visits nearest first", r == [(1, 0), (5, 0), (10, 0)])

print("\n-- photo de-duplication")
flat = [[100] * 9 for _ in range(8)]
grad = [[c * 20 for c in range(9)] for _ in range(8)]
grad_r = [[(8 - c) * 20 for c in range(9)] for _ in range(8)]
grad_lit = [[c * 20 + 30 for c in range(9)] for _ in range(8)]
chk("same picture, same hash", fg.dhash_bits(grad) == fg.dhash_bits(grad))
chk("mirror image differs completely",
    fg.hamming(fg.dhash_bits(grad), fg.dhash_bits(grad_r)) == 64)
chk("a brighter shot of the same thing hashes the same",
    fg.dhash_bits(grad) == fg.dhash_bits(grad_lit))
# DIFF differs from 1 in all 64 bits: a genuinely different picture. (An
# earlier draft used 2**63, which is only 2 bits from 1 -- the picture check
# rightly called that the same scene and the "new spot" test failed.)
DIFF = (2**64 - 1) ^ 1
chk("(test data: DIFF really is a different picture)", fg.hamming(1, DIFF) == 64)
base = {"lat": 14.632000, "lon": 121.090000, "conf": 0.70, "hash": 1}
near = dict(base, lat=14.632010, conf=0.60, hash=DIFF)          # ~1.1 m
far = dict(base, lat=14.632100, conf=0.60, hash=DIFF)           # ~11 m
chk("1 m away and less sure: skip",
    fg.dedup_decision(near, [base], 3.0, 6)[0] == fg.SKIP)
chk("1 m away and MORE sure: replace the old pair",
    fg.dedup_decision(dict(near, conf=0.90), [base], 3.0, 6) ==
    (fg.REPLACE, base))
chk("11 m away: a new spot, save",
    fg.dedup_decision(far, [base], 3.0, 6)[0] == fg.SAVE)
nofix_same = {"lat": None, "lon": None, "conf": 0.6, "hash": 1}
chk("no fix but the same picture: skip",
    fg.dedup_decision(nofix_same, [base], 3.0, 6)[0] == fg.SKIP)
nofix_new = {"lat": None, "lon": None, "conf": 0.6, "hash": DIFF}
chk("no fix and a different picture: save",
    fg.dedup_decision(nofix_new, [base], 3.0, 6)[0] == fg.SAVE)
drift = dict(far, hash=1)
chk("standing still with GPS drift past the radius: caught by picture",
    fg.dedup_decision(drift, [base], 3.0, 6)[0] == fg.SKIP)
chk("nothing saved yet: save",
    fg.dedup_decision(base, [], 3.0, 6)[0] == fg.SAVE)

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
