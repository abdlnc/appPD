#!/usr/bin/env python3
"""
Unit tests for agv_control.coverage_planner -- the RETURN TO START phase.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/coverage_planner_test.py

No ROS, no robot. The follower is driven straight into the end of the
fill-in pass (every cell marked covered) and then fed positions by hand, so
each case checks one rule: where it heads, when it calls itself home, when
it gives up. The closed-loop behaviour -- steering, GPS drift, the fence --
is ~/coverage_sim.py's job.
"""
import math
import sys

try:
    from agv_control import coverage_planner as cp
    from agv_control import field_geometry as fg
except ImportError:
    sys.exit("cannot import agv_control -- "
             "source ~/ros2_ws/install/setup.bash first")

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


FR = fg.LocalFrame(14.6320, 121.0900)
RECT = [(0, 0), (80, 0), (80, 40), (0, 40)]           # 80 x 40 m
plan, why = cp.make_plan([FR.to_ll(*p) for p in RECT], 3.0)
assert plan is not None, why


def finished_follower(home):
    """A follower that started at `home` and has covered everything."""
    f = cp.CoverageFollower(plan)
    f.update(0.0, home, 0)                    # first fix = the start
    for k in f.grid.cells:
        f.grid.cells[k] = {0}
    f.phase, f.resume_phase = cp.FILL, cp.FILL
    f.lane = len(f.plan.lanes) - 1
    return f


def give_heading(f, t, p):
    """Feed a 6 m straight run ending at p, so TrackHeading has a heading."""
    for k in range(7):
        q = (p[0] - 6 + k, p[1])
        out = f.update(t + k, q, +1)
    return out, t + 7


print("-- point_well_inside")
ins = plan.inset_poly                          # stop line: 3..77 x 3..37
q = fg.point_well_inside((40, 20), ins, 4.0)
chk("a point well inside is kept as it is", q == (40, 20))
q = fg.point_well_inside((5, 5), ins, 4.0)
chk("a corner start is moved in to 4 m inside the stop line",
    abs(q[0] - 7) < 1e-6 and abs(q[1] - 7) < 1e-6, f"{q}")
q = fg.point_well_inside((-10, 20), ins, 4.0)
chk("a start outside the boundary is brought in",
    fg.point_in_polygon(q, ins) and fg.dist_to_boundary(q, ins) >= 4.0 - 1e-6,
    f"{q}")
chk("a field too small for the margin gives None",
    fg.point_well_inside((1, 1), [(0, 0), (6, 0), (6, 6), (0, 6)], 4.0)
    is None)

print("\n-- the first fix of a run is the start")
f = cp.CoverageFollower(plan)
f.update(0.0, (30.0, 20.0), 0)
f.update(1.0, (31.0, 20.0), +1)
chk("home is the FIRST position, not a later one", f.home == (30.0, 20.0))

print("\n-- coverage done, far from the start: RETURN")
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
chk("phase becomes RETURN", f.phase == cp.RETURN, f.phase)
chk("and it drives (GO), not holds", out[0] == "GO", str(out))
chk("at the return speed cap", out[0] == "GO" and out[2] == cp.RETURN_CAP)
chk("status reports metres to go", f.status()["home_m"] is not None and
    abs(f.status()["home_m"] - math.dist((70, 30), (40, 20))) < 0.2,
    str(f.status()["home_m"]))
# heading east (+x) at (70,30); home (40,20) is behind and to the south --
# the right-hand side. Plain pursuit would barely steer (sin(a) ~ 0 behind).
want = -(cp.TURN_DIAMETER_M / 2) / cp.RETURN_TURN_RADIUS_M
chk("home behind-right: turns round RIGHT on the wide circle",
    out[0] == "GO" and abs(out[1] - want) < 1e-9, f"{out[1]:.2f}")
d0 = f.to_dict()
chk("the start point is saved with the progress",
    d0["home"] == [40.0, 20.0] and d0["phase"] == cp.RETURN)

print("\n-- turning round: the side with room for the circle")
# Heading east along the BOTTOM edge (y = 8.5, stop line at y = 3), start
# behind-right. Right means a 10 m circle down through the stop line.
f2 = finished_follower((10.0, 6.5))
out2, t2 = give_heading(f2, 10.0, (60.0, 8.5))
chk("start behind-right, but no room on the right: turns LEFT",
    out2[0] == "GO" and out2[1] > 0 and f2.turn_side == 1, str(out2))
out2 = f2.update(t2, (60.5, 9.5), +1)
chk("and keeps turning left while the start is still behind",
    out2[0] == "GO" and out2[1] > 0, str(out2))
f3 = finished_follower((10.0, 30.0))
out3, _ = give_heading(f3, 10.0, (60.0, 20.0))
chk("mid-field, start behind-left: room both sides, turns toward it",
    out3[0] == "GO" and out3[1] > 0 and f3.turn_side == 1, str(out3))

print("\n-- arriving")
out = f.update(t, (41.5, 20.5), +1)
chk("within HOME_RADIUS_M: DONE and holds", f.phase == cp.DONE and
    out[0] == "HOLD", str(out))
chk("says it is back", "back at the start" in out[1], out[1])
out = f.update(t + 1, (41.5, 20.5), 0)
chk("stays held afterwards", out[0] == "HOLD" and "back at the start"
    in out[1])
chk("status note says so", "back at the start" in f.status()["note"])

print("\n-- passing close but missing the radius (GPS drift)")
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
f.update(t, (43.5, 21.0), +1)                   # 3.6 m: inside NEAR, not home
chk("near but not home: still RETURN", f.phase == cp.RETURN)
f.update(t + 1, (37.0, 25.0), +1)               # 5.8 m: > 3.6 + 1.5 (passed)
chk("then moving away by HOME_PASSED_M: counts as arrived",
    f.phase == cp.DONE and "back at the start" in f.done_why, f.done_why)

print("\n-- a long way off and not getting closer: no false arrival")
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
f.update(t, (60.0, 30.0), +1)
f.update(t + 1, (66.0, 30.0), +1)               # further away, but never near
chk("still RETURN (never came within HOME_NEAR_M)", f.phase == cp.RETURN)

print("\n-- gives up rather than drive forever")
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
out = f.update(t + 10_000, (69.0, 30.0), +1)
chk("past RETURN_TIMEOUT: DONE, holds where it is",
    f.phase == cp.DONE and out[0] == "HOLD", str(out))
chk("and says it could not get back", "could not get back" in out[1], out[1])

print("\n-- started at the corner (outside the 4 m margin)")
f = finished_follower((5.0, 5.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
chk("heads for the start moved in, not the start itself",
    f.home_target is not None and abs(f.home_target[0] - 7) < 1e-6 and
    abs(f.home_target[1] - 7) < 1e-6, f"{f.home_target}")

print("\n-- finished right where it started: no drive back")
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (41.0, 20.0))
chk("DONE at once", f.phase == cp.DONE and "back at the start" in f.done_why,
    f.done_why)

print("\n-- RETURN_TO_START off: stops where it finishes (old behaviour)")
cp.RETURN_TO_START = False
f = finished_follower((40.0, 20.0))
out, t = give_heading(f, 10.0, (70.0, 30.0))
chk("DONE, plain 'coverage complete'", f.phase == cp.DONE and
    out == ("HOLD", "coverage complete"), str(out))
cp.RETURN_TO_START = True

print("\n-- a STOP mid-return resumes the return, to the ORIGINAL start")
g = cp.CoverageFollower(plan)
g.load_dict(d0)
chk("loaded home is the original start", g.home == (40.0, 20.0))
g.update(100.0, (65.0, 25.0), 0)                # first fix after resume
chk("does not take the resume position as the start",
    g.home == (40.0, 20.0))
out, t = give_heading(g, 101.0, (65.0, 25.0))
chk("goes back to RETURN after rebuilding its heading",
    g.phase == cp.RETURN and out[0] == "GO", f"{g.phase} {out}")
g.reset_heading()
chk("STOP during RETURN: resumes as RETURN",
    g.phase == cp.ACQUIRE and g.resume_phase == cp.RETURN)

print("\n-- an older save with no start point")
old = dict(d0)
old.pop("home")
old["phase"] = cp.LANE
g = cp.CoverageFollower(plan)
g.load_dict(old)
g.update(0.0, (20.0, 10.0), 0)
chk("the start is taken at the next fix instead", g.home == (20.0, 10.0))

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
