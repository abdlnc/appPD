#!/usr/bin/env python3
"""
Closed-loop simulation of COVER mode -- no ROS, no robot.

Drives the REAL planner and lane follower (agv_control.coverage_planner)
with a simulated robot that steers like this one and a GPS that errs like
this one, then reports how it went and draws it.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/coverage_sim.py                         # 80 x 40 m field
    python3 ~/coverage_sim.py --field 120 60 --noise 1.25 --png ~/cover.png
    python3 ~/coverage_sim.py --spacing 4 --turn 6    # forces loop-out turns
    python3 ~/coverage_sim.py --plan-only --field 100 50 --speed 0.55

WHAT IS MODELLED, because each one is a way a lane follower fails:
  * Ackermann steering: turn rate = speed / turning radius x steer. It
    cannot pivot. Steering slews (it takes ~0.7 s to reach full lock).
  * GPS at 1 Hz, with first-order Gauss-Markov error -- it DRIFTS over tens
    of seconds, which fools a controller in a way white noise never does.
  * The fence acting on the NOISY position, exactly as fence_node does,
    with motor_node's rule (forward blocked at the line) and a simplified
    control_node recovery (reverse until back inside, turn in).

WHAT IS NOT: obstacles, the camera, wheel slip. Lidar avoidance is
control_node's job and has its own tests; this is about the route.

TWO coverage figures are reported, and the gap between them matters:
  believed  -- what the robot's own grid says (painted with GPS positions)
  actual    -- the same grid painted with where the robot REALLY was
With noisy GPS the robot believes in lanes it did not quite drive.
"""

import argparse
import math
import random
import sys

try:
    from agv_control import coverage_planner as cp
    from agv_control import field_geometry as fg
except ImportError:
    sys.exit("cannot import agv_control -- source ~/ros2_ws/install/setup.bash")

LAT0, LON0 = 14.6320, 121.0900
DT = 0.1                      # physics step, s
GPS_DT = 1.0                  # GGA rate
V_MAX = 0.6                   # m/s at command 1.0 (gps_node sim figure)
STEER_RATE = 1.5              # full-scale steer change per second (servo slew)
REV_SPEED = 0.40              # control_node REV_SPEED_ESCALATED
INSET = 3.0
BACKOFF = 6.0                 # control_node FENCE_BACKOFF_M


def run(args, rng):
    fr = fg.LocalFrame(LAT0, LON0)
    W, H = args.field
    rect = [(0, 0), (W, 0), (W, H), (0, H)]
    ll = [fr.to_ll(*p) for p in rect]
    global INSET
    INSET = args.inset
    cp.TURN_DIAMETER_M = args.turn
    plan, why = cp.make_plan(ll, INSET, spacing=args.spacing,
                             turn_diameter=args.turn)
    if plan is None:
        print("PLAN REFUSED:", why)
        return None
    if args.plan_only:
        return plan, None
    # plan.frame is anchored at the first corner -- same as fr here
    f = cp.CoverageFollower(plan)
    truth_grid = fg.CoverageGrid(plan.inset_poly, cp.COVER_CELL_M, plan.spacing)

    # start just inside a corner, facing along the long axis
    x, y = 6.0, 6.0
    start_true = (x, y)
    return_t0 = None             # when the drive back to the start began
    hdg = 90.0 - math.degrees(plan.angle)       # compass from math angle
    steer = 0.0
    ex = ey = 0.0
    t = 0.0
    next_gps = 0.0
    cmd = ("HOLD", "start")
    gps = None
    direction = 0
    path_true, path_gps = [], []
    fence_trips = 0
    holds = 0                    # times recovery gave up and held
    retries = 0                  # recoveries that stalled and started over
    hold_time = 0.0              # seconds spent held by the guard
    deeper_blocks = 0            # steps the NEVER DEEPER guard blocked
    band_min = None
    fence_track = fg.TrackHeading(min_baseline=3.0)   # as fence_node
    worst_out = 0.0              # furthest the TRUE robot got past the line
    recov = None                 # (phase, until_or_turn)
    last_true = None
    lane_xte = []
    T_MAX = args.minutes * 60.0

    def fence(p):
        return fg.fence_status(p, plan.poly, INSET)

    while t < T_MAX:
        # ---------------- GPS (1 Hz, drifting error)
        if t >= next_gps:
            next_gps += GPS_DT
            a = GPS_DT / args.tau
            k = args.noise * math.sqrt(2 * a)
            ex += -ex * a + k * rng.gauss(0, 1)
            ey += -ey * a + k * rng.gauss(0, 1)
            gps = (x + ex, y + ey)
            path_gps.append(gps)
            fence_track.feed(t, gps[0], gps[1], direction)
            if recov is None:
                cmd = f.update(t, gps, direction)
                if f.phase == cp.RETURN and return_t0 is None:
                    return_t0 = t
                if f.phase == cp.DONE:
                    break


        # ---------------- fence (on the NOISY position) + recovery
        # Mirrors fence_node + motor_node + control_node.fence_recover.
        if gps:
            fstate = fence(gps)
            depth = fg.past_line_depth(gps, plan.poly, INSET)
            inward = fg.inward_bearing(gps, plan.poly, plan.inset_poly)
        else:
            fstate, depth, inward = "IN", -99.0, 0.0
        h = fence_track.heading()        # fence_node's own, 3 m baseline
        allow = (+1 if h is not None and fg.allowed_direction(h, inward) > 0
                 else -1)
        turn = None
        if h is not None:
            turn = -1.0 if fg.norm180(inward - h) > 0 else 1.0   # R->-1 L->+1
        if fstate in ("LINE", "OUT"):
            band_min = depth if band_min is None else min(band_min, depth)
        elif -depth >= 2.0:                       # FENCE_BAND_RESET_M
            band_min = None
        want_v, want_s = 0.0, 0.0
        if recov is None and fstate in ("LINE", "OUT"):
            fence_trips += 1
            recov = {"ph": "BACK", "best": depth, "best_t": t, "dir": None,
                     "until": t + 45.0}
        if recov is not None:
            ph = recov["ph"]
            if ph == "BACK":
                margin = -depth
                if fstate == "IN" and margin >= BACKOFF:
                    recov = ({"ph": "TURN", "until": t + 6.0, "turn": turn}
                             if turn is not None else None)
                else:
                    if depth < recov["best"] - 0.2:
                        recov["best"], recov["best_t"] = depth, t
                    if t - recov["best_t"] > 8.0 or t > recov["until"]:
                        # As control_node: held for good only if the NEVER
                        # DEEPER guard is what stopped it; otherwise the way
                        # back is clear (no obstacles here), so try again.
                        if band_min is not None and depth > band_min + 1.5:
                            holds += 1
                            recov = {"ph": "HOLD"}
                        else:
                            retries += 1
                            recov = None
                    else:
                        fwd = (fstate != "IN" and allow > 0) or \
                              (fstate == "IN" and recov["dir"] == "F")
                        if fwd:
                            want_v = min(0.5, REV_SPEED)
                            want_s = turn if turn is not None else 0.0
                            recov["dir"] = "F"
                        else:
                            want_v = -REV_SPEED
                            recov["dir"] = "R"
            elif ph == "TURN":
                want_s = recov["turn"]
                want_v = 0.5 if fstate == "IN" else 0.0
                if t > recov["until"] or fstate != "IN":
                    recov = None
            elif ph == "HOLD":
                # As control_node's hold loop: wait while the NEVER DEEPER
                # guard holds it, release once it clears (GPS error drifting
                # back), then try again. Releasing cannot add up to an
                # escape: band_min only resets once back IN, so the guard
                # keeps measuring from the same shallowest point.
                hold_time += DT
                if band_min is None or depth <= band_min + 1.5:
                    recov = None
        elif cmd[0] == "GO":
            want_s, want_v = cmd[1], min(1.0, cmd[2])
        # motor_node: only the way back in, and NEVER DEEPER
        if fstate in ("LINE", "OUT"):
            if band_min is not None and depth > band_min + 1.5:
                want_v = 0.0
                deeper_blocks += 1
            elif allow > 0 and want_v < 0:
                want_v = 0.0
            elif allow < 0 and want_v > 0:
                want_v = 0.0
        if recov is not None and recov.get("ph") == "HOLD":
            want_v = 0.0
        direction = 1 if want_v > 0.02 else (-1 if want_v < -0.02 else 0)

        # ---------------- physics (bicycle model, steering slew)
        ds = max(-STEER_RATE * DT, min(STEER_RATE * DT, want_s - steer))
        steer += ds
        v = want_v * V_MAX
        yaw = math.degrees(v / (args.turn / 2.0)) * steer   # +steer = LEFT
        hdg = (hdg - yaw * DT) % 360.0
        br = math.radians(hdg)
        x += v * DT * math.sin(br)
        y += v * DT * math.cos(br)
        p = (x, y)
        path_true.append(p)
        if not fg.point_in_polygon(p, plan.poly):
            worst_out = max(worst_out, fg.dist_to_boundary(p, plan.poly))
        if last_true and f.phase in (cp.LANE, cp.TURN, cp.FILL) and \
                fg.point_in_polygon(p, plan.inset_poly):
            truth_grid.mark_segment(last_true, p, f.lane)
        last_true = p
        if f.phase == cp.LANE and recov is None:
            a, b = plan.lanes[f.lane] if f.lane < len(f.plan.lanes) else (p, p)
            a, b = f.plan.lanes[f.lane]
            lane_xte.append(abs(fg.cross_track(a, b, p)))
        t += DT

    res = {
        "time_min": t / 60.0,
        "done": f.phase == cp.DONE,
        "phase": f.phase,
        "lanes": f"{min(f.lane + 1, len(f.plan.lanes))}/{len(f.plan.lanes)}",
        "believed_cov": f.grid.coverage_pct(),
        "actual_cov": truth_grid.coverage_pct(),
        "overlap": f.grid.overlap_pct(),
        "fence_trips": fence_trips,
        "worst_out_m": worst_out,
        "xte_median": sorted(lane_xte)[len(lane_xte) // 2] if lane_xte else 0,
        "xte_p95": sorted(lane_xte)[int(len(lane_xte) * .95)] if lane_xte else 0,
        "fill_done": f.fill_done,
        "holds": holds,
        "hold_min": hold_time / 60.0,
        "retries": retries,
        "deeper_blocks": deeper_blocks,
        "held_at_end": recov is not None and recov.get("ph") == "HOLD",
        "end_note": f.done_why,
        "return_min": ((t - return_t0) / 60.0 if return_t0 is not None
                       else 0.0),
        # TRUE distances at the end: to where it really started, and to the
        # point it aimed for (the start moved well inside the stop line)
        "home_err_m": math.dist((x, y), start_true),
        "target_err_m": (math.dist((x, y), f.home_target)
                         if f.home_target is not None else None),
    }
    return plan, (res, path_true, path_gps, f)


def draw(plan, path_true, path_gps, fname):
    import cv2
    import numpy as np
    pts = plan.poly
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    S = 8.0
    pad = 30
    w = int((max(xs) - min(xs)) * S) + 2 * pad
    h = int((max(ys) - min(ys)) * S) + 2 * pad
    img = np.full((h, w, 3), 255, np.uint8)

    def px(p):
        return (int((p[0] - min(xs)) * S) + pad,
                h - (int((p[1] - min(ys)) * S) + pad))
    cv2.polylines(img, [np.array([px(p) for p in plan.poly])], True,
                  (40, 40, 40), 2)
    cv2.polylines(img, [np.array([px(p) for p in plan.inset_poly])], True,
                  (0, 140, 255), 1)
    for a, b in plan.lanes:
        cv2.line(img, px(a), px(b), (200, 200, 200), 1)
    for i in range(1, len(path_gps)):
        cv2.line(img, px(path_gps[i - 1]), px(path_gps[i]), (230, 180, 120), 1)
    for i in range(1, len(path_true), 3):
        cv2.line(img, px(path_true[i - 1]), px(path_true[i]), (60, 140, 40), 2)
    cv2.putText(img, "boundary(black) stop line(orange) lanes(grey) "
                "true path(green) GPS(blue)", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(fname, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", nargs=2, type=float, default=[80.0, 40.0],
                    metavar=("W", "H"), help="field size, metres")
    ap.add_argument("--spacing", type=float, default=cp.LANE_SPACING_M)
    ap.add_argument("--turn", type=float, default=cp.TURN_DIAMETER_M,
                    help="turning diameter at full lock, metres")
    ap.add_argument("--speed", type=float, default=cp.CRUISE_SPEED_MPS,
                    help="cruise ground speed, m/s (reporting only)")
    ap.add_argument("--tau", type=float, default=30.0,
                    help="GPS error correlation time, s. 120 with --noise 4.5 "
                         "matches 5 min of this robot's receiver, parked, on "
                         "2026-10-19 (sigma 3.0 m E / 5.4 m N, 10 m excursions)")
    ap.add_argument("--inset", type=float, default=3.0,
                    help="stop line distance inside the boundary, m")
    ap.add_argument("--noise", type=float, default=1.25,
                    help="GPS error sigma, m (0 = perfect)")
    ap.add_argument("--minutes", type=float, default=240.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--png", default=None)
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--seeds", type=int, default=0,
                    help="run this many GPS-noise seeds and summarise")
    args = ap.parse_args()

    if args.seeds:
        rows = []
        for sd in range(1, args.seeds + 1):
            out = run(args, random.Random(sd))
            if out is None:
                sys.exit(1)
            r = out[1][0]
            rows.append(r)
            print(f"  seed {sd:2d}: {'done' if r['done'] else r['phase']:7s} "
                  f"actual {r['actual_cov']:5.1f}%  trips {r['fence_trips']:3d}  "
                  f"xte95 {r['xte_p95']:5.2f} m  past boundary "
                  f"{r['worst_out_m']:.2f} m  ends {r['home_err_m']:5.1f} m "
                  f"from start")
        n = len(rows)
        fin = sum(r["done"] for r in rows)
        out_max = max(r["worst_out_m"] for r in rows)
        print(f"{n} runs, field {args.field[0]:.0f}x{args.field[1]:.0f} m, "
              f"stop line {args.inset} m in, GPS sigma {args.noise} m, "
              f"turning diameter {args.turn} m")
        print(f"  finished          {fin}/{n}")
        print(f"  actual coverage   mean {sum(r['actual_cov'] for r in rows)/n:.1f}%"
              f"   worst {min(r['actual_cov'] for r in rows):.1f}%")
        print(f"  believed coverage mean {sum(r['believed_cov'] for r in rows)/n:.1f}%")
        print(f"  overlap           mean {sum(r['overlap'] for r in rows)/n:.1f}%")
        print(f"  lane tracking     median-of-medians "
              f"{sorted(r['xte_median'] for r in rows)[n//2]:.2f} m, worst 95th "
              f"{max(r['xte_p95'] for r in rows):.2f} m")
        print(f"  stop-line trips   total {sum(r['fence_trips'] for r in rows)}, "
              f"held {sum(r['holds'] for r in rows)}x for "
              f"{sum(r['hold_min'] for r in rows):.1f} min in all")
        tgt = [r["target_err_m"] for r in rows
               if r["done"] and r["target_err_m"] is not None]
        if tgt:
            print(f"  back at the start  {sum('back at the start' in r['end_note'] for r in rows)}/{n};"
                  f" true distance from the return point median "
                  f"{sorted(tgt)[len(tgt)//2]:.1f} m, worst {max(tgt):.1f} m")
        print(f"  WORST distance past the BOUNDARY: {out_max:.2f} m"
              f"   {'<- MUST BE 0' if out_max > 0 else '(never crossed)'}")
        return

    out = run(args, random.Random(args.seed))
    if out is None:
        sys.exit(1)
    plan, sim = out
    s = plan.summary(args.speed)
    print(f"PLAN  field {args.field[0]:.0f} x {args.field[1]:.0f} m, "
          f"stop line {INSET} m in, turning diameter {args.turn} m")
    print(f"      {s['lanes']} lanes {s['spacing_m']} m apart, "
          f"{s['lane_length_m']} m of lane, route {s['route_length_m']} m, "
          f"{s['turns']} turns ({s['loop_out_turns']} loop-out)")
    print(f"      at {args.speed} m/s: about {s['minutes']} min, soil photo "
          f"every {s['soil_spacing_m']} m -> about {s['soil_photos']} photos")
    if sim is None:
        return
    res, pt, pg, f = sim
    print(f"RUN   {'finished' if res['done'] else 'NOT finished (' + res['phase'] + ')'}"
          f" in {res['time_min']:.1f} min, lane {res['lanes']}, "
          f"{res['fill_done']} fill-in points")
    print(f"      coverage: believed {res['believed_cov']:.1f}%   "
          f"actual {res['actual_cov']:.1f}%   overlap {res['overlap']:.1f}%")
    print(f"      lane tracking (true): median {res['xte_median']:.2f} m, "
          f"95% within {res['xte_p95']:.2f} m")
    print(f"      fence: {res['fence_trips']} stop-line trips, "
          f"{res['retries']} retried, {res['holds']} hold(s){' (HELD at the end)' if res['held_at_end'] else ''}, "
          f"never-deeper guard fired {res['deeper_blocks']}x")
    print(f"      robot got {res['worst_out_m']:.2f} m past the BOUNDARY at worst")
    if res["end_note"]:
        tgt = res["target_err_m"]
        print(f"      end: {res['end_note']}")
        print(f"      drive back took {res['return_min']:.1f} min; really "
              f"{res['home_err_m']:.1f} m from where it started"
              + (f", {tgt:.1f} m from the return point "
                 f"({f.home_target[0]:.1f}, {f.home_target[1]:.1f})"
                 if tgt is not None else ""))
    if args.png:
        draw(plan, pt, pg, args.png)
        print("      drawn ->", args.png)


if __name__ == "__main__":
    main()
