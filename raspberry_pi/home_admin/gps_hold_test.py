#!/usr/bin/env python3
"""
Unit tests for gps_node's parked hold (StationaryHold) and its NMEA
checksum check.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/gps_hold_test.py

No robot, no receiver: positions and drive commands are fed by hand.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.expanduser("~"))
import gps_node as g                          # noqa: E402

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


LAT0, LON0 = 14.6320, 121.0900
M = 1.0 / g.M_PER_DEG_LAT                     # one metre of latitude


def metres(a, b):
    return math.hypot((b[0] - a[0]) * g.M_PER_DEG_LAT,
                      (b[1] - a[1]) * g.M_PER_DEG_LAT *
                      math.cos(math.radians(a[0])))


def drift(rng, n, sigma=3.0):
    """n fixes of a parked receiver wandering about (LAT0, LON0)."""
    e = [0.0, 0.0]
    out = []
    for _ in range(n):
        e = [0.97 * e[0] + rng.gauss(0, sigma * 0.25),
             0.97 * e[1] + rng.gauss(0, sigma * 0.25)]
        out.append((LAT0 + e[1] * M, LON0 + e[0] * M))
    return out


print("-- parked since boot")
h = g.StationaryHold()
fixes = drift(random.Random(1), 7200)         # an hour at 2 Hz
outs = [h.apply(i * 0.5, la, lo) for i, (la, lo) in enumerate(fixes)]
spread = max(metres((LAT0, LON0), f) for f in fixes)
chk("(test data: the raw fixes really do wander)", spread > 5,
    f"up to {spread:.1f} m")
chk("published position never moves", len({(a, b) for a, b, _ in outs}) == 1)
chk("and it is the first fix", outs[0][:2] == fixes[0])
chk("flagged as held", all(x[2] for x in outs))
chk("logged once", "parked" in (h.take_event() or "") and
    h.take_event() is None)

print("\n-- a 113 m jump while parked (seen 2026-10-21) is not followed")
la, lo, held = h.apply(4000.0, LAT0 - 113 * M, LON0 + 27 * M)
chk("held point unchanged", held and (la, lo) == fixes[0])

print("\n-- driving")
h = g.StationaryHold()
h.apply(0.0, LAT0, LON0)
h.take_event()
h.commanded(1.0, 0.5)
la, lo, held = h.apply(1.2, LAT0 + 0.3 * M, LON0)
chk("a drive command releases at once: raw position out", not held and
    la == LAT0 + 0.3 * M)
ev = h.take_event()
chk("and the release is still there to log after the next fix",
    ev is not None and "released" in ev, str(ev))
chk("reported once", h.take_event() is None)
h.commanded(1.5, -0.4)                        # reversing is driving too
la, lo, held = h.apply(2.0, LAT0 + 0.6 * M, LON0)
chk("reverse commands: raw", not held)
h.commanded(2.0, 0.0)                         # stop command
la, lo, held = h.apply(3.0, LAT0 + 0.8 * M, LON0)
chk("just stopped (< HOLD_STILL_S): still raw -- it may be rolling",
    not held)
la, lo, held = h.apply(3.6, LAT0 + 0.9 * M, LON0)
chk("still for HOLD_STILL_S: held, at the latest fix",
    held and la == LAT0 + 0.9 * M and "parked" in (h.take_event() or ""))
la, lo, held = h.apply(30.0, LAT0 + 5.0 * M, LON0 + 4.0 * M)
chk("5 m of drift later: still the held point", held and
    la == LAT0 + 0.9 * M and lo == LON0)
h.commanded(31.0, 0.3)
la, lo, held = h.apply(31.5, LAT0 + 5.2 * M, LON0 + 4.0 * M)
chk("next drive: the live position again", not held and
    la == LAT0 + 5.2 * M)

print("\n-- steering only")
h = g.StationaryHold()
h.apply(0.0, LAT0, LON0)
h.commanded(1.0, 0.0)                         # angular only: linear 0
h.commanded(1.0, 0.01)                        # below MOVING_CMD
la, lo, held = h.apply(1.5, LAT0 + 3 * M, LON0)
chk("steering (or a tiny command) is not driving: stays held", held and
    la == LAT0)

print("\n-- the lidar: told to drive but not moving")


def scan(front, rear, side=3.0, noise=0.0, rng=None):
    """A 360-ray scan as the robot's lidar gives it (mounted rotated 180
    degrees): `front` m dead ahead, `rear` m dead behind, `side` elsewhere.
    None = nothing in range that way."""
    amin, inc = -math.pi, 2 * math.pi / 360
    out = []
    for i in range(360):
        deg = (math.degrees(amin + i * inc) + g.LIDAR_ANGLE_OFFSET_DEG) % 360
        if deg <= 20 or deg >= 340:
            d = front
        elif abs(deg - 180) <= 20:
            d = rear
        else:
            d = side
        d = math.inf if d is None else d
        if rng and not math.isinf(d):
            d += rng.gauss(0, noise)
        out.append(d)
    return out, amin, inc


rng = random.Random(3)


def run(hold, t0, secs, front_at, rear_at, cmd=0.5):
    """Drive commands at 10 Hz and scans at 10 Hz for `secs`; front/rear
    distances as functions of time. Returns the last apply()."""
    t = t0
    out = None
    while t < t0 + secs:
        hold.commanded(t, cmd)
        f, r = front_at(t), rear_at(t)
        hold.scan(t, *scan(f, r, noise=0.005, rng=rng))
        if abs((t * 2) - round(t * 2)) < 1e-6:          # fixes at 2 Hz
            out = hold.apply(t, LAT0 + rng.gauss(0, 3) * M, LON0)
        t = round(t + 0.1, 6)
    return out


h = g.StationaryHold()
out = run(h, 0.0, 4.0, lambda t: 0.51, lambda t: 1.73)
chk("AUTO commanding, lidar unchanged (2026-10-21): held", out[2])
chk("and says why", "lidar sees no movement" in (h.take_event() or ""))
out = run(h, 4.0, 1.0, lambda t: 0.51 - 0.3 * (t - 4.0),
          lambda t: 1.73 + 0.3 * (t - 4.0))
chk("starts moving (0.3 m/s): released within the second", not out[2])
chk("and says so", "released" in (h.take_event() or ""))
out = run(h, 5.0, 4.0, lambda t: 2.0 - 0.1 * (t - 5.0), lambda t: None)
chk("moving slowly (0.1 m/s), only the front in range: not held",
    not out[2])
out = run(h, 9.0, 4.0, lambda t: None, lambda t: None)
chk("open field, nothing ahead or behind: drive commands decide (moving)",
    not out[2])
h2 = g.StationaryHold()
out = run(h2, 0.0, 4.0, lambda t: None, lambda t: 1.73)
chk("only the rear in range and unchanged: held", out[2])
out = run(h2, 4.0, 3.0, lambda t: 6.0 if t < 5.5 else None,
          lambda t: 1.73)
chk("something comes into range ahead: not 'still' (scene changed)",
    not g.LidarStill.still(h2.lidar, 5.6))
h3 = g.StationaryHold()
h3.commanded(0.0, 0.5)
for k in range(30):                               # scans stop coming
    h3.scan(k * 0.1, *scan(0.5, 1.7))
out = h3.apply(9.0, LAT0, LON0)
chk("lidar silent (stale scans) while driving: no lidar hold",
    h3.lidar.still(9.0) is False)

print("\n-- NMEA checksum")
good = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
chk("a real sentence passes", g.nmea_ok(good))
chk("one digit changed fails", not g.nmea_ok(good.replace("4807.038",
                                                           "4807.938")))
chk("a sentence cut short fails", not g.nmea_ok(good[:40]))
chk("no checksum at all fails", not g.nmea_ok(good[:good.rfind("*")]))
chk("two sentences run together fails",
    not g.nmea_ok(good[:30] + good))

print("\n-- the receiver's own error estimate (GST)")
import time as _time                          # noqa: E402
from types import SimpleNamespace             # noqa: E402


def fix(err, accepted=False, gst_age=0.5):
    now = _time.time()
    return SimpleNamespace(lat=LAT0, lon=LON0, fix=1, sats=11, hdop=1.1,
                           last_gga=now, quality_ok=accepted,
                           err_m=err, last_gst=now - gst_age)


q = g.GPSNode._quality_check
A, K = g.MAX_ERR_ACCEPT_M, g.MAX_ERR_KEEP_M
bad = K + 4.0
chk(f"HDOP 1.1 but the receiver says {bad:.1f} m: rejected",
    q(fix(bad))[0] is False and f"{bad:.1f} m" in q(fix(bad))[1],
    q(fix(bad))[1])
chk("it says 2 m (open field): accepted", q(fix(2.0))[0])
mid = (A + K) / 2.0
chk(f"accepted, then {mid:.1f} m: kept (hysteresis, keep <= {K:.0f})",
    q(fix(mid, accepted=True))[0])
chk(f"not yet accepted, {mid:.1f} m: not accepted (accept <= {A:.0f})",
    not q(fix(mid))[0])
chk(f"accepted, then {K + 0.5:.1f} m: dropped",
    not q(fix(K + 0.5, accepted=True))[0])
chk("(the keep limit is above the accept limit)", K > A)
chk("no GST for 10 s: judged on sats and HDOP alone (as before)",
    q(fix(30.0, gst_age=10.0))[0])

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
