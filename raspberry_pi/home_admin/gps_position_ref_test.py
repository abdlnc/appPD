#!/usr/bin/env python3
"""
Unit tests for gps_node.PositionRef -- the robot position you set on the
map, carried along by lidar SLAM, or by corrected GPS where the lidar
cannot see enough.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/gps_position_ref_test.py

No robot: taps, GPS fixes and SLAM poses are fed by hand. SLAM poses are
made the way slam_toolbox reports them on THIS robot -- base_link is the
lidar's frame, and the lidar is mounted backwards, so SLAM's yaw points
out of the robot's REAR. Getting that wrong would put the robot behind
itself, so the driving cases check it.
"""
import math
import os
import sys

sys.path.insert(0, os.path.expanduser("~"))
import gps_node as g                          # noqa: E402

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


LAT0, LON0 = 14.6320, 121.0900


def ll(e, n):
    """(lat, lon) e metres east, n metres north of (LAT0, LON0)."""
    return g._ll(LAT0, LON0, e, n)


def en(lat, lon):
    return g._enu(LAT0, LON0, lat, lon)


class SlamRobot:
    """A robot as slam_toolbox sees it: pose (x, y, yaw) in SLAM's own
    frame, which starts at an arbitrary angle, with yaw = the REAR."""

    def __init__(self, yaw0=0.7, x=3.0, y=-2.0):
        self.x, self.y = x, y
        self.front = yaw0 + math.pi           # front angle in SLAM's frame

    def pose(self):
        return (self.x, self.y, self.front - math.pi)

    def drive(self, metres):
        self.x += metres * math.cos(self.front)
        self.y += metres * math.sin(self.front)

    def turn(self, deg_right):
        self.front -= math.radians(deg_right)  # right = clockwise = -angle


def drive(r, bot, metres, t, gps, slam_ok=True, step=0.15):
    """Drive `metres` in 0.15 m steps (0.3 m/s at the 2 Hz publish rate),
    calling fuse() each step as gps_node does. gps = (east, north) the GPS
    reports throughout. Returns (last fuse() result, time)."""
    out = None
    done = 0.0
    while done < metres - 1e-9:
        d = min(step, metres - done)
        bot.drive(d)
        done += d
        t += 0.5
        out = r.fuse(t, ll(*gps), ll(*gps), bot.pose(), slam_ok)
    return out, t


def near(p, e, n, tol=0.05):
    pe, pn = en(p[0], p[1])
    return math.hypot(pe - e, pn - n) <= tol, f"at E {pe:.2f} N {pn:.2f}"


print("-- nothing set")
r = g.PositionRef()
chk("no manual position: fuse() says use GPS as usual",
    r.fuse(0.0, ll(5, 5), ll(5, 5), (0, 0, 0), True) is None)

print("\n-- setting it: two taps")
bot = SlamRobot()
r = g.PositionRef()
raw_mean = ll(20.0, -10.0)                   # GPS thinks it is over there
r.tap(0.0, *ll(0, 0), raw_mean, bot.pose())
ev = r.take_event() or ""
chk("first tap: position set, and how far off GPS was", "SET" in ev and
    "22 m off" in ev, ev)
out = r.fuse(1.0, ll(20, -10), raw_mean, bot.pose(), True)
chk("between the taps: the tapped spot", near(out, 0, 0)[0])
r.tap(2.0, *ll(0.3, 0.4), raw_mean, bot.pose())
chk("a facing tap 0.5 m away is refused (too close to tell)",
    r.stage == "FACE" and "at least" in (r.take_event() or ""))
r.tap(3.0, *ll(0, 8), raw_mean, bot.pose())  # it faces north
ev = r.take_event() or ""
chk("second tap: facing north, tracking by lidar", r.stage == "ON" and
    "faces 0 deg" in ev and "LIDAR" in ev, ev)

print("\n-- driving, tracked by lidar")
out, t = drive(r, bot, 10.0, 3.0, (25, 0))
good, where = near(out, 0, 10)
chk("10 m forward: 10 m NORTH of the tapped spot (not south)", good, where)
chk("heading reported: north", abs((out[2] + 180) % 360 - 180) < 0.5,
    f"{out[2]:.1f}")
chk("source is SLAM", out[3] == "SLAM")
bot.turn(90)
out, t = drive(r, bot, 5.0, t, (30, 0))
good, where = near(out, 5, 10)
chk("turn right, 5 m: now 5 m east as well", good, where)
chk("heading reported: east", abs(out[2] - 90) < 0.5, f"{out[2]:.1f}")
oe, on = r.off
chk("the GPS correction is kept fresh meanwhile (GPS 25 m east)",
    abs(oe - (5 - 30)) < 0.05 and abs(on - 10) < 0.05, f"{r.off}")

print("\n-- the lidar cannot see enough: corrected GPS")
out, t = drive(r, bot, 4.0, t, (34, 0), slam_ok=False)
good, where = near(out, 9, 10)
chk("GPS + the last correction: still in the right place", good, where)
chk("source is GPS, heading left to the receiver", out[3] == "GPS" and
    out[2] is None)
chk("and says so", "CORRECTED GPS" in (r.take_event() or ""))
out = r.fuse(t + 0.5, None, None, bot.pose(), False)
chk("no GPS fix either: no position (not a made-up one)", out is None)
out, t = drive(r, bot, 2.0, t + 0.5, (36, 0), slam_ok=False)
out = r.fuse(t + 0.5, ll(36, 0), ll(36, 0), bot.pose(), True)
good, where = near(out, 11, 10)
chk("lidar sees again: carries on from the corrected-GPS spot", good, where)
chk("re-tied, and says so", "can see again" in (r.take_event() or ""))
out, t = drive(r, bot, 3.0, t + 0.5, (39, 0))
good, where = near(out, 14, 10)
chk("and the lidar tracks the next 3 m east from there", good, where)

print("\n-- SLAM restarts or snaps: re-tied, no jump")
bot.x += 40.0                                # slam_toolbox's frame jumped
out = r.fuse(t + 0.5, ll(39, 0), ll(39, 0), bot.pose(), True)
good, where = near(out, 14, 10)
chk("a 40 m SLAM jump does not move the robot", good, where)
chk("and says so", "jumped" in (r.take_event() or ""))
out, t = drive(r, bot, 2.0, t + 0.5, (41, 0))
good, where = near(out, 16, 10)
chk("then tracks on from there", good, where)

print("\n-- corrected GPS does not last")
r2 = g.PositionRef()
bot2 = SlamRobot()
r2.tap(0.0, *ll(0, 0), ll(10, 0), bot2.pose())
r2.tap(1.0, *ll(0, 5), ll(10, 0), bot2.pose())
r2.take_event()
out = r2.fuse(2.0, ll(10, 0), ll(10, 0), bot2.pose(), False)
chk("no lidar from the start: corrected GPS", out is not None and
    out[3] == "GPS" and near(out, 0, 0)[0])
out = r2.fuse(2.0 + g.OFFSET_TRUST_S + 1, ll(10, 0), ll(10, 0),
              bot2.pose(), False)
chk(f"after {g.OFFSET_TRUST_S:.0f} s without lidar: dropped, plain GPS",
    out is None and r2.stage == "OFF")
chk("and says so", "expired" in (r2.take_event() or ""))

print("\n-- SLAM not running when you tapped")
r3 = g.PositionRef()
r3.tap(0.0, *ll(0, 0), ll(10, 0), None)
r3.tap(1.0, *ll(0, 5), ll(10, 0), None)
chk("says it will use corrected GPS only",
    "SLAM is not running" in (r3.take_event() or ""))
out = r3.fuse(2.0, ll(10, 0), ll(10, 0), (1.0, 2.0, 0.3), True)
chk("SLAM appearing later is NOT used (never lined up with the world)",
    out[3] == "GPS")

print("\n-- robot moved between the taps")
r4 = g.PositionRef()
b4 = SlamRobot()
r4.tap(0.0, *ll(0, 0), ll(10, 0), b4.pose())
b4.drive(2.0)
r4.tap(1.0, *ll(0, 5), ll(10, 0), b4.pose())
chk("not lined up with SLAM (the tapped spot is no longer where it is)",
    r4.anchor is None)

print("\n-- clear")
r.clear()
chk("CLEAR: back to plain GPS", r.fuse(t + 1, ll(1, 1), ll(1, 1),
                                       bot.pose(), True) is None)
chk("and says so", "plain GPS" in (r.take_event() or ""))

print("\n-- a new first tap starts over")
r5 = g.PositionRef()
b5 = SlamRobot()
r5.tap(0.0, *ll(0, 0), None, b5.pose())
r5.tap(1.0, *ll(5, 0), None, b5.pose())       # faces east
r5.take_event()
r5.tap(2.0, *ll(50, 50), None, b5.pose())
chk("a third tap is a new position, waiting for its facing tap",
    r5.stage == "FACE" and r5.pos == ll(50, 50))

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
