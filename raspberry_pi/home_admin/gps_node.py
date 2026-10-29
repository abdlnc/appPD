#!/usr/bin/env python3
"""
AGV GPS Node  (ROS2 / rclpy)  --  REAL or SIMULATED

Publishes:
    /gps      (std_msgs/String) : "lat,lon,heading,speed_kmh,fix,sats"
    /gps_accuracy (std_msgs/String) : the receiver's own error estimate,
                                  metres ("" if it gives none) -- see
                                  ERROR ESTIMATE below
    /position_ref (std_msgs/String, 1 Hz JSON) : the position you set and
                                  what is tracking it, for the app ("R:")
    /gps_raw  (std_msgs/String) : the same, WITHOUT the parked hold below --
                                  what the receiver says. The boundary fence
                                  reads this one (see below), and gps_drift.py.
Subscribes:
    /cmd_vel, /scan             : is the robot moving? (parked hold)
    /set_position               : "TAP lat,lon" | "CLEAR" -- see YOU SET
                                  WHERE IT IS (from bridge_node)
    TF map -> base_link         : slam_toolbox's lidar position

  * With a FIX  -> "lat,lon,heading,speed,fix,sats"   (sats = used in solution)
    ...but only a fix that passes the QUALITY FILTER (enough satellites, low
    enough HDOP -- see MIN_SATS_ACCEPT). A weak fix is published exactly like
    no fix, because a position that jumps metres is worse than none.
  * ACQUIRING   -> ",,,0.0,0,<sats_in_view>"          (no fix yet; live feedback)
    The empty lat/lon tells the app "still acquiring", while <sats_in_view>
    (parsed from GSV) lets the app show a rising count instead of a blank card.

PARKED = HELD STILL. A parked GPS wanders -- measured on this robot: a
median 3.1 m in 20 s and up to 9.9 m, without moving an inch -- and every
consumer saw that as motion: the map marker crawling, the fence flipping
IN/OUT at the stop line, photos tagged metres apart. So when nothing has
told the wheels to move (/cmd_vel) for HOLD_STILL_S -- OR the lidar shows
it is not moving although told to (stuck, boxed in, on a stand, motor power
off: seen 2026-10-21, AUTO commanding forward and reverse, lidar distances
unchanged to the centimetre, GPS wandering) -- /gps holds the position
steady until it moves again (StationaryHold, LidarStill). The receiver's
own "static hold" is NOT the way: it decides "parked" from its speed
estimate, and froze this slow robot mid-drive (see receiver dynamics,
below). No fix is still no fix: the hold never invents one.

THE FENCE DOES NOT USE THE HOLD. fence_node reads /gps_raw. The lidar can
in principle be fooled -- something pacing the robot dead ahead, nothing
behind in range -- and a held position must never be what stands between
the robot and the edge of the field. The fence keeps the receiver's own
positions, exactly as it did before this hold existed.

PICKED UP AND CARRIED, it still shows where it was parked (unless lidar
tracking is on -- YOU SET WHERE IT IS -- which sees it move), until the next
drive command (any -- a joystick nudge will do). There is no automatic
"it was moved" release because nothing available can tell: parked on
2026-10-21 the receiver's own speed read 2 km/h or more 30% of the time
(up to 19 km/h -- more than it reads while driving), and its position
jumped up to 113 m away for minutes at a time. A release on either would
have fired on the receiver's noise, not on a carry -- the drift again.

YOU SET WHERE IT IS. Among houses this receiver can be tens of metres off
(113 m, parked, 2026-10-21). So you can tell the robot where it is:
touch ~/agv_set_position, then on the app's map tap where the robot REALLY
is, then tap a point it is FACING (a few metres in front of it). From then
on /gps is that spot plus how far the robot has moved since:
  * by LIDAR (slam_toolbox), while the lidar can see enough around it --
    walls, houses, fences. Accurate to a few tens of cm, and the heading
    is real, not guessed from GPS positions.
  * by GPS CORRECTED by the error it had when you tapped, when the lidar
    cannot see enough (an open field). The correction is refreshed all
    the time SLAM is working, so the hand-over does not jump; but GPS
    error changes, so after OFFSET_TRUST_S without SLAM it is no better
    than plain GPS, and the manual position is dropped (logged): tap again.
CLEAR (the app's "clear waypoint" while the flag is there) drops it at
once. The fence never uses any of this -- it reads /gps_raw. Not in --sim
(the lidar on the bench sees a still room).

TWO MODES:
  (default) REAL  : reads u-blox NEO-M8N NMEA on /dev/ttyAMA0 @ 9600.
  --sim           : SIMULATION (always has a fix; FIX 1 SAT 12 signature).

Run:
    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/gps_node.py            # real GPS (outdoors / open sky)
    python3 ~/gps_node.py --sim      # simulation (indoor dev/testing)

If real mode says "No module named 'serial'":  pip install pyserial
"""

import sys
import json
import math
import random
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros

PORT = "/dev/ttyAMA0"
BAUD = 9600

# ---- fix QUALITY filter ----
# A receiver will report a "fix" from as few as 4 satellites, but with that
# little geometry a tiny timing error swings the answer by metres, and a
# satellite entering or leaving view makes the position JUMP -- seen on this
# robot as a marker hopping around while parked (4 used, 5 in view).
# A wrong position is worse than none (the same rule the photo tagging
# follows), so a fix below this quality is published as NO fix.
#
# HDOP = the receiver's own estimate of how bad its satellite geometry is
# (1 = ideal, <2 good, >5 poor). It is in the GGA sentence alongside the
# satellite count, which is all that was read before.
#
# Two thresholds (hysteresis), like the no-path hold in control_node: a fix
# has to be clearly GOOD to be accepted, but once accepted it is only dropped
# when clearly BAD. A single threshold would flicker fix/no-fix every second
# whenever reception sits right on it, and every flicker stops NAV mode.
# ---- receiver dynamics ----
# The module smooths position when it believes it is stationary, which is how
# a 14m walk came back as 1m of wander: at 0.2-0.4 km/h the robot looks parked
# to it. Two settings change that, sent as a UBX-CFG-NAV5 message at startup:
#
#   dynModel 3 (pedestrian)  -- expect a slow-moving platform rather than the
#                               general-purpose default
#   staticHoldThresh 0       -- never freeze position at low speed
#
# Applied to RAM ONLY, deliberately: nothing is written to the module's flash,
# so a power cycle restores whatever it had before and there is nothing to
# undo if this turns out wrong.
#
# A receiver that ignores it (a clone with its own firmware) costs one logged
# warning; the node carries on either way.
UBX_DYN_MODEL        = 3   # 0=portable 2=stationary 3=pedestrian 4=automotive
UBX_STATIC_HOLD_CMS  = 0   # cm/s; 0 disables static hold entirely

MIN_SATS_ACCEPT = 6        # to START trusting the fix
MAX_HDOP_ACCEPT = 2.5
MIN_SATS_KEEP   = 5        # to KEEP trusting a fix already accepted

# ---- ERROR ESTIMATE: the receiver's own, in metres ----
# HDOP describes only where the satellites are in the sky. It said 0.9-1.2
# ("excellent") on 2026-10-21 while the position sat 113 m away for minutes:
# signals bouncing off houses (multipath) do not show in it at all. The
# receiver's error estimate does respond to them -- it is worked out from
# how badly the satellite ranges disagree. It comes in the NMEA GST sentence
# (std. deviation of latitude and longitude error, metres), which the NEO-M8N
# does not send unless asked: configure_receiver turns it on (RAM only, like
# the rest). Error = the two combined.
#
# A fix the receiver itself puts worse than MAX_ERR_ACCEPT_M is not
# accepted; once accepted it is dropped above MAX_ERR_KEEP_M (hysteresis, as
# the satellite and HDOP limits). A receiver that never sends GST is judged
# on satellites and HDOP alone, as before (logged once).
# Measured parked by the house, 2026-10-22, 5 min: the estimate never went
# below 7.7 m (median 9.2), and the real spread agreed (median 4 m, up to
# 13.5 m from the mean). It does NOT pick out the worse moments within that
# -- fixes rated 5-8 m were no closer than ones rated 8-12 -- it tells you
# the PLACE is poor. So by choice this limit is strict: among the houses
# there is effectively no GPS; in the open field, where the M8N usually
# rates itself 1-3 m, there is. (A set position -- YOU SET WHERE IT IS --
# keeps working by lidar either way.)
#
# TEMPORARY (2026-10-29, on request): 10 / 13 m instead of 5 / 8 m. Under
# open sky the receiver used only 8 of 25 satellites in view and settled at
# a 7.5 m estimate -- a weak signal at the antenna (placement / electrical
# noise), not the sky. Go back to 5 / 8 once the antenna is sorted.
ERR_GATE = True
MAX_ERR_ACCEPT_M = 10.0
MAX_ERR_KEEP_M = 13.0
GST_STALE_S = 3.0

# ---- staleness: no fresh position = no fix ----
# The reader thread STORES the last position it parsed, and publish_gps sends
# whatever is stored every 0.5s. So when the serial stream dies -- which this
# receiver's wiring has done, sending zero bytes for minutes at a time -- the
# last fix used to go out as a perfectly valid position, forever, with its
# satellite count frozen too. Harmless-looking on a map; dangerous for the
# boundary fence, which would see the robot parked "inside" while it drove
# out of the field. A position older than this is now published as no fix.
# GGA arrives once a second, so 3s is three missed sentences in a row.
GGA_STALE_S = 3.0
MAX_HDOP_KEEP   = 3.5

# ---- parked hold (see PARKED = HELD STILL, top) ----
HOLD_STILL_S = 2.0          # no drive command this long = parked (it has
                            # rolled to a stop)
MOVING_CMD = 0.02           # |linear.x| above this is a drive command.
                            # Steering alone does not move a front-steer robot.
# Lidar: "not moving" = the distances dead ahead and dead behind (median of
# each +/-LIDAR_SECTOR_DEG sector) have stayed within LIDAR_STILL_M for the
# whole of HOLD_STILL_S. Moving along its axis -- the only way this robot
# moves -- changes one or both at once: 0.1 m/s is 0.2 m in 2 s, five times
# the threshold. Measured parked on 2026-10-21 they held to +/-0.01 m.
# Nothing in range either way (an open field): no lidar evidence, and the
# drive commands alone decide.
LIDAR_SECTOR_DEG = 15.0
LIDAR_STILL_M = 0.04
LIDAR_SEE_M = 8.0           # beyond this a return is too sparse to judge by
LIDAR_MIN_SCANS = 5         # in the HOLD_STILL_S window (10 Hz scanner)
LIDAR_FRESH_S = 0.5         # no scan this recent = lidar not running
LIDAR_ANGLE_OFFSET_DEG = 180.0   # as control_node: mounted rotated 180 deg
MIN_VALID_DIST = 0.15            # as control_node: own chassis / noise

# ---- you set where it is (see YOU SET WHERE IT IS, top) ----
# SLAM is trusted while at least SLAM_MIN_SEEN of the lidar's rays return
# something within SLAM_SEE_M, averaged over SLAM_SEEN_S. Measured among the
# houses on 2026-10-21: 0.76 (0.67 at worst). TODO: measure in the field --
# 0.30 is a guess at where scan matching stops having enough to hold on to.
SLAM_SEE_M = 10.0
SLAM_MIN_SEEN = 0.30
SLAM_SEEN_S = 2.0
SLAM_FRESH_S = 1.0          # no SLAM position this recent = SLAM not running
# The SLAM position moving further than this between two readings (2 Hz) is
# not driving -- slam_toolbox restarted (its map starts again at 0,0) or its
# scan matching snapped somewhere else. Re-anchored to the last good position.
SLAM_JUMP_M = 1.0
RAW_MEAN_S = 5.0            # GPS error at a tap: the mean of this much raw GPS
# Corrected GPS without SLAM: how long the correction is trusted. The error
# is correlated over ~30-120 s; past roughly half that, subtracting an old
# error adds as much as it removes.
OFFSET_TRUST_S = 60.0
FACE_MIN_M = 1.0            # the "facing" tap must be at least this far off
FACE_TIMEOUT_S = 120.0      # position tapped but no facing tap: dropped
SLAM_YAW_OFFSET_DEG = LIDAR_ANGLE_OFFSET_DEG   # base_link = laser frame, and
                            # the lidar is mounted backwards: SLAM's yaw is
                            # the robot's REAR. Its front is this much round.

# Simulation start point (Quezon City) + virtual robot limits
SIM_START_LAT = 14.7481338
SIM_START_LON = 121.0616677
SIM_MAX_SPEED_MPS = 0.6     # ground speed at linear.x = 1.0
SIM_MAX_TURN_DPS = 35.0     # turn rate at angular.z = 1.0
SIM_DT = 0.2
M_PER_DEG_LAT = 111320.0

# ---- optional realism for bench-testing COVER and the fence ----
# Both OFF by default, so the existing NAV demo behaves exactly as before.
#
# --sim-ackermann: the default sim turns on the spot (heading changes even
# at zero speed). The real robot steers its front wheels and CANNOT pivot:
# its turn rate is proportional to its speed. Coverage turns and the fence
# recovery depend on exactly that, so a sim that pivots would pass tests
# the robot fails. Bicycle model: yaw rate = v / R * steer, R = half the
# turning diameter at full lock.
SIM_TURN_DIAMETER_M = 3.0   # TODO: measure on the robot (full lock circle)
#
# --sim-noise: the default sim is a perfect GPS. The NEO-M8N is good to
# about +/-2.5m, and its error is CORRELATED -- it wanders over tens of
# seconds rather than jittering independently each fix, which is why it
# fools a controller. Modelled as a first-order Gauss-Markov drift per axis
# (SIM_NOISE_SIGMA_M standard deviation, so +/-2.5m is about 2 sigma).
# Also, like the real receiver, no course is reported while barely moving.
SIM_NOISE_SIGMA_M = 1.25
SIM_NOISE_TAU_S = 30.0
SIM_COG_MIN_MPS = 0.2


def nmea_ok(line):
    """True if an NMEA sentence's *XX checksum matches. A line garbled on
    the serial link otherwise parses as a perfectly good, wrong position."""
    star = line.rfind('*')
    if not line.startswith('$') or star < 1 or len(line) < star + 3:
        return False
    c = 0
    for ch in line[1:star]:
        c ^= ord(ch)
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def nmea_to_deg(val, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    if not val or '.' not in val:
        return None
    dot = val.index('.')
    deg_len = dot - 2
    if deg_len < 1:
        return None
    try:
        deg = float(val[:deg_len])
        minutes = float(val[deg_len:])
    except ValueError:
        return None
    dec = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        dec = -dec
    return dec


def _enu(lat0, lon0, lat, lon):
    """(east, north) metres of (lat, lon) from (lat0, lon0)."""
    return ((lon - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0)),
            (lat - lat0) * M_PER_DEG_LAT)


def _ll(lat0, lon0, e, n):
    return (lat0 + n / M_PER_DEG_LAT,
            lon0 + e / (M_PER_DEG_LAT * math.cos(math.radians(lat0))))


class PositionRef:
    """Where the robot is, from a position you tapped plus how far it has
    moved since (see YOU SET WHERE IT IS, top). Pure Python -- tested by
    ~/gps_position_ref_test.py.

    tap(t, lat, lon, raw_mean, slam): your taps -- position, then facing.
    fuse(t, raw, raw_mean, slam, slam_ok) on every publish -> None (no
    manual position: publish GPS as usual) or (lat, lon, compass|None,
    source). slam = (x, y, yaw_rad) in slam_toolbox's map frame, or None.
    take_event() -> what to log."""

    def __init__(self):
        self.stage = "OFF"           # OFF | FACE (waiting for tap 2) | ON
        self.pos = None              # tapped position (lat, lon)
        self.face_t = None           # when the position was tapped
        self.pos_slam = None         # SLAM pose when it was tapped
        self.anchor = None           # (x0, y0, rot, lat0, lon0): SLAM -> world
        self.off = None              # (east, north) GPS correction, metres
        self.off_t = None            # when the correction was last measured
        self.last = None             # last fused (lat, lon, compass|None)
        self.last_slam = None        # last SLAM pose seen, for jump checks
        self.using = None            # "SLAM" | "GPS" -- for change logs
        self.event = None

    def take_event(self):
        ev, self.event = self.event, None
        return ev

    def _say(self, msg):
        self.event = msg if self.event is None else self.event + "; " + msg

    def clear(self, why="cleared"):
        ev = self.event
        if self.stage != "OFF":
            ev = (ev + "; " if ev else "") + \
                f"manual position {why} -- plain GPS again"
        self.__init__()
        self.event = ev             # survives the reset, to be logged

    def _anchor_at(self, lat, lon, compass, slam):
        """Tie SLAM's frame to the world: the robot, at SLAM pose `slam`,
        is at (lat, lon) facing `compass` (degrees, clockwise from north)."""
        x0, y0, yaw = slam
        front = yaw + math.radians(SLAM_YAW_OFFSET_DEG)
        # ENU angle of the robot's front, minus its angle in SLAM's frame
        rot = math.radians(90.0 - compass) - front
        self.anchor = (x0, y0, rot, lat, lon)

    def _from_slam(self, slam):
        x0, y0, rot, lat0, lon0 = self.anchor
        x, y, yaw = slam
        dx, dy = x - x0, y - y0
        e = dx * math.cos(rot) - dy * math.sin(rot)
        n = dx * math.sin(rot) + dy * math.cos(rot)
        lat, lon = _ll(lat0, lon0, e, n)
        ang = yaw + math.radians(SLAM_YAW_OFFSET_DEG) + rot   # ENU, CCW
        compass = (90.0 - math.degrees(ang)) % 360.0
        return lat, lon, compass

    def tap(self, t, lat, lon, raw_mean, slam):
        if self.stage == "FACE":
            plat, plon = self.pos
            e, n = _enu(plat, plon, lat, lon)
            if math.hypot(e, n) < FACE_MIN_M:
                self._say(f"facing tap only {math.hypot(e, n):.1f} m from "
                          f"the robot -- tap a point at least {FACE_MIN_M:.0f}"
                          f" m in front of it")
                return
            compass = math.degrees(math.atan2(e, n)) % 360.0
            self.stage = "ON"
            if self.pos_slam is not None and slam is not None and \
                    math.dist(self.pos_slam[:2], slam[:2]) < 0.5:
                self._anchor_at(plat, plon, compass, self.pos_slam)
                self.last = (plat, plon, compass)
                self._say(f"robot faces {compass:.0f} deg -- tracking it by "
                          f"LIDAR from here (corrected GPS where the lidar "
                          f"cannot see enough)")
            else:
                self.last = (plat, plon, None)
                self._say("robot faces {:.0f} deg, but SLAM is not running "
                          "(or the robot moved between taps) -- tracking by "
                          "CORRECTED GPS only, good for about {:.0f} s "
                          "unless SLAM picks up".format(compass,
                                                        OFFSET_TRUST_S))
            return
        # first tap (or a new position): where it is
        self.__init__()
        self.stage = "FACE"
        self.pos = (lat, lon)
        self.face_t = t
        self.pos_slam = slam
        self.last = (lat, lon, None)
        if raw_mean is not None:
            e, n = _enu(raw_mean[0], raw_mean[1], lat, lon)
            self.off, self.off_t = (e, n), t
            gps = f"GPS was {math.hypot(e, n):.0f} m off"
        else:
            gps = "no GPS fix to correct"
        self._say(f"robot position SET ({gps}) -- now tap a point it is "
                  f"facing, a few metres in front of it")

    def fuse(self, t, raw, raw_mean, slam, slam_ok):
        if self.stage == "OFF":
            return None
        if self.stage == "FACE":
            if t - self.face_t > FACE_TIMEOUT_S:
                # left half-done (app closed, walked away): not a position
                # anyone is still vouching for
                self.clear("not finished -- no facing tap within "
                           f"{FACE_TIMEOUT_S:.0f} s")
                return None
            # held in STOP between the taps: the tapped spot, as tapped
            return self.pos[0], self.pos[1], None, "SET"

        jumped = False
        if slam is not None and self.last_slam is not None and \
                math.dist(slam[:2], self.last_slam[:2]) > SLAM_JUMP_M:
            jumped = True
        if slam is not None:
            self.last_slam = slam

        # SLAM is only ever lined up with the world at your taps. Without
        # that (SLAM not running then) it is corrected GPS until expiry.
        if slam_ok and slam is not None and self.anchor is not None:
            if jumped:
                # SLAM's pose AND yaw jumped: re-tie both to the last good
                # position and heading
                self._say("SLAM jumped -- lidar tracking re-tied to the "
                          "last good position")
                self._anchor_at(self.last[0], self.last[1],
                                self.last[2], slam)
            elif self.using == "GPS":
                # back from corrected GPS: SLAM may have lost track of
                # distance while it could not see, but holds rotation far
                # better -- keep its heading, re-tie the position
                self._say("lidar can see again -- lidar tracking re-tied to "
                          "the current (corrected GPS) position")
                self._anchor_at(self.last[0], self.last[1],
                                self._from_slam(slam)[2], slam)
            lat, lon, compass = self._from_slam(slam)
            if raw_mean is not None:       # keep the fallback fresh
                self.off = _enu(raw_mean[0], raw_mean[1], lat, lon)
                self.off_t = t
            self.using = "SLAM"
            self.last = (lat, lon, compass)
            return lat, lon, compass, "SLAM"

        # no usable SLAM: corrected GPS
        if self.off is None or t - self.off_t > OFFSET_TRUST_S:
            self.clear(f"expired ({OFFSET_TRUST_S:.0f} s without lidar "
                       f"tracking)" if self.off is not None else
                       "lost (no lidar tracking and no GPS to correct)")
            return None
        if raw is None:
            return None                     # no fix: publish no fix
        if self.using != "GPS":
            self._say("lidar cannot see enough -- CORRECTED GPS for now "
                      f"(good for about {OFFSET_TRUST_S:.0f} s)")
            self.using = "GPS"
        lat, lon = _ll(raw[0], raw[1], *self.off)
        self.last = (lat, lon, self.last[2] if self.last else None)
        return lat, lon, None, "GPS"


class LidarStill:
    """Does the lidar say the robot is standing still? Pure Python.
    feed() every scan; still(t) -> True / False (False also when it cannot
    tell -- nothing in range, or no recent scans)."""

    def __init__(self):
        self.hist = []               # (t, front_median|None, rear_median|None)

    @staticmethod
    def _sectors(ranges, angle_min, inc):
        front, rear = [], []
        for i, r in enumerate(ranges):
            if not (MIN_VALID_DIST <= r <= LIDAR_SEE_M):  # also drops inf/nan
                continue
            deg = (math.degrees(angle_min + i * inc)
                   + LIDAR_ANGLE_OFFSET_DEG) % 360.0
            if deg <= LIDAR_SECTOR_DEG or deg >= 360.0 - LIDAR_SECTOR_DEG:
                front.append(r)
            elif abs(deg - 180.0) <= LIDAR_SECTOR_DEG:
                rear.append(r)

        def med(v):
            if len(v) < 3:
                return None
            v = sorted(v)
            return v[len(v) // 2]
        return med(front), med(rear)

    def feed(self, t, ranges, angle_min, inc):
        f, r = self._sectors(ranges, angle_min, inc)
        self.hist.append((t, f, r))
        while self.hist and t - self.hist[0][0] > HOLD_STILL_S + 1.0:
            self.hist.pop(0)

    def still(self, t):
        if not self.hist or t - self.hist[-1][0] > LIDAR_FRESH_S:
            return False
        win = [h for h in self.hist if t - h[0] <= HOLD_STILL_S]
        if len(win) < LIDAR_MIN_SCANS or \
                win[0][0] > t - HOLD_STILL_S + 0.5:
            return False                       # not a full window yet
        seen = 0
        for k in (1, 2):
            vals = [h[k] for h in win]
            if all(v is None for v in vals):
                continue                       # nothing that way: no evidence
            if any(v is None for v in vals):
                return False                   # came into / out of range
            if max(vals) - min(vals) > LIDAR_STILL_M:
                return False                   # it moved
            seen += 1
        return seen > 0


class StationaryHold:
    """Holds the published position still while the robot is parked. Pure
    Python (no ROS), tested by ~/gps_hold_test.py. See PARKED = HELD STILL.

    commanded(t, linear_x) on every /cmd_vel, scan(...) on every /scan;
    apply(t, lat, lon) on every fix -> (lat, lon, held). The caller handles no-fix itself: a
    lost fix publishes as lost, and the held point survives it (losing the
    fix does not move the robot)."""

    def __init__(self):
        self.last_move = -math.inf   # never commanded yet = parked since boot
        self.anchor = None           # (lat, lon) being held
        self.event = None            # what changed, to log -- take_event()
        self.lidar = LidarStill()

    def commanded(self, t, linear_x):
        if abs(linear_x) > MOVING_CMD:
            self.last_move = t

    def scan(self, t, ranges, angle_min, inc):
        self.lidar.feed(t, ranges, angle_min, inc)

    def parked(self, t):
        """None if moving, else why it counts as parked."""
        if t - self.last_move >= HOLD_STILL_S:
            return "no drive command"
        if self.lidar.still(t):
            return ("told to drive, but the lidar sees no movement -- stuck, "
                    "boxed in, wheels off the ground or motor power off?")
        return None

    def take_event(self):
        """What changed since last asked (or None), for the log."""
        ev, self.event = self.event, None
        return ev

    def apply(self, t, lat, lon):
        why = self.parked(t)
        if why is None:
            if self.anchor is not None:
                self.anchor = None
                self.event = "moving -- GPS hold released"
            return lat, lon, False
        if self.anchor is None:
            self.anchor = (lat, lon)
            self.event = f"parked ({why}) -- GPS position held still"
        return self.anchor[0], self.anchor[1], True


# ============================ REAL GPS ============================
class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        import serial
        self.pub = self.create_publisher(String, '/gps', 10)
        self.raw_pub = self.create_publisher(String, '/gps_raw', 10)
        self.hold = StationaryHold()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.ref = PositionRef()       # see YOU SET WHERE IT IS (top)
        self.create_subscription(String, '/set_position', self.setpos_cb, 10)
        self.posref_pub = self.create_publisher(String, '/position_ref', 10)
        self.ref_note = ""          # last thing PositionRef said, for the app
        self.create_timer(1.0, self.publish_posref)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.raw_hist = []          # (t, lat, lon) trusted raw fixes, RAW_MEAN_S
        self.seen_hist = []         # (t, fraction of rays returning)
        self.lat = None
        self.lon = None
        self.heading = ""
        self.speed_kmh = 0.0
        self.fix = 0
        self.sats = 0
        self.hdop = None            # None = not reported yet
        self.quality_ok = False     # passes the filter (see MIN_SATS_*)
        self.last_gga = 0.0         # when a GGA last arrived (see GGA_STALE_S)
        self.sats_in_view = {}      # per-talker (GP/GL/GA/GB) satellites in view
        self.bad_lines = 0          # NMEA lines dropped for a bad checksum
        self.err_m = None           # receiver's error estimate (GST), metres
        self.last_gst = 0.0
        self.acc_pub = self.create_publisher(String, '/gps_accuracy', 10)
        self._gst_warned = False
        self._started = time.time()
        self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        self.configure_receiver()
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(0.5, self.publish_gps)
        self._last_log = 0
        self.get_logger().info(f"GPS node ready (REAL) on {PORT} @ {BAUD}")

    def cmd_cb(self, msg: Twist):
        self.hold.commanded(time.time(), msg.linear.x)

    def scan_cb(self, msg: LaserScan):
        now = time.time()
        self.hold.scan(now, msg.ranges, msg.angle_min, msg.angle_increment)
        if msg.ranges:
            seen = sum(1 for r in msg.ranges
                       if MIN_VALID_DIST <= r <= SLAM_SEE_M) / len(msg.ranges)
            self.seen_hist.append((now, seen))
            while self.seen_hist and now - self.seen_hist[0][0] > SLAM_SEEN_S:
                self.seen_hist.pop(0)

    # ---- you set where it is ----
    def _slam_pose(self):
        """(x, y, yaw) from slam_toolbox, or None if it is not running."""
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_link',
                                              rclpy.time.Time())
        except Exception:
            return None
        st = tf.header.stamp
        if time.time() - (st.sec + st.nanosec * 1e-9) > SLAM_FRESH_S:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _slam_ok(self, now):
        if not self.seen_hist or now - self.seen_hist[-1][0] > LIDAR_FRESH_S:
            return False
        v = [f for _, f in self.seen_hist]
        return sum(v) / len(v) >= SLAM_MIN_SEEN

    def _raw_mean(self, now):
        v = [(a, b) for t, a, b in self.raw_hist if now - t <= RAW_MEAN_S]
        if not v:
            return None
        return (sum(a for a, _ in v) / len(v), sum(b for _, b in v) / len(v))

    def setpos_cb(self, msg: String):
        d = msg.data.strip()
        now = time.time()
        if d.upper() == "CLEAR":
            self.ref.clear()
        elif d.upper().startswith("TAP "):
            try:
                lat, lon = (float(x) for x in d[4:].split(","))
            except ValueError:
                self.get_logger().warn(f"bad /set_position: {d!r}")
                return
            raw_mean = self._raw_mean(now) if self.quality_ok else None
            self.ref.tap(now, lat, lon, raw_mean, self._slam_pose())
        self.hold.anchor = None     # publish the new position at once
        ev = self.ref.take_event()
        if ev:
            self.get_logger().warn(ev)
            self.ref_note = ev
        self.publish_posref()

    def publish_posref(self):
        """For the app: is a set position active, and what tracks it."""
        r = self.ref
        left = None
        if r.stage == "ON" and r.using == "GPS" and r.off_t is not None:
            left = max(0, int(OFFSET_TRUST_S - (time.time() - r.off_t)))
        d = {"stage": r.stage, "using": r.using, "secs_left": left,
             "lidar": r.anchor is not None, "note": self.ref_note}
        m = String()
        m.data = json.dumps(d, separators=(",", ":"))
        self.posref_pub.publish(m)

    # ---- receiver configuration (UBX) ----
    @staticmethod
    def _ubx(cls, mid, payload=b""):
        """Frame a UBX message: sync, class, id, length, payload, checksum."""
        body = bytes([cls, mid]) + len(payload).to_bytes(2, "little") + payload
        a = b = 0
        for byte in body:
            a = (a + byte) & 0xFF
            b = (b + a) & 0xFF
        return b"\xB5\x62" + body + bytes([a, b])

    def configure_receiver(self):
        """Tell the module to expect a slow-moving vehicle, and never to
        freeze its position when it thinks it has stopped.

        The setting is polled back rather than assumed: this UART also carries
        NMEA, and a silently ignored write would look exactly like the bug
        being fixed."""
        payload = bytearray(36)
        # mask: bit0 = apply dynModel, bit6 = apply the staticHold settings
        payload[0:2] = (0x0001 | 0x0040).to_bytes(2, "little")
        payload[2] = UBX_DYN_MODEL
        payload[22] = UBX_STATIC_HOLD_CMS           # staticHoldThresh, cm/s
        payload[28:30] = (0).to_bytes(2, "little")  # staticHoldMaxDist, m
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self._ubx(0x06, 0x24, bytes(payload)))
            self.ser.flush()
            time.sleep(0.4)
            self.ser.write(self._ubx(0x06, 0x24))   # poll it back
            self.ser.flush()
            got = self._read_ubx(0x06, 0x24, timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f"Receiver configuration failed: {e}")
            return
        if got is None or len(got) < 29:
            self.get_logger().warn(
                "Receiver did not answer the configuration poll -- it may "
                "ignore UBX. Position smoothing at low speed may persist.")
            return
        dyn, hold = got[2], got[22]
        if dyn == UBX_DYN_MODEL and hold == UBX_STATIC_HOLD_CMS:
            self.get_logger().info(
                f"Receiver configured: dynamic model {dyn} (slow vehicle), "
                f"static hold {hold} cm/s (off) -- RAM only, reverts on a "
                f"power cycle")
        else:
            self.get_logger().warn(
                f"Receiver kept its own settings: dynModel={dyn} "
                f"staticHold={hold} (asked for {UBX_DYN_MODEL}/"
                f"{UBX_STATIC_HOLD_CMS})")
        self.enable_gst()

    def enable_gst(self):
        """Ask for the GST sentence once a second (UBX-CFG-MSG, NMEA class
        F0 id 07, rate 1 on this port), RAM only. Whether it worked shows up
        as GST lines arriving -- publish_gps logs it if they never do."""
        try:
            self.ser.write(self._ubx(0x06, 0x01, bytes([0xF0, 0x07, 0x01])))
            self.ser.flush()
        except Exception as e:
            self.get_logger().warn(f"Could not ask for GST: {e}")

    def _read_ubx(self, cls, mid, timeout=2.0):
        """Wait for one UBX message of this class/id, stepping over the NMEA
        lines sharing the port. Returns its payload, or None."""
        end = time.time() + timeout
        buf = b""
        while time.time() < end:
            chunk = self.ser.read(256)
            if chunk:
                buf += chunk
            i = buf.find(b"\xB5\x62")
            while i >= 0 and len(buf) >= i + 6:
                ln = int.from_bytes(buf[i + 4:i + 6], "little")
                if len(buf) < i + 8 + ln:
                    break                       # wait for the rest to arrive
                if buf[i + 2] == cls and buf[i + 3] == mid:
                    return buf[i + 6:i + 6 + ln]
                buf = buf[i + 8 + ln:]
                i = buf.find(b"\xB5\x62")
            if len(buf) > 4096:
                buf = buf[-1024:]
        return None

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not raw.startswith('$'):
                continue
            if not nmea_ok(raw):
                self.bad_lines += 1
                continue
            raw = raw[:raw.rfind('*')]          # fields only, no checksum
            parts = raw.split(',')
            tag = parts[0][3:] if len(parts[0]) >= 6 else parts[0]
            try:
                if tag == 'GGA' and len(parts) >= 8:
                    self.last_gga = time.time()
                    lat = nmea_to_deg(parts[2], parts[3])
                    lon = nmea_to_deg(parts[4], parts[5])
                    if lat is not None:
                        self.lat = lat
                    if lon is not None:
                        self.lon = lon
                    self.fix = int(parts[6]) if parts[6] else 0
                    self.sats = int(parts[7]) if parts[7] else 0
                    try:
                        self.hdop = float(parts[8]) if (len(parts) > 8
                                                         and parts[8]) else None
                    except ValueError:
                        self.hdop = None
                elif tag == 'RMC' and len(parts) >= 9:
                    if parts[7]:
                        try:
                            self.speed_kmh = float(parts[7]) * 1.852
                        except ValueError:
                            pass
                    if parts[8]:
                        self.heading = parts[8]
                elif tag == 'VTG' and len(parts) >= 8:
                    if parts[1]:
                        self.heading = parts[1]
                    if parts[7]:
                        try:
                            self.speed_kmh = float(parts[7])
                        except ValueError:
                            pass
                elif tag == 'GST' and len(parts) >= 8:
                    # $..GST,time,rms,smaj,smin,orient,lat_err,lon_err,alt_err
                    if parts[6] and parts[7]:
                        self.err_m = math.hypot(float(parts[6]),
                                                float(parts[7]))
                        self.last_gst = time.time()
                elif tag == 'GSV' and len(parts) >= 4:
                    # satellites in view for this constellation (GP/GL/GA/GB)
                    talker = parts[0][1:3]
                    try:
                        self.sats_in_view[talker] = int(parts[3])
                    except ValueError:
                        pass
            except Exception:
                pass

    def _quality_check(self):
        """Apply the fix-quality filter, with hysteresis. Returns (ok, why)."""
        if self.lat is None or self.lon is None or self.fix < 1:
            return False, "no fix"
        age = time.time() - self.last_gga
        if age > GGA_STALE_S:
            return False, (f"no data from the receiver for {age:.0f}s -- "
                           f"the last position is stale, not current")
        # No HDOP reported = geometry unknown = not trusted. The NEO-M8N always
        # sends it with a fix, so this only bites on a malformed sentence.
        hdop = self.hdop if self.hdop is not None else 99.0
        if self.quality_ok:
            min_sats, max_hdop = MIN_SATS_KEEP, MAX_HDOP_KEEP
        else:
            min_sats, max_hdop = MIN_SATS_ACCEPT, MAX_HDOP_ACCEPT
        if self.sats < min_sats:
            return False, f"only {self.sats} sats (need {min_sats})"
        if hdop > max_hdop:
            return False, f"HDOP {hdop:.1f} (need <= {max_hdop})"
        if ERR_GATE and time.time() - self.last_gst <= GST_STALE_S and \
                self.err_m is not None:
            max_err = MAX_ERR_KEEP_M if self.quality_ok else MAX_ERR_ACCEPT_M
            if self.err_m > max_err:
                return False, (f"receiver puts its own error at "
                               f"{self.err_m:.1f} m (need <= {max_err:.0f})")
        return True, ""

    def publish_gps(self):
        msg = String()
        gst_fresh = time.time() - self.last_gst <= GST_STALE_S
        am = String()
        am.data = f"{self.err_m:.2f}" if gst_fresh and self.err_m else ""
        self.acc_pub.publish(am)
        if not gst_fresh and not self._gst_warned and \
                time.time() - self._started > 15.0 and \
                time.time() - self.last_gga < GGA_STALE_S:
            self._gst_warned = True
            self.get_logger().warn(
                "Receiver sends no error estimate (GST) -- fixes are judged "
                "on satellites and HDOP only, which do not see signals "
                "bouncing off buildings")
        ok, why = self._quality_check()
        if ok != self.quality_ok:
            self.quality_ok = ok
            hd = f"{self.hdop:.1f}" if self.hdop is not None else "?"
            if ok:
                self.get_logger().info(
                    f"GPS fix ACCEPTED: {self.sats} sats, HDOP {hd}")
            elif self.fix >= 1:
                self.get_logger().warn(
                    f"GPS fix REJECTED as unreliable: {why} -- publishing as "
                    f"no fix rather than a position that may be metres off")
        raw = None
        now = time.time()
        if ok:
            raw = (f"{self.lat:.7f},{self.lon:.7f},{self.heading},"
                   f"{self.speed_kmh:.1f},{self.fix},{self.sats}")
            self.raw_hist.append((now, self.lat, self.lon))
        while self.raw_hist and now - self.raw_hist[0][0] > RAW_MEAN_S:
            self.raw_hist.pop(0)
        fused = self.ref.fuse(now, (self.lat, self.lon) if ok else None,
                              self._raw_mean(now) if ok else None,
                              self._slam_pose(), self._slam_ok(now))
        ev = self.ref.take_event()
        if ev:
            self.get_logger().warn(ev)
            self.ref_note = ev
        if fused is not None:
            # your set position, carried along
            flat, flon, compass, src = fused
            if src == "SLAM":
                # No parked hold on SLAM: it does not wander like GPS (6 cm,
                # still, 2026-10-21), and it DOES see the robot pushed or
                # carried, which the hold would hide.
                lat, lon, held = flat, flon, False
                self.hold.anchor = None   # re-anchor if it falls back to GPS
            else:
                lat, lon, held = self.hold.apply(now, flat, flon)
            hdg = f"{compass:.1f}" if compass is not None else self.heading
            spd = 0.0 if held or not ok else self.speed_kmh
            msg.data = (f"{lat:.7f},{lon:.7f},{hdg},{spd:.1f},"
                        f"{self.fix if ok else 1},{self.sats}")
        elif ok:
            # real fix -> full position (held still while parked)
            lat, lon, held = self.hold.apply(now, self.lat, self.lon)
            spd = 0.0 if held else self.speed_kmh
            msg.data = (f"{lat:.7f},{lon:.7f},{self.heading},"
                        f"{spd:.1f},{self.fix},{self.sats}")
        else:
            # no fix, or a fix too poor to trust -> publish "acquiring" with
            # satellites IN VIEW so the app shows a live, rising count instead
            # of a blank screen. Every consumer (app, NAV, path recorder,
            # photo tagging) already treats this as "no position".
            siv = sum(self.sats_in_view.values()) if self.sats_in_view else 0
            if time.time() - self.last_gga > GGA_STALE_S:
                siv = 0     # a frozen count is as misleading as a frozen fix
            msg.data = f",,,0.0,0,{siv}"
        self.pub.publish(msg)
        ev = self.hold.take_event()
        if ev:
            self.get_logger().info(ev)
        rm = String()
        rm.data = raw if raw is not None else msg.data
        self.raw_pub.publish(rm)

        self._last_log += 1
        if self._last_log >= 10:
            self._last_log = 0
            hd = f"{self.hdop:.1f}" if self.hdop is not None else "?"
            if self.quality_ok:
                self.get_logger().info(
                    f"FIX  sats={self.sats} hdop={hd} lat={self.lat:.6f} "
                    f"lon={self.lon:.6f} hdg={self.heading or '-'} "
                    f"spd={self.speed_kmh:.1f}km/h"
                    + (f" err={self.err_m:.1f}m"
                       if self.err_m is not None else "")
                    + (" (parked: /gps held)" if self.hold.anchor else "")
                    + (f" (manual position: {self.ref.using or 'SET'})"
                       if self.ref.stage != "OFF" else ""))
            elif self.lat is not None and self.fix >= 1:
                err = f" err={self.err_m:.1f}m" if self.err_m is not None else ""
                self.get_logger().info(
                    f"weak fix held back: sats={self.sats} hdop={hd}{err} "
                    f"-- {why}")
            else:
                siv = sum(self.sats_in_view.values()) if self.sats_in_view else 0
                self.get_logger().info(
                    f"acquiring... {siv} sat(s) in view (no fix yet)")
            if self.bad_lines:
                self.get_logger().warn(
                    f"{self.bad_lines} garbled NMEA line(s) dropped (bad "
                    f"checksum) in the last 5 s -- serial wiring/noise?")
                self.bad_lines = 0

    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ============================ SIMULATION ============================
class GPSSim(Node):
    def __init__(self):
        super().__init__('gps_node')   # same node/topic name as real
        self.pub = self.create_publisher(String, '/gps', 10)
        self.raw_pub = self.create_publisher(String, '/gps_raw', 10)
        # As the real node, but on drive commands only -- no lidar: on the
        # bench the wheels are off the ground and the lidar sees a still
        # room, which would freeze the simulated drive.
        self.hold = StationaryHold()
        self.lat = SIM_START_LAT
        self.lon = SIM_START_LON
        self.heading = 90.0            # start facing East
        self.lin = 0.0
        self.ang = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(SIM_DT, self.step)
        self._log = 0
        self.ackermann = "--sim-ackermann" in sys.argv
        self.noise = "--sim-noise" in sys.argv
        self.ex = 0.0           # current GPS error, metres east/north
        self.ey = 0.0
        self.get_logger().warn(
            "GPS node in SIMULATION mode (NO real GPS). Virtual robot driven "
            "by /cmd_vel. Use real mode outdoors for the field test."
            + (f" Ackermann steering, {SIM_TURN_DIAMETER_M} m turning circle."
               if self.ackermann else " Pivot steering (turns on the spot).")
            + (f" GPS noise ON (sigma {SIM_NOISE_SIGMA_M} m, drifting)."
               if self.noise else " Perfect GPS."))

    def cmd_cb(self, msg: Twist):
        self.lin = max(-1.0, min(1.0, msg.linear.x))
        self.ang = max(-1.0, min(1.0, msg.angular.z))
        self.hold.commanded(time.time(), msg.linear.x)

    def step(self):
        spd = self.lin * SIM_MAX_SPEED_MPS
        # heading: compass bearing, +angular.z = LEFT (CCW) = decreasing
        if self.ackermann:
            # bicycle model: no motion, no turning; reversing turns the
            # other way, as a real car does
            yaw_dps = math.degrees(spd / (SIM_TURN_DIAMETER_M / 2.0)) * self.ang
        else:
            yaw_dps = self.ang * SIM_MAX_TURN_DPS
        self.heading = (self.heading - yaw_dps * SIM_DT) % 360.0
        br = math.radians(self.heading)
        d_north = spd * SIM_DT * math.cos(br)
        d_east = spd * SIM_DT * math.sin(br)
        self.lat += d_north / M_PER_DEG_LAT
        self.lon += d_east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        lat, lon, hdg = self.lat, self.lon, f"{self.heading:.1f}"
        if self.noise:
            a = SIM_DT / SIM_NOISE_TAU_S
            k = SIM_NOISE_SIGMA_M * math.sqrt(2 * a)
            self.ex += -self.ex * a + k * random.gauss(0, 1)
            self.ey += -self.ey * a + k * random.gauss(0, 1)
            lat += self.ey / M_PER_DEG_LAT
            lon += self.ex / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
            if abs(spd) < SIM_COG_MIN_MPS:
                hdg = ""                  # no course while barely moving
        raw = String()
        raw.data = (f"{lat:.7f},{lon:.7f},{hdg},"
                    f"{abs(spd) * 3.6:.1f},1,12")
        self.raw_pub.publish(raw)
        lat, lon, held = self.hold.apply(time.time(), lat, lon)
        ev = self.hold.take_event()
        if ev:
            self.get_logger().info(ev)
        msg = String()
        msg.data = (f"{lat:.7f},{lon:.7f},{hdg},"
                    f"{0.0 if held else abs(spd) * 3.6:.1f},1,12")
        self.pub.publish(msg)
        self._log += 1
        if self._log >= 25:
            self._log = 0
            self.get_logger().info(
                f"[SIM] lat={self.lat:.6f} lon={self.lon:.6f} "
                f"hdg={self.heading:.0f} spd={abs(spd) * 3.6:.1f}km/h")


def main():
    sim = "--sim" in sys.argv
    rclpy.init()
    node = GPSSim() if sim else GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
