#!/usr/bin/env python3
"""
AGV Control Node  (ROS2 / rclpy)  --  AUTONOMOUS DECISION ONLY (no GPIO).

Subscribes:
    /scan           (sensor_msgs/LaserScan)
    /mode           (std_msgs/String)   only acts while mode == "AUTO"
    /ai_detections  (std_msgs/String)   camera caution (see CAMERA CAUTION)

Publishes:
    /cmd_vel (geometry_msgs/Twist)  linear.x = throttle (-1..1),
                                    angular.z = steer (-1..1, + = LEFT)

Bins the scan into 6 zones and runs reactive obstacle avoidance. A 10 Hz timer
streams the current command so the motor_node's command watchdog stays fed.
Reactive logic ported from agv_main.py.

CAMERA CAUTION (deep-learning model feeding into the driving decision)
----------------------------------------------------------------------
The Lidar remains the sole authority on STOPPING and on which way to turn.
The camera can only ever do one thing here: make the robot DRIVE SLOWER.

Why it is limited to that: inference takes ~3s per frame (AI_MIN_INTERVAL),
so by the time a detection arrives it describes the world 3-4s ago. At
FWD_SPEED the robot has already travelled well past MIN_DIST_FRONT in that
time, so a camera detection is far too late to be a stop trigger -- treating
it as one would give the illusion of camera safety while actually braking
for things that are no longer there. Slowing down, by contrast, is useful
precisely because it is not time-critical: it buys back reaction distance
for the Lidar, which IS fast enough to stop.

What this gains: the Lidar is a single 2D slice at mount height, so a trowel
on the ground, a low rock or a sparse seedling can return almost nothing to
/scan while being obvious to the model. Those now at least slow the robot.

Fail-safe by construction: caution needs a FRESH detection (within CAM_TTL).
If ai_detector_node is not running, dies, or simply sees nothing, no
messages arrive, the TTL expires and the speed returns to FWD_SPEED -- i.e.
exactly today's Lidar-only behaviour. The camera cannot hold the robot
still, cannot steer it, and cannot trigger or clear the boxed-in hold.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from . import scan_filter

# ----------- Distance thresholds (METERS - LaserScan is in meters) -----------
# 2026-10-03: all three cut by 30% on request (0.90 -> 0.63, 0.40 -> 0.28),
# which also suits FWD_SPEED coming back down to 0.80. These are avoid()'s
# own working distances: when it starts a maneuver, and how much room it
# needs to turn into or reverse into.
#
# This is the tightest the robot has driven. At 0.63m and 80% duty it begins
# avoiding roughly two thirds as far out as it did a day ago, so there is
# less room to stop if it misreads a return. Raise these first if it starts
# clipping obstacles.
MIN_DIST_FRONT     = 0.70     # obstacle trigger in the front zones (was 0.40)
MIN_CLEARANCE_TURN = 0.28     # room needed to turn out of trouble (was 0.40)
MIN_CLEARANCE_REAR = 0.28     # room needed to reverse out (was 0.40)
MIN_VALID_DIST     = 0.15     # ignore closer than this = own chassis / noise

# ----------- Lidar orientation (measured: mounted rotated 180 deg) -----------
ANGLE_OFFSET_DEG = 180.0

# ----------- Drive speeds (normalized -1..1; motor_node scales to PWM) -----------
# FWD_SPEED raised 0.40 -> 0.60 (faster autonomous driving).
# Effective duty = FWD_SPEED * motor_node's SPEED_SCALE (0.60), so 24% -> 36%.
# SAFETY TRADEOFF: avoidance still triggers at MIN_DIST_FRONT (0.60m) on a
# 10Hz loop, so going faster leaves less room to react. If it starts clipping
# obstacles, either bring this back down or raise MIN_DIST_FRONT to buy back
# the reaction distance. Reverse/turn speeds deliberately left alone -- those
# happen in tight spots where more speed is not an improvement.
# 2026-09-24: FWD_SPEED 0.60 -> 1.00, the last of the software headroom.
# 2026-10-03: 1.00 -> 0.80 -> 0.50 over the day's testing. At 0.50 the robot
# drives at half the driver's output, which is also what makes the 0.40m
# front trigger below workable: slower approach, less distance needed to
# stop. REV/TURN left alone -- those happen in tight spots where speed is not
# an improvement.
# Superseded by ENV_PROFILES below for AUTO cruising -- kept because
# REV_SPEED and TURN_SPEED (the maneuver speeds) still come from here, and
# because a reader looking for "how fast does it drive" should find the
# profiles rather than a stale number pretending to be in charge.
FWD_SPEED  = 0.50          # see ENV_PROFILES; no longer read for cruising
REV_SPEED  = 0.60
TURN_SPEED = 0.50

# ----------- Camera caution (deep-learning model) -----------
# Cruise speed used INSTEAD of FWD_SPEED while the model currently sees an
# obstacle ahead. Deliberately not a stop: see CAMERA CAUTION in the header.
# Effective duty = CAM_CAUTION_SPEED * motor_node's SPEED_SCALE (0.60).
CAM_CAUTION_SPEED = 0.35
# Minimum confidence of the model's BEST box before it slows the robot.
# Higher than ai_detector_node's CONF_THRES (0.25) on purpose: ordinary
# scenery produces a trickle of marginal 0.25-0.42 detections, and honouring
# those would leave the robot crawling permanently in a planted field.
CAM_MIN_CONF = 0.40
# How long one detection keeps the robot cautious. Must comfortably exceed
# ai_detector_node's AI_MIN_INTERVAL (3.0s) or caution would flicker off
# between consecutive inferences of the same obstacle. This is also the
# fail-safe: nothing arriving for this long = full speed again.
CAM_TTL = 4.0

# ----------- Obstacle WARNING thresholds (meters) -----------
# Kept in step with MIN_DIST_FRONT: DANGER is meant to light up exactly when
# avoidance triggers, and CAUTION comfortably before it. Leaving DANGER at
# 0.60 while avoidance fired at 0.90 would have the robot swerving while the
# app still showed amber.
WARN_CAUTION = 1.40     # yellow: heads-up (left wide on purpose)
WARN_DANGER  = 0.70     # red: very close (tracks MIN_DIST_FRONT)

# ----------- Boxed-in ("NO PATH") distances -----------
# The boxed-in test uses its OWN distances, larger than the ones avoid()
# makes its decisions with. Reason, from testing on the robot: sharing
# avoid()'s thresholds meant NO PATH only appeared once an obstacle was
# practically inside the chassis -- a hand had to be within the robot's own
# frame before all four directions counted as blocked. Too late to be useful.
#
# The trade-off, stated plainly: NO PATH can now appear while avoid() still
# has a maneuver it could attempt -- it will reverse with 0.5m behind it,
# where this calls 0.70m blocked. The robot stops and asks for help slightly
# sooner than it is strictly out of options, which is the right direction to
# err in for a warning a human has to respond to.
#
# SIDES stay at MIN_CLEARANCE_TURN: turning out needs far less room than
# driving or reversing. Note that with the AND in path_blocked(), whichever
# distance is SMALLEST decides when the warning fires -- so the sides are now
# the binding constraint.
BLOCKED_FRONT = 1.20          # was MIN_DIST_FRONT (0.90)
BLOCKED_REAR  = 0.70          # was MIN_CLEARANCE_REAR (0.40)
# Deliberately a number of its own, not MIN_CLEARANCE_TURN any more: that
# shrank to 0.28 with the avoidance cut, and letting the NO PATH warning
# follow it down would undo the widening asked for a day earlier.
BLOCKED_SIDE  = 0.40

# ----------- No-path safety hold -----------
# Consecutive CLEAR scans required before driving again after being boxed in.
# Asymmetric on purpose: the hold engages on a SINGLE blocked scan (stopping
# is the fail-safe direction) but releases slowly, so Lidar noise sitting on
# the threshold can't make the robot lurch start-stop-start. ~0.5s at 10Hz.
BLOCKED_RELEASE_SCANS = 5

# ----------- Environment profiles (OUTDOOR / INDOOR) -----------
# Chosen from the app's developer settings and sent down as /env_mode. Two
# jobs need genuinely different behaviour from the same robot:
#
# OUTDOOR -- open ground. Drive at full output, and treat the whole 120-degree
#   front arc as the path: anything in it is in the way.
#
# INDOOR -- corridors and doorways. Drive at half output, and only the narrow
#   centre cone counts as the path. This is the important difference: the
#   front-left and front-right zones span 30-90 degrees either side, so in a
#   corridor the WALLS BESIDE the robot fall into them and trigger avoidance
#   even though the way ahead is completely clear. Ignoring them for the
#   trigger lets the robot drive through gaps it physically fits in. The
#   corners are still read -- they decide which way to turn once the centre
#   really is blocked -- and the clearances shrink so a tight turn is allowed.
#
# corners_trigger is the whole difference between "stops at every doorway" and
# "drives down the corridor".
ENV_PROFILES = {
    "OUTDOOR": {
        "fwd": 1.00,            # full duty on open ground
        "front": 0.70,          # avoidance trigger, metres
        "turn": 0.28,           # room needed to turn out
        "rear": 0.28,           # room needed to reverse out
        "corners_trigger": True,
        "caution": 1.40,        # app's amber ring
    },
    "INDOOR": {
        "fwd": 0.50,            # unchanged from today
        # 2026-10-06: 0.45 -> 0.60, the robot was reaching walls ahead of it.
        # 0.45m is less room than it sounds: returns closer than
        # MIN_VALID_DIST (0.15m) are discarded as chassis/noise, so a wall
        # the robot has already closed on reads as CLEAR and it drives on.
        # Triggering earlier keeps it out of that blind spot. Raising this
        # does NOT narrow the gaps it fits through -- corridors are about
        # corners_trigger below, not about this distance.
        "front": 0.60,
        "turn": 0.18,
        "rear": 0.18,
        "corners_trigger": False,   # walls beside the robot are not obstacles
        "caution": 0.90,
    },
}
DEFAULT_ENV = "OUTDOOR"

# ----------- Avoidance maneuver timing -----------
# Come to a FULL STOP before reversing, and again before driving out of the
# turn. motor_node enforces its own REVERSE_PAUSE at the hardware level to
# protect the gears, but that one only triggers on a direction flip while
# moving; this is the deliberate pause in the maneuver itself, long enough
# that the robot is visibly stationary before it backs away from something it
# just nearly hit.
STOP_BEFORE_REVERSE = 0.8      # s; was 0.5
STOP_AFTER_REVERSE  = 0.4      # s; was 0.2, before the turn begins
REVERSE_TIME        = 1.2      # s of backing away from the obstacle
SETTLE_TIME         = 0.4      # s to let the steering reach full lock
TURN_TIME           = 1.0      # s of driving while turned

# ----------- Gap seeking ("drive through what it fits through") -----------
# The robot used to treat a near return in the front arc as a wall: stop,
# reverse, turn, try again. That is correct for a wall and wrong for a
# doorway, a row gap or two obstacles with space between them -- it would
# shuffle back and forth in front of a gap it could simply have driven
# through.
#
# So before concluding the way is blocked, look for a heading along which a
# corridor as wide as the ROBOT is clear, and steer into it. Only when no
# such heading exists does the stop/reverse/turn maneuver run.
#
# WHY THIS RUNS DURING CRUISE, not just when the front zone trips: this
# machine steers with its front wheels and drives both rear wheels at the
# same duty (see motor_node), so it cannot pivot on the spot the way a
# differential-drive robot can -- it has a turning radius and must DRIVE to
# change heading. By the time an obstacle is at the 0.60-0.70m trigger
# there is no longer room to steer around it. Looking GAP_LOOKAHEAD ahead
# and easing into the gap early is what makes it possible at all.
#
# ROBOT_WIDTH IS A PHYSICAL MEASUREMENT, 60cm at the widest point as
# measured on this machine. It is the one number here that is not a tuning
# knob: set it wrong small and the robot will aim at gaps it cannot fit
# through and wedge itself; set it wrong large and it refuses gaps it could
# take, which is the behaviour this whole section exists to remove.
# Re-measure it after any change to wheels, axles or mudguards.
ROBOT_WIDTH     = 0.60     # m, widest point
ROBOT_LENGTH    = 0.80     # m, front bumper to rear bumper
GAP_SIDE_MARGIN = 0.05     # m of slack each side

# WHY LENGTH MATTERS TO A GAP. A long body does not sweep its own width
# unless it goes in straight: entering at an angle, the corners stick out,
# and the space needed is
#
#     ROBOT_WIDTH * cos(angle) + ROBOT_LENGTH * sin(angle)
#
# For this 60x80cm machine that is 0.60m straight on, 0.84m at 20 degrees
# and 0.92m at 30. An earlier version of this check required a flat 0.70m
# at every angle, which is 14cm short at 20 degrees and 22cm short at 30 --
# and since gap seeking deliberately picks angled headings, that was the
# normal case, not an edge case. It would have clipped doorframes.
#
# The requirement is therefore computed per candidate heading, not once.
# The practical effect is that tight gaps have to be approached nearly
# straight on, which is simply true of a robot this shape.

# How far ahead the corridor must be clear. Shorter and the robot commits
# too late to steer; longer and distant clutter rules out gaps that will
# have opened up by the time it arrives.
GAP_LOOKAHEAD = 1.50       # m

# Headings considered, in degrees either side of straight ahead, and the
# step between them. The arc is wider than the robot can steer to in one
# go on purpose: a far-off gap still pulls it in the right direction.
GAP_ARC_DEG  = 60.0
GAP_STEP_DEG = 5.0

# Heading error that maps to full steering lock -- same convention as
# nav_node's STEER_FULL_DEG, so the two modes feel the same to drive.
GAP_STEER_FULL_DEG = 45.0

# Closer than this in front, threading is abandoned: there is no room left
# to turn into the gap, so the stop/reverse/turn maneuver is the only
# honest option.
GAP_MIN_FRONT = 0.35       # m

# Cruise speed while steering through a gap, instead of prof['fwd']. Going
# slower buys back the reaction distance that aiming at a 10cm clearance
# spends, and a gap misjudged at speed is how a robot wedges itself.
GAP_SPEED = 0.35

# ----------- Turn direction: commitment and hysteresis -----------
# Each avoidance maneuver used to pick its direction from scratch, with no
# memory of the one before it. That is what made the robot oscillate in
# front of an obstacle: it would reverse and swing left, and the next
# trigger -- now looking at DIFFERENT side distances, because the robot had
# just moved -- would swing right, undoing the first escape. Repeat until
# something intervenes. Reported from the field as "it should be turning
# right but decides to turn left again".
#
# Two fixes, and they are both needed:
#
# TURN_COMMIT_TIME -- once a direction is chosen, a new maneuver starting
#   within this long of the last one ending REUSES it instead of deciding
#   again. The robot is still dealing with the same obstacle, so it keeps
#   working the same way around it. Drive clear for longer than this and
#   the next obstacle gets a fresh decision.
#
# TURN_SIDE_MARGIN -- one side must be clearer than the other by at least
#   this much to win. Without it, two readings a centimetre apart (noise,
#   not geometry) flip the choice between attempts, which is the same
#   oscillation arriving by a different route.
TURN_COMMIT_TIME = 5.0         # s
TURN_SIDE_MARGIN = 0.15        # m

# After this many committed repeats against the same obstacle, the same
# maneuver plainly is not working, so make it bigger rather than keep
# repeating it identically: back off further and turn for longer, in the
# SAME direction (flipping would be the oscillation again). If that still
# does not clear it, path_blocked() eventually reports NO PATH and a human
# is called -- which is the designed escape hatch, not a failure here.
TURN_ESCALATE_AFTER = 2
TURN_ESCALATE_MAX   = 2.5      # cap on the reverse/turn time multiplier


class AGVControl(Node):
    def __init__(self):
        super().__init__('control_node')

        self.mode = "STOP"
        self.is_avoiding = False
        self.cmd = Twist()             # current desired command
        self.zones = {k: 9999.0 for k in
                      ('front', 'front_r', 'right', 'back', 'left', 'front_l')}
        self._last_log = 0.0
        self._was_blocked = False   # rising-edge log guard for path_blocked()
        self._blocked = False       # latest path_blocked() result
        self.blocked_hold = False   # True = boxed in, refuse to drive
        self._clear_scans = 0       # consecutive clear scans while held

        # --- camera caution (see CAMERA CAUTION in the header) ---
        self._cam_time = 0.0        # when the last qualifying detection arrived
        self._cam_conf = 0.0        # its best-box confidence
        self._cam_count = 0         # how many boxes that detection had

        # --- gap seeking (see ROBOT_WIDTH) ---
        self._gap_sign = 0.0        # side of the last gap taken, for hysteresis
        self._gap_logged = 0.0      # rate-limit for the threading log

        # --- turn direction memory (see TURN_COMMIT_TIME) ---
        self._last_turn = 0.0       # steer sign of the last maneuver, 0 = none
        self._last_turn_end = 0.0   # when that maneuver finished
        self._turn_repeats = 0      # consecutive committed repeats

        # --- sunlight filter (see scan_filter) ---
        self._scan_blinded = False  # last scan had too few returns to trust
        self._blind_log = 0.0       # rate-limit for the blinded warning
        self._was_cautious = False  # edge-trigger guard for the log line

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle', 10)
        # What the robot is DOING about an obstacle, in words, for the app:
        # "STOPPING", "REVERSING", "TURNING LEFT", "TURNING RIGHT", "HOLDING",
        # or "" when the maneuver is over. Separate from /obstacle, which says
        # what the Lidar SEES -- seeing and reacting are different facts, and
        # the app may want either without the other.
        self.action_pub = self.create_publisher(String, '/avoid_action', 10)
        self._last_action = None

        # Active environment profile. The numbers below come from it rather
        # than from module constants, so a mode change takes effect on the
        # next scan without restarting anything.
        self.env = DEFAULT_ENV
        self.prof = dict(ENV_PROFILES[DEFAULT_ENV])
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)
        self.create_subscription(String, '/ai_detections', self.ai_cb, 10)
        self.create_subscription(String, '/env_mode', self.env_cb, 10)
        self.create_timer(0.1, self.tick)      # 10 Hz command stream

        self.get_logger().info(
            "Control node ready (autonomous logic). Waiting for mode AUTO.")

    # ----------------------- mode + command stream -----------------------
    def mode_cb(self, msg: String):
        m = msg.data.strip().upper()
        if m != self.mode:
            self.get_logger().info(f"Mode -> {m}")
        self.mode = m
        if m != "AUTO":
            self.cmd = Twist()         # zero out (we won't publish anyway)

    def publish_action(self, text):
        """Announce the current avoidance action. Published on CHANGE only,
        so the app sees transitions rather than a stream of repeats."""
        if text == self._last_action:
            return
        self._last_action = text
        m = String()
        m.data = text
        self.action_pub.publish(m)
        if text:
            self.get_logger().info(f"action: {text}")

    def set_cmd(self, lx, az):
        t = Twist()
        t.linear.x = float(lx)
        t.angular.z = float(az)
        self.cmd = t

    def tick(self):
        # only this node publishes /cmd_vel while AUTONOMOUS
        if self.mode != "AUTO":
            return
        if self.blocked_hold:
            # Boxed in -> hold at zero regardless of whatever avoid() last
            # computed. Enforced HERE, at the single point that publishes
            # /cmd_vel, so an already-running avoid() thread cannot drive
            # through the hold. Note we keep PUBLISHING (zeros) rather than
            # going silent: motor_node's command watchdog needs to stay fed,
            # and an explicit zero is a clearer intent than a timeout.
            self.pub.publish(Twist())
            return
        self.pub.publish(self.cmd)

    def env_cb(self, msg: String):
        name = msg.data.strip().upper()
        if name not in ENV_PROFILES or name == self.env:
            return
        self.env = name
        self.prof = dict(ENV_PROFILES[name])
        p = self.prof
        self.get_logger().info(
            f"Environment -> {name}: speed {p['fwd']:.2f}, front trigger "
            f"{p['front']:.2f}m, corners {'count' if p['corners_trigger'] else 'ignored'}"
            f", turn/rear clearance {p['turn']:.2f}/{p['rear']:.2f}m")

    # ----------------------- camera caution -----------------------
    def ai_cb(self, msg: String):
        """ai_detector_node's "<count>|Obstacle:<conf>,Obstacle:<conf>,..."

        Only the count and the BEST confidence are used. The class name is
        already "Obstacle" for every box by the time it gets here (the model's
        real 24 classes are overwritten in ai_detector_node), so there is no
        per-class policy to apply -- every detection is treated the same.
        """
        raw = msg.data.strip()
        try:
            head, _, tail = raw.partition('|')
            count = int(head)
        except ValueError:
            return                      # malformed -> ignore, stay uncautious
        if count <= 0:
            return                      # nothing seen; let the TTL lapse
        best = 0.0
        for item in tail.split(','):
            _, _, c = item.rpartition(':')
            try:
                best = max(best, float(c))
            except ValueError:
                continue
        if best < CAM_MIN_CONF:
            return                      # too marginal to slow down for
        self._cam_time = time.time()
        self._cam_conf = best
        self._cam_count = count

    def camera_caution(self):
        """True while a FRESH, confident camera detection should slow us."""
        return (time.time() - self._cam_time) < CAM_TTL

    def cruise_speed(self):
        """FWD_SPEED normally; CAM_CAUTION_SPEED while the model sees
        something ahead. This is the ONLY place the camera influences
        driving."""
        cautious = self.camera_caution()
        if cautious != self._was_cautious:
            self._was_cautious = cautious
            if cautious:
                self.get_logger().warn(
                    f"CAMERA sees {self._cam_count} obstacle(s) "
                    f"(conf {self._cam_conf:.2f}) -- slowing "
                    f"{self.prof['fwd']:.2f} -> {CAM_CAUTION_SPEED:.2f}")
            else:
                self.get_logger().info(
                    f"camera clear for {CAM_TTL:.0f}s -- back to "
                    f"{self.prof['fwd']:.2f}")
        return CAM_CAUTION_SPEED if cautious else self.prof['fwd']

    # ----------------------- scan processing -----------------------
    def scan_cb(self, msg: LaserScan):
        # Strip sunlight phantoms BEFORE anything reads the scan, so the
        # zones, the boxed-in test and the app all see the same cleaned
        # view. Rejected rays come back as inf, i.e. indistinguishable
        # from a ray that never returned -- see scan_filter.
        ranges, st = scan_filter.clean(list(msg.ranges), list(msg.intensities))
        self._scan_blinded = scan_filter.blinded(st)
        if self._scan_blinded:
            # Not "clear" -- cannot see. Logged rather than acted on,
            # because there is no safe automatic response: stopping on a
            # blinded scan would strand the robot in the sun, and driving
            # on is what it already does. The operator needs to know.
            now = time.time()
            if now - self._blind_log > 2.0:
                self._blind_log = now
                self.get_logger().warn(
                    f"LIDAR possibly sun-blinded: only "
                    f"{st['valid_frac']*100:.1f}% of rays returned anything "
                    f"-- obstacles may be invisible")
        buckets = {k: [] for k in self.zones}
        ang = msg.angle_min
        for r in ranges:
            a = ang
            ang += msg.angle_increment
            if math.isinf(r) or math.isnan(r) or r < MIN_VALID_DIST:
                continue
            deg = (math.degrees(a) + ANGLE_OFFSET_DEG) % 360.0
            if deg >= 330 or deg < 30:  buckets['front'].append(r)
            elif deg < 90:              buckets['front_l'].append(r)   # CCW = left
            elif deg < 150:             buckets['left'].append(r)
            elif deg < 210:             buckets['back'].append(r)
            elif deg < 270:             buckets['right'].append(r)
            else:                       buckets['front_r'].append(r)   # 270..330
        self.zones = {k: (min(v) if v else 9999.0) for k, v in buckets.items()}

        # ---- no-path safety hold (see tick()) ----
        self._blocked = self.path_blocked()
        if self._blocked:
            self._clear_scans = 0
            self.blocked_hold = True
        elif self.blocked_hold:
            self._clear_scans += 1
            if self._clear_scans >= BLOCKED_RELEASE_SCANS:
                self.blocked_hold = False
                self._clear_scans = 0

        self.publish_obstacle()

        # throttled status log (~2 Hz)
        now = time.time()
        if now - self._last_log > 0.5:
            self._last_log = now
            z = self.zones
            self.get_logger().info(
                f"F:{z['front']:.2f} FL:{z['front_l']:.2f} L:{z['left']:.2f} "
                f"B:{z['back']:.2f} R:{z['right']:.2f} FR:{z['front_r']:.2f}"
                f"  (m) [{self.mode}]")

        # autonomous decision (only when AUTO and not mid-maneuver)
        if self.mode != "AUTO" or self.is_avoiding:
            return
        if self.blocked_hold:
            # Nowhere to go -- avoid() has no maneuver that helps, so don't
            # start one. tick() is already holding the output at zero.
            self.set_cmd(0.0, 0.0)
            return
        z = self.zones
        front = self.prof['front']
        # INDOOR ignores the corner zones here on purpose -- see ENV_PROFILES.
        hit = z['front'] < front
        if self.prof['corners_trigger']:
            hit = hit or z['front_r'] < front or z['front_l'] < front

        # --- gap seeking, before anything is called blocked ---
        # Steer through what the robot physically fits through rather than
        # stopping for it. Runs during ordinary cruising too, not only when
        # the front zone trips, because this machine has to start turning
        # well before an obstacle is at the trigger distance -- see the
        # ROBOT_WIDTH section.
        #
        # Skipped once something is inside GAP_MIN_FRONT: at that range
        # there is no room left to steer around anything, and pretending
        # otherwise would drive the robot into it at an angle.
        if z['front'] > GAP_MIN_FRONT:
            steer, found = self.pick_gap(ranges, msg.angle_min,
                                         msg.angle_increment)
            if found:
                if steer == 0.0:
                    # Straight ahead fits: ordinary cruising, full speed.
                    self.set_cmd(self.cruise_speed(), 0.0)
                else:
                    # Thread the gap, slower -- see GAP_SPEED.
                    now = time.time()
                    if now - self._gap_logged > 1.0:
                        self._gap_logged = now
                        self.get_logger().info(
                            f"gap at {steer * GAP_STEER_FULL_DEG:+.0f} deg "
                            f"fits ({ROBOT_WIDTH:.2f}m + margin) -- "
                            f"steering through instead of stopping")
                    self.set_cmd(min(self.cruise_speed(), GAP_SPEED), steer)
                return

        if hit:
            # Nothing the robot fits through: fall back to the maneuver.
            self.is_avoiding = True
            threading.Thread(target=self.avoid, args=(dict(z),),
                             daemon=True).start()
        else:
            # cruise forward, centered -- at reduced speed if the camera
            # currently sees an obstacle (avoid() speeds are left alone:
            # those maneuvers are already slow and Lidar-governed).
            self.set_cmd(self.cruise_speed(), 0.0)

    # ----------------------- no-way-out detection -----------------------
    def path_blocked(self):
        """True when EVERY escape route is blocked -- forward, reverse and
        both turn directions. This is the robot boxed in with nowhere left to
        go, which the avoid() maneuver below cannot resolve on its own (it
        would just keep shuffling against the obstacles). Reported to the app
        so a human can intervene.

        Uses the same thresholds avoid() makes its decisions with, so the
        warning fires exactly when those decisions have run out of options --
        not on some separate, looser guess.

        Tried and REVERTED on 2026-09-25: requiring all six zones inside the
        DANGER ring (0.90m) instead. It fired earlier and read more simply,
        but it stopped the robot in places avoid() could still have driven out
        of -- it will reverse with 0.5m behind it -- and "NO PATH" has to mean
        the robot genuinely cannot move, or a human gets called to a robot
        that was never stuck. The asymmetry is the point: the front needs
        room to DRIVE, the sides and rear only need room to TURN or BACK OUT,
        which is less. The distances are BLOCKED_* above, set wider than
        avoid()'s own thresholds so the warning arrives before an obstacle is
        inside the robot's frame.
        """
        z = self.zones
        fwd_blocked = (z['front'] < BLOCKED_FRONT
                       and z['front_l'] < BLOCKED_FRONT
                       and z['front_r'] < BLOCKED_FRONT)
        rear_blocked = z['back'] < BLOCKED_REAR
        turn_blocked = (z['left'] < BLOCKED_SIDE
                        and z['right'] < BLOCKED_SIDE)
        return fwd_blocked and rear_blocked and turn_blocked

    # ----------------------- obstacle warning (to app) -----------------------
    def publish_obstacle(self):
        closest = min(self.zones, key=self.zones.get)
        dist = self.zones[closest]
        # DANGER tracks the active profile's trigger, so red still means
        # "this is where it reacts" in either environment.
        if dist < self.prof['front']:
            level = "DANGER"
        elif dist < self.prof['caution']:
            level = "CAUTION"
        else:
            level = "CLEAR"

        # Optional 4th field, appended only when boxed in. Deliberately an
        # EXTRA field rather than a new level: the app keys obstacle map-pins
        # off "DANGER", so replacing the level would silently stop pinning at
        # exactly the moment you'd most want a pin. Older parsers that only
        # read 3 fields keep working unchanged.
        blocked = self._blocked      # computed once per scan in scan_cb
        if blocked != self._was_blocked:
            self._was_blocked = blocked
            if blocked:
                self.publish_action("HOLDING")
                self.get_logger().warn(
                    "NO PATH -- boxed in (front/rear/both sides all blocked)"
                    " -- HOLDING at zero until a way out opens")
            else:
                self.publish_action("")
                self.get_logger().info("path clear again -- resuming")

        m = String()
        if level == "CLEAR":
            m.data = "CLEAR"
        else:
            m.data = f"{level}|{closest}|{dist:.2f}" + ("|BLOCKED" if blocked else "")
        self.obstacle_pub.publish(m)

    # ----------------------- gap seeking -----------------------
    def pick_gap(self, ranges, angle_min, angle_increment):
        """Find a heading the robot physically fits through.

        Returns (steer, found). steer is in the same -1..+1 units as
        /cmd_vel's angular.z, + = LEFT.

        The test for each candidate heading is a straight corridor wide
        enough for the body AT THAT ANGLE -- see ROBOT_LENGTH, which is why
        the width is not a constant -- running along the heading out to
        GAP_LOOKAHEAD. If no return falls inside it, the robot fits.

        That corridor is a STRAIGHT approximation of a path the robot can
        only reach by curving into it, so it slightly overstates what is
        reachable at large deviations. GAP_SIDE_MARGIN absorbs the error at
        the small angles that matter, and GAP_MIN_FRONT stops it being
        trusted once an obstacle is too close to steer around at all. It is
        a reactive gap-follower, not a planner -- it does not know where the
        gap leads, only that the robot fits into it now.
        """
        # Collect the nearby returns once, in robot-frame bearings, rather
        # than re-deriving the angle inside the candidate loop.
        pts = []
        for k, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r < MIN_VALID_DIST or r > GAP_LOOKAHEAD:
                continue
            deg = (math.degrees(angle_min + k * angle_increment)
                   + ANGLE_OFFSET_DEG) % 360.0
            brg = deg if deg <= 180.0 else deg - 360.0
            # Returns from behind the robot cannot block a forward corridor.
            if abs(brg) > 90.0:
                continue
            pts.append((math.radians(brg), r))

        # Candidate headings, ordered by how far they deviate from straight
        # ahead, so the robot always prefers the gap that costs it least
        # course change. Within a tie the side of the LAST gap is tried
        # first: without that, two symmetric gaps would have the robot
        # flip-flopping between them scan to scan.
        first_sign = 1.0 if self._gap_sign >= 0 else -1.0
        cands = [0.0]
        d = GAP_STEP_DEG
        while d <= GAP_ARC_DEG:
            cands.append(first_sign * d)
            cands.append(-first_sign * d)
            d += GAP_STEP_DEG

        for cand in cands:
            th = math.radians(cand)
            # Half-width the body actually needs at THIS heading -- see the
            # ROBOT_LENGTH note above.
            half = (ROBOT_WIDTH * math.cos(abs(th))
                    + ROBOT_LENGTH * math.sin(abs(th))) / 2.0 \
                + GAP_SIDE_MARGIN
            blocked = False
            for phi, r in pts:
                rel = phi - th
                # Along the candidate heading, and perpendicular to it.
                if r * math.cos(rel) <= 0.0:
                    continue                  # beside or behind: no block
                if abs(r * math.sin(rel)) < half:
                    blocked = True
                    break
            if not blocked:
                if cand != 0.0:
                    self._gap_sign = 1.0 if cand > 0 else -1.0
                steer = max(-1.0, min(1.0, cand / GAP_STEER_FULL_DEG))
                return steer, True

        return 0.0, False

    # ----------------------- turn direction -----------------------
    def pick_turn(self, z):
        """Which way to go around the obstacle. +1.0 = LEFT, -1.0 = RIGHT.

        Also returns how many times this direction has been reused, which
        avoid() uses to escalate. See TURN_COMMIT_TIME for why this has
        memory at all."""
        now = time.time()

        # 1. Already committed: still the same obstacle, so keep working the
        #    same way around it rather than undoing the last attempt.
        if (self._last_turn != 0.0
                and (now - self._last_turn_end) < TURN_COMMIT_TIME):
            self._turn_repeats += 1
            return self._last_turn, self._turn_repeats

        # A fresh obstacle: forget the last one.
        self._turn_repeats = 0

        # 2. Turn away from the side the obstacle is actually ON. The front
        #    corner zones are where something in the robot's path shows up;
        #    the old code only consulted them when a corner was already
        #    within the turn clearance (0.18-0.28m), which measured 0 times
        #    in 60 scans -- i.e. effectively never.
        fl, fr = z['front_l'], z['front_r']
        if abs(fl - fr) > TURN_SIDE_MARGIN:
            return (-1.0 if fl < fr else +1.0), 0

        # 3. Corners agree (both clear, or equally blocked): use what is
        #    beside the robot, which is what the old code always did.
        if abs(z['left'] - z['right']) > TURN_SIDE_MARGIN:
            return (+1.0 if z['right'] < z['left'] else -1.0), 0

        # 4. Genuinely symmetric -- open ground, or a wall square ahead.
        #    Reuse the last direction if there is one so repeated attempts
        #    stay consistent, otherwise pick left and stick to it.
        return (self._last_turn if self._last_turn != 0.0 else +1.0), 0

    # ----------------------- avoidance maneuver (thread) -----------------------
    def avoid(self, z):
        log = self.get_logger()
        log.warn("OBSTACLE - avoiding")

        # 1. STOP, and be seen to stop, before anything else.
        self.publish_action("STOPPING")
        self.set_cmd(0.0, 0.0)
        time.sleep(STOP_BEFORE_REVERSE)

        # Direction is chosen BEFORE reversing, so the choice is made from
        # the scan that saw the obstacle rather than from wherever the robot
        # ends up after backing away.
        steer, repeats = self.pick_turn(z)

        # Repeating the same escape and getting nowhere: make it bigger.
        grow = 1.0
        if repeats >= TURN_ESCALATE_AFTER:
            grow = min(TURN_ESCALATE_MAX,
                       1.0 + 0.5 * (repeats - TURN_ESCALATE_AFTER + 1))
            log.warn(f"still blocked after {repeats} attempts the same way "
                     f"-- backing off and turning {grow:.1f}x longer")

        # 2. reverse straight if the rear is clear, then stop AGAIN before the
        #    turn -- the robot never goes straight from backwards into a
        #    turning move.
        if self.mode == "AUTO" and z['back'] > self.prof['rear']:
            self.publish_action("REVERSING")
            self.set_cmd(-REV_SPEED, 0.0)
            time.sleep(REVERSE_TIME * grow)
            self.publish_action("STOPPING")
            self.set_cmd(0.0, 0.0)
            time.sleep(STOP_AFTER_REVERSE)

        log.info(f"going {'LEFT' if steer > 0 else 'RIGHT'} around it"
                 f"{f' (attempt {repeats + 1})' if repeats else ''}")

        if self.mode == "AUTO":
            # "AVOIDING" rather than naming the turn direction: which way it
            # swings is a detail of the maneuver, and the operator wants to
            # know the robot is handling an obstacle, not read a steering
            # commentary.
            self.publish_action("AVOIDING")
            self.set_cmd(0.0, steer)             # settle the steering first
            time.sleep(SETTLE_TIME)
            self.set_cmd(TURN_SPEED, steer)      # then drive while turned
            time.sleep(TURN_TIME * grow)

        self.set_cmd(0.0, 0.0)                   # stop, re-center
        self.publish_action("")                  # maneuver over
        # Remember the direction and WHEN it finished: a new maneuver inside
        # TURN_COMMIT_TIME reuses it instead of contradicting it.
        self._last_turn = steer
        self._last_turn_end = time.time()
        self.is_avoiding = False
        log.info("resume")


def main(args=None):
    rclpy.init(args=args)
    node = AGVControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
