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
FWD_SPEED  = 0.50
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
        self._was_cautious = False  # edge-trigger guard for the log line

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)
        self.create_subscription(String, '/ai_detections', self.ai_cb, 10)
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
                    f"{FWD_SPEED:.2f} -> {CAM_CAUTION_SPEED:.2f}")
            else:
                self.get_logger().info(
                    f"camera clear for {CAM_TTL:.0f}s -- back to "
                    f"{FWD_SPEED:.2f}")
        return CAM_CAUTION_SPEED if cautious else FWD_SPEED

    # ----------------------- scan processing -----------------------
    def scan_cb(self, msg: LaserScan):
        buckets = {k: [] for k in self.zones}
        ang = msg.angle_min
        for r in msg.ranges:
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
        if (z['front'] < MIN_DIST_FRONT or z['front_r'] < MIN_DIST_FRONT
                or z['front_l'] < MIN_DIST_FRONT):
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
        if dist < WARN_DANGER:
            level = "DANGER"
        elif dist < WARN_CAUTION:
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
                self.get_logger().warn(
                    "NO PATH -- boxed in (front/rear/both sides all blocked)"
                    " -- HOLDING at zero until a way out opens")
            else:
                self.get_logger().info("path clear again -- resuming")

        m = String()
        if level == "CLEAR":
            m.data = "CLEAR"
        else:
            m.data = f"{level}|{closest}|{dist:.2f}" + ("|BLOCKED" if blocked else "")
        self.obstacle_pub.publish(m)

    # ----------------------- avoidance maneuver (thread) -----------------------
    def avoid(self, z):
        log = self.get_logger()
        log.warn("OBSTACLE - avoiding")
        self.set_cmd(0.0, 0.0)
        time.sleep(0.5)

        # reverse straight if the rear is clear
        if self.mode == "AUTO" and z['back'] > MIN_CLEARANCE_REAR:
            self.set_cmd(-REV_SPEED, 0.0)
            time.sleep(1.2)
            self.set_cmd(0.0, 0.0)
            time.sleep(0.2)

        # decide which way to turn AWAY from the obstacle
        # angular.z: +1.0 = LEFT, -1.0 = RIGHT
        if z['front_r'] < MIN_CLEARANCE_TURN:
            steer = +1.0                         # obstacle front-right -> go left
        elif z['front_l'] < MIN_CLEARANCE_TURN:
            steer = -1.0                         # obstacle front-left  -> go right
        else:
            steer = +1.0 if z['right'] < z['left'] else -1.0

        if self.mode == "AUTO":
            self.set_cmd(0.0, steer)             # settle the steering first
            time.sleep(0.4)
            self.set_cmd(TURN_SPEED, steer)      # then drive while turned
            time.sleep(1.0)

        self.set_cmd(0.0, 0.0)                   # stop, re-center
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
