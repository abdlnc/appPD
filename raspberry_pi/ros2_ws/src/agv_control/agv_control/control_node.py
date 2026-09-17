#!/usr/bin/env python3
"""
AGV Control Node  (ROS2 / rclpy)  --  AUTONOMOUS DECISION ONLY (no GPIO).

Subscribes:
    /scan  (sensor_msgs/LaserScan)
    /mode  (std_msgs/String)        only acts while mode == "AUTO"

Publishes:
    /cmd_vel (geometry_msgs/Twist)  linear.x = throttle (-1..1),
                                    angular.z = steer (-1..1, + = LEFT)

Bins the scan into 6 zones and runs reactive obstacle avoidance. A 10 Hz timer
streams the current command so the motor_node's command watchdog stays fed.
Reactive logic ported from agv_main.py.
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
MIN_DIST_FRONT     = 0.60     # obstacle trigger in the front zones
MIN_CLEARANCE_TURN = 0.40
MIN_CLEARANCE_REAR = 0.40
MIN_VALID_DIST     = 0.15     # ignore closer than this = own chassis / noise

# ----------- Lidar orientation (measured: mounted rotated 180 deg) -----------
ANGLE_OFFSET_DEG = 180.0

# ----------- Drive speeds (normalized -1..1; motor_node scales to PWM) -----------
# FWD_SPEED raised 0.40 -> 0.60 (client request: faster autonomous driving).
# Effective duty = FWD_SPEED * motor_node's SPEED_SCALE (0.60), so 24% -> 36%.
# SAFETY TRADEOFF: avoidance still triggers at MIN_DIST_FRONT (0.60m) on a
# 10Hz loop, so going faster leaves less room to react. If it starts clipping
# obstacles, either bring this back down or raise MIN_DIST_FRONT to buy back
# the reaction distance. Reverse/turn speeds deliberately left alone -- those
# happen in tight spots where more speed is not an improvement.
FWD_SPEED  = 0.60
REV_SPEED  = 0.60
TURN_SPEED = 0.50

# ----------- Obstacle WARNING thresholds (meters) -----------
WARN_CAUTION = 1.00     # yellow: heads-up
WARN_DANGER  = 0.60     # red: very close

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

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)
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
            self.set_cmd(FWD_SPEED, 0.0)        # cruise forward, centered

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
        """
        z = self.zones
        fwd_blocked = (z['front'] < MIN_DIST_FRONT
                       and z['front_l'] < MIN_DIST_FRONT
                       and z['front_r'] < MIN_DIST_FRONT)
        rear_blocked = z['back'] < MIN_CLEARANCE_REAR
        turn_blocked = (z['left'] < MIN_CLEARANCE_TURN
                        and z['right'] < MIN_CLEARANCE_TURN)
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
