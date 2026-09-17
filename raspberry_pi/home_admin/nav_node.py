#!/usr/bin/env python3
"""
AGV Navigation Node  (ROS2 / rclpy)  --  GPS WAYPOINT FOLLOWING (contract 2.2)

Subscribes:
    /waypoint (std_msgs/String)        "lat,lon" or "CLEAR"   (from bridge/app)
    /gps      (std_msgs/String)        "lat,lon,heading,speed,fix,sats"
    /scan     (sensor_msgs/LaserScan)  obstacle Stop/Reroute (2.2 Safety)
    /mode     (std_msgs/String)        acts ONLY while mode == "NAV"

Publishes:
    /cmd_vel  (geometry_msgs/Twist)    linear.x = throttle, angular.z = steer
                                       (+angular.z = LEFT, matches motor_node)

Behaviour:
  * Steer toward the great-circle BEARING to the waypoint; drive forward.
  * STOP within ARRIVAL_RADIUS (GPS is ~2.5 m, so don't expect cm precision).
  * Lidar obstacle ahead -> Stop / Reroute (turn to the clearer side); this has
    PRIORITY over the goal (2.2 Safety).
  * Heading comes from GPS course-over-ground; if unknown (stationary), creep
    forward to acquire a course, then steer.

NO colcon build needed. Run:
    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/nav_node.py
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ---- navigation tuning ----
ARRIVAL_RADIUS = 3.0       # m; within this = arrived
HEADING_TOL = 12.0         # deg; within this, go straight at cruise speed
STEER_FULL_DEG = 45.0      # heading error that maps to full steer
# Raised 0.60 -> 0.80 (client request: faster autonomous driving). Same
# tradeoff as control_node's FWD_SPEED: the Lidar stop/reroute still kicks in
# at FRONT_STOP_DIST (0.60m), so there is less room to react at speed. Only
# the aligned-and-cruising case is faster; TURN_SPEED is left alone so heading
# corrections stay controlled.
CRUISE_SPEED = 0.80        # forward speed when roughly aligned
TURN_SPEED = 0.40          # forward speed while correcting heading
LOOP_HZ = 5.0

# ---- obstacle (Lidar) ----
ANGLE_OFFSET_DEG = 180.0   # lidar mounted rotated 180 (matches control_node)
FRONT_STOP_DIST = 0.60     # m; obstacle ahead within this -> stop/reroute
MIN_VALID_DIST = 0.15      # ignore closer than this (own chassis / noise)

R_EARTH = 6371000.0


def bearing_deg(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def distance_m(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    y = math.radians(lat2 - lat1)
    return R_EARTH * math.hypot(x, y)


def norm180(a):
    return (a + 180.0) % 360.0 - 180.0


class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/waypoint', self.wp_cb, 10)
        self.create_subscription(String, '/gps', self.gps_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)

        self.mode = "STOP"
        self.target = None              # (lat, lon)
        self.lat = None
        self.lon = None
        self.heading = None             # deg, or None if unknown
        self.front_dist = 9999.0
        self.left_dist = 9999.0
        self.right_dist = 9999.0
        self._arrived_logged = False

        self.create_timer(1.0 / LOOP_HZ, self.tick)
        self.get_logger().info("Nav node ready. Waiting for waypoint + mode NAV.")

    def mode_cb(self, msg):
        self.mode = msg.data.strip().upper()

    def wp_cb(self, msg):
        d = msg.data.strip()
        if d.upper() == "CLEAR":
            self.target = None
            self.get_logger().info("Waypoint cleared")
            return
        try:
            lat, lon = d.split(',')
            self.target = (float(lat), float(lon))
            self._arrived_logged = False
            self.get_logger().info(f"New waypoint: {self.target}")
        except (ValueError, IndexError):
            self.get_logger().warn("Bad waypoint: " + d)

    def gps_cb(self, msg):
        p = msg.data.split(',')
        if len(p) < 2:
            return
        try:
            self.lat = float(p[0])
            self.lon = float(p[1])
        except ValueError:
            return
        if len(p) >= 3 and p[2].strip():
            try:
                self.heading = float(p[2])
            except ValueError:
                pass

    def scan_cb(self, msg):
        front = []
        left = []
        right = []
        ang = msg.angle_min
        for r in msg.ranges:
            a = ang
            ang += msg.angle_increment
            if math.isinf(r) or math.isnan(r) or r < MIN_VALID_DIST:
                continue
            deg = (math.degrees(a) + ANGLE_OFFSET_DEG) % 360.0
            if deg >= 330 or deg < 30:
                front.append(r)
            elif deg < 90:
                left.append(r)        # CCW = left
            elif deg > 270:
                right.append(r)
        self.front_dist = min(front) if front else 9999.0
        self.left_dist = min(left) if left else 9999.0
        self.right_dist = min(right) if right else 9999.0

    def send(self, lx, az):
        t = Twist()
        t.linear.x = float(max(-1.0, min(1.0, lx)))
        t.angular.z = float(max(-1.0, min(1.0, az)))
        self.pub.publish(t)

    def tick(self):
        # only drive while in NAV mode with a target and a known position
        if self.mode != "NAV" or self.target is None or self.lat is None:
            return

        tlat, tlon = self.target
        dist = distance_m(self.lat, self.lon, tlat, tlon)

        # arrived?
        if dist <= ARRIVAL_RADIUS:
            self.send(0.0, 0.0)
            if not self._arrived_logged:
                self._arrived_logged = True
                self.get_logger().info(
                    f"Arrived (<= {ARRIVAL_RADIUS:.0f} m). Stopping.")
            return

        # obstacle ahead? Stop / Reroute (2.2 Safety) -- PRIORITY over goal
        if self.front_dist < FRONT_STOP_DIST:
            if (self.left_dist < FRONT_STOP_DIST
                    and self.right_dist < FRONT_STOP_DIST):
                self.send(0.0, 0.0)               # boxed in -> stop
            elif self.left_dist >= self.right_dist:
                self.send(0.10, +1.0)             # turn LEFT (clearer side)
            else:
                self.send(0.10, -1.0)             # turn RIGHT (clearer side)
            return

        # need a heading; if unknown (stationary GPS), creep to acquire course
        if self.heading is None:
            self.send(0.25, 0.0)
            return

        # steer toward the waypoint bearing
        brg = bearing_deg(self.lat, self.lon, tlat, tlon)
        err = norm180(brg - self.heading)          # + = target to the RIGHT
        steer = max(-1.0, min(1.0, -err / STEER_FULL_DEG))  # +angular.z = LEFT
        speed = CRUISE_SPEED if abs(err) < HEADING_TOL else TURN_SPEED
        self.send(speed, steer)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
