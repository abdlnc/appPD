#!/usr/bin/env python3
"""
AGV Fence Node  (ROS2 / rclpy)  --  the field boundary.

Owns the field boundary the operator draws in the app, and says, five times
a second, where the robot is relative to it. It does NOT stop the robot
itself: motor_node does, because motor_node is the one place every drive
command passes through (see FENCE ENFORCEMENT in motor_node.py). This node
only has to be right about where the robot is.

Subscribes:
    /gps_raw    (std_msgs/String)  "lat,lon,heading,speed,fix,sats" -- the
                                   receiver's own positions, not the parked
                                   hold on /gps (see gps_node.py)
    /fence_cmd  (std_msgs/String)  from bridge_node:
                    "SET <inset_m>|lat,lon;lat,lon;..."   replace the boundary
                    "ADD lat,lon"                          append a corner
                    "UNDO"                                 drop the last corner
                    "CLEAR"                                remove the boundary
                    "INSET <m>"                            change the stop line
    /cmd_vel    (geometry_msgs/Twist)  only to know forward from reverse, for
                                       the heading estimate

Publishes:
    /fence_state  (std_msgs/String, 5 Hz)
        "OFF"                       no active boundary: nothing is fenced
        "NOFIX"                     a boundary is active but the robot cannot
                                    prove where it is -- motor_node stops it
        "IN|<m>|<turn>"             inside, <m> metres clear of the stop line
        "LINE|<turn>|<depth>|<dir>" past the stop line, still inside the boundary
        "OUT|<turn>|<depth>|<dir>"  outside the boundary
      <depth> is metres PAST THE STOP LINE: 0 at the line, the inset at the
      boundary, more outside. It only grows going the wrong way, which is
      what motor_node's "never deeper" guard watches.
      <dir> is the direction that heads back in: F (forward) if the robot
      faces within 90 degrees of the way in, else R (reverse). R when the
      heading is unknown: the robot almost always reaches the line driving
      forward, so backing up is the best guess -- and the depth guard
      catches the times it is wrong.
      <turn> is L, R or ? -- which way to turn to face back into the field,
      from the robot's heading; ? when the heading is not known yet. Sent
      while IN too: a robot that has just backed away from the line needs
      to know which way to turn in, and by then it is inside again.
    /fence  (std_msgs/String, latched JSON)  the boundary, its stop line and
            any validation error, for the app, the planner and the recorders

STORAGE: ~/agv_boundary.json, written atomically, so a power cut mid-write
leaves the previous boundary rather than half a file. Corners are stored
even while there are too few to make a field (corner-by-corner editing),
but a boundary is only ACTIVE -- enforced -- once it validates: at least 3
corners, edges that do not cross, a sensible area, and room left inside the
stop line. An inactive boundary is reported with its reason, never guessed
at.

THE STOP LINE is a soft fence inset_m inside the real boundary, to absorb
GPS error: at +/-2.5m, a robot that stopped AT the boundary would routinely
be over it. Default 3 m.

NO colcon build for this file. Run:
    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/fence_node.py
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from agv_control import field_geometry as fg

BOUNDARY_FILE = os.path.expanduser("~/agv_boundary.json")
DEFAULT_INSET_M = 3.0       # stop line distance inside the boundary
STATE_HZ = 5.0

# A position this old is not "where the robot is". gps_node already turns a
# dead serial stream into "no fix" after 3s; this is the second line, for a
# gps_node that has itself died and stopped publishing at all.
GPS_STALE_S = 2.0

# Heading from positions a few metres apart -- see field_geometry.TrackHeading.
HEADING_BASELINE_M = 3.0
# Below this commanded speed the robot is treated as stopped, so a creeping
# command does not flip the forward/reverse estimate back and forth.
MOVING_CMD = 0.02

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FenceNode(Node):
    def __init__(self):
        super().__init__('fence_node')
        self.state_pub = self.create_publisher(String, '/fence_state', 10)
        self.fence_pub = self.create_publisher(String, '/fence', LATCHED)
        # /gps_raw, NOT /gps: /gps is held still while parked, and the
        # fence must never rest on a held position (gps_node.py, THE FENCE
        # DOES NOT USE THE HOLD). Same format, same no-fix rules.
        self.create_subscription(String, '/gps_raw', self.gps_cb, 10)
        self.create_subscription(String, '/fence_cmd', self.cmd_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        self.points = []            # [(lat, lon)] as stored
        self.inset = DEFAULT_INSET_M
        self.active = False         # validated and enforced
        self.error = ""
        self.frame = None
        self.poly = None            # local x/y, when active
        self.inset_xy = None        # stop line, local x/y
        self.stop_line = None       # inset polygon in lat/lon, for drawing

        self.pos = None             # (x, y) local, latest usable fix
        self.pos_time = 0.0
        self.direction = 0          # +1 forward / -1 reverse, from /cmd_vel
        self.track = fg.TrackHeading(min_baseline=HEADING_BASELINE_M)
        self._last_kind = None

        self._load()
        self.create_timer(1.0 / STATE_HZ, self.tick)

    # ------------------------------------------------------------ storage
    def _load(self):
        try:
            with open(BOUNDARY_FILE) as f:
                d = json.load(f)
            self.points = [(float(a), float(b)) for a, b in d.get("points", [])]
            self.inset = float(d.get("inset_m", DEFAULT_INSET_M))
        except FileNotFoundError:
            self.points = []
        except (ValueError, TypeError, KeyError) as e:
            # A corrupt file is not a boundary. Say so, and do not guess.
            self.get_logger().error(
                f"{BOUNDARY_FILE} is unreadable ({e}) -- no boundary loaded")
            self.points = []
        self._apply(save=False)

    def _save(self):
        tmp = BOUNDARY_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"points": [[a, b] for a, b in self.points],
                       "inset_m": self.inset,
                       "saved": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)
        os.replace(tmp, BOUNDARY_FILE)

    def _apply(self, save=True):
        """Re-validate after any change, decide active or not, publish."""
        if save:
            if self.points:
                self._save()
            elif os.path.exists(BOUNDARY_FILE):
                os.remove(BOUNDARY_FILE)
        self.active, self.error = False, ""
        self.frame = self.poly = self.stop_line = self.inset_xy = None
        if self.points:
            ok, why, clean = fg.validate_boundary(self.points, self.inset)
            if ok:
                self.frame = fg.LocalFrame(*clean[0])
                self.poly = [self.frame.to_xy(*p) for p in clean]
                ins = fg.inset_polygon(self.poly, self.inset)
                self.inset_xy = ins
                self.stop_line = [self.frame.to_ll(*p) for p in ins]
                self.active = True
                self.track.reset()
            else:
                self.error = why
        if self.active:
            self.get_logger().info(
                f"Boundary ACTIVE: {len(self.poly)} corners, "
                f"{abs(fg.polygon_area(self.poly)):.0f} m2, stop line "
                f"{self.inset:.1f} m inside")
        elif self.points:
            self.get_logger().warn(
                f"Boundary NOT active ({len(self.points)} corner(s) stored): "
                f"{self.error}")
        else:
            self.get_logger().info("No boundary -- nothing is fenced")
        self._publish_fence()

    def _publish_fence(self):
        d = {"points": [[a, b] for a, b in self.points],
             "inset_m": self.inset,
             "active": self.active,
             "error": self.error,
             "stop_line": [[a, b] for a, b in (self.stop_line or [])]}
        if self.active:
            d["area_m2"] = round(abs(fg.polygon_area(self.poly)), 1)
        m = String()
        m.data = json.dumps(d, separators=(",", ":"))
        self.fence_pub.publish(m)

    # ------------------------------------------------------------ commands
    def cmd_cb(self, msg: String):
        text = msg.data.strip()
        verb, _, arg = text.partition(" ")
        verb = verb.upper()
        try:
            if verb == "SET":
                inset_s, _, pts_s = arg.partition("|")
                pts = []
                for pair in pts_s.split(";"):
                    if pair.strip():
                        a, b = pair.split(",")
                        pts.append((float(a), float(b)))
                self.points = pts
                if inset_s.strip():
                    self.inset = float(inset_s)
            elif verb == "ADD":
                a, b = arg.split(",")
                self.points.append((float(a), float(b)))
                self.get_logger().warn(
                    f"BOUNDARY CORNER {len(self.points)} ADDED at "
                    f"{float(a):.6f}, {float(b):.6f}")
            elif verb == "UNDO":
                if self.points:
                    self.points.pop()
            elif verb == "CLEAR":
                self.points = []
            elif verb == "INSET":
                self.inset = float(arg)
            else:
                self.get_logger().warn(f"unknown fence command: {text!r}")
                return
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"malformed fence command {text!r}: {e}")
            return
        self._apply()

    # ------------------------------------------------------------ position
    def vel_cb(self, msg: Twist):
        if msg.linear.x > MOVING_CMD:
            self.direction = +1
        elif msg.linear.x < -MOVING_CMD:
            self.direction = -1

    def gps_cb(self, msg: String):
        p = msg.data.split(",")
        try:
            lat, lon = float(p[0]), float(p[1])
        except (ValueError, IndexError):
            self.pos = None           # "acquiring" -- no position right now
            return
        if self.frame is None:
            return
        self.pos = self.frame.to_xy(lat, lon)
        self.pos_time = time.time()
        self.track.feed(self.pos_time, self.pos[0], self.pos[1], self.direction)

    # ------------------------------------------------------------ state
    def tick(self):
        if not self.active:
            out = "OFF"
        elif self.pos is None or time.time() - self.pos_time > GPS_STALE_S:
            out = "NOFIX"
        else:
            kind = fg.fence_status(self.pos, self.poly, self.inset)
            depth = fg.past_line_depth(self.pos, self.poly, self.inset)
            inward = fg.inward_bearing(self.pos, self.poly, self.inset_xy)
            turn, allow = "?", "R"
            h = self.track.heading()
            if h is not None:
                turn = "R" if fg.norm180(inward - h) > 0 else "L"
                allow = "F" if fg.allowed_direction(h, inward) > 0 else "R"
            if kind == fg.FENCE_IN:
                out = f"IN|{-depth:.1f}|{turn}"
            else:
                out = f"{kind}|{turn}|{depth:.1f}|{allow}"
        kind = out.split("|")[0]
        if kind != self._last_kind:
            # Two separate calls, never one call choosing its severity: rclpy
            # ties a severity to each CALL SITE and raises if the same line
            # logs at a second one -- which killed this node the first time
            # the robot went from IN to OUT.
            msg = f"Fence: {self._last_kind or 'start'} -> {out}"
            if kind in ("OFF", "IN"):
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)
            self._last_kind = kind
        m = String()
        m.data = out
        self.state_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
