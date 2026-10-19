#!/usr/bin/env python3
"""
AGV Coverage Node  (ROS2 / rclpy)  --  COVER mode's route planner.

A thin ROS wrapper: the planning and lane following live in
agv_control.coverage_planner, which is pure Python and tested in closed-loop
simulation (~/coverage_sim.py). This node feeds it GPS and publishes what it
says. It NEVER publishes /cmd_vel: control_node drives COVER, through its
obstacle avoidance, and only takes a preferred direction from here.

Subscribes:
    /gps          (std_msgs/String)  position
    /mode         (std_msgs/String)  active only while "COVER"
    /fence        (std_msgs/String, latched JSON)  the boundary -- see fence_node
    /fence_state  (std_msgs/String)  NOFIX means hold
    /cover_cmd    (std_msgs/String)  "RESET" -- forget progress, start over
    /cmd_vel      (geometry_msgs/Twist)  forward vs reverse, for the heading

Publishes:
    /cover_route  (std_msgs/String, 5 Hz)  "<steer>,<speed cap>" | "HOLD|<why>"
    /cover_status (std_msgs/String, 1 Hz JSON)  lane x of N, coverage %, ...
    /cover_plan   (std_msgs/String, latched JSON)  the lanes, for the app map

FAIL SAFE. Anything it cannot vouch for is a HOLD: no boundary, a boundary
the lanes cannot be planned in, no GPS fix, a GPS that has gone quiet. And
control_node treats a HOLD -- or silence from this node -- as stop.

RETURN TO START. After the last lane and the fill-in pass the planner
drives back to where the run started, then holds (coverage_planner.py,
RETURN TO START). The phase goes RETURN, then DONE; /cover_status carries
"home_m" (metres to go) while returning and "note" (how it ended) after.

RESUME. Progress is saved to ~/agv_coverage_state.json, keyed to the
boundary it was made in. Leaving COVER (STOP, an emergency stop, the phone
dropping) saves it; starting COVER again resumes from the same lane. A
DIFFERENT boundary starts fresh -- lanes from one field mean nothing in
another. "RESET" starts fresh on purpose.

NO colcon build for this file. Run:
    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/coverage_node.py
"""

import hashlib
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from agv_control import coverage_planner as cp

STATE_FILE = os.path.expanduser("~/agv_coverage_state.json")
ROUTE_HZ = 5.0
GPS_STALE_S = 2.0
SAVE_EVERY_S = 10.0
MOVING_CMD = 0.02

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def boundary_key(points, inset):
    raw = json.dumps([[round(a, 7), round(b, 7)] for a, b in points]
                     + [round(inset, 2)])
    return hashlib.sha1(raw.encode()).hexdigest()[:16]


class CoverageNode(Node):
    def __init__(self):
        super().__init__('coverage_node')
        self.route_pub = self.create_publisher(String, '/cover_route', 10)
        self.status_pub = self.create_publisher(String, '/cover_status', 10)
        self.plan_pub = self.create_publisher(String, '/cover_plan', LATCHED)
        self.create_subscription(String, '/gps', self.gps_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)
        self.create_subscription(String, '/fence', self.fence_cb, LATCHED)
        self.create_subscription(String, '/fence_state', self.fstate_cb, 10)
        self.create_subscription(String, '/cover_cmd', self.cmd_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        self.mode = "STOP"
        self.plan = None
        self.follower = None
        self.key = None
        self.plan_error = "no boundary"
        self.fence_state = "OFF"
        self.direction = 0
        self.last_cmd = ("HOLD", "not started")
        self.gps_time = 0.0
        self._saved_at = 0.0
        self._last_said = None

        self.create_timer(1.0 / ROUTE_HZ, self.tick)
        self.create_timer(1.0, self.publish_status)

    # ------------------------------------------------------------ plan
    def fence_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        pts = [tuple(p) for p in d.get("points", [])]
        inset = float(d.get("inset_m", 3.0))
        if not d.get("active"):
            self.plan = self.follower = self.key = None
            self.plan_error = d.get("error") or "no boundary"
            self._publish_plan()
            return
        key = boundary_key(pts, inset)
        if key == self.key:
            return                                 # unchanged
        plan, why = cp.make_plan(pts, inset)
        if plan is None:
            self.plan = self.follower = self.key = None
            self.plan_error = why
            self.get_logger().warn(f"COVER cannot plan this boundary: {why}")
            self._publish_plan()
            return
        self.plan, self.key, self.plan_error = plan, key, ""
        self.follower = cp.CoverageFollower(plan)
        self._try_resume()
        s = plan.summary()
        self.get_logger().info(
            f"COVER plan: {s['lanes']} lanes {s['spacing_m']} m apart, "
            f"route {s['route_length_m']} m ({s['loop_out_turns']} loop-out "
            f"turns); at {cp.CRUISE_SPEED_MPS} m/s about {s['minutes']} min, "
            f"a soil photo every {s['soil_spacing_m']} m -> about "
            f"{s['soil_photos']} photos.  (speed and turning circle are TODO "
            f"placeholders until measured -- see coverage_planner.py)")
        self._publish_plan()

    def _publish_plan(self):
        if self.plan is None:
            d = {"lanes": [], "error": self.plan_error}
        else:
            d = self.plan.to_latlon()
        m = String()
        m.data = json.dumps(d, separators=(",", ":"))
        self.plan_pub.publish(m)

    # ------------------------------------------------------------ resume
    def _try_resume(self):
        try:
            with open(STATE_FILE) as f:
                st = json.load(f)
        except (OSError, ValueError):
            return
        if st.get("key") != self.key:
            self.get_logger().info("saved COVER progress is for another "
                                   "boundary -- starting fresh")
            return
        self.follower.load_dict(st.get("follower", {}))
        self.get_logger().info(
            f"COVER progress restored: lane {self.follower.lane + 1}/"
            f"{len(self.follower.plan.lanes)}, "
            f"{self.follower.grid.coverage_pct():.0f}% covered")

    def _save(self):
        if self.follower is None:
            return
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump({"key": self.key,
                           "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "follower": self.follower.to_dict()}, f)
            os.replace(tmp, STATE_FILE)
        except OSError as e:
            self.get_logger().warn(f"coverage progress not saved: {e}")
        self._saved_at = time.time()

    # ------------------------------------------------------------ inputs
    def mode_cb(self, msg: String):
        m = msg.data.strip().upper()
        if m == self.mode:
            return
        if self.mode == "COVER":
            self._save()                  # leaving COVER: keep the progress
        if m == "COVER" and self.follower is not None:
            # It may have been pushed, carried or driven by hand since; do not
            # trust the old heading or paint coverage across the gap.
            self.follower.reset_heading()
        self.mode = m

    def cmd_cb(self, msg: String):
        if msg.data.strip().upper() == "RESET":
            if self.plan is not None:
                self.follower = cp.CoverageFollower(self.plan)
            try:
                os.remove(STATE_FILE)
            except FileNotFoundError:
                pass
            self.get_logger().warn("COVER progress RESET -- next run starts "
                                   "from the beginning")

    def fstate_cb(self, msg: String):
        self.fence_state = msg.data.split("|")[0].strip().upper()

    def vel_cb(self, msg: Twist):
        if msg.linear.x > MOVING_CMD:
            self.direction = +1
        elif msg.linear.x < -MOVING_CMD:
            self.direction = -1
        else:
            self.direction = 0

    def gps_cb(self, msg: String):
        if self.mode != "COVER" or self.follower is None:
            return
        p = msg.data.split(",")
        try:
            lat, lon = float(p[0]), float(p[1])
            pos = self.follower.plan.frame.to_xy(lat, lon)
        except (ValueError, IndexError):
            pos = None
        if pos is not None:
            self.gps_time = time.time()
        was = self.follower.phase
        self.last_cmd = self.follower.update(time.time(), pos, self.direction)
        now = self.follower.phase
        if now != was and (now == cp.DONE or
                           (now == cp.RETURN and was == cp.FILL)):
            if now == cp.RETURN:
                self.get_logger().info(
                    "COVER: field covered -- driving back to the start")
            else:
                self.get_logger().info(f"COVER: {self.follower.done_why}")
            self._save()
        if time.time() - self._saved_at > SAVE_EVERY_S:
            self._save()

    # ------------------------------------------------------------ outputs
    def _route(self):
        if self.mode != "COVER":
            return "HOLD|not in COVER"
        if self.follower is None:
            return f"HOLD|{self.plan_error}"
        if self.fence_state == "NOFIX":
            return "HOLD|no GPS fix"
        if time.time() - self.gps_time > GPS_STALE_S:
            return "HOLD|no recent GPS position"
        c = self.last_cmd
        if c[0] == "GO":
            return f"{c[1]:.3f},{c[2]:.2f}"
        return f"HOLD|{c[1]}"

    def tick(self):
        r = self._route()
        if r.startswith("HOLD") and r != self._last_said and \
                self.mode == "COVER":
            self.get_logger().warn(f"COVER holding: {r[5:]}")
        self._last_said = r if r.startswith("HOLD") else None
        m = String()
        m.data = r
        self.route_pub.publish(m)

    def publish_status(self):
        if self.follower is None:
            d = {"phase": "NOPLAN", "error": self.plan_error}
        else:
            d = self.follower.status()
            d["active"] = self.mode == "COVER"
        m = String()
        m.data = json.dumps(d, separators=(",", ":"))
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
