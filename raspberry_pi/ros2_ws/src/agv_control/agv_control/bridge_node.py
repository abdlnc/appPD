#!/usr/bin/env python3
"""
AGV Bridge Node  (ROS2 <-> Flutter app over WebSocket)

Keeps the EXISTING Flutter app protocol, so NO app changes are needed:
    app  -> robot:  "x,y"                 joystick: x=steer(-1..1), y=speed(-1..1)
                    "MODE:AUTO" / "MODE:MANUAL" / "MODE:STOP"
                    "CAPTURE:ON" / "CAPTURE:OFF"  image saving, see CAPTURE below
                    "MODE:COVER"           systematic coverage of the field
                    "FENCE:SET <inset>|lat,lon;lat,lon;..."  set the boundary
                    "FENCE:CLEAR" / "FENCE:UNDO" / "FENCE:ADD lat,lon"
                    "COVER:RESET"          forget coverage progress, start over
                    "POS:TAP lat,lon"      set where the robot IS: 1st tap its
                                          position, 2nd a point it faces (see
                                          gps_node.py, YOU SET WHERE IT IS)
                    "POS:CLEAR"            drop it -- plain GPS again
                    "SAVEMAP"              save the live SLAM map (see below)
                    "RESETMAP"             wipe the live SLAM map (see below)
    robot -> app:   "C:<base64 jpeg>"     camera frame
                    "L:<deg,mm;deg,mm;...>"  Lidar point cloud (radar view)
                    "B:<fw>,<fh>|x1,y1,x2,y2,conf;..."  AI detection boxes,
                                          normalised 0-1, for the live-view
                                          overlay (see /ai_boxes)
                    "MAPSAVED:<name>"     SAVEMAP succeeded -> ~/agv_maps/<name>.png
                    "MAPERR:<reason>"     SAVEMAP failed (e.g. SLAM isn't running)
                    "MAPRESET:ok"         RESETMAP succeeded, map is now blank
                    "MAPRESETERR:<reason>" RESETMAP failed
                    "Z:<json>"             the field boundary (sent on connect
                                          and on every change)
                    "P:<json>"             the planned coverage lanes
                    "V:<json>"             coverage progress
                    "R:<json>"             the position you set, and what
                                          tracks it (gps_node /position_ref)
                    All four are NEW prefixes: an older app build ignores
                    them, so it keeps working unchanged.

Translates to ROS2:
    publishes  /cmd_vel (geometry_msgs/Twist)  manual joystick, only while MANUAL
    publishes  /mode    (std_msgs/String)      AUTO / MANUAL / STOP
    publishes  /env_mode (std_msgs/String)     OUTDOOR / INDOOR (from the app)
    publishes  /capture_enabled (std_msgs/String, latched)  ON / OFF
    relays     /field_type -> "F:<supported|unsupported>|<conf>" to the app
    relays     /avoid_action -> "A:<STOPPING|REVERSING|TURNING LEFT|...>"
    subscribes /image_raw/compressed (sensor_msgs/CompressedImage)
               -> relayed to the app as "C:<base64>"  (filled by camera_node)
    subscribes /scan (sensor_msgs/LaserScan)
               -> relayed to the app as "L:<deg,mm;...>", angle-corrected and
                  downsampled the same way control_node's obstacle zones are
                  (ANGLE_OFFSET_DEG=180 for the Lidar's mounting orientation).

SAVEMAP runs ~/save_slam_map.sh in a background thread (off the asyncio
loop, so it can't stall the camera/Lidar/GPS streams) -- it shells out to
`ros2 run nav2_map_server map_saver_cli`, which reads whatever the SLAM
node currently has on the /map topic. It does NOT start SLAM itself and
does NOT touch the Lidar driver -- if slam_toolbox isn't running (see
~/agv_slam/slam.launch.py), this just fails with a clear MAPERR reason.
bridge_node/agv.service are never affected either way.

rclpy spins in a background thread; the websockets server runs in asyncio.
Requires:  pip install websockets
"""

import asyncio
import base64
import json
import signal
import math
import os
import subprocess
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan

from . import scan_filter

import websockets

WS_HOST    = "0.0.0.0"
WS_PORT    = 8765
STREAM_FPS = 12          # camera frames pushed to the app per second
PUBLISH_HZ = 10          # manual command stream rate (keeps motor watchdog fed)

# ----------- Lidar relay (radar view in the app) -----------
LIDAR_ANGLE_OFFSET_DEG = 180.0   # mounting correction, matches control_node/nav_node
LIDAR_MIN_VALID_M      = 0.15    # ignore chassis/noise returns, matches control_node
LIDAR_MAX_POINTS       = 150     # downsample so each "L:" message stays small
LIDAR_STREAM_HZ        = 5       # radar refresh rate pushed to the app

# ----------- SLAM map save (SAVEMAP command) -----------
SAVE_MAP_SCRIPT   = os.path.expanduser("~/save_slam_map.sh")
SAVE_MAP_TIMEOUT  = 35.0   # s; bounds save_slam_map.sh's own internal retries

# ----------- SLAM map reset (RESETMAP command) -----------
# This slam_toolbox build exposes NO "reset map" service (checked: only
# clear_changes / pause_new_measurements / serialize_map / etc.), so the only
# way to actually wipe the map is to restart the node. SLAM therefore lives in
# its own systemd unit (agv-slam.service, see ~/start_slam.sh) precisely so
# this restart does NOT take bridge_node/camera/motors -- and the app's
# WebSocket connection -- down with it. Verified: bridge_node's PID is
# unchanged across an agv-slam restart.
RESET_MAP_CMD     = ["sudo", "-n", "systemctl", "restart", "agv-slam.service"]
RESET_MAP_TIMEOUT = 30.0   # s; the unit itself settles in ~2s, this is just a bound

# Top forward/reverse speed for MANUAL driving, as a fraction of full stick.
# Set to match AUTO mode's cruise speed (control_node.py's FWD_SPEED = 0.50)
# so the robot drives at the same pace whether a human or the obstacle
# avoidance is steering. Without it, full stick sent 1.0
# and manual was noticeably FASTER than AUTO.
# This scales the stick rather than clipping it, so the full throw still maps
# smoothly onto 0..MANUAL_SPEED instead of going dead past 60%.
# Kept as its own constant because bridge_node can't import control_node's:
# they are separate nodes in separate processes. If FWD_SPEED changes there,
# change this too. NAV mode is deliberately different (nav_node.py's
# CRUISE_SPEED = 0.80) -- set 0.80 here instead to match NAV.
# Steering (angular.z) is NOT scaled: it's an angle, not a speed.
# Manual top speed per environment, matching control_node's ENV_PROFILES so
# hand-driving and AUTO move at the same pace in either mode.
MANUAL_SPEED_BY_ENV = {"OUTDOOR": 1.00, "INDOOR": 0.50}

# ----------- CAPTURE: when the cameras are allowed to save images -----------
# Images are saved only while a run is being recorded -- from START AUTO, or
# from the record button beside the developer joystick -- and never at any
# other time. Before this, the soil camera saved a photo every minute from
# the moment the robot powered on, and the front camera saved every
# detection, so a robot left sitting in the shed filled the gallery with
# pictures of the shed.
#
# The app is the source of truth for when a run starts and ends (it owns
# the run log) and says so with CAPTURE:ON / CAPTURE:OFF. Two things here
# are deliberately independent of what the app says:
#
#   * Entering STOP turns capture OFF, whatever the reason -- the app's
#     emergency stop, the phone disconnecting, a waypoint being cleared.
#     A stopped robot has nothing to record, and a dropped connection must
#     not leave the cameras saving with nobody watching.
#   * Entering AUTO turns capture ON, so pressing START AUTO captures even if
#     the CAPTURE:ON that follows it is lost, or comes from an older app
#     build that does not send it.
#
# Capture only governs SAVING. Both models keep running regardless: the
# front detector still feeds AUTO's camera slow-down and the live boxes,
# and the soil classifier still feeds the field-type warning.
#
# Published latched (transient-local), so a camera node that starts after
# the bridge -- or restarts on its own -- receives the current state at once
# instead of guessing. Every node defaults to OFF until told otherwise.
CAPTURE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ----------- No-rebuild interim, for the currently installed app -----------
# Flag files, the same convention as ~/agv_use_sim. They let the boundary and
# COVER mode be tested with the app ALREADY on the phone, before rebuilding:
#
#   touch ~/agv_fence_edit   Every waypoint tapped on the map is saved as a
#                            boundary CORNER instead of a destination, and the
#                            robot is forced to STOP rather than drive to it.
#                            The app's "clear waypoint" removes the last corner.
#                            rm the file to go back to normal waypoints.
#
#   touch ~/agv_coverage     START AUTO runs COVER instead -- but only while an
#                            ACTIVE boundary exists. Without one it stays plain
#                            AUTO and says so in the log.
#
#   touch ~/agv_set_position Map taps tell the robot where it IS: first tap =
#                            its real position, second tap = a point it is
#                            facing (gps_node.py, YOU SET WHERE IT IS). The
#                            robot is held in STOP. "Clear waypoint" drops the
#                            set position (plain GPS again). rm the file for
#                            normal waypoints -- the set position stays.
#                            Checked before ~/agv_fence_edit.
#
# All are checked on every use, not at start-up, so touching or removing
# one takes effect immediately with no restart.
FENCE_EDIT_FLAG = os.path.expanduser("~/agv_fence_edit")
COVERAGE_FLAG = os.path.expanduser("~/agv_coverage")
SET_POSITION_FLAG = os.path.expanduser("~/agv_set_position")
DEFAULT_ENV = "OUTDOOR"
MANUAL_SPEED = MANUAL_SPEED_BY_ENV[DEFAULT_ENV]
                            # (map_saver_cli can transiently fail right after
                            # a restart -- the script retries up to 3x itself)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        self.mode = "STOP"
        self.latest_obstacle = "CLEAR"
        self.latest_boxes = None         # "<fw>,<fh>|..." from /ai_boxes
        self.latest_field = None         # "supported|0.96" from /field_type
        self.latest_action = None        # "REVERSING" etc from /avoid_action
        self.env = DEFAULT_ENV           # OUTDOOR / INDOOR, set by the app
        self.manual_speed = MANUAL_SPEED
        self.latest_gps = None
        self.manual_cmd = Twist()
        self.latest_jpeg = None          # bytes: compressed JPEG from camera_node
        self.latest_scan_str = None      # "deg,mm;deg,mm;..." for the app's radar view
        self.saving_map = False          # guard against overlapping SAVEMAP requests
        self.resetting_map = False       # guard against overlapping RESETMAP requests

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        # Latched-ish: republished whenever the app sets it, and once at
        # startup, so control_node hears it even if it started second.
        self.env_pub = self.create_publisher(String, '/env_mode', 10)

        # see CAPTURE above -- OFF until a run starts
        self.capture_pub = self.create_publisher(String, '/capture_enabled',
                                                 CAPTURE_QOS)
        self.capture_on = False
        self._publish_capture()

        # --- field boundary and COVER mode ---
        self.fence_cmd_pub = self.create_publisher(String, '/fence_cmd', 10)
        self.setpos_pub = self.create_publisher(String, '/set_position', 10)
        self.cover_cmd_pub = self.create_publisher(String, '/cover_cmd', 10)
        self.latest_fence = None         # JSON from fence_node (latched)
        self.fence_active = False
        self.latest_plan = None          # JSON from coverage_node (latched)
        self.latest_cover = None         # JSON progress from coverage_node
        self.latest_posref = None        # JSON from gps_node (set position)
        self.create_subscription(String, '/position_ref', self.posref_cb, 10)
        self.create_subscription(String, '/fence', self.fence_cb, CAPTURE_QOS)
        self.create_subscription(String, '/cover_plan', self.plan_cb,
                                 CAPTURE_QOS)
        self.create_subscription(String, '/cover_status', self.cover_cb, 10)
        self.waypoint_pub = self.create_publisher(String, '/waypoint', 10)
        self.create_subscription(CompressedImage, '/image_raw/compressed',
                                 self.img_cb, 10)

        self.create_subscription(String, '/obstacle', self.obstacle_cb, 10)
        self.create_subscription(String, '/ai_boxes', self.boxes_cb, 10)
        self.create_subscription(String, '/field_type', self.field_cb, 10)
        self.create_subscription(String, '/avoid_action', self.action_cb, 10)
        self.create_subscription(String, '/gps', self.gps_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.get_logger().info(f"Bridge node ready. WebSocket on :{WS_PORT}")

    # ---- camera frames arrive here from camera_node (Phase 5) ----
    def img_cb(self, msg: CompressedImage):
        self.latest_jpeg = bytes(msg.data)

    def obstacle_cb(self, msg: String):
        self.latest_obstacle = msg.data

    def boxes_cb(self, msg: String):
        self.latest_boxes = msg.data

    def action_cb(self, msg: String):
        self.latest_action = msg.data

    def field_cb(self, msg: String):
        self.latest_field = msg.data

    def fence_cb(self, msg: String):
        self.latest_fence = msg.data
        try:
            self.fence_active = bool(json.loads(msg.data).get("active"))
        except ValueError:
            self.fence_active = False

    def plan_cb(self, msg: String):
        self.latest_plan = msg.data

    def cover_cb(self, msg: String):
        self.latest_cover = msg.data

    def posref_cb(self, msg: String):
        self.latest_posref = msg.data

    def send_position_tap(self, arg):
        """POS:TAP / POS:CLEAR from the app -> gps_node, robot held in STOP
        (the same as a tap with ~/agv_set_position)."""
        m = String()
        m.data = "CLEAR" if arg.upper() == "CLEAR" else arg
        self.setpos_pub.publish(m)
        self.get_logger().info(f"Set position from app: {m.data}")
        self.set_mode("STOP")

    def send_fence_cmd(self, text):
        m = String()
        m.data = text
        self.fence_cmd_pub.publish(m)

    def send_cover_cmd(self, text):
        m = String()
        m.data = text
        self.cover_cmd_pub.publish(m)

    def gps_cb(self, msg: String):
        self.latest_gps = msg.data

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.latest_scan_str = ""
            return
        # Same sunlight filter the control node uses, so the radar view in
        # the app shows what the robot is actually acting on rather than a
        # noisier picture of its own.
        ranges, _ = scan_filter.clean(list(msg.ranges), list(msg.intensities))
        step = max(1, n // LIDAR_MAX_POINTS)   # downsample to ~LIDAR_MAX_POINTS
        parts = []
        ang = msg.angle_min
        for i, r in enumerate(ranges):
            a = ang
            ang += msg.angle_increment
            if i % step != 0:
                continue
            if math.isinf(r) or math.isnan(r) or r < LIDAR_MIN_VALID_M:
                continue
            deg = (math.degrees(a) + LIDAR_ANGLE_OFFSET_DEG) % 360.0
            parts.append(f"{deg:.1f},{r * 1000.0:.0f}")   # app expects mm
        self.latest_scan_str = ";".join(parts)

    # ---- mode handling ----
    def publish_mode(self):
        m = String(); m.data = self.mode
        self.mode_pub.publish(m)

    def set_mode(self, m):
        m = m.strip().upper()
        if m not in ("AUTO", "MANUAL", "STOP", "NAV", "COVER"):
            return
        if m == "AUTO" and os.path.exists(COVERAGE_FLAG):
            # see "No-rebuild interim"
            if self.fence_active:
                self.get_logger().warn(
                    "*** ~/agv_coverage present: START AUTO -> COVER mode ***")
                m = "COVER"
            else:
                self.get_logger().warn(
                    "~/agv_coverage present but NO ACTIVE BOUNDARY -- running "
                    "plain AUTO. Set a boundary first to use COVER.")
        if m != self.mode:
            self.mode = m
            self.get_logger().info(f"Mode -> {m} (from app)")
        self.publish_mode()
        # see CAPTURE: STOP always ends capture, AUTO always starts it
        if m == "STOP":
            self.set_capture(False, "robot stopped")
        elif m in ("AUTO", "COVER"):
            # COVER is a run like AUTO: objective 1.3's one-a-minute soil
            # photo applies to it too.
            self.set_capture(True, f"{m} started")

    # ---- image capture on/off (see CAPTURE) ----
    def _publish_capture(self):
        msg = String()
        msg.data = "ON" if self.capture_on else "OFF"
        self.capture_pub.publish(msg)

    def set_capture(self, on, why):
        on = bool(on)
        if on != self.capture_on:
            self.capture_on = on
            self.get_logger().info(
                f"Image capture {'ON' if on else 'OFF'} ({why})")
        # Republished even when unchanged: cheap, and it means a camera node
        # that somehow missed the latched copy is corrected at the next run
        # boundary rather than never.
        self._publish_capture()

    # ---- manual joystick -> Twist ----
    def set_env(self, name):
        """OUTDOOR / INDOOR from the app's developer settings."""
        name = name.strip().upper()
        if name not in MANUAL_SPEED_BY_ENV:
            return
        self.env = name
        self.manual_speed = MANUAL_SPEED_BY_ENV[name]
        m = String()
        m.data = name
        self.env_pub.publish(m)
        self.get_logger().info(
            f"Environment -> {name} (manual top speed {self.manual_speed:.2f})")

    def set_manual(self, x, y):
        # joystick screen coords -> ROS: forward = -y, left = -x
        t = Twist()
        t.linear.x = max(-1.0, min(1.0, -y)) * self.manual_speed
        t.angular.z = max(-1.0, min(1.0, -x))
        self.manual_cmd = t

    def handle_waypoint(self, payload):
        payload = payload.strip()
        if os.path.exists(SET_POSITION_FLAG):
            # see "No-rebuild interim": the tap is WHERE THE ROBOT IS
            m = String()
            m.data = "CLEAR" if payload.upper() == "CLEAR" else "TAP " + payload
            self.setpos_pub.publish(m)
            self.get_logger().warn(
                "*** SET POSITION (~/agv_set_position): "
                + ("set position cleared" if m.data == "CLEAR" else
                   "tap " + payload + " sent to gps_node")
                + ", robot held in STOP. rm ~/agv_set_position for normal "
                "waypoints. ***")
            self.set_mode("STOP")
            return
        if os.path.exists(FENCE_EDIT_FLAG):
            # see "No-rebuild interim": the tap is a CORNER, not a destination
            if payload.upper() == "CLEAR":
                self.send_fence_cmd("UNDO")
                self.get_logger().warn(
                    "*** FENCE EDIT (~/agv_fence_edit): waypoint clear -> "
                    "last boundary corner REMOVED ***")
            else:
                self.send_fence_cmd("ADD " + payload)
                self.get_logger().warn(
                    "*** FENCE EDIT (~/agv_fence_edit): waypoint " + payload +
                    " SAVED AS A BOUNDARY CORNER, robot held in STOP. "
                    "rm ~/agv_fence_edit for normal waypoints. ***")
            self.set_mode("STOP")
            return
        wp = String()
        if payload.upper() == "CLEAR":
            wp.data = "CLEAR"
            self.waypoint_pub.publish(wp)
            self.set_mode("STOP")
            self.get_logger().info("Waypoint cleared -> STOP")
        else:
            wp.data = payload
            self.waypoint_pub.publish(wp)
            self.set_mode("NAV")
            self.get_logger().info("Waypoint set: " + payload + " -> NAV")

    def publish_manual(self):
        if self.mode == "MANUAL":
            self.cmd_pub.publish(self.manual_cmd)

    # ---- SAVEMAP: save the live SLAM map (blocking -- call via an executor,
    #      never directly on the asyncio loop) ----
    def save_slam_map(self):
        name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        try:
            result = subprocess.run(
                [SAVE_MAP_SCRIPT, name],
                capture_output=True, text=True, timeout=SAVE_MAP_TIMEOUT)
        except subprocess.TimeoutExpired:
            return False, ("timed out waiting for a map -- is SLAM running? "
                            "(systemctl status agv-slam.service)")
        except Exception as e:
            return False, str(e)
        if result.returncode != 0:
            self.get_logger().warn("SAVEMAP failed: " + result.stderr.strip()[-300:])
            return False, ("save failed -- is SLAM running? "
                            "(systemctl status agv-slam.service)")
        self.get_logger().info("SAVEMAP saved: " + name)
        return True, name

    # ---- RESETMAP: wipe the live SLAM map by restarting agv-slam.service
    #      (blocking -- call via an executor, never directly on the asyncio
    #      loop). Only clears the LIVE map; already-saved snapshots in
    #      ~/agv_maps/ are left alone. ----
    def reset_slam_map(self):
        try:
            result = subprocess.run(
                RESET_MAP_CMD,
                capture_output=True, text=True, timeout=RESET_MAP_TIMEOUT)
        except subprocess.TimeoutExpired:
            return False, "timed out restarting agv-slam.service"
        except Exception as e:
            return False, str(e)
        if result.returncode != 0:
            self.get_logger().warn(
                "RESETMAP failed: " + result.stderr.strip()[-300:])
            return False, ("restart failed -- is agv-slam.service installed? "
                            "(systemctl status agv-slam.service)")
        self.get_logger().info("RESETMAP: slam_toolbox restarted, map cleared")
        return True, "ok"


async def ws_handler(websocket, node: BridgeNode):
    node.get_logger().info("Flutter app connected")

    # per-connection task: push latest camera frame to the app
    async def send_camera():
        delay = 1.0 / STREAM_FPS
        try:
            while True:
                jpg = node.latest_jpeg
                if jpg is not None:
                    b64 = base64.b64encode(jpg).decode('utf-8')
                    await websocket.send("C:" + b64)
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_obstacle():
        delay = 0.2
        last = None
        try:
            while True:
                obs = node.latest_obstacle
                if obs != last:
                    await websocket.send("O:" + obs)
                    last = obs
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_gps():
        delay = 0.5
        last = None
        try:
            while True:
                g = node.latest_gps
                if g is not None and g != last:
                    await websocket.send("G:" + g)
                    last = g
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_action():
        # What the robot is doing about an obstacle, as "A:<text>" (empty when
        # the maneuver is over). Polled faster than the other streams because
        # a turn lasts barely a second -- at 1Hz the app could miss it
        # entirely and show nothing while the robot visibly swerved.
        delay = 0.15
        last = None
        try:
            while True:
                a = node.latest_action
                if a is not None and a != last:
                    await websocket.send("A:" + a)
                    last = a
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_field():
        # Field type from the soil camera's classifier, as "F:<label>|<conf>".
        # Send-on-change: a new verdict only exists once a minute, when a soil
        # snapshot is taken, so there is nothing to poll for in between.
        delay = 1.0
        last = None
        try:
            while True:
                f = node.latest_field
                if f is not None and f != last:
                    await websocket.send("F:" + f)
                    last = f
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_boxes():
        # Send-on-change: the detector only produces a new result every
        # AI_MIN_INTERVAL (~3s), so polling faster would just resend the same
        # payload. Zero-detection results ARE sent (payload ends at "|") so the
        # app clears its overlay instead of leaving stale boxes up.
        delay = 0.3
        last = None
        try:
            while True:
                b = node.latest_boxes
                if b is not None and b != last:
                    await websocket.send("B:" + b)
                    last = b
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_on_change(prefix, getter, delay):
        # Boundary, plan and coverage progress: each sent once on connect
        # (the app needs the saved boundary straight away) and again only
        # when it changes.
        last = None
        try:
            while True:
                v = getter()
                if v is not None and v != last:
                    await websocket.send(prefix + v)
                    last = v
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def send_lidar():
        delay = 1.0 / LIDAR_STREAM_HZ
        try:
            while True:
                scan = node.latest_scan_str
                if scan:
                    await websocket.send("L:" + scan)
                await asyncio.sleep(delay)
        except websockets.exceptions.ConnectionClosed:
            pass

    cam_task = asyncio.create_task(send_camera())
    obs_task = asyncio.create_task(send_obstacle())
    gps_task = asyncio.create_task(send_gps())
    lidar_task = asyncio.create_task(send_lidar())
    boxes_task = asyncio.create_task(send_boxes())
    field_task = asyncio.create_task(send_field())
    action_task = asyncio.create_task(send_action())
    fence_task = asyncio.create_task(
        send_on_change("Z:", lambda: node.latest_fence, 0.5))
    plan_task = asyncio.create_task(
        send_on_change("P:", lambda: node.latest_plan, 1.0))
    cover_task = asyncio.create_task(
        send_on_change("V:", lambda: node.latest_cover, 1.0))
    posref_task = asyncio.create_task(
        send_on_change("R:", lambda: node.latest_posref, 0.5))
    try:
        async for message in websocket:
            msg = message.strip()
            if msg.upper().startswith("ENV:"):
                # Environment profile from the app's developer settings.
                node.set_env(msg[4:])
            elif msg.upper().startswith("MODE:"):
                node.set_mode(msg.split(":", 1)[1])
            elif msg.upper().startswith("CAPTURE:"):
                arg = msg.split(":", 1)[1].strip().upper()
                if arg in ("ON", "OFF"):
                    node.set_capture(arg == "ON", "from app")
            elif msg.upper().startswith("WP:"):
                node.handle_waypoint(msg.split(":", 1)[1])
            elif msg.upper().startswith("FENCE:"):
                # "FENCE:SET 3|lat,lon;..." -> fence_node "SET 3|lat,lon;..."
                node.send_fence_cmd(msg.split(":", 1)[1].strip())
            elif msg.upper().startswith("POS:"):
                arg = msg.split(":", 1)[1].strip()
                if arg.upper() == "CLEAR" or arg.upper().startswith("TAP "):
                    node.send_position_tap(arg)
            elif msg.upper().startswith("COVER:"):
                arg = msg.split(":", 1)[1].strip().upper()
                if arg == "START":
                    node.set_mode("COVER")
                elif arg == "RESET":
                    node.send_cover_cmd("RESET")
            elif msg.upper() == "SAVEMAP":
                if node.saving_map:
                    await websocket.send("MAPERR:already saving, wait a moment")
                else:
                    async def _do_save_map():
                        node.saving_map = True
                        try:
                            loop = asyncio.get_event_loop()
                            ok, info = await loop.run_in_executor(
                                None, node.save_slam_map)
                            reply = ("MAPSAVED:" if ok else "MAPERR:") + info
                            await websocket.send(reply)
                        except websockets.exceptions.ConnectionClosed:
                            pass
                        finally:
                            node.saving_map = False
                    asyncio.create_task(_do_save_map())
            elif msg.upper() == "RESETMAP":
                if node.resetting_map:
                    await websocket.send(
                        "MAPRESETERR:already resetting, wait a moment")
                else:
                    async def _do_reset_map():
                        node.resetting_map = True
                        try:
                            loop = asyncio.get_event_loop()
                            ok, info = await loop.run_in_executor(
                                None, node.reset_slam_map)
                            reply = ("MAPRESET:" if ok else "MAPRESETERR:") + info
                            await websocket.send(reply)
                        except websockets.exceptions.ConnectionClosed:
                            pass
                        finally:
                            node.resetting_map = False
                    asyncio.create_task(_do_reset_map())
            else:
                # joystick "x,y" -- grabbing the stick takes manual control
                try:
                    parts = msg.split(',')
                    x = float(parts[0]); y = float(parts[1])
                    if node.mode != "MANUAL":
                        node.set_mode("MANUAL")
                    node.set_manual(x, y)
                except (ValueError, IndexError):
                    pass   # ignore malformed messages
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        cam_task.cancel()
        obs_task.cancel()
        gps_task.cancel()
        lidar_task.cancel()
        boxes_task.cancel()
        field_task.cancel()
        action_task.cancel()
        fence_task.cancel()
        plan_task.cancel()
        cover_task.cancel()
        posref_task.cancel()
        # app gone -> STOP for safety
        node.set_mode("STOP")
        node.manual_cmd = Twist()
        node.get_logger().info("Flutter app disconnected -> STOP")


async def publisher_loop(node: BridgeNode):
    # stream the manual command at a fixed rate so the motor watchdog stays fed
    delay = 1.0 / PUBLISH_HZ
    while True:
        node.publish_manual()
        await asyncio.sleep(delay)


async def ws_main(node: BridgeNode):
    """Serve the app until a shutdown signal arrives.

    The obvious form of this -- `async with websockets.serve(...)` wrapped
    around publisher_loop -- hangs on exit while the phone still holds a
    connection: leaving the context manager waits for open connections to
    finish, and this one never does. systemd then waits out its full 90s stop
    timeout on every restart before killing the process, which also delays
    the STOP the robot should get promptly when the bridge goes down.

    So the signals are handled here: they set `stop`, the publisher task is
    cancelled, and the server is closed with a hard 2s bound on waiting for
    connections to drain."""
    async def handler(websocket):
        await ws_handler(websocket, node)

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except (NotImplementedError, RuntimeError):
            pass          # not all platforms/loops allow this; fall back to
                          # the default behaviour rather than failing to start

    server = await websockets.serve(handler, WS_HOST, WS_PORT)
    pub = asyncio.create_task(publisher_loop(node))
    try:
        await stop.wait()
    finally:
        node.get_logger().info("Bridge shutting down")
        pub.cancel()
        server.close()
        try:
            await asyncio.wait_for(server.wait_closed(), timeout=2.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            pass          # a client that will not let go must not hold up
                          # the whole service's shutdown


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    # spin ROS2 in the background; asyncio owns the main thread
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        asyncio.run(ws_main(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
