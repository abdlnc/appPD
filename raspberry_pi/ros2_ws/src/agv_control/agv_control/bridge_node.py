#!/usr/bin/env python3
"""
AGV Bridge Node  (ROS2 <-> Flutter app over WebSocket)

Keeps the EXISTING Flutter app protocol, so NO app changes are needed:
    app  -> robot:  "x,y"                 joystick: x=steer(-1..1), y=speed(-1..1)
                    "MODE:AUTO" / "MODE:MANUAL" / "MODE:STOP"
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

Translates to ROS2:
    publishes  /cmd_vel (geometry_msgs/Twist)  manual joystick, only while MANUAL
    publishes  /mode    (std_msgs/String)      AUTO / MANUAL / STOP
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
import math
import os
import subprocess
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan

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
                            # (map_saver_cli can transiently fail right after
                            # a restart -- the script retries up to 3x itself)


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        self.mode = "STOP"
        self.latest_obstacle = "CLEAR"
        self.latest_boxes = None         # "<fw>,<fh>|..." from /ai_boxes
        self.latest_gps = None
        self.manual_cmd = Twist()
        self.latest_jpeg = None          # bytes: compressed JPEG from camera_node
        self.latest_scan_str = None      # "deg,mm;deg,mm;..." for the app's radar view
        self.saving_map = False          # guard against overlapping SAVEMAP requests
        self.resetting_map = False       # guard against overlapping RESETMAP requests

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.waypoint_pub = self.create_publisher(String, '/waypoint', 10)
        self.create_subscription(CompressedImage, '/image_raw/compressed',
                                 self.img_cb, 10)

        self.create_subscription(String, '/obstacle', self.obstacle_cb, 10)
        self.create_subscription(String, '/ai_boxes', self.boxes_cb, 10)
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

    def gps_cb(self, msg: String):
        self.latest_gps = msg.data

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.latest_scan_str = ""
            return
        step = max(1, n // LIDAR_MAX_POINTS)   # downsample to ~LIDAR_MAX_POINTS
        parts = []
        ang = msg.angle_min
        for i, r in enumerate(msg.ranges):
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
        if m not in ("AUTO", "MANUAL", "STOP", "NAV"):
            return
        if m != self.mode:
            self.mode = m
            self.get_logger().info(f"Mode -> {m} (from app)")
        self.publish_mode()

    # ---- manual joystick -> Twist ----
    def set_manual(self, x, y):
        # joystick screen coords -> ROS: forward = -y, left = -x
        t = Twist()
        t.linear.x = max(-1.0, min(1.0, -y))
        t.angular.z = max(-1.0, min(1.0, -x))
        self.manual_cmd = t

    def handle_waypoint(self, payload):
        payload = payload.strip()
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
    try:
        async for message in websocket:
            msg = message.strip()
            if msg.upper().startswith("MODE:"):
                node.set_mode(msg.split(":", 1)[1])
            elif msg.upper().startswith("WP:"):
                node.handle_waypoint(msg.split(":", 1)[1])
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
    async def handler(websocket):
        await ws_handler(websocket, node)
    async with websockets.serve(handler, WS_HOST, WS_PORT):
        await publisher_loop(node)      # runs forever alongside the server


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
