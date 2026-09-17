#!/usr/bin/env python3
"""
AGV Soil Camera Node  (ROS2 / rclpy)

Owns the SECOND USB camera -- the one mounted facing down at the soil below
the robot. Separate from camera_node.py (which owns the front-facing obstacle
/ pest camera at /dev/video0); this one is purely a periodic snapshot logger,
no live video relay, no AI.

Job: every CAPTURE_INTERVAL seconds, save a full-res JPEG of whatever the
downward camera currently sees into CAPTURE_DIR. Those images are served to
the app by the SAME gallery_server.py as the front camera's captures/
detections, just under a third "soil" kind (see gallery_server.py's DIRS).

A background thread reads frames continuously (same reasoning as
camera_node.py: keeps the buffer fresh so the saved frame isn't a stale one,
and a slow read never stalls the ROS executor).

Requires:  pip install opencv-python-headless   (already installed for
camera_node.py; nothing new to install)

NO colcon build needed. Run:
    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/soil_camera_node.py
"""

import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2

# ----------------- camera settings -----------------
CAMERA_INDEX  = 2           # /dev/video2 -- the second UVC camera (soil-facing)
                             # (NOT 0/1: those belong to the front camera_node
                             # and its metadata node -- do not fall back onto them)

# ----------------- snapshot capture -----------------
CAPTURE_INTERVAL = 60.0     # seconds between saved snapshots
CAPTURE_DIR       = os.path.expanduser("~/agv_soil_images")

# ----------------- stamp burned onto each image -----------------
# Soil photos get the capture time (and GPS, when there is a fix) drawn onto
# the image itself, not just in the filename -- these get viewed/exported one
# at a time from the app's gallery, where the filename isn't always in front
# of you. Safe to burn in here because nothing machine-reads these images;
# the FRONT camera's captures deliberately do NOT get an overlay, since the
# YOLOv8 obstruction model reads those and the pixels should stay pristine.
STAMP_SCALE     = 0.5
STAMP_THICKNESS = 1
STAMP_MARGIN    = 8         # px from the bottom-left corner


class SoilCameraNode(Node):
    def __init__(self):
        super().__init__('soil_camera_node')
        os.makedirs(CAPTURE_DIR, exist_ok=True)

        self.cap = self._open_camera()
        self.latest_frame = None        # full-res frame (BGR)
        self.lock = threading.Lock()
        self.running = True
        self.last_capture = 0.0

        # latest GPS fix, or (None, None) when there is no fix RIGHT NOW.
        # Cleared on loss rather than held, so an image is never tagged with a
        # stale position (same rule as camera_node.py).
        self.gps_lat = None
        self.gps_lon = None
        self.create_subscription(String, '/gps', self.gps_cb, 10)

        # background reader so cap.read() never blocks the ROS executor
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

        self.create_timer(1.0, self.tick)      # check every second; save on interval
        self.get_logger().info(
            f"Soil camera node ready (index {CAMERA_INDEX}). "
            f"Saving snapshots to {CAPTURE_DIR} every {int(CAPTURE_INTERVAL)}s")

    def gps_cb(self, msg: String):
        """/gps is "lat,lon,heading,speed,fix,sats", or ",,,0.0,0,<sats>" while
        acquiring -- the latter fails to parse, which clears the position."""
        p = msg.data.split(',')
        try:
            lat = float(p[0])
            lon = float(p[1])
        except (ValueError, IndexError):
            self.gps_lat = None
            self.gps_lon = None
            return
        self.gps_lat = lat
        self.gps_lon = lon

    def _gps_suffix(self):
        if self.gps_lat is None or self.gps_lon is None:
            return "_nogps"
        return f"_lat{self.gps_lat:.6f}_lon{self.gps_lon:.6f}"

    def _stamp(self, frame, when):
        """Burn "<time>  |  <gps>" into the bottom-left corner. Drawn twice --
        thick black first, then white on top -- so it stays readable over both
        dark soil and bright sunlit soil, without needing a filled box."""
        if self.gps_lat is None or self.gps_lon is None:
            gps = "GPS: no fix"
        else:
            gps = f"GPS {self.gps_lat:.6f}, {self.gps_lon:.6f}"
        text = when.strftime("%Y-%m-%d %H:%M:%S") + "   |   " + gps
        org = (STAMP_MARGIN, frame.shape[0] - STAMP_MARGIN)
        for color, thick in ((0, 0, 0), STAMP_THICKNESS + 2), \
                            ((255, 255, 255), STAMP_THICKNESS):
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX,
                        STAMP_SCALE, color, thick, cv2.LINE_AA)
        return frame

    def _open_camera(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(f"Soil camera opened at index {CAMERA_INDEX}")
            return cap
        cap.release()
        self.get_logger().error(
            f"No camera found at index {CAMERA_INDEX} - node will idle "
            f"(check `v4l2-ctl --list-devices` / /dev/video{CAMERA_INDEX})")
        return None

    def _reader_loop(self):
        while self.running:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self.lock:
                        self.latest_frame = frame
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.2)

    def tick(self):
        now = time.time()
        if now - self.last_capture < CAPTURE_INTERVAL:
            return
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return
        self.last_capture = now
        when = datetime.now()
        fname = os.path.join(
            CAPTURE_DIR,
            when.strftime("soil_%Y%m%d_%H%M%S") + self._gps_suffix() + ".jpg")
        try:
            cv2.imwrite(fname, self._stamp(frame, when))
            self.get_logger().info(f"Saved soil snapshot: {fname}")
        except Exception as e:
            self.get_logger().warn(f"Soil snapshot save failed: {e}")

    def shutdown(self):
        self.running = False
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SoilCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
