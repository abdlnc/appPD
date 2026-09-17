#!/usr/bin/env python3
"""
AGV Camera Node  (ROS2 / rclpy)

Owns the USB webcam (the ONLY node that opens it). Two jobs:
  1. Publishes /image_raw/compressed (sensor_msgs/CompressedImage, JPEG) at
     PUBLISH_FPS -> the bridge_node relays these to the Flutter app (live view).
     Downscaled to STREAM_WIDTH x STREAM_HEIGHT at JPEG_QUALITY for bandwidth.
  2. Publishes /camera/full/compressed at AI_FEED_HZ -- FULL-RES, good quality,
     for ai_detector_node to run the YOLOv8 obstruction model on.

This node no longer SAVES anything itself. Captures are written by
ai_detector_node, and only when the model actually sees an obstacle in the
frame (client request: the CAMERA decides what is an obstacle, not the Lidar).
A separate full-res topic is used rather than reusing the app's live view
because that one is deliberately downscaled and heavily compressed -- fine for
a human eye, needlessly lossy for the detector.

A background thread reads frames continuously so a slow camera read never
stalls the ROS executor. Requires:  pip install opencv-python-headless
"""

import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2

# ----------------- camera / stream settings -----------------
CAMERA_INDEX  = 0           # /dev/video0 (falls back to 1, 2)
STREAM_WIDTH  = 480         # downscaled size sent to the app
STREAM_HEIGHT = 360
JPEG_QUALITY  = 50          # 0-100 (lower = smaller / faster)
PUBLISH_FPS   = 12          # frames published per second

# The front camera is physically mounted upside down on this robot -> every
# frame is rotated 180 deg BEFORE it goes anywhere else (app live view, AI
# feed, and therefore the raw/annotated images ai_detector_node saves, since
# those come from the same corrected frame). One flip here, not three places.
# Set False if the camera is ever remounted right-side up.
FLIP_UPSIDE_DOWN = True

# ----------------- full-res feed for the AI obstruction detector -----------
# Rate-limited on purpose: inference costs ~3s/frame on this CPU, so offering
# frames faster than it can chew just queues work that gets dropped. Raise for
# a more responsive detector, lower to give CPU back to SLAM.
AI_FEED_HZ = 0.5            # full-res frames offered to the detector per second


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cap = self._open_camera()
        self.latest_frame = None        # full-res frame (BGR)
        self.lock = threading.Lock()
        self.running = True

        self.pub = self.create_publisher(CompressedImage,
                                         '/image_raw/compressed', 10)
        # depth 2: a stale full-res frame is useless to the detector, so don't
        # let them pile up while it is busy with the previous one.
        self.ai_pub = self.create_publisher(CompressedImage,
                                            '/camera/full/compressed', 2)

        # background reader so cap.read() never blocks the ROS executor
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

        self.create_timer(1.0 / PUBLISH_FPS, self.tick)          # app live view
        self.create_timer(1.0 / AI_FEED_HZ, self.ai_tick)        # detector feed
        self.get_logger().info(
            f"Camera node ready. /image_raw/compressed at {PUBLISH_FPS}fps "
            f"(app), /camera/full/compressed at {AI_FEED_HZ}Hz (AI detector). "
            f"Saving is done by ai_detector_node, on detection only.")

    def _open_camera(self):
        for idx in (CAMERA_INDEX, 1, 2):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.get_logger().info(f"Camera opened at index {idx}")
                return cap
            cap.release()
        self.get_logger().error("No camera found at index 0/1/2 - node will idle")
        return None

    def _reader_loop(self):
        while self.running:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    if FLIP_UPSIDE_DOWN:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self.lock:
                        self.latest_frame = frame
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.2)

    def tick(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return

        # 1) publish a downscaled JPEG for the app
        stream = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT))
        ok, buf = cv2.imencode('.jpg', stream,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            self.pub.publish(msg)

    def ai_tick(self):
        """Offer a FULL-RES frame to ai_detector_node. Full quality on purpose:
        this is what the obstruction model actually reads, unlike the
        downscaled/heavily-compressed app view published by tick()."""
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.ai_pub.publish(msg)

    def shutdown(self):
        self.running = False
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
