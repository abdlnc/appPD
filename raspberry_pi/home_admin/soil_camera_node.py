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
import numpy as np

# ----------------- camera settings -----------------
# Preferred index for the soil camera. NOT relied upon: USB cameras
# re-enumerate, and this camera has already moved from /dev/video2 to
# /dev/video3 across a replug. _open_camera() therefore probes candidates and
# keeps the first that actually DELIVERS A FRAME -- cv2.VideoCapture happily
# "opens" a non-existent index and then returns nothing, which looked exactly
# like a working node that silently saved no images.
CAMERA_INDEX  = 2           # first choice; others are tried if it yields nothing
FRONT_CAMERA_INDEX = 0      # camera_node owns this one -- never take it
MAX_CAMERA_INDEX   = 9      # how far to probe when searching
                             # (NOT 0/1: those belong to the front camera_node
                             # and its metadata node -- do not fall back onto them)

# ----------------- snapshot capture -----------------
CAPTURE_INTERVAL = 60.0     # seconds between saved snapshots
CAPTURE_DIR       = os.path.expanduser("~/agv_soil_images")
# One session = one run of this node, i.e. one time the robot was switched on.
# Written into every filename as "_s<YYYYmmdd-HHMM>" so the app can group and
# filter a run's photos together. Matches ai_detector_node's tag format.
SESSION_ID        = datetime.now().strftime("%Y%m%d-%H%M")

# ---- field-type classifier (TFLite) ----
# A 2-class image classifier run on each soil snapshot: is the robot over a
# field type this system supports? Output is [supported, unsupported] and
# sums to 1.0; index 0 is SUPPORTED (confirmed by the client).
#
# TFLite rather than PyTorch on purpose: the interpreter is a few MB against
# the ~1GB the YOLO process holds, and this model runs in 7.5 ms -- so at one
# frame a minute it costs about 0.01% of a core and cannot disturb the front
# camera's obstacle detection.
#
# Runtime is ai_edge_litert: the older tflite-runtime 2.14 refuses this file
# ("FULLY_CONNECTED version 12"). Install it with numpy pinned --
#     pip install ai-edge-litert "numpy<2"
# -- because unpinned it pulls numpy 2, which breaks ROS2, OpenCV and Torch.
FIELD_MODEL_PATH = os.path.expanduser("~/agv_models/agv_field_classifier.tflite")
FIELD_INPUT_SIZE = 224
# Pixel scaling used at training time. 0-1 (Keras rescale=1./255) is the
# assumption; the three plausible scalings agreed to within 0.03 on real soil
# images, so this is safe but worth correcting if the training code differs.
FIELD_SCALE_0_1 = True
FIELD_LABELS = ("supported", "unsupported")   # index order of the output

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

        self.camera_index = CAMERA_INDEX      # updated by _open_camera()
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

        # field-type classifier: optional, and a failure here must never stop
        # the node from doing its main job of saving soil images
        self.field_pub = self.create_publisher(String, '/field_type', 10)
        self.field = self._load_field_model()
        self.field_label = None      # last verdict, burned into the next image

        # background reader so cap.read() never blocks the ROS executor
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

        self.create_timer(1.0, self.tick)      # check every second; save on interval
        self.get_logger().info(
            f"Soil camera node ready (index {self.camera_index}). "
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

    def _load_field_model(self):
        try:
            from ai_edge_litert.interpreter import Interpreter
            it = Interpreter(model_path=FIELD_MODEL_PATH)
            it.allocate_tensors()
            self.get_logger().info(
                f"Field classifier loaded: {os.path.basename(FIELD_MODEL_PATH)}")
            return it
        except Exception as e:
            self.get_logger().warn(
                f"Field classifier unavailable ({type(e).__name__}: {e}) -- "
                f"soil images will still be saved, without a field type")
            return None

    def classify_field(self, frame):
        """Return (label, confidence) for this frame, or (None, 0.0)."""
        if self.field is None:
            return None, 0.0
        try:
            inp = self.field.get_input_details()[0]
            out = self.field.get_output_details()[0]
            x = cv2.resize(frame, (FIELD_INPUT_SIZE, FIELD_INPUT_SIZE))
            x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32)
            if FIELD_SCALE_0_1:
                x /= 255.0
            self.field.set_tensor(inp['index'], x[None, ...])
            self.field.invoke()
            probs = self.field.get_tensor(out['index']).ravel()
            idx = int(np.argmax(probs))
            return FIELD_LABELS[idx], float(probs[idx])
        except Exception as e:
            self.get_logger().warn(f"Field classification failed: {e}")
            return None, 0.0

    def _stamp(self, frame, when):
        """Burn "<time>  |  <gps>" into the bottom-left corner. Drawn twice --
        thick black first, then white on top -- so it stays readable over both
        dark soil and bright sunlit soil, without needing a filled box."""
        if self.gps_lat is None or self.gps_lon is None:
            gps = "GPS: no fix"
        else:
            gps = f"GPS {self.gps_lat:.6f}, {self.gps_lon:.6f}"
        text = when.strftime("%Y-%m-%d %H:%M:%S") + "   |   " + gps
        # Only an UNSUPPORTED verdict is drawn. "Supported" is the normal
        # case and stamping it on every image adds a line that says nothing --
        # the absence of the warning is the answer. On its OWN line above the
        # time+GPS, which already fills the width at 640px.
        lines = [text]
        if self.field_label == "unsupported":
            lines.insert(0, "FIELD: UNSUPPORTED")
        y = frame.shape[0] - STAMP_MARGIN
        for line in reversed(lines):          # bottom line first, upwards
            org = (STAMP_MARGIN, y)
            for color, thick in ((0, 0, 0), STAMP_THICKNESS + 2), \
                                ((255, 255, 255), STAMP_THICKNESS):
                cv2.putText(frame, line, org, cv2.FONT_HERSHEY_SIMPLEX,
                            STAMP_SCALE, color, thick, cv2.LINE_AA)
            y -= int(26 * STAMP_SCALE / 0.5)  # line spacing follows the scale
        return frame

    def _try_index(self, idx):
        """Open one index and require an actual frame from it.

        Reading a frame is the whole point: isOpened() alone returns True for
        an index with no device behind it, and for the metadata-only node some
        UVC cameras expose alongside their capture node."""
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(5):                 # first read after open often fails
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            time.sleep(0.15)
        cap.release()
        return None

    def _open_camera(self):
        tried = []
        for idx in [CAMERA_INDEX] + [i for i in range(MAX_CAMERA_INDEX + 1)
                                     if i not in (CAMERA_INDEX,
                                                  FRONT_CAMERA_INDEX)]:
            if not os.path.exists(f"/dev/video{idx}"):
                continue
            tried.append(idx)
            cap = self._try_index(idx)
            if cap is not None:
                self.camera_index = idx
                if idx != CAMERA_INDEX:
                    self.get_logger().warn(
                        f"Soil camera not at index {CAMERA_INDEX}; using "
                        f"index {idx} instead (USB re-enumeration)")
                else:
                    self.get_logger().info(f"Soil camera opened at index {idx}")
                return cap
        self.get_logger().error(
            f"No working soil camera found (probed {tried or 'nothing'}) - "
            f"node will idle. Check `v4l2-ctl --list-devices`.")
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

        # Classify BEFORE stamping, so the verdict can be burned into the
        # image and published while it still describes this frame.
        label, conf = self.classify_field(frame)
        if label is not None:
            self.field_label = label
            m = String()
            m.data = f"{label}|{conf:.3f}"
            self.field_pub.publish(m)
            if label == "unsupported":
                self.get_logger().warn(
                    f"FIELD TYPE UNSUPPORTED ({conf:.2f}) -- see the app's "
                    f"warning banner")
        fname = os.path.join(
            CAPTURE_DIR,
            when.strftime("soil_%Y%m%d_%H%M%S") + self._gps_suffix()
            + "_s" + SESSION_ID + ".jpg")
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
