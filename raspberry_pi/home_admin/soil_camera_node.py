#!/usr/bin/env python3
"""
AGV Soil Camera Node  (ROS2 / rclpy)

Owns the SECOND USB camera -- the one mounted facing down at the soil below
the robot. Separate from camera_node.py (which owns the front-facing obstacle
/ pest camera at /dev/video0); this one is purely a periodic snapshot logger,
no live video relay, no AI.

Job: every CAPTURE_INTERVAL seconds, classify what the downward camera sees
and -- ONLY while a run is being recorded -- save it as a full-res JPEG into
CAPTURE_DIR. Those images are served to the app by the SAME gallery_server.py
as the front camera's captures/detections, under a third "soil" kind (see
gallery_server.py's DIRS).

When the robot is saving is decided by the bridge and published on
/capture_enabled -- see bridge_node.py's CAPTURE note. This node only obeys:
nothing is saved until it hears ON, and a run's first photo is taken the
moment ON arrives rather than up to a minute later.

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
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

import json

import cv2
import numpy as np

try:
    from agv_control import field_geometry as fg
except ImportError:          # the node still works, without dedup/boundary
    fg = None

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
# 2026-10-19: switched to the PyTorch (TorchScript) model on request. The
# TFLite model is kept beside it -- to go back, point this at
# agv_field_classifier.tflite; the backend follows the file extension.
#
# KNOWN BEHAVIOUR OF THE .pt, measured before switching: it does not separate
# field ground from indoor scenes. On 40 field-trip photos and 40 bench
# photos, checked by eye, it gives P(supported) about 0.10 on field ground
# and 0.07 indoors, so nearly every soil photo is classified UNSUPPORTED --
# tagged "_cls-notsoil" (hidden from the app's SOIL tab) and showing the
# UNSUPPORTED FIELD TYPE warning almost all the time. No input format fixes
# that: every size, scaling and channel order tried scored 50-60% on that
# set, where the TFLite model scores 76% (98% on field ground). The input
# used here is the one the model's own graph asks for -- it contains Keras'
# MobileNetV3 rescaling layer, so raw 0-255 RGB, like the TFLite -- and the
# conversion (Keras -> ONNX -> onnx2torch) is the likely culprit.
FIELD_MODEL_PATH = os.path.expanduser("~/agv_models/agv_field_classifier.pt")
FIELD_INPUT_SIZE = 224
# Feed RAW 0-255 pixels. The model normalises them itself.
#
# This was wrong from the day the classifier was deployed until 2026-10-15,
# and it meant the classifier never worked at all. The file is a Keras
# MobileNetV3Small, which carries its own preprocessing layer: its first two
# operations, read straight out of the graph, are
#
#     MUL  input * 0.007843   (= 1/127.5)
#     ADD  -1.0               layer "MobileNetV3Small/rescaling"
#
# so it expects 0-255 and maps that to -1..1 internally. This node divided
# by 255 first, so every image reached the network squeezed into
# -1.000..-0.992 -- effectively the same input every time -- and it answered
# "supported" to everything: flat grey, white, black, sky blue and a
# checkerboard all came back supported at 0.90-0.97, and so did all 4,435
# photos in the archive. That is why the app's UNSUPPORTED FIELD TYPE
# warning never once fired.
#
# The original justification for 0-1 was that "the three plausible scalings
# agreed on real soil images". A saturated model agrees with itself on
# everything, so that check could not have caught this. The test that does
# is to feed things that are plainly NOT field ground and see whether the
# verdict moves. With 0-255 it does: of the same 4,438 photos, 319 (7.2%)
# come back unsupported, they cluster in the indoor bench-testing sessions,
# and the most confident of them are a desk, a room and a wall.
FIELD_SCALE_0_1 = False
FIELD_LABELS = ("supported", "unsupported")   # index order of the output

# The verdict is written into each soil photo's FILENAME so the app can
# filter by it -- the gallery server has no metadata channel, the name is
# the only one there is. Shown in the app as "Soil" / "Not soil": the
# classifier is asked whether the robot is over a supported field, and a
# supported field is soil. A photo saved while the classifier is unavailable
# gets no tag at all and shows as unclassified -- never guessed.
#
#   soil_20261015_101500_lat14.632479_lon121.090634_s20261015-1000_cls-soil.jpg
SOIL_CLASS_TAG = {"supported": "soil", "unsupported": "notsoil"}

# ---- duplicate soil photos ----
# A soil photo near an earlier one -- the same spot, or a near-identical
# picture -- is still TAKEN: objective 1.3 requires one a minute, and that
# is not this node's to drop. It is MARKED, "_dup" in the name, so the app
# can tell the repeats apart. SOIL_DUP_POLICY = "skip" drops them instead,
# for when the objective allows it.
#
# Same two tests as the front camera (field_geometry.dedup_decision):
# within SOIL_DUP_RADIUS_M of a recent soil photo, both with a fix; or a
# dHash within SOIL_DUP_HASH_MAX bits of one. The picture test matters most
# here: measured on this robot's own archive, consecutive photos of the SAME
# patch of ground, robot parked, were put 6 to 14 m apart by the GPS -- and
# were all within 1 bit by dHash. 4 bits flagged no photo of genuinely
# different ground in the same test. The hash is taken from the clean frame,
# before the time/GPS text is burned in, which would otherwise make every
# photo look slightly different.
SOIL_DUP_POLICY = "tag"          # "tag" or "skip"
SOIL_DUP_RADIUS_M = 3.0
SOIL_DUP_HASH_MAX = 4
SOIL_DUP_RECENT = 30             # how many recent soil photos to compare with
DUP_TAG = "_dup"
# Taken outside the field boundary: still saved, marked "_outside".
OUTSIDE_TAG = "_outside"

# /capture_enabled is latched by bridge_node; subscribe the same way so the
# current state arrives on startup instead of only at the next change.
CAPTURE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

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


def load_field_model(path=None):
    """Load the field classifier: TorchScript for .pt, TFLite otherwise.
    Module-level, not a method, so ~/soil_backfill_classify.py classifies
    archived photos with EXACTLY the same code as live ones."""
    path = path or FIELD_MODEL_PATH
    if path.endswith(".pt"):
        import torch
        # One thread: this runs once a minute, and the front camera's YOLO
        # model needs the cores far more.
        torch.set_num_threads(1)
        m = torch.jit.load(path, map_location="cpu").eval()
        return ("torch", m)
    from ai_edge_litert.interpreter import Interpreter
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    return ("tflite", it)


def classify_frame(model, frame):
    """(label, confidence) for one BGR frame. Both models take RGB 224x224
    at raw 0-255 (they rescale internally); the .pt wants it channel-first."""
    kind, m = model
    x = cv2.resize(frame, (FIELD_INPUT_SIZE, FIELD_INPUT_SIZE))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32)
    if FIELD_SCALE_0_1:
        x /= 255.0
    if kind == "torch":
        import torch
        with torch.no_grad():
            t = torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1)[None]))
            probs = m(t).numpy().ravel()
    else:
        inp = m.get_input_details()[0]
        out = m.get_output_details()[0]
        m.set_tensor(inp['index'], x[None, ...])
        m.invoke()
        probs = m.get_tensor(out['index']).ravel()
    idx = int(np.argmax(probs))
    return FIELD_LABELS[idx], float(probs[idx])


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

        # Saving is OFF until the bridge says a run has started. When it turns
        # ON, the next tick shoots immediately rather than waiting out the
        # interval -- pressing START should produce a photo of where it began.
        self.capture_on = False
        self._shoot_now = False
        self.create_subscription(String, '/capture_enabled', self.capture_cb,
                                 CAPTURE_QOS)

        # --- duplicates and the field boundary (see SOIL_DUP_*) ---
        self.recent = []                 # [{lat, lon, conf, hash}]
        self.fence = None                # (frame, polygon) when active
        self.create_subscription(String, '/fence', self.fence_cb, CAPTURE_QOS)

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

        self.create_timer(1.0, self.tick)      # check every second; act on interval
        self.get_logger().info(
            f"Soil camera node ready (index {self.camera_index}). "
            f"Classifying every {int(CAPTURE_INTERVAL)}s; saving to "
            f"{CAPTURE_DIR} only while a run is recorded")

    def fence_cb(self, msg: String):
        self.fence = None
        if fg is None:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get("active") and d.get("points"):
            fr = fg.LocalFrame(*d["points"][0])
            self.fence = (fr, [fr.to_xy(a, b) for a, b in d["points"]])

    def _outside(self):
        if self.fence is None or self.gps_lat is None:
            return False
        fr, poly = self.fence
        return not fg.point_in_polygon(fr.to_xy(self.gps_lat, self.gps_lon), poly)

    @staticmethod
    def _dhash(frame):
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        g = g[:int(g.shape[0] * 0.85)]       # same crop as the archive check
        g = cv2.resize(g, (9, 8), interpolation=cv2.INTER_AREA)
        return fg.dhash_bits(g.tolist())

    def capture_cb(self, msg: String):
        on = msg.data.strip().upper() == "ON"
        if on and not self.capture_on:
            self._shoot_now = True             # first photo of the run, now
        if on != self.capture_on:
            self.get_logger().info(
                f"Soil capture {'ON -- saving' if on else 'OFF -- not saving'}")
        self.capture_on = on

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
            model = load_field_model()
            self.get_logger().info(
                f"Field classifier loaded: {os.path.basename(FIELD_MODEL_PATH)}")
            return model
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
            return classify_frame(self.field, frame)
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
        due = self._shoot_now or (now - self.last_capture >= CAPTURE_INTERVAL)
        if not due:
            return
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return
        self.last_capture = now
        self._shoot_now = False
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
        # Classified either way -- the field-type warning in the app depends
        # on it -- but only SAVED during a recorded run. See capture_cb.
        if not self.capture_on:
            return

        tag = SOIL_CLASS_TAG.get(label)
        dup = False
        if fg is not None:
            new = {"lat": self.gps_lat, "lon": self.gps_lon, "conf": 0.0,
                   "hash": self._dhash(frame)}
            verdict, why = fg.dedup_decision(new, self.recent,
                                             SOIL_DUP_RADIUS_M,
                                             SOIL_DUP_HASH_MAX)
            dup = verdict != fg.SAVE      # conf 0 never REPLACEs: tag only
            self.recent = (self.recent + [dict(new, conf=1.0)])[-SOIL_DUP_RECENT:]
            if dup and SOIL_DUP_POLICY == "skip":
                self.get_logger().info(f"Soil photo SKIPPED as a duplicate: {why}")
                return
            if dup:
                self.get_logger().info(f"Soil photo marked duplicate: {why}")
        fname = os.path.join(
            CAPTURE_DIR,
            when.strftime("soil_%Y%m%d_%H%M%S") + self._gps_suffix()
            + "_s" + SESSION_ID
            + (f"_cls-{tag}" if tag else "")
            + (DUP_TAG if dup else "")
            + (OUTSIDE_TAG if self._outside() else "") + ".jpg")
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
