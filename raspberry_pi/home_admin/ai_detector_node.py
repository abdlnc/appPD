#!/usr/bin/env python3
"""
AGV AI Detector Node  (ROS2 / rclpy)  --  contract 2.3 DEPLOYMENT/EXECUTION

Runs the CLIENT-PROVIDED YOLOv8 model (best.pt -- ~/agv_models/best.pt,
trained on a 24-class custom obstruction dataset: plants/rocks/trees/
gardening tools/etc.) on the FRONT camera's live feed, and saves a photo
only when the model actually SEES an obstacle.

  * Subscribes /camera/full/compressed (full-res JPEG from camera_node, at
    its AI_FEED_HZ). Frames arriving while a previous inference is still
    running are dropped -- there is no queue to fall behind on.
  * Runs inference at most every AI_MIN_INTERVAL seconds (~3s/frame on this
    CPU anyway); this is the CPU floor, raise it to give SLAM more headroom.
  * On ZERO detections: saves nothing at all.
  * On ONE OR MORE detections: saves the raw frame to ~/agv_captures/ AND the
    annotated one to ~/agv_detections/, sharing a basename.
  * Publishes /ai_detections (std_msgs/String): "count|Obstacle:conf,..."
  * Publishes /ai_boxes (std_msgs/String): "<fw>,<fh>|x1,y1,x2,y2,conf;..."
    with box corners NORMALISED 0-1 against the frame it inferred on, so the
    app can scale them onto a video widget of any size. Published on EVERY
    inference including zero-detection ones (payload ends at "|"), so the app
    clears its overlay promptly instead of leaving boxes up. This is what
    draws boxes on the live view -- note they are inherently 1-2s stale and
    only refresh every AI_MIN_INTERVAL; see the app side, which labels their
    age and expires them rather than pretending they are live.

THE CAMERA decides what an obstacle is here, not the Lidar (client request).
The Lidar's own obstacle warning is separate and still drives avoidance in
control_node.py -- these two are deliberately independent.

  * Every box is labeled "Obstacle" regardless of the model's own 24 class
    names (client request) -- see _relabel_obstacle(). The real class name is
    thrown away entirely, not just hidden; only count/confidence/box survive.

Filenames carry the GPS position when there is a fix:
    capture_20260905_120000_lat14.748134_lon121.061668.jpg
    capture_20260905_120000_nogps.jpg          (no fix at that moment)
The position is CLEARED when the fix is lost rather than kept at its last
value -- tagging a photo with a stale position is worse than "nogps".

  Previously used the older RT-DETR model (rtdetr_best.pt, still kept at
  ~/agv_models/rtdetr_best.pt for reference/rollback, not deleted), and
  previously watched ~/agv_captures/ for files that camera_node saved on a
  timer. camera_node no longer saves anything.

NO colcon build needed. Run directly:

    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/ai_detector_node.py

Optional one-shot self-test on a single image (no ROS, just proves inference):

    python3 ~/ai_detector_node.py --test /path/to/image.jpg
"""

import os
import sys
import time
import threading

MODEL_PATH   = os.path.expanduser("~/agv_models/best.pt")
CAPTURE_DIR  = os.path.expanduser("~/agv_captures")
DETECT_DIR   = os.path.expanduser("~/agv_detections")
CONF_THRES   = 0.25         # confidence threshold for drawing/reporting
AI_MIN_INTERVAL = 3.0       # s; floor between inferences (CPU guard)
AI_TOPIC     = "/camera/full/compressed"

# ---- guards on SAVING (not on detecting) ----------------------------------
# Detection still reports everything >= CONF_THRES, but a photo is only worth
# keeping if the model is reasonably sure. Observed in the field: ordinary
# scenery produces a steady trickle of 0.25-0.26 "obstacles" right at the
# threshold, which saved ~1 image every 7s and would bury the gallery.
# Set SAVE_CONF = CONF_THRES to go back to saving every single detection.
SAVE_CONF     = 0.50        # min confidence of the BEST box before saving
# And once something is saved, don't re-save the same thing every few seconds
# while the robot sits in front of it / drives up to it.
SAVE_COOLDOWN = 15.0        # s between saved photos


def _load_model():
    from ultralytics import YOLO
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError("Model not found: " + MODEL_PATH)
    return YOLO(MODEL_PATH)


def _relabel_obstacle(result):
    """Overwrite every class name in this result with "Obstacle" (client
    request) -- affects BOTH the saved annotated image (r.save() reads
    result.names to draw box labels) and _summarize()'s output below, since
    both read from the same result.names dict. Confidence/box position are
    untouched; only the specific class identity is discarded."""
    result.names = {k: "Obstacle" for k in result.names}


def _summarize(result):
    """Return (count, 'Obstacle:conf,...') from an ultralytics result."""
    names = result.names
    items = []
    if result.boxes is not None:
        for b in result.boxes:
            cls = names.get(int(b.cls[0]), str(int(b.cls[0])))
            conf = float(b.conf[0])
            items.append(f"{cls}:{conf:.2f}")
    return len(items), ",".join(items)


# ------------------------------------------------------------------ one-shot
def run_test(image_path):
    print("Loading model:", MODEL_PATH)
    model = _load_model()
    os.makedirs(DETECT_DIR, exist_ok=True)
    print("Running inference on:", image_path)
    t = time.time()
    results = model.predict(image_path, conf=CONF_THRES, verbose=False)
    dt = time.time() - t
    r = results[0]
    _relabel_obstacle(r)
    count, summary = _summarize(r)
    out = os.path.join(DETECT_DIR, "selftest_detected.jpg")
    r.save(filename=out)               # writes annotated image
    print(f"=== INFERENCE OK in {dt:.1f}s ===")
    print(f"Detections: {count}  [{summary}]")
    print("Annotated image saved:", out)


# ------------------------------------------------------------------ ROS node
def run_node():
    import numpy as np
    import cv2
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage
    from datetime import datetime

    class AIDetector(Node):
        def __init__(self, model):
            super().__init__('ai_detector_node')
            self.model = model
            os.makedirs(CAPTURE_DIR, exist_ok=True)
            os.makedirs(DETECT_DIR, exist_ok=True)
            self.pub = self.create_publisher(String, '/ai_detections', 10)
            self.box_pub = self.create_publisher(String, '/ai_boxes', 10)
            self.busy = False
            self.last_infer = 0.0
            self.last_save = 0.0

            # latest GPS fix, or (None, None) when there is no fix RIGHT NOW.
            # Cleared on loss rather than held, so a photo is never tagged with
            # a stale position.
            self.gps_lat = None
            self.gps_lon = None
            self.create_subscription(String, '/gps', self.gps_cb, 10)

            # depth 1: if we are mid-inference the newest frame is the only one
            # worth having; older ones are already out of date.
            self.create_subscription(CompressedImage, AI_TOPIC,
                                     self.frame_cb, 1)
            self.get_logger().info(
                f"AI detector ready. Watching {AI_TOPIC} (camera decides what "
                f"an obstacle is); saves to {CAPTURE_DIR} + {DETECT_DIR} "
                f"ONLY on detection.")

        # ---- GPS tagging ----
        def gps_cb(self, msg: String):
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

        # ---- live frame -> inference ----
        def frame_cb(self, msg: CompressedImage):
            # Drop the frame outright if busy or too soon -- deliberately no
            # queueing: a backlog of stale frames is worse than missing one.
            if self.busy or (time.time() - self.last_infer) < AI_MIN_INTERVAL:
                return
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                 cv2.IMREAD_COLOR)
            if frame is None:
                return
            self.busy = True
            self.last_infer = time.time()
            threading.Thread(target=self._infer, args=(frame,),
                             daemon=True).start()

        def _infer(self, frame):
            try:
                t = time.time()
                results = self.model.predict(frame, conf=CONF_THRES,
                                             verbose=False)
                dt = time.time() - t
                r = results[0]
                _relabel_obstacle(r)
                count, summary = _summarize(r)

                msg = String()
                msg.data = f"{count}|{summary}"
                self.pub.publish(msg)

                # Normalised boxes for the app's live-view overlay. Sent even
                # when count == 0 so the app can clear, rather than leaving a
                # stale box on screen until the next detection.
                fh, fw = frame.shape[:2]
                parts = []
                if r.boxes is not None:
                    for b in r.boxes:
                        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                        parts.append("%.4f,%.4f,%.4f,%.4f,%.2f" % (
                            x1 / fw, y1 / fh, x2 / fw, y2 / fh,
                            float(b.conf[0])))
                bmsg = String()
                bmsg.data = f"{fw},{fh}|" + ";".join(parts)
                self.box_pub.publish(bmsg)

                if count == 0:
                    # Camera sees nothing -> save nothing (client request).
                    self.get_logger().debug(f"[{dt:.1f}s] no obstacle in view")
                    return

                # Detected something, but is it worth a photo?
                best = max((float(b.conf[0]) for b in r.boxes), default=0.0)
                if best < SAVE_CONF:
                    self.get_logger().debug(
                        f"[{dt:.1f}s] {count} low-confidence "
                        f"(best {best:.2f} < {SAVE_CONF}) -- not saved")
                    return
                if time.time() - self.last_save < SAVE_COOLDOWN:
                    self.get_logger().debug(
                        f"[{dt:.1f}s] {count} detection(s) within cooldown "
                        f"-- not saved")
                    return
                self.last_save = time.time()

                base = ("capture_" + datetime.now().strftime("%Y%m%d_%H%M%S")
                        + self._gps_suffix())
                raw = os.path.join(CAPTURE_DIR, base + ".jpg")
                out = os.path.join(DETECT_DIR, base + "_detected.jpg")
                cv2.imwrite(raw, frame)     # untouched frame, no overlay
                r.save(filename=out)        # annotated copy
                self.get_logger().info(
                    f"[{dt:.1f}s] {count} detection(s): {summary}  -> {out}")
            except Exception as e:
                self.get_logger().error("Inference failed: " + str(e))
            finally:
                self.busy = False

    rclpy.init()
    print("Loading model:", MODEL_PATH, "(may take a few seconds)")
    model = _load_model()
    node = AIDetector(model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == "--test":
        run_test(sys.argv[2])
    else:
        run_node()


if __name__ == "__main__":
    main()
