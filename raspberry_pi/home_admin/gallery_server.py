#!/usr/bin/env python3
"""
AGV Gallery Server  --  read-only HTTP viewer for saved images

Serves the camera snapshots (~/agv_captures/), AI-annotated detection
images (~/agv_detections/), downward soil-camera snapshots
(~/agv_soil_images/), and saved SLAM maps (~/agv_maps/) over plain HTTP
so the Flutter app -- and any browser on the same network, including on
the Pi itself -- can list and view them. Independent of ROS2/GPIO/camera:
safe to start/stop/restart on its own without affecting the rest of the
stack.

Endpoints:
    GET /list/captures    -> JSON array of {name,size,mtime}, newest first
    GET /list/detections  -> same, for ~/agv_detections
    GET /list/soil        -> same, for ~/agv_soil_images (soil_camera_node.py)
    GET /list/maps        -> same, for ~/agv_maps (save_slam_map.sh's .png output;
                              the .pgm/.yaml originals are filtered out, see below)
    GET /img/captures/<name>    -> the image bytes (jpg/jpeg/png)
    GET /img/detections/<name>  -> the image bytes
    GET /img/soil/<name>        -> the image bytes
    GET /img/maps/<name>        -> the image bytes

NO colcon build. Run:
    python3 ~/gallery_server.py
"""

import os
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8080
MAX_LIST = 200                  # newest N images per folder (avoid huge payloads)
ALLOWED_EXT = (".jpg", ".jpeg", ".png")

DIRS = {
    "captures": os.path.expanduser("~/agv_captures"),
    "detections": os.path.expanduser("~/agv_detections"),
    "soil": os.path.expanduser("~/agv_soil_images"),
    # save_slam_map.sh writes <name>.pgm + <name>.yaml + <name>.png here;
    # ALLOWED_EXT below means only the .png shows up in listings -- the raw
    # .pgm/.yaml (ROS2's own map format, not web-viewable) are just ignored,
    # not deleted or touched.
    "maps": os.path.expanduser("~/agv_maps"),
}

CONTENT_TYPES = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png"}


def _list_dir(path):
    if not os.path.isdir(path):
        return []
    entries = []
    for name in os.listdir(path):
        if not name.lower().endswith(ALLOWED_EXT):
            continue
        full = os.path.join(path, name)
        try:
            st = os.stat(full)
        except OSError:
            continue
        entries.append({"name": name, "size": st.st_size, "mtime": st.st_mtime})
    entries.sort(key=lambda e: e["mtime"], reverse=True)
    return entries[:MAX_LIST]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass   # keep journalctl quiet; failures still come back as HTTP status codes

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parts = [p for p in self.path.split("?")[0].split("/") if p]

        if len(parts) == 2 and parts[0] == "list":
            kind = parts[1]
            if kind not in DIRS:
                self._send_json({"error": "unknown kind"}, 404)
                return
            self._send_json(_list_dir(DIRS[kind]))
            return

        if len(parts) == 3 and parts[0] == "img":
            kind, name = parts[1], parts[2]
            if kind not in DIRS:
                self._send_json({"error": "unknown kind"}, 404)
                return
            # path-traversal guard: basename must match the original request
            safe_name = os.path.basename(name)
            if safe_name != name or not safe_name.lower().endswith(ALLOWED_EXT):
                self._send_json({"error": "bad filename"}, 400)
                return
            real_dir = os.path.realpath(DIRS[kind])
            real_full = os.path.realpath(os.path.join(real_dir, safe_name))
            if (not real_full.startswith(real_dir + os.sep)
                    or not os.path.isfile(real_full)):
                self._send_json({"error": "not found"}, 404)
                return
            ext = os.path.splitext(safe_name)[1].lower()
            with open(real_full, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type",
                              CONTENT_TYPES.get(ext, "application/octet-stream"))
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        self._send_json({"error": "not found"}, 404)


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Gallery server ready on :{PORT} "
          f"(captures + detections, read-only)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
