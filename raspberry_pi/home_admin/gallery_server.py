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
                             (newest MAX_LIST by default; ?limit=N for more,
                             which the app's map screen uses so older
                             geotagged photos still appear on the field map)
    GET /list/detections  -> same, for ~/agv_detections
    GET /list/soil        -> same, for ~/agv_soil_images (soil_camera_node.py)
    DELETE /img/<kind>/<name>
                          -> delete one image. The ONLY write this server
                             does; see _resolve() for the guards, which are
                             the same ones GET uses.
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
from urllib.parse import urlparse, parse_qs

PORT = 8080
MAX_LIST = 200                  # default: newest N images per folder
# The map screen needs EVERY geotagged photo, not just the newest 200, or
# older runs silently vanish from the field map. It asks for a bigger slice
# with /list/<kind>?limit=N. Capped so a malformed request can't ask the Pi
# to serialize an unbounded directory listing.
MAX_LIST_CAP = 5000
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


def _list_dir(path, limit=MAX_LIST):
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
    return entries[:limit]


class Handler(BaseHTTPRequestHandler):
    def _resolve(self, kind, name):
        """Validate kind+name and return the real path, or None after
        sending the error.

        Shared by GET and DELETE on purpose: a delete that validated its path
        even slightly differently from the read path is exactly how a
        traversal bug gets in. Three checks, all of which must pass:
          1. the kind is one we serve
          2. the name is a BARE basename with an allowed extension -- no
             slashes, no "..", nothing that could climb out
          3. the resolved real path is still inside the real directory, and
             is a regular file
        """
        if kind not in DIRS:
            self._send_json({"error": "unknown kind"}, 404)
            return None
        safe_name = os.path.basename(name)
        if safe_name != name or not safe_name.lower().endswith(ALLOWED_EXT):
            self._send_json({"error": "bad filename"}, 400)
            return None
        real_dir = os.path.realpath(DIRS[kind])
        real_full = os.path.realpath(os.path.join(real_dir, safe_name))
        if (not real_full.startswith(real_dir + os.sep)
                or not os.path.isfile(real_full)):
            self._send_json({"error": "not found"}, 404)
            return None
        return real_full

    def do_DELETE(self):
        parts = [p for p in self.path.split("?")[0].split("/") if p]
        if len(parts) != 3 or parts[0] != "img":
            self._send_json({"error": "not found"}, 404)
            return
        kind, name = parts[1], parts[2]
        real_full = self._resolve(kind, name)
        if real_full is None:
            return
        try:
            os.remove(real_full)
        except OSError as e:
            self._send_json({"error": f"delete failed: {e}"}, 500)
            return
        # A front-camera detection has a raw twin under a different kind;
        # deleting only the annotated copy would leave the gallery showing
        # the same moment in the CAPTURES tab, which is not what the person
        # asked for when they deleted the obstacle photo.
        also = []
        twin_kind = {"detections": "captures", "captures": "detections"}.get(kind)
        if twin_kind:
            base = os.path.basename(real_full)
            twin = (base.replace("_detected.", ".") if kind == "detections"
                    else base.replace(".", "_detected.", 1))
            twin_path = self._resolve_quiet(twin_kind, twin)
            if twin_path:
                try:
                    os.remove(twin_path)
                    also.append(f"{twin_kind}/{twin}")
                except OSError:
                    pass
        self._send_json({"deleted": f"{kind}/{os.path.basename(real_full)}",
                         "also_deleted": also})

    def _resolve_quiet(self, kind, name):
        """_resolve() without sending an error response -- for the twin
        lookup, where "not there" is a normal outcome."""
        if kind not in DIRS:
            return None
        safe_name = os.path.basename(name)
        if safe_name != name or not safe_name.lower().endswith(ALLOWED_EXT):
            return None
        real_dir = os.path.realpath(DIRS[kind])
        real_full = os.path.realpath(os.path.join(real_dir, safe_name))
        if (not real_full.startswith(real_dir + os.sep)
                or not os.path.isfile(real_full)):
            return None
        return real_full

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
        query = parse_qs(urlparse(self.path).query)

        if len(parts) == 2 and parts[0] == "list":
            kind = parts[1]
            if kind not in DIRS:
                self._send_json({"error": "unknown kind"}, 404)
                return
            # ?limit=N -- opt-in bigger listing (see MAX_LIST_CAP). Anything
            # unparseable falls back to the default rather than erroring.
            limit = MAX_LIST
            try:
                limit = max(1, min(MAX_LIST_CAP, int(query.get("limit", [MAX_LIST])[0])))
            except (ValueError, TypeError):
                pass
            self._send_json(_list_dir(DIRS[kind], limit))
            return

        if len(parts) == 3 and parts[0] == "img":
            kind, name = parts[1], parts[2]
            real_full = self._resolve(kind, name)
            if real_full is None:
                return
            safe_name = os.path.basename(real_full)
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
