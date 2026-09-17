#!/usr/bin/env python3
"""
draw_slam_path.py  --  PGM -> PNG for the app gallery, with the robot's
traversed path drawn on top (and its GPS data, when a fix was available).

Called by ~/save_slam_map.sh. Usage:
    python3 ~/draw_slam_path.py <base>        # <base> = path WITHOUT extension

Reads   <base>.pgm + <base>.yaml   (the standard ROS map, left UNTOUCHED)
Writes  <base>.png                 (the app-facing image -- overlay goes here
                                    only, so the .pgm stays clean for nav2)

WHY THE TRACK IS IN MAP COORDS, NOT GPS:
  The map image's pixels are defined by the .yaml's `origin` (world coords of
  the BOTTOM-LEFT pixel) and `resolution` (m/px), in the SLAM "map" frame --
  metres from wherever slam_toolbox happened to start. There is NO
  georeferencing between that frame and lat/lon, so GPS coordinates cannot be
  plotted directly onto this image. path_recorder_node.py therefore logs the
  robot's MAP-frame position from /pose alongside the GPS reading at that same
  moment; we draw using the former and annotate using the latter.

SESSION SAFETY:
  Restarting slam_toolbox (e.g. the app's RESET MAPPED AREA button) starts a
  BRAND NEW map frame -- older track points are in a different coordinate
  system and would be drawn in the wrong place. path_recorder runs under
  agv.service and does NOT restart with SLAM, so its track file can straddle
  both. We therefore drop every point older than the CURRENT slam_toolbox
  process start time. Deterministic; no guessing from the data.
"""

import os
import sys
import csv
import glob
import time
import subprocess

import cv2
import numpy as np

PATHS_DIR   = os.path.expanduser("~/agv_paths")
TARGET_PX   = 800      # upscale the (tiny) map so the path + text are legible
MAX_SCALE   = 12
LINE_BGR    = (60, 200, 90)     # traversed path
START_BGR   = (80, 220, 80)
END_BGR     = (60, 60, 235)
BAR_H       = 34       # info bar height at the bottom, in upscaled px


def slam_start_epoch():
    """Epoch seconds when the running slam_toolbox started, or None."""
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cmd = f.read().decode("utf-8", "ignore")
        except OSError:
            continue
        if "async_slam_toolbox_node" not in cmd:
            continue
        try:
            out = subprocess.run(["ps", "-o", "etimes=", "-p", d],
                                 capture_output=True, text=True, timeout=5)
            return time.time() - int(out.stdout.strip())
        except Exception:
            return None
    return None


def read_yaml(path):
    """Minimal reader -- only the two keys we need, no yaml dependency."""
    res, origin = None, None
    with open(path) as f:
        for line in f:
            if line.startswith("resolution:"):
                res = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                nums = line.split("[", 1)[1].split("]", 1)[0].split(",")
                origin = (float(nums[0]), float(nums[1]))
    return res, origin


def load_track(since_epoch):
    """Newest slam_track_*.csv, filtered to the current SLAM session."""
    files = sorted(glob.glob(os.path.join(PATHS_DIR, "slam_track_*.csv")))
    if not files:
        return []
    pts = []
    with open(files[-1]) as f:
        for row in csv.DictReader(f):
            try:
                epoch = float(row["epoch"])
            except (ValueError, KeyError):
                continue
            if since_epoch is not None and epoch < since_epoch:
                continue     # belongs to a previous SLAM map frame
            try:
                x, y = float(row["map_x"]), float(row["map_y"])
            except (ValueError, KeyError):
                continue
            lat = row.get("lat") or ""
            lon = row.get("lon") or ""
            pts.append((x, y, lat.strip(), lon.strip()))
    return pts


def main():
    if len(sys.argv) < 2:
        print("usage: draw_slam_path.py <base-path-without-extension>",
              file=sys.stderr)
        return 1
    base = sys.argv[1]
    img = cv2.imread(base + ".pgm", cv2.IMREAD_UNCHANGED)
    if img is None:
        print("ERROR: could not read " + base + ".pgm", file=sys.stderr)
        return 1
    res, origin = read_yaml(base + ".yaml")
    h, w = img.shape[:2]

    scale = max(1, min(MAX_SCALE, TARGET_PX // max(w, h)))
    canvas = cv2.cvtColor(
        cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST),
        cv2.COLOR_GRAY2BGR)

    pts = load_track(slam_start_epoch()) if (res and origin) else []

    # map-frame metres -> pixel. origin is the world coord of the BOTTOM-LEFT
    # pixel, and image row 0 is the TOP, hence the y flip.
    px = []
    for x, y, lat, lon in pts:
        c = (x - origin[0]) / res
        r = h - 1 - (y - origin[1]) / res
        if -w <= c <= 2 * w and -h <= r <= 2 * h:      # ignore wild outliers
            px.append((int(c * scale), int(r * scale), lat, lon))

    for i in range(1, len(px)):
        cv2.line(canvas, px[i - 1][:2], px[i][:2], LINE_BGR,
                 max(1, scale // 3), cv2.LINE_AA)
    if px:
        cv2.circle(canvas, px[0][:2], max(2, scale), START_BGR, -1, cv2.LINE_AA)
        cv2.circle(canvas, px[-1][:2], max(2, scale), END_BGR, -1, cv2.LINE_AA)

    # ---- info bar: GPS for the path, when any fix was recorded ----
    fixes = [(la, lo) for _, _, la, lo in pts if la and lo]
    if not px:
        info = "no path recorded this SLAM session"
    elif fixes:
        info = (f"path {len(px)} pts | start {fixes[0][0]},{fixes[0][1]}"
                f" | end {fixes[-1][0]},{fixes[-1][1]}")
    else:
        info = f"path {len(px)} pts | GPS: no fix recorded"

    bar = np.zeros((BAR_H, canvas.shape[1], 3), dtype=np.uint8)
    cv2.putText(bar, info, (8, BAR_H - 11), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (255, 255, 255), 1, cv2.LINE_AA)
    out = np.vstack([canvas, bar])

    if not cv2.imwrite(base + ".png", out):
        print("ERROR: could not write " + base + ".png", file=sys.stderr)
        return 1
    print(f"  path points drawn: {len(px)}"
          f"{' (with GPS)' if fixes else ' (no GPS fix)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
