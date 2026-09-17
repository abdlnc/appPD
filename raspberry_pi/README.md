# AGV — Raspberry Pi Source Code

**Project:** Deep Learning-Based Path Obstruction Detection for Agricultural AGVs
with Field Mapping System

This folder holds the **Raspberry Pi–side** source code of the AGV system — the
robot's ROS2 nodes, the standalone helper programs, and the systemd units that
start it all on boot. The rest of this repository is the **Flutter mobile app**
that talks to it over a WebSocket (port 8765) and pulls saved images over HTTP
(port 8080).

Nothing here is built or bundled by the Flutter app — this folder is source
kept alongside the app so the two halves of the system stay in one place and
in sync.

---

## WHERE EACH FILE LIVES ON THE PI

| Folder here | Location on the Raspberry Pi |
|---|---|
| `home_admin/` | `/home/admin/` |
| `ros2_ws/src/agv_control/` | `/home/admin/ros2_ws/src/agv_control/` |
| `systemd/agv.service` | `/etc/systemd/system/agv.service` |
| `systemd/agv-slam.service` | `/etc/systemd/system/agv-slam.service` |

## WHAT EACH FILE DOES

**`ros2_ws/src/agv_control/` — ROS2 package (needs `colcon build` after any edit):**

- `agv_control/motor_node.py` — The ONLY file that controls the motors and
  steering pins. Has a **speed cap** (`SPEED_SCALE = 0.60`) and a **reverse
  guard** (`REVERSE_PAUSE = 0.4s`: the robot fully stops first before going
  backward, to protect the rear gears). Also has a watchdog: if commands stop
  arriving, the motors stop on their own.
- `agv_control/control_node.py` — AUTO mode: reads the Lidar and steers the
  robot around obstacles. Also detects **no possible path** (forward, reverse
  and both turn directions all blocked — the robot boxed in with nothing
  `avoid()` can do about it) and flags it to the app by appending a 4th field
  to the obstacle message: `DANGER|zone|dist|BLOCKED`, **and stops the robot** —
  it holds `/cmd_vel` at zero instead of letting `avoid()` keep shuffling
  against the obstacles. The hold is enforced in `tick()` (the single place
  this node publishes), so an already-running `avoid()` maneuver can't drive
  through it. It engages on one blocked scan but needs `BLOCKED_RELEASE_SCANS`
  (5, ~0.5s) consecutive clear scans to release, so Lidar noise on the
  threshold can't cause start-stop lurching. **AUTO mode only** — MANUAL is
  untouched, so a human can always drive the robot back out. (NAV mode already
  had its own boxed-in stop in `nav_node.py`.)
- `agv_control/bridge_node.py` — The link between the robot and the mobile app
  (WebSocket, port 8765). Handles manual driving, mode changes, waypoints, and
  sends the camera / obstacle / GPS data to the app. Also relays the AI's
  detection boxes as `B:` messages for the app's live-view overlay, and accepts
  the `SAVEMAP` / `RESETMAP` commands behind the app's two map buttons. If the
  app disconnects, the robot stops as a safety measure.
- `agv_control/camera_node.py` — Runs the front USB webcam. Publishes TWO
  streams and saves nothing itself: `/image_raw/compressed` (downscaled, JPEG
  50) for the app's live view, and `/camera/full/compressed` (full-res, JPEG
  92, at `AI_FEED_HZ`) for the AI detector. Two topics on purpose — the app's
  view is deliberately lossy for bandwidth, which is fine for a human eye but
  needlessly degrades what the model reads. Every frame is rotated 180° before
  either stream is built (`FLIP_UPSIDE_DOWN = True`) — the camera is physically
  mounted upside down on this robot. One flip at the single frame-read point,
  so the app view, the AI feed, and the images `ai_detector_node` saves are all
  correctly oriented from the same source. Set `FLIP_UPSIDE_DOWN = False` if
  the camera is ever remounted right-side up.
- `launch/agv_full.launch.py` — One command that starts the whole core system
  (Lidar + the 4 nodes above).
- `package.xml`, `setup.py`, `setup.cfg`, `resource/agv_control` — ROS2/ament
  package metadata. Boring, but `colcon build` fails without them.

**`home_admin/` — standalone programs (no build needed):**

- `gps_node.py` — Reads the real GPS module. While still searching for a
  signal, it reports how many satellites it can see (so the app is never
  blank). Also has a `--sim` mode (fake GPS for indoor logic demos — clearly
  labeled). Real GPS needs open sky, so it works outdoors.
- `nav_node.py` — NAV mode: drives the robot to a GPS waypoint, with the Lidar
  still watching for obstacles (including its own boxed-in stop).
  `CRUISE_SPEED` is 0.80; AUTO mode's equivalent is `control_node.py`'s
  `FWD_SPEED` at 0.60. Both leave their turn speeds alone. Faster driving means
  less room to react at the unchanged 0.60m stop thresholds — lower them again,
  or raise the thresholds, if the robot starts clipping obstacles.
- `path_recorder_node.py` — Saves the route the robot actually traveled into
  `~/agv_paths/` (`.csv` and `.geojson`) — this is the field-map trail. Also
  writes `slam_track_<time>.csv` (epoch, map_x, map_y, lat, lon): the robot's
  position in the SLAM **map frame** paired with the GPS reading at that same
  moment. That pairing is what lets the path be drawn onto the SLAM map — the
  map's pixels are metres from an arbitrary SLAM origin with NO georeferencing
  to lat/lon, so GPS alone can't be plotted on it. Driven by `/pose`, not GPS,
  so a path is still recorded indoors with no fix (lat/lon columns just stay
  blank).
- `ai_detector_node.py` — Runs the YOLOv8 obstruction-detection model
  (`best.pt`, 24-class: plants/rocks/trees/gardening tools/etc.) on the front
  camera's **live feed**, and **saves a photo only when the model actually sees
  an obstacle** (the CAMERA decides what an obstacle is, not the Lidar — the
  two are deliberately independent; the Lidar still drives avoidance in
  `control_node.py`). On a detection it saves the raw frame to
  `~/agv_captures/` and the annotated one to `~/agv_detections/` under a
  shared, GPS-tagged basename; on zero detections it saves nothing. Every box
  is labeled "Obstacle" regardless of which of the 24 classes it is (the class
  name is discarded, not just hidden). Also publishes `/ai_boxes` — normalised
  box corners for the app's live-view overlay — on **every** inference,
  including zero-detection ones, so the app clears its overlay instead of
  leaving stale boxes up. Has a `--test` mode for checking one photo.
  Three tuning knobs worth knowing, all in the file header: `AI_MIN_INTERVAL`
  (CPU floor between inferences), `SAVE_CONF` (0.50 — how sure the model must
  be before a photo is worth keeping) and `SAVE_COOLDOWN` (15s — stops the same
  obstacle being re-saved as you drive up to it). `SAVE_CONF` matters: ordinary
  scenery produces a steady trickle of 0.25–0.42 "obstacles", which at
  `CONF_THRES` alone saved ~1 image every 7s and buried the gallery. Set
  `SAVE_CONF = CONF_THRES` to save every detection.
- `soil_camera_node.py` — Owns the SECOND USB camera (`/dev/video2`, mounted
  facing down at the soil), separate from `camera_node.py`'s front camera.
  Saves one snapshot every 60 seconds into `~/agv_soil_images/`. No live feed,
  no AI — just a periodic photo log. Each image gets the capture time and GPS
  **burned into the picture** (bottom-left, white-on-black outline) as well as
  in the filename — safe to draw on these because nothing machine-reads them,
  unlike the front camera's AI input.
- `gallery_server.py` — Read-only HTTP file server (port 8080) so the saved
  images in `~/agv_captures/`, `~/agv_detections/`, `~/agv_soil_images/` and
  `~/agv_maps/` can be listed and viewed from the Flutter app, or any browser
  on the network. Independent of the ROS2/GPIO/camera stack.
- `draw_slam_path.py` — Does the `.pgm` → `.png` conversion for the gallery AND
  draws the robot's traversed path on top, annotated with its GPS coordinates
  when a fix was recorded. Also upscales (~800px) so the path and text are
  legible — the raw maps are only ~70px across. Drops track points older than
  the running `slam_toolbox` process: restarting SLAM starts a brand-new map
  frame, so older points are in a different coordinate system and would be
  drawn in the wrong place. Overlay goes only into the `.png`; the
  `.pgm`/`.yaml` stay clean for nav2.
- `save_slam_map.sh` — Saves the live SLAM map via the standard ROS2
  `map_saver_cli` (`.pgm` + `.yaml`) and calls `draw_slam_path.py` for the
  `.png`, so it shows up in the app's gallery MAPS tab with the path drawn on.
  Run by hand (see `TERMINAL_COMMANDS.md` section 2), or remotely by the app's
  SAVE SLAM MAP SNAPSHOT button (via `bridge_node.py`'s `SAVEMAP` command).
  Retries automatically if run too soon after a fresh restart.
- `agv_slam/` — SLAM (`slam_toolbox`) config + launch file. **Runs
  automatically from boot**, but as its OWN service (`agv-slam.service`),
  deliberately NOT bundled into `agv_autostart.launch.py`. Reason: this
  slam_toolbox build has no "reset map" service, so the only way to clear the
  map is restarting the node — keeping it separate lets the app's RESET MAPPED
  AREA button restart just SLAM without dropping the app's WebSocket
  connection, camera or motors. Doesn't touch the Lidar hardware itself, just
  subscribes to the `/scan` topic `agv_full`'s Lidar driver already publishes,
  so there's no port conflict. **`mapper.yaml` carries important tuning notes —
  read them before changing anything there** (this system has no odometry,
  which makes SLAM fragile in specific, documented ways).
- `agv_autostart.launch.py` — The master launch used by the auto-start (core
  system + the standalone programs above; NOT SLAM). GPS mode is controlled by
  a flag file `~/agv_use_sim` (no file = REAL GPS, default).
- `start_agv.sh` — Small helper that loads the ROS2 environment before
  launching everything.
- `start_slam.sh` — Same idea, but for `agv-slam.service` (SLAM only).
- `install_autostart.sh` — One-time installer for the auto-start service.
- `steering_test.py` — Small standalone hardware check for the steering servo.

**`systemd/agv.service`** — Makes everything start by itself when the Pi is
turned on.

**`systemd/agv-slam.service`** — Runs SLAM mapping, separately, so it can be
restarted (= map wiped) on its own. Handy commands:

```bash
sudo systemctl status agv-slam.service          # is mapping running?
sudo systemctl restart agv-slam.service         # clear the map / restart
sudo systemctl disable --now agv-slam.service   # turn mapping off entirely
```

> **Note:** `agv-slam.service` must NOT declare `After=agv.service`. `agv.service`
> already orders itself after `multi-user.target`, so adding that creates an
> ordering cycle — and systemd resolves a cycle by **silently deleting the start
> job**. The unit stays "enabled", nothing is logged, and SLAM simply never
> starts at boot. `Wants=` alone is correct and is what's committed here.

## NOT IN THIS FOLDER (on purpose)

- `agv_models/` — the AI models (`best.pt`, and the older unused
  `rtdetr_best.pt`). Too large for git and owned separately; they live at
  `~/agv_models/` on the Pi.
- Generated data folders — `agv_captures/`, `agv_detections/`, `agv_paths/`,
  `agv_maps/`, `agv_soil_images/`. All runtime output.
- `motor_test.py` and RViz settings (`~/.rviz2/`) — machine-local scratch.
- Editor/build leftovers: `__pycache__/`, `*.bak` node backups, and the
  default ament linter boilerplate in `test/`.

## HOW TO DEPLOY ONTO A PI

1. Copy each folder to its location using the table above.
2. Build the ROS2 package (only needed for the `ros2_ws` files):
   ```bash
   mamba activate ros2_humble
   source ~/ros2_ws/install/setup.bash
   cd ~/ros2_ws && colcon build --packages-select agv_control
   ```
3. Install the auto-start:
   ```bash
   sudo bash ~/install_autostart.sh
   sudo systemctl enable --now agv.service
   ```
4. Install the SLAM service too:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now agv-slam.service
   ```
5. Drop the AI model in place: `~/agv_models/best.pt`.

To snapshot a complete copy off a running Pi (including everything excluded
above), one command does it:

```bash
tar -czf agv_backup.tar.gz ~/ros2_ws/src ~/agv_slam ~/*.py ~/*.sh \
    ~/agv_autostart.launch.py \
    /etc/systemd/system/agv.service /etc/systemd/system/agv-slam.service
```

See `TERMINAL_COMMANDS.md` for the full command reference — daily use,
SLAM/RViz mapping, AI test, and troubleshooting.
