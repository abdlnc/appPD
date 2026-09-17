# AGV — Complete Terminal Commands Guide (B.I.T.S.)

Every command you need to run, test, and maintain the AGV system on the
Raspberry Pi. (Login user: `admin`.)

---

## 0. GOLDEN RULE — in every NEW terminal, load the environment first

```
mamba activate ros2_humble
source ~/ros2_ws/install/setup.bash
```

If you skip this, Python will complain with `ModuleNotFoundError: rclpy /
ultralytics`. (You do NOT need this for `systemctl` / `journalctl` commands.)

---

## 1. NORMAL USE — nothing to type

The whole system **starts by itself** when the Pi is turned on. Just wait
about 30–60 seconds after boot, open the mobile app, and connect to the Pi's
IP address. That's it.

### Service control (when you need it)
| What you want | Command |
|---|---|
| Check if it's running | `systemctl status agv.service` |
| Watch live logs | `journalctl -u agv.service -f` |
| Start it manually | `sudo systemctl start agv.service` |
| Stop it (before manual tests / SLAM) | `sudo systemctl stop agv.service` |
| Restart it (after a code change) | `sudo systemctl restart agv.service` |
| Turn OFF auto-start on boot | `sudo systemctl disable agv.service` |
| Turn auto-start back ON | `sudo systemctl enable agv.service` |
| Safely shut down the Pi | `sudo shutdown -h now` |

**⚠ IMPORTANT:** the service holds the Lidar, the motor pins, and the GPS
port. Before hardware tests (`steering_test.py`, `motor_test.py`) or any
manual program that touches those, stop it first:
`sudo systemctl stop agv.service`. If you don't, you will get `device busy`
or pin-conflict errors.

**SLAM is the exception** — it does NOT need agv.service stopped (it only
subscribes to `/scan`, it never opens the Lidar itself). See section 2.

---

## 2. SLAM LIVE MAPPING (RViz2) — the map that builds while the robot moves

**SLAM runs automatically from boot** as its own service,
`agv-slam.service` — you don't need to launch it by hand. It doesn't touch
the Lidar hardware directly (just subscribes to the `/scan` topic
`agv.service`'s Lidar driver already publishes, plus its own TF frames),
so there's no port conflict and the app connection is never affected.

It's a SEPARATE service from `agv.service` on purpose: this slam_toolbox
build has no "reset map" service, so wiping the map means restarting the
node — and keeping it separate means that restart doesn't take the app's
WebSocket connection, camera and motors down with it.

```
sudo systemctl status agv-slam.service      # is mapping running?
sudo systemctl restart agv-slam.service     # WIPE the map, start over
ros2 topic hz /map                          # should be publishing
ros2 topic hz /pose                         # should be ~2Hz; nothing = broken
rviz2                                       # optional live view
#   Inside RViz2:  set Fixed Frame = map
#   Add > Map        (topic: /map)
#   Add > LaserScan  (topic: /scan)
```

The app's **RESET MAPPED AREA** button (Lidar Map screen) runs that same
restart remotely, via `bridge_node.py`'s `RESETMAP` command. It only clears
the LIVE map — snapshots already saved in `~/agv_maps/` are left alone.

**If SAVEMAP suddenly fails with "is SLAM running?" after a reboot**, check
`systemctl is-active agv-slam.service` first. If it's `inactive` but
`enabled`, and `journalctl -u agv-slam.service -b` shows *nothing at all*,
look for this in the full boot log:
```
sudo journalctl -b | grep -i "ordering cycle"
```
`agv.service` declares `After=multi-user.target`, so adding
`After=agv.service` to `agv-slam.service` creates a cycle
(multi-user.target → wants agv-slam → after agv.service → after
multi-user.target). systemd breaks it by silently DELETING agv-slam's start
job — enabled, no error, just never starts. Do NOT add `After=agv.service`
back; `Wants=` alone is correct (ROS2 doesn't care about start order).

While the **robot moves, the map grows** — black = walls/obstacles,
gray = open space. (Slower movement = cleaner map.)

**No odometry exists in this system** (no wheel encoders, `odom->base_link`
is a permanently static transform — see `slam.launch.py`), so SLAM relies
entirely on scan-matching to notice motion at all. `agv_slam/mapper.yaml`
has comments explaining the tuning this needed — two failure modes, pulling
in opposite directions:
  * **Map frozen, never updates** (fixed by `minimum_travel_distance`/
    `minimum_travel_heading` = 0 -- otherwise it gates re-processing on the
    odom frame's motion, which never changes, so it never even tries after
    the first scan). If `ros2 topic hz /pose` shows nothing publishing at
    all, this gating is broken again -- check those two params first.
  * **Map fills with noise / exploding extent** (a wide search space +
    matching every scan with zero odometry = easy to snap onto a
    plausible-but-wrong match, then compound from there -- confirmed: saw
    a map balloon from ~7.7x5.5m to ~19.4x14.3m of pure noise in one
    session). Current settings (`correlation_search_space_dimension: 1.2`,
    `minimum_time_interval: 0.5`, `ceres_loss_function: HuberLoss`) are a
    middle ground, not a guarantee -- if this recurs, check the last few
    `~/agv_maps/*.png` for a "starburst" of radiating streaks (that's the
    signature) and consider narrowing the search space further, or slow
    down how fast/jerky the Lidar is being moved during testing.

### Save the map (any terminal, environment loaded — or just tap **SAVE
### SLAM MAP SNAPSHOT** on the app's Lidar Map screen, which does the same
### thing remotely over the WebSocket):
```
~/save_slam_map.sh              # name = map_YYYYMMDD_HHMMSS
~/save_slam_map.sh field_map    # or give it your own name
```
Output: `~/agv_maps/<name>.pgm` + `<name>.yaml` (the standard ROS2 map
format, unchanged) **and** `<name>.png` (auto-converted, so the app can
display it -- browsers/Flutter can't render raw `.pgm`). The old two-line
`map_saver_cli` command still works by hand if you only want the ROS2
files and don't need the PNG. The script retries automatically (up to 3x)
if `map_saver_cli` hits the transient "Failed to spin map subscription"
error that can happen if you save within the first ~30-40s of a fresh
`agv.service` restart/boot (SLAM just needs a moment to settle in).

Open the gallery's **MAPS** tab any time after saving to view/download it
— no restart needed, the app was never disconnected.

**If SLAM is ever a resource concern** (it costs some steady-state CPU/RAM
for its Ceres solver, even with the robot sitting still, on top of
`ai_detector_node`'s YOLOv8 inference), turn mapping off entirely:
```
sudo systemctl disable --now agv-slam.service   # off, and stays off on boot
sudo systemctl enable --now agv-slam.service    # back on
```
Stopping it never affects `agv.service` or the app connection.

---

## 3. AI DETECTION — run the model on one photo

```
# see the newest camera snapshots:
ls -t ~/agv_captures/ | head

# run the model on one (replace the filename with the newest from the list):
python3 ~/ai_detector_node.py --test ~/agv_captures/capture_20260629_235145.jpg
```
You will see `=== INFERENCE OK ===` plus the detections; the marked-up image
is saved in `~/agv_detections/`. (During normal use, this runs automatically
on every new snapshot — nothing to type.)

---

## 4. SAVED IMAGE GALLERY (view captures / AI detections / soil / maps)

Browse the saved photos from any device on the same network -- phone
browser, laptop, or the Pi itself over VNC:

```
http://<pi-ip>:8080/list/captures       # JSON list of raw snapshots
http://<pi-ip>:8080/list/detections     # JSON list of AI-annotated images
http://<pi-ip>:8080/list/soil           # JSON list of soil-camera snapshots
http://<pi-ip>:8080/list/maps           # JSON list of saved SLAM maps (PNGs)
http://<pi-ip>:8080/img/captures/<name>.jpg      # view one image
http://<pi-ip>:8080/img/detections/<name>.jpg
http://<pi-ip>:8080/img/soil/<name>.jpg
http://<pi-ip>:8080/img/maps/<name>.png
```
Runs automatically with the rest of the system (`gallery_server.py`, no
GPU/GPIO access, safe to restart on its own). In the app: the photo-library
icon on the Control screen opens the same gallery with a CAPTURES /
DETECTIONS / SOIL / MAPS tab switch.
- SOIL is fed by `soil_camera_node.py`, which owns the second USB camera
  (`/dev/video2`, facing down at the soil) and saves one snapshot every
  60s into `~/agv_soil_images/` — independent of the front
  camera_node/ai_detector_node pipeline.
- MAPS is fed by `~/save_slam_map.sh` (see section 2) — it only shows the
  `.png` files that script writes into `~/agv_maps/`, not the `.pgm`/`.yaml`
  ROS2 map format sitting alongside them (gallery_server.py only lists
  `.jpg`/`.jpeg`/`.png`).

---

## 5. HARDWARE TESTS (motors + steering)

Lift the robot so the wheels spin freely. Service stopped. Environment loaded.
```
python3 ~/motor_test.py        # wheels: forward, backward, left, right
python3 ~/steering_test.py     # steering: left, center, right
```

---

## 6. GPS CHECKS

Real GPS needs **open sky (outdoors)**. Indoors, it sees very few satellites
and cannot lock — that is normal, not a defect.

```
# read the raw GPS data straight from the module (service stopped):
stty -F /dev/ttyAMA0 9600 raw
timeout 12 cat /dev/ttyAMA0
```
How to read it: in `$G_GSV` lines, the number after the third comma = how
many satellites it can see. `$G_RMC` with `A` = it has a position lock
(`V` = not yet). In the app: while searching, you'll see "N sat(s) in view" —
at 4+ a lock is close; 8–12 is a clean signal.

### GPS Simulation mode (for indoor LOGIC demos only — not the real location)
```
touch ~/agv_use_sim && sudo systemctl restart agv.service    # SIM on
rm -f ~/agv_use_sim && sudo systemctl restart agv.service    # back to REAL
```
In SIM mode the app always shows `FIX 1 · SAT 12`. **Default and production
mode = REAL.**

---

## 7. AFTER EDITING CODE

**Standalone programs** (`~/gps_node.py`, `nav_node.py`, etc.) — just restart:
```
sudo systemctl restart agv.service
```

**Package programs** (`~/ros2_ws/src/agv_control/agv_control/*.py`) — build first:
```
mamba activate ros2_humble
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon build --packages-select agv_control
sudo systemctl restart agv.service
```

### Tuning values inside motor_node.py:
- `SPEED_SCALE = 0.60` — top speed (0.50 = slower; avoid going below 0.40,
  the motors may not have enough power to move)
- `REVERSE_PAUSE = 0.4` — seconds of full stop before reversing (gear guard)

---

## 8. QUICK TROUBLESHOOTING

```
ros2 topic list                          # which topics are alive
ros2 topic hz /scan                      # Lidar (should be ~10 Hz)
ros2 topic hz /image_raw/compressed      # camera (should be ~12 Hz)
ros2 topic echo /gps --once              # current GPS message
journalctl -u agv.service | grep -i error | tail
ls /dev/video*                           # is the camera detected?
ls -l /dev/ttyUSB0 /dev/ttyAMA0          # Lidar + GPS ports
curl -s localhost:8080/list/captures     # is the gallery server serving?
curl -s localhost:8080/list/soil         # is the soil camera saving?
curl -s localhost:8080/list/maps         # are saved SLAM maps showing up?
```

---
*B.I.T.S. — Bacolod Information Technology Solutions*
