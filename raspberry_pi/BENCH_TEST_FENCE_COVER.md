# Bench test: field boundary, COVER mode, duplicate photos

Every step works with the app **already on the phone**. A rebuilt app adds
the boundary editor, START COVER and the lane drawing on the map, but none of
this needs it.

> **SAFETY — read first.** `--sim` simulates only the **GPS**. `motor_node`
> still drives the **real wheels** from every command. For steps 4 onwards,
> put the robot **on a stand with the wheels off the ground**.

All commands assume:

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
source ~/ros2_ws/install/setup.bash
```

## 1. Tests that need no robot (run these first)

```bash
python3 ~/field_geometry_test.py        # boundary, lanes, coverage, dedup maths
python3 ~/coverage_planner_test.py      # driving back to the start
python3 ~/fence_enforcement_test.py     # what motor_node lets through at the fence
python3 ~/gap_seeking_test.py           # AUTO unchanged with no boundary set
python3 ~/turn_direction_test.py
python3 ~/scan_filter_test.py
```

Each ends `ALL PASS`.

## 2. See the plan for your field, no robot

Fill in your measurements (all currently TODO placeholders in
`coverage_planner.py`):

```bash
python3 ~/coverage_sim.py --plan-only --field 100 50 --speed 0.55 --turn 3.0
```

This prints the lane count, route length, run time, and soil-photo spacing
(speed × 60 s).

## 3. Measure your GPS IN THE FIELD (most important)

Park the robot where it will work, service running:

```bash
python3 ~/gps_drift.py --minutes 10
```

At the house on 2026-10-19 the position wandered a median **3.1 m in 20 s**
and **9.9 m** at worst while parked. At that level COVER cannot hold its
lanes and a 3 m stop line cannot hold the robot. If the field is much better
(under ~1 m in 20 s), both should work. Simulate your own numbers:

```bash
python3 ~/coverage_sim.py --noise <bigger sigma from gps_drift> --tau 120 --seeds 10
```

## 4. Switch to simulated GPS (wheels off the ground)

```bash
touch ~/agv_use_sim ~/agv_sim_ackermann     # steers like the real robot
touch ~/agv_sim_noise                       # optional: drifting GPS error
sudo systemctl restart agv.service
journalctl -u agv.service -f | grep -i sim  # "SIMULATION ... Ackermann steering"
```

## 5. Set a boundary around the simulated robot

From the existing app, without a rebuild:

```bash
touch ~/agv_fence_edit
```

Open the GPS map and tap 4 corners around the robot. Each tap logs
`BOUNDARY CORNER n ADDED`; the robot is held in STOP. "Clear waypoint" removes
the last corner. Then:

```bash
rm ~/agv_fence_edit
```

Or set a 60 × 40 m field around the sim start point directly:

```bash
ros2 topic pub --once /fence_cmd std_msgs/String \
  "{data: 'SET 3.0|14.7480439,121.0615747;14.7480439,121.0621327;14.7484036,121.0621327;14.7484036,121.0615747'}"
```

Check it:

```bash
ros2 topic echo /fence_state     # IN|<m inside the stop line>|<turn>
ros2 topic echo /fence --once    # "active":true, the stop line
```

## 6. Run COVER

```bash
touch ~/agv_coverage          # START AUTO now runs COVER (needs an active boundary)
```

Press **START AUTO** in the app. In the journal:

```
*** ~/agv_coverage present: START AUTO -> COVER mode ***
COVER plan: 5 lanes 6.0 m apart, route 249.0 m ...
```

Watch progress:

```bash
ros2 topic echo /cover_status    # {"phase":"LANE","lane":2,"lanes":5,"coverage":31.5,...}
ros2 topic echo /cover_route     # "<steer>,<cap>" or "HOLD|<why>"
```

### 6a. The drive back to the start

When the last lane and the fill-in pass are done, the journal shows
`COVER: field covered -- driving back to the start`, `/cover_status` goes
`"phase":"RETURN"` with `"home_m"` counting down, and the app card reads
**Field done -- returning to start (N m to go)**. It ends with one of:

- `COVER: coverage complete, back at the start`: it got there and holds.
- `COVER: coverage complete, but could not get back to the start -- stopped N m short`:
  out of time (blocked, or the boundary in the way). It holds where it is.

"The start" is the first GPS fix after START. If that was within 4 m of
the stop line, it returns to the nearest point 4 m inside instead. It
gets as close as the GPS allows: within 2.5 m of where the GPS says the
start is.

To see it without driving a whole field, simulate it:

```bash
python3 ~/coverage_planner_test.py     # the return rules, ALL PASS
python3 ~/coverage_sim.py --noise 0    # ends "back at the start"
```

To switch it off (stop wherever coverage ends, as before), set
`RETURN_TO_START = False` in `coverage_planner.py` and run colcon build.

## 7. Resume after a stop

Press **EMERGENCY STOP**, then START AUTO again. The journal shows
`COVER progress restored: lane N/...`. To start over:

```bash
ros2 topic pub --once /cover_cmd std_msgs/String "{data: RESET}"
```

## 8. The fence

- **Stop line.** Drive outward with the dev joystick (MANUAL is not fenced),
  then START AUTO. The journal shows `FIELD BOUNDARY -- stop line reached`,
  and the robot backs in, then turns in.
- **No fix → stop.** With a boundary active:

  ```bash
  pkill -f gps_node.py
  ```

  `/fence_state` goes `NOFIX`, motor_node logs
  `FENCE: no GPS fix -- cannot prove it is inside`, and the wheels stop.
  Restart the service afterwards.
- **Fence node dies → stop.**

  ```bash
  pkill -f fence_node.py
  ```

  motor_node logs `fence node silent with a boundary saved` and stops.

## 9. Duplicate photos

In a recorded run, leave the robot parked for 3+ minutes. Soil photos after
the first are named `..._dup.jpg`, and they're still saved. Detections of the
same thing log `detection NOT saved: ... at least as confident` or
`better shot ... replacing ...`.

## 10. Put everything back

```bash
rm -f ~/agv_use_sim ~/agv_sim_ackermann ~/agv_sim_noise ~/agv_coverage ~/agv_fence_edit
ros2 topic pub --once /fence_cmd std_msgs/String "{data: CLEAR}"   # or: rm ~/agv_boundary.json
rm -f ~/agv_coverage_state.json
sudo systemctl restart agv.service
```

**Before field use, make sure `~/agv_use_sim` is gone.**
