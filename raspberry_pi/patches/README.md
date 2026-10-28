# Patches to third-party packages

## `sllidar_ros2_retry_and_respawn.patch`

For Slamtec's lidar driver, `~/ros2_ws/src/sllidar_ros2` (a git clone of
https://github.com/Slamtec/sllidar_ros2, last upstream commit `3430009`).

**Why:** after a quick restart the lidar can answer the scan-mode query
with nothing (`scan mode 'Standard' is not supported by lidar, supported
modes:` with an empty list, then `Can not start scan: 80008001`). The stock
driver then keeps running with the lidar stopped and never retries, so the
robot has no lidar until someone restarts it by hand (2026-10-28).

**What it changes:**
- `src/sllidar_node.cpp`: tries to start the scan up to 5 times, stopping
  the lidar and waiting 1 s between tries. If it still won't start, it exits.
- `launch/sllidar_c1_launch.py`: `respawn=True`, `respawn_delay=3.0`, so a
  driver that exits is started again.

**Reapply** (fresh Pi, or after updating the driver):

```bash
cd ~/ros2_ws/src/sllidar_ros2
git apply ~/patches/sllidar_ros2_retry_and_respawn.patch   # wherever this file is
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
cd ~/ros2_ws && colcon build --packages-select sllidar_ros2
```
