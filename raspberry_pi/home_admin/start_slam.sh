#!/usr/bin/env bash
# B.I.T.S. AGV -- SLAM (slam_toolbox) start wrapper
#
# SLAM runs as its OWN systemd unit (agv-slam.service) rather than being
# bundled into agv_autostart.launch.py, specifically so it can be restarted
# on its own WITHOUT dropping the app's WebSocket connection.
#
# Why that matters: slam_toolbox (this build) exposes no "reset map" service
# -- the only way to clear the map is to restart the node. The app's
# "RESET MAPPED AREA" button does exactly that, via
#     sudo systemctl restart agv-slam.service
# (see bridge_node.py's RESETMAP handler). If SLAM were still inside
# agv.service, that reset would also kill bridge_node/camera/motors and
# disconnect the app every time.
#
# Same environment bootstrap as start_agv.sh -- systemd starts with a BARE
# environment, so conda + ROS2 must be set up here. Errors show up in:
#     journalctl -u agv-slam.service -f

# --- 1. locate the conda/mamba install (RoboStack/Miniforge) -----------------
CONDA_BASE=""
for base in "$HOME/miniforge3" "$HOME/mambaforge" "$HOME/miniconda3" "$HOME/anaconda3"; do
    if [ -f "$base/etc/profile.d/conda.sh" ]; then
        CONDA_BASE="$base"
        break
    fi
done
if [ -z "$CONDA_BASE" ]; then
    echo "ERROR: conda/mamba base not found under \$HOME ($HOME)." >&2
    exit 1
fi

# --- 2. activate the ros2_humble env -----------------------------------------
source "$CONDA_BASE/etc/profile.d/conda.sh"
if [ -f "$CONDA_BASE/etc/profile.d/mamba.sh" ]; then
    source "$CONDA_BASE/etc/profile.d/mamba.sh"
fi
mamba activate ros2_humble 2>/dev/null || conda activate ros2_humble
if [ "$CONDA_DEFAULT_ENV" != "ros2_humble" ]; then
    echo "ERROR: failed to activate the 'ros2_humble' env." >&2
    exit 1
fi

# --- 3. source the ROS2 workspace --------------------------------------------
if [ ! -f "$HOME/ros2_ws/install/setup.bash" ]; then
    echo "ERROR: ~/ros2_ws/install/setup.bash not found. Build the workspace first." >&2
    exit 1
fi
source "$HOME/ros2_ws/install/setup.bash"

# --- 4. launch SLAM (replaces this shell so systemd tracks ros2 launch) ------
# NOTE: does NOT start a Lidar driver -- it only subscribes to /scan, which
# agv.service's sllidar_node already publishes. Launching a second Lidar
# driver here is what would cause a serial-port conflict.
exec ros2 launch "$HOME/agv_slam/slam.launch.py"
