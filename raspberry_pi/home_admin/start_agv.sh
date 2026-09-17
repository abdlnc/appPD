#!/usr/bin/env bash
# B.I.T.S. AGV auto-start wrapper
# Activates the RoboStack conda env (ros2_humble), sources the ROS2 workspace,
# then launches the whole AGV system. Called by agv.service on boot.
#
# systemd starts with a BARE environment, so everything (conda, ROS, PATH) must
# be set up here. Errors below show up in:  journalctl -u agv.service -f

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
    echo "       Edit start_agv.sh and hard-set CONDA_BASE to your install dir." >&2
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

# --- 4. launch everything (replaces this shell so systemd tracks ros2 launch) -
exec ros2 launch "$HOME/agv_autostart.launch.py"
