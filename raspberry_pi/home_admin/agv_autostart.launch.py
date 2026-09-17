"""
AGV AUTO-START master launch  --  B.I.T.S.

Brings up the whole core system on boot:
  agv_full.launch.py (lidar + motor + control + bridge + camera)
  + gps_node.py, nav_node.py, path_recorder_node.py, ai_detector_node.py,
    soil_camera_node.py (2nd camera, /dev/video2, facing down at the soil)
SLAM is NOT launched here -- it runs as its own systemd unit,
agv-slam.service (see ~/start_slam.sh). It still starts automatically on
boot, just separately, so that the app's "RESET MAPPED AREA" button can
restart ONLY slam_toolbox (the sole way to clear its map -- this build has
no reset service) without dropping the app's WebSocket connection, camera
or motors along with it.

    sudo systemctl status agv-slam.service     # is mapping running?
    sudo systemctl restart agv-slam.service    # clear the map / restart it
    sudo systemctl disable --now agv-slam.service   # turn mapping off entirely

SLAM never opens the Lidar itself -- it only subscribes to /scan, which
agv_full.launch.py's sllidar_node (below) already publishes. Launching a
second Lidar driver is what would cause a serial-port conflict.

GPS MODE TOGGLE (REAL is the default / production state):
  * REAL GPS (default): walang flag file -> totoong NEO-M8N GPS.
  * SIM GPS (indoor demo): kung may file na  ~/agv_use_sim  -> tatakbo ang
    gps_node nang --sim (virtual GPS). May malaking warning sa journal.

  Buksan ang SIM:   touch ~/agv_use_sim   && sudo systemctl restart agv.service
  Ibalik sa REAL:   rm -f ~/agv_use_sim   && sudo systemctl restart agv.service

  *** BAGO I-TURNOVER SA CLIENT: siguraduhing WALA ang ~/agv_use_sim (REAL). ***
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

HOME = os.path.expanduser("~")
USE_SIM = os.path.exists(os.path.join(HOME, "agv_use_sim"))   # flag file = SIM GPS


def _node(script, args=None):
    cmd = ["python3", os.path.join(HOME, script)] + (args or [])
    return ExecuteProcess(cmd=cmd, output="screen")


def generate_launch_description():
    agv_full = os.path.join(
        get_package_share_directory("agv_control"),
        "launch", "agv_full.launch.py")

    gps_args = ["--sim"] if USE_SIM else []

    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(agv_full)),
        # NOTE: SLAM is intentionally NOT here -- see agv-slam.service / the
        # module docstring above.
        _node("gps_node.py", gps_args),
        _node("nav_node.py"),
        _node("path_recorder_node.py"),
        _node("ai_detector_node.py"),   # comment out this line if RAM is tight
        _node("soil_camera_node.py"),   # 2nd camera (/dev/video2) -> ~/agv_soil_images
        _node("gallery_server.py"),     # read-only HTTP viewer, port 8080
    ]

    if USE_SIM:
        actions.insert(0, LogInfo(msg=(
            "*** GPS NASA --sim MODE (may flag ~/agv_use_sim). HINDI totoong GPS. "
            "Gawin: rm ~/agv_use_sim + restart, para ibalik sa REAL bago turnover. ***")))

    return LaunchDescription(actions)
