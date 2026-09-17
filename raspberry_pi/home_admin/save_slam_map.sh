#!/usr/bin/env bash
# save_slam_map.sh -- save the current SLAM map + convert it for the app gallery
#
# Run this from the SLAM-mapper terminal (Terminal 2 in TERMINAL_COMMANDS.md
# section 2), once the map looks good in RViz2. Environment must already be
# loaded (Rule 0: mamba activate ros2_humble && source ~/ros2_ws/install/setup.bash).
#
# What it does:
#   1. ros2 run nav2_map_server map_saver_cli -> ~/agv_maps/<name>.pgm + .yaml
#      (the standard ROS2 map format -- unchanged, still produced as before)
#   2. Converts the .pgm to a .png alongside it (via ~/draw_slam_path.py),
#      because the app's gallery (gallery_server.py's "maps" kind, served over
#      HTTP) can't display raw PGM -- browsers/Flutter's Image widget need a
#      normal web image format. That step ALSO draws the robot's traversed
#      path onto the PNG, annotated with its GPS coordinates when a fix was
#      available, and upscales it so both are actually legible (the raw maps
#      are only ~70px across).
#      The .pgm/.yaml stay untouched for anything that still wants the
#      original ROS2 map format (e.g. nav2 AMCL localization).
#
# Usage:
#   ./save_slam_map.sh              # name = map_YYYYMMDD_HHMMSS
#   ./save_slam_map.sh field_map    # name = field_map (overwrites if it exists)
#
# agv.service/bridge_node keeps running the whole time -- SLAM only
# subscribes to the existing /scan topic, it doesn't touch the Lidar
# hardware directly (see TERMINAL_COMMANDS.md section 2). So the app stays
# connected throughout; just open the gallery's MAPS tab once this is done.

set -e

MAPS_DIR="$HOME/agv_maps"
mkdir -p "$MAPS_DIR"

NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
OUT="$MAPS_DIR/$NAME"

echo "Saving SLAM map -> $OUT.pgm / $OUT.yaml ..."
# map_saver_cli occasionally fails with "Failed to spin map subscription"
# if it's run within the first ~30-40s of slam_toolbox starting (e.g. right
# after a fresh `agv.service` restart/boot, while the Pi is also busy
# spinning up the camera/AI detector/etc.) -- transient, not a real error,
# so retry a few times before actually giving up.
ATTEMPTS=3
for i in $(seq 1 $ATTEMPTS); do
    if ros2 run nav2_map_server map_saver_cli -f "$OUT"; then
        break
    fi
    if [ "$i" -eq "$ATTEMPTS" ]; then
        echo "map_saver_cli failed after $ATTEMPTS attempts -- is slam_toolbox actually running?" >&2
        exit 1
    fi
    echo "map_saver_cli attempt $i failed, retrying in 3s (probably just starting up)..." >&2
    sleep 3
done

echo "Converting to PNG (+ traversed path overlay) -> $OUT.png ..."
# draw_slam_path.py does the PGM->PNG conversion AND draws the robot's
# traversed path on top, annotated with its GPS when a fix was recorded.
# The .pgm/.yaml are left untouched -- overlay lives only in the .png, which
# is the app-facing gallery image; nav2 etc. still get the clean ROS map.
python3 "$HOME/draw_slam_path.py" "$OUT"

echo "Done. Saved:"
echo "  $OUT.pgm  (original ROS2 map format)"
echo "  $OUT.yaml (map metadata: resolution, origin, thresholds)"
echo "  $OUT.png  (for the app's gallery MAPS tab, with the path drawn on)"
