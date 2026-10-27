# Costmaps of the saved SLAM maps

One costmap per SLAM map in `~/agv_maps`, same name:

| File | What |
|---|---|
| `<name>.png` | Picture, 4× enlarged. **Black** wall, **red** too close (the robot's centre can't be here without its body touching), **yellow → orange** getting close, **white** clear, **grey** unexplored. |
| `<name>.pgm` + `<name>.yaml` | The costmap itself, in ROS map format. Loads in Nav2's `map_server` or RViz like any map. Pixel = 0 free, 1–98 near an obstacle, 99 too close, 100 wall, 255 unknown. Same resolution and origin as the SLAM map. |
| `costmap_params.yaml` | The settings: the robot's 60 × 80 cm footprint, the inflation radius (0.55 m) and fall-off. Nav2's own costmap format. |

Make new ones after saving more maps (existing ones are skipped):

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
python3 ~/make_costmaps.py            # new maps only
python3 ~/make_costmaps.py --force    # all of them, e.g. after changing costmap_params.yaml
```

**Unexplored stays unexplored.** The SLAM maps mark unexplored cells grey (205), but their own `.yaml` thresholds make Nav2's map_server load grey as *free*. The costmaps keep it as unknown.
