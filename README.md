# AGV — Path Obstruction Detection & Field Mapping

Deep Learning-Based Path Obstruction Detection for Agricultural AGVs with Field
Mapping System.

This repository holds **both halves** of the system:

| Folder | What it is |
|---|---|
| `lib/`, `android/`, `assets/` … | The **Flutter mobile app** (`robot_controller`) — the operator's remote: live camera with AI detection overlay, Lidar view, manual/auto/nav driving, SLAM map controls, and the image gallery. |
| [`raspberry_pi/`](raspberry_pi/) | The **Raspberry Pi source code** that runs on the robot — ROS2 nodes (motors, Lidar obstacle avoidance, app bridge, camera), the YOLOv8 obstruction detector, GPS/path recording, SLAM config, and the systemd auto-start units. See its [README](raspberry_pi/README.md). |

The two talk over a **WebSocket on port 8765** (control + telemetry) and
**HTTP on port 8080** (saved images and maps).

## Building the app

```bash
flutter pub get
flutter build apk --release
```

## Deploying the robot code

See [`raspberry_pi/README.md`](raspberry_pi/README.md) for where each file goes
on the Pi, and `raspberry_pi/TERMINAL_COMMANDS.md` for the full command
reference.
