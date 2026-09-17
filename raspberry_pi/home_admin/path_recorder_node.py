#!/usr/bin/env python3
"""
AGV Path Recorder Node  (ROS2 / rclpy)  --  TRAVERSED PATH (contract 2.2 Mapping)

Inilalista at sine-save ang AKTWAL na rutang dinaanan ng robot (GPS trail).

Subscribes:
    /gps  (std_msgs/String)   "lat,lon,heading,speed,fix,sats"  (gps_node)

Sine-save sa:
    ~/agv_paths/path_YYYYMMDD_HHMMSS.csv      (timestamp, lat, lon, heading)
    ~/agv_paths/path_YYYYMMDD_HHMMSS.geojson  (LineString -- pwedeng buksan sa
                                               geojson.io / QGIS para makita)

Nag-pa-publish din ng tumatakbong trail sa /path (std_msgs/String,
"lat,lon;lat,lon;...") para magamit ng app (optional na breadcrumb).

SLAM TRACK (para ma-drawing ang dinaanan SA IBABAW ng SLAM map):
    ~/agv_paths/slam_track_YYYYMMDD_HHMMSS.csv   (epoch, map_x, map_y, lat, lon)

    Kailangan ito dahil ang SLAM map ay naka-METERS sa "map" frame (arbitrary
    ang origin, galing sa kung saan nag-start ang slam_toolbox) -- WALANG
    georeferencing papuntang lat/lon. Kaya hindi pwedeng i-plot ang GPS trail
    diretso sa map image. Ang kinukuha natin dito: ang posisyon sa MAP frame
    (mula /pose) + ang GPS na katapat nito sa parehong sandali.

    Naka-drive sa /pose, HINDI sa GPS -- kaya may path pa rin kahit walang GPS
    fix (indoor). Blangko lang ang lat/lon columns pag walang fix. Ginagamit
    ito ng ~/draw_slam_path.py tuwing nagse-save ng map.

NO colcon build. Patakbuhin:
    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/path_recorder_node.py
"""

import os
import json
import math
import time
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

MIN_STEP_M = 1.0           # mag-record ng bagong point pag lumayo nang ganito (m)
# Mas pino kaysa MIN_STEP_M: ilang metro lang ang buong SLAM map, kaya 1m na
# hakbang ay sobrang coarse para sa guhit sa ibabaw ng map.
TRACK_MIN_STEP_M = 0.25    # hakbang sa MAP frame bago mag-record ng track point
SAVE_EVERY_S = 5.0         # i-flush din sa disk kada ganito
R_EARTH = 6371000.0
OUT_DIR = os.path.expanduser("~/agv_paths")


def distance_m(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    y = math.radians(lat2 - lat1)
    return R_EARTH * math.hypot(x, y)


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(OUT_DIR, "path_" + stamp + ".csv")
        self.geojson_path = os.path.join(OUT_DIR, "path_" + stamp + ".geojson")
        self.points = []        # (epoch, lat, lon, heading)
        self.last = None        # (lat, lon)

        # --- SLAM-frame track (para sa overlay sa map image) ---
        self.track_path = os.path.join(OUT_DIR, "slam_track_" + stamp + ".csv")
        self.track_last = None       # (map_x, map_y) ng huling na-record
        self.track_n = 0
        # Kasalukuyang GPS, o (None, None) pag WALANG fix ngayon. Nililinis pag
        # nawala ang fix -- mas masama ang lumang posisyon kaysa walang laman.
        self.cur_lat = None
        self.cur_lon = None
        with open(self.track_path, "w") as f:
            f.write("epoch,map_x,map_y,lat,lon\n")

        self.path_pub = self.create_publisher(String, '/path', 10)
        self.create_subscription(String, '/gps', self.gps_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
                                 self.pose_cb, 10)
        self.create_timer(SAVE_EVERY_S, self.save_geojson)
        self.get_logger().info("Path recorder ready -> " + self.csv_path)
        self.get_logger().info("SLAM track -> " + self.track_path)

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Posisyon sa SLAM 'map' frame + katapat na GPS sa sandaling iyon."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.track_last is not None:
            dx = x - self.track_last[0]
            dy = y - self.track_last[1]
            if math.hypot(dx, dy) < TRACK_MIN_STEP_M:
                return
        self.track_last = (x, y)
        self.track_n += 1
        lat = "" if self.cur_lat is None else "%.7f" % self.cur_lat
        lon = "" if self.cur_lon is None else "%.7f" % self.cur_lon
        with open(self.track_path, "a") as f:
            f.write("%.3f,%.4f,%.4f,%s,%s\n" % (time.time(), x, y, lat, lon))

    def gps_cb(self, msg):
        p = msg.data.split(',')
        if len(p) < 2:
            return
        try:
            lat = float(p[0])
            lon = float(p[1])
        except ValueError:
            # walang fix -> linisin, para hindi maka-tag ng lumang posisyon
            self.cur_lat = None
            self.cur_lon = None
            return
        self.cur_lat = lat
        self.cur_lon = lon
        heading = p[2].strip() if (len(p) >= 3 and p[2].strip()) else ""

        # mag-record lang pag lumayo nang MIN_STEP_M (iwas redundant points)
        if self.last is not None:
            if distance_m(self.last[0], self.last[1], lat, lon) < MIN_STEP_M:
                return
        self.last = (lat, lon)
        self.points.append((time.time(), lat, lon, heading))

        with open(self.csv_path, "a") as f:
            f.write("%.3f,%.7f,%.7f,%s\n"
                    % (self.points[-1][0], lat, lon, heading))

        trail = ";".join("%.7f,%.7f" % (la, lo)
                         for _, la, lo, _ in self.points)
        m = String()
        m.data = trail
        self.path_pub.publish(m)

        if len(self.points) % 10 == 0:
            self.get_logger().info("Recorded %d points" % len(self.points))

    def save_geojson(self):
        if not self.points:
            return
        coords = [[lo, la] for _, la, lo, _ in self.points]  # GeoJSON = [lon,lat]
        geo = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {"name": "AGV traversed path",
                               "points": len(self.points)},
                "geometry": {"type": "LineString", "coordinates": coords},
            }],
        }
        with open(self.geojson_path, "w") as f:
            json.dump(geo, f, indent=2)

    def shutdown(self):
        self.save_geojson()
        self.get_logger().info(
            "Saved %d points -> %s (+ .geojson); %d SLAM track points -> %s"
            % (len(self.points), self.csv_path, self.track_n, self.track_path))


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
