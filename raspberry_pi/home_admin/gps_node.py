#!/usr/bin/env python3
"""
AGV GPS Node  (ROS2 / rclpy)  --  REAL or SIMULATED

Publishes:
    /gps  (std_msgs/String) : "lat,lon,heading,speed_kmh,fix,sats"

  * With a FIX  -> "lat,lon,heading,speed,fix,sats"   (sats = used in solution)
  * ACQUIRING   -> ",,,0.0,0,<sats_in_view>"          (no fix yet; live feedback)
    The empty lat/lon tells the app "still acquiring", while <sats_in_view>
    (parsed from GSV) lets the app show a rising count instead of a blank card.

TWO MODES:
  (default) REAL  : reads u-blox NEO-M8N NMEA on /dev/ttyAMA0 @ 9600.
  --sim           : SIMULATION (always has a fix; FIX 1 SAT 12 signature).

Run:
    mamba activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/gps_node.py            # real GPS (outdoors / open sky)
    python3 ~/gps_node.py --sim      # simulation (indoor dev/testing)

If real mode says "No module named 'serial'":  pip install pyserial
"""

import sys
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

PORT = "/dev/ttyAMA0"
BAUD = 9600

# Simulation start point (Quezon City) + virtual robot limits
SIM_START_LAT = 14.7481338
SIM_START_LON = 121.0616677
SIM_MAX_SPEED_MPS = 0.6     # ground speed at linear.x = 1.0
SIM_MAX_TURN_DPS = 35.0     # turn rate at angular.z = 1.0
SIM_DT = 0.2
M_PER_DEG_LAT = 111320.0


def nmea_to_deg(val, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    if not val or '.' not in val:
        return None
    dot = val.index('.')
    deg_len = dot - 2
    if deg_len < 1:
        return None
    try:
        deg = float(val[:deg_len])
        minutes = float(val[deg_len:])
    except ValueError:
        return None
    dec = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        dec = -dec
    return dec


# ============================ REAL GPS ============================
class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        import serial
        self.pub = self.create_publisher(String, '/gps', 10)
        self.lat = None
        self.lon = None
        self.heading = ""
        self.speed_kmh = 0.0
        self.fix = 0
        self.sats = 0
        self.sats_in_view = {}      # per-talker (GP/GL/GA/GB) satellites in view
        self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(0.5, self.publish_gps)
        self._last_log = 0
        self.get_logger().info(f"GPS node ready (REAL) on {PORT} @ {BAUD}")

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not raw.startswith('$'):
                continue
            parts = raw.split(',')
            tag = parts[0][3:] if len(parts[0]) >= 6 else parts[0]
            try:
                if tag == 'GGA' and len(parts) >= 8:
                    lat = nmea_to_deg(parts[2], parts[3])
                    lon = nmea_to_deg(parts[4], parts[5])
                    if lat is not None:
                        self.lat = lat
                    if lon is not None:
                        self.lon = lon
                    self.fix = int(parts[6]) if parts[6] else 0
                    self.sats = int(parts[7]) if parts[7] else 0
                elif tag == 'RMC' and len(parts) >= 9:
                    if parts[7]:
                        try:
                            self.speed_kmh = float(parts[7]) * 1.852
                        except ValueError:
                            pass
                    if parts[8]:
                        self.heading = parts[8]
                elif tag == 'VTG' and len(parts) >= 8:
                    if parts[1]:
                        self.heading = parts[1]
                    if parts[7]:
                        try:
                            self.speed_kmh = float(parts[7])
                        except ValueError:
                            pass
                elif tag == 'GSV' and len(parts) >= 4:
                    # satellites in view for this constellation (GP/GL/GA/GB)
                    talker = parts[0][1:3]
                    try:
                        self.sats_in_view[talker] = int(parts[3])
                    except ValueError:
                        pass
            except Exception:
                pass

    def publish_gps(self):
        msg = String()
        if self.lat is not None and self.lon is not None and self.fix >= 1:
            # real fix -> full position
            msg.data = (f"{self.lat:.7f},{self.lon:.7f},{self.heading},"
                        f"{self.speed_kmh:.1f},{self.fix},{self.sats}")
        else:
            # no fix yet -> publish "acquiring" with satellites IN VIEW so the
            # app shows a live, rising count instead of a blank screen.
            siv = sum(self.sats_in_view.values()) if self.sats_in_view else 0
            msg.data = f",,,0.0,0,{siv}"
        self.pub.publish(msg)

        self._last_log += 1
        if self._last_log >= 10:
            self._last_log = 0
            if self.lat is not None and self.fix >= 1:
                self.get_logger().info(
                    f"FIX  sats={self.sats} lat={self.lat:.6f} "
                    f"lon={self.lon:.6f} hdg={self.heading or '-'} "
                    f"spd={self.speed_kmh:.1f}km/h")
            else:
                siv = sum(self.sats_in_view.values()) if self.sats_in_view else 0
                self.get_logger().info(
                    f"acquiring... {siv} sat(s) in view (no fix yet)")

    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ============================ SIMULATION ============================
class GPSSim(Node):
    def __init__(self):
        super().__init__('gps_node')   # same node/topic name as real
        self.pub = self.create_publisher(String, '/gps', 10)
        self.lat = SIM_START_LAT
        self.lon = SIM_START_LON
        self.heading = 90.0            # start facing East
        self.lin = 0.0
        self.ang = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(SIM_DT, self.step)
        self._log = 0
        self.get_logger().warn(
            "GPS node in SIMULATION mode (NO real GPS). Virtual robot driven "
            "by /cmd_vel. Use real mode outdoors for the field test.")

    def cmd_cb(self, msg: Twist):
        self.lin = max(-1.0, min(1.0, msg.linear.x))
        self.ang = max(-1.0, min(1.0, msg.angular.z))

    def step(self):
        # heading: compass bearing, +angular.z = LEFT (CCW) = decreasing
        self.heading = (self.heading - self.ang * SIM_MAX_TURN_DPS * SIM_DT) % 360.0
        spd = self.lin * SIM_MAX_SPEED_MPS
        br = math.radians(self.heading)
        d_north = spd * SIM_DT * math.cos(br)
        d_east = spd * SIM_DT * math.sin(br)
        self.lat += d_north / M_PER_DEG_LAT
        self.lon += d_east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        msg = String()
        msg.data = (f"{self.lat:.7f},{self.lon:.7f},{self.heading:.1f},"
                    f"{abs(spd) * 3.6:.1f},1,12")
        self.pub.publish(msg)
        self._log += 1
        if self._log >= 25:
            self._log = 0
            self.get_logger().info(
                f"[SIM] lat={self.lat:.6f} lon={self.lon:.6f} "
                f"hdg={self.heading:.0f} spd={abs(spd) * 3.6:.1f}km/h")


def main():
    sim = "--sim" in sys.argv
    rclpy.init()
    node = GPSSim() if sim else GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
