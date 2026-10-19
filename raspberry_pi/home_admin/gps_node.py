#!/usr/bin/env python3
"""
AGV GPS Node  (ROS2 / rclpy)  --  REAL or SIMULATED

Publishes:
    /gps  (std_msgs/String) : "lat,lon,heading,speed_kmh,fix,sats"

  * With a FIX  -> "lat,lon,heading,speed,fix,sats"   (sats = used in solution)
    ...but only a fix that passes the QUALITY FILTER (enough satellites, low
    enough HDOP -- see MIN_SATS_ACCEPT). A weak fix is published exactly like
    no fix, because a position that jumps metres is worse than none.
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
import random
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

PORT = "/dev/ttyAMA0"
BAUD = 9600

# ---- fix QUALITY filter ----
# A receiver will report a "fix" from as few as 4 satellites, but with that
# little geometry a tiny timing error swings the answer by metres, and a
# satellite entering or leaving view makes the position JUMP -- seen on this
# robot as a marker hopping around while parked (4 used, 5 in view).
# A wrong position is worse than none (the same rule the photo tagging
# follows), so a fix below this quality is published as NO fix.
#
# HDOP = the receiver's own estimate of how bad its satellite geometry is
# (1 = ideal, <2 good, >5 poor). It is in the GGA sentence alongside the
# satellite count, which is all that was read before.
#
# Two thresholds (hysteresis), like the no-path hold in control_node: a fix
# has to be clearly GOOD to be accepted, but once accepted it is only dropped
# when clearly BAD. A single threshold would flicker fix/no-fix every second
# whenever reception sits right on it, and every flicker stops NAV mode.
# ---- receiver dynamics ----
# The module smooths position when it believes it is stationary, which is how
# a 14m walk came back as 1m of wander: at 0.2-0.4 km/h the robot looks parked
# to it. Two settings change that, sent as a UBX-CFG-NAV5 message at startup:
#
#   dynModel 3 (pedestrian)  -- expect a slow-moving platform rather than the
#                               general-purpose default
#   staticHoldThresh 0       -- never freeze position at low speed
#
# Applied to RAM ONLY, deliberately: nothing is written to the module's flash,
# so a power cycle restores whatever it had before and there is nothing to
# undo if this turns out wrong.
#
# A receiver that ignores it (a clone with its own firmware) costs one logged
# warning; the node carries on either way.
UBX_DYN_MODEL        = 3   # 0=portable 2=stationary 3=pedestrian 4=automotive
UBX_STATIC_HOLD_CMS  = 0   # cm/s; 0 disables static hold entirely

MIN_SATS_ACCEPT = 6        # to START trusting the fix
MAX_HDOP_ACCEPT = 2.5
MIN_SATS_KEEP   = 5        # to KEEP trusting a fix already accepted

# ---- staleness: no fresh position = no fix ----
# The reader thread STORES the last position it parsed, and publish_gps sends
# whatever is stored every 0.5s. So when the serial stream dies -- which this
# receiver's wiring has done, sending zero bytes for minutes at a time -- the
# last fix used to go out as a perfectly valid position, forever, with its
# satellite count frozen too. Harmless-looking on a map; dangerous for the
# boundary fence, which would see the robot parked "inside" while it drove
# out of the field. A position older than this is now published as no fix.
# GGA arrives once a second, so 3s is three missed sentences in a row.
GGA_STALE_S = 3.0
MAX_HDOP_KEEP   = 3.5

# Simulation start point (Quezon City) + virtual robot limits
SIM_START_LAT = 14.7481338
SIM_START_LON = 121.0616677
SIM_MAX_SPEED_MPS = 0.6     # ground speed at linear.x = 1.0
SIM_MAX_TURN_DPS = 35.0     # turn rate at angular.z = 1.0
SIM_DT = 0.2
M_PER_DEG_LAT = 111320.0

# ---- optional realism for bench-testing COVER and the fence ----
# Both OFF by default, so the existing NAV demo behaves exactly as before.
#
# --sim-ackermann: the default sim turns on the spot (heading changes even
# at zero speed). The real robot steers its front wheels and CANNOT pivot:
# its turn rate is proportional to its speed. Coverage turns and the fence
# recovery depend on exactly that, so a sim that pivots would pass tests
# the robot fails. Bicycle model: yaw rate = v / R * steer, R = half the
# turning diameter at full lock.
SIM_TURN_DIAMETER_M = 3.0   # TODO: measure on the robot (full lock circle)
#
# --sim-noise: the default sim is a perfect GPS. The NEO-M8N is good to
# about +/-2.5m, and its error is CORRELATED -- it wanders over tens of
# seconds rather than jittering independently each fix, which is why it
# fools a controller. Modelled as a first-order Gauss-Markov drift per axis
# (SIM_NOISE_SIGMA_M standard deviation, so +/-2.5m is about 2 sigma).
# Also, like the real receiver, no course is reported while barely moving.
SIM_NOISE_SIGMA_M = 1.25
SIM_NOISE_TAU_S = 30.0
SIM_COG_MIN_MPS = 0.2


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
        self.hdop = None            # None = not reported yet
        self.quality_ok = False     # passes the filter (see MIN_SATS_*)
        self.last_gga = 0.0         # when a GGA last arrived (see GGA_STALE_S)
        self.sats_in_view = {}      # per-talker (GP/GL/GA/GB) satellites in view
        self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        self.configure_receiver()
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(0.5, self.publish_gps)
        self._last_log = 0
        self.get_logger().info(f"GPS node ready (REAL) on {PORT} @ {BAUD}")

    # ---- receiver configuration (UBX) ----
    @staticmethod
    def _ubx(cls, mid, payload=b""):
        """Frame a UBX message: sync, class, id, length, payload, checksum."""
        body = bytes([cls, mid]) + len(payload).to_bytes(2, "little") + payload
        a = b = 0
        for byte in body:
            a = (a + byte) & 0xFF
            b = (b + a) & 0xFF
        return b"\xB5\x62" + body + bytes([a, b])

    def configure_receiver(self):
        """Tell the module to expect a slow-moving vehicle, and never to
        freeze its position when it thinks it has stopped.

        The setting is polled back rather than assumed: this UART also carries
        NMEA, and a silently ignored write would look exactly like the bug
        being fixed."""
        payload = bytearray(36)
        # mask: bit0 = apply dynModel, bit6 = apply the staticHold settings
        payload[0:2] = (0x0001 | 0x0040).to_bytes(2, "little")
        payload[2] = UBX_DYN_MODEL
        payload[22] = UBX_STATIC_HOLD_CMS           # staticHoldThresh, cm/s
        payload[28:30] = (0).to_bytes(2, "little")  # staticHoldMaxDist, m
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self._ubx(0x06, 0x24, bytes(payload)))
            self.ser.flush()
            time.sleep(0.4)
            self.ser.write(self._ubx(0x06, 0x24))   # poll it back
            self.ser.flush()
            got = self._read_ubx(0x06, 0x24, timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f"Receiver configuration failed: {e}")
            return
        if got is None or len(got) < 29:
            self.get_logger().warn(
                "Receiver did not answer the configuration poll -- it may "
                "ignore UBX. Position smoothing at low speed may persist.")
            return
        dyn, hold = got[2], got[22]
        if dyn == UBX_DYN_MODEL and hold == UBX_STATIC_HOLD_CMS:
            self.get_logger().info(
                f"Receiver configured: dynamic model {dyn} (slow vehicle), "
                f"static hold {hold} cm/s (off) -- RAM only, reverts on a "
                f"power cycle")
        else:
            self.get_logger().warn(
                f"Receiver kept its own settings: dynModel={dyn} "
                f"staticHold={hold} (asked for {UBX_DYN_MODEL}/"
                f"{UBX_STATIC_HOLD_CMS})")

    def _read_ubx(self, cls, mid, timeout=2.0):
        """Wait for one UBX message of this class/id, stepping over the NMEA
        lines sharing the port. Returns its payload, or None."""
        end = time.time() + timeout
        buf = b""
        while time.time() < end:
            chunk = self.ser.read(256)
            if chunk:
                buf += chunk
            i = buf.find(b"\xB5\x62")
            while i >= 0 and len(buf) >= i + 6:
                ln = int.from_bytes(buf[i + 4:i + 6], "little")
                if len(buf) < i + 8 + ln:
                    break                       # wait for the rest to arrive
                if buf[i + 2] == cls and buf[i + 3] == mid:
                    return buf[i + 6:i + 6 + ln]
                buf = buf[i + 8 + ln:]
                i = buf.find(b"\xB5\x62")
            if len(buf) > 4096:
                buf = buf[-1024:]
        return None

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
                    self.last_gga = time.time()
                    lat = nmea_to_deg(parts[2], parts[3])
                    lon = nmea_to_deg(parts[4], parts[5])
                    if lat is not None:
                        self.lat = lat
                    if lon is not None:
                        self.lon = lon
                    self.fix = int(parts[6]) if parts[6] else 0
                    self.sats = int(parts[7]) if parts[7] else 0
                    try:
                        self.hdop = float(parts[8]) if (len(parts) > 8
                                                         and parts[8]) else None
                    except ValueError:
                        self.hdop = None
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

    def _quality_check(self):
        """Apply the fix-quality filter, with hysteresis. Returns (ok, why)."""
        if self.lat is None or self.lon is None or self.fix < 1:
            return False, "no fix"
        age = time.time() - self.last_gga
        if age > GGA_STALE_S:
            return False, (f"no data from the receiver for {age:.0f}s -- "
                           f"the last position is stale, not current")
        # No HDOP reported = geometry unknown = not trusted. The NEO-M8N always
        # sends it with a fix, so this only bites on a malformed sentence.
        hdop = self.hdop if self.hdop is not None else 99.0
        if self.quality_ok:
            min_sats, max_hdop = MIN_SATS_KEEP, MAX_HDOP_KEEP
        else:
            min_sats, max_hdop = MIN_SATS_ACCEPT, MAX_HDOP_ACCEPT
        if self.sats < min_sats:
            return False, f"only {self.sats} sats (need {min_sats})"
        if hdop > max_hdop:
            return False, f"HDOP {hdop:.1f} (need <= {max_hdop})"
        return True, ""

    def publish_gps(self):
        msg = String()
        ok, why = self._quality_check()
        if ok != self.quality_ok:
            self.quality_ok = ok
            hd = f"{self.hdop:.1f}" if self.hdop is not None else "?"
            if ok:
                self.get_logger().info(
                    f"GPS fix ACCEPTED: {self.sats} sats, HDOP {hd}")
            elif self.fix >= 1:
                self.get_logger().warn(
                    f"GPS fix REJECTED as unreliable: {why} -- publishing as "
                    f"no fix rather than a position that may be metres off")
        if ok:
            # real fix -> full position
            msg.data = (f"{self.lat:.7f},{self.lon:.7f},{self.heading},"
                        f"{self.speed_kmh:.1f},{self.fix},{self.sats}")
        else:
            # no fix, or a fix too poor to trust -> publish "acquiring" with
            # satellites IN VIEW so the app shows a live, rising count instead
            # of a blank screen. Every consumer (app, NAV, path recorder,
            # photo tagging) already treats this as "no position".
            siv = sum(self.sats_in_view.values()) if self.sats_in_view else 0
            if time.time() - self.last_gga > GGA_STALE_S:
                siv = 0     # a frozen count is as misleading as a frozen fix
            msg.data = f",,,0.0,0,{siv}"
        self.pub.publish(msg)

        self._last_log += 1
        if self._last_log >= 10:
            self._last_log = 0
            hd = f"{self.hdop:.1f}" if self.hdop is not None else "?"
            if self.quality_ok:
                self.get_logger().info(
                    f"FIX  sats={self.sats} hdop={hd} lat={self.lat:.6f} "
                    f"lon={self.lon:.6f} hdg={self.heading or '-'} "
                    f"spd={self.speed_kmh:.1f}km/h")
            elif self.lat is not None and self.fix >= 1:
                self.get_logger().info(
                    f"weak fix held back: sats={self.sats} hdop={hd} "
                    f"(needs {MIN_SATS_ACCEPT}+ sats, HDOP <= {MAX_HDOP_ACCEPT})")
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
        self.ackermann = "--sim-ackermann" in sys.argv
        self.noise = "--sim-noise" in sys.argv
        self.ex = 0.0           # current GPS error, metres east/north
        self.ey = 0.0
        self.get_logger().warn(
            "GPS node in SIMULATION mode (NO real GPS). Virtual robot driven "
            "by /cmd_vel. Use real mode outdoors for the field test."
            + (f" Ackermann steering, {SIM_TURN_DIAMETER_M} m turning circle."
               if self.ackermann else " Pivot steering (turns on the spot).")
            + (f" GPS noise ON (sigma {SIM_NOISE_SIGMA_M} m, drifting)."
               if self.noise else " Perfect GPS."))

    def cmd_cb(self, msg: Twist):
        self.lin = max(-1.0, min(1.0, msg.linear.x))
        self.ang = max(-1.0, min(1.0, msg.angular.z))

    def step(self):
        spd = self.lin * SIM_MAX_SPEED_MPS
        # heading: compass bearing, +angular.z = LEFT (CCW) = decreasing
        if self.ackermann:
            # bicycle model: no motion, no turning; reversing turns the
            # other way, as a real car does
            yaw_dps = math.degrees(spd / (SIM_TURN_DIAMETER_M / 2.0)) * self.ang
        else:
            yaw_dps = self.ang * SIM_MAX_TURN_DPS
        self.heading = (self.heading - yaw_dps * SIM_DT) % 360.0
        br = math.radians(self.heading)
        d_north = spd * SIM_DT * math.cos(br)
        d_east = spd * SIM_DT * math.sin(br)
        self.lat += d_north / M_PER_DEG_LAT
        self.lon += d_east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        lat, lon, hdg = self.lat, self.lon, f"{self.heading:.1f}"
        if self.noise:
            a = SIM_DT / SIM_NOISE_TAU_S
            k = SIM_NOISE_SIGMA_M * math.sqrt(2 * a)
            self.ex += -self.ex * a + k * random.gauss(0, 1)
            self.ey += -self.ey * a + k * random.gauss(0, 1)
            lat += self.ey / M_PER_DEG_LAT
            lon += self.ex / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
            if abs(spd) < SIM_COG_MIN_MPS:
                hdg = ""                  # no course while barely moving
        msg = String()
        msg.data = (f"{lat:.7f},{lon:.7f},{hdg},"
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
