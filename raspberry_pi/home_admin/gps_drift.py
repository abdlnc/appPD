#!/usr/bin/env python3
"""
Measure how much the GPS position wanders while the robot stands still.

Reads /gps_raw -- the receiver's own positions. /gps itself is held still
while the robot is parked (gps_node.py, PARKED = HELD STILL), so it would
read zero wander and tell you nothing about the receiver.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/gps_drift.py              # 10 minutes, robot parked, service running
    python3 ~/gps_drift.py --minutes 20

WHY THIS MATTERS. The field boundary and COVER mode both steer by GPS. How
well they work is set almost entirely by how far the reported position
wanders over 5-20 seconds -- that is the error in the heading estimate and
the noise the boundary guard has to tell from real movement. Measured on
2026-10-19 with the robot parked near the house (10-11 satellites, HDOP
1.2-1.6, which looks fine): spread 3.0 m east / 5.4 m north, 9.9 m at worst
in 5 minutes, and a median 3.1 m of wander over just 20 s. At that level
COVER cannot hold its lanes and a 3 m stop line cannot hold the robot (see
~/coverage_sim.py). An open field may well be much better -- this is how to
find out. Run it IN THE FIELD, with the robot where it will work.

READING THE RESULT, against the 3 m stop line and 6 m lanes:
  wander over 20 s, median   under ~1 m  -> COVER and the fence should work
                             1-2 m       -> marginal; widen the stop line
                             over ~2 m   -> GPS too poor here for either
Then plug the numbers into the simulator to see what to expect:
    python3 ~/coverage_sim.py --noise <sigma> --tau 120 --seeds 10
using the larger of the two sigmas printed below.
"""
import argparse
import math
import statistics as st
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Rec(Node):
    def __init__(self):
        super().__init__('gps_drift')
        self.rows = []
        self.create_subscription(String, '/gps_raw', self.cb, 50)

    def cb(self, m):
        p = m.data.split(",")
        try:
            self.rows.append((time.time(), float(p[0]), float(p[1])))
        except (ValueError, IndexError):
            pass            # no fix at this moment


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    n = Rec()
    end = time.time() + a.minutes * 60
    print(f"recording /gps_raw for {a.minutes:.0f} min -- keep the robot still")
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.5)
    rclpy.shutdown()

    # GGA arrives once a second; gps_node republishes at 2 Hz -- one per second
    rows = []
    for r in n.rows:
        if not rows or int(r[0]) != int(rows[-1][0]):
            rows.append(r)
    if len(rows) < 60:
        raise SystemExit(f"only {len(rows)} fixes -- not enough (no fix?)")
    lat0 = st.mean(r[1] for r in rows)
    lon0 = st.mean(r[2] for r in rows)
    k = math.radians(1) * 6371000.0
    xy = [(t, (lo - lon0) * k * math.cos(math.radians(lat0)), (la - lat0) * k)
          for t, la, lo in rows]
    print(f"\n{len(xy)} fixes over {(xy[-1][0] - xy[0][0]) / 60:.1f} min")
    print(f"spread: sigma east {st.pstdev(p[1] for p in xy):.2f} m, "
          f"north {st.pstdev(p[2] for p in xy):.2f} m; worst "
          f"{max(math.hypot(p[1], p[2]) for p in xy):.2f} m from the mean")
    print("\nwander over a time gap:")
    for dt in (1, 5, 10, 20, 60):
        d = []
        for i, (t, x, y) in enumerate(xy):
            for j in range(i + 1, len(xy)):
                if xy[j][0] - t >= dt:
                    d.append(math.hypot(xy[j][1] - x, xy[j][2] - y))
                    break
        if d:
            d.sort()
            print(f"  {dt:3d} s: median {d[len(d) // 2]:.2f} m, "
                  f"95% {d[int(len(d) * .95)]:.2f} m, max {d[-1]:.2f} m")


if __name__ == "__main__":
    main()
