#!/usr/bin/env python3
"""
Measure the Lidar scan, and show what the sunlight filter does to it.

RUN THIS IN THE SUN. scan_filter's thresholds were set from indoor scans
plus the geometry of the C1's 0.5-degree ray spacing; the one thing that
could not be measured indoors is what a sun phantom actually looks like on
this unit. This tool is how you check, and how you re-tune if needed.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/scan_stats.py                  # 40 scans
    python3 ~/scan_stats.py --scans 200      # a longer look
    python3 ~/scan_stats.py --show-rejected  # inspect what got thrown away

The service can stay running -- this only subscribes.

WHAT TO LOOK FOR
----------------
  * "rejected" climbing a long way in sun compared with shade is the filter
    doing its job.
  * "changed INSIDE the trigger band" should stay at or near 0. Anything
    here means the filter is altering a reading avoidance acts on, which is
    the one thing it is built not to do. Investigate before trusting it.
  * "made CLOSER" must be 0, always. The filter can only ever remove
    returns, so a reading moving closer means something is wrong.
  * "sun-blinded" scans mean the receiver is saturated and obstacles may be
    returning nothing at all. No filter helps there -- shade the unit, or
    drive at a different time of day.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from agv_control import scan_filter
except ImportError:
    sys.exit("cannot import agv_control.scan_filter -- "
             "source ~/ros2_ws/install/setup.bash first")

ANGLE_OFFSET_DEG = 180.0        # matches control_node / nav_node
MIN_VALID_DIST = 0.15
TRIGGER_BAND = 0.80             # scan_filter.TRUST_BAND, for the report


def zone_of(deg):
    if deg >= 330 or deg < 30:
        return 'front'
    if deg < 90:
        return 'front_l'
    if deg < 150:
        return 'left'
    if deg < 210:
        return 'back'
    if deg < 270:
        return 'right'
    return 'front_r'


def zone_minima(ranges, angle_min, angle_increment):
    buckets = {}
    for k, r in enumerate(ranges):
        if scan_filter._bad(r) or r < MIN_VALID_DIST:
            continue
        deg = (math.degrees(angle_min + k * angle_increment)
               + ANGLE_OFFSET_DEG) % 360.0
        buckets.setdefault(zone_of(deg), []).append(r)
    return {z: min(v) for z, v in buckets.items()}


class Collector(Node):
    def __init__(self, want):
        super().__init__('scan_stats')
        self.want = want
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self.scans.append, 10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scans', type=int, default=40)
    ap.add_argument('--show-rejected', action='store_true',
                    help="print the neighbourhood of rejected near returns")
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.scans)
    print(f"collecting {args.scans} scans from /scan ...")
    waited = 0.0
    while len(node.scans) < args.scans and waited < args.scans * 0.5 + 20:
        rclpy.spin_once(node, timeout_sec=0.5)
        waited += 0.5
    scans = node.scans
    if not scans:
        rclpy.shutdown()
        sys.exit("no scans arrived -- is sllidar_node running?")

    first = scans[0]
    print(f"\nscans: {len(scans)}   rays/scan: {len(first.ranges)}   "
          f"intensities: {'yes' if len(first.intensities) else 'NO'}")
    print(f"filter: TRUST_BAND={scan_filter.TRUST_BAND}m  "
          f"NEAR_BAND={scan_filter.NEAR_BAND}m  "
          f"MIN_AGREE={scan_filter.MIN_AGREE}  "
          f"WINDOW_RAYS={scan_filter.WINDOW_RAYS}  "
          f"MIN_INTENSITY={scan_filter.MIN_INTENSITY}")

    near = rejected = blind = 0
    valid_fracs = []
    diffs = []
    for sc in scans:
        raw = list(sc.ranges)
        inten = list(sc.intensities)
        near += sum(1 for r in raw
                    if not scan_filter._bad(r) and r < scan_filter.NEAR_BAND)
        out, st = scan_filter.clean(raw, inten)
        rejected += st['rejected']
        valid_fracs.append(st['valid_frac'])
        if scan_filter.blinded(st):
            blind += 1
        zr = zone_minima(raw, sc.angle_min, sc.angle_increment)
        zf = zone_minima(out, sc.angle_min, sc.angle_increment)
        for z, a in zr.items():
            b = zf.get(z, 9999.0)
            if abs(b - a) > 0.02:
                diffs.append((z, a, b))

    print(f"\nrays returning anything: "
          f"{100*sum(valid_fracs)/len(valid_fracs):.1f}% average "
          f"({100*min(valid_fracs):.1f}% worst)")
    print(f"near-band returns: {near}   rejected: {rejected}"
          f"   ({100*rejected/max(1, near):.2f}%)")
    print(f"scans flagged sun-blinded: {blind}/{len(scans)}")

    closer = [d for d in diffs if d[2] < d[1]]
    critical = [d for d in diffs if d[1] < TRIGGER_BAND]
    print(f"\nzone minima changed: {len(diffs)} of {6*len(scans)}")
    print(f"  made CLOSER (must be 0):                  {len(closer)}")
    print(f"  changed INSIDE the trigger band (<{TRIGGER_BAND}m): "
          f"{len(critical)}")
    for z, a, b in diffs[:10]:
        flag = "  <-- INSIDE TRIGGER BAND" if a < TRIGGER_BAND else ""
        print(f"    {z:8s} {a:.2f}m -> {b:.2f}m{flag}")

    if args.show_rejected:
        print("\nrejected near returns, with their neighbourhoods:")
        shown = 0
        for sc in scans:
            raw = list(sc.ranges)
            inten = list(sc.intensities)
            out, _ = scan_filter.clean(raw, inten)
            n = len(raw)
            for k in range(n):
                if scan_filter._bad(raw[k]) or raw[k] >= scan_filter.NEAR_BAND:
                    continue
                if not scan_filter._bad(out[k]):
                    continue
                if shown >= 10:
                    break
                shown += 1
                print(f"  ray {k}: {raw[k]:.3f}m intensity="
                      f"{inten[k] if inten else '-'}")
                cells = []
                for d in range(-6, 7):
                    j = (k + d) % n
                    v = "inf" if scan_filter._bad(raw[j]) else f"{raw[j]:.2f}"
                    cells.append(f"{'>' if d == 0 else ''}{v}"
                                 f"@{int(inten[j]) if inten else '-'}")
                print("     " + " ".join(cells))
            if shown >= 10:
                break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
