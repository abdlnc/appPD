#!/usr/bin/env python3
"""
Unit tests for ~/make_costmaps.py -- costmaps from the SLAM maps.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    python3 ~/costmap_test.py

No robot, no ROS: a hand-made map with one obstacle, checked against the
Nav2 formula by hand.
"""
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.expanduser("~"))
import make_costmaps as mc                     # noqa: E402

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


RES = 0.05
P = {"footprint": [(0.40, 0.30), (0.40, -0.30), (-0.40, -0.30),
                   (-0.40, 0.30)],
     "padding": 0.0, "track_unknown": True,
     "inflation_radius": 0.55, "cost_scaling_factor": 3.0}

print("-- the robot")
ins, circ = mc.footprint_radii(P["footprint"])
chk("60 x 80 cm, centred: inscribed radius 0.30 m", abs(ins - 0.30) < 1e-9)
chk("circumscribed radius 0.50 m (centre to a corner)",
    abs(circ - 0.50) < 1e-9)
chk("the shipped settings read back the same",
    mc.load_params()["footprint"] == P["footprint"] and
    mc.load_params()["inflation_radius"] == 0.55)

print("\n-- one obstacle in the middle of a free 41 x 41 map")
occ = np.zeros((41, 41), np.int8)
occ[20, 20] = 1
c = mc.compute_costs(occ, RES, P)
chk("the obstacle itself: LETHAL 254", c[20, 20] == 254)
chk("0.30 m away (6 cells): INSCRIBED 253", c[20, 26] == 253)
want = int(252 * math.exp(-3.0 * (0.40 - 0.30)))
chk("0.40 m away: 252 * exp(-3 * 0.10)", c[20, 28] == want,
    f"{c[20, 28]} vs {want}")
chk("costs fall with distance", c[20, 26] > c[20, 28] > c[20, 30] > 0)
chk("0.60 m away (beyond 0.55): free", c[20, 32] == 0)
diag = int(252 * math.exp(-3.0 * (math.hypot(5, 5) * RES - 0.30)))
chk("the same in every direction, and round (diagonal by true distance)",
    c[20, 28] == c[28, 20] == c[12, 20] and abs(int(c[25, 25]) - diag) <= 1,
    f"diagonal 0.35 m: {c[25, 25]} vs {diag}")

print("\n-- unknown space")
occ2 = np.zeros((41, 41), np.int8)
occ2[:, 30:] = -1                              # right side unknown
occ2[20, 27] = 1                               # obstacle near it
c2 = mc.compute_costs(occ2, RES, P)
chk("unknown far from obstacles stays unknown (255)", c2[0, 40] == 255)
chk("unknown inside the inscribed radius becomes INSCRIBED",
    c2[20, 30] == 253)
chk("unknown in the outer band stays unknown (inflate_unknown false)",
    c2[20, 34] == 255)
chk("unknown never spreads cost into free space", c2[0, 25] == 0)

print("\n-- how it is written")
t = mc.to_occupancy(np.array([0, 1, 126, 252, 253, 254, 255], np.uint8))
chk("translation as Nav2 publishes it: 0,1,..,98,99,100, unknown 255",
    list(t) == [0, 1, 1 + 97 * 125 // 251, 98, 99, 100, 255], str(list(t)))

with tempfile.TemporaryDirectory() as d:
    img = np.full((41, 41), 254, np.uint8)     # free (white-ish) pgm
    img[20, 20] = 0                            # one occupied cell
    img[:, 35:] = 205                          # unknown strip
    with open(os.path.join(d, "m.pgm"), "wb") as f:
        f.write(b"P5\n41 41\n255\n" + img.tobytes())
    with open(os.path.join(d, "m.yaml"), "w") as f:
        f.write("image: m.pgm\nmode: trinary\nresolution: 0.05\n"
                "origin: [-1.0, -2.0, 0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    out = os.path.join(d, "out")
    os.makedirs(out)
    mc.make_one(os.path.join(d, "m.yaml"), P, out)
    with open(os.path.join(out, "m.pgm"), "rb") as f:
        data = f.read()
    px = np.frombuffer(data[-41 * 41:], np.uint8).reshape(41, 41)
    chk("occupied -> 100, free -> 0, map_saver's grey 205 -> UNKNOWN",
        px[20, 20] == 100 and px[0, 0] == 0 and px[0, 40] == 255,
        f"{px[20, 20]} {px[0, 0]} {px[0, 40]}")
    meta = open(os.path.join(out, "m.yaml")).read()
    chk("yaml: raw mode, same resolution and origin as the SLAM map",
        "mode: raw" in meta and "resolution: 0.05" in meta and
        "origin: [-1.0, -2.0, 0]" in meta)
    chk("a picture is written too, 4x enlarged",
        os.path.getsize(os.path.join(out, "m.png")) > 0)

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
