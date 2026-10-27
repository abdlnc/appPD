#!/usr/bin/env python3
"""
Make costmaps from the saved SLAM maps.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    python3 ~/make_costmaps.py                 # every map in ~/agv_maps
    python3 ~/make_costmaps.py map_20261027_093533   # just one
    python3 ~/make_costmaps.py --force         # redo ones already made

For each ~/agv_maps/<name>.yaml (+ its .pgm) it writes, in ~/agv_costmaps:
    <name>.pgm   the costmap, one byte per cell (see VALUES)
    <name>.yaml  its metadata -- loads in Nav2's map_server / RViz as a map
    <name>.png   a coloured picture of it, 4x enlarged, for looking at

SETTINGS come from ~/agv_costmaps/costmap_params.yaml, in Nav2's own costmap
format: the robot's footprint and the inflation radius / fall-off. Change
them there, not here.

HOW: the same as Nav2's costmap with a static layer (the SLAM map) and an
inflation layer, done in plain Python -- Nav2's costmap package is not
installed on the robot. Grey (205) cells are UNKNOWN -- unexplored -- even
though the saved yaml's own thresholds would make them free (see
SAVER_UNKNOWN). Occupied map cells are LETHAL (254). Cells within
the robot's inscribed radius of one are INSCRIBED (253): its centre cannot
be there without the body touching. Out to inflation_radius the cost falls
off as 252 * exp(-cost_scaling_factor * (distance - inscribed radius)),
which is Nav2's InflationLayer formula. Unknown cells stay unknown (255)
unless they are inside the inscribed radius, as Nav2 does.

VALUES in the .pgm are what Nav2 publishes for a costmap as an occupancy
grid -- 0 free, 1-98 inflated, 99 inscribed, 100 lethal -- with 255 for
unknown, and the .yaml says "mode: raw" so map_server loads them as they
are (checked: pixels 0, 1, 50, 100, 255 load as 0, 1, 50, 100, -1).

Nothing in ~/agv_maps is changed.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

MAP_DIR = os.path.expanduser("~/agv_maps")
OUT_DIR = os.path.expanduser("~/agv_costmaps")
PARAMS = os.path.join(OUT_DIR, "costmap_params.yaml")
VIEW_SCALE = 4                    # the .png is enlarged this much

# Nav2 cost values (nav2_costmap_2d/cost_values.hpp)
FREE, INSCRIBED, LETHAL, NO_INFORMATION = 0, 253, 254, 255


# ------------------------------------------------------------ settings
def load_params(path=PARAMS):
    with open(path) as f:
        p = yaml.safe_load(f)["costmap"]["ros__parameters"]
    fp = p["footprint"]
    if isinstance(fp, str):
        fp = yaml.safe_load(fp)
    pad = float(p.get("footprint_padding", 0.0))
    infl = p["inflation_layer"]
    return {
        "footprint": [(float(x), float(y)) for x, y in fp],
        "padding": pad,
        "track_unknown": bool(p.get("track_unknown_space", True)),
        "inflation_radius": float(infl["inflation_radius"]),
        "cost_scaling_factor": float(infl["cost_scaling_factor"]),
    }


def footprint_radii(footprint, padding=0.0):
    """(inscribed, circumscribed) radius of a polygon about the origin, as
    Nav2 computes them: nearest edge and farthest corner."""
    n = len(footprint)
    inscribed = math.inf
    for i in range(n):
        (x1, y1), (x2, y2) = footprint[i], footprint[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(x1 * dx + y1 * dy) / L2))
        inscribed = min(inscribed, math.hypot(x1 + t * dx, y1 + t * dy))
    circumscribed = max(math.hypot(x, y) for x, y in footprint)
    return inscribed + padding, circumscribed + padding


# ------------------------------------------------------------ the map
# map_saver writes unknown cells as 205 -- and free_thresh 0.25 in the same
# yaml, by which 205 (0.196) counts as FREE. Checked 2026-10-27: Nav2's
# map_server loads all 19242 unknown cells of map_20261027_093533 as free.
# A costmap must not treat unexplored space as clear, so 205 is unknown here.
SAVER_UNKNOWN = 205


def read_map(yaml_path):
    """The SLAM map as an array: 1 occupied, 0 free, -1 unknown -- the
    yaml's thresholds, except map_saver's unknown value (SAVER_UNKNOWN)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"cannot read {img_path}")
    if meta.get("mode", "trinary") == "raw":
        occ = np.where(img == 255, -1,
                       np.where(img >= 65, 1, 0)).astype(np.int8)
        return occ, meta
    p = (img.astype(np.float32) / 255.0 if int(meta.get("negate", 0))
         else (255.0 - img.astype(np.float32)) / 255.0)
    occ = np.full(img.shape, -1, np.int8)
    occ[p > float(meta.get("occupied_thresh", 0.65))] = 1
    occ[p < float(meta.get("free_thresh", 0.25))] = 0
    if not int(meta.get("negate", 0)):
        occ[img == SAVER_UNKNOWN] = -1
    return occ, meta


# ------------------------------------------------------------ costs
def compute_costs(occ, resolution, params):
    """Nav2 costs (0-255) for each cell. See the module note."""
    inscribed, _ = footprint_radii(params["footprint"], params["padding"])
    radius = params["inflation_radius"]
    k = params["cost_scaling_factor"]

    lethal = occ == 1
    cost = np.zeros(occ.shape, np.uint8)
    if params["track_unknown"]:
        cost[occ == -1] = NO_INFORMATION
    if lethal.any():
        # distance (m) from each cell centre to the nearest occupied cell
        src = np.where(lethal, 0, 255).astype(np.uint8)
        dist = cv2.distanceTransform(src, cv2.DIST_L2,
                                     cv2.DIST_MASK_PRECISE) * resolution
        infl = np.zeros(occ.shape, np.float64)
        band = (dist > inscribed) & (dist <= radius)
        infl[band] = (INSCRIBED - 1) * np.exp(-k * (dist[band] - inscribed))
        infl = infl.astype(np.uint8)
        infl[dist <= inscribed] = INSCRIBED
        unknown = cost == NO_INFORMATION
        # Nav2 InflationLayer: never lowers a cost; overwrites unknown only
        # with INSCRIBED or more (inflate_unknown false)
        known = ~unknown
        cost[known] = np.maximum(cost[known], infl[known])
        over = unknown & (infl >= INSCRIBED)
        cost[over] = infl[over]
        cost[lethal] = LETHAL
    return cost


def to_occupancy(cost):
    """Nav2's costmap -> occupancy grid translation (Costmap2DPublisher),
    with -1 (unknown) stored as 255 for the .pgm."""
    table = np.zeros(256, np.uint8)
    for i in range(1, 253):
        table[i] = 1 + (97 * (i - 1)) // 251
    table[INSCRIBED] = 99
    table[LETHAL] = 100
    table[NO_INFORMATION] = 255             # -1 in the grid
    return table[cost]


def colour(cost):
    """A picture: white free, grey unknown, black lethal, red inscribed,
    yellow -> orange for the inflated band. North up."""
    h, w = cost.shape
    img = np.full((h, w, 3), 255, np.uint8)              # BGR
    img[cost == NO_INFORMATION] = (160, 160, 160)
    band = (cost > FREE) & (cost < INSCRIBED)
    if band.any():
        f = cost[band].astype(np.float32) / (INSCRIBED - 1)
        img[band] = np.stack([np.full_like(f, 40),             # B
                              (230 - 150 * f),                 # G
                              np.full_like(f, 255)], -1).astype(np.uint8)
    img[cost == INSCRIBED] = (40, 40, 220)
    img[cost == LETHAL] = (0, 0, 0)
    return cv2.resize(img, (w * VIEW_SCALE, h * VIEW_SCALE),
                      interpolation=cv2.INTER_NEAREST)


# ------------------------------------------------------------ files
def make_one(yaml_path, params, out_dir=OUT_DIR):
    occ, meta = read_map(yaml_path)
    res = float(meta["resolution"])
    cost = compute_costs(occ, res, params)
    name = os.path.splitext(os.path.basename(yaml_path))[0]
    pgm = os.path.join(out_dir, name + ".pgm")
    h, w = cost.shape
    with open(pgm, "wb") as f:                       # binary P5, raw values
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(to_occupancy(cost).tobytes())
    inscribed, circ = footprint_radii(params["footprint"], params["padding"])
    with open(os.path.join(out_dir, name + ".yaml"), "w") as f:
        f.write(
            f"# Costmap of ~/agv_maps/{name}.yaml, made by ~/make_costmaps.py\n"
            f"# from costmap_params.yaml: inscribed radius {inscribed:.2f} m,"
            f" inflation radius {params['inflation_radius']:.2f} m, cost"
            f" scaling {params['cost_scaling_factor']:g}.\n"
            f"# Pixel = occupancy as Nav2 publishes a costmap: 0 free, 1-98"
            f" inflated, 99 inscribed, 100 lethal, 255 unknown.\n"
            f"image: {name}.pgm\n"
            f"mode: raw\n"
            f"resolution: {res}\n"
            f"origin: {list(meta['origin'])}\n"
            f"negate: 0\n"
            f"occupied_thresh: {meta.get('occupied_thresh', 0.65)}\n"
            f"free_thresh: {meta.get('free_thresh', 0.25)}\n")
    # PNG: map row 0 is the top (north-up, as map_server reads it)
    cv2.imwrite(os.path.join(out_dir, name + ".png"), colour(cost))
    return cost


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*", help="map names (default: all)")
    ap.add_argument("--force", action="store_true",
                    help="remake costmaps that already exist")
    ap.add_argument("--maps", default=MAP_DIR)
    ap.add_argument("--out", default=OUT_DIR)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    params = load_params(os.path.join(a.out, "costmap_params.yaml"))
    inscribed, circ = footprint_radii(params["footprint"], params["padding"])
    print(f"robot: inscribed radius {inscribed:.2f} m, circumscribed "
          f"{circ:.2f} m; inflation radius {params['inflation_radius']:.2f}"
          f" m, cost scaling {params['cost_scaling_factor']:g}")
    if params["inflation_radius"] < circ:
        print("  note: inflation radius is less than the circumscribed "
              "radius -- a corner can touch where the cost reads 0")
    names = a.names or sorted(n[:-5] for n in os.listdir(a.maps)
                              if n.endswith(".yaml"))
    made = skipped = failed = 0
    for name in names:
        src = os.path.join(a.maps, name + ".yaml")
        if not a.force and os.path.exists(os.path.join(a.out, name + ".yaml")):
            skipped += 1
            continue
        try:
            cost = make_one(src, params, a.out)
            made += 1
            if a.names:
                n = cost.size
                print(f"  {name}: {cost.shape[1]}x{cost.shape[0]} cells -- "
                      f"lethal {np.sum(cost == LETHAL) / n:.1%}, inscribed "
                      f"{np.sum(cost == INSCRIBED) / n:.1%}, inflated "
                      f"{np.sum((cost > 0) & (cost < INSCRIBED)) / n:.1%}, "
                      f"unknown {np.sum(cost == NO_INFORMATION) / n:.1%}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  {name}: FAILED -- {e}", file=sys.stderr)
    print(f"made {made}, already there {skipped}"
          + (f", FAILED {failed}" if failed else "") + f"  ->  {a.out}")


if __name__ == "__main__":
    main()
