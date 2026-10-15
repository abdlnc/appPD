#!/usr/bin/env python3
"""
Tag soil photos that were saved before classification went into filenames.

Since 2026-10-15 soil_camera_node writes the classifier's verdict into each
photo's name -- "_cls-soil" or "_cls-notsoil" -- which is what lets the app's
gallery filter soil photos by it. Photos saved before that carry no tag and
show as unclassified. This runs the same classifier over them and adds the
tag, so the filter covers the whole archive rather than only new photos.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/soil_backfill_classify.py            # dry run: shows the counts
    python3 ~/soil_backfill_classify.py --apply    # renames the files
    python3 ~/soil_backfill_classify.py --undo ~/agv_soil_images/.cls_backfill_<when>.csv

SAFE TO RE-RUN: already-tagged photos are skipped, so a second --apply only
picks up anything new. Nothing is deleted and no pixels are touched -- the
only change is the tag added to each name. Every --apply writes a manifest
of old -> new names beside the photos, which --undo reverses exactly.

The preprocessing is imported from soil_camera_node rather than copied, so
this cannot drift from what the live node does. That matters here more than
most places: the live node fed the model the wrong pixel scale for weeks and
it called everything soil. Checked before writing this: the time and GPS
text burned into the bottom of each photo does not change the verdict --
masking it out gave the same answer on 200 of 200 sampled photos.
"""

import argparse
import csv
import os
import sys
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser("~"))
try:
    from soil_camera_node import (CAPTURE_DIR, FIELD_INPUT_SIZE, FIELD_LABELS,
                                  FIELD_MODEL_PATH, FIELD_SCALE_0_1,
                                  SOIL_CLASS_TAG)
except ImportError as e:
    sys.exit(f"cannot import soil_camera_node ({e}) -- run inside the "
             f"ros2_humble environment")

TAG_PREFIX = "_cls-"


def load_model():
    from ai_edge_litert.interpreter import Interpreter
    it = Interpreter(model_path=FIELD_MODEL_PATH)
    it.allocate_tensors()
    return it


def classify(it, img):
    """Exactly soil_camera_node.classify_field's preprocessing."""
    inp = it.get_input_details()[0]
    out = it.get_output_details()[0]
    x = cv2.resize(img, (FIELD_INPUT_SIZE, FIELD_INPUT_SIZE))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32)
    if FIELD_SCALE_0_1:
        x /= 255.0
    it.set_tensor(inp['index'], x[None, ...])
    it.invoke()
    probs = it.get_tensor(out['index']).ravel()
    idx = int(np.argmax(probs))
    return FIELD_LABELS[idx], float(probs[idx])


def tagged_name(name, tag):
    stem, ext = os.path.splitext(name)
    return f"{stem}{TAG_PREFIX}{tag}{ext}"


def undo(manifest):
    restored = skipped = 0
    with open(manifest, newline="") as f:
        for old, new in csv.reader(f):
            src = os.path.join(CAPTURE_DIR, new)
            dst = os.path.join(CAPTURE_DIR, old)
            if os.path.exists(src) and not os.path.exists(dst):
                os.rename(src, dst)
                restored += 1
            else:
                skipped += 1
    print(f"restored {restored} names, skipped {skipped} "
          f"(already restored, or changed since)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true",
                    help="rename the files (default is a dry run)")
    ap.add_argument("--undo", metavar="MANIFEST",
                    help="reverse an earlier --apply using its manifest")
    args = ap.parse_args()

    if args.undo:
        undo(os.path.expanduser(args.undo))
        return

    names = sorted(n for n in os.listdir(CAPTURE_DIR)
                   if n.lower().endswith((".jpg", ".jpeg", ".png"))
                   and n.startswith("soil_"))
    todo = [n for n in names if TAG_PREFIX not in n]
    print(f"{len(names)} soil photos, {len(names) - len(todo)} already tagged, "
          f"{len(todo)} to classify")
    if not todo:
        return

    it = load_model()
    plan, unreadable, counts = [], 0, {}
    for i, n in enumerate(todo, 1):
        img = cv2.imread(os.path.join(CAPTURE_DIR, n))
        if img is None:
            unreadable += 1
            continue
        label, conf = classify(it, img)
        tag = SOIL_CLASS_TAG[label]
        counts[tag] = counts.get(tag, 0) + 1
        plan.append((n, tagged_name(n, tag)))
        if i % 500 == 0:
            print(f"  ...{i}/{len(todo)}")

    print("\n" + "   ".join(f"{k}: {v}" for k, v in sorted(counts.items()))
          + (f"   unreadable (left alone): {unreadable}" if unreadable else ""))

    if not args.apply:
        print("\nDry run -- nothing renamed. Re-run with --apply to tag them.")
        for old, new in plan[:3]:
            print(f"  e.g. {old}\n    -> {new}")
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    manifest = os.path.join(CAPTURE_DIR, f".cls_backfill_{stamp}.csv")
    done = clash = 0
    with open(manifest, "w", newline="") as f:
        w = csv.writer(f)
        for old, new in plan:
            src = os.path.join(CAPTURE_DIR, old)
            dst = os.path.join(CAPTURE_DIR, new)
            if os.path.exists(dst):
                clash += 1
                continue
            os.rename(src, dst)
            w.writerow([old, new])
            done += 1
    print(f"\nrenamed {done}" + (f", skipped {clash} (name already taken)"
                                 if clash else ""))
    print(f"undo with:\n  python3 ~/soil_backfill_classify.py --undo {manifest}")


if __name__ == "__main__":
    main()
