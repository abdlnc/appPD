#!/usr/bin/env python3
"""
Unit tests for agv_control.scan_filter -- the Lidar sunlight filter.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/scan_filter_test.py

No robot, no Lidar and no ROS graph needed: every scan here is synthetic.
For the real-world counterpart -- what the filter does to actual scans,
which is the part that must be checked in sunlight -- use ~/scan_stats.py.

THE RULE THESE TESTS ENCODE is deliberately asymmetric. Inside TRUST_BAND
(the distances avoidance acts on) nothing corroborated may be discarded,
even a weak return, because a missed obstacle is the expensive failure.
Beyond it, phantoms are filtered hard, because there a phantom costs a
nuisance CAUTION rather than a collision. If a change makes the "KEPT by
design" tests fail, it has traded collision safety for a quieter warning
light -- which is the wrong direction.
"""

import sys, math, random

try:
    from agv_control import scan_filter as sf
except ImportError:
    sys.exit("cannot import agv_control.scan_filter -- "
             "source ~/ros2_ws/install/setup.bash first")


ok = True
def chk(label, cond, extra=""):
    global ok; ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:60s} {extra}")

N = 720
def blank(): return [math.inf]*N, [0]*N
def kept(out): return sum(1 for x in out if not sf._bad(x))

print(f"TRUST_BAND={sf.TRUST_BAND}m  NEAR_BAND={sf.NEAR_BAND}m  "
      f"MIN_AGREE={sf.MIN_AGREE}  MIN_INTENSITY={sf.MIN_INTENSITY}\n")

print("-- real obstacles must survive (the failure that matters)")
for w_cm, dist in ((10,0.50),(10,0.70),(5,0.60),(2,0.50),(2,0.70),(10,1.20)):
    r, inten = blank()
    half = math.degrees(math.atan((w_cm/200.0)/dist))
    rays = max(2, int(round(2*half/0.5)))
    for k in range(100, 100+rays): r[k] = dist; inten[k] = 5
    out, _ = sf.clean(r, inten)
    chk(f"{w_cm}cm at {dist:.2f}m ({rays} rays, strong): survives",
        kept(out) >= 1, f"kept {kept(out)}/{rays}")

print("\n-- inside the trust band, WEAK returns must still survive")
print("   (a dark or grazing-angle object reads weak however close it is)")
for dist in (0.30, 0.50, 0.70):
    r, inten = blank()
    r[300] = dist; r[301] = dist; inten[300] = 1; inten[301] = 1
    out, _ = sf.clean(r, inten)
    chk(f"2-ray intensity-1 pair at {dist:.2f}m: KEPT by design",
        kept(out) == 2, f"kept {kept(out)}/2")
r, inten = blank()
for k in range(200, 215): r[k] = 0.40; inten[k] = 1
out, _ = sf.clean(r, inten)
chk("wide intensity-1 cluster at 0.40m: KEPT by design", kept(out) == 15)
r, inten = blank()
for k in range(200, 206): r[k] = 0.40; inten[k] = 0
out, _ = sf.clean(r, inten)
chk("even intensity-0 cluster at 0.40m: KEPT by design", kept(out) == 6)

print("\n-- isolated rays are removed in BOTH bands")
for dist in (0.30, 0.50, 0.70, 1.00, 1.40):
    r, inten = blank(); r[300] = dist; inten[300] = 6
    out, _ = sf.clean(r, inten)
    chk(f"single strong ray at {dist:.2f}m, nothing near it: removed",
        kept(out) == 0)

print("\n-- phantom fields, measured per band")
def phantom_rate(lo, hi):
    k_tot = t_tot = 0
    for seed in range(200):
        random.seed(seed)
        r, inten = blank()
        for k in random.sample(range(N), 25):
            r[k] = random.uniform(lo, hi); inten[k] = random.choice([0,1,2,3])
        out, _ = sf.clean(r, inten)
        k_tot += kept(out); t_tot += 25
    return 100*(t_tot-k_tot)/t_tot
far = phantom_rate(sf.TRUST_BAND, sf.NEAR_BAND)
near = phantom_rate(0.20, sf.TRUST_BAND)
# Floors sit below the measured rates (96.3% / 84.3% at the time of
# writing) so these catch a regression without breaking on noise. The
# field is 25 phantoms in 720 rays -- denser than reality, which inflates
# the coincidence pairs that get through.
chk("beyond the trust band: >=95% removed", far >= 95.0, f"{far:.1f}%")
chk("inside the trust band: >=80% removed (spatial test only)",
    near >= 80.0, f"{near:.1f}% -- the rest are coincidence pairs")

print("\n-- far returns untouched")
r, inten = blank(); r[500] = 4.0; inten[500] = 0
out, _ = sf.clean(r, inten)
chk("isolated weak return at 4.0m: untouched", out[500] == 4.0)

print("\n-- wrap-around at the 0/360 seam")
r, inten = blank()
for k in (-2,-1,0,1,2): r[k % N] = 0.50; inten[k % N] = 6
out, _ = sf.clean(r, inten)
chk("cluster straddling the seam: kept",
    not sf._bad(out[0]) and not sf._bad(out[N-1]))

print("\n-- blinding detection")
r, inten = blank()
for k in range(10): r[k] = 2.0; inten[k] = 5
_, st = sf.clean(r, inten)
chk("10/720 rays returning = blinded", sf.blinded(st), f"{st['valid_frac']*100:.1f}%")
r, inten = blank()
for k in range(300): r[k] = 2.0; inten[k] = 5
_, st = sf.clean(r, inten)
chk("300/720 returning = not blinded", not sf.blinded(st), f"{st['valid_frac']*100:.1f}%")
_, st = sf.clean([], [])
chk("empty scan is not reported as blinded", not sf.blinded(st))

print("\n-- degrades safely with no intensity data at all")
r, _ = blank()
for k in range(200, 215): r[k] = 0.40
out, _ = sf.clean(r, None)
chk("no intensities: real cluster kept", kept(out) == 15)
r, _ = blank(); r[300] = 0.40
out, _ = sf.clean(r, None)
chk("no intensities: isolated ray removed", kept(out) == 0)

print("\n-- the cleaned scan is a drop-in replacement")
r, inten = blank()
for k in range(200, 215): r[k] = 0.40; inten[k] = 5
out, st = sf.clean(r, inten)
chk("same length as the input", len(out) == len(r))
chk("input list not mutated", r[300] == math.inf and len(r) == N)
chk("stats add up", st['total'] == N and st['valid'] == 15)

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
