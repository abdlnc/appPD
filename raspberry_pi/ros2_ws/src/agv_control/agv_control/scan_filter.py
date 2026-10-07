#!/usr/bin/env python3
"""
Sunlight / stray-return filter for the SLLIDAR C1 scan.

WHY THIS EXISTS
---------------
Direct sunlight is broadband IR, which is exactly what the Lidar's own
receiver is looking for. In strong sun the unit does two things:

  1. returns rays that nothing actually reflected -- phantom points, usually
     weak and usually isolated, which the robot reads as an obstacle and
     swerves or stops for; and
  2. drops rays entirely, because the receiver is saturated -- a REAL
     obstacle then returns nothing at all.

Only (1) can be filtered: a phantom can be recognised and thrown away.
(2) cannot -- no filter invents a reading that never arrived -- so it is
DETECTED and reported instead, via blinded() below. That distinction
matters: a filter that quietly hid (2) would be worse than no filter,
because the robot would drive confidently through a scan it cannot see in.

HOW A PHANTOM IS TOLD FROM AN OBSTACLE
--------------------------------------
Two independent tests, both of which a real obstacle passes easily:

  * SPATIAL AGREEMENT (the main one). The C1 puts out 720 rays over 360
    degrees, i.e. one every 0.5 degrees. Anything solid spans several of
    them: a 10cm obstacle at 0.5m subtends ~11 degrees, about 23 rays; even
    a 2cm cable subtends ~4 rays. A sun phantom is typically one ray, or a
    few scattered ones that disagree with each other. So a near return is
    kept only if at least MIN_AGREE other valid returns within
    WINDOW_RAYS agree with it to within NEIGHBOUR_TOL. This costs nothing
    in reaction time -- it is decided within the same scan.

    CRUCIALLY, a DROPPED ray neither supports nor vetoes. This unit drops
    around 58% of its rays as a matter of course, even indoors with no sun
    in sight, so a real surface arrives as valid returns interleaved with
    gaps. An earlier version of this filter walked outwards and stopped at
    the first gap; measured against live scans it threw away a quarter of
    all genuine near returns and pushed the front zone from 0.48m to
    1.62m -- i.e. it hid real obstacles. The window is searched, not
    walked, for exactly that reason.

    This is where the filter's one real cost sits. The old code treated a
    SINGLE return as an obstacle, so any filter at all is a reduction in
    sensitivity. MIN_AGREE = 1 is the mildest setting that still removes a
    lone phantom: an object has to produce two agreeing returns, which at
    this unit's hit rate a 2cm cable at 0.5m still manages. Raising
    MIN_AGREE rejects more phantoms and more thin obstacles, in that
    order.

  * RETURN STRENGTH, but NOT inside the trigger distance. The C1 reports
    a per-ray quality in `intensities` (measured on this unit: 0-11, and
    always 0 for a dropped ray), and returns below MIN_INTENSITY are
    dropped as too weak to believe.

    This test is deliberately SWITCHED OFF closer than TRUST_BAND. The
    reason is that `intensities` is not calibrated reflectivity: a black
    matte surface, or any surface seen at a grazing angle, comes back weak
    no matter how close it is. Measured on live scans, the rejections this
    test made inside the trigger band were adjacent PAIRS of rays at 0.70
    and 0.75m with intensity 0-1 and dropouts either side -- which is
    exactly what a thin or dark object at a glancing angle looks like, and
    also what a phantom looks like. The two are not separable by strength.

    So inside TRUST_BAND the robot keeps anything that has a corroborating
    neighbour, strong or weak, and only genuinely isolated rays are
    dropped. Beyond TRUST_BAND -- where a phantom causes a nuisance
    CAUTION rather than a missed collision -- strength is applied. The
    asymmetry is the point: a false stop costs a few seconds, a missed
    obstacle costs the robot.

DELIBERATELY NOT DONE: temporal persistence (requiring a return to appear
in N consecutive scans). It is the obvious third test and it was left out
on purpose -- at 10Hz it would add 100-200ms to every genuine stop, and
the spatial test already removes isolated noise without costing anything.
If sun phantoms are ever seen arriving as wide, strong, stable clusters,
revisit this -- but measure first.

SCOPE: only returns closer than NEAR_BAND are examined. Far returns do not
drive any decision, and filtering them would mean rejecting the sparse,
weak-but-real returns off distant ground for no benefit. Nothing here
modifies /scan itself, so slam_toolbox keeps receiving the raw scan, which
is what its own mapping filters expect.

THRESHOLDS ARE MEASURED INDOORS, NOT IN SUN. The geometry argument for
MIN_NEIGHBOURS holds anywhere, but MIN_INTENSITY comes from this unit's
indoor distribution. If phantoms still get through in direct sun, raise
MIN_INTENSITY first and re-check with ~/scan_stats.py.
"""

import math

# ----------------------------- tuning -----------------------------
# How far either side to look for corroboration, in rays. 6 rays = 3
# degrees at this unit's 0.5-degree spacing, which spans ~5cm at 0.5m.
WINDOW_RAYS = 6

# How many OTHER valid returns in that window must agree before a near
# return is believed. See the spatial-agreement note in the header for why
# this is 1 and not more.
MIN_AGREE = 1

# How closely those neighbours must agree, in metres. Wide enough to follow
# a surface seen at a steep angle, narrow enough that unrelated points do
# not vouch for each other.
NEIGHBOUR_TOL = 0.15

# Minimum reported return strength, applied only BEYOND TrustBand. 0-11
# on this unit; a dropped ray is always 0.
MIN_INTENSITY = 2

# Closer than this, return strength is ignored entirely and spatial
# agreement is the only test -- see the header. Set just above the widest
# avoidance trigger (OUTDOOR's 0.70m) so the whole decision-making band is
# covered by the conservative rule.
TRUST_BAND = 0.80

# Only returns closer than this are filtered at all (metres).
NEAR_BAND = 1.50

# Below this fraction of rays returning anything, the scan is treated as
# sun-blinded rather than as open space. Measured indoors with ~42% of rays
# valid; in an empty field the honest figure is far lower, so this is set
# low enough not to cry wolf on open ground.
BLIND_VALID_FRAC = 0.04


def _bad(r):
    return r is None or math.isinf(r) or math.isnan(r) or r <= 0.0


def clean(ranges, intensities=None, near_band=NEAR_BAND):
    """Return (cleaned_ranges, stats).

    cleaned_ranges is a new list with every rejected ray set to inf, so
    callers can treat it exactly like a raw scan -- a rejected ray is
    indistinguishable from one that never returned.

    stats = {'valid': n, 'total': n, 'rejected': n, 'valid_frac': f}
    """
    n = len(ranges)
    out = list(ranges)
    valid = rejected = 0
    if n == 0:
        return out, {'valid': 0, 'total': 0, 'rejected': 0, 'valid_frac': 0.0}

    has_int = intensities is not None and len(intensities) == n

    for k in range(n):
        r = ranges[k]
        if _bad(r):
            continue
        valid += 1
        if r >= near_band:
            continue                      # far: not our business

        # Inside TRUST_BAND, strength is not used at all -- neither to
        # reject this ray nor to disqualify the ones vouching for it.
        use_strength = has_int and r >= TRUST_BAND

        # --- strength test ---
        if use_strength and intensities[k] < MIN_INTENSITY:
            out[k] = math.inf
            rejected += 1
            continue

        # --- spatial agreement test ---
        # Search the window either side for valid returns that agree. Gaps
        # are skipped, not treated as disagreement -- see the header. The
        # scan wraps at 360, hence the modulo.
        # A corroborating ray must itself be strong enough to be believed.
        # Without this, two weak phantoms that happen to land near each
        # other vouch for one another and both survive -- measured: a ray
        # rejected for intensity was still propping up its neighbour.
        agree = 0
        for d in range(1, WINDOW_RAYS + 1):
            for direction in (-1, 1):
                j = (k + direction * d) % n
                nb = ranges[j]
                if _bad(nb) or abs(nb - r) > NEIGHBOUR_TOL:
                    continue
                if use_strength and intensities[j] < MIN_INTENSITY:
                    continue
                agree += 1
            if agree >= MIN_AGREE:
                break
        if agree < MIN_AGREE:
            out[k] = math.inf
            rejected += 1

    return out, {
        'valid': valid,
        'total': n,
        'rejected': rejected,
        'valid_frac': (valid / n) if n else 0.0,
    }


def blinded(stats):
    """True when so few rays returned anything that the scan cannot be
    trusted as 'clear'. This is the un-filterable failure in the header:
    the robot is not seeing open space, it is not seeing AT ALL."""
    return stats['total'] > 0 and stats['valid_frac'] < BLIND_VALID_FRAC
