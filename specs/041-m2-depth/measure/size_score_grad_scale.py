#!/usr/bin/env python3
"""041 P0-3 — size `kScoreGradScale` from RECORDED ticks.

The last unsized constant in the new input vector. Sized the way `kAccelScale_g`
was sized from the +-11 g flight record: measure the realized distribution, then
pick a divisor that keeps the common regime in tanh's near-linear zone and lets
a genuine excursion reach ~1 without saturating. NOT a round number picked to
look tidy.

WHAT IS MEASURED
----------------
`SCORE_GRAD_*` is d(score)/d(position) in body axes, times the streak
multiplier. The Lorentzian is

    score = 1 / D,  D = 1 + (d/S)^2 + (theta_c/C)^2

with d = |chase - rabbit|, theta = acos(-along/d) the angle off the tail-chase
line, theta_c = min(theta, pi/2), S the directional distance scale
(behind/ahead) and C the cone half-angle.

Its world-frame gradient decomposes into two ORTHOGONAL parts:

    grad d     = u_hat                       (radial, away from the rabbit)
    grad theta = t_hat/L - (along/(d*L))*u_hat,  |grad theta| = 1/d   (tangential)

(orthogonality: grad_theta . u_hat = (along/d)/L - along/(d*L) = 0)

so the NORM needs only scalars the dmp already carries:

    |grad score| = (1/D^2) * sqrt( (2d/S^2)^2 + (2*theta_c/(C^2 * d))^2 )

The angle term vanishes where theta is clamped (theta >= pi/2, i.e. ahead of
the rabbit and off to the side), which is handled explicitly.

The three emitted slots are the BODY-FRAME COMPONENTS of that vector, so each
component is bounded by the norm. Sizing on the norm is therefore conservative:
no component can exceed what is reported here.

INPUT: a pathgen `dmp-dump --csv-only` CSV (columns dist, along, mult).
"""
import argparse
import csv
import math
import sys


def percentile(sorted_vals, q):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * q
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="pathgen per-tick CSV from dmp-dump --csv-only")
    ap.add_argument("--dist-scale-behind", type=float, default=7.0,
                    help="FitDistScaleBehind, m (autoc.ini)")
    ap.add_argument("--dist-scale-ahead", type=float, default=2.0,
                    help="FitDistScaleAhead, m (autoc.ini)")
    ap.add_argument("--cone-angle-deg", type=float, default=45.0,
                    help="FitConeAngleDeg (autoc.ini)")
    ap.add_argument("--streak-threshold", type=float, default=0.5,
                    help="FitStreakThreshold — used only to split the report")
    args = ap.parse_args()

    cone = math.radians(args.cone_angle_deg)
    half_pi = math.pi / 2.0

    grads = []          # |grad score| (unscaled, per metre)
    weighted = []       # |grad score| * streak multiplier  <- what the slot carries
    weighted_in = []    # ... restricted to in-envelope ticks (stpPt >= threshold)
    skipped = 0

    with open(args.csv, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                d = float(row["dist"])
                a = float(row["along"])
                mult = float(row["mult"])
                stp = float(row["stpPt"])
            except (KeyError, ValueError, TypeError):
                skipped += 1
                continue
            if not (d > 1e-6):
                skipped += 1
                continue

            cos_ang = max(-1.0, min(1.0, -a / d))
            theta = math.acos(cos_ang)
            clamped = theta >= half_pi
            theta_c = half_pi if clamped else theta

            S = args.dist_scale_behind if a <= 0.0 else args.dist_scale_ahead
            D = 1.0 + (d / S) ** 2 + (theta_c / cone) ** 2

            radial = 2.0 * d / (S * S)
            # d(theta)/dp has magnitude 1/d, and is zero once theta is clamped.
            tangential = 0.0 if clamped else 2.0 * theta_c / (cone * cone * d)
            g = math.sqrt(radial * radial + tangential * tangential) / (D * D)

            grads.append(g)
            weighted.append(g * mult)
            if stp >= args.streak_threshold:
                weighted_in.append(g * mult)

    if not weighted:
        print("no usable ticks", file=sys.stderr)
        return 1

    def report(name, vals):
        v = sorted(vals)
        print(f"\n{name}  (n={len(v)})")
        for q in (0.05, 0.25, 0.50, 0.75, 0.95, 0.99, 1.00):
            print(f"  p{int(q*100):>3} = {percentile(v, q):.5f}")
        print(f"  mean = {sum(v)/len(v):.5f}")

    print(f"ticks used  : {len(weighted)}   skipped: {skipped}")
    print(f"cone        : behind={args.dist_scale_behind} ahead={args.dist_scale_ahead} "
          f"coneDeg={args.cone_angle_deg}")
    report("|grad score|  (per metre, no multiplier)", grads)
    report("|grad score| x streak multiplier  <-- THE SLOT", weighted)
    if weighted_in:
        report(f"... restricted to in-envelope ticks (stpPt >= {args.streak_threshold})",
               weighted_in)

    w = sorted(weighted)
    p95 = percentile(w, 0.95)
    p99 = percentile(w, 0.99)
    mx = w[-1]
    print("\n--- sizing ---")
    print("kAccelScale_g precedent: divisor set so the common regime lands well")
    print("inside tanh's linear zone and a genuine excursion reaches ~1.4.")
    for cand in (0.05, 0.1, 0.2, 0.25, 0.5, 1.0):
        print(f"  scale={cand:<5} -> p50={percentile(w,0.5)/cand:6.3f} "
              f"p95={p95/cand:6.3f}  p99={p99/cand:6.3f}  max={mx/cand:7.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
