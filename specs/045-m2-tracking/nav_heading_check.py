#!/usr/bin/env python3
"""
nav_heading_check.py — AHRS heading vs GPS course in straight, level, unaccelerated flight (045 PRE-TASK N2, mag calibration).

Reads `align.npz` from nav_accel_fit.py. Picks 0.5 s blocks at GPS frames with groundspeed > --min-speed, |roll| < 12°, |pitch| < 15°,
| |f| − 1 g | < 0.35 and course change < 6°, then reports heading − course, binned by course. A mis-calibrated mag shows as a
course-dependent sinusoid (1st / 2nd harmonic); a constant offset is wind crab or a mag yaw misalignment. The harmonic fit needs
samples in ≥ 4 course sectors — fly a box or a circle at altitude for it. 11-08 had 22 samples in two sectors (offset −5…−10°):
inconclusive.

Usage: nav_heading_check.py DECODE_DIR [--min-speed 10]
"""
import argparse

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("dir")
    ap.add_argument("--min-speed", type=float, default=10.0)
    args = ap.parse_args()
    z = np.load(f"{args.dir}/align.npz")
    t, acc, att, gt, gv = z["t"], z["acc"], z["att"], z["gt"], z["gv"]
    rs, ps = int(z["rs"]), int(z["ps"])
    roll, pitch, hdg = np.degrees(rs * att[:, 0]), np.degrees(ps * att[:, 1]), np.degrees(att[:, 2]) % 360
    f = np.linalg.norm(acc, axis=1)
    gs = np.hypot(gv[:, 0], gv[:, 1])
    crs = np.degrees(np.arctan2(gv[:, 1], gv[:, 0])) % 360
    out = []
    for i in range(1, len(gt) - 1):
        m = (t >= gt[i] - 0.25) & (t < gt[i] + 0.25)
        if m.sum() < 50 or gs[i] < args.min_speed:
            continue
        if np.abs(roll[m]).max() > 12 or np.abs(pitch[m]).max() > 15 or np.abs(f[m] - 1).max() > 0.35:
            continue
        if abs(((crs[i + 1] - crs[i - 1] + 180) % 360) - 180) > 6:
            continue
        h = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(hdg[m]))))) % 360
        out.append((gt[i], h, crs[i], ((h - crs[i] + 180) % 360) - 180))
    if not out:
        print("no straight & level samples")
        return
    O = np.array(out)
    print(f"straight & level samples: {len(O)} (t {O[:, 0].min():.0f}–{O[:, 0].max():.0f} s)")
    print(f"heading − GPS course: median {np.median(O[:, 3]):+.1f}°, IQR {np.percentile(O[:, 3], 25):+.1f}..{np.percentile(O[:, 3], 75):+.1f}")
    sectors = 0
    for lo in range(0, 360, 45):
        m = (O[:, 2] >= lo) & (O[:, 2] < lo + 45)
        if m.any():
            sectors += 1
            print(f"  course {lo:3d}–{lo + 45:3d}: n {m.sum():3d}  offset median {np.median(O[m, 3]):+6.1f}°")
    if sectors < 4:
        print(f"harmonic fit skipped: {sectors} course sectors (need ≥ 4)")
        return
    c = np.radians(O[:, 2])
    X = np.c_[np.ones(len(c)), np.cos(c), np.sin(c), np.cos(2 * c), np.sin(2 * c)]
    k, *_ = np.linalg.lstsq(X, O[:, 3], rcond=None)
    print("fit: const %+.1f°  1st harmonic %.1f°  2nd harmonic %.1f°  residual RMS %.1f°"
          % (k[0], np.hypot(k[1], k[2]), np.hypot(k[3], k[4]), np.sqrt(np.mean((O[:, 3] - X @ k) ** 2))))


if __name__ == "__main__":
    main()
