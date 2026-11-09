#!/usr/bin/env python3
"""
nav_dead_reckon.py — how far INAV's IMU dead-reckons, and a replay of INAV's no-GPS velocity rule (045 PRE-TASK N1).

Reads `align.npz` from nav_accel_fit.py. Two parts:
1. Sliding windows over the airborne interval: start from GPS velocity (de-lagged), integrate the earth-frame IMU acceleration for
   `--len` s, compare with GPS at the end. Per window: error at 0° heading offset, the best heading offset and its error, max |pitch|,
   mean |specific force|. Binned by max |pitch|: if error grows with manoeuvre severity and the best offset scatters, the attitude
   solution is the limit, not a fixed misalignment.
2. `--replay T0 T1`: through a GPS-out interval, GPS truth vs IMU integration from GPS vs INAV's rule
   (vel += accNEU·dt − vel·w_xy_res_v·dt / accWeight, navigation_pos_estimator.c) vs the logged `navVel`. If the replay tracks
   `navVel`, the model of what INAV did is right.

11-08 (044 flight-analysis § 6): 2.3 s error p50 2.9 → 11 m/s as max pitch goes < 30° → > 70°; the 378.47–380.76 window averaged
4.4 g, IMU +14 m/s east vs GPS +23 north (best offset −85°); the replay tracked `navVel` within ~2–3 m/s.

Usage: nav_dead_reckon.py DECODE_DIR [--len 2.3] [--step 1.0] [--replay T0 T1] [--res-v 0.5]   (INAV seconds)
"""
import argparse

import numpy as np

G0 = 9.80665


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("dir")
    ap.add_argument("--len", type=float, default=2.3, help="window length, s (default 2.3: the 11-08 freeze)")
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--replay", nargs=2, type=float, metavar=("T0", "T1"))
    ap.add_argument("--res-v", type=float, default=0.5, help="inav_w_xy_res_v of the flown config (default 0.5)")
    args = ap.parse_args()
    z = np.load(f"{args.dir}/align.npz")
    t, acc, att, navV, w5, gt, gv = z["t"], z["acc"], z["att"], z["navV"], z["w5"], z["gt"], z["gv"]
    rs, ps, lag, air = int(z["rs"]), int(z["ps"]), float(z["lag"]), z["air"]
    dt = np.r_[np.diff(t), np.median(np.diff(t))]
    gvi = lambda tt, k: np.interp(tt - lag, gt, gv[:, k])
    pitch = np.degrees(ps * att[:, 1])
    f = np.linalg.norm(acc, axis=1)

    def ne(yoff):
        r, p, y = rs * att[:, 0], ps * att[:, 1], att[:, 2] + yoff
        fx, fy, fz = acc[:, 0], -acc[:, 1], -acc[:, 2]
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.c_[cy * cp * fx + (cy * sp * sr - sy * cr) * fy + (cy * sp * cr + sy * sr) * fz,
                     sy * cp * fx + (sy * sp * sr + cy * cr) * fy + (sy * sp * cr - cy * sr) * fz] * G0

    offs = np.arange(-180, 180, 5)
    NE = {o: ne(np.radians(o)) for o in offs}

    def window(t0, T):
        m = (t >= t0) & (t < t0 + T)
        dv = np.array([gvi(t0 + T, 0) - gvi(t0, 0), gvi(t0 + T, 1) - gvi(t0, 1)])
        e = np.array([np.linalg.norm((NE[o][m] * dt[m, None]).sum(0) - dv) for o in offs])
        j = int(np.argmin(e))
        return e[list(offs).index(0)], offs[j], e[j], np.abs(pitch[m]).max(), f[m].mean()

    W = np.array([(t0,) + window(t0, args.len) for t0 in np.arange(air[0], air[1] - args.len, args.step)])
    print(f"IMU-only dead reckoning, {args.len} s windows ({len(W)}): horizontal velocity error p50 {np.median(W[:, 1]):.1f}"
          f"  p90 {np.percentile(W[:, 1], 90):.1f}  max {W[:, 1].max():.1f} m/s")
    for lo, hi in ((0, 30), (30, 50), (50, 70), (70, 91)):
        m = (W[:, 4] >= lo) & (W[:, 4] < hi)
        if m.any():
            print(f"  max|pitch| {lo:2d}-{hi:2d}°: n {m.sum():3d}  err p50 {np.median(W[m, 1]):5.1f}  p90 {np.percentile(W[m, 1], 90):5.1f}"
                  f"  best offset p50 {np.median(W[m, 2]):+4.0f}° (IQR {np.percentile(W[m, 2], 25):+.0f}..{np.percentile(W[m, 2], 75):+.0f})")
    print("worst windows (t0, err@0, best offset, err@best, max|pitch|, mean |f| g):")
    for r in W[np.argsort(-W[:, 1])][:8]:
        print("  %.1f  %5.1f  %+4.0f  %5.1f  %3.0f  %.2f" % tuple(r))

    if args.replay:
        t0, t1 = args.replay
        e0, bo, be, pm, gl = window(t0, t1 - t0)
        print(f"\nreplay {t0}–{t1}: err@0 {e0:.1f} m/s, best offset {bo:+.0f}° → {be:.1f}; max|pitch| {pm:.0f}°, mean |f| {gl:.2f} g")
        idx = np.nonzero((t >= t0) & (t <= t1))[0]
        N, E = NE[0][:, 0], NE[0][:, 1]
        vI = navV[idx[0], :2].copy()
        vP = np.array([gvi(t0, 0), gvi(t0, 1)])
        print("     t   GPS N     E | IMU-from-GPS N     E | INAV rule N     E | navVel N     E | accW")
        nxt = t0
        for i in idx:
            a = np.array([N[i], E[i]])
            vP = vP + a * dt[i]
            vI = vI + a * dt[i] - vI * args.res_v * dt[i] / max(w5[i], 1e-3)
            if t[i] >= nxt:
                print(f"{t[i]:7.2f} {gvi(t[i], 0):6.1f} {gvi(t[i], 1):5.1f} | {vP[0]:14.1f} {vP[1]:5.1f} | {vI[0]:11.1f} {vI[1]:5.1f}"
                      f" | {navV[i, 0]:8.1f} {navV[i, 1]:5.1f} | {w5[i]:.2f}")
                nxt += 0.25


if __name__ == "__main__":
    main()
