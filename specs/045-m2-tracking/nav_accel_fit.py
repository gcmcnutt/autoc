#!/usr/bin/env python3
"""
nav_accel_fit.py — INAV IMU vs GPS: earth-frame acceleration fit, vibration and accel weight (045 PRE-TASK N1).

Rotates the blackbox `accSmooth` (FLU, acc_1G 2048) to NED with INAV's `attitude[]` (decideg roll / pitch / heading), adds gravity,
and fits it against GPS-derived acceleration (Δ`GPS_velned` between G frames): roll / pitch sign convention and GPS lag by least
squares, then a heading-offset sweep and per-axis correlation / scale. A fixed board or mag misalignment shows as a non-zero best
offset. Also prints vibration (`debug[0..2]`), clip count (`debug[3]`) and INAV's accelerometer weight (`debug[5]`) per window — these
need the log flown with `debug_mode = VIBE` (6). Writes `align.npz` into the decode dir for nav_dead_reckon.py / nav_heading_check.py.

11-08 result (044 flight-analysis § 6): roll +1 / pitch −1, lag 0.15 s, best heading offset ≈ 0°, no clipping, accWeight at its
0.30 floor in every span.

Usage: blackbox_decode LOG.TXT  (→ LOG.01.csv, LOG.01.gps.csv)
       nav_accel_fit.py DECODE_DIR --log LOG.01 --air T0 T1 [--span T0 T1 ...] [--window T0 T1]   (INAV seconds)
"""
import argparse
import csv

import numpy as np

G0 = 9.80665


def load(d, log):
    with open(f"{d}/{log}.csv") as f:
        rd = csv.reader(f)
        h = [c.strip() for c in next(rd)]
        want = ["time (us)", "accSmooth[0]", "accSmooth[1]", "accSmooth[2]", "attitude[0]", "attitude[1]", "attitude[2]", "accVib",
                "debug[0]", "debug[1]", "debug[2]", "debug[3]", "debug[5]", "navVel[0]", "navVel[1]", "navVel[2]"]
        ix = [h.index(k) for k in want]
        A = np.array([[float(r[i]) for i in ix] for r in rd if len(r) >= len(h)])
    with open(f"{d}/{log}.gps.csv") as f:
        g = list(csv.DictReader(f, skipinitialspace=True))
    gt = np.array([int(r["time (us)"]) for r in g]) / 1e6
    gv = np.array([[float(r[f"GPS_velned[{k}]"]) for k in range(3)] for r in g]) / 100
    keep = np.r_[True, np.diff(gt) > 0.05]   # repeated G frames
    return A, gt[keep], gv[keep]


def earth_acc(acc, att, rs, ps, yoff=0.0):
    """Body specific force (g, FLU) → NED kinematic acceleration (m/s²), ZYX Euler from INAV attitude."""
    r, p, y = rs * att[:, 0], ps * att[:, 1], att[:, 2] + yoff
    fx, fy, fz = acc[:, 0], -acc[:, 1], -acc[:, 2]   # FLU → FRD
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    n = cy * cp * fx + (cy * sp * sr - sy * cr) * fy + (cy * sp * cr + sy * sr) * fz
    e = sy * cp * fx + (sy * sp * sr + cy * cr) * fy + (sy * sp * cr - cy * sr) * fz
    dn = -sp * fx + cp * sr * fy + cp * cr * fz
    return np.c_[n, e, dn] * G0 + np.array([0, 0, G0])


def at_gps(t, a, gt, lag):
    """Mean of `a` over each GPS interval, the GPS taken `lag` s late."""
    c = np.cumsum(np.r_[np.zeros((1, 3)), a], axis=0)
    i0 = np.searchsorted(t, gt[:-1] - lag)
    i1 = np.minimum(np.maximum(np.searchsorted(t, gt[1:] - lag), i0 + 1), len(t))
    return (c[i1] - c[i0]) / (i1 - i0)[:, None]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("dir")
    ap.add_argument("--log", default="bb.01", help="decoded log stem (default bb.01)")
    ap.add_argument("--air", nargs=2, type=float, required=True, metavar=("T0", "T1"), help="airborne interval, INAV s")
    ap.add_argument("--span", nargs=2, type=float, action="append", default=[], metavar=("T0", "T1"))
    ap.add_argument("--window", nargs=2, type=float, action="append", default=[], metavar=("T0", "T1"))
    args = ap.parse_args()
    A, gt, gv = load(args.dir, args.log)
    t, acc, att = A[:, 0] / 1e6, A[:, 1:4] / 2048.0, np.radians(A[:, 4:7] / 10.0)
    vib, clip, w5, navV = A[:, 8:11] / 100, A[:, 11], A[:, 12] / 1000, A[:, 13:16] / 100
    ga = np.diff(gv, axis=0) / np.diff(gt)[:, None]
    gtm = 0.5 * (gt[1:] + gt[:-1])
    air = (gtm > args.air[0]) & (gtm < args.air[1]) & (np.abs(ga).max(axis=1) < 40)

    best = None
    for rs in (1, -1):
        for ps in (1, -1):
            a = earth_acc(acc, att, rs, ps)
            for lag in np.arange(0, 0.45, 0.05):
                err = np.sqrt(np.mean((at_gps(t, a, gt, lag)[air] - ga[air]) ** 2, axis=0))
                if best is None or err.sum() < best[0]:
                    best = (err.sum(), rs, ps, lag, err)
    _, rs, ps, lag, err = best
    print(f"best convention: roll {rs:+d} pitch {ps:+d}, GPS lag {lag:.2f} s; RMS err N/E/D {np.round(err, 2)} m/s²")
    print("heading offset sweep (deg → horizontal RMS err, m/s²):")
    for yo in (-60, -30, -15, -8, -4, 0, 4, 8, 15, 30, 60, 180):
        m = at_gps(t, earth_acc(acc, att, rs, ps, np.radians(yo)), gt, lag)
        print(f"  {yo:+4d}: {np.sqrt(np.mean(np.sum((m[air, :2] - ga[air, :2]) ** 2, axis=1))):.2f}")
    m = at_gps(t, earth_acc(acc, att, rs, ps), gt, lag)
    for k, nm in enumerate("NED"):
        x, y = m[air, k], ga[air, k]
        print(f"  {nm}: corr {np.corrcoef(x, y)[0, 1]:.3f}  gps = {np.dot(x, y) / np.dot(x, x):.3f} × imu  "
              f"bias {np.mean(y - x):+.2f}  imu std {x.std():.2f}  gps std {y.std():.2f}")

    def stats(mask, label):
        print(f"{label:30s} vib x/y/z p50 {np.round(np.median(vib[mask], axis=0), 2)} p99 {np.round(np.percentile(vib[mask], 99, axis=0), 2)}"
              f"  accVib p50 {np.median(A[mask, 7]):.0f}  clip max {clip[mask].max():.0f}"
              f"  accWeight min/p50 {w5[mask].min():.2f}/{np.median(w5[mask]):.2f}")
    stats((t > args.air[0]) & (t < args.air[1]), "airborne")
    for s, e in args.window:
        stats((t > s) & (t < e), f"window {s:.1f}-{e:.1f}")
    for s, e in args.span:
        stats((t > s) & (t < e), f"span {s:.1f}-{e:.1f}")
    np.savez(f"{args.dir}/align.npz", t=t, acc=acc, att=att, navV=navV, w5=w5, gt=gt, gv=gv, rs=rs, ps=ps, lag=lag,
             air=np.array(args.air))
    print(f"wrote {args.dir}/align.npz")


if __name__ == "__main__":
    main()
