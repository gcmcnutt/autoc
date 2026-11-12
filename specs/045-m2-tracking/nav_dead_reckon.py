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

3. `--span` (045 T025): a span-local propagator per engaged span. From engage, integrate the earth-frame specific force + gravity
   to velocity and position, with the attitude from INAV and, separately, from gyro-only propagation (span_attitude.py), starting
   from the de-lagged GPS state at engage. Compare with GPS at 5 / 10 / 17 s / end, and do the same for INAV's own estimate
   (navVel, navPos − navPos(engage)). ⚠️ The receiver (gps_dyn_model AIR_2G on these flights) smooths its velocity under load
   (span_attitude.py gps_vs_imu_load.csv), so GPS truth is taken at the nearest GPS sample within ±1.5 s of each time that has
   |f_imu| <= 2 g, a 3D fix and >= 5 sats; with none, the cell is empty. Writes n1/nav_drift.csv.

Usage: nav_dead_reckon.py DECODE_DIR [--len 2.3] [--step 1.0] [--replay T0 T1] [--res-v 0.5]   (INAV seconds)
       nav_dead_reckon.py --span [--flight 1108 ...] [--out specs/045-m2-tracking/n1/nav_drift.csv]
"""
import argparse
import csv
import os
import sys

import numpy as np

G0 = 9.80665
HERE = os.path.dirname(os.path.abspath(__file__))


def span_mode(flights, out, max_g=2.0, search=1.5):
    sys.path.insert(0, HERE)
    import span_attitude as sa
    rows = []
    for key in flights:
        stem = sa.FLIGHTS[key]
        bb, gps = sa.load_bb(stem), sa.load_gps(stem)
        t = bb["t"]
        rs, ps, sg = sa.CONV
        R_inav = sa.euler_to_R(rs * bb["att"][:, 0], ps * bb["att"][:, 1], bb["att"][:, 2])
        w = np.radians(bb["gyro"]) * sg
        fb = sa.flu_to_frd(bb["acc"]) * G0
        spans = sa.engaged_spans(t, bb["msp"])
        lag, _ = sa.fit_lag(bb, gps, R_inav, spans[0][0] - 60, spans[-1][1] + 30)
        gt = gps["t"] - lag                      # GPS sample i describes the truth at gt[i]
        gok = (gps["fix"] >= 2) & (gps["nsat"] >= 5)
        f_at_gps = np.linalg.norm(fb[np.clip(np.searchsorted(t, gt), 0, len(t) - 1)], axis=1) / G0
        nav_ned = lambda X: X * np.array([1.0, 1.0, -1.0])   # INAV NEU -> NED
        print(f"== {key}: GPS lag {lag:.3f} s")
        for n, (a, b) in enumerate(spans, 1):
            i0, i1 = np.searchsorted(t, a), np.searchsorted(t, b)
            j0 = np.argmin(np.abs(gt - t[i0]))
            if not gok[j0] or abs(gt[j0] - t[i0]) > 0.2:
                print(f"  span {n}: no valid GPS state at engage; skipped")
                continue
            v0 = np.array([np.interp(t[i0], gt, gps["vel"][:, k]) for k in range(3)])
            p0 = np.array([np.interp(t[i0], gt, gps["pos"][:, k]) for k in range(3)])
            dt = np.diff(t[i0:i1 + 1])
            Rg = sa.propagate(t, w, R_inav[i0], i0, i1)
            prop = {}
            for src, Rs in (("inav_att", R_inav[i0:i1 + 1]), ("gyro_att", Rg)):
                acc = np.einsum("nij,nj->ni", Rs, fb[i0:i1 + 1]) + np.array([0.0, 0.0, G0])
                v = v0 + np.r_[np.zeros((1, 3)), np.cumsum(0.5 * (acc[1:] + acc[:-1]) * dt[:, None], axis=0)]
                p = p0 + np.r_[np.zeros((1, 3)), np.cumsum(0.5 * (v[1:] + v[:-1]) * dt[:, None], axis=0)]
                prop[src] = (v, p)
            vI = nav_ned(bb["navVel"][i0:i1 + 1])
            pI = p0 + nav_ned(bb["navPos"][i0:i1 + 1] - bb["navPos"][i0])
            prop["inav_est"] = (vI, pI)
            L = b - a
            for label, tau in [(f"{x:g}", x) for x in (5.0, 10.0, 17.0)] + [("end", L)]:
                if tau > L + 1e-6:
                    continue
                cand = np.nonzero(gok & (f_at_gps <= max_g) & (np.abs(gt - (a + tau)) <= search) & (gt <= b) & (gt >= a))[0]
                if len(cand) == 0:
                    for src in prop:
                        rows.append([key, n, f"{L:.2f}", label, src, "", "", "", "", "", ""])
                    continue
                j = cand[np.argmin(np.abs(gt[cand] - (a + tau)))]
                k = min(np.searchsorted(t, gt[j]), i1) - i0
                vt, pt = gps["vel"][j], gps["pos"][j]
                for src, (v, p) in prop.items():
                    dv, dp = v[k] - vt, p[k] - pt
                    rows.append([key, n, f"{L:.2f}", label, src, f"{t[i0 + k] - a:.2f}",
                                 f"{np.hypot(dv[0], dv[1]):.2f}", f"{abs(dv[2]):.2f}",
                                 f"{np.hypot(dp[0], dp[1]):.1f}", f"{abs(dp[2]):.1f}", f"{gps['eph'][j]:.1f}"])
            for r in rows:
                if r[0] == key and r[1] == n and r[5] != "":
                    print(f"  span {n} {r[3]:>4s} (t {r[5]:>5s} s) {r[4]:9s}: |dv| h {r[6]:>6s} v {r[7]:>6s} m/s   "
                          f"|dp| h {r[8]:>6s} v {r[9]:>6s} m   (GPS eph {r[10]} m)")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["flight", "span", "len_s", "at", "source", "t_eval_s", "vel_err_h_mps", "vel_err_v_mps",
                     "pos_err_h_m", "pos_err_v_m", "gps_eph_m"])
        wr.writerows(rows)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("dir", nargs="?", help="decode dir with align.npz (window / replay modes)")
    ap.add_argument("--span", action="store_true", help="045 T025 span-local propagator (reads the CSVs directly)")
    ap.add_argument("--flight", action="append", help="span mode: flight key from span_attitude.FLIGHTS (default all)")
    ap.add_argument("--out", default=os.path.join(HERE, "n1", "nav_drift.csv"), help="span mode output CSV")
    ap.add_argument("--len", type=float, default=2.3, help="window length, s (default 2.3: the 11-08 freeze)")
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--replay", nargs=2, type=float, metavar=("T0", "T1"))
    ap.add_argument("--res-v", type=float, default=0.5, help="inav_w_xy_res_v of the flown config (default 0.5)")
    args = ap.parse_args()
    if args.span:
        sys.path.insert(0, HERE)
        import span_attitude as sa
        span_mode(args.flight or sorted(sa.FLIGHTS), args.out)
        return
    if not args.dir:
        ap.error("DECODE_DIR is required unless --span")
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
