#!/usr/bin/env python3
"""span_attitude.py — 045 N1 (tasks T024; research R3/R4): attitude over each engaged span, judged against GPS.

Per engaged span (mspOverrideFlags == 2 in the blackbox), two attitude sources are compared with a GPS-derived reference:
  inav  — INAV's logged attitude (attitude[0..2]);
  gyro  — gyro-only propagation of gyroADC from engage, initialised from INAV's attitude at engage (so both agree at t = 0).
Reference: the specific force measured by the accelerometer, rotated to the earth frame by the candidate attitude, must match the
specific force implied by GPS (Δ GPS_velned / Δt − g). Over a window of GPS intervals the best earth-frame correction rotation is
solved (Wahba / SVD); its angle is the source's attitude error. Under sustained manoeuvring the specific-force direction sweeps
through the earth frame, so all three axes are observable; `obs` (λmin/λmax of the information matrix) says how well.
The reference has its own uncertainty, reported per window as a 1-sigma angle from the fit residuals (10 Hz GPS differencing
noise, the fitted GPS lag, the 15 Hz accSmooth filter and accelerometer scale errors all land in the residual). Read an error
smaller than ~2 sigma as "not distinguishable from zero".

⚠️ VALIDITY (found 2026-11-12): on the 11-08 and 10-18 flights the GPS-implied |specific force| falls behind the IMU's under load
(ratio ~1.0 at 2 g, ~0.8 at 3 g, ~0.7 at 4 g, ~0.55 above 5 g; attitude-independent, written to gps_vs_imu_load.csv). The
receiver flew gps_dyn_model = AIR_2G, which caps its dynamics at 2 g. So only GPS intervals with |f_imu| <= --max-g (default 2 g)
enter the reference, over --window (default 4 s) windows. Errors are therefore measured at the low-load moments inside each span.

Also prints the span envelope (gross / net rotation, peak rates, |f| statistics; the research R3 table) and, with --spectrum, the
gyroRaw spectrum peaks (research R7: aliased at the 240 Hz blackbox rate).

The module doubles as the loader library for nav_dead_reckon.py --span and nav_span_scan.py --blackbox.

Usage (repo root):
  python3 specs/045-m2-tracking/span_attitude.py                      # all known flights -> specs/045-m2-tracking/n1/
  python3 specs/045-m2-tracking/span_attitude.py --flight 1108 --window 2.0 --out-dir /tmp/n1
  python3 specs/045-m2-tracking/span_attitude.py --stem path/to/blackbox_log_X.01    # any decoded log, spans auto-detected
Inputs: blackbox_decode output <stem>.csv and <stem>.gps.csv. The 10-18 second log needs
  blackbox_decode --index 2 flight-results/flight-20261018/blackbox_log_2026-10-18_125223.TXT
"""
import argparse
import csv
import os
import sys

import numpy as np

G0 = 9.80665
HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHTS = {  # key -> decoded blackbox stem (repo-relative)
    "1108": "flight-results/flight-20261108/blackbox_log_2026-11-08_160651.01",
    "1018a": "flight-results/flight-20261018/blackbox_log_2026-10-18_125223.01",
    "1018b": "flight-results/flight-20261018/blackbox_log_2026-10-18_125223.02",
}
# Known conventions (research R3; nav_accel_fit.py): INAV Euler roll +1 / pitch -1; gyro FLU -> FRD signs +1, -1, -1.
CONV = (1, -1, np.array([1.0, -1.0, -1.0]))
TAUS = (5.0, 10.0, 17.0)

BB_COLS = ["time (us)", "gyroADC[0]", "gyroADC[1]", "gyroADC[2]", "gyroRaw[0]", "gyroRaw[1]", "gyroRaw[2]",
           "accSmooth[0]", "accSmooth[1]", "accSmooth[2]", "attitude[0]", "attitude[1]", "attitude[2]",
           "navPos[0]", "navPos[1]", "navPos[2]", "navVel[0]", "navVel[1]", "navVel[2]", "navEPH", "navEPV", "navFlags",
           "mspOverrideFlags", "debug[5]"]


# ----------------------------------------------------------------------------------------------------------------- loaders
def load_bb(stem):
    """Blackbox main CSV -> dict of arrays (SI units; body vectors still in INAV FLU)."""
    with open(stem + ".csv") as f:
        r = csv.reader(f)
        h = [c.strip() for c in next(r)]
        ix = [h.index(c) for c in BB_COLS]
        rows = []
        for row in r:
            try:
                rows.append([float(row[i]) for i in ix])
            except (ValueError, IndexError):
                pass
    A = np.array(rows)
    return {
        "t": A[:, 0] / 1e6,
        "gyro": A[:, 1:4],            # deg/s, filtered (gyroADC), FLU
        "graw": A[:, 4:7],            # deg/s, unfiltered, FLU
        "acc": A[:, 7:10] / 2048.0,   # g, accSmooth (15 Hz BIQUAD), FLU
        "att": np.radians(A[:, 10:13] / 10.0),
        "navPos": A[:, 13:16] / 100.0,  # m, NEU
        "navVel": A[:, 16:19] / 100.0,  # m/s, NEU
        "eph": A[:, 19] / 100.0, "epv": A[:, 20] / 100.0,
        "navFlags": A[:, 21].astype(int), "msp": A[:, 22].astype(int),
        "accw": A[:, 23] / 1000.0,    # INAV accWeight (debug_mode = VIBE)
    }


def load_gps(stem):
    """GPS CSV -> dict; repeated G frames (< 50 ms apart) dropped. Position in m, local NED about the first fix."""
    with open(stem + ".gps.csv") as f:
        g = list(csv.DictReader(f, skipinitialspace=True))
    t = np.array([int(r["time (us)"]) for r in g]) / 1e6
    keep = np.r_[True, np.diff(t) > 0.05]
    col = lambda k, s=1.0: np.array([float(r[k]) for r in g])[keep] * s
    lat, lon, alt = col("GPS_coord[0]"), col("GPS_coord[1]"), col("GPS_altitude")
    ok = col("GPS_fixType") >= 2
    lat0, lon0, alt0 = lat[ok][0], lon[ok][0], alt[ok][0]
    R_E = 6371000.0
    pn = np.radians(lat - lat0) * R_E
    pe = np.radians(lon - lon0) * R_E * np.cos(np.radians(lat0))
    return {
        "t": t[keep], "fix": col("GPS_fixType"), "nsat": col("GPS_numSat"),
        "pos": np.c_[pn, pe, -(alt - alt0)],
        "vel": np.c_[col("GPS_velned[0]"), col("GPS_velned[1]"), col("GPS_velned[2]")] / 100.0,
        "eph": col("GPS_eph") / 100.0, "epv": col("GPS_epv") / 100.0,
    }


def engaged_spans(t, msp, min_len=3.0):
    """Runs with mspOverrideFlags == 2 (the override active), at least min_len s long."""
    on = (msp == 2).astype(int)
    d = np.diff(np.r_[0, on, 0])
    starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0] - 1
    return [(t[a], t[b]) for a, b in zip(starts, ends) if t[b] - t[a] >= min_len]


# ---------------------------------------------------------------------------------------------------------------- geometry
def euler_to_R(roll, pitch, yaw):
    """FRD body -> NED, ZYX Euler; works on scalars or arrays (returns (..., 3, 3))."""
    roll, pitch, yaw = np.asarray(roll, float), np.asarray(pitch, float), np.asarray(yaw, float)
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    R = np.empty(roll.shape + (3, 3))
    R[..., 0, 0] = cp * cy; R[..., 0, 1] = sr * sp * cy - cr * sy; R[..., 0, 2] = cr * sp * cy + sr * sy
    R[..., 1, 0] = cp * sy; R[..., 1, 1] = sr * sp * sy + cr * cy; R[..., 1, 2] = cr * sp * sy - sr * cy
    R[..., 2, 0] = -sp;     R[..., 2, 1] = sr * cp;                R[..., 2, 2] = cr * cp
    return R


def expm_so3_batch(W):
    th = np.linalg.norm(W, axis=1)
    K = np.zeros((len(W), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -W[:, 2], W[:, 1], -W[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = W[:, 2], -W[:, 1], W[:, 0]
    safe = th > 1e-12
    a = np.where(safe, np.sin(th) / np.where(safe, th, 1), 1.0)
    b = np.where(safe, (1 - np.cos(th)) / np.where(safe, th, 1) ** 2, 0.5)
    return np.eye(3) + a[:, None, None] * K + b[:, None, None] * (K @ K)


def propagate(t, w, R0, i0, i1):
    """Body rates w (rad/s, FRD) integrated from R0 at sample i0 to i1 (inclusive); returns (i1 - i0 + 1, 3, 3)."""
    dR = expm_so3_batch(0.5 * (w[i0:i1] + w[i0 + 1:i1 + 1]) * np.diff(t[i0:i1 + 1])[:, None])
    out = np.empty((i1 - i0 + 1, 3, 3))
    out[0] = R = R0
    for k in range(len(dR)):
        R = R @ dR[k]
        out[k + 1] = R
    return out


def rot_angle(R):
    c = (np.trace(R, axis1=-2, axis2=-1) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def flu_to_frd(v):
    return v * np.array([1.0, -1.0, -1.0])


# ---------------------------------------------------------------------------------------------------------- GPS reference
def gps_intervals(gps, lag, t_lo, t_hi):
    """GPS intervals usable as reference: both ends 3D fix, >= 5 sats, 50-250 ms long, a moving velocity.
    Returns (t_start, t_end) in IMU time (GPS de-lagged by `lag`) and f_n, the GPS specific force (NED, m/s^2)."""
    t, v = gps["t"] - lag, gps["vel"]
    dt = np.diff(t)
    a = np.diff(v, axis=0) / dt[:, None]
    good = ((gps["fix"][:-1] >= 2) & (gps["fix"][1:] >= 2) & (gps["nsat"][:-1] >= 5) & (gps["nsat"][1:] >= 5)
            & (dt > 0.05) & (dt < 0.25) & np.any(np.diff(v, axis=0) != 0, axis=1) & (np.abs(a).max(axis=1) < 80)
            & (t[:-1] >= t_lo) & (t[1:] <= t_hi))
    fn = a - np.array([0.0, 0.0, G0])
    return t[:-1][good], t[1:][good], fn[good]


def interval_means(t, X, ts, te):
    """Mean of rows of X over each [ts, te) (X sampled at t)."""
    c = np.cumsum(np.r_[np.zeros((1,) + X.shape[1:]), X], axis=0)
    i0 = np.searchsorted(t, ts)
    i1 = np.maximum(np.searchsorted(t, te), i0 + 1)
    i1 = np.minimum(i1, len(t))
    return (c[i1] - c[i0]) / (i1 - i0)[:, None]


MIN_INTERVALS = 10  # fewer usable GPS intervals in a window => no estimate (the fit is unstable)


def rotvec(C):
    th = np.arccos(np.clip((np.trace(C) - 1) / 2, -1, 1))
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([C[2, 1] - C[1, 2], C[0, 2] - C[2, 0], C[1, 0] - C[0, 1]])


def wahba(fn, p):
    """Best earth-frame rotation C with C p ~ fn (NED). Returns a dict: err (total angle), sig (1-sigma), tilt / sig_tilt
    (the part about horizontal axes), head / sig_head (signed part about down: a heading error), obs = lmin/lmax, n."""
    n = len(fn)
    nan = dict(err=np.nan, sig=np.nan, tilt=np.nan, sig_tilt=np.nan, head=np.nan, sig_head=np.nan, obs=np.nan, n=n)
    if n < MIN_INTERVALS:
        return nan
    B = fn.T @ p
    U, _, Vt = np.linalg.svd(B)
    C = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    r = fn - p @ C.T
    s2 = np.sum(r ** 2) / max(3 * n - 3, 1)
    q = p @ C.T
    info = np.sum(np.einsum("ni,nj->nij", q, q) * -1 + (np.sum(q ** 2, axis=1))[:, None, None] * np.eye(3), axis=0)
    ev = np.linalg.eigvalsh(info)
    cov = s2 * np.linalg.inv(info)
    phi = np.degrees(rotvec(C))
    sd = np.degrees(np.sqrt(np.diag(cov)))
    return dict(err=rot_angle(C), sig=np.degrees(np.sqrt(np.trace(cov))), tilt=np.hypot(phi[0], phi[1]),
                sig_tilt=np.hypot(sd[0], sd[1]), head=phi[2], sig_head=sd[2], obs=ev[0] / ev[-1], n=n)


def fit_lag(bb, gps, R_all, t_lo, t_hi):
    """GPS lag (s) that best matches INAV-attitude specific force to GPS over [t_lo, t_hi]."""
    fb = flu_to_frd(bb["acc"]) * G0
    pf = np.einsum("nij,nj->ni", R_all, fb)
    best = None
    for lag in np.arange(0.0, 0.451, 0.025):
        ts, te, fn = gps_intervals(gps, lag, t_lo, t_hi)
        p = interval_means(bb["t"], pf, ts, te)
        e = np.sqrt(np.mean(np.sum((p - fn) ** 2, axis=1)))
        if best is None or e < best[0]:
            best = (e, lag)
    return best[1], best[0]


# -------------------------------------------------------------------------------------------------------------- analysis
def load_table(key, bb, gps, lag, spans, R_inav):
    """|f_gps| / |f_imu| and the INAV-attitude residual binned by IMU load: where the GPS reference is valid (attitude-free ratio)."""
    fb = flu_to_frd(bb["acc"]) * G0
    ts, te, fn = gps_intervals(gps, lag, spans[0][0] - 60, spans[-1][1] + 30)
    fi = interval_means(bb["t"], fb, ts, te)
    p = interval_means(bb["t"], np.einsum("nij,nj->ni", R_inav, fb), ts, te)
    gi, gg = np.linalg.norm(fi, axis=1) / G0, np.linalg.norm(fn, axis=1) / G0
    out = []
    for lo, hi in ((0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.5), (3.5, 5.0), (5.0, 9.0)):
        m = (gi >= lo) & (gi < hi)
        if m.sum() < 5:
            continue
        k = np.sum(gg[m] * gi[m]) / np.sum(gi[m] ** 2)
        res = np.linalg.norm(fn[m] - p[m], axis=1)
        out.append([key, lo, hi, int(m.sum()), k, np.median(gg[m] / gi[m]), np.median(res)])
    return out


def analyse_flight(key, stem, window, spectrum, max_g):
    bb, gps = load_bb(stem), load_gps(stem)
    t = bb["t"]
    rs, ps, sg = CONV
    R_inav = euler_to_R(rs * bb["att"][:, 0], ps * bb["att"][:, 1], bb["att"][:, 2])
    w = np.radians(bb["gyro"]) * sg
    fb = flu_to_frd(bb["acc"]) * G0
    spans = engaged_spans(t, bb["msp"])
    air = (spans[0][0] - 60, spans[-1][1] + 30)
    lag, lag_rms = fit_lag(bb, gps, R_inav, *air)
    print(f"== {key} ({os.path.basename(stem)}): {len(spans)} engaged spans; GPS lag {lag:.3f} s "
          f"(INAV-attitude RMS {lag_rms:.2f} m/s^2 over the airborne interval)")
    rows, env = [], []
    for n, (a, b) in enumerate(spans, 1):
        i0, i1 = np.searchsorted(t, a), np.searchsorted(t, b)
        Rg = propagate(t, w, R_inav[i0], i0, i1)
        ts, te, fn = gps_intervals(gps, lag, a - window - 1.0, b)
        tm = 0.5 * (ts + te)
        lowg = np.linalg.norm(interval_means(t, fb, ts, te), axis=1) / G0 <= max_g

        def pred(Rs, j0):  # interval means of R f over the given attitude history (rows j0 ...)
            return interval_means(t[j0:j0 + len(Rs)], np.einsum("nij,nj->ni", Rs, fb[j0:j0 + len(Rs)]), ts, te)

        p_inav = pred(R_inav, 0)
        p_gyro = pred(Rg, i0)
        L = b - a
        evals = [("pre", None)] + [(f"{tau:g}", tau) for tau in TAUS] + [("end", L), ("span", L)]
        for label, tau in evals:
            if tau is None:
                m = (tm >= a - window - 0.5) & (tm < a - 0.5)
            elif tau > L + 1e-6:
                for src in ("inav", "gyro"):
                    rows.append([key, n, a, b, L, src, label, *([np.nan] * 10)])
                continue
            else:
                if label == "span":
                    lo, hi = 0.0, L
                elif label == "end":
                    lo, hi = L - window, L
                else:
                    lo, hi = tau - window / 2, tau + window / 2
                m = (tm >= a + max(lo, 0)) & (tm <= a + min(hi, L))
            m = m & lowg
            for src, p in (("inav", p_inav), ("gyro", p_gyro)):
                if tau is None and src == "gyro":
                    continue
                W = wahba(fn[m], p[m])
                d = np.degrees(np.arccos(np.clip(np.sum(fn[m] * p[m], 1) /
                                                 (np.linalg.norm(fn[m], axis=1) * np.linalg.norm(p[m], axis=1)), -1, 1)))
                dmed = np.median(d) if len(d) else np.nan
                gv = np.nan
                if tau is not None and label != "span":
                    j = min(np.searchsorted(t, a + tau), i1) - i0
                    gv = rot_angle(Rg[j].T @ R_inav[i0 + j])
                rows.append([key, n, a, b, L, src, label, W["err"], W["sig"], W["obs"], dmed, W["n"], gv,
                             W["tilt"], W["sig_tilt"], W["head"], W["sig_head"]])
        # envelope (research R3 columns)
        seg = bb["gyro"][i0:i1] * sg
        dt = np.diff(t[i0:i1 + 1])[:, None]
        f = np.linalg.norm(bb["acc"][i0:i1], axis=1)
        env.append([key, n, a, b, L, *np.sum(np.abs(seg) * dt, 0), *np.sum(seg * dt, 0), *np.abs(seg).max(0),
                    f.mean(), np.percentile(f, 95), np.mean(f > 2) * 100])
    if spectrum:
        fs = 1 / np.median(np.diff(t))
        for ax in range(3):
            P = None
            for (a, b) in spans:
                x = bb["graw"][np.searchsorted(t, a):np.searchsorted(t, b), ax]
                for s in range(0, len(x) - 512, 256):
                    seg = (x[s:s + 512] - x[s:s + 512].mean()) * np.hanning(512)
                    pw = np.abs(np.fft.rfft(seg)) ** 2
                    P = pw if P is None else P + pw
            fr = np.fft.rfftfreq(512, 1 / fs)
            mk = fr > 15
            top = np.argsort(P[mk])[-4:][::-1]
            print(f"  gyroRaw axis {ax} spectrum (fs {fs:.0f} Hz) top peaks > 15 Hz: {fr[mk][top].round(1)} Hz")
    return rows, env, lag, load_table(key, bb, gps, lag, spans, R_inav)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--flight", action="append", choices=sorted(FLIGHTS), help="known flight key (repeatable; default all)")
    ap.add_argument("--stem", action="append", default=[], help="any decoded log stem (<stem>.csv + <stem>.gps.csv)")
    ap.add_argument("--window", type=float, default=4.0, help="reference window length, s (default 4.0)")
    ap.add_argument("--max-g", type=float, default=2.0, help="GPS intervals above this IMU load are excluded (default 2.0 g)")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "n1"))
    ap.add_argument("--spectrum", action="store_true", help="also print gyroRaw spectrum peaks (research R7)")
    args = ap.parse_args()
    todo = [(k, FLIGHTS[k]) for k in (args.flight or ([] if args.stem else sorted(FLIGHTS)))]
    todo += [(os.path.basename(s), s) for s in args.stem]
    os.makedirs(args.out_dir, exist_ok=True)
    allrows, allenv, allload = [], [], []
    for key, stem in todo:
        if not os.path.exists(stem + ".csv"):
            sys.exit(f"span_attitude: missing {stem}.csv (decode the .TXT with blackbox_decode first)")
        rows, env, _, load = analyse_flight(key, stem, args.window, args.spectrum, args.max_g)
        allrows += rows
        allenv += env
        allload += load
        for L_ in load:
            print(f"  load {L_[1]:.1f}-{L_[2]:.1f} g: n {L_[3]:4d}  |f_gps|/|f_imu| fit {L_[4]:.2f} median {L_[5]:.2f}"
                  f"  INAV-attitude residual median {L_[6]:5.1f} m/s^2")
        for e in env:
            print(f"  span {e[1]} {e[4]:5.1f} s  |rot| r/p/y {np.round(e[5:8]).astype(int)}  net {np.round(e[8:11]).astype(int)}"
                  f"  max|rate| {np.round(e[11:14]).astype(int)} dps  |f| mean {e[14]:.2f} p95 {e[15]:.2f} g  >2g {e[16]:.0f} %")
        for r in rows:
            if r[5] == "inav" or r[6] == "pre":
                g = next((x for x in rows if x[1] == r[1] and x[6] == r[6] and x[5] == "gyro"), None)
                gtxt = "" if g is None else f"  gyro {g[7]:5.1f} ± {g[8]:4.1f} (tilt {g[13]:4.1f}, head {g[15]:+5.1f})"
                print(f"    span {r[1]} {r[6]:>4s}: inav {r[7]:5.1f} ± {r[8]:4.1f} (tilt {r[13]:4.1f} ± {r[14]:4.1f},"
                      f" head {r[15]:+5.1f} ± {r[16]:4.1f}){gtxt}  deg  (obs {r[9]:.2f}, n {r[11]},"
                      f" gyro-vs-inav {r[12]:5.1f})")
    hdr = ["flight", "span", "t0_s", "t1_s", "len_s", "source", "at", "err_deg", "sigma_deg", "obs", "dir_median_deg",
           "n_intervals", "gyro_vs_inav_deg", "tilt_deg", "sigma_tilt_deg", "heading_deg", "sigma_heading_deg"]
    with open(os.path.join(args.out_dir, "attitude_drift.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(hdr)
        for r in allrows:
            wr.writerow([f"{x:.3f}" if isinstance(x, float) else x for x in r])
    with open(os.path.join(args.out_dir, "span_envelope.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["flight", "span", "t0_s", "t1_s", "len_s", "gross_roll_deg", "gross_pitch_deg", "gross_yaw_deg",
                     "net_roll_deg", "net_pitch_deg", "net_yaw_deg", "max_rate_roll_dps", "max_rate_pitch_dps",
                     "max_rate_yaw_dps", "f_mean_g", "f_p95_g", "pct_time_over_2g"])
        for e in allenv:
            wr.writerow([f"{x:.3f}" if isinstance(x, float) else x for x in e])
    with open(os.path.join(args.out_dir, "gps_vs_imu_load.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["flight", "f_imu_lo_g", "f_imu_hi_g", "n_intervals", "gps_over_imu_fit", "gps_over_imu_median",
                     "inav_residual_median_mps2"])
        for e in allload:
            wr.writerow([f"{x:.3f}" if isinstance(x, float) else x for x in e])
    print(f"wrote {args.out_dir}/attitude_drift.csv, span_envelope.csv, gps_vs_imu_load.csv")


if __name__ == "__main__":
    main()
