#!/usr/bin/env python3
"""mount_survey.py — foam-tape mount resonances per board from a high-rate log (045 research R7, FR-007b; quickstart § 5).

Bench, props on, article restrained: blackbox at denom 1 (2 kHz), groups GYRO_RAW + ACC, throttle sweep idle → full over
~20 s, then three steady points. For each board's log this script:
  1. splits the log into 0.5 s Hann windows (50 % overlap), each with its mean throttle;
  2. finds the spectral peaks (summed over axes) in each window;
  3. separates FIXED modes (the same frequency across the throttle bins: mount / structure resonances) from MOTOR lines
     (frequency rising with throttle: prop / motor rotation and its harmonics).
Output: one table per board (kind, frequency or line, bins present, dominant axis, level).

  python3 mount_survey.py --input FC:fc_denom1.csv [--input XIAO:xiao_imu.csv --gyro-col gyro --throttle-col motor[0]]
  python3 mount_survey.py --selftest

⚠️ The committed flight logs run at 240 Hz (blackbox_rate_denom 8): everything above 120 Hz folds back (R7). The script warns
when fs < 1 kHz; such a run shows the aliasing, it does not measure mounts.
Columns: time (us); <gyro-col>[0..2] (default gyroRaw, °/s); optional <acc-col>[0..2]; the throttle column (default
motor[0], scaled by --throttle-range, default 1000:2000)."""
import argparse
import csv
import os
import sys
import tempfile

import numpy as np

AXES = ("roll", "pitch", "yaw")


def load(path, gyro_col, throttle_col):
    with open(path, newline="") as f:
        r = csv.reader(f)
        h = [c.strip() for c in next(r)]
        need = ["time (us)"] + [f"{gyro_col}[{i}]" for i in range(3)] + [throttle_col]
        miss = [n for n in need if n not in h]
        if miss:
            sys.exit(f"mount_survey: {path} lacks columns {miss}")
        ix = [h.index(n) for n in need]
        rows = []
        for row in r:
            try:
                rows.append([float(row[j]) for j in ix])
            except (ValueError, IndexError):
                continue
    d = np.array(rows)
    return d[:, 0] / 1e6, d[:, 1:4], d[:, 4]


def spectra(t, gyro, thr, win_s):
    fs = 1.0 / np.median(np.diff(t))
    n = int(round(win_s * fs))
    hop = max(1, n // 2)
    w = np.hanning(n)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    P, T = [], []
    for i0 in range(0, len(t) - n + 1, hop):
        seg = gyro[i0:i0 + n] - gyro[i0:i0 + n].mean(0)
        P.append(np.abs(np.fft.rfft(seg * w[:, None], axis=0)) ** 2)
        T.append(thr[i0:i0 + n].mean())
    return fs, freqs, np.array(P), np.array(T)


def find_peaks(p_db, freqs, fmin, fmax, prom_db=6.0, k=6):
    """Local maxima at least `prom_db` above a running median (±7 bins), between fmin and fmax; the k strongest."""
    n = len(p_db)
    med = np.array([np.median(p_db[max(0, i - 7):i + 8]) for i in range(n)])
    out = []
    for i in range(1, n - 1):
        if freqs[i] < fmin or freqs[i] > fmax:
            continue
        if p_db[i] > p_db[i - 1] and p_db[i] >= p_db[i + 1] and p_db[i] - med[i] >= prom_db:
            out.append((p_db[i] - med[i], i))
    return [i for _, i in sorted(out, reverse=True)[:k]]


def survey(t, gyro, thr_raw, thr_range, win_s=0.5, nbins=8, fmin=15.0):
    """Per-window peaks → MOTOR lines first (f rising linearly with throttle, >= 25 % across the span; seeded RANSAC on
    the (throttle, f) peak cloud, up to 3 lines), then FIXED modes among the remaining peaks (a frequency present
    across a wide throttle range)."""
    lo, hi = thr_range
    thr = np.clip((thr_raw - lo) / (hi - lo), 0, 1)
    fs, freqs, P, T = spectra(t, gyro, thr, win_s)
    df = freqs[1]
    res = {"fs": fs, "nyquist": fs / 2, "windows": len(T), "fixed": [], "lines": []}
    run = np.where(T > 0.05)[0]
    res["bins"] = len(run)  # windows with the motor running
    if len(run) < 6:
        return res
    pts = []  # (window, thr, f, level_db, axis)
    for wi in run:
        pa = P[wi]
        pdb = 10 * np.log10(pa.sum(1) + 1e-30)
        for i in find_peaks(pdb, freqs, fmin, 0.98 * fs / 2):
            pts.append((wi, T[wi], freqs[i], pdb[i], AXES[int(np.argmax(pa[i]))]))
    pts = np.array(pts, dtype=object)
    if len(pts) == 0:
        return res
    used = np.zeros(len(pts), bool)
    thr_span = np.ptp(T[run])
    tolf = lambda f: np.maximum(2 * df, 0.03 * f)
    # MOTOR lines: RANSAC on the remaining (throttle, f) points
    rng = np.random.default_rng(1)
    for _ in range(3):
        idx = np.where(~used)[0]
        if len(idx) < 6:
            break
        x = pts[idx, 1].astype(float); y = pts[idx, 2].astype(float)
        best, best_in = None, None
        for _ in range(3000):
            a, b = rng.choice(len(idx), 2, replace=False)
            if abs(x[a] - x[b]) < 0.1 * thr_span:
                continue
            slope = (y[b] - y[a]) / (x[b] - x[a])
            if slope <= 0:
                continue
            icpt = y[a] - slope * x[a]
            if slope * thr_span < 0.25 * (icpt + slope * np.mean(x)):
                continue  # a motor line moves >= 25 % across the throttle span; a mount mode does not
            inl = np.abs(y - (icpt + slope * x)) <= tolf(y)
            if best_in is None or inl.sum() > best_in.sum():
                best, best_in = (icpt, slope), inl
        if best_in is None or len(set(pts[idx[best_in], 0])) < 0.3 * len(run):
            break
        slope, icpt = np.polyfit(x[best_in], y[best_in], 1)
        r = np.corrcoef(x[best_in], y[best_in])[0, 1]
        axs = list(pts[idx[best_in], 4])
        res["lines"].append({"intercept": float(icpt), "slope": float(slope), "r": float(r),
                             "bins": len(set(pts[idx[best_in], 0])), "axis": max(set(axs), key=axs.count),
                             "level_db": float(pts[idx[best_in], 3].astype(float).max())})
        used[idx[best_in]] = True
    # FIXED: frequencies seen in >= 30 % of running windows, spread over >= half the throttle span
    fr = pts[:, 2].astype(float)
    for f0 in np.unique(np.round(fr / df) * df)[np.argsort(-np.bincount(np.searchsorted(np.unique(np.round(fr / df) * df),
                                                                        np.round(fr / df) * df)))]:
        m = (~used) & (np.abs(fr - f0) <= tolf(f0))
        wins = set(pts[m, 0])
        if len(wins) >= 0.3 * len(run) and np.ptp(pts[m, 1].astype(float)) >= 0.5 * thr_span:
            axs = list(pts[m, 4])
            res["fixed"].append({"freq": float(np.median(fr[m])), "spread": float(np.ptp(fr[m])), "bins": len(wins),
                                 "axis": max(set(axs), key=axs.count), "level_db": float(pts[m, 3].astype(float).max())})
            used |= m
    res["lines"].sort(key=lambda ln: ln["slope"])
    return res


def print_table(label, res, thr_range):
    print(f"\n== {label}: fs {res['fs']:.0f} Hz (Nyquist {res['nyquist']:.0f} Hz), {res['windows']} windows, "
          f"{res.get('bins', 0)} windows with the motor running")
    if res["fs"] < 1000:
        print(f"   ⚠️ fs < 1 kHz: content above {res['nyquist']:.0f} Hz is ALIASED (folded back); this does not measure the"
              f" mount (R7). Log at blackbox denom 1–2 on the bench.")
    if not res["fixed"] and not res["lines"]:
        print("   no fixed mode or motor line met the criteria (needs a throttle sweep with the motor running, props on)")
        return
    print(f"   {'kind':6s} {'frequency':34s} {'wins':>5s}  {'axis':6s} {'level dB':>8s}")
    for m in sorted(res["fixed"], key=lambda m: m["freq"]):
        print(f"   {'fixed':6s} {m['freq']:7.1f} Hz (spread {m['spread']:4.1f}){'':13s} {m['bins']:>5d}  {m['axis']:6s} "
              f"{m['level_db']:8.1f}")
    for k, ln in enumerate(res["lines"]):
        tag = "motor" if k == 0 else f"motor{k + 1}"
        txt = f"{ln['intercept']:.0f} + {ln['slope']:.0f}·thr Hz (r {ln['r']:.2f})"
        print(f"   {tag:6s} {txt:34s} {ln['bins']:>5d}  {ln['axis']:6s} {ln['level_db']:8.1f}")
    print(f"   (thr = throttle scaled to 0–1 over {thr_range[0]:.0f}–{thr_range[1]:.0f})")


def selftest():
    rng = np.random.default_rng(7)
    fs = 2000.0
    thr = np.concatenate([np.linspace(0.1, 1.0, int(20 * fs)), np.full(int(5 * fs), 0.3), np.full(int(5 * fs), 0.6),
                          np.full(int(5 * fs), 0.9)])
    n = len(thr); t = np.arange(n) / fs
    fm = 40 + 160 * thr                                   # motor fundamental, Hz
    ph = 2 * np.pi * np.cumsum(fm) / fs
    motor = (2 + 3 * thr) * np.sin(ph) + 0.6 * (2 + 3 * thr) * np.sin(2 * ph)
    mount = 1.5 * (0.3 + thr) * np.sin(2 * np.pi * 112.0 * t + 0.3)   # fixed mount mode, excited by vibration
    g = np.stack([motor + rng.normal(0, 0.5, n), 0.4 * motor + 2.5 * mount + rng.normal(0, 0.5, n),
                  0.2 * motor + rng.normal(0, 0.5, n)], 1)
    tmp = tempfile.mkdtemp(prefix="mount_survey_")
    p = os.path.join(tmp, "synth.csv")
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time (us)", "gyroRaw[0]", "gyroRaw[1]", "gyroRaw[2]", "motor[0]"])
        for k in range(n):
            w.writerow([int(t[k] * 1e6), f"{g[k, 0]:.3f}", f"{g[k, 1]:.3f}", f"{g[k, 2]:.3f}", int(1000 + 1000 * thr[k])])
    tt, gg, th = load(p, "gyroRaw", "motor[0]")
    res = survey(tt, gg, th, (1000, 2000))
    print_table("selftest (synthetic: motor 40 + 160·thr Hz + 2nd harmonic, fixed mode 112 Hz on pitch)", res, (1000, 2000))
    fixed_ok = any(abs(m["freq"] - 112) <= 3 and m["axis"] == "pitch" for m in res["fixed"])
    fund = res["lines"][0] if res["lines"] else None
    line_ok = fund is not None and abs(fund["slope"] - 160) < 16 and abs(fund["intercept"] - 40) < 10
    harm_ok = any(abs(ln["slope"] - 320) < 32 for ln in res["lines"][1:])
    no_false_fixed = all(abs(m["freq"] - 112) <= 3 for m in res["fixed"])
    ok = fixed_ok and line_ok and harm_ok and no_false_fixed
    print(f"\nmount_survey selftest: {'PASS' if ok else 'FAIL'} (fixed 112 Hz pitch: {fixed_ok}; motor line: {line_ok}; "
          f"2nd harmonic: {harm_ok}; no false fixed modes: {no_false_fixed})")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", action="append", default=[], metavar="LABEL:CSV")
    ap.add_argument("--gyro-col", default="gyroRaw")
    ap.add_argument("--throttle-col", default="motor[0]")
    ap.add_argument("--throttle-range", default="1000:2000")
    ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
    ap.add_argument("--win", type=float, default=0.5, help="window length, s (default 0.5)")
    ap.add_argument("--nbins", type=int, default=8)
    ap.add_argument("--fmin", type=float, default=15.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if not a.input:
        ap.error("give at least one --input LABEL:CSV (or --selftest)")
    rng = tuple(float(x) for x in a.throttle_range.split(":"))
    for spec in a.input:
        label, path = spec.split(":", 1)
        t, g, th = load(path, a.gyro_col, a.throttle_col)
        m = np.ones(len(t), bool)
        if a.t0 is not None:
            m &= t >= a.t0
        if a.t1 is not None:
            m &= t <= a.t1
        print_table(label, survey(t[m], g[m], th[m], rng, a.win, a.nbins, a.fmin), rng)


if __name__ == "__main__":
    main()
