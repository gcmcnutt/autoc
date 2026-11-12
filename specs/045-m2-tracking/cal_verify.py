#!/usr/bin/env python3
"""cal_verify.py — verify INAV's calibrations independently, from logged data (045 research R6b; quickstart §§ 3–4b).

INAV's calibrations are self-certifying (no cross-axis term in the accel fit, a validation that cannot reject, no soft-iron
matrix for the compass). This script re-fits them from a blackbox_decode CSV and prints each R6b criterion with PASS/FAIL.
Times (--t0/--t1, windows) are absolute log seconds, i.e. "time (us)" / 1e6, as in the N1 span lists.

  accel CSV          static poses (auto-segmented): |f| per pose; with >= 9 poses an ellipsoid fit (offset, per-axis scale,
                     cross-axis). R6b: |f| = 1.000 ± 0.005 g in every pose; cross-axis < 1 %.
  gyro-static CSV    residual mean rate per axis over a static window. R6b: < 0.05 °/s. ⚠️ the blackbox logs gyro as
                     integer °/s (blackbox.c lrintf), so a quiet sensor cannot resolve 0.05 °/s: flagged UNRESOLVED.
  gyro-scale CSV     turn-count scale factor: integrate one axis over each moving span and compare with ±N·360°.
                     Do N turns one way, then N back: the CW/CCW pair cancels the (quantized) bias. R6b: record; > 0.5 % noted.
  mag CSV            ellipsoid fit on magADC through a tumble: residual hard iron, soft-iron matrix, |m| variation, dip.
                     R6b: |m| variation < 5 %; dip ≈ the site's (≈ 61° at the field).
  heading CSV        static poses in order vs --bearings: INAV heading (attitude[2]) and, with magADC, a tilt-compensated mag
                     heading. R6b: < 5° at every bearing.
  selftest           synthesize data with known errors (offsets, scales, cross-axis, soft iron, bias, scale error) and check
                     every fit recovers them (the script's own test).

Columns: time (us); gyroADC[0..2] (or --gyro-col gyroRaw); accSmooth[0..2] (or --acc-col accADC) scaled by --acc-1g
(header "acc_1G", 2048 on MPU6000 / ICM42688P); magADC[0..2]; attitude[2] (decidegrees).
"""
import argparse
import csv
import os
import sys
import tempfile

import numpy as np

trapz = getattr(np, "trapezoid", None) or np.trapz  # numpy 2 renamed it

R6B = {"acc_mag_tol": 0.005, "acc_cross": 0.01, "gyro_zero": 0.05, "gyro_scale_note": 0.005, "mag_var": 0.05,
       "heading": 5.0}


# --- loading -----------------------------------------------------------------------------------------------------------

def load(path, gyro_col="gyroADC", acc_col="accSmooth", acc_1g=2048.0, want_mag=False, want_att=False):
    with open(path, newline="") as f:
        r = csv.reader(f)
        h = [c.strip() for c in next(r)]
        def idx(name):
            if name not in h:
                return None
            return h.index(name)
        it = idx("time (us)")
        ig = [idx(f"{gyro_col}[{i}]") for i in range(3)]
        ia = [idx(f"{acc_col}[{i}]") for i in range(3)]
        im = [idx(f"magADC[{i}]") for i in range(3)]
        ih = idx("attitude[2]")
        if it is None or None in ig or None in ia:
            sys.exit(f"cal_verify: {path} lacks time (us) / {gyro_col}[0..2] / {acc_col}[0..2]")
        if want_mag and None in im:
            sys.exit(f"cal_verify: {path} has no magADC[0..2] (blackbox MAG group off?)")
        rows = []
        for row in r:
            try:
                v = [float(row[it])] + [float(row[j]) for j in ig] + [float(row[j]) for j in ia]
                v += [float(row[j]) for j in im] if None not in im else [np.nan] * 3
                v += [float(row[ih])] if ih is not None else [np.nan]
            except (ValueError, IndexError):
                continue
            rows.append(v)
    d = np.array(rows)
    return {"t": d[:, 0] / 1e6, "gyro": d[:, 1:4], "acc": d[:, 4:7] / acc_1g, "mag": d[:, 7:10],
            "heading": d[:, 10] / 10.0}


def window(d, t0, t1):
    m = np.ones(len(d["t"]), bool)
    if t0 is not None:
        m &= d["t"] >= t0
    if t1 is not None:
        m &= d["t"] <= t1
    return {k: v[m] for k, v in d.items()}


# --- static segmentation ------------------------------------------------------------------------------------------------

def static_segments(t, gyro, acc, gyro_thr=3.0, acc_thr=0.01, min_dur=3.0, trim=0.5):
    """Runs where the rolling mean |ω| < gyro_thr °/s and every accel axis' rolling std < acc_thr g, lasting >= min_dur s
    after trimming `trim` s at each end. Returns [(i0, i1)] index ranges (i1 exclusive)."""
    if len(t) < 10:
        return []
    fs = 1.0 / np.median(np.diff(t))
    w = max(3, int(round(0.5 * fs)))
    k = np.ones(w) / w
    gm = np.convolve(np.linalg.norm(gyro, axis=1), k, "same")
    am = np.stack([np.convolve(acc[:, i], k, "same") for i in range(3)], 1)
    a2 = np.stack([np.convolve(acc[:, i] ** 2, k, "same") for i in range(3)], 1)
    astd = np.sqrt(np.maximum(a2 - am ** 2, 0)).max(1)
    ok = (gm < gyro_thr) & (astd < acc_thr)
    segs, i, n, tr = [], 0, len(ok), int(round(trim * fs))
    while i < n:
        if not ok[i]:
            i += 1; continue
        j = i
        while j < n and ok[j]:
            j += 1
        a, b = i + tr, j - tr
        if b > a and t[b - 1] - t[a] >= min_dur:
            segs.append((a, b))
        i = j
    return segs


def moving_segments(t, n, statics, min_dur=2.0):
    """Gaps between static segments (and before / after them) lasting >= min_dur s."""
    edges, prev = [], 0
    for a, b in statics:
        if a > prev:
            edges.append((prev, a))
        prev = b
    if prev < n:
        edges.append((prev, n))
    return [(a, b) for a, b in edges if b - a > 1 and t[b - 1] - t[a] >= min_dur]


# --- ellipsoid fit ------------------------------------------------------------------------------------------------------

def ellipsoid_fit(x):
    """Fit (x - c)^T A (x - c) = 1 (algebraic least squares). Returns center c and the symmetric S = sqrt(A), so that
    |S (x - c)| = 1 on the fitted surface. Raises if the points do not determine an ellipsoid."""
    x = np.asarray(x, float)
    if len(x) < 9:
        raise ValueError(f"ellipsoid fit needs >= 9 points, got {len(x)}")
    X, Y, Z = x[:, 0], x[:, 1], x[:, 2]
    D = np.stack([X * X, Y * Y, Z * Z, 2 * X * Y, 2 * X * Z, 2 * Y * Z, 2 * X, 2 * Y, 2 * Z], 1)
    v, *_ = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)
    A = np.array([[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]])
    bv = v[6:9]
    c = -np.linalg.solve(A, bv)
    kk = 1.0 + c @ A @ c
    An = A / kk
    w, V = np.linalg.eigh(An)
    if np.any(w <= 0):
        raise ValueError("points do not determine an ellipsoid (poses or tumble do not span all directions)")
    S = V @ np.diag(np.sqrt(w)) @ V.T
    return c, S


def scale_cross(S):
    d = np.diag(S)
    cross = S / np.sqrt(np.outer(d, d))
    np.fill_diagonal(cross, 0.0)
    return d - 1.0, cross


# --- subcommands --------------------------------------------------------------------------------------------------------

def pf(ok):
    return "PASS" if ok else "FAIL"


def run_accel(d, a):
    segs = static_segments(d["t"], d["gyro"], d["acc"], a.static_gyro, a.static_acc, a.min_pose)
    poses = np.array([d["acc"][i0:i1].mean(0) for i0, i1 in segs]) if segs else np.zeros((0, 3))
    out = {"n_poses": len(poses), "poses": poses, "segs": segs}
    print(f"accel: {len(poses)} static poses (|ω| < {a.static_gyro} °/s, acc std < {a.static_acc} g, >= {a.min_pose} s)")
    for k, ((i0, i1), p) in enumerate(zip(segs, poses)):
        m = np.linalg.norm(p)
        print(f"  pose {k + 1:2d}  t {d['t'][i0]:8.2f}–{d['t'][i1 - 1]:8.2f}  f = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}) g  "
              f"|f| {m:.4f}  {pf(abs(m - 1) <= R6B['acc_mag_tol'])}")
    if len(poses):
        worst = np.max(np.abs(np.linalg.norm(poses, axis=1) - 1))
        print(f"  R6b |f| = 1.000 ± {R6B['acc_mag_tol']} g in every pose: {pf(worst <= R6B['acc_mag_tol'])} "
              f"(worst |1 − |f|| = {worst:.4f} g)")
    if len(poses) < 9:
        print(f"  ellipsoid fit: needs >= 9 poses (R6b asks for >= 12: the 6 faces + 6 diagonals); skipped")
        return out
    c, S = ellipsoid_fit(poses)
    se, cross = scale_cross(S)
    resid = np.linalg.norm((poses - c) @ S.T, axis=1) - 1
    out.update(offset=c, S=S, scale_err=se, cross=cross)
    print(f"  ellipsoid fit (true = S·(raw − c)): offset c = ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f}) g")
    print(f"    per-axis scale error S_ii − 1 = ({se[0]:+.4f}, {se[1]:+.4f}, {se[2]:+.4f})  (+ ⇒ the axis reads low)")
    mc = np.max(np.abs(cross))
    print(f"    cross-axis xy {cross[0, 1]:+.4f}  xz {cross[0, 2]:+.4f}  yz {cross[1, 2]:+.4f}   "
          f"R6b cross-axis < {R6B['acc_cross']:.0%}: {pf(mc < R6B['acc_cross'])}")
    print(f"    fit residual rms {np.sqrt(np.mean(resid ** 2)):.4f} g")
    return out


def gyro_resolution(x):
    """Smallest non-zero step between distinct values (the logged LSB)."""
    u = np.unique(np.round(x, 6))
    dd = np.diff(u)
    return float(dd[dd > 0].min()) if len(dd) and np.any(dd > 0) else float("nan")


def run_gyro_static(d, a):
    if a.t0 is not None or a.t1 is not None:
        w = window(d, a.t0, a.t1); src = "the given window"
    else:
        segs = static_segments(d["t"], d["gyro"], d["acc"], a.static_gyro, a.static_acc, a.min_pose)
        if not segs:
            print("gyro-static: no static window found; pass --t0/--t1"); return {}
        i0, i1 = max(segs, key=lambda s: s[1] - s[0])
        w = {k: v[i0:i1] for k, v in d.items()}; src = "the longest static segment"
    g = w["gyro"]
    n = len(g)
    mean, std = g.mean(0), g.std(0)
    lsb = gyro_resolution(g.ravel())
    print(f"gyro-static over {src}: t {w['t'][0]:.2f}–{w['t'][-1]:.2f} s ({n} samples), logged LSB {lsb:g} °/s")
    out = {"mean": mean, "std": std, "lsb": lsb, "resolved": []}
    for i, ax in enumerate("xyz"):
        resolved = not (std[i] < 0.5 * lsb)  # without ~0.5 LSB of noise the quantized mean is not the true mean
        out["resolved"].append(resolved)
        verdict = pf(abs(mean[i]) < R6B["gyro_zero"]) if resolved else "UNRESOLVED (noise < ½ LSB: no dither)"
        print(f"  {ax}: mean {mean[i]:+.3f} °/s  std {std[i]:.3f}  R6b < {R6B['gyro_zero']} °/s: {verdict}")
    if not all(out["resolved"]):
        print("  ⚠️ the blackbox logs integer °/s; a quiet static sensor cannot show a 0.05 °/s residual. Read the zero from a\n"
              "     finer source (INAV gyro_zero_* in LSB after a still calibration), or log with the motor running for dither.")
    return out


def run_gyro_scale(d, a):
    ax = "xyz".index(a.axis)
    t, g = d["t"], d["gyro"][:, ax]
    statics = static_segments(t, d["gyro"], d["acc"], a.static_gyro, a.static_acc, a.min_pose)
    if a.windows:
        spans = []
        for w in a.windows.split(","):
            t0, t1 = (float(x) for x in w.split(":"))
            m = np.where((t >= t0) & (t <= t1))[0]
            spans.append((m[0], m[-1] + 1))
    else:
        spans = moving_segments(t, len(t), statics)
    bias = np.mean(np.concatenate([g[i0:i1] for i0, i1 in statics])) if statics else 0.0
    target = a.turns * 360.0
    print(f"gyro-scale axis {a.axis}: {len(spans)} moving spans, {len(statics)} static segments "
          f"(static bias estimate {bias:+.3f} °/s, quantized), N = {a.turns} turns = {target:.0f}°")
    ints = []
    for i0, i1 in spans:
        I = trapz(g[i0:i1], t[i0:i1])
        Ib = trapz(g[i0:i1] - bias, t[i0:i1])
        dur = t[i1 - 1] - t[i0]
        ints.append((I, dur))
        sgn = 1 if I >= 0 else -1
        print(f"  span t {t[i0]:8.2f}–{t[i1 - 1]:8.2f} ({dur:5.1f} s): ∫ω = {I:+9.1f}°  "
              f"one-way scale error (bias-corrected) {(sgn * Ib - target) / target:+.4%}")
    out = {"spans": ints, "bias": bias}
    pos = [x for x in ints if x[0] > 0]; neg = [x for x in ints if x[0] < 0]
    if pos and neg:
        Ip = max(pos, key=lambda x: abs(x[0]))[0]; In = min(neg, key=lambda x: x[0])[0]
        se = (Ip - In - 2 * target) / (2 * target)  # constant bias cancels when the two spans last about as long
        out["scale_err"] = se
        print(f"  CW/CCW pair (bias-free): scale error {se:+.4%}   "
              f"R6b: record; > {R6B['gyro_scale_note']:.1%} ⇒ note in the R4 budget: "
              f"{'NOTE' if abs(se) > R6B['gyro_scale_note'] else 'within'}")
    else:
        print("  no CW/CCW pair: the one-way numbers carry the bias error (≈ bias × span duration); do N turns back too")
    return out


def dip_deg(acc, mag):
    """Dip from paired static accel (specific force, points up at rest) and mag samples: asin(m·down / |m|)."""
    u = acc / np.linalg.norm(acc, axis=1, keepdims=True)
    return np.degrees(np.arcsin(np.sum(mag * -u, 1) / np.linalg.norm(mag, axis=1)))


def mag_heading_deg(acc, mag, declination=0.0):
    """Tilt-compensated heading of the body x axis (FLU body frame), clockwise from true north."""
    u = acc / np.linalg.norm(acc, axis=1, keepdims=True)
    mh = mag - np.sum(mag * u, 1, keepdims=True) * u
    n = mh / np.linalg.norm(mh, axis=1, keepdims=True)
    w = np.cross(u, n)  # north × west = up in a right-handed NWU frame
    fx = np.array([1.0, 0.0, 0.0])
    return (np.degrees(np.arctan2(-(w @ fx), n @ fx)) + declination) % 360.0


def run_mag(d, a):
    m = d["mag"]
    good = np.all(np.isfinite(m), 1)
    m = m[good]
    # keep distinct readings only (the compass updates slower than the blackbox)
    keep = np.r_[True, np.any(np.diff(m, axis=0) != 0, 1)]
    m = m[keep]
    print(f"mag: {len(m)} distinct samples")
    norms = np.linalg.norm(m, axis=1)
    med = np.median(norms)
    var = (np.percentile(norms, 98) - np.percentile(norms, 2)) / med
    out = {"var": var}
    print(f"  |m| (as logged) median {med:.1f}, 2–98 % spread {var:.2%}   R6b < {R6B['mag_var']:.0%}: "
          f"{pf(var < R6B['mag_var'])}")
    try:
        c, S = ellipsoid_fit(m)
    except ValueError as e:
        print(f"  ellipsoid fit: {e}"); return out
    radius = 1.0 / np.cbrt(np.linalg.det(S))
    Srel = S * radius
    out.update(hard=c, soft=Srel, radius=radius)
    print(f"  residual hard iron c = ({c[0]:+.1f}, {c[1]:+.1f}, {c[2]:+.1f})  ({np.linalg.norm(c) / radius:.1%} of the field)")
    print("  residual soft iron (unit determinant):")
    for row in Srel:
        print(f"    [{row[0]:+.4f} {row[1]:+.4f} {row[2]:+.4f}]")
    # dip from static samples
    segs = static_segments(d["t"], d["gyro"], d["acc"], a.static_gyro, a.static_acc, 1.0)
    if segs:
        idx = np.concatenate([np.arange(i0, i1) for i0, i1 in segs])
        mm, aa = d["mag"][idx], d["acc"][idx]
        ok = np.all(np.isfinite(mm), 1)
        if ok.any():
            dips = dip_deg(aa[ok], mm[ok])
            out["dip"] = float(np.median(dips))
            print(f"  dip over {len(segs)} static segments: median {out['dip']:.1f}° (spread {np.ptp(dips):.1f}°); "
                  f"expect ≈ {a.expect_dip:.0f}° at the site")
    else:
        print("  dip: no static segments (hold still for >= 1 s at a few attitudes)")
    return out


def run_heading(d, a):
    bearings = [float(x) for x in a.bearings.split(",")]
    segs = static_segments(d["t"], d["gyro"], d["acc"], a.static_gyro, a.static_acc, a.min_pose)
    print(f"heading: {len(segs)} static poses for {len(bearings)} bearings (in order)")
    if len(segs) < len(bearings):
        print("  fewer static poses than bearings: hold each bearing still for >= --min-pose s"); return {}
    errs_inav, errs_mag = [], []
    for (i0, i1), b in zip(segs, bearings):
        hi = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(d["heading"][i0:i1])))))
        ei = (hi - b + 180) % 360 - 180
        errs_inav.append(ei)
        line = f"  bearing {b:5.1f}°: INAV heading {hi % 360:6.1f}° (err {ei:+5.1f}°)"
        mm = d["mag"][i0:i1]
        if np.all(np.isfinite(mm)):
            hm = mag_heading_deg(d["acc"][i0:i1].mean(0, keepdims=True), mm.mean(0, keepdims=True), a.declination)[0]
            em = (hm - b + 180) % 360 - 180
            errs_mag.append(em)
            line += f"   mag-derived {hm:6.1f}° (err {em:+5.1f}°)"
        print(line)
    wi = np.max(np.abs(errs_inav))
    print(f"  R6b < {R6B['heading']}° everywhere (INAV heading): {pf(wi < R6B['heading'])} (worst {wi:.1f}°)")
    if errs_mag:
        wm = np.max(np.abs(errs_mag))
        print(f"  mag-derived (declination {a.declination}°): worst {wm:.1f}° — {pf(wm < R6B['heading'])}")
    return {"inav": errs_inav, "mag": errs_mag}


# --- selftest -----------------------------------------------------------------------------------------------------------

def rot(axis, ang):
    c, s = np.cos(ang), np.sin(ang)
    x, y, z = axis / np.linalg.norm(axis)
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def write_csv(path, t, gyro, acc_counts, mag=None, heading=None):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        h = ["time (us)"] + [f"gyroADC[{i}]" for i in range(3)] + [f"accSmooth[{i}]" for i in range(3)]
        if mag is not None:
            h += [f"magADC[{i}]" for i in range(3)]
        if heading is not None:
            h += ["attitude[2]"]
        w.writerow(h)
        for k in range(len(t)):
            row = [int(t[k] * 1e6)] + list(gyro[k]) + list(acc_counts[k])
            if mag is not None:
                row += list(mag[k])
            if heading is not None:
                row += [heading[k]]
            w.writerow(row)


def selftest():
    rng = np.random.default_rng(45)
    fs, acc1g = 250.0, 2048.0
    ns = argparse.Namespace(static_gyro=3.0, static_acc=0.01, min_pose=3.0, t0=None, t1=None, axis="z", turns=10,
                            windows=None, expect_dip=61.0, bearings="0,90,180,270", declination=12.9)
    tmp = tempfile.mkdtemp(prefix="cal_verify_")
    fails = []

    def check(name, ok, detail):
        print(f"[selftest] {name}: {'ok' if ok else 'FAILED'}  {detail}")
        if not ok:
            fails.append(name)

    # 1. accel: 12 poses (6 faces + 6 diagonals), symmetric distortion S_true, offset b_true
    dirs = [np.array(v, float) for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
                                          (1, 1, 0), (1, 0, 1), (0, 1, 1), (-1, 1, 0), (1, 0, -1), (0, -1, 1)]]
    S_true = np.array([[1.010, 0.004, -0.003], [0.004, 0.990, 0.002], [-0.003, 0.002, 1.005]])
    b_true = np.array([0.020, -0.015, 0.030])
    Si = np.linalg.inv(S_true)
    T, G, A = [], [], []
    tt = 0.0
    prev = Si @ (dirs[0] / np.linalg.norm(dirs[0])) + b_true
    for u in dirs:
        meas = Si @ (u / np.linalg.norm(u)) + b_true
        for k in range(int(2 * fs)):  # 2 s transition: rotating, accel blending
            s = k / (2 * fs)
            T.append(tt); tt += 1 / fs
            G.append(rng.normal(0, 1, 3) + np.array([60.0, -40.0, 30.0]))
            A.append((1 - s) * prev + s * meas + rng.normal(0, 0.05, 3))
        for k in range(int(10 * fs)):  # 10 s static
            T.append(tt); tt += 1 / fs
            G.append(np.round(rng.normal(0, 0.8, 3)))
            A.append(meas + rng.normal(0, 0.002, 3))
        prev = meas
    p = os.path.join(tmp, "accel.csv")
    write_csv(p, np.array(T), np.array(G), np.round(np.array(A) * acc1g))
    r = run_accel(load(p, acc_1g=acc1g), ns)
    ok = r["n_poses"] == 12 and "S" in r and np.max(np.abs(r["S"] - S_true)) < 2e-3 and np.max(np.abs(r["offset"] - b_true)) < 2e-3
    check("accel ellipsoid", ok, f"poses {r['n_poses']}, max|S−S_true| "
          f"{np.max(np.abs(r.get('S', S_true * 0) - S_true)):.2e}, max|c−b| {np.max(np.abs(r.get('offset', b_true * 0) - b_true)):.2e}")

    # 2. gyro static: dithered (resolvable) and quiet (unresolvable) cases, 1 °/s quantization
    bias = np.array([0.30, -0.20, 0.70])
    n = int(60 * fs)
    t = np.arange(n) / fs
    acc = np.tile([0, 0, acc1g], (n, 1))
    p = os.path.join(tmp, "gstat_dither.csv")
    write_csv(p, t, np.round(bias + rng.normal(0, 1.0, (n, 3))), acc)
    r = run_gyro_static(load(p, acc_1g=acc1g), ns)
    check("gyro-static dithered", all(r["resolved"]) and np.max(np.abs(r["mean"] - bias)) < 0.05,
          f"mean {np.round(r['mean'], 3)} vs bias {bias}")
    p = os.path.join(tmp, "gstat_quiet.csv")
    write_csv(p, t, np.round(bias + rng.normal(0, 0.1, (n, 3))), acc)
    r = run_gyro_static(load(p, acc_1g=acc1g), ns)
    check("gyro-static quiet flagged", not any(r["resolved"]), f"resolved flags {r['resolved']}")

    # 3. gyro scale: 10 turns CW then CCW about z at 120 °/s, scale error +0.7 %, bias 0.4 °/s, quantized
    se_true, rate, b = 0.007, 120.0, 0.4
    seg = lambda dur: int(dur * fs)
    rates = np.concatenate([np.zeros(seg(5)), np.full(seg(30), rate), np.zeros(seg(5)), np.full(seg(30), -rate),
                            np.zeros(seg(5))])
    n = len(rates); t = np.arange(n) / fs
    gz = np.round(rates * (1 + se_true) + b + rng.normal(0, 1.0, n))
    gyro = np.stack([np.round(rng.normal(0, 1.0, n)), np.round(rng.normal(0, 1.0, n)), gz], 1)
    acc = np.tile([0, 0, acc1g], (n, 1)).astype(float) + rng.normal(0, 2, (n, 3)) * (rates[:, None] != 0) * 30
    p = os.path.join(tmp, "gscale.csv")
    write_csv(p, t, gyro, np.round(acc))
    r = run_gyro_scale(load(p, acc_1g=acc1g), ns)
    check("gyro-scale CW/CCW", "scale_err" in r and abs(r["scale_err"] - se_true) < 5e-4,
          f"scale error {r.get('scale_err', float('nan')):+.4%} vs {se_true:+.2%}")

    # 4. mag: tumble with hard + soft iron; dip 61°; plus 4 level static poses for dip / heading
    B = 470.0 * np.array([np.cos(np.radians(61)), 0.0, -np.sin(np.radians(61))])  # NWU world: north, west, up
    # a residual calibration error of the size INAV leaves (~2 % soft iron, ~3 % hard iron)
    soft = np.array([[1.02, 0.01, -0.005], [0.01, 0.985, 0.008], [-0.005, 0.008, 1.0]])
    hard = np.array([12.0, -8.0, 5.0])

    # exact math first: an undistorted field at dip 61°, level, nose at magnetic heading 30°
    R = rot(np.array([0, 0, 1.0]), -np.radians(30.0))
    a0, m0 = (R.T @ np.array([0, 0, 1.0]))[None, :], (R.T @ B)[None, :]
    check("dip / heading math", abs(dip_deg(a0, m0)[0] - 61.0) < 1e-6 and abs(mag_heading_deg(a0, m0, 0.0)[0] - 30.0) < 1e-6,
          f"dip {dip_deg(a0, m0)[0]:.4f}°, heading {mag_heading_deg(a0, m0, 0.0)[0]:.4f}°")
    T, G, A, M = [], [], [], []
    tt = 0.0
    for k in range(4000):  # tumble: random attitudes
        R = rot(rng.normal(size=3), rng.uniform(0, 2 * np.pi))  # body -> world
        T.append(tt); tt += 1 / fs
        G.append(np.full(3, 50.0))
        A.append(R.T @ np.array([0, 0, 1.0]))
        M.append(soft @ (R.T @ B) + hard)
    headings, true_err = [0.0, 90.0, 180.0, 270.0], [2.0, -3.0, 1.0, 4.0]
    H = []
    decl = ns.declination
    for hdg, err in zip(headings, true_err):  # level, nose at true bearing hdg ⇒ magnetic heading hdg − decl
        mag_hdg = np.radians(hdg - decl)
        R = rot(np.array([0, 0, 1.0]), -mag_hdg)  # body -> world (NWU: clockwise heading = negative yaw about up)
        for k in range(int(6 * fs)):
            T.append(tt); tt += 1 / fs
            G.append(np.round(rng.normal(0, 0.8, 3)))
            A.append(R.T @ np.array([0, 0, 1.0]) + rng.normal(0, 0.001, 3))
            M.append(soft @ (R.T @ B) + hard + rng.normal(0, 0.5, 3))
            H.append(((hdg + err) % 360) * 10)
        for k in range(int(2 * fs)):
            T.append(tt); tt += 1 / fs
            G.append(np.full(3, 40.0)); A.append(np.array([0.3, 0.3, 1.2])); M.append(M[-1]); H.append(H[-1])
    H = [0.0] * 4000 + H
    p = os.path.join(tmp, "mag.csv")
    write_csv(p, np.array(T), np.array(G), np.round(np.array(A) * acc1g), np.array(M), np.array(H))
    d = load(p, acc_1g=acc1g, want_mag=True)
    r = run_mag(d, ns)
    soft_rel = soft / np.cbrt(np.linalg.det(soft))
    Sinv_rel = np.linalg.inv(r["soft"]) if "soft" in r else soft_rel * 0
    ok = "hard" in r and np.max(np.abs(r["hard"] - hard)) < 1.5 and np.max(np.abs(Sinv_rel - soft_rel)) < 5e-3 \
        and abs(r.get("dip", 0) - 61.0) < 2.5
    check("mag hard/soft/dip", ok, f"hard {np.round(r.get('hard', hard * 0), 1)} vs {hard}, "
          f"max|S⁻¹−soft| {np.max(np.abs(Sinv_rel - soft_rel)):.1e}, dip {r.get('dip', float('nan')):.1f}°")
    # heading pipeline on an undistorted field: the INAV heading carries the injected errors, the mag-derived one ~0
    T, G, A, M, H = [], [], [], [], []
    tt = 0.0
    for hdg, err in zip(headings, true_err):
        R = rot(np.array([0, 0, 1.0]), -np.radians(hdg - decl))
        for k in range(int(6 * fs)):
            T.append(tt); tt += 1 / fs
            G.append(np.round(rng.normal(0, 0.8, 3))); A.append(R.T @ np.array([0, 0, 1.0]) + rng.normal(0, 0.001, 3))
            M.append(R.T @ B + rng.normal(0, 0.5, 3)); H.append(((hdg + err) % 360) * 10)
        for k in range(int(2 * fs)):
            T.append(tt); tt += 1 / fs
            G.append(np.full(3, 40.0)); A.append(np.array([0.3, 0.3, 1.2])); M.append(M[-1]); H.append(H[-1])
    p = os.path.join(tmp, "heading.csv")
    write_csv(p, np.array(T), np.array(G), np.round(np.array(A) * acc1g), np.array(M), np.array(H))
    r = run_heading(load(p, acc_1g=acc1g, want_mag=True), ns)
    ok = len(r.get("inav", [])) == 4 and np.max(np.abs(np.array(r["inav"]) - true_err)) < 0.5
    check("heading (INAV attitude[2])", ok, f"errors {np.round(r.get('inav', []), 1)} vs {true_err}")
    ok = len(r.get("mag", [])) == 4 and np.max(np.abs(r["mag"])) < 0.5
    check("heading (mag-derived, with declination)", ok, f"errors {np.round(r.get('mag', []), 2)}")

    print("cal_verify selftest:", "PASS" if not fails else f"FAIL {fails}")
    return 0 if not fails else 1


# --- CLI ----------------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("csv")
        p.add_argument("--acc-1g", type=float, default=2048.0)
        p.add_argument("--gyro-col", default="gyroADC")
        p.add_argument("--acc-col", default="accSmooth")
        p.add_argument("--t0", type=float); p.add_argument("--t1", type=float)
        p.add_argument("--static-gyro", type=float, default=3.0, help="°/s, rolling mean |ω| (default 3)")
        p.add_argument("--static-acc", type=float, default=0.01, help="g, rolling std per axis (default 0.01)")
        p.add_argument("--min-pose", type=float, default=3.0, help="s (default 3)")
        return p

    common(sub.add_parser("accel"))
    common(sub.add_parser("gyro-static"))
    p = common(sub.add_parser("gyro-scale"))
    p.add_argument("--axis", choices="xyz", required=True)
    p.add_argument("--turns", type=float, required=True)
    p.add_argument("--windows", help="explicit moving spans t0:t1[,t0:t1] (default: auto between static segments)")
    p = common(sub.add_parser("mag"))
    p.add_argument("--expect-dip", type=float, default=61.0)
    p = common(sub.add_parser("heading"))
    p.add_argument("--bearings", required=True, help="true bearings in pose order, e.g. 0,45,90,...")
    p.add_argument("--declination", type=float, default=12.9, help="for the mag-derived heading (°E, default 12.9)")
    sub.add_parser("selftest")
    a = ap.parse_args()

    if a.cmd == "selftest":
        sys.exit(selftest())
    d = load(a.csv, a.gyro_col, a.acc_col, a.acc_1g, want_mag=a.cmd == "mag")
    if a.cmd != "gyro-static":
        d = window(d, a.t0, a.t1)
    {"accel": run_accel, "gyro-static": run_gyro_static, "gyro-scale": run_gyro_scale, "mag": run_mag,
     "heading": run_heading}[a.cmd](d, a)


if __name__ == "__main__":
    main()
