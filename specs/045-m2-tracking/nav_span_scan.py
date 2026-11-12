#!/usr/bin/env python3
"""
nav_span_scan.py — find nav-estimate faults in a xiao tick CSV (045 PRE-TASK N1 / N3).

Per engaged span: ticks where horizontal velocity is exactly zero (INAV's XY estimate invalid ⇒ frozen position, zero velocity),
position steps that velocity does not explain (residual of Δpos − mean(vel)·Δt), and the largest tick-to-tick steps in velocity,
rabbit, attitude and distance. Pure stdlib, so it runs anywhere the decoder does. First found the 11-08 span 1 faults
(044 flight-analysis-20261108.md § 6): 46 frozen ticks, a 106 m residual at tick 215 → 216, 25 m at 332 → 333.

Blackbox mode (045 T026): the same faults from the INAV blackbox, per engaged span (mspOverrideFlags == 2), resampled to the
xiao's 20 Hz tick so the definitions match data-model.md § Estimator fault flags:
  FROZEN     navVel x = y = 0 exactly and navPos xy unchanged vs the previous tick, >= --frozen-min ticks (default 2)
  STEP_XY/Z  |Δpos − v̄·Δt| above --step-xy / --step-z in one tick (the INAV estimate jumped)
  INVALID_XY navFlags bit 2 clear (estPosStatus not TRUSTED);  INVALID_Z  bit 0 clear (estAltStatus not TRUSTED)
  EPH_CAP    navEPH >= --max-eph (the config's inav_max_eph_epv)
  LOW_SATS   GPS numSat < 6;  NO_FIX  GPS fixType < 2 (3D);  GPS_GAP  GPS update interval > 0.2 s (GPS_STALE)
Writes the events to --out (faults.csv) and the clean-tick residual distribution (the threshold evidence) to --resid-out.
numpy + span_attitude.py (same directory) are needed in this mode only.

Usage: nav_span_scan.py xiao.csv [--step 5] [--top 5]     (xiao.csv from src/analytics/flightlog_decode.py -o)
       nav_span_scan.py --blackbox [--flight 1108 ...] [--max-eph 10] [--step-xy 5] [--step-z 5]
                        [--out specs/045-m2-tracking/n1/faults.csv] [--resid-out specs/045-m2-tracking/n1/nav_residuals.csv]
"""
import argparse
import collections
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def scan(sid, rows, step, top):
    g = lambda r, k: float(r[k])
    t = [g(r, "timestamp_ms") for r in rows]
    P = [(g(r, "pos_n"), g(r, "pos_e"), g(r, "pos_d")) for r in rows]
    V = [(g(r, "vel_n"), g(r, "vel_e"), g(r, "vel_d")) for r in rows]
    R = [(g(r, "rabbit_n"), g(r, "rabbit_e"), g(r, "rabbit_d")) for r in rows]
    Q = [(g(r, "quat_w"), g(r, "quat_x"), g(r, "quat_y"), g(r, "quat_z")) for r in rows]
    n = len(rows) - 1
    series = {
        "pos resid m": [norm([P[i + 1][k] - P[i][k] - 0.5 * (V[i][k] + V[i + 1][k]) * (t[i + 1] - t[i]) / 1000 for k in range(3)])
                        for i in range(n)],
        "dvel m/s": [norm([b - a for a, b in zip(V[i], V[i + 1])]) for i in range(n)],
        "rabbit step m": [norm([b - a for a, b in zip(R[i], R[i + 1])]) for i in range(n)],
        "att step deg": [math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(Q[i], Q[i + 1])))))) for i in range(n)],
    }
    D = [norm([a - b for a, b in zip(P[i], R[i])]) for i in range(len(rows))]
    series["dist step m"] = [abs(D[i + 1] - D[i]) for i in range(n)]
    frozen = [i for i, v in enumerate(V) if v[0] == 0 and v[1] == 0]
    steps = [i + 1 for i, x in enumerate(series["pos resid m"]) if x > step]
    flag = "  ⚠️ FAULT" if frozen or steps else ""
    print(f"\n== span {sid}: {len(rows)} ticks, {(t[-1] - t[0]) / 1000:.1f} s, path {rows[0]['path_index']}{flag}")
    if frozen:
        print(f"  frozen XY (vel_n = vel_e = 0): {len(frozen)} ticks, first {frozen[0]} last {frozen[-1]}"
              f" ({(t[frozen[0]] - t[0]) / 1000:.2f}–{(t[frozen[-1]] - t[0]) / 1000:.2f} s)")
    for i in steps:
        d = [b - a for a, b in zip(P[i - 1], P[i])]
        print(f"  pos step at tick {i - 1} → {i} ({(t[i] - t[0]) / 1000:.2f} s): Δn {d[0]:+.1f} Δe {d[1]:+.1f} Δd {d[2]:+.1f} m")
    for name, a in series.items():
        s = sorted(a)
        worst = sorted(range(len(a)), key=lambda j: -a[j])[:top]
        print(f"  {name:14s} p50 {s[len(s) // 2]:7.3f} p99 {s[int(0.99 * len(s))]:7.3f}  top (tick, value, s): "
              + ", ".join(f"({j + 1}, {a[j]:.2f}, {(t[j + 1] - t[0]) / 1000:.2f})" for j in worst))


def runs(mask):
    """[(i_start, i_end_inclusive)] of True runs."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((start, i - 1)); start = None
    if start is not None:
        out.append((start, len(mask) - 1))
    return out


def blackbox_mode(args):
    import numpy as np
    sys.path.insert(0, HERE)
    import span_attitude as sa
    events, resid = [], []
    for key in args.flight or sorted(sa.FLIGHTS):
        bb, gps = sa.load_bb(sa.FLIGHTS[key]), sa.load_gps(sa.FLIGHTS[key])
        t = bb["t"]
        for n, (a, b) in enumerate(sa.engaged_spans(t, bb["msp"]), 1):
            tk = np.arange(a, b, args.tick)
            ix = np.clip(np.searchsorted(t, tk, side="right") - 1, 0, len(t) - 1)
            P, V = bb["navPos"][ix], bb["navVel"][ix]
            eph, fl = bb["eph"][ix], bb["navFlags"][ix]
            dt = np.diff(tk)[:, None]
            r = P[1:] - P[:-1] - 0.5 * (V[1:] + V[:-1]) * dt
            rh, rv = np.hypot(r[:, 0], r[:, 1]), np.abs(r[:, 2])
            frozen = np.r_[False, (V[1:, 0] == 0) & (V[1:, 1] == 0) & np.all(P[1:, :2] == P[:-1, :2], axis=1)]
            masks = {
                "FROZEN": frozen,
                "STEP_XY": np.r_[False, rh > args.step_xy], "STEP_Z": np.r_[False, rv > args.step_z],
                "INVALID_XY": (fl & 4) == 0, "INVALID_Z": (fl & 1) == 0,
                "EPH_CAP": eph >= args.max_eph,
            }
            gix = np.clip(np.searchsorted(gps["t"], tk, side="right") - 1, 0, len(gps["t"]) - 1)
            age = tk - gps["t"][gix]
            masks["LOW_SATS"] = gps["nsat"][gix] < 6
            masks["NO_FIX"] = gps["fix"][gix] < 2
            masks["GPS_GAP"] = age > 0.2
            mag = {"FROZEN": None, "STEP_XY": np.r_[0, rh], "STEP_Z": np.r_[0, rv], "INVALID_XY": None, "INVALID_Z": None,
                   "EPH_CAP": eph, "LOW_SATS": gps["nsat"][gix], "NO_FIX": None, "GPS_GAP": age}
            near = np.zeros(len(tk), bool)  # ticks near any validity / freeze event: excluded from the clean distribution
            for kind, m in masks.items():
                for i0, i1 in runs(m):
                    if kind == "FROZEN" and i1 - i0 + 1 < args.frozen_min:
                        continue
                    x = mag[kind]
                    val = "" if x is None else (f"{x[i0:i1 + 1].min():.0f}" if kind == "LOW_SATS" else f"{x[i0:i1 + 1].max():.2f}")
                    events.append([key, n, kind, f"{tk[i0] - a:.2f}", f"{tk[i1] - a + args.tick:.2f}",
                                   f"{(i1 - i0 + 1) * args.tick:.2f}", val])
                    if kind in ("FROZEN", "INVALID_XY", "INVALID_Z", "EPH_CAP", "NO_FIX"):
                        lo, hi = max(i0 - int(1 / args.tick), 0), min(i1 + int(1 / args.tick), len(tk) - 1)
                        near[lo:hi + 1] = True
                    if kind in ("STEP_XY", "STEP_Z"):  # the step ticks themselves are faults, not clean residuals
                        near[i0:i1 + 1] = True
            bad = [e for e in events if e[0] == key and e[1] == n]
            print(f"== {key} span {n}: {b - a:.1f} s, {len(tk)} ticks" + ("  ⚠️ " + ", ".join(sorted({e[2] for e in bad})) if bad else ""))
            for e in bad:
                print(f"  {e[2]:10s} {e[3]:>6s}–{e[4]:>6s} s ({e[5]} s)" + (f"  max/min {e[6]}" if e[6] else ""))
            clean = ~near[1:]
            for i in range(len(rh)):
                resid.append((key, n, rh[i], rv[i], bool(clean[i])))
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "span", "kind", "t_start_s", "t_end_s", "duration_s", "magnitude"])
        w.writerows(events)
    import numpy as np
    H = np.array([x[2] for x in resid]); Z = np.array([x[3] for x in resid]); C = np.array([x[4] for x in resid])
    with open(args.resid_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["set", "axis", "n_ticks", "p50_m", "p99_m", "p99_9_m", "max_m"])
        for name, m in (("clean", C), ("near_faults", ~C), ("all", np.ones(len(C), bool))):
            for axis, X in (("horizontal", H), ("vertical", Z)):
                x = X[m]
                if len(x):
                    row = [name, axis, len(x), *(f"{np.percentile(x, q):.2f}" for q in (50, 99, 99.9)), f"{x.max():.2f}"]
                    w.writerow(row)
                    print(f"  residual {name:11s} {axis:10s} n {len(x):5d}  p50 {row[3]}  p99 {row[4]}  p99.9 {row[5]}  max {row[6]} m")
    print(f"wrote {args.out}, {args.resid_out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("csv", nargs="?", help="xiao tick CSV (xiao mode)")
    ap.add_argument("--blackbox", action="store_true", help="045 T026: scan the INAV blackbox of the known flights")
    ap.add_argument("--flight", action="append", help="blackbox mode: flight key from span_attitude.FLIGHTS (default all)")
    ap.add_argument("--tick", type=float, default=0.05, help="blackbox mode: resample tick, s (default 0.05 = the xiao's 20 Hz)")
    ap.add_argument("--max-eph", type=float, default=10.0, help="blackbox mode: inav_max_eph_epv of the flown config, m (default 10)")
    ap.add_argument("--step-xy", type=float, default=5.0, help="blackbox mode: STEP_XY threshold, m per tick (default 5; drift-report)")
    ap.add_argument("--step-z", type=float, default=5.0, help="blackbox mode: STEP_Z threshold, m per tick (default 5; drift-report)")
    ap.add_argument("--frozen-min", type=int, default=2, help="blackbox mode: FROZEN needs this many consecutive ticks (default 2)")
    ap.add_argument("--out", default=os.path.join(HERE, "n1", "faults.csv"))
    ap.add_argument("--resid-out", default=os.path.join(HERE, "n1", "nav_residuals.csv"))
    ap.add_argument("--step", type=float, default=5.0, help="position residual (m) reported as a step (default 5)")
    ap.add_argument("--top", type=int, default=5)
    args = ap.parse_args()
    if args.blackbox:
        blackbox_mode(args)
        return
    if not args.csv:
        ap.error("a xiao CSV is required unless --blackbox")
    by = collections.OrderedDict()
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            by.setdefault(r["span_id"], []).append(r)
    for sid, rows in by.items():
        scan(sid, rows, args.step, args.top)


if __name__ == "__main__":
    main()
