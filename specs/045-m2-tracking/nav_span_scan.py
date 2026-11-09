#!/usr/bin/env python3
"""
nav_span_scan.py — find nav-estimate faults in a xiao tick CSV (045 PRE-TASK N1 / N3).

Per engaged span: ticks where horizontal velocity is exactly zero (INAV's XY estimate invalid ⇒ frozen position, zero velocity),
position steps that velocity does not explain (residual of Δpos − mean(vel)·Δt), and the largest tick-to-tick steps in velocity,
rabbit, attitude and distance. Pure stdlib, so it runs anywhere the decoder does. First found the 11-08 span 1 faults
(044 flight-analysis-20261108.md § 6): 46 frozen ticks, a 106 m residual at tick 215 → 216, 25 m at 332 → 333.

Usage: nav_span_scan.py xiao.csv [--step 5] [--top 5]     (xiao.csv from src/analytics/flightlog_decode.py -o)
"""
import argparse
import collections
import csv
import math


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


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("csv")
    ap.add_argument("--step", type=float, default=5.0, help="position residual (m) reported as a step (default 5)")
    ap.add_argument("--top", type=int, default=5)
    args = ap.parse_args()
    by = collections.OrderedDict()
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            by.setdefault(r["span_id"], []).append(r)
    for sid, rows in by.items():
        scan(sid, rows, args.step, args.top)


if __name__ == "__main__":
    main()
