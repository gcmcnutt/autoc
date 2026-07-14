#!/usr/bin/env python3
"""039 — per-flight report from a v3 xiao log + INAV blackbox.

Answers the flight-review questions the logs can answer without eyeballs:
  - load factor: accSmooth[2] (g) inside each engaged span (max pull, min/unload)
  - tracking quality: |pos - rabbit| per tick, per span and overall
  - per-axis control character: amplitude, dCtrl, saturation, reversal rate,
    and body-rate RMS (the pitch-oscillation-vs-smooth-roll question)

Usage: flight_report.py flight.bin blackbox.01.csv [--plot out.png]
"""

import argparse
import csv
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DECODER = os.path.join(HERE, "..", "..", "src", "analytics", "flightlog_decode.py")
spec = importlib.util.spec_from_file_location("flightlog_decode", DECODER)
fld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fld)


def fit_clock(anchors):
    n = len(anchors)
    sx = sum(x for x, _ in anchors)
    sy = sum(y for _, y in anchors)
    sxx = sum(x * x for x, _ in anchors)
    sxy = sum(x * y for x, y in anchors)
    denom = n * sxx - sx * sx
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    return a, b


def load_blackbox_acc(path):
    """[(t_ms, acc_g[3])] sorted."""
    out = []
    with open(path) as f:
        reader = csv.reader(f)
        hdr = [h.strip() for h in next(reader)]
        idx = {name: i for i, name in enumerate(hdr)}
        c_t = idx["time (us)"]
        c_a = [idx[f"accSmooth[{i}] (g)"] for i in range(3)]
        for r in reader:
            try:
                out.append((float(r[c_t]) / 1000.0, [float(r[c]) for c in c_a]))
            except (ValueError, IndexError):
                continue
    out.sort(key=lambda r: r[0])
    return out


def pct(sorted_vals, q):
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * q))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("flight_bin")
    ap.add_argument("blackbox_csv")
    ap.add_argument("--plot", metavar="PNG")
    args = ap.parse_args()

    with open(args.flight_bin, "rb") as f:
        blob = f.read()
    header, spans, events, _warn, _fs = fld.decode(blob)
    print(f"flight log v{header['format_version']}  program: {header['program']}")

    anchors = [(e["timestamp_ms"], e["value"]) for e in events if e["code"] == 11]
    a, b = fit_clock(anchors)
    print(f"clock: inav_ms = {a:.6f}*xiao_ms + {b:.1f} ({len(anchors)} anchors)")

    engages = {e["value"]: e["timestamp_ms"] for e in events if e["code"] == 3}
    disengages = {e["value"]: e["timestamp_ms"] for e in events if e["code"] == 4}

    bb = load_blackbox_acc(args.blackbox_csv)

    print("\n== load factor (blackbox accSmooth[2], g) ==")
    overall_max, overall_min = -99.0, 99.0
    overall_where = None
    for s in spans:
        sid = s["engage"]["span_id"]
        t0 = a * engages[sid] + b
        t1 = a * disengages[sid] + b
        window = [(t, acc[2]) for t, acc in bb if t0 <= t <= t1]
        if not window:
            print(f"  span {sid}: no blackbox coverage")
            continue
        gz = [g for _, g in window]
        gmax, gmin = max(gz), min(gz)
        tmax = next(t for t, g in window if g == gmax)
        print(f"  span {sid} (path {s['engage'].get('path_index')}): "
              f"max +{gmax:.2f} g  min {gmin:+.2f} g  "
              f"({len(window)} samples, peak at inav t={tmax / 1000.0:.1f} s)")
        if gmax > overall_max:
            overall_max, overall_where = gmax, sid
        overall_min = min(overall_min, gmin)
    print(f"  OVERALL: max +{overall_max:.2f} g (span {overall_where}), min {overall_min:+.2f} g")

    print("\n== tracking: |pos - rabbit| per tick (m) ==")
    all_d = []
    for s in spans:
        sid = s["engage"]["span_id"]
        d = [math.dist((r["pos_n"], r["pos_e"], r["pos_d"]),
                       (r["rabbit_n"], r["rabbit_e"], r["rabbit_d"]))
             for r in s["ticks"]]
        if not d:
            continue
        ds = sorted(d)
        all_d += d
        n = len(ds)
        print(f"  span {sid} (path {s['engage'].get('path_index')}, {n} ticks): "
              f"mean {sum(ds) / n:.1f}  p50 {pct(ds, 0.5):.1f}  "
              f"p95 {pct(ds, 0.95):.1f}  max {ds[-1]:.1f}")
    ds = sorted(all_d)
    print(f"  OVERALL ({len(ds)} ticks): mean {sum(ds) / len(ds):.1f}  "
          f"p50 {pct(ds, 0.5):.1f}  p95 {pct(ds, 0.95):.1f}  max {ds[-1]:.1f}")

    print("\n== per-axis control character (engaged ticks) ==")
    axes = [("roll", "out_roll", "gyro_p"), ("pitch", "out_pitch", "gyro_q"),
            ("throttle", "out_throttle", None)]
    for name, ocol, gcol in axes:
        u = [r[ocol] for s in spans for r in s["ticks"]]
        du = [abs(u[i] - u[i - 1]) for i in range(1, len(u))]
        sat = sum(1 for v in u if abs(v) > 0.95) / len(u) * 100.0
        # reversal rate: sign changes of du per second (20 Hz ticks)
        signs = [math.copysign(1, u[i] - u[i - 1]) for i in range(1, len(u))
                 if u[i] != u[i - 1]]
        revs = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
        dur_s = len(u) / 20.0
        line = (f"  {name:<9} ⟨|u|⟩={sum(map(abs, u)) / len(u):.3f}  "
                f"⟨|Δu|⟩={sum(du) / len(du):.3f}  sat%={sat:4.1f}  "
                f"reversals/s={revs / dur_s:.1f}")
        if gcol:
            g = [r[gcol] for s in spans for r in s["ticks"]]
            rms = math.sqrt(sum(v * v for v in g) / len(g))
            line += f"  rate RMS={math.degrees(rms):.0f} deg/s"
        print(line)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 1, figsize=(13, 10))
        # Gz with span shading
        t_all = [(t - bb[0][0]) / 1000.0 for t, _ in bb]
        axs[0].plot(t_all, [acc[2] for _, acc in bb], lw=0.6)
        for s in spans:
            sid = s["engage"]["span_id"]
            axs[0].axvspan((a * engages[sid] + b - bb[0][0]) / 1000.0,
                           (a * disengages[sid] + b - bb[0][0]) / 1000.0,
                           alpha=0.15)
        axs[0].set_ylabel("accSmooth z (g)")
        axs[0].set_title("load factor — engaged spans shaded")
        axs[0].grid(alpha=0.3)
        # dist to rabbit
        for s in spans:
            ts = [(a * r["timestamp_ms"] + b - bb[0][0]) / 1000.0 for r in s["ticks"]]
            d = [math.dist((r["pos_n"], r["pos_e"], r["pos_d"]),
                           (r["rabbit_n"], r["rabbit_e"], r["rabbit_d"]))
                 for r in s["ticks"]]
            axs[1].plot(ts, d, lw=0.9, label=f"span {s['engage']['span_id']} "
                                             f"(path {s['engage'].get('path_index')})")
        axs[1].set_ylabel("|pos − rabbit| (m)")
        axs[1].legend(fontsize=8)
        axs[1].grid(alpha=0.3)
        # outputs
        for s in spans:
            ts = [(a * r["timestamp_ms"] + b - bb[0][0]) / 1000.0 for r in s["ticks"]]
            axs[2].plot(ts, [r["out_roll"] for r in s["ticks"]], lw=0.6, color="C0")
            axs[2].plot(ts, [r["out_pitch"] for r in s["ticks"]], lw=0.6, color="C1")
        axs[2].set_ylabel("NN out (roll=blue, pitch=orange)")
        axs[2].set_xlabel("time since blackbox start (s)")
        axs[2].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"\nwrote {args.plot}")


if __name__ == "__main__":
    main()
