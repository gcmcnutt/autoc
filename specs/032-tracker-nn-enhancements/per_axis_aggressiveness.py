#!/usr/bin/env python3
"""030 per-axis control aggressiveness — single-generation snapshot.

Adapted from 029's per_axis_aggressiveness.py. Body unchanged — column
lookup is by header name (outPt/outRl/outTh + Scn + Pth/Wnd:Step:),
which exist identically in tracker-mode data.dat per the M2 schema.
Only defaults differ (output path, label).

For one generation's data.dat trajectories (one trajectory per path/wind
scenario, ~245 total), compute per-axis aggregates:

    dctrl_X = mean over consecutive ticks of |out_X[t] - out_X[t-1]|
    mag_X   = mean over ticks of |out_X|

where X ∈ {pt, rl, th}. Default: latest gen in the file.

Why per-axis: spec §Q4 routing decision needs to know which axis the
elite is bang-bang on, not the sum across axes. Different controllers
bang-bang on different axes (rnn1 = roll, cadence7 flight = pitch).

Output:
- Text summary: per-axis mean ± stddev across paths, plus distribution
  quantiles. Spec-gate per-axis budgets (sum÷3) shown for context.
- PNG: 6 histograms (3 axes × {dctrl, mag}), one bar per scenario.

Usage:
    python3 per_axis_aggressiveness.py [--in data.dat] [--gen N]
                                       [--pop-size N] [--label NAME]
                                       [--out PNG]
"""

import argparse
from collections import defaultdict
from pathlib import Path
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_pth_wnd(field: str):
    pw, _step, _ = field.split(":")
    path, wind = pw.split("/")
    return int(path), int(wind)


def detect_max_gen(path: Path, pop_size: int) -> int:
    """Scan data.dat to find the highest gen present (using gen = scn // (pop+1))."""
    div = pop_size + 1
    max_gen = -1
    col_scn = None
    with open(path) as f:
        header = next(f).split()
        col_scn = header.index("Scn")
        for raw in f:
            parts = raw.split()
            if len(parts) <= col_scn:
                continue
            try:
                scn = int(parts[col_scn])
            except ValueError:
                continue
            g = scn // div
            if g > max_gen:
                max_gen = g
    return max_gen


def load_for_gen(path: Path, target_gen: int, pop_size: int):
    """Stream rows for the target gen only.
    Returns per_scenario[(path,wind)] = (dpt, drl, dth, mpt, mrl, mth) means."""
    out = {}
    div = pop_size + 1
    header = None
    cols = {}
    current_key = None
    current_prev = None
    sums = {"dpt": 0.0, "drl": 0.0, "dth": 0.0,
            "mpt": 0.0, "mrl": 0.0, "mth": 0.0}
    steps = 0

    def reset_sums():
        for k in sums:
            sums[k] = 0.0

    def flush():
        nonlocal current_key, steps, current_prev
        if current_key is not None and steps > 0:
            (path_idx, wind_idx, _scn) = current_key
            n = float(steps)
            out[(path_idx, wind_idx)] = (
                sums["dpt"] / n, sums["drl"] / n, sums["dth"] / n,
                sums["mpt"] / n, sums["mrl"] / n, sums["mth"] / n)
        current_key = None
        reset_sums()
        steps = 0
        current_prev = None

    with open(path) as f:
        for raw in f:
            parts = raw.split()
            if header is None:
                header = parts
                cols["scn"] = header.index("Scn")
                cols["pw"] = header.index("Pth/Wnd:Step:")
                cols["pt"] = header.index("outPt")
                cols["rl"] = header.index("outRl")
                cols["th"] = header.index("outTh")
                continue
            if len(parts) <= max(cols.values()):
                continue
            try:
                scn = int(parts[cols["scn"]])
                gen = scn // div
                if gen != target_gen:
                    if current_key is not None:
                        flush()
                    continue
                path_idx, wind_idx = parse_pth_wnd(parts[cols["pw"]])
                pt = float(parts[cols["pt"]])
                rl = float(parts[cols["rl"]])
                th = float(parts[cols["th"]])
            except ValueError:
                continue
            key = (path_idx, wind_idx, scn)
            if key != current_key:
                flush()
                current_key = key
                current_prev = (pt, rl, th)
                sums["mpt"] += abs(pt)
                sums["mrl"] += abs(rl)
                sums["mth"] += abs(th)
                steps = 1
                continue
            ppt, prl, pth_ = current_prev
            sums["dpt"] += abs(pt - ppt)
            sums["drl"] += abs(rl - prl)
            sums["dth"] += abs(th - pth_)
            sums["mpt"] += abs(pt)
            sums["mrl"] += abs(rl)
            sums["mth"] += abs(th)
            steps += 1
            current_prev = (pt, rl, th)
    flush()
    return out


def summarize(values, label, gate_budget):
    if not values:
        return None
    n = len(values)
    mean = statistics.mean(values)
    sd = statistics.stdev(values) if n > 1 else 0.0
    sorted_v = sorted(values)
    p25 = sorted_v[int(0.25 * n)]
    p50 = sorted_v[int(0.50 * n)]
    p75 = sorted_v[int(0.75 * n)]
    p95 = sorted_v[int(0.95 * n)]
    pmax = max(values)
    over = sum(1 for v in values if v > gate_budget)
    over_pct = 100.0 * over / n
    print(f"  {label:>10}: mean={mean:.3f} σ={sd:.3f}   "
          f"p25={p25:.3f} p50={p50:.3f} p75={p75:.3f} p95={p95:.3f} max={pmax:.3f}   "
          f"({over}/{n}={over_pct:.0f}% over budget {gate_budget})")
    return mean, sd, sorted_v


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="inp", type=Path, default=Path("data.dat"))
    p.add_argument("--gen", type=int, default=-1,
                   help="generation to analyze (-1 = autodetect latest)")
    p.add_argument("--pop-size", type=int, default=5000,
                   help="autoc.ini PopulationSize (rnn1=3500, rnn2=5000, rnn3=5000)")
    p.add_argument("--label", default="run")
    p.add_argument("--out", type=Path,
                   default=Path("specs/032-tracker-nn-enhancements/per_axis_aggressiveness.png"))
    args = p.parse_args()

    if not args.inp.is_file():
        raise SystemExit(f"input not found: {args.inp}")

    if args.gen < 0:
        print(f"Scanning {args.inp} for max gen…", file=sys.stderr)
        args.gen = detect_max_gen(args.inp, args.pop_size)
        print(f"  → latest gen = {args.gen}", file=sys.stderr)

    print(f"Loading {args.inp} for gen={args.gen} (pop_size={args.pop_size})…",
          file=sys.stderr)
    data = load_for_gen(args.inp, args.gen, args.pop_size)
    if not data:
        raise SystemExit(f"no rows found for gen={args.gen} (check --pop-size)")

    n_scenarios = len(data)
    print(f"  → {n_scenarios} scenarios at gen {args.gen}", file=sys.stderr)

    # Per-axis aggregates
    dpt = [v[0] for v in data.values()]
    drl = [v[1] for v in data.values()]
    dth = [v[2] for v in data.values()]
    mpt = [v[3] for v in data.values()]
    mrl = [v[4] for v in data.values()]
    mth = [v[5] for v in data.values()]

    # Spec-gate per-axis budgets (sum ≤ 0.80 / ≤ 2.00 split across 3 axes)
    DCTRL_BUDGET = 0.27   # ~ 0.80 / 3
    MAG_BUDGET = 0.67     # ~ 2.00 / 3

    print(f"\n=== {args.label}: per-axis aggressiveness, gen {args.gen}, "
          f"{n_scenarios} scenarios ===")
    print("\n  --- dctrl <|Δ|> per axis ---")
    summarize(dpt, "pitch", DCTRL_BUDGET)
    summarize(drl, "roll", DCTRL_BUDGET)
    summarize(dth, "throttle", DCTRL_BUDGET)
    print("\n  --- amplitude <|out|> per axis ---")
    summarize(mpt, "pitch", MAG_BUDGET)
    summarize(mrl, "roll", MAG_BUDGET)
    summarize(mth, "throttle", MAG_BUDGET)

    print(f"\n  Spec-gate per-axis budgets: dctrl ≤ {DCTRL_BUDGET}, "
          f"amplitude ≤ {MAG_BUDGET}  (sum-over-axes ≤ 0.80 / ≤ 2.00)")

    # Distribution histograms (6 panels: 3 axes × 2 metrics)
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=False)
    rows = [
        ("pitch",    dpt, mpt, "tab:red"),
        ("roll",     drl, mrl, "tab:blue"),
        ("throttle", dth, mth, "tab:green"),
    ]
    for i, (name, dvals, mvals, color) in enumerate(rows):
        ax_d, ax_m = axes[i]
        ax_d.hist(dvals, bins=40, color=color, alpha=0.75, edgecolor="white")
        ax_d.axvline(DCTRL_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"budget {DCTRL_BUDGET}")
        ax_d.axvline(statistics.mean(dvals), color="black", linestyle="-",
                     linewidth=0.8, label=f"mean {statistics.mean(dvals):.3f}")
        ax_d.set_xlabel(f"{name} <|Δ|>")
        ax_d.set_ylabel("scenario count")
        ax_d.legend(fontsize=8, loc="upper right")
        ax_d.grid(True, linewidth=0.3, alpha=0.4)

        ax_m.hist(mvals, bins=40, color=color, alpha=0.75, edgecolor="white")
        ax_m.axvline(MAG_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"budget {MAG_BUDGET}")
        ax_m.axvline(statistics.mean(mvals), color="black", linestyle="-",
                     linewidth=0.8, label=f"mean {statistics.mean(mvals):.3f}")
        ax_m.set_xlabel(f"{name} <|out|>")
        ax_m.set_ylabel("scenario count")
        ax_m.legend(fontsize=8, loc="upper right")
        ax_m.grid(True, linewidth=0.3, alpha=0.4)

    fig.suptitle(f"{args.label} — per-axis aggressiveness, gen {args.gen} "
                 f"({n_scenarios} scenarios)", fontsize=12)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
