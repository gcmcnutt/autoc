#!/usr/bin/env python3
"""035 M1 per-axis control aggressiveness — single-generation snapshot.

New 035 consumer of the dmp-dump CSV contract (the pre-035 data.dat-fed
per_axis_aggressiveness.py is historical, left untouched). Format matches the
034 6-panel chart EXACTLY (3 axes × {<|Δ|>, <|out|>}) including the spec-gate
**budget goal lines** — only the data source changes (dmp-dump CSV, not data.dat).

Per axis X in {pt, rl, th}, per scenario:
    dctrl_X[scn] = mean_t |out_X[t] - out_X[t-1]|     (tick-to-tick change)
    mag_X[scn]   = mean_t |out_X[t]|                  (output amplitude)

Stdlib csv + numpy + matplotlib (no pandas). Usage:
    dmp-dump s3://autoc-m1/<run>/ --csv-only \\
        | python3 per_axis_aggressiveness.py --label <name> --gen <N> -o out.png
    python3 per_axis_aggressiveness.py csv_file.csv -o out.png
"""
import argparse
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# axis col, label, color (matches 034)
AXES = [("out_pt", "pitch", "tab:red"),
        ("out_rl", "roll", "tab:blue"),
        ("out_th", "throttle", "tab:green")]

# Spec-gate per-axis budgets (sum-over-axes ≤ 0.80 dctrl / ≤ 2.00 amplitude),
# the dashed "goal" lines — same as 034 so runs compare directly.
DCTRL_BUDGET = 0.27   # ~ 0.80 / 3
MAG_BUDGET = 0.67     # ~ 2.00 / 3


def load_csv(src):
    reader = csv.DictReader(src)
    need = {"scenario", "out_pt", "out_rl", "out_th"}
    missing = need - set(reader.fieldnames or [])
    if missing:
        sys.stderr.write(f"error: CSV missing {sorted(missing)} "
                         f"(got {reader.fieldnames})\n")
        sys.exit(1)
    rows = {"scenario": [], "out_pt": [], "out_rl": [], "out_th": []}
    for r in reader:
        for k in rows:
            rows[k].append(r[k])
    return {k: np.array([float(x) if x not in ("", None) else np.nan for x in v])
            for k, v in rows.items()}


def per_scenario(data, col):
    """(dctrl[], mag[]) — one entry per scenario."""
    scen = data["scenario"]
    dctrl, mag = [], []
    for sc in np.unique(scen):
        out = data[col][scen == sc]
        out = out[~np.isnan(out)]
        if out.size < 2:
            continue
        dctrl.append(float(np.mean(np.abs(np.diff(out)))))
        mag.append(float(np.mean(np.abs(out))))
    return np.array(dctrl), np.array(mag)


def summarize(vals, label, budget):
    n = len(vals)
    if n == 0:
        print(f"  {label:8s} (no data)")
        return
    over = int(np.sum(vals > budget))
    print(f"  {label:8s} mean {np.mean(vals):.3f} ± {np.std(vals):.3f}   "
          f"p50/p95 {np.median(vals):.3f}/{np.percentile(vals, 95):.3f}   "
          f"({over}/{n}={100*over/n:.0f}% over budget {budget})")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-",
                    help="dmp-dump --csv-only file, or '-' for stdin")
    ap.add_argument("-o", "--output", default="per_axis_aggressiveness.png")
    ap.add_argument("--label", default="run")
    ap.add_argument("--gen", type=int, default=-1, help="gen for the title")
    args = ap.parse_args()

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        data = load_csv(fh)
    finally:
        if fh is not sys.stdin:
            fh.close()

    n_scenarios = len(np.unique(data["scenario"]))
    aggr = {name: per_scenario(data, col) for col, name, _ in AXES}

    gen_str = f"gen {args.gen}" if args.gen >= 0 else "latest gen"
    print(f"=== {args.label}: per-axis aggressiveness, {gen_str} "
          f"({n_scenarios} scenarios) ===")
    print("  --- dctrl <|Δ|> per axis ---")
    for _, name, _ in AXES:
        summarize(aggr[name][0], name, DCTRL_BUDGET)
    print("  --- amplitude <|out|> per axis ---")
    for _, name, _ in AXES:
        summarize(aggr[name][1], name, MAG_BUDGET)
    print(f"\n  Spec-gate per-axis budgets: dctrl ≤ {DCTRL_BUDGET}, "
          f"amplitude ≤ {MAG_BUDGET}  (sum-over-axes ≤ 0.80 / ≤ 2.00)")

    # 6 panels: 3 axes × {dctrl, mag}. Matches 034.
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=False)
    for i, (col, name, color) in enumerate(AXES):
        dvals, mvals = aggr[name]
        ax_d, ax_m = axes[i]
        ax_d.hist(dvals, bins=40, color=color, alpha=0.75, edgecolor="white")
        ax_d.axvline(DCTRL_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"budget {DCTRL_BUDGET}")
        ax_d.axvline(np.mean(dvals), color="black", linestyle="-", linewidth=0.8,
                     label=f"mean {np.mean(dvals):.3f}")
        ax_d.set_xlabel(f"{name} <|Δ|>")
        ax_d.set_ylabel("scenario count")
        ax_d.legend(fontsize=8, loc="upper right")
        ax_d.grid(True, linewidth=0.3, alpha=0.4)

        ax_m.hist(mvals, bins=40, color=color, alpha=0.75, edgecolor="white")
        ax_m.axvline(MAG_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"budget {MAG_BUDGET}")
        ax_m.axvline(np.mean(mvals), color="black", linestyle="-", linewidth=0.8,
                     label=f"mean {np.mean(mvals):.3f}")
        ax_m.set_xlabel(f"{name} <|out|>")
        ax_m.set_ylabel("scenario count")
        ax_m.legend(fontsize=8, loc="upper right")
        ax_m.grid(True, linewidth=0.3, alpha=0.4)

    fig.suptitle(f"{args.label} — per-axis aggressiveness, {gen_str} "
                 f"({n_scenarios} scenarios)", fontsize=12)
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"\nwrote {args.output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
