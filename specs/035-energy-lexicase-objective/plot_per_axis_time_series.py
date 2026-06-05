#!/usr/bin/env python3
"""035 FR-P03/FR-005 — per-axis time series + aggressiveness from dmp-dump CSV.

New 035 consumer of the dmp-dump CSV contract (the pre-035 plot scripts are
historical and left untouched). Reads `dmp-dump ... --csv-only` output and
emits a per-axis (pitch/roll/throttle) time-series PNG plus the per-axis
aggressiveness comparator dCtrl/<|out|> — the config-stable signal used to
compare runs across variation ramps (project_late_run_fitness_interpretation).

Usage:
    dmp-dump s3://autoc-m1/<run>/gen<N>.dmp.zst --csv-only \\
        | python3 plot_per_axis_time_series.py -o out.png
    python3 plot_per_axis_time_series.py csv_file.csv -o out.png [--scenario K]
"""
import argparse
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("out_pt", "pitch"), ("out_rl", "roll"), ("out_th", "throttle")]


def aggressiveness(series: pd.Series) -> float:
    """dCtrl/<|out|>: mean abs tick-to-tick change over mean abs output.
    Variation-stable per-axis comparator (1.0 ~ bang-bang, ->0 smooth)."""
    out = series.to_numpy(dtype=float)
    if out.size < 2:
        return float("nan")
    dctrl = np.mean(np.abs(np.diff(out)))
    mag = np.mean(np.abs(out))
    return float(dctrl / mag) if mag > 1e-9 else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-",
                    help="dmp-dump --csv-only file, or '-' for stdin")
    ap.add_argument("-o", "--output", default="per_axis_time_series.png")
    ap.add_argument("--scenario", type=int, default=None,
                    help="plot a single scenario index (default: first)")
    args = ap.parse_args()

    src = sys.stdin if args.csv == "-" else args.csv
    df = pd.read_csv(src)
    for col, _ in AXES:
        if col not in df.columns:
            sys.stderr.write(f"error: column '{col}' not in CSV "
                             f"(got {list(df.columns)})\n")
            return 1

    # Per-axis aggressiveness over the WHOLE run (all scenarios) — the headline.
    print("per-axis aggressiveness dCtrl/<|out|> (all scenarios):")
    for col, name in AXES:
        print(f"  {name:8s} {aggressiveness(df[col]):.4f}")

    sc = args.scenario if args.scenario is not None else int(df["scenario"].iloc[0])
    one = df[df["scenario"] == sc]
    if one.empty:
        sys.stderr.write(f"error: scenario {sc} not present\n")
        return 1

    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, (col, name) in zip(axs, AXES):
        ax.plot(one["tick"], one[col], lw=0.8)
        ax.set_ylabel(f"{name}\n(out, tanh)")
        ax.axhline(0, color="k", lw=0.4, alpha=0.4)
        ax.grid(True, alpha=0.3)
        ax.text(0.99, 0.95, f"aggr={aggressiveness(one[col]):.3f}",
                transform=ax.transAxes, ha="right", va="top", fontsize=8)
    axs[-1].set_xlabel("tick")
    axs[0].set_title(f"Per-axis NN output — scenario {sc} "
                     f"({len(df['scenario'].unique())} scenarios in run)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
