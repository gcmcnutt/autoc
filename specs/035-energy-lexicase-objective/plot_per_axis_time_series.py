#!/usr/bin/env python3
"""035 FR-P03/FR-005 — per-axis time series + aggressiveness from dmp-dump CSV.

New 035 consumer of the dmp-dump CSV contract (the pre-035 plot scripts are
historical and left untouched). Reads `dmp-dump ... --csv-only` output and
emits a per-axis (pitch/roll/throttle) time-series PNG plus the per-axis
aggressiveness comparator dCtrl/<|out|> — the config-stable signal used to
compare runs across variation ramps (project_late_run_fitness_interpretation).

Stdlib csv + numpy + matplotlib only (no pandas). Usage:
    dmp-dump s3://autoc-m1/<run>/gen<N>.dmp.zst --csv-only \\
        | python3 plot_per_axis_time_series.py -o out.png
    python3 plot_per_axis_time_series.py csv_file.csv -o out.png [--scenario K]
"""
import argparse
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("out_pt", "pitch"), ("out_rl", "roll"), ("out_th", "throttle")]


def aggressiveness(out: np.ndarray) -> float:
    """dCtrl/<|out|>: mean abs tick-to-tick change over mean abs output.
    Variation-stable per-axis comparator (->1 bang-bang, ->0 smooth)."""
    if out.size < 2:
        return float("nan")
    mag = np.mean(np.abs(out))
    return float(np.mean(np.abs(np.diff(out))) / mag) if mag > 1e-9 else float("nan")


def load_csv(src):
    """Return dict of column-name -> np.ndarray(float) from a dmp-dump CSV."""
    reader = csv.DictReader(src)
    cols = {name: [] for name in (reader.fieldnames or [])}
    for row in reader:
        for k, v in row.items():
            cols[k].append(v)
    return {k: np.array([float(x) if x not in ("", None) else np.nan for x in vs])
            for k, vs in cols.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-",
                    help="dmp-dump --csv-only file, or '-' for stdin")
    ap.add_argument("-o", "--output", default="per_axis_time_series.png")
    ap.add_argument("--scenario", type=int, default=None,
                    help="plot a single scenario index (default: first)")
    args = ap.parse_args()

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        data = load_csv(fh)
    finally:
        if fh is not sys.stdin:
            fh.close()

    for col, _ in AXES + [("scenario", ""), ("tick", "")]:
        if col not in data:
            sys.stderr.write(f"error: column '{col}' not in CSV "
                             f"(got {list(data.keys())})\n")
            return 1

    # Per-axis aggressiveness over the WHOLE run (all scenarios) — the headline.
    print("per-axis aggressiveness dCtrl/<|out|> (all scenarios):")
    for col, name in AXES:
        print(f"  {name:8s} {aggressiveness(data[col]):.4f}")

    scen = data["scenario"]
    sc = args.scenario if args.scenario is not None else int(scen[0])
    mask = scen == sc
    if not mask.any():
        sys.stderr.write(f"error: scenario {sc} not present\n")
        return 1
    n_scen = len(np.unique(scen))

    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    tick = data["tick"][mask]
    for ax, (col, name) in zip(axs, AXES):
        out = data[col][mask]
        ax.plot(tick, out, lw=0.8)
        ax.set_ylabel(f"{name}\n(out, tanh)")
        ax.axhline(0, color="k", lw=0.4, alpha=0.4)
        ax.grid(True, alpha=0.3)
        ax.text(0.99, 0.95, f"aggr={aggressiveness(out):.3f}",
                transform=ax.transAxes, ha="right", va="top", fontsize=8)
    axs[-1].set_xlabel("tick")
    axs[0].set_title(f"Per-axis NN output — scenario {sc} ({n_scen} scenarios in run)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
