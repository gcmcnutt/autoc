#!/usr/bin/env python3
"""035 M1 per-axis control aggressiveness — single-generation snapshot.

New 035 consumer of the dmp-dump CSV contract (the pre-035 data.dat-fed
per_axis_aggressiveness.py is historical and left untouched). Same analysis as
032/033/029: for one generation's trajectories (one per path/wind scenario),
per axis X in {pt, rl, th}:

    dctrl_X[scn] = mean_t |out_X[t] - out_X[t-1]|     (tick-to-tick change)
    mag_X[scn]   = mean_t |out_X[t]|                  (output magnitude)

Why per-axis: routing needs to know WHICH axis the elite is bang-bang on, not
the sum across axes (different controllers bang-bang on different axes).

Output:
- Text: per-axis mean ± std across scenarios + quantiles (the success-criterion
  numbers; compare a 035 elite vs a baseline elite by running twice).
- PNG: 6 histograms (3 axes x {dctrl, mag}), one bar per scenario.

Stdlib csv + numpy + matplotlib (no pandas). Usage:
    dmp-dump s3://autoc-m1/<run>/gen<N>.dmp.zst --csv-only \\
        | python3 per_axis_aggressiveness.py -o out.png --label <name>
    python3 per_axis_aggressiveness.py csv_file.csv -o out.png
"""
import argparse
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("out_pt", "pitch"), ("out_rl", "roll"), ("out_th", "throttle")]


def load_csv(src):
    reader = csv.DictReader(src)
    need = {"scenario", "out_pt", "out_rl", "out_th"}
    missing = need - set(reader.fieldnames or [])
    if missing:
        sys.stderr.write(f"error: CSV missing columns {sorted(missing)} "
                         f"(got {reader.fieldnames})\n")
        sys.exit(1)
    rows = {"scenario": [], "out_pt": [], "out_rl": [], "out_th": []}
    for r in reader:
        for k in rows:
            rows[k].append(r[k])
    return {k: np.array([float(x) if x not in ("", None) else np.nan for x in v])
            for k, v in rows.items()}


def per_scenario(data, col):
    """Return (dctrl[], mag[]) arrays, one entry per scenario."""
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-",
                    help="dmp-dump --csv-only file, or '-' for stdin")
    ap.add_argument("-o", "--output", default="per_axis_aggressiveness.png")
    ap.add_argument("--label", default="run")
    args = ap.parse_args()

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        data = load_csv(fh)
    finally:
        if fh is not sys.stdin:
            fh.close()

    n_scen = len(np.unique(data["scenario"]))
    print(f"=== {args.label}: per-axis aggressiveness, {n_scen} scenarios ===")
    print(f"{'axis':8s} {'dctrl mean±std':>20s} {'mag mean±std':>20s} "
          f"{'dctrl p50/p95':>16s}")
    stats = {}
    for col, name in AXES:
        dctrl, mag = per_scenario(data, col)
        stats[name] = (dctrl, mag)
        print(f"{name:8s} {np.mean(dctrl):8.4f}±{np.std(dctrl):<7.4f}    "
              f"{np.mean(mag):8.4f}±{np.std(mag):<7.4f}   "
              f"{np.median(dctrl):6.4f}/{np.percentile(dctrl,95):6.4f}")

    fig, axes = plt.subplots(3, 2, figsize=(13, 9))
    for row, (col, name) in enumerate(AXES):
        dctrl, mag = stats[name]
        for c, (vals, what) in enumerate([(dctrl, "dctrl |Δout|"), (mag, "mag |out|")]):
            ax = axes[row][c]
            ax.hist(vals, bins=30, color="steelblue", alpha=0.8)
            ax.axvline(np.mean(vals), color="crimson", lw=1.2,
                       label=f"mean {np.mean(vals):.3f}")
            ax.set_title(f"{name} — {what}")
            ax.set_ylabel("scenarios")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
    fig.suptitle(f"{args.label} — per-axis aggressiveness ({n_scen} scenarios)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
