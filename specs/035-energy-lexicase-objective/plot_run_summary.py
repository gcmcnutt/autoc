#!/usr/bin/env python3
"""035 — evolution/energy/aggressiveness over generations, OFF THE DMPS.

Consumes `dmp-dump --run-summary` output (per-gen aggregate CSV computed from
the elite dmps, not the run .log). This is the "analytics off the log" path:
best_fitness here is byte-verified equal to the log's #NNGen best, but it also
carries the energy/stability/streak/crash + per-axis aggressiveness trend that
the energy objective (US1) is graded on.

Panels: (1) best fitness, (2) mean energy_score (the throttle-energy cost),
(3) per-axis aggressiveness dCtrl/<|out|>, (4) crashes + mean max-streak.

Stdlib csv + numpy + matplotlib (no pandas); CLI flags only. Usage:
    dmp-dump s3://autoc-m1/<run>/ --run-summary | python3 plot_run_summary.py -o out.png
    python3 plot_run_summary.py summary.csv -o out.png --label <name>
"""
import argparse
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(src):
    reader = csv.DictReader(src)
    cols = {n: [] for n in (reader.fieldnames or [])}
    for row in reader:
        for k, v in row.items():
            cols[k].append(v)
    return {k: np.array([float(x) for x in v]) for k, v in cols.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-", help="run-summary CSV or '-'")
    ap.add_argument("-o", "--output", default="run_summary.png")
    ap.add_argument("--label", default="run")
    args = ap.parse_args()

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        d = load(fh)
    finally:
        if fh is not sys.stdin:
            fh.close()

    need = ["gen", "best_fitness", "mean_energy", "aggr_pitch", "aggr_roll",
            "aggr_throttle", "crashes", "mean_streak"]
    miss = [c for c in need if c not in d]
    if miss:
        sys.stderr.write(f"error: CSV missing {miss} (got {list(d.keys())})\n")
        return 1
    g = d["gen"]

    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    axs[0][0].plot(g, d["best_fitness"], color="navy")
    axs[0][0].set_title("best fitness (elite)")
    axs[0][0].set_ylabel("fitness"); axs[0][0].grid(True, alpha=0.3)

    axs[0][1].plot(g, d["mean_energy"], color="darkorange")
    axs[0][1].set_title("mean energy_score  (throttle-energy cost — US1 target)")
    axs[0][1].set_ylabel("energy_score"); axs[0][1].grid(True, alpha=0.3)

    for col, name, c in [("aggr_pitch", "pitch", "tab:green"),
                         ("aggr_roll", "roll", "tab:red"),
                         ("aggr_throttle", "throttle", "tab:blue")]:
        axs[1][0].plot(g, d[col], label=name, color=c)
    axs[1][0].set_title("per-axis aggressiveness dCtrl/<|out|>")
    axs[1][0].set_xlabel("gen"); axs[1][0].set_ylabel("aggressiveness")
    axs[1][0].legend(); axs[1][0].grid(True, alpha=0.3)

    ax = axs[1][1]
    ax.plot(g, d["crashes"], color="firebrick", label="crashes")
    ax.set_xlabel("gen"); ax.set_ylabel("crashes", color="firebrick")
    ax2 = ax.twinx()
    ax2.plot(g, d["mean_streak"], color="seagreen", label="mean streak")
    ax2.set_ylabel("mean max-streak", color="seagreen")
    ax.set_title("crashes + mean streak"); ax.grid(True, alpha=0.3)

    fig.suptitle(f"{args.label} — run summary off the dmps ({len(g)} gens)")
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
