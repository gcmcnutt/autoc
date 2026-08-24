#!/usr/bin/env python3
"""035 M1 per-axis aggressiveness time-series — pt/rl/th evolution across gens.

New 035 consumer matching the 034 format's TOP TWO panels (the over-generations
bang-bang + saturation detectors). Reads `dmp-dump --run-summary` CSV (per-gen
dctrl_/mag_ columns) instead of the retired data.dat — the pre-035 data.dat-fed
script is historical, left untouched.

Watch dCtrl per axis trend DOWN over gens (the energy objective's goal) and mag
stay off the full-throw ceiling. Per axis because bang-bang lives on one axis
(roll for the RNN topology); the slope tells which axis is refining vs stuck.

(The 034 chart's bottom two per-path airframe-rotation-rate panels are a
follow-on — they need per-path data the run-summary doesn't emit yet.)

Stdlib csv + numpy + matplotlib. Usage:
    dmp-dump s3://autoc-m1/<run>/ --run-summary | python3 plot_per_axis_time_series.py -o out.png
    python3 plot_per_axis_time_series.py run_summary.csv -o out.png --label <name>
"""
import argparse
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("pitch", "tab:red"), ("roll", "tab:blue"), ("throttle", "tab:green")]
# ============================================================================
# SERVO-DEMAND REFERENCE LINES — provenance, because the name used to overstate
# what these are (operator 2026-08-22: "it is a decent measure of how hard are
# we pushing the servos. Maybe we rename.").
#
# ⛔ THEY ARE NOT A PHYSICAL LIMIT. Traced to the 027 go/no-go gate table
# (specs/027-recurrent-nn/plan.md), which set them as RELATIVE improvement bars
# against the `cadence7` controller:
#     dCtrl  cadence7 ~1.00  ->  <= 0.80   "≥ 20 % reduction in stick speed"
#     |out|  cadence7 ~2.20  ->  <= 2.00   "≥ 10 % reduction in saturation"
# Nothing in them derives from servo slew limits, hinge moments or the airframe.
#
# ⚠️ THREE THINGS THE OLD "budget" LABEL IMPLIED AND SHOULD NOT HAVE:
#   1. The baseline is obsolete — cadence7 was 10 Hz, feedforward, 33 inputs.
#      Beating it is progress against a historical reference, not evidence of
#      nearing a limit.
#   2. The per-axis value is just sum/3. The 027 gate was on the SUM over axes,
#      so one axis at 0.5 with two at 0.1 PASSES the real gate and fails the
#      drawn line; three at 0.28 does the reverse.
#   3. Equal thirds assume the axes are equivalent efforts. They are not —
#      pitch and roll trade geometrically through the bank vector while
#      throttle is decoupled (operator 2026-08-21). regime_control.py budgets
#      the bank PAIR instead, which is the defensible grouping.
#
# ⭐ What they ARE good for, and why they stay: a consistent, comparable measure
# of how hard the controller works the servos, on the same scale every run since
# 027. An EMPIRICAL limit is what 041 P2-10 (variation-sweep eval) is meant to
# establish; replace these references with that result rather than carrying the
# 027-era target forward a fourth time.
# ============================================================================
SERVO_SLEW_REF = 0.27
SERVO_DEFLECT_REF = 0.67
DCTRL_BUDGET = SERVO_SLEW_REF     # back-compat alias
MAG_BUDGET = SERVO_DEFLECT_REF    # back-compat alias


def load(src):
    reader = csv.DictReader(src)
    cols = {n: [] for n in (reader.fieldnames or [])}
    for row in reader:
        for k, v in row.items():
            cols[k].append(v)
    return {k: np.array([float(x) for x in v]) for k, v in cols.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="-",
                    help="dmp-dump --run-summary CSV, or '-' for stdin")
    ap.add_argument("-o", "--output", default="per_axis_time_series.png")
    ap.add_argument("--label", default="run")
    ap.add_argument("--total-gens", type=int, default=None)
    args = ap.parse_args()

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        d = load(fh)
    finally:
        if fh is not sys.stdin:
            fh.close()

    need = ["gen"] + [f"dctrl_{n}" for n, _ in AXES] + [f"mag_{n}" for n, _ in AXES]
    miss = [c for c in need if c not in d]
    if miss:
        sys.stderr.write(f"error: CSV missing {miss} — needs dmp-dump --run-summary "
                         f"(got {list(d.keys())})\n")
        return 1
    gen = d["gen"]
    n_paths = sum(1 for p in range(6) if f"path{p}_rollrate" in d)

    fig, axes = plt.subplots(4, 1, figsize=(11, 14), sharex=True)

    # Panel 1 — change rate (bang-bang detector)
    for name, color in AXES:
        axes[0].plot(gen, d[f"dctrl_{name}"], label=name, color=color, lw=1.0)
    axes[0].axhline(DCTRL_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                    label=f"servo-slew ref {SERVO_SLEW_REF} (027; sum 0.80)")
    axes[0].set_ylabel("Mean |dctrl| per tick\n(slick rate)")
    axes[0].set_title("change rate (bang-bang detector) — watch dCtrl trend DOWN")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[0].grid(True, linewidth=0.3, alpha=0.4)

    # Panel 2 — amplitude (saturation detector)
    for name, color in AXES:
        axes[1].plot(gen, d[f"mag_{name}"], label=name, color=color, lw=1.0)
    axes[1].axhline(MAG_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                    label=f"servo-deflection ref {SERVO_DEFLECT_REF} (027; sum 2.00)")
    axes[1].axhline(1.0, color="black", linestyle=":", linewidth=0.8,
                    label="full-throw ceiling 1.0")
    axes[1].set_ylabel("Mean |out| per tick\n(amplitude)")
    axes[1].set_title("amplitude (saturation detector)")
    axes[1].legend(fontsize=8, loc="upper right")
    axes[1].grid(True, linewidth=0.3, alpha=0.4)

    # Panels 3 & 4 — per-path airframe rotation rate (deg/sec)
    path_colors = plt.cm.tab10(np.linspace(0, 1, max(n_paths, 1)))
    for which, ax, lbl in [("rollrate", axes[2], "roll"), ("pitchrate", axes[3], "pitch")]:
        for p in range(n_paths):
            col = f"path{p}_{which}"
            if col in d:
                ax.plot(gen, d[col], label=f"path {p}", color=path_colors[p], lw=0.9)
        ax.set_ylabel(f"per-path {lbl} rate\n(deg/sec)")
        ax.set_title(f"per-path airframe {lbl}-rotation RATE (deg/sec, normalized by duration)")
        ax.legend(fontsize=7, loc="upper right", ncol=2)
        ax.grid(True, linewidth=0.3, alpha=0.4)
    axes[3].set_xlabel("Generation")

    if args.total_gens:
        axes[3].set_xlim(0, args.total_gens)

    fig.suptitle(f"{args.label} — per-axis aggressiveness time series\n"
                 f"top: change rate (bang-bang) / amplitude (saturation); "
                 f"bottom: per-path airframe rotation rate", fontsize=12)
    fig.tight_layout()
    fig.savefig(args.output, dpi=110)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
