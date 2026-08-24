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
SERVO_SLEW_REF = 0.27    # = 0.80 / 3 — 027 gate, cadence7 −20 % stick speed
SERVO_DEFLECT_REF = 0.67  # = 2.00 / 3 — 027 gate, cadence7 −10 % saturation

# Back-compat aliases (other modules / older invocations may import these).
DCTRL_BUDGET = SERVO_SLEW_REF
MAG_BUDGET = SERVO_DEFLECT_REF


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
          f"({over}/{n}={100*over/n:.0f}% over servo-demand ref {budget})")


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
    print(f"\n  Servo-demand refs (027 gate, NOT a physical limit): slew ≤ {SERVO_SLEW_REF}, "
          f"amplitude ≤ {MAG_BUDGET}  (sum-over-axes ≤ 0.80 / ≤ 2.00)")

    # 6 panels: 3 axes × {dctrl, mag}. Matches 034.
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=False)
    for i, (col, name, color) in enumerate(AXES):
        dvals, mvals = aggr[name]
        ax_d, ax_m = axes[i]
        ax_d.hist(dvals, bins=24, color=color, alpha=0.75, edgecolor="white")
        ax_d.axvline(DCTRL_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"servo-slew ref {SERVO_SLEW_REF} (027)")
        ax_d.axvline(np.mean(dvals), color="black", linestyle="-", linewidth=0.8,
                     label=f"mean {np.mean(dvals):.3f}")
        ax_d.set_xlabel(f"{name} <|Δ|>")
        ax_d.set_ylabel("scenario count")
        ax_d.legend(fontsize=8, loc="upper right")
        ax_d.grid(True, linewidth=0.3, alpha=0.4)

        ax_m.hist(mvals, bins=24, color=color, alpha=0.75, edgecolor="white")
        ax_m.axvline(MAG_BUDGET, color="gray", linestyle="--", linewidth=0.8,
                     label=f"servo-deflection ref {SERVO_DEFLECT_REF} (027)")
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
