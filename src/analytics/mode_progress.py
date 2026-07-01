"""Per-generation mode-competence trend (M2) — perception / track / range / blind-streak.

Reward-tuning moves the score; what matters for "is the controller getting better at the
ACTUAL job" is the mode competence over generations. Cheap (log-only, #GenDiag per gen):
  avgVis    — fraction of ticks the target is in FOV (perception competence)
  avgInRamp — fraction of ticks in the streak ramp (track occupancy)
  avgRngMed — median range to target (m) (how close it holds)
  maxLost   — worst consecutive blind/lost streak (ticks) (reacquire failure)

A plateau across these = the mode competence has saturated (architecture ceiling) even if
the elite fitness keeps climbing. Overlays runs for A/B.

Usage:
  python3 src/analytics/mode_progress.py \
    --run t11:logs/autoc-037-t11-m2.log \
    --run t14:logs/autoc-037-t14-m2-hullpen-oob-ramp.log \
    --run t15:logs/autoc-037-t15-m2-streak0p3.log \
    -o specs/037-20hz-control-loop/mode_progress.png
"""
import argparse
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GD = re.compile(r"#GenDiag gen=(\d+).*?avgVis=([\d.]+).*?avgInRamp=([\d.]+).*?"
                r"avgRngMed=([\d.]+).*?maxLost=(\d+)")

# maxLost is tick-denominated in the log → reads 2× at 20 Hz vs 10 Hz. Converted
# to SECONDS at parse time (× --tick-sec) so reacquire is cadence-comparable (038 T004).
PANELS = [("avgVis", "target in FOV (perception)", 1),
          ("avgInRamp", "track occupancy (in streak ramp)", 2),
          ("avgRngMed", "median range to target (m)", 3),
          ("maxLost", "worst blind streak (s)", 4)]


def parse(path):
    g = []
    for line in open(path, errors="replace"):
        m = GD.search(line)
        if m:
            g.append(tuple(float(x) for x in m.groups()))
    if not g:
        return None
    a = np.array(g)
    return {"gen": a[:, 0], "avgVis": a[:, 1], "avgInRamp": a[:, 2],
            "avgRngMed": a[:, 3], "maxLost": a[:, 4]}


def roll(y, w=11):
    if len(y) < w:
        return y
    pad = w // 2
    yp = np.pad(y, pad, mode="edge")
    return np.array([np.median(yp[i:i + w]) for i in range(len(y))])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", action="append", required=True, help="NAME:logfile (repeatable)")
    p.add_argument("-o", "--out", required=True)
    p.add_argument("--tick-sec", type=float, default=0.05,
                   help="control-loop cadence (sec/tick) — converts maxLost (blind-streak ticks) "
                        "to SECONDS so reacquire is cadence-comparable (default 0.05 = 20 Hz).")
    args = p.parse_args()

    runs = []
    for spec in args.run:
        name, _, path = spec.partition(":")
        d = parse(path)
        if d:
            d["maxLost"] = d["maxLost"] * args.tick_sec  # ticks → seconds (T004)
            runs.append((name, d))
    if not runs:
        print("no #GenDiag parsed"); return

    colors = ["tab:blue", "tab:red", "tab:green", "tab:purple", "tab:orange"]
    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    axs = axs.ravel()
    for pi, (key, ylab, _) in enumerate(PANELS):
        ax = axs[pi]
        for k, (name, d) in enumerate(runs):
            c = colors[k % len(colors)]
            ax.plot(d["gen"], d[key], ".", color=c, alpha=0.15, ms=3)
            ax.plot(d["gen"], roll(d[key]), "-", color=c, lw=1.8, label=name)
        ax.set_title(ylab, fontsize=10); ax.set_xlabel("generation")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)

    print("=== final-gen mode competence ===")
    for name, d in runs:
        print(f"  {name:>5} gen{int(d['gen'][-1]):>4}: avgVis {d['avgVis'][-1]:.2f}  "
              f"avgInRamp {d['avgInRamp'][-1]:.2f}  avgRngMed {d['avgRngMed'][-1]:.1f}m  "
              f"maxLost {d['maxLost'][-1]:.1f}s")
    fig.suptitle("Per-generation mode competence (perception / track / range / reacquire) — "
                 "plateau = competence ceiling", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
