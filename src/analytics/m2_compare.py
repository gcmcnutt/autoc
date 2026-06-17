"""Cross-run M2 (tracker) comparison — log-only, on comparable axes.

Raw `best` fitness is NOT comparable across M2 runs (nonlinear tracking multiplier,
per-run scenario count + course difficulty, cadence). The comparable measures are:
  - pctInStreak (%)            — a fraction, cadence/scenario-independent
  - max streak in SECONDS      — avgMaxStreak (ticks) × tick-sec, so 10 Hz and
                                 20 Hz runs line up (a 20 Hz tick is 50 ms, a
                                 10 Hz tick 100 ms — raw tick-streak is ~2× at 20 Hz)
`best` is shown only as a faint reference with a "not comparable" caption.

Each run's cadence is read from the log's ControlIntervalMsec (default 100 ms =
10 Hz for pre-037 runs that don't print it). Pure `#NNGen` parsing — no dmp/S3,
so it works after the run buckets are decommissioned.

Usage:
  python3 src/analytics/m2_compare.py \
    --run 034-m2:logs/autoc-034-m2-tracker-origm1-r1.log \
    --run 035-t7-m2:logs/autoc-035-t7-m2-energy-r1.log \
    --run 037-t11-m2:logs/autoc-037-t11-m2.log \
    -o specs/037-20hz-control-loop/m2_compare.png
"""
import argparse
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

NNGEN = re.compile(
    r"#NNGen gen=(\d+) best=([-\d.]+) avg=([-\d.]+) worst=([-\d.]+) "
    r"bestSigma=([\d.]+) avgMaxStreak=([\d.]+) pctInStreak=([\d.]+)")


def load(path):
    """Parse a run .log → dict of arrays + cadence (sec/tick)."""
    g, best, streak, pct = [], [], [], []
    ctrl_msec = None
    with open(path) as f:
        for line in f:
            if ctrl_msec is None and "ControlIntervalMsec" in line:
                mm = re.search(r"ControlIntervalMsec[ =:]+(\d+)", line)
                if mm:
                    ctrl_msec = int(mm.group(1))
            m = NNGEN.search(line)
            if m:
                g.append(int(m.group(1)))
                best.append(float(m.group(2)))
                streak.append(float(m.group(6)))
                pct.append(float(m.group(7)))
    tick_sec = (ctrl_msec or 100) / 1000.0   # default 100 ms = 10 Hz (pre-037)
    return {"gen": np.array(g), "best": np.array(best),
            "streak_s": np.array(streak) * tick_sec, "pct": np.array(pct),
            "tick_sec": tick_sec}


def parse_run(s):
    name, _, path = s.partition(":")
    if not path:
        raise argparse.ArgumentTypeError(f"--run wants NAME:LOG, got {s!r}")
    return name, path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", type=parse_run, action="append", required=True,
                   help="NAME:LOG (repeatable)")
    p.add_argument("--title", default="M2 (tracker) runs — comparable axes")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    runs = []
    for name, path in args.run:
        try:
            d = load(path)
        except OSError as e:
            print(f"skip {name}: {e}", file=sys.stderr); continue
        if len(d["gen"]) == 0:
            print(f"skip {name}: no #NNGen lines", file=sys.stderr); continue
        runs.append((name, d))
    if not runs:
        sys.exit("no runs with #NNGen data")

    colors = ["tab:red", "tab:blue", "tab:green", "tab:purple", "tab:orange"]
    fig, axs = plt.subplots(3, 1, figsize=(11, 11), sharex=True)
    fig.suptitle(args.title + "\npctInStreak + max-streak(s) are comparable; "
                 "best is NOT (nonlinear mult × scenario difficulty × cadence)", fontsize=11)

    for i, (name, d) in enumerate(runs):
        c = colors[i % len(colors)]
        hz = round(1.0 / d["tick_sec"])
        lab = f"{name} ({hz}Hz, {len(d['gen'])}g)"
        axs[0].plot(d["gen"], d["pct"], color=c, lw=1.8, alpha=0.9,
                    label=f"{lab} — end {d['pct'][-1]:.1f}%, pk {d['pct'].max():.1f}%")
        axs[1].plot(d["gen"], d["streak_s"], color=c, lw=1.8, alpha=0.9,
                    label=f"{lab} — end {d['streak_s'][-1]:.2f}s")
        axs[2].plot(d["gen"], d["best"], color=c, lw=1.2, alpha=0.6, label=lab)

    axs[0].set_ylabel("pctInStreak (%)"); axs[0].set_title("tracking — % ticks in cone (COMPARABLE)", fontsize=10)
    axs[1].set_ylabel("avgMaxStreak (seconds)"); axs[1].set_title("max streak in seconds (cadence-normalized, COMPARABLE)", fontsize=10)
    axs[2].set_ylabel("best fitness"); axs[2].set_title("best fitness (NOT comparable — reference only)", fontsize=10)
    axs[2].set_xlabel("generation")
    for ax in axs:
        ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    print("\n=== summary (comparable) ===")
    for name, d in runs:
        print(f"  {name:18s} {round(1/d['tick_sec'])}Hz  gens {len(d['gen']):4d}  "
              f"pctInStreak end {d['pct'][-1]:5.1f}% / pk {d['pct'].max():5.1f}%  "
              f"maxStreak end {d['streak_s'][-1]:.2f}s")


if __name__ == "__main__":
    main()
