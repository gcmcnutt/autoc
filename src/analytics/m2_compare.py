"""Cross-run M2 (tracker) comparison — log-only, on comparable axes.

Raw `best` fitness is NOT comparable across M2 runs (nonlinear tracking multiplier,
per-run scenario count + course difficulty, cadence). The comparable measures are:
  - pctInStreak (%)            — a fraction, cadence/scenario-independent
  - max streak in SECONDS      — avgMaxStreak (ticks) × tick-sec, so 10 Hz and
                                 20 Hz runs line up (a 20 Hz tick is 50 ms, a
                                 10 Hz tick 100 ms — raw tick-streak is ~2× at 20 Hz)
`best` is shown only as a faint reference with a "not comparable" caption.

Each run's cadence is read from its own log's ControlIntervalMsec (041 T009,
shared `cadence.py`). A log that does not state its cadence is an ERROR, not a
10 Hz assumption -- pass --tick-sec for such runs. Pure `#NNGen` parsing -- no
dmp/S3, so it works after the run buckets are decommissioned.

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

from cadence import CadenceUnknown, tick_sec_from_log   # 041 T009

NNGEN = re.compile(
    r"#NNGen gen=(\d+) best=([-\d.]+) avg=([-\d.]+) worst=([-\d.]+) "
    r"bestSigma=([\d.]+) avgMaxStreak=([\d.]+) pctInStreak=([\d.]+)")


def load(path, tick_sec_override=None):
    """Parse a run .log → dict of arrays + cadence (sec/tick)."""
    g, best, streak, pct = [], [], [], []
    with open(path) as f:
        for line in f:
            m = NNGEN.search(line)
            if m:
                g.append(int(m.group(1)))
                best.append(float(m.group(2)))
                streak.append(float(m.group(6)))
                pct.append(float(m.group(7)))
    # 041 T009 — cadence comes from the log via the shared helper. The previous
    # `(ctrl_msec or 100)/1000` silently assumed 10 Hz whenever the config print
    # was missing, which halves every streak-second in a 20 Hz run and reads as
    # a competence difference in a CROSS-RUN comparison plot — the one place the
    # error is least visible. It now raises instead.
    tick_sec = tick_sec_from_log(path, tick_sec_override)
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
    p.add_argument("--tick-sec", type=float, default=None,
                   help="control cadence (sec/tick) for runs whose log predates the "
                        "config print. Normally omitted -- the log is authoritative, "
                        "and a value disagreeing with it is an error (041 T009).")
    args = p.parse_args()

    runs = []
    for name, path in args.run:
        try:
            d = load(path, args.tick_sec)
        except OSError as e:
            print(f"skip {name}: {e}", file=sys.stderr); continue
        except CadenceUnknown as e:
            # Not skippable: a missing cadence in a CROSS-RUN plot would put two
            # runs on axes that differ by 2x with nothing on the figure saying so.
            sys.exit(f"{name}: {e}")
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
