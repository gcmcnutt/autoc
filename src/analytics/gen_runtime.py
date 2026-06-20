"""Per-generation wall-clock curve — a proxy for population diversity / collapse.

Each gen evaluates the whole population across all scenarios; a scenario that
crashes early (OOB / hull) terminates and stops simulating, so total gen time
∝ total sim ticks ∝ how much of the population is flying *full* scenarios vs
dying early. So the runtime curve reads as:
  - rising  → population learning to complete scenarios (more ticks/gen)
  - falling / flattening → more early terminations, or convergence to a single
    (often crashy) behavior — a diversity-loss / collapse signature
Parsed from the per-gen log timestamps (#GenCrash, one line per gen).

CAVEAT: wall-clock also depends on machine load / concurrent runs / worker
count — absolute seconds are noisy across runs; the *shape/trend within a run*
and gross cross-run differences are the signal, not small absolute gaps.

Usage:
  python3 src/analytics/gen_runtime.py \
    --run t11:logs/autoc-037-t11-m2.log \
    --run t12:logs/autoc-037-t12-m2-hullpen.log \
    --run t14:logs/autoc-037-t14-m2-hullpen-oob-ramp.log \
    -o specs/037-20hz-control-loop/gen_runtime.png
"""
import argparse
import re
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TS = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*#GenCrash gen=(\d+)\b")


def parse(path):
    gens, times = [], []
    for line in open(path, errors="replace"):
        m = TS.match(line)
        if not m:
            continue
        gens.append(int(m.group(2)))
        times.append(datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
    # per-gen seconds = delta to previous gen line (drop gen 1 / non-monotone)
    out = []
    for i in range(1, len(gens)):
        if gens[i] == gens[i - 1] + 1:
            dt = (times[i] - times[i - 1]).total_seconds()
            if 0 < dt < 3600:           # drop pauses / clock weirdness
                out.append((gens[i], dt))
    return out


def roll_med(y, w=11):
    y = np.asarray(y, float)
    if len(y) < w:
        return y
    pad = w // 2
    yp = np.pad(y, pad, mode="edge")
    return np.array([np.median(yp[i:i + w]) for i in range(len(y))])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", action="append", required=True, help="NAME:logfile (repeatable)")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    colors = ["tab:blue", "tab:red", "tab:green", "tab:purple", "tab:orange"]
    fig, ax = plt.subplots(figsize=(11, 5.2))
    for i, spec in enumerate(args.run):
        name, _, path = spec.partition(":")
        data = parse(path)
        if not data:
            print(f"{name}: no per-gen timing parsed"); continue
        g = np.array([d[0] for d in data])
        s = np.array([d[1] for d in data])
        sm = roll_med(s)
        c = colors[i % len(colors)]
        ax.plot(g, s, ".", color=c, alpha=0.18, ms=3)
        ax.plot(g, sm, "-", color=c, lw=1.8, label=f"{name}  (median {np.median(s):.1f}s/gen, n={len(g)})")
        early = s[g <= 50]; late = s[g >= max(g) - 50]
        print(f"=== {name}: {len(g)} gens, median {np.median(s):.1f}s/gen "
              f"| early(≤50) {np.median(early):.1f}s | late(last50) {np.median(late):.1f}s "
              f"| ratio late/early {np.median(late)/max(np.median(early),1e-9):.2f}")
    ax.set_xlabel("generation"); ax.set_ylabel("wall-clock seconds / gen")
    ax.set_title("Per-generation wall-clock — proxy for population diversity / collapse\n"
                 "(rising = learning to complete scenarios; faded dots = raw, line = rolling median)",
                 fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
