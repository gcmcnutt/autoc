#!/usr/bin/env python3
"""Fitness-ramp plot, mirroring 023's comparison PNGs.

Parses `Gen N  Best=... Avg=... Worst=... Sigma=...` lines from autoc run
logs and plots best-so-far across generations. Highlights the named focus
run; comparison runs are rendered thin and translucent. Vertical markers at
every 10% of `--total-gens` to make ramp-schedule comparisons easy.

Usage (default — produces the cadence7 PNG for 024):
    python3 specs/024-sim-real-fidelity/plot_fitness_ramp.py

Flags:
    --focus NAME:PATH      run to highlight (thick red line)
    --compare NAME:PATH    comparison run (may repeat)
    --out PATH             output PNG
    --total-gens N         X-axis extent + 10% marker spacing (default 400)
"""

import argparse
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt

GEN_RE = re.compile(r"Gen\s+(\d+)\s+Best=(-?\d+\.?\d*)")


def load_gens(path: Path):
    gens, best = [], []
    for line in path.read_text().splitlines():
        m = GEN_RE.search(line)
        if not m:
            continue
        gens.append(int(m.group(1)))
        best.append(float(m.group(2)))
    if not gens:
        raise SystemExit(f"no Gen lines found in {path}")
    return gens, best


def parse_pair(arg):
    if ":" not in arg:
        raise argparse.ArgumentTypeError(f"expected NAME:PATH, got {arg!r}")
    name, path = arg.split(":", 1)
    return name, Path(path)


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--focus",
        type=parse_pair,
        default=("cadence7", Path("logs/autoc-024-cadence7.log")),
        help="run to highlight (NAME:PATH)",
    )
    p.add_argument(
        "--compare",
        type=parse_pair,
        action="append",
        default=[],
        help="comparison run (NAME:PATH, may repeat)",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=Path("specs/024-sim-real-fidelity/autoc-024-cadence7-fitness.png"),
    )
    p.add_argument("--total-gens", type=int, default=400)
    p.add_argument(
        "--title",
        default="autoc-024: cadence7 (dt=0.005, fps=20, live) vs 023 hb1-adjust4/test7",
    )
    args = p.parse_args()

    if not args.compare:
        args.compare = [
            ("hb1-adjust4", Path("logs/autoc-023-hb1-adjust4.log")),
            ("test7", Path("logs/autoc-023-test7.log")),
        ]

    fig, ax = plt.subplots(figsize=(14, 7))

    # Comparison runs — thin, translucent
    colors = ["tab:blue", "tab:green", "tab:purple", "tab:orange"]
    for i, (name, path) in enumerate(args.compare):
        gens, best = load_gens(path)
        ax.plot(
            gens,
            best,
            linewidth=1.0,
            alpha=0.45,
            color=colors[i % len(colors)],
            label=f"{name} gen {gens[-1]}: {best[-1]:.0f}",
        )

    # Focus run — thick red
    name, path = args.focus
    gens, best = load_gens(path)
    status = "live" if gens[-1] < args.total_gens else "final"
    ax.plot(
        gens,
        best,
        linewidth=2.5,
        color="red",
        label=f"{name} ({status}) gen {gens[-1]}: {best[-1]:.0f}",
    )

    # 10% markers spanning the full target gen range (labels at top, 023 style)
    for pct in range(10, 101, 10):
        x = args.total_gens * pct / 100
        ax.axvline(x, color="red", linewidth=0.5, alpha=0.3)
        ax.text(
            x,
            0.97,
            f"{pct}%",
            color="red",
            alpha=0.7,
            ha="center",
            va="top",
            fontsize=9,
            transform=ax.get_xaxis_transform(),
        )

    ax.set_xlabel("Generation")
    ax.set_ylabel("Best Fitness")
    ax.set_title(args.title)
    ax.set_xlim(0, args.total_gens)
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.legend(loc="lower left", framealpha=0.9)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
