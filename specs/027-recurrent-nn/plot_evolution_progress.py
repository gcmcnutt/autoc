#!/usr/bin/env python3
"""Combined evolution-progress plot — fitness ramp + diagnostic panels in one.

Renders five stacked panels (027 v4 layout):

  1. Best / Avg / Worst fitness for the focus run, with best-fitness
     curves for comparison priors overlaid (translucent).
  2. Streak quality — avgMaxStreak + pctInStreak (focus only; priors
     don't log these).
  3. Stability — sum of per-scenario stability scores (Σ |Δ from center|;
     lower / more negative = surfaces near center; focus only).
  4. Energy — sum of per-scenario energy scores (Σ throttle integral;
     lower / more negative = less throttle used; focus only).
  5. bestSigma — focus + comparison priors.

Focus run data comes from data.stc (#NNGen lines with full fields).
Comparison runs are parsed from training logs (Gen N Best= Sigma=
text lines, sufficient for the two overlaid panels).

Usage:
    python3 plot_evolution_progress.py
        --focus NAME:STC_PATH
        [--compare NAME:LOG_PATH ...]
        [--out PNG_PATH]
        [--total-gens N]
        [--title STR]

    Default focus: rnn1:data.stc
    Default compares: cadence7, pid1 (025 + 026 anchors)
    Default out: specs/027-recurrent-nn/evolution_progress.png
"""

import argparse
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# #NNGen regex — full fields from data.stc.
# Field history: 027 v3 added `energy=`; 027 v4 added `stability=` before it.
# Older logs may have `smoothness=` (pre-v3) or no extra fields (pre-027).
# Stability and energy both optional (None if absent).
NNGEN_RE = re.compile(
    r"#NNGen\s+gen=(\d+)"
    r"\s+best=(-?\d+\.?\d*)"
    r"\s+avg=(-?\d+\.?\d*)"
    r"\s+worst=(-?\d+\.?\d*)"
    r"\s+bestSigma=(\d+\.?\d*)"
    r"\s+avgMaxStreak=(\d+\.?\d*)"
    r"\s+pctInStreak=(\d+\.?\d*)"
    r"(?:\s+stability=(-?\d+\.?\d*))?"
    r"(?:\s+(?:energy|smoothness)=(-?\d+\.?\d*))?"
)

# Log regex — cadence7 / pid1 / older training logs
LOG_GEN_RE = re.compile(
    r"Gen\s+(\d+)\s+Best=(-?\d+\.?\d*)\s+Avg=(-?\d+\.?\d*)\s+Worst=(-?\d+\.?\d*)\s+Sigma=(\d+\.?\d*)"
)


def load_stc(path: Path):
    rows = dict(gens=[], best=[], avg=[], worst=[],
                sigma=[], streak=[], pct=[], stability=[], energy=[])
    for line in path.read_text().splitlines():
        m = NNGEN_RE.search(line)
        if not m:
            continue
        rows["gens"].append(int(m.group(1)))
        rows["best"].append(float(m.group(2)))
        rows["avg"].append(float(m.group(3)))
        rows["worst"].append(float(m.group(4)))
        rows["sigma"].append(float(m.group(5)))
        rows["streak"].append(float(m.group(6)))
        rows["pct"].append(float(m.group(7)))
        rows["stability"].append(float(m.group(8)) if m.group(8) is not None else 0.0)
        rows["energy"].append(float(m.group(9)) if m.group(9) is not None else 0.0)
    return rows


def load_log(path: Path):
    rows = dict(gens=[], best=[], sigma=[])
    for line in path.read_text().splitlines():
        m = LOG_GEN_RE.search(line)
        if not m:
            continue
        rows["gens"].append(int(m.group(1)))
        rows["best"].append(float(m.group(2)))
        rows["sigma"].append(float(m.group(5)))
    return rows


def parse_pair(s: str):
    if ":" not in s:
        raise argparse.ArgumentTypeError(f"expected NAME:PATH, got {s!r}")
    name, path = s.split(":", 1)
    return name, Path(path)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--focus", type=parse_pair, default=("rnn1", Path("data.stc")),
                   help="focus run NAME:STC_PATH (default rnn1:data.stc)")
    p.add_argument("--compare", type=parse_pair, action="append", default=[],
                   help="comparison run NAME:LOG_PATH (may repeat)")
    p.add_argument("--out", type=Path,
                   default=Path("specs/027-recurrent-nn/evolution_progress.png"))
    p.add_argument("--total-gens", type=int, default=400,
                   help="x-axis extent + 10%% marker spacing")
    p.add_argument("--title", default=None,
                   help="optional override for the top-panel title")
    args = p.parse_args()

    if not args.compare:
        # Default comparisons: the two priors 027 is measured against
        args.compare = [
            ("cadence7", Path("logs/autoc-024-cadence7.log")),
            ("pid1",     Path("logs/autoc-026-pid1.log")),
        ]

    focus_name, focus_path = args.focus
    if not focus_path.is_file():
        raise SystemExit(f"focus stc not found: {focus_path}")
    f = load_stc(focus_path)
    if not f["gens"]:
        raise SystemExit(f"no #NNGen lines in {focus_path}")

    compares = []
    for name, pth in args.compare:
        if pth.is_file():
            compares.append((name, load_log(pth)))
        else:
            print(f"warn: compare log not found, skipping: {pth}", file=sys.stderr)

    n = len(f["gens"])
    status = "live" if f["gens"][-1] < args.total_gens else "final"
    print(f"loaded {n} gens from {focus_path} ({status})")
    print(f"  gens {f['gens'][0]}-{f['gens'][-1]}")
    print(f"  best: {f['best'][0]:.1f} -> {f['best'][-1]:.1f}")
    print(f"  stability: {f['stability'][0]:.1f} -> {f['stability'][-1]:.1f}")
    print(f"  energy: {f['energy'][0]:.1f} -> {f['energy'][-1]:.1f}")
    print(f"  pctInStreak: {f['pct'][0]:.1f}% -> {f['pct'][-1]:.1f}%")
    print(f"  sigma: {f['sigma'][0]:.3f} -> {f['sigma'][-1]:.3f}")

    fig, (ax_fit, ax_streak, ax_stability, ax_energy, ax_sigma) = plt.subplots(
        5, 1, figsize=(14, 16), sharex=True
    )

    # --- Panel 1: fitness (focus best/avg/worst + comparisons) -------------
    compare_colors = ["tab:blue", "tab:green", "tab:purple", "tab:orange"]
    for i, (name, cd) in enumerate(compares):
        if not cd["gens"]:
            continue
        ax_fit.plot(cd["gens"], cd["best"], linewidth=1.0, alpha=0.40,
                    color=compare_colors[i % len(compare_colors)],
                    label=f"{name} (final {cd['best'][-1]:.0f})")
    ax_fit.plot(f["gens"], f["worst"], color="tab:gray", linewidth=0.8, alpha=0.6,
                label=f"{focus_name} worst")
    ax_fit.plot(f["gens"], f["avg"], color="tab:blue", linewidth=1.0, alpha=0.85,
                label=f"{focus_name} avg")
    ax_fit.plot(f["gens"], f["best"], color="red", linewidth=2.2,
                label=f"{focus_name} best ({status}) gen {f['gens'][-1]}: {f['best'][-1]:.0f}")
    ax_fit.fill_between(f["gens"], f["best"], f["worst"],
                        color="tab:gray", alpha=0.06)

    # 10% vertical markers (project convention — matches plot_fitness_ramp)
    for pct in range(10, 101, 10):
        x = args.total_gens * pct / 100
        ax_fit.axvline(x, color="red", linewidth=0.4, alpha=0.25)
        ax_fit.text(x, 0.97, f"{pct}%", color="red", alpha=0.6,
                    ha="center", va="top", fontsize=8,
                    transform=ax_fit.get_xaxis_transform())
    title = args.title or (
        f"{focus_name} — evolution progress vs priors\n"
        "fitness | streak quality | stability (027 v4) | energy | bestSigma"
    )
    ax_fit.set_ylabel("Fitness (lower = better)")
    ax_fit.set_title(title)
    ax_fit.set_xlim(0, args.total_gens)
    ax_fit.grid(True, linewidth=0.4, alpha=0.4)
    ax_fit.legend(loc="lower left", framealpha=0.9, fontsize=9)

    # --- Panel 2: streak (focus only) --------------------------------------
    ax_streak.plot(f["gens"], f["streak"], "tab:purple", linewidth=1.6,
                   label=f"avgMaxStreak (final {f['streak'][-1]:.1f})")
    ax_streak.set_ylabel("avgMaxStreak (ticks)", color="tab:purple")
    ax_streak.tick_params(axis="y", labelcolor="tab:purple")
    ax_streak.grid(True, linewidth=0.4, alpha=0.4)
    ax_streak_r = ax_streak.twinx()
    ax_streak_r.plot(f["gens"], f["pct"], "tab:orange", linewidth=1.4, alpha=0.85,
                     label=f"pctInStreak (final {f['pct'][-1]:.1f}%)")
    ax_streak_r.set_ylabel("pctInStreak (%)", color="tab:orange")
    ax_streak_r.tick_params(axis="y", labelcolor="tab:orange")
    l1, lab1 = ax_streak.get_legend_handles_labels()
    l2, lab2 = ax_streak_r.get_legend_handles_labels()
    ax_streak.legend(l1 + l2, lab1 + lab2, loc="upper left", framealpha=0.9, fontsize=9)

    # --- Panel 3: stability (focus only, 027 v4) ---------------------------
    ax_stability.plot(f["gens"], f["stability"], "tab:olive", linewidth=1.6,
                      label=f"{focus_name} stability (final {f['stability'][-1]:.1f})")
    ax_stability.axhline(y=0, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                         label="0 (both surfaces saturated ceiling)")
    ax_stability.set_ylabel("Σ (|out_pt|-1)+(|out_rl|-1) (lower = surfaces near center)")
    ax_stability.grid(True, linewidth=0.4, alpha=0.4)
    ax_stability.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 4: energy (focus only, 027 v3) ------------------------------
    ax_energy.plot(f["gens"], f["energy"], "tab:cyan", linewidth=1.6,
                   label=f"{focus_name} energy (final {f['energy'][-1]:.1f})")
    ax_energy.axhline(y=0, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                      label="0 (max throttle ceiling)")
    ax_energy.set_ylabel("Σ (out_th-1)/2  (lower = less throttle)")
    ax_energy.grid(True, linewidth=0.4, alpha=0.4)
    ax_energy.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 4: bestSigma (focus + comparisons) --------------------------
    for i, (name, cd) in enumerate(compares):
        if not cd["gens"]:
            continue
        ax_sigma.plot(cd["gens"], cd["sigma"], linewidth=1.0, alpha=0.45,
                      color=compare_colors[i % len(compare_colors)],
                      label=f"{name} (final {cd['sigma'][-1]:.3f})")
    ax_sigma.plot(f["gens"], f["sigma"], "red", linewidth=1.8,
                  label=f"{focus_name} (final {f['sigma'][-1]:.3f})")
    ax_sigma.axhline(y=0.05, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                     label="sigma floor 0.05")
    ax_sigma.set_xlabel("Generation")
    ax_sigma.set_ylabel("Best-individual sigma")
    ax_sigma.grid(True, linewidth=0.4, alpha=0.4)
    ax_sigma.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_sigma.set_ylim(bottom=0)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
