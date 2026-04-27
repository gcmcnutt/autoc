#!/usr/bin/env python3
"""028 6-panel evolution progress plot.

Extends 027's 5-panel layout (fitness | streak | stability | energy | sigma)
with a 6th telemetry panel:

  - Top half: whh_xh_ratio (signal 1) — W_hh / W_xh activation ratio.
  - Bottom half: w_xh0_cv, w_xh1_cv, w_hh_cv (signal 2) — population CV per
    weight block.

See specs/028-deeper-rnn/contracts/plot_panel_api.md for the full panel
contract; data-model.md §1.2 for the field schema.

Usage:
    python3 plot_evolution_progress.py
        --focus NAME:STC_PATH
        [--compare NAME:LOG_PATH ...]
        [--out PNG_PATH]
        [--total-gens N]
        [--title STR]

    Default focus: dalone:data.stc
    Default compares: cadence7, cadence7-redux
    Default out: specs/028-deeper-rnn/evolution_progress.png
"""

import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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
    r"(?:\s+whh_xh_ratio=(-?\d+\.?\d*|nan))?"
    r"(?:\s+w_xh0_cv=(-?\d+\.?\d*|nan))?"
    r"(?:\s+w_xh1_cv=(-?\d+\.?\d*|nan))?"
    r"(?:\s+w_hh_cv=(-?\d+\.?\d*|nan))?"
)

LOG_GEN_RE = re.compile(
    r"Gen\s+(\d+)\s+Best=(-?\d+\.?\d*)\s+Avg=(-?\d+\.?\d*)\s+Worst=(-?\d+\.?\d*)\s+Sigma=(\d+\.?\d*)"
)


def _parse_float_or_nan(s):
    if s is None:
        return float("nan")
    if s.lower() == "nan":
        return float("nan")
    return float(s)


def load_stc(path: Path):
    rows = dict(gens=[], best=[], avg=[], worst=[], sigma=[],
                streak=[], pct=[], stability=[], energy=[],
                whh_xh_ratio=[], w_xh0_cv=[], w_xh1_cv=[], w_hh_cv=[])
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
        rows["stability"].append(_parse_float_or_nan(m.group(8)) if m.group(8) is not None else 0.0)
        rows["energy"].append(_parse_float_or_nan(m.group(9)) if m.group(9) is not None else 0.0)
        rows["whh_xh_ratio"].append(_parse_float_or_nan(m.group(10)))
        rows["w_xh0_cv"].append(_parse_float_or_nan(m.group(11)))
        rows["w_xh1_cv"].append(_parse_float_or_nan(m.group(12)))
        rows["w_hh_cv"].append(_parse_float_or_nan(m.group(13)))
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


def parse_pair(s):
    if ":" not in s:
        raise argparse.ArgumentTypeError(f"expected NAME:PATH, got {s!r}")
    name, path = s.split(":", 1)
    return name, Path(path)


def _all_zeros_or_nan(xs):
    """True iff every entry is 0.0 or NaN — used to detect the no-recurrent sentinel."""
    return all((math.isnan(x) or x == 0.0) for x in xs)


def _all_nan(xs):
    return all(math.isnan(x) for x in xs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--focus", type=parse_pair, default=("dalone", Path("data.stc")),
                   help="focus run NAME:STC_PATH (default dalone:data.stc)")
    p.add_argument("--compare", type=parse_pair, action="append", default=[],
                   help="comparison run NAME:LOG_PATH (may repeat)")
    p.add_argument("--out", type=Path,
                   default=Path("specs/028-deeper-rnn/evolution_progress.png"))
    p.add_argument("--total-gens", type=int, default=400)
    p.add_argument("--title", default=None)
    args = p.parse_args()

    if not args.compare:
        args.compare = [
            ("cadence7", Path("logs/autoc-024-cadence7.log")),
            ("cadence7-redux", Path("logs/autoc-027-cadence7redux.log")),
        ]

    focus_name, focus_path = args.focus
    if not focus_path.is_file():
        raise SystemExit(f"focus stc not found: {focus_path}")
    f = load_stc(focus_path)
    if not f["gens"]:
        raise SystemExit(f"no #NNGen lines in {focus_path}")

    compares = [(name, load_log(pth)) for name, pth in args.compare if pth.is_file()]

    n = len(f["gens"])
    status = "live" if f["gens"][-1] < args.total_gens else "final"
    print(f"loaded {n} gens from {focus_path} ({status})")

    # Two-column layout:
    #   Left column  — run-shape: fitness | streak | stability | energy
    #   Right column — search/architecture: sigma | whh_xh_ratio | block CV
    # Two independent GridSpecs so each column can have its own row count
    # (4 left, 3 right) while filling the same vertical extent.
    fig = plt.figure(figsize=(20, 14))
    gs_left = fig.add_gridspec(
        4, 1, left=0.05, right=0.48, top=0.95, bottom=0.05,
        hspace=0.30, height_ratios=[3, 2, 2, 2])
    gs_right = fig.add_gridspec(
        3, 1, left=0.55, right=0.98, top=0.95, bottom=0.05,
        hspace=0.30, height_ratios=[3, 2, 2])
    ax_fit       = fig.add_subplot(gs_left[0])
    ax_streak    = fig.add_subplot(gs_left[1], sharex=ax_fit)
    ax_stability = fig.add_subplot(gs_left[2], sharex=ax_fit)
    ax_energy    = fig.add_subplot(gs_left[3], sharex=ax_fit)
    ax_sigma     = fig.add_subplot(gs_right[0])
    ax_ratio     = fig.add_subplot(gs_right[1], sharex=ax_sigma)
    ax_cv        = fig.add_subplot(gs_right[2], sharex=ax_sigma)

    compare_colors = ["tab:blue", "tab:green", "tab:purple", "tab:orange"]

    # --- Panel 1: fitness ---
    for i, (name, cd) in enumerate(compares):
        if cd["gens"]:
            ax_fit.plot(cd["gens"], cd["best"], linewidth=1.0, alpha=0.40,
                        color=compare_colors[i % len(compare_colors)],
                        label=f"{name} (final {cd['best'][-1]:.0f})")
    ax_fit.plot(f["gens"], f["worst"], color="tab:gray", linewidth=0.8, alpha=0.6, label=f"{focus_name} worst")
    ax_fit.plot(f["gens"], f["avg"], color="tab:blue", linewidth=1.0, alpha=0.85, label=f"{focus_name} avg")
    ax_fit.plot(f["gens"], f["best"], color="red", linewidth=2.2,
                label=f"{focus_name} best ({status}) gen {f['gens'][-1]}: {f['best'][-1]:.0f}")
    ax_fit.fill_between(f["gens"], f["best"], f["worst"], color="tab:gray", alpha=0.06)
    for pct in range(10, 101, 10):
        x = args.total_gens * pct / 100
        ax_fit.axvline(x, color="red", linewidth=0.4, alpha=0.25)
    title = args.title or f"{focus_name} — 028 evolution progress (6-panel telemetry)"
    ax_fit.set_ylabel("Fitness (lower = better)")
    ax_fit.set_title(title)
    ax_fit.set_xlim(0, args.total_gens)
    ax_fit.grid(True, linewidth=0.4, alpha=0.4)
    ax_fit.legend(loc="lower left", framealpha=0.9, fontsize=9)

    # --- Panel 2: streak ---
    ax_streak.plot(f["gens"], f["streak"], "tab:purple", linewidth=1.6,
                   label=f"avgMaxStreak (final {f['streak'][-1]:.1f})")
    ax_streak.set_ylabel("avgMaxStreak", color="tab:purple")
    ax_streak.tick_params(axis="y", labelcolor="tab:purple")
    ax_streak.grid(True, linewidth=0.4, alpha=0.4)
    axr = ax_streak.twinx()
    axr.plot(f["gens"], f["pct"], "tab:orange", linewidth=1.4, alpha=0.85,
             label=f"pctInStreak (final {f['pct'][-1]:.1f}%)")
    axr.set_ylabel("pctInStreak (%)", color="tab:orange")
    axr.tick_params(axis="y", labelcolor="tab:orange")
    l1, lab1 = ax_streak.get_legend_handles_labels()
    l2, lab2 = axr.get_legend_handles_labels()
    ax_streak.legend(l1 + l2, lab1 + lab2, loc="upper left", framealpha=0.9, fontsize=9)

    # --- Panel 3: stability ---
    ax_stability.plot(f["gens"], f["stability"], "tab:olive", linewidth=1.6,
                      label=f"stability (final {f['stability'][-1]:.1f})")
    ax_stability.axhline(y=0, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)
    ax_stability.set_ylabel("Σ stability_score")
    ax_stability.grid(True, linewidth=0.4, alpha=0.4)
    ax_stability.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 4: energy ---
    ax_energy.plot(f["gens"], f["energy"], "tab:cyan", linewidth=1.6,
                   label=f"energy (final {f['energy'][-1]:.1f})")
    ax_energy.axhline(y=0, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)
    ax_energy.set_ylabel("Σ energy_score")
    ax_energy.grid(True, linewidth=0.4, alpha=0.4)
    ax_energy.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 5: sigma ---
    for i, (name, cd) in enumerate(compares):
        if cd["gens"]:
            ax_sigma.plot(cd["gens"], cd["sigma"], linewidth=1.0, alpha=0.45,
                          color=compare_colors[i % len(compare_colors)],
                          label=f"{name} (final {cd['sigma'][-1]:.3f})")
    ax_sigma.plot(f["gens"], f["sigma"], "red", linewidth=1.8,
                  label=f"{focus_name} (final {f['sigma'][-1]:.3f})")
    ax_sigma.axhline(y=0.05, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                     label="sigma floor 0.05")
    ax_sigma.set_ylabel("bestSigma")
    ax_sigma.grid(True, linewidth=0.4, alpha=0.4)
    ax_sigma.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_sigma.set_ylim(bottom=0)
    ax_sigma.set_xlim(0, args.total_gens)

    # --- Panel 6 top: whh_xh_ratio (signal 1) ---
    if _all_zeros_or_nan(f["whh_xh_ratio"]):
        ax_ratio.text(0.5, 0.5, "no recurrent layer (whh_xh_ratio == 0)",
                      transform=ax_ratio.transAxes, ha="center", va="center",
                      fontsize=11, color="gray", style="italic")
        ax_ratio.set_yticks([])
    else:
        ax_ratio.plot(f["gens"], f["whh_xh_ratio"], "tab:red", linewidth=1.4,
                      label=f"whh_xh_ratio (final {f['whh_xh_ratio'][-1]:.3f})")
        # Threshold placeholder — calibrate from data-model.md §2.3.
        # Default to 0.10 ("block engaged" lower bound from the identity-W_hh test).
        ax_ratio.axhline(y=0.10, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                         label="threshold 0.10 (block-engaged calibration)")
        ax_ratio.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_ratio.set_ylabel("whh_xh_ratio")
    ax_ratio.grid(True, linewidth=0.4, alpha=0.4)
    ax_ratio.set_ylim(bottom=0)

    # --- Panel 6 bottom: w_*_cv (signal 2) ---
    ax_cv.plot(f["gens"], f["w_xh0_cv"], "tab:orange", linewidth=1.2,
               label=f"w_xh0_cv (final {f['w_xh0_cv'][-1]:.3f})")
    ax_cv.plot(f["gens"], f["w_xh1_cv"], "tab:blue", linewidth=1.2,
               label=f"w_xh1_cv (final {f['w_xh1_cv'][-1]:.3f})")
    if not _all_nan(f["w_hh_cv"]):
        ax_cv.plot(f["gens"], f["w_hh_cv"], "tab:red", linewidth=1.4,
                   label=f"w_hh_cv (final {f['w_hh_cv'][-1]:.3f})")
    ax_cv.set_xlabel("Generation")
    ax_energy.set_xlabel("Generation")  # left-column bottom panel
    ax_cv.set_ylabel("Block CV")
    ax_cv.grid(True, linewidth=0.4, alpha=0.4)
    ax_cv.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_cv.set_ylim(bottom=0)

    # tight_layout fights with twinx (panel 2) and the two manually-positioned
    # gridspecs — left/right/top/bottom on the gridspecs already do the layout.
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
