#!/usr/bin/env python3
"""037 M1 evolution progress plot (20 Hz era).

Copied from specs/034-energy-objective-cleanup/plot_evolution_progress.py
(left untouched per feedback_historical_scripts_immutable). ONE change: the
crash panel parses the per-gen `#GenCrash` summary line instead of the
per-scenario `[N] CRASH/OK` lines, which 037 T005 removed from the training
log. Crash count = hullStrike + eval + sim + boot (the isCrash categories);
timeLimit / rabbitComplete / none are clean terminations.

Original docstring follows.

033 phase-1 M1 evolution progress plot.

Adapted from `specs/032-tracker-nn-enhancements/plot_evolution_progress.py`
(which evolved from 029's plot). Same `#NNGen gen=N best=... streak=... ...`
line format and panel structure. As of 034 T052 the `#NNGen` lines (and the
other gen markers) live in the run **.log** — the data.stc breadcrumb file
was retired — so the focus input is now a `.log`. The regex uses `re.search`,
so the logger's `<timestamp>: <info>` prefix is ignored. Historical committed
`.stc` snapshots still parse identically (same `#NNGen` token) and remain
valid `--compare` inputs.

Use this to compare the 033 M1 smoothness-on run head-to-head against
the 029 pastonly3 baseline (M1 smoothness-off, same 6-path × 49-wind
scenario count, same NN topology). Streak% drop vs 029 baseline is the
phase-1 closeout gate per spec §2.B Clarifications Q1 (≥2/3 of 029
baseline = continue; below = iterate on floor/mode).

Panels (left column):
  1. Fitness — best / avg / worst across gens. Variation-step markers every
     40 gens (matches `VariationRampStep` in autoc.ini).
  2. Streak — avgMaxStreak (left axis) + pctInStreak (right axis). Watch
     the 033 line vs the 029 baseline — significant drop here indicates
     the smoothness penalty is over-discounting and the controller can't
     compound streaks (iterate gate per spec).
  3. Stability — Σ stability_score per gen.
  4. Energy — Σ energy_score per gen.
  5. (optional) Crash rate — when --crash-log is given, parse per-scenario
     CRASH/OK lines from the training .log and plot crash % per gen on a
     symlog axis.

Panels (right column):
  6. Sigma — bestSigma evolution across gens vs the NN sigma floor.
  7. whh_xh_ratio — recurrent W_hh / W_xh activation ratio (signal 1 from 028).
  8. Block CV — w_xh0 / w_xh1 / w_hh population CV (signal 2 from 028).

Usage:
    python3 plot_evolution_progress.py
        --focus 034-craft:logs/autoc-034-craft-confirm.log
        --compare 029-pastonly3:specs/029-no-future-arch/pastonly3.stc
        [--out specs/034-energy-objective-cleanup/evolution_progress.png]
        [--total-gens 800]
        [--title STR]
        [--crash-log logs/autoc-034-craft-confirm.log]

    Default focus: newest logs/autoc-*.log (the live run's log)
    Default total-gens: 800 (matches autoc.ini production scale)
"""

import argparse
import math
import re
import sys
from pathlib import Path

from cadence import CadenceUnknown, tick_sec_from_log   # 041 T009

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

# Modern log gen-boundary marker (post-028): `NN_ELITE_SAME: gen=N fitness=…`
# or `NN_ELITE_DIVERGED: gen=N …`. Legacy logs used `Gen N Best=…` (above).
# Used by load_crashes() as a per-gen boundary marker in the .log.
NN_ELITE_RE = re.compile(r"NN_ELITE_(?:SAME|DIVERGED):\s+gen=(\d+)")

# Variant capturing fitness too — REQUIRED for --compare against historical
# runs (e.g. 029 pastonly3) whose .log predates #NNGen-in-log and carries only
# the NN_ELITE elite-tick line. Sigma is absent there (NaN), best-fitness still
# plottable. (Restored 2026-06-02: T053 removed this, but cross-run comparison
# to pre-T052 logs needs it; the FOCUS loader still prefers #NNGen.)
NN_ELITE_BEST_RE = re.compile(
    r"NN_ELITE_(?:SAME|DIVERGED):\s+gen=(\d+)\s+fitness=(-?\d+\.?\d*)")

# 037 T005 — per-scenario [N] CRASH/OK lines are gone from the training log;
# the per-gen #GenCrash summary is the crash source now:
#   #GenCrash gen=N hullStrike=A eval=B [egFloor=.. egCeil=.. egRadius=..]
#             sim=C boot=D timeLimit=E rabbitComplete=F none=G total=T
#
# ⚠️ Parsed BY FIELD NAME, not positionally. The previous regex pinned the
# field ORDER, so 041 P2-4's egress-kind split (egFloor/egCeil/egRadius,
# inserted after eval=) made every line stop matching — and the failure was not
# "no crash data", it was an IndexError deep in the plot, several frames from
# the cause. A key=value line should be read as key=value.
GENCRASH_LINE = re.compile(r"#GenCrash\s+(.*)$")
GENCRASH_KV = re.compile(r"(\w+)=(-?\d+)")


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
    """Load comparator best-fitness trajectory. Auto-detects format in order:
      1. `#NNGen gen=N best=...` (post-028 .stc / log) — has sigma.
      2. `Gen N Best=...` (pre-028 worker logs) — has sigma.
      3. `NN_ELITE_(SAME|DIVERGED): gen=N fitness=X` (current autoc.log) —
         sigma absent (NaN), best-fitness still plottable.
    Tries each in order, stops at the first one with hits."""
    rows = dict(gens=[], best=[], sigma=[])
    text = path.read_text()
    for line in text.splitlines():
        m = NNGEN_RE.search(line)
        if not m:
            continue
        rows["gens"].append(int(m.group(1)))
        rows["best"].append(float(m.group(2)))
        rows["sigma"].append(float(m.group(5)))
    if rows["gens"]:
        return rows
    for line in text.splitlines():
        m = LOG_GEN_RE.search(line)
        if not m:
            continue
        rows["gens"].append(int(m.group(1)))
        rows["best"].append(float(m.group(2)))
        rows["sigma"].append(float(m.group(5)))
    if rows["gens"]:
        return rows
    for line in text.splitlines():
        m = NN_ELITE_BEST_RE.search(line)
        if not m:
            continue
        rows["gens"].append(int(m.group(1)))
        rows["best"].append(float(m.group(2)))
        rows["sigma"].append(float("nan"))
    return rows


def load_crashes(path: Path):
    """Parse per-gen crash counts from the #GenCrash summary lines (037 T005)."""
    rows = dict(gens=[], crashes=[], total=[], rate=[])
    for line in path.read_text().splitlines():
        m = GENCRASH_LINE.search(line)
        if not m:
            continue
        kv = {k: int(v) for k, v in GENCRASH_KV.findall(m.group(1))}
        # Required keys only. Anything else on the line (the 041 egress-kind
        # split, or whatever is added next) is ignored rather than fatal.
        if not {"gen", "total"} <= kv.keys():
            continue
        gen, total = kv["gen"], kv["total"]
        if total <= 0:
            continue
        # isCrash categories; absent key == 0 so an older log still parses.
        crashes = sum(kv.get(k, 0) for k in ("hullStrike", "eval", "sim", "boot"))
        rows["gens"].append(gen)
        rows["crashes"].append(crashes)
        rows["total"].append(total)
        rows["rate"].append(100.0 * crashes / total)
    return rows


def parse_pair(s):
    if ":" not in s:
        raise argparse.ArgumentTypeError(f"expected NAME:PATH, got {s!r}")
    name, path = s.split(":", 1)
    return name, Path(path)


def _all_zeros_or_nan(xs):
    return all((math.isnan(x) or x == 0.0) for x in xs)


def _all_nan(xs):
    return all(math.isnan(x) for x in xs)


def _newest_run_log():
    """Newest logs/autoc-*.log — the live run's log (post-034-T052 the #NNGen
    markers live there, not in data.stc)."""
    logs = sorted(Path("logs").glob("autoc-*.log"), key=lambda p: p.stat().st_mtime)
    return logs[-1] if logs else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--focus", type=parse_pair, default=None,
                   help="focus run NAME:LOG_PATH — the run .log carrying #NNGen "
                        "lines (default: newest logs/autoc-*.log). A historical "
                        ".stc snapshot also works (same #NNGen token).")
    p.add_argument("--compare", type=parse_pair, action="append", default=[],
                   help="comparison run NAME:PATH (may repeat) — a run .log or a "
                        "committed .stc snapshot. Pass "
                        "033-pop8000-wind36-r1:specs/033-m1-smooth-plus-variations/pop8000-wind36-r1-data.stc "
                        "for the smoothness-stub baseline at the same pop/wind scale.")
    p.add_argument("--run-name", default="034-phase1",
                   help="Filename prefix for the output PNG.")
    p.add_argument("--out", type=Path, default=None,
                   help="Output PNG path. Defaults to "
                        "specs/034-energy-objective-cleanup/<run-name>_evolution_progress.png.")
    p.add_argument("--total-gens", type=int, default=800,
                   help="x-axis extent + 40-gen variation-step markers (default 800 — "
                        "matches autoc.ini production scale)")
    p.add_argument("--tick-sec", type=float, default=None,
                   help="control-loop cadence (sec/tick) — avgMaxStreak is tick-denominated, so "
                        "this converts it to SECONDS for cross-cadence comparison. Normally "
                        "OMITTED: 041 T009 reads the cadence from the focus log's own "
                        "ControlIntervalMsec, and a value disagreeing with it is an error. "
                        "Supply it only for pre-config-print logs (0.10 = 10 Hz).")
    p.add_argument("--title", default=None)
    p.add_argument("--crash-log", type=Path, default=None,
                   help="path to focus run's training log file (.log); when set, "
                        "the evolution plot adds a crash-rate panel parsed from "
                        "the per-scenario CRASH/OK lines (tracker-mode includes "
                        "HullStrike on top of legacy Eval/TimeLimit/RabbitComplete)")
    args = p.parse_args()

    # Resolve --out from --run-name when --out not explicitly given.
    if args.out is None:
        args.out = Path(f"specs/034-energy-objective-cleanup/{args.run_name}_evolution_progress.png")

    if args.focus is None:
        newest = _newest_run_log()
        if newest is None:
            raise SystemExit("no --focus given and no logs/autoc-*.log found")
        focus_name, focus_path = newest.stem, newest
    else:
        focus_name, focus_path = args.focus
    if not focus_path.is_file():
        raise SystemExit(f"focus log not found: {focus_path}")

    # 041 T009 — the focus RUN states its own cadence; trust that over any flag.
    # generate_pngs.sh derives --tick-sec from the CURRENT ini, which is the
    # wrong source when re-plotting an older run: it would rescale every
    # streak-second by the ratio of the two cadences and say nothing.
    try:
        args.tick_sec = tick_sec_from_log(focus_path, args.tick_sec, label=focus_name)
    except CadenceUnknown as e:
        raise SystemExit(str(e))
    f = load_stc(focus_path)
    if not f["gens"]:
        raise SystemExit(f"no #NNGen lines in {focus_path}")

    compares = [(name, load_log(pth)) for name, pth in args.compare if pth.is_file()]

    crash_data = None
    if args.crash_log is not None and args.crash_log.is_file():
        crash_data = load_crashes(args.crash_log)
        print(f"crash log: {len(crash_data['gens'])} gens, "
              f"final crash rate {crash_data['rate'][-1]:.1f}% "
              f"({crash_data['crashes'][-1]}/{crash_data['total'][-1]})")

    n = len(f["gens"])
    status = "live" if f["gens"][-1] < args.total_gens else "final"
    print(f"loaded {n} gens from {focus_path} ({status})")

    fig = plt.figure(figsize=(20, 14))
    if crash_data is not None:
        left_rows = 5
        left_heights = [3, 2, 2, 2, 1.2]
    else:
        left_rows = 4
        left_heights = [3, 2, 2, 2]
    gs_left = fig.add_gridspec(
        left_rows, 1, left=0.05, right=0.48, top=0.95, bottom=0.05,
        hspace=0.30, height_ratios=left_heights)
    gs_right = fig.add_gridspec(
        3, 1, left=0.55, right=0.98, top=0.95, bottom=0.05,
        hspace=0.30, height_ratios=[3, 2, 2])
    ax_fit       = fig.add_subplot(gs_left[0])
    ax_streak    = fig.add_subplot(gs_left[1], sharex=ax_fit)
    ax_stability = fig.add_subplot(gs_left[2], sharex=ax_fit)
    ax_energy    = fig.add_subplot(gs_left[3], sharex=ax_fit)
    ax_crash     = fig.add_subplot(gs_left[4], sharex=ax_fit) if crash_data else None
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
    VARIATION_STEP = 40
    for x in range(VARIATION_STEP, args.total_gens + 1, VARIATION_STEP):
        ax_fit.axvline(x, color="red", linewidth=0.4, alpha=0.25)
        ax_fit.text(x, 0.97, str(x), color="red", alpha=0.5,
                    ha="center", va="top", fontsize=7,
                    transform=ax_fit.get_xaxis_transform())
    # ⚠️ Was hardcoded "034 energy-objective-cleanup" — the feature this script was
    # COPIED FROM, not the feature it is plotting. Every evolution chart from 035
    # through 041 carried it, including the 041-t7 run that took the all-time M1
    # record. A title naming a fixed feature can only ever be right once; derive
    # it from the run instead so it cannot go stale again.
    title = args.title or f"{focus_name} — evolution progress"
    ax_fit.set_ylabel("Fitness (lower = better)")
    ax_fit.set_title(title)
    ax_fit.set_xlim(0, args.total_gens)
    ax_fit.grid(True, linewidth=0.4, alpha=0.4)
    ax_fit.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 2: streak ---
    # avgMaxStreak is tick-denominated → reads 2× at 20 Hz vs 10 Hz. Convert to
    # SECONDS (× tick_sec) so cross-cadence runs are comparable (038 P0-E / T004).
    streak_sec = [s * args.tick_sec for s in f["streak"]]
    ax_streak.plot(f["gens"], streak_sec, "tab:purple", linewidth=1.6,
                   label=f"avgMaxStreak (final {streak_sec[-1]:.2f}s)")
    ax_streak.set_ylabel(f"avgMaxStreak (s, {1.0/args.tick_sec:.0f} Hz)", color="tab:purple")
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

    # --- Panel 5 (left col, optional): crash rate from .log file ---
    if ax_crash is not None and crash_data is not None and crash_data["gens"]:
        ax_crash.plot(crash_data["gens"], crash_data["rate"], color="tab:red",
                      linewidth=1.4,
                      label=f"crash rate (final {crash_data['rate'][-1]:.1f}% — "
                            f"{crash_data['crashes'][-1]}/{crash_data['total'][-1]})")
        ax_crash.set_yscale("symlog", linthresh=0.41)
        ax_crash.set_ylim(bottom=0, top=120)
        ax_crash.axhline(0, color="gray", linestyle=":", linewidth=0.6, alpha=0.5)
        from matplotlib.ticker import FixedLocator, FuncFormatter
        ax_crash.yaxis.set_major_locator(FixedLocator([0, 0.5, 1, 5, 10, 50, 100]))
        ax_crash.yaxis.set_major_formatter(FuncFormatter(lambda v, _p: f"{v:g}%"))
        ax_crash.set_ylabel("Crash % (symlog)")
        ax_crash.grid(True, linewidth=0.4, alpha=0.4, which="both")
        ax_crash.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # --- Panel 5 (right col): sigma ---
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
    (ax_crash or ax_energy).set_xlabel("Generation")
    ax_cv.set_ylabel("Block CV")
    ax_cv.grid(True, linewidth=0.4, alpha=0.4)
    ax_cv.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_cv.set_ylim(bottom=0)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
