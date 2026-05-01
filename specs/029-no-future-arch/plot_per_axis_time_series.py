#!/usr/bin/env python3
"""028 per-axis aggressiveness time-series — pt/rl/th evolution across gens.

Companion to specs/024-sim-real-fidelity/plot_control_aggressiveness.py.
That script aggregates pt+rl+th into a single dCtrl/<|out|> per path-gen.
This script breaks them out: 3 lines per panel (one per control axis), one
trace each for the population's mean over all included path/wind scenarios.

Why per-axis: bang-bang lives on a single axis (roll for the RNN
architecture, pitch for FF). The per-axis slope tells you which axis is
refining and which is stuck. From more-rnn3 mid-run: amplitude trends
down on all 3 axes slightly, but dCtrl plateaus on roll while throttle
*creeps up*. Aggregate hides both signals.

Usage:
    python3 plot_per_axis_time_series.py
        [--in data.dat] [--label NAME] [--total-gens N]
        [--pop-size N] [--out PNG]
"""

import argparse
from collections import defaultdict
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_pth_wnd(field: str):
    pw, _step, _ = field.split(":")
    path, wind = pw.split("/")
    return int(path), int(wind)


def load(path: Path, pop_size: int):
    """Stream rows; return per_gen[gen] = (dpt, drl, dth, mpt, mrl, mth) means.

    Streams to keep peak memory bounded; aggregates incrementally per
    (gen, path, wind, scenario) so we get within-trajectory deltas without
    holding the full row stream.
    """
    div = pop_size + 1
    sums = defaultdict(lambda: {"dpt": 0.0, "drl": 0.0, "dth": 0.0,
                                "mpt": 0.0, "mrl": 0.0, "mth": 0.0,
                                "n": 0})
    cols = {}
    header = None
    last_key = None
    last_pt = last_rl = last_th = None

    with open(path) as f:
        for raw in f:
            parts = raw.split()
            if header is None:
                header = parts
                cols["scn"] = header.index("Scn")
                cols["pw"] = header.index("Pth/Wnd:Step:")
                cols["pt"] = header.index("outPt")
                cols["rl"] = header.index("outRl")
                cols["th"] = header.index("outTh")
                continue
            if len(parts) <= max(cols.values()):
                continue
            try:
                scn = int(parts[cols["scn"]])
                gen = scn // div
                path_idx, _wind = parse_pth_wnd(parts[cols["pw"]])
                pt = float(parts[cols["pt"]])
                rl = float(parts[cols["rl"]])
                th = float(parts[cols["th"]])
            except (ValueError, IndexError):
                continue
            cur_key = (path_idx, scn)
            s = sums[gen]
            s["mpt"] += abs(pt); s["mrl"] += abs(rl); s["mth"] += abs(th)
            s["n"] += 1
            if last_key == cur_key and last_pt is not None:
                s["dpt"] += abs(pt - last_pt)
                s["drl"] += abs(rl - last_rl)
                s["dth"] += abs(th - last_th)
            last_key = cur_key
            last_pt, last_rl, last_th = pt, rl, th

    # Means per gen
    out = {}
    for gen, s in sums.items():
        n = s["n"]
        # dctrl uses (n-1) within trajectories — but with many trajectories per gen, n-1 ≈ n is fine
        if n < 2:
            continue
        out[gen] = (s["dpt"] / max(n - 1, 1), s["drl"] / max(n - 1, 1),
                    s["dth"] / max(n - 1, 1),
                    s["mpt"] / n, s["mrl"] / n, s["mth"] / n)
    return out


def smooth(vals, window=7):
    n = len(vals)
    out = []
    for i in range(n):
        lo = max(0, i - window // 2)
        hi = min(n, i + window // 2 + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, default=Path("data.dat"))
    ap.add_argument("--label", default="run")
    ap.add_argument("--total-gens", type=int, default=800)
    ap.add_argument("--pop-size", type=int, default=5000)
    ap.add_argument("--out", type=Path,
                    default=Path("specs/028-deeper-rnn/per_axis_time_series.png"))
    args = ap.parse_args()

    print(f"Loading {args.inp} (pop={args.pop_size})…")
    data = load(args.inp, args.pop_size)
    if not data:
        raise SystemExit(f"no rows in {args.inp}")
    gens = sorted(data.keys())
    print(f"  → gens {gens[0]}-{gens[-1]} ({len(gens)} total)")

    dpt = [data[g][0] for g in gens]
    drl = [data[g][1] for g in gens]
    dth = [data[g][2] for g in gens]
    mpt = [data[g][3] for g in gens]
    mrl = [data[g][4] for g in gens]
    mth = [data[g][5] for g in gens]

    # Late-run summary
    last50 = gens[-50:] if len(gens) >= 50 else gens
    idx_last = [gens.index(g) for g in last50]
    print(f"\nlate-run mean (last {len(last50)} gens):")
    for name, vals in [("pt", dpt), ("rl", drl), ("th", dth)]:
        m = sum(vals[i] for i in idx_last) / len(idx_last)
        print(f"  dCtrl_{name} = {m:.3f}")
    for name, vals in [("pt", mpt), ("rl", mrl), ("th", mth)]:
        m = sum(vals[i] for i in idx_last) / len(idx_last)
        print(f"  |out|_{name} = {m:.3f}")

    fig, (ax_d, ax_m) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    series = [
        ("pitch (pt)", "tab:red"),
        ("roll (rl)",  "tab:blue"),
        ("throttle (th)", "tab:green"),
    ]

    # Top: dCtrl per axis
    for (name, c), vals in zip(series, [dpt, drl, dth]):
        ax_d.plot(gens, vals, linewidth=0.7, alpha=0.30, color=c)
        ax_d.plot(gens, smooth(vals), linewidth=2.0, alpha=0.95, color=c, label=name)
    ax_d.axhline(0.27, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                 label="per-axis budget 0.27 (sum 0.80/3)")
    ax_d.set_ylabel("Mean |Δout| per tick  (stick speed)")
    ax_d.set_title(
        f"{args.label} — per-axis aggressiveness time series\n"
        "top: change rate (bang-bang detector)  |  bottom: amplitude (saturation detector)\n"
        "watch dCtrl slope — flat = bang-bang locked; trending down = smoothness emerging"
    )
    ax_d.grid(True, linewidth=0.5, alpha=0.4)
    ax_d.legend(loc="upper left", framealpha=0.9, fontsize=9)
    ax_d.set_ylim(bottom=0)

    # Bottom: amplitude per axis
    for (name, c), vals in zip(series, [mpt, mrl, mth]):
        ax_m.plot(gens, vals, linewidth=0.7, alpha=0.30, color=c)
        ax_m.plot(gens, smooth(vals), linewidth=2.0, alpha=0.95, color=c, label=name)
    ax_m.axhline(0.67, color="gray", linestyle="--", linewidth=0.7, alpha=0.6,
                 label="per-axis budget 0.67 (sum 2.00/3)")
    ax_m.axhline(1.0, color="black", linestyle=":", linewidth=0.6, alpha=0.5,
                 label="full-throw ceiling 1.0")
    ax_m.set_xlabel("Generation")
    ax_m.set_ylabel("Mean |out| per tick  (stick amplitude)")
    ax_m.grid(True, linewidth=0.5, alpha=0.4)
    ax_m.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_m.set_xlim(0, args.total_gens)
    ax_m.set_ylim(bottom=0, top=1.05)

    # 40-gen variation-ramp markers
    for ax in (ax_d, ax_m):
        for g in range(40, args.total_gens + 1, 40):
            ax.axvline(g, color="red", linewidth=0.4, alpha=0.25)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
