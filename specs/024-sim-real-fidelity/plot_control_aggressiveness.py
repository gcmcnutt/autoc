#!/usr/bin/env python3
r"""Control aggressiveness evolution across generations — two-panel view.

For each (generation, path, wind) trajectory in a training data.dat, compute:

    dctrl_mean = mean over consecutive pairs of (|ΔoutPt| + |ΔoutRl| + |ΔoutTh|)
    mag_mean   = mean over ticks of (|outPt| + |outRl| + |outTh|)

Top panel:    dctrl_mean  "how fast the stick moves"    (bang-bang proxy)
Bottom panel: mag_mean    "how hard the stick is pushed" (amplitude proxy)

Reading the pair:
- HIGH dctrl + HIGH mag = full-throw bang-bang.
- HIGH dctrl + LOW  mag = fine-grained tracking (rapid corrections at small
  amplitude). Desirable for precise close-in tracking.
- LOW  dctrl + HIGH mag = slow, sustained deflection (banked turn, climb).
- LOW  dctrl + LOW  mag = quiet stick (cruise or dead controller).

The |Δout| metric alone can't distinguish the first two regimes — adding the
magnitude panel does.

Default: reads /tmp/cadence7_starter_paths.dat (prefiltered to paths 0-4,
wind 00 across all gens), plots 5 lines per panel (one per path), writes PNG
to specs/024-sim-real-fidelity/control_aggressiveness_cadence7.png.

Usage:
    # 1. Prefilter (5 starter paths, wind 00, all gens):
    awk 'NR==1 || /^[0-9]+ [0-9]+ 00[01234]\/00:/' data.dat > /tmp/cadence7_starter_paths.dat
    # 2. Plot:
    python3 plot_control_aggressiveness.py [input.dat] [output.png]
"""

import sys
from collections import defaultdict
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


POP_SIZE = 3500  # autoc.ini PopulationSize default — overridable via argv[5]


def parse_pth_wnd(field: str):
    # field like "002/00:0045:"
    pw, _step, _ = field.split(":")
    path, wind = pw.split("/")
    return int(path), int(wind)


def gen_from_scn(scn: int) -> int:
    # Scn = (POP_SIZE + 1) * gen (empirically: 3501 * gen for pop=3500).
    # Integer division works because all training rows follow the pattern.
    return scn // (POP_SIZE + 1)


def load(path: Path):
    """Stream rows, returning per_path[path][gen] = (dctrl_mean, mag_mean)."""
    per_path = defaultdict(lambda: defaultdict(list))
    header = None
    col_scn = col_pw = col_outPt = col_outRl = col_outTh = None
    current_key = None
    current_prev = None  # (pt, rl, th)
    dctrl_sum = 0.0
    mag_sum = 0.0
    steps = 0

    def flush():
        nonlocal current_key, dctrl_sum, mag_sum, steps, current_prev
        if current_key is not None and steps > 0:
            path_idx, gen, _scn = current_key
            per_path[path_idx][gen].append((dctrl_sum / steps, mag_sum / steps))
        current_key = None
        dctrl_sum = 0.0
        mag_sum = 0.0
        steps = 0
        current_prev = None

    with open(path) as f:
        for raw in f:
            parts = raw.split()
            if header is None:
                header = parts
                col_scn = header.index("Scn")
                col_pw = header.index("Pth/Wnd:Step:")
                col_outPt = header.index("outPt")
                col_outRl = header.index("outRl")
                col_outTh = header.index("outTh")
                continue
            if len(parts) < max(col_scn, col_pw, col_outPt, col_outRl, col_outTh) + 1:
                continue
            try:
                scn = int(parts[col_scn])
                path_idx, _wind = parse_pth_wnd(parts[col_pw])
                pt = float(parts[col_outPt])
                rl = float(parts[col_outRl])
                th = float(parts[col_outTh])
            except ValueError:
                continue
            gen = gen_from_scn(scn)
            key = (path_idx, gen, scn)
            mag_now = abs(pt) + abs(rl) + abs(th)
            if key != current_key:
                flush()
                current_key = key
                current_prev = (pt, rl, th)
                # first row of a trajectory: count its magnitude but no dctrl yet
                mag_sum += mag_now
                steps += 1
                continue
            ppt, prl, pth_ = current_prev
            dctrl_sum += abs(pt - ppt) + abs(rl - prl) + abs(th - pth_)
            mag_sum += mag_now
            steps += 1
            current_prev = (pt, rl, th)
    flush()

    out = {}
    for path_idx in sorted(per_path.keys()):
        gens = sorted(per_path[path_idx].keys())
        dctrl_vals = []
        mag_vals = []
        for g in gens:
            samples = per_path[path_idx][g]
            dctrl_vals.append(sum(d for d, _ in samples) / len(samples))
            mag_vals.append(sum(m for _, m in samples) / len(samples))
        out[path_idx] = (gens, dctrl_vals, mag_vals)
    return out


def main():
    inp = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/cadence7_starter_paths.dat")
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(
        "specs/024-sim-real-fidelity/control_aggressiveness_cadence7.png"
    )
    label = sys.argv[3] if len(sys.argv) > 3 else "cadence7"
    total_gens = int(sys.argv[4]) if len(sys.argv) > 4 else 400  # x-axis extent
    global POP_SIZE
    if len(sys.argv) > 5:
        POP_SIZE = int(sys.argv[5])  # gen = Scn / (POP_SIZE+1) — must match autoc.ini

    data = load(inp)
    print(f"Paths found: {sorted(data.keys())}")
    for p, (gens, dctrl, mag) in data.items():
        print(f"  path {p}: {len(gens)} gens, "
              f"mean dCtrl {sum(dctrl)/len(dctrl):.3f}, "
              f"mean |out| {sum(mag)/len(mag):.3f}")

    fig, (ax_dc, ax_mag) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    cmap = plt.get_cmap("tab10")
    window = 7

    def smooth(vals):
        n = len(vals)
        out = []
        for i in range(n):
            lo = max(0, i - window // 2)
            hi = min(n, i + window // 2 + 1)
            out.append(sum(vals[lo:hi]) / (hi - lo))
        return out

    for p, (gens, dctrl, mag) in data.items():
        color = cmap(p)
        ax_dc.plot(gens, dctrl, linewidth=0.7, alpha=0.30, color=color)
        ax_dc.plot(gens, smooth(dctrl), linewidth=1.8, alpha=0.95,
                   color=color, label=f"path {p}, wind 00")
        ax_mag.plot(gens, mag, linewidth=0.7, alpha=0.30, color=color)
        ax_mag.plot(gens, smooth(mag), linewidth=1.8, alpha=0.95, color=color)

    # VariationRampStep=40 boundaries
    for ax in (ax_dc, ax_mag):
        for g in range(40, total_gens + 1, 40):
            ax.axvline(g, color="red", linewidth=0.5, alpha=0.35)
    ax_dc.text(40, 0.05, "ramp@40", color="red", alpha=0.7, fontsize=9,
               ha="left", va="bottom", transform=ax_dc.get_xaxis_transform())

    # Top panel — change rate
    ax_dc.set_ylabel("Mean |Δout| per tick  (stick speed)")
    ax_dc.set_title(
        f"{label} control evolution — top: change rate  |  bottom: magnitude\n"
        "HIGH dctrl + LOW |out| = fine-grained tracking  (desirable close-in)\n"
        "HIGH dctrl + HIGH |out| = full-throw bang-bang"
    )
    ax_dc.grid(True, linewidth=0.5, alpha=0.4)
    ax_dc.legend(loc="upper left", framealpha=0.9)
    ax_dc.set_ylim(bottom=0)

    # Bottom panel — magnitude
    ax_mag.set_xlabel("Generation")
    ax_mag.set_ylabel("Mean |out| per tick  (stick amplitude)")
    ax_mag.grid(True, linewidth=0.5, alpha=0.4)
    ax_mag.set_xlim(0, total_gens)
    ax_mag.set_ylim(bottom=0, top=3.1)
    # dashed reference line: absolute ceiling (|outPt| + |outRl| + |outTh| ≤ 3)
    ax_mag.axhline(3.0, color="gray", linewidth=0.6, linestyle="--", alpha=0.5)
    ax_mag.text(total_gens - 5, 3.0, " ceiling = 3.0 (full throw all 3 axes)",
                color="gray", alpha=0.7, fontsize=8, ha="right", va="top")

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
