#!/usr/bin/env python3
"""030 #GenDiag multi-panel chart — operator-instrumentation per-gen diagnostics.

Parses #GenDiag lines emitted in data.stc by the polish-pass diagnostics
addition (2026-05-11). One row per gen with 18 metrics:

  loss_total, far, angle, over, fov, cone, hull,
  avgVis, avgInRamp,
  avgRngMin, avgRngMed, avgRngP95,
  avgFlips, maxClose, maxLost,
  avgSpiral, avgThrPt, avgThrRl

Plots them as a 6-panel chart by gen so trends are visible at a glance.

Usage:
    python3 plot_gen_diag.py [--in data.stc] [--total-gens N] [--label NAME]
                              [--out PNG]
"""

import argparse
import re
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


FIELD_RE = re.compile(r"(\w+)=([-\d.]+)")


def parse_gen_diag(path: Path):
    rows = []
    for line in path.read_text().splitlines():
        if not line.startswith("#GenDiag "):
            continue
        fields = dict(FIELD_RE.findall(line))
        if not fields or "gen" not in fields:
            continue
        rows.append({k: float(v) if "." in v else int(float(v)) for k, v in fields.items()})
    rows.sort(key=lambda r: r["gen"])
    return rows


def column(rows, name):
    return [r.get(name) for r in rows]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, default=Path("data.stc"))
    ap.add_argument("--label", default="run")
    ap.add_argument("--total-gens", type=int, default=200)
    ap.add_argument("--out", type=Path,
                    default=Path("specs/032-tracker-nn-enhancements/gen_diag.png"))
    ap.add_argument("--variation-ramp", type=int, default=40,
                    help="vertical-line marker every N gens (default 40)")
    args = ap.parse_args()

    rows = parse_gen_diag(args.inp)
    if not rows:
        print(f"no #GenDiag rows found in {args.inp}")
        return
    gens = column(rows, "gen")
    print(f"loaded {len(rows)} gens of #GenDiag from {args.inp}")
    print(f"  latest gen={gens[-1]}, loss_total={rows[-1].get('loss_total')}")

    fig = plt.figure(figsize=(14, 16))
    gs = fig.add_gridspec(6, 1, top=0.96, bottom=0.04, left=0.07, right=0.98, hspace=0.32)
    ax_loss = fig.add_subplot(gs[0])
    ax_vis = fig.add_subplot(gs[1], sharex=ax_loss)
    ax_rng = fig.add_subplot(gs[2], sharex=ax_loss)
    ax_motion = fig.add_subplot(gs[3], sharex=ax_loss)
    ax_extremes = fig.add_subplot(gs[4], sharex=ax_loss)
    ax_throttle = fig.add_subplot(gs[5], sharex=ax_loss)

    title = f"{args.label} — #GenDiag per-generation diagnostics (x = {args.total_gens} gens)"
    ax_loss.set_title(title)

    # Variation-ramp markers
    for x in range(args.variation_ramp, args.total_gens + 1, args.variation_ramp):
        for ax in (ax_loss, ax_vis, ax_rng, ax_motion, ax_extremes, ax_throttle):
            ax.axvline(x, color="red", linewidth=0.4, alpha=0.25)
        ax_loss.text(x, 0.97, str(x), color="red", alpha=0.5,
                     ha="center", va="top", fontsize=7,
                     transform=ax_loss.get_xaxis_transform())

    # Panel 1: loss decomposition (stacked-by-category) — geometric streak-break
    # taxonomy (fov/cone dead-code branches removed 2026-05-11 polish).
    far = column(rows, "far")
    angle = column(rows, "angle")
    over = column(rows, "over")
    hull = column(rows, "hull")
    loss_total = column(rows, "loss_total")
    ax_loss.fill_between(gens, 0, far, label="far", alpha=0.6, color="tab:red")
    ax_loss.fill_between(gens, far, [a + b for a, b in zip(far, angle)],
                          label="angle", alpha=0.6, color="tab:orange")
    base = [a + b for a, b in zip(far, angle)]
    if any(v > 0 for v in over):
        ax_loss.fill_between(gens, base, [b + o for b, o in zip(base, over)],
                              label="over", alpha=0.6, color="tab:olive")
        base = [b + o for b, o in zip(base, over)]
    if any(v > 0 for v in hull):
        ax_loss.fill_between(gens, base, [b + h for b, h in zip(base, hull)],
                              label="hull", alpha=0.6, color="tab:gray")
    ax_loss.plot(gens, loss_total, "k-", linewidth=1.5, label="loss_total")
    ax_loss.set_ylabel("Loss (counts)")
    ax_loss.legend(loc="upper right", framealpha=0.9, fontsize=9, ncols=4)
    ax_loss.grid(True, linewidth=0.4, alpha=0.4)
    ax_loss.set_xlim(0, args.total_gens)

    # Panel 2: visibility metrics
    ax_vis.plot(gens, column(rows, "avgVis"), "tab:blue", linewidth=1.6, label="avgVis (frac ticks visible)")
    ax_vis.plot(gens, column(rows, "avgInRamp"), "tab:green", linewidth=1.2, label="avgInRamp (frac ticks in streak)")
    ax_vis.set_ylabel("Fraction")
    ax_vis.set_ylim(0, 1)
    ax_vis.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_vis.grid(True, linewidth=0.4, alpha=0.4)

    # Panel 3: range metrics
    ax_rng.plot(gens, column(rows, "avgRngMin"), "tab:green", linewidth=1.4, label="avgRngMin")
    ax_rng.plot(gens, column(rows, "avgRngMed"), "tab:blue", linewidth=1.6, label="avgRngMed")
    ax_rng.plot(gens, column(rows, "avgRngP95"), "tab:red", linewidth=1.2, label="avgRngP95")
    ax_rng.set_ylabel("Range (m)")
    ax_rng.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_rng.grid(True, linewidth=0.4, alpha=0.4)

    # Panel 4: motion metrics
    ax_motion.plot(gens, column(rows, "avgFlips"), "tab:purple", linewidth=1.4, label="avgFlips (attitude flips/scn)")
    axr = ax_motion.twinx()
    axr.plot(gens, column(rows, "avgSpiral"), "tab:orange", linewidth=1.4,
             label="avgSpiral (rot/trans ratio)")
    ax_motion.set_ylabel("avgFlips (count)", color="tab:purple")
    ax_motion.tick_params(axis="y", labelcolor="tab:purple")
    axr.set_ylabel("avgSpiral (ratio)", color="tab:orange")
    axr.tick_params(axis="y", labelcolor="tab:orange")
    l1, lab1 = ax_motion.get_legend_handles_labels()
    l2, lab2 = axr.get_legend_handles_labels()
    ax_motion.legend(l1 + l2, lab1 + lab2, loc="upper right", framealpha=0.9, fontsize=9)
    ax_motion.grid(True, linewidth=0.4, alpha=0.4)

    # Panel 5: extremes
    ax_extremes.plot(gens, column(rows, "maxClose"), "tab:red", linewidth=1.4, label="maxClose (m, neg=approach)")
    axr2 = ax_extremes.twinx()
    axr2.plot(gens, column(rows, "maxLost"), "tab:olive", linewidth=1.4,
              label="maxLost (consec ticks lost-of-sight)")
    ax_extremes.set_ylabel("maxClose (m)", color="tab:red")
    ax_extremes.tick_params(axis="y", labelcolor="tab:red")
    axr2.set_ylabel("maxLost (ticks)", color="tab:olive")
    axr2.tick_params(axis="y", labelcolor="tab:olive")
    l1, lab1 = ax_extremes.get_legend_handles_labels()
    l2, lab2 = axr2.get_legend_handles_labels()
    ax_extremes.legend(l1 + l2, lab1 + lab2, loc="upper right", framealpha=0.9, fontsize=9)
    ax_extremes.grid(True, linewidth=0.4, alpha=0.4)

    # Panel 6: throttle axes
    ax_throttle.plot(gens, column(rows, "avgThrPt"), "tab:blue", linewidth=1.4, label="avgThrPt (pitch aggressiveness)")
    ax_throttle.plot(gens, column(rows, "avgThrRl"), "tab:red", linewidth=1.4, label="avgThrRl (roll aggressiveness)")
    ax_throttle.set_ylabel("Throttle metric")
    ax_throttle.set_xlabel(f"Generation (0..{args.total_gens})")
    ax_throttle.legend(loc="upper right", framealpha=0.9, fontsize=9)
    ax_throttle.grid(True, linewidth=0.4, alpha=0.4)

    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
