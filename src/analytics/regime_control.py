#!/usr/bin/env python3
"""Control effort by FLIGHT REGIME — physically grouped, frame-free. One generation.

⭐ THE QUESTION (operator 2026-08-21): "expose patrol, intercept, track as a
function of controls" — does the controller work where the work buys score?

⛔ WHY PER-AXIS / EULER ANALYSIS IS WRONG HERE. The first version of this report
split effort per NN output pin (pitch, roll, throttle) and reasoned about bank
angle. Both were mistakes, and the operator caught both:

  1. "throttle and pitch are DIRECT controls, they move the craft directly
     relative to the target. roll is *indirect* — a pure roll doesn't change
     tracking at all; roll needs pitch to result in a velocity vector change."

     So pitch and roll are NOT independent efforts to be compared side by side.
     They trade geometrically through the bank vector: the pair sets WHERE the
     lift vector points, and only the pair produces a trajectory change. This
     report therefore groups them — bank-pair magnitude sqrt(pt² + rl²) — and
     keeps THROTTLE separate, because throttle is genuinely decoupled from that
     geometry. Same grouping the project's own smoothness-lexicase note reached
     from the opposite direction.

  2. "bank angle — this is ALSO all attitude flying."

     Measured on 041-t7 gen 289: 24.1% of ticks sit in the 60-120° knife-edge
     zone and 6.0% are INVERTED past 120°. Euler bank angle assumes a reference
     "up"; a level-turn identity like load = 1/cos(bank) is meaningless at 128°.
     An earlier "pitch doesn't rise with bank, so the turn is uncoordinated"
     reading was an artifact of forcing a planar frame onto all-attitude flight.
     Everything attitude-related here is FRAME-FREE: body-rate magnitude
     |omega| = |(p,q,r)|, which needs no reference direction and is identical
     inverted, knife-edge or upright.

⚠️ AND CONTROL AUTHORITY IS LAGGED. corr(roll_cmd, gyroP) is NEGATIVE at zero
lag (-0.305) and peaks at +0.713 at 150 ms — servo plus inertia. Any
command-vs-response correlation computed at zero lag measures noise. Where this
report relates command to response it uses the measured lag.

⚠️ THE Σ IS LENGTH-CONFOUNDED. stability_score sums per tick (-2 ideal, 0
worst), so a longer-surviving run accumulates a "better" total with its surfaces
no closer to centre. Everything here is per-tick.

REGIMES — numerically identical to dynamics_progress.py so the reports agree:
    track      stpPt >= 0.5
    intercept  stpPt < 0.5 AND closing (5-tick smoothed)
    patrol     otherwise

Usage: regime_control.py <tick.csv> --label NAME --gen N -o out.png
"""
import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TRACK_THRESHOLD = 0.5
CLOSE_SMOOTH_TICKS = 5
BUDGET = 0.27                      # per-axis change-rate budget
BANK_BUDGET = BUDGET * np.sqrt(2)  # the pair, as a vector norm
GYRO_SCALE = 6.0                   # kGyroScale_radps — dump stores scaled rates
REGIMES = ["track", "intercept", "patrol"]
RCOLOR = {"track": "#2a9d3f", "intercept": "#e07b1a", "patrol": "#8888aa"}
ROLL_LAG_TICKS = 3                 # measured: corr peaks at 150 ms @ 20 Hz


def classify(d):
    n = len(d["dist"])
    stppt = d["stpPt"] if "stpPt" in d.dtype.names else np.zeros(n)
    kern = np.ones(CLOSE_SMOOTH_TICKS) / CLOSE_SMOOTH_TICKS
    dd = np.gradient(np.convolve(d["dist"], kern, mode="same"))
    reg = np.full(n, "patrol", dtype=object)
    reg[(stppt < TRACK_THRESHOLD) & (dd < 0)] = "intercept"
    reg[stppt >= TRACK_THRESHOLD] = "track"
    return reg


def within_delta(v, scen):
    """First difference inside each scenario; boundaries -> NaN, never a fake step."""
    o = np.full(len(v), np.nan)
    for s in np.unique(scen):
        m = np.where(scen == s)[0]
        if len(m) > 1:
            o[m[1:]] = np.diff(v[m])
    return o


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tick_csv")
    ap.add_argument("--label", required=True)
    ap.add_argument("--gen", type=int, required=True)
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()

    d = np.genfromtxt(a.tick_csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
    names = d.dtype.names
    for need in ("dist", "out_pt", "out_rl", "out_th", "gyrP", "gyrQ", "gyrR"):
        if need not in names:
            print(f"regime_control: tick CSV lacks '{need}'", file=sys.stderr)
            return 1
    scen = d["scenario"] if "scenario" in names else np.zeros(len(d["dist"]))
    reg = classify(d)
    occ = {r: 100 * float(np.mean(reg == r)) for r in REGIMES}

    # --- physical groups ---------------------------------------------------
    bank_mag = np.hypot(d["out_pt"], d["out_rl"])            # WHERE lift points
    dpt = within_delta(d["out_pt"], scen)
    drl = within_delta(d["out_rl"], scen)
    bank_rate = np.hypot(dpt, drl)                            # how fast it moves
    thr_mag = np.abs(d["out_th"])
    thr_rate = np.abs(within_delta(d["out_th"], scen))

    # frame-free airframe response — no reference "up" anywhere
    om = np.degrees(np.sqrt((d["gyrP"] * GYRO_SCALE) ** 2 +
                            (d["gyrQ"] * GYRO_SCALE) ** 2 +
                            (d["gyrR"] * GYRO_SCALE) ** 2))

    fig, ax = plt.subplots(4, 1, figsize=(12, 16))
    fig.suptitle(
        f"{a.label} — control effort by REGIME, gen {a.gen}\n"
        "bank-pair (pitch+roll, they trade geometrically) vs throttle (decoupled) · "
        "attitude measures are FRAME-FREE",
        fontsize=12)

    # --- panel 1: COMMAND effort, grouped ----------------------------------
    w = 0.25
    groups = [("bank |Δ(pt,rl)|", bank_rate, BANK_BUDGET),
              ("throttle |Δth|", thr_rate, BUDGET)]
    x = np.arange(len(groups))
    for i, r in enumerate(REGIMES):
        m = reg == r
        vals = [np.nanmean(g[m]) if np.isfinite(g[m]).any() else 0.0 for _, g, _ in groups]
        ax[0].bar(x + (i - 1) * w, vals, w, label=f"{r} ({occ[r]:.1f}%)", color=RCOLOR[r])
    for i, (_, _, bud) in enumerate(groups):
        ax[0].hlines(bud, i - 0.4, i + 0.4, ls="--", color="k", lw=1)
    ax[0].set_xticks(x); ax[0].set_xticklabels([n for n, _, _ in groups])
    ax[0].set_ylabel("mean |Δ| per tick")
    ax[0].set_title("1. COMMAND WORK, physically grouped  (dashed = budget; bank budget = 0.27·√2)")
    ax[0].legend(); ax[0].grid(alpha=.3, axis="y")

    # --- panel 2: AIRFRAME response, frame-free ----------------------------
    vals = [np.mean(om[reg == r]) if (reg == r).sum() else np.nan for r in REGIMES]
    b = ax[1].bar(REGIMES, vals, color=[RCOLOR[r] for r in REGIMES])
    ax[1].axhline(np.mean(om), ls="--", c="k", lw=1, label=f"pooled {np.mean(om):.0f}°/s")
    ax[1].set_ylabel("|ω| = |(p,q,r)|   (deg/s)")
    ax[1].set_title("2. WHAT THE AIRFRAME ACTUALLY DOES — frame-free, valid inverted/knife-edge.\n"
                    "Flat across regimes = rotating hard everywhere, not only where it scores")
    for r, v, rect in zip(REGIMES, vals, b):
        ax[1].text(rect.get_x() + rect.get_width() / 2, v, f"{v:.0f}°/s",
                   ha="center", va="bottom", fontsize=9)
    ax[1].legend(); ax[1].grid(alpha=.3, axis="y")

    # --- panel 3: continuous vs distance -----------------------------------
    dist = d["dist"]
    edges = np.unique(np.percentile(dist, np.linspace(0, 100, 13)))
    ctr, sb, st_, so = [], [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (dist >= lo) & (dist < hi)
        if m.sum() < 20:
            continue
        ctr.append(0.5 * (lo + hi))
        sb.append(np.nanmean(bank_rate[m])); st_.append(np.nanmean(thr_rate[m]))
        so.append(np.mean(om[m]))
    ax[2].plot(ctr, sb, marker="o", label="bank |Δ(pt,rl)|", color="#3b6fb6")
    ax[2].plot(ctr, st_, marker="o", label="throttle |Δth|", color="#2a9d3f")
    ax[2].axhline(BANK_BUDGET, ls="--", c="#3b6fb6", lw=1)
    ax[2].axhline(BUDGET, ls="--", c="#2a9d3f", lw=1)
    ax[2].set_xlabel("target distance (m)"); ax[2].set_ylabel("mean |Δ| per tick")
    ax[2].set_title("3. CONTINUOUS VIEW — regime is a 3-bucket quantisation of this")
    ax[2].legend(loc="upper left"); ax[2].grid(alpha=.3)
    a2 = ax[2].twinx(); a2.plot(ctr, so, marker="s", ls=":", color="#999", label="|ω| (right)")
    a2.set_ylabel("|ω| deg/s", color="#999"); a2.legend(loc="upper right")

    # --- panel 4: all-attitude context — WHY Euler was abandoned ----------
    if all(k in names for k in ("qw", "qx", "qy", "qz")):
        qw, qx, qy, qz = d["qw"], d["qx"], d["qy"], d["qz"]
        phi = np.abs(np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))))
        zones = [(0, 60, "conventional-ish", "#7fb37f"),
                 (60, 120, "knife-edge", "#e0b83a"),
                 (120, 180, "INVERTED", "#c0504d")]
        for lo, hi, lab, c in zones:
            k = (phi >= lo) & (phi < hi)
            ax[3].bar(f"{lo}-{hi}°\n{lab}", 100 * k.mean(), color=c)
            ax[3].text(f"{lo}-{hi}°\n{lab}", 100 * k.mean(), f"{100*k.mean():.1f}%",
                       ha="center", va="bottom", fontsize=10)
        ax[3].set_ylabel("% of ticks")
        ax[3].set_title("4. WHY THE EULER FRAME IS ABANDONED ABOVE — attitude distribution.\n"
                        "Anything assuming a reference 'up' (bank angle, 1/cos load) is invalid "
                        "for the non-green share")
        ax[3].grid(alpha=.3, axis="y")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
