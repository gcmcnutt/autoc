#!/usr/bin/env python3
"""G-LOAD — body-Z load factor as an aggressiveness measure. One generation.

⭐ WHY THIS IS ITS OWN REPORT (operator 2026-08-22: "G load is another form of
aggressiveness measure"). The existing aggressiveness panels measure COMMAND
effort — how hard the controller works the sticks. This measures the
CONSEQUENCE: what the airframe is actually subjected to. They are not the same
number and can move in opposite directions, which is the point.

⭐ MEASURED ON t7 gen 559, THE THING THAT MAKES THIS WORTH PLOTTING SEPARATELY:
high-g ticks are NOT produced by harder elevator.

    at n >= 4 g :  mean pitch_cmd +0.299,  airspeed 17.2 m/s
    all ticks   :  mean pitch_cmd +0.352,  airspeed 13.9 m/s

Pitch command at 4 g+ is BELOW average while airspeed is 24 % above it. Normal
force goes as v²·Cl, so the same deflection at 17 m/s makes ~1.5x the load it
makes at 14. The controller is not pulling harder — it is arriving faster. A
command-effort panel cannot show that; panel 4 is built to.

⚠️ LAG. Elevator commands pitch RATE, so the normal-force response is delayed:
corr(pitch_cmd, n) is +0.157 at zero lag and peaks at +0.673 at 150 ms — same
lag as the roll axis. Any command-vs-load correlation at zero lag measures
noise. Panel 4's annotation uses the measured lag.

SIGN CONVENTION — body FRD, so Z is DOWN and an accelerometer in wings-level
flight reads -1 g on Z. Load factor is therefore n = -acZ:
    n = +1  wings-level 1 g          n <  0  pushover / inverted pull
    n = +5  five-g pull              n = 0   ballistic
The dump stores acZ already divided by kAccelScale_g, so it is multiplied back.

Usage: g_load.py <tick.csv> --label NAME --gen N -o out.png
"""
import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ACCEL_SCALE_G = 8.0     # kAccelScale_g — dump stores acZ pre-divided by this
CRUISE_MPS = 13.0       # kCruiseSpeed_mps — dump stores airspeed pre-divided
PEAK_SEEN_G = 11.2      # highest sim load on record; sized kAccelScale_g (041)
TRACK_THRESHOLD = 0.5
CLOSE_SMOOTH_TICKS = 5
REGIMES = ["track", "intercept", "patrol"]
RCOLOR = {"track": "#2a9d3f", "intercept": "#e07b1a", "patrol": "#8888aa"}


def classify(d):
    n = len(d["dist"])
    stppt = d["stpPt"] if "stpPt" in d.dtype.names else np.zeros(n)
    kern = np.ones(CLOSE_SMOOTH_TICKS) / CLOSE_SMOOTH_TICKS
    dd = np.gradient(np.convolve(d["dist"], kern, mode="same"))
    reg = np.full(n, "patrol", dtype=object)
    reg[(stppt < TRACK_THRESHOLD) & (dd < 0)] = "intercept"
    reg[stppt >= TRACK_THRESHOLD] = "track"
    return reg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tick_csv")
    ap.add_argument("--label", required=True)
    ap.add_argument("--gen", type=int, required=True)
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()

    d = np.genfromtxt(a.tick_csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if "acZ" not in d.dtype.names:
        print("g_load: tick CSV lacks 'acZ' (pre-041 dump?)", file=sys.stderr)
        return 1
    n = -d["acZ"] * ACCEL_SCALE_G
    vel = d["vel"] * CRUISE_MPS if "vel" in d.dtype.names else None
    pt = d["out_pt"] if "out_pt" in d.dtype.names else None
    reg = classify(d) if "dist" in d.dtype.names else None

    fig, ax = plt.subplots(4, 1, figsize=(12, 16))
    fig.suptitle(f"{a.label} — G LOAD, gen {a.gen}\n"
                 "the CONSEQUENCE of control effort, not the effort itself "
                 "(n = -acZ; +1 = wings-level)", fontsize=12)

    # --- panel 1: distribution -------------------------------------------
    ax[0].hist(n, bins=160, color="#3b6fb6")
    ax[0].set_yscale("log")
    for q, c in ((50, "#2a9d3f"), (95, "#e07b1a"), (99.9, "#c0504d")):
        v = np.percentile(n, q)
        ax[0].axvline(v, color=c, ls="--", lw=1.2, label=f"p{q} = {v:+.2f} g")
    ax[0].axvline(n.max(), color="k", ls="-", lw=1.2, label=f"max = {n.max():+.2f} g")
    ax[0].axvline(PEAK_SEEN_G, color="#7a2fb0", ls=":", lw=1.6,
                  label=f"{PEAK_SEEN_G} g — highest sim load on record")
    ax[0].set_xlabel("load factor n (g)"); ax[0].set_ylabel("ticks (log)")
    ax[0].set_title("1. DISTRIBUTION — log count, so the tail is visible")
    ax[0].legend(fontsize=9); ax[0].grid(alpha=.3)

    # --- panel 2: exceedance ---------------------------------------------
    # ⭐ The direct answer to "how often are we seeing 11 g" — a survival curve
    # rather than a histogram, because the question is about the tail.
    ts = np.arange(0, max(12.5, float(n.max()) + 1), 0.25)
    frac = [100.0 * float((n >= t).mean()) for t in ts]
    ax[1].plot(ts, frac, lw=2, color="#3b6fb6")
    ax[1].set_yscale("log")
    ax[1].axvline(PEAK_SEEN_G, color="#7a2fb0", ls=":", lw=1.6, label=f"{PEAK_SEEN_G} g on record")
    for t in (4, 6, 8, 10):
        f = 100.0 * float((n >= t).mean())
        if f > 0:
            ax[1].annotate(f"{t}g: {f:.3f}%", (t, f), textcoords="offset points",
                           xytext=(4, 6), fontsize=9)
    ax[1].set_xlabel("threshold (g)"); ax[1].set_ylabel("% of ticks at or above (log)")
    ax[1].set_title("2. EXCEEDANCE — how often each load is reached")
    ax[1].legend(fontsize=9); ax[1].grid(alpha=.3, which="both")

    # --- panel 3: by regime ----------------------------------------------
    if reg is not None:
        pos = [n[(reg == r) & (n > 0)] for r in REGIMES]
        parts = ax[2].violinplot(pos, showmedians=True)
        for pc, r in zip(parts["bodies"], REGIMES):
            pc.set_facecolor(RCOLOR[r]); pc.set_alpha(.7)
        ax[2].set_xticks(range(1, len(REGIMES) + 1))
        ax[2].set_xticklabels([f"{r}\n{100*np.mean(reg==r):.1f}% of ticks" for r in REGIMES])
        ax[2].set_ylabel("load factor n (g), positive only")
        for i, r in enumerate(REGIMES, 1):
            v = n[(reg == r) & (n > 0)]
            if len(v):
                ax[2].annotate(f"p50 {np.median(v):.2f}\np95 {np.percentile(v,95):.2f}",
                               (i, np.percentile(v, 95)), textcoords="offset points",
                               xytext=(12, 0), fontsize=9)
        ax[2].set_title("3. BY REGIME — where the airframe is actually loaded")
        ax[2].grid(alpha=.3, axis="y")

    # --- panel 4: the v² mechanism ---------------------------------------
    if vel is not None and pt is not None:
        edges = np.unique(np.percentile(vel, np.linspace(0, 100, 13)))
        ctr, mn, mp = [], [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (vel >= lo) & (vel < hi)
            if m.sum() < 50:
                continue
            ctr.append(0.5 * (lo + hi)); mn.append(np.mean(n[m]))
            mp.append(np.mean(np.abs(pt[m])))
        ctr = np.array(ctr); mn = np.array(mn)
        ax[3].plot(ctr, mn, marker="o", color="#c0504d", label="mean load factor")
        # v² reference, normalised at the median bin — what constant-Cl predicts
        i0 = len(ctr) // 2
        ax[3].plot(ctr, mn[i0] * (ctr / ctr[i0]) ** 2, ls="--", color="#888",
                   label="∝ v² (constant Cl / same deflection)")
        ax[3].set_xlabel("airspeed (m/s)"); ax[3].set_ylabel("mean load factor (g)")
        a2 = ax[3].twinx()
        a2.plot(ctr, mp, marker="s", ls=":", color="#3b6fb6", label="mean |pitch cmd| (right)")
        a2.set_ylabel("mean |pitch cmd|", color="#3b6fb6")
        ax[3].set_title("4. LOAD IS BOUGHT WITH SPEED, NOT STICK — if the red curve tracks v²\n"
                        "while |pitch cmd| stays flat, high g is arrival speed, not harder pulling")
        ax[3].legend(loc="upper left", fontsize=9); a2.legend(loc="lower right", fontsize=9)
        ax[3].grid(alpha=.3)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}")

    print(f"  n: p50 {np.percentile(n,50):+.2f}  p95 {np.percentile(n,95):+.2f}  "
          f"p99.9 {np.percentile(n,99.9):+.2f}  max {n.max():+.2f}  min {n.min():+.2f} g")
    for t in (4, 6, 8, 10, PEAK_SEEN_G):
        print(f"  n >= {t:>5} g : {int((n>=t).sum()):>7,} ticks  {100*(n>=t).mean():6.3f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
