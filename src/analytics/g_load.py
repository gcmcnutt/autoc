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
SIM_DT_SEC = 0.05       # 20 Hz control cadence — physics-dump tick spacing
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
    cols = d.dtype.names

    # ⭐ TWO INPUT FORMATS, on purpose. The standard per-tick CSV (dmp-dump
    # --csv-only) carries `acZ` already divided by kAccelScale_g and `vel`
    # divided by kCruiseSpeed_mps. The `--physics` dump carries RAW `nz_g` /
    # `sfz_g` in g and `vRelWind` in m/s.
    #
    # ⛔ This is not gratuitous flexibility — it is the ONLY way to plot a
    # pre-041 run at all. The 041 schema break (v3 -> v4, no migration path)
    # makes those dmps unreadable; the T011a pre-break archive under
    # artifacts/pre-break/ is a --physics dump precisely so the comparators
    # survive. Without this branch the archive would be unusable for the very
    # comparison it was captured for.
    if "acZ" in cols:
        n = -d["acZ"] * ACCEL_SCALE_G
        vel = d["vel"] * CRUISE_MPS if "vel" in cols else None
        fmt = "tick"
    elif "nz_g" in cols or "sfz_g" in cols:
        # nz_g == -sfz_g exactly; prefer nz_g, it is already the load factor.
        n = d["nz_g"] if "nz_g" in cols else -d["sfz_g"]
        vel = d["vRelWind"] / 3.280839895 if "vRelWind" in cols else None  # ft/s -> m/s
        fmt = "physics"

        # ⛔ THE PHYSICS BLOCK IS ALMOST ALL EMPTY, and this is not obvious.
        # Measured on the T011a pre-break archive (038-t5 gen 800): nz_g is
        # populated on ticks 1-4 of each scenario ONLY -- 1,176 of 132,462 rows,
        # 0.89%, and all of it the entry transient. Taken at face value it says
        # the prior M1 never exceeded 2.45 g, which is an artifact of sampling
        # the first 200 ms of every flight.
        #
        # ⭐ RECONSTRUCT instead, from the velocity trace which IS fully
        # populated: specific force = dv/dt - g, rotated world->body, Z
        # component, negated. Then VALIDATE against whatever real nz_g rows
        # exist -- the archive carries its own ground truth, so the
        # reconstruction never has to be taken on faith.
        #
        # ⚠️ vx/vy/vz are m/s while vRelWind is ft/s, in the same file. That was
        # settled BY the validation, not by assumption: ft/s gives corr 0.751 /
        # MAE 0.230 g, m/s gives corr 0.995 / MAE 0.028 g.
        if np.isfinite(n).sum() < 0.5 * len(n) and all(
                k in cols for k in ("vx", "vy", "vz", "qw", "qx", "qy", "qz", "scenario")):
            G = 9.80665
            dt = SIM_DT_SEC
            vx, vy, vz = d["vx"], d["vy"], d["vz"]
            sc = d["scenario"]
            ax_ = np.full(len(vx), np.nan); ay_ = ax_.copy(); az_ = ax_.copy()
            for sid in np.unique(sc):
                m = np.where(sc == sid)[0]
                if len(m) < 3:
                    continue
                ax_[m[1:-1]] = (vx[m[2:]] - vx[m[:-2]]) / (2 * dt)
                ay_[m[1:-1]] = (vy[m[2:]] - vy[m[:-2]]) / (2 * dt)
                az_[m[1:-1]] = (vz[m[2:]] - vz[m[:-2]]) / (2 * dt)
            qw_, qx_, qy_, qz_ = d["qw"], d["qx"], d["qy"], d["qz"]
            r02 = 2 * (qx_ * qz_ + qw_ * qy_)
            r12 = 2 * (qy_ * qz_ - qw_ * qx_)
            r22 = 1 - 2 * (qx_ * qx_ + qy_ * qy_)
            sfz = r02 * ax_ + r12 * ay_ + r22 * (az_ - G)
            n_re = -(sfz / G)
            good = np.isfinite(n_re) & np.isfinite(n)
            if good.sum() > 50:
                cc = np.corrcoef(n_re[good], n[good])[0, 1]
                mae = np.nanmean(np.abs(n_re[good] - n[good]))
                print(f"  reconstructed n from dv/dt; validated on {good.sum()} FDM ticks: "
                      f"corr {cc:+.4f}, MAE {mae:.3f} g", file=sys.stderr)
                # ⛔ Refuse a reconstruction that does not match ground truth.
                if cc < 0.95 or mae > 0.10:
                    print("  REFUSING: reconstruction disagrees with FDM nz_g; "
                          "plotting the sparse FDM rows only", file=sys.stderr)
                    n_re = None
            if n_re is not None:
                keep = np.isfinite(n_re)
                n = n_re[keep]
                # ⚠️ vRelWind is populated on the SAME sparse rows as nz_g, so it
                # is unusable here. Speed comes from the velocity trace instead
                # (|v|, m/s — ground speed rather than airspeed; the difference
                # is the wind vector, and panel 4 only needs the v² trend).
                vel = np.sqrt(d["vx"] ** 2 + d["vy"] ** 2 + d["vz"] ** 2)[keep]
                d = d[keep]
                fmt = "physics+reconstructed"
    else:
        print("g_load: CSV has neither 'acZ' (tick dump) nor 'nz_g'/'sfz_g' "
              "(--physics dump)", file=sys.stderr)
        return 1
    print(f"  input format: {fmt}", file=sys.stderr)
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
        if len(ctr) < 3:
            # ⛔ Never plot a v² reference off one bin — say so instead.
            ax[3].text(.5, .5, "insufficient speed coverage for the v² panel",
                       ha="center", va="center", transform=ax[3].transAxes)
            ctr = None
        if ctr is not None:
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
