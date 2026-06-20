"""Topographic map of the configured tracking cone (stepScore field) — a design tool.

Visualizes the per-tick tracking score as a function of the chase's position relative
to the trail rabbit, exactly as FitnessComputer::decomposeStepScore computes it:

    dist        = sqrt(along² + lateral²)
    angle       = acos(-along / dist)         # 0 = directly behind, π = ahead
    angleClamp  = min(angle, π/2)             # ahead saturates at "sideways"
    distScale   = behind (along≤0) | ahead (along>0)
    score       = 1 / (1 + (dist/distScale)² + (angleClamp/coneRad)²)

The streak threshold is a horizontal cut through this surface: streak accrues only
where score ≥ threshold. So the threshold contour IS the boundary of the
streak-eligible region — lowering it (e.g. 0.5→0.3) expands that region outward
(especially BEHIND, the forgiving direction), letting a lagging chase on a hard
course earn streak from further out.

Geometry (trail rabbit at origin): along<0 = behind it (lagging, away from target);
along>0 = ahead (overshooting toward the target + crash hull). Reads the actual ini
params so it reflects the live config.

Usage:
  python3 src/analytics/cone_topo.py --ini autoc-tracker.ini --thresholds 0.5,0.3 \
    -o specs/037-20hz-control-loop/cone_topo.png
"""
import argparse
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def ini_get(txt, key, default):
    m = re.search(rf"^\s*{key}\s*=\s*([-0-9.]+)", txt, re.M)
    return float(m.group(1)) if m else default


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ini", required=True)
    p.add_argument("--thresholds", default="0.5,0.3", help="comma list of streak thresholds to overlay")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    txt = open(args.ini).read()
    behind = ini_get(txt, "FitDistScaleBehind", 7.0)
    ahead = ini_get(txt, "FitDistScaleAhead", 2.0)
    cone_deg = ini_get(txt, "FitConeAngleDeg", 45.0)
    cur_thr = ini_get(txt, "FitStreakThreshold", 0.5)
    trail = ini_get(txt, "TrailDistance", 3.048)
    hull_r = ini_get(txt, "CrashHullRadius", 1.0)
    cone_rad = np.deg2rad(cone_deg)
    thr_list = [float(x) for x in args.thresholds.split(",")]

    # score field over (along, lateral); along: behind(neg) .. ahead(pos)
    A = np.linspace(-18, 8, 600)      # along (m)
    L = np.linspace(-9, 9, 500)       # lateral (m)
    AA, LL = np.meshgrid(A, L)
    dist = np.sqrt(AA**2 + LL**2)
    with np.errstate(divide="ignore", invalid="ignore"):
        cosang = np.clip(-AA / np.where(dist < 1e-9, 1e-9, dist), -1, 1)
    angle = np.minimum(np.arccos(cosang), np.pi / 2)
    dscale = np.where(AA <= 0.0, behind, ahead)
    score = 1.0 / (1.0 + (dist / dscale)**2 + (angle / cone_rad)**2)
    score[dist < 1e-6] = 1.0

    fig, ax = plt.subplots(figsize=(12, 7))
    levels = np.linspace(0, 1, 21)
    cf = ax.contourf(AA, LL, score, levels=levels, cmap="terrain", alpha=0.9)
    ax.contour(AA, LL, score, levels=np.arange(0.1, 1.0, 0.1), colors="k", linewidths=0.4, alpha=0.4)
    fig.colorbar(cf, ax=ax, label="step score (1=on trail rabbit)")

    # streak-threshold contours = boundary of streak-eligible region.
    # (The region is an entirely BEHIND-the-rabbit cone: directly ahead the angle
    # clamps to π/2 → score caps at ~0.2, so streak never accrues ahead.)
    tcolors = ["red", "magenta", "darkorange", "purple"]
    extents = {}
    for i, thr in enumerate(thr_list):
        ax.contour(AA, LL, score, levels=[thr], colors=[tcolors[i % len(tcolors)]],
                   linewidths=2.4, linestyles="-" if thr == cur_thr else "--")
        along0 = score[np.argmin(np.abs(L)), :]                 # along profile at lateral≈0
        behind_reach = A[along0 >= thr].min() if np.any(along0 >= thr) else 0.0
        mask = score >= thr
        latw = np.abs(LL[mask]).max() if mask.any() else 0.0
        area = mask.mean() * (A[-1] - A[0]) * (L[-1] - L[0])    # m² of streak-eligible region
        extents[thr] = (behind_reach, latw, area)
        tag = "  (current)" if thr == cur_thr else ""
        ax.plot([], [], color=tcolors[i % len(tcolors)], lw=2.4,
                ls="-" if thr == cur_thr else "--",
                label=f"streak≥{thr}{tag}: reaches {behind_reach:.1f} m behind, ±{latw:.1f} m wide, {area:.0f} m²")

    # landmarks
    ax.plot(0, 0, "w*", ms=18, mec="k", mew=1.2, label="trail rabbit (ideal, score=1)")
    ax.plot(trail, 0, "kv", ms=11, label=f"target (+{trail:.2f}m ahead)")
    th = np.linspace(0, 2*np.pi, 100)
    ax.plot(trail + hull_r*np.cos(th), hull_r*np.sin(th), "r:", lw=1.6, label=f"crash hull (r={hull_r}m)")

    ax.axhline(0, color="k", lw=0.5, alpha=0.4)
    ax.axvline(0, color="k", lw=0.5, alpha=0.4)
    ax.set_xlabel("along-track from trail rabbit (m):  ◄ behind (lag, scale %.0f)   ahead (overshoot, scale %.0f) ►"
                  % (behind, ahead))
    ax.set_ylabel("lateral offset (m)")
    ax.set_title(f"Tracking-cone topography — {args.ini}\n"
                 f"behind={behind} ahead={ahead} cone={cone_deg}°  |  streak threshold contours: "
                 f"where the multiplier can engage", fontsize=10)
    ax.legend(fontsize=8, loc="upper left"); ax.set_aspect("equal"); ax.grid(alpha=0.2)
    fig.tight_layout(); fig.savefig(args.out, dpi=120)
    print(f"params: behind={behind} ahead={ahead} cone={cone_deg}° trail={trail} hull={hull_r} current_thr={cur_thr}")
    for thr in thr_list:
        b, w, a = extents[thr]
        print(f"  streak≥{thr}: reaches {b:.1f} m behind, ±{w:.1f} m lateral half-width, {a:.0f} m² eligible region")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
