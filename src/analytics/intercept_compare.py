"""A/B overlay of two tracker runs on the policy/encounter axes — to see how a
fitness change (e.g. 038 hull-crash penalty) shifts the *emergent tactic*,
instead of eyeballing two separate intercept_analysis PNGs.

Reuses intercept_analysis's CSV loader (chase/target geometry, derived dist +
closing-rate, beacon visibility). Overlays, on shared axes:
  1. outTh vs spn0  — distance→throttle policy (the discriminator; intercept Panel B)
  2. outTh vs dspn  — closure→throttle policy (Panel C)
  3. min-dist-to-target histogram — does the encounter envelope actually change? (Panel E)

Usage:
  python3 src/analytics/intercept_compare.py \
    --a t11-off:/tmp/t11_g400.csv --b t12-on:/tmp/t12_g400.csv \
    --gen 400 -o specs/037-20hz-control-loop/t11_vs_t12_tactics.png
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import intercept_analysis as ia  # noqa: E402


def gather(scenarios, xkey, ykey, vis_only=True):
    xs, ys = [], []
    for d in scenarios.values():
        if xkey not in d or ykey not in d:
            continue
        x = np.asarray(d[xkey], float); y = np.asarray(d[ykey], float)
        n = min(len(x), len(y))
        x, y = x[:n], y[:n]
        if vis_only and "blC0" in d and "brC0" in d:
            bl = np.asarray(d["blC0"])[:n]; br = np.asarray(d["brC0"])[:n]
            m = (bl < ia.CEP_VISIBLE_THRESH) & (br < ia.CEP_VISIBLE_THRESH)
            x, y = x[m], y[m]
        xs.append(x); ys.append(y)
    return (np.concatenate(xs), np.concatenate(ys)) if xs else (np.array([]), np.array([]))


def binned(x, y, lo, hi, nb=22, mincount=8):
    edges = np.linspace(lo, hi, nb + 1)
    idx = np.digitize(x, edges) - 1
    cen, mn, sd = [], [], []
    for b in range(nb):
        m = idx == b
        if m.sum() >= mincount:
            cen.append(0.5 * (edges[b] + edges[b + 1]))
            mn.append(float(y[m].mean())); sd.append(float(y[m].std()))
    return np.array(cen), np.array(mn), np.array(sd)


def min_dists(scenarios):
    out = []
    for d in scenarios.values():
        dt = d.get("dist_target")
        if dt is not None and len(dt) > 2:
            out.append(float(np.min(dt)))
    return np.array(out)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--a", required=True, help="NAME:CSV (baseline, e.g. penalty-off)")
    p.add_argument("--b", required=True, help="NAME:CSV (test, e.g. penalty-on)")
    p.add_argument("--gen", default="?", help="gen label for the title")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    (na, fa) = args.a.split(":", 1)
    (nb, fb) = args.b.split(":", 1)
    A = ia.load_dmp_dump_csv(fa)
    B = ia.load_dmp_dump_csv(fb)
    cA, cB = "tab:blue", "tab:red"

    fig, axs = plt.subplots(1, 3, figsize=(16, 4.8))
    fig.suptitle(f"tactic A/B @ gen {args.gen}:  {na} (blue) vs {nb} (red) — "
                 "policy curves on shared axes (binned mean ± σ)", fontsize=11)

    # Panel 1: outTh vs spn0 (distance→throttle). spn0 bigger = closer.
    ax = axs[0]
    for (S, c, lbl) in ((A, cA, na), (B, cB, nb)):
        x, y = gather(S, "spn0", "outTh")
        x, y = x[x > 1e-4], y[x > 1e-4]   # visible/non-gated ticks only
        if len(x) == 0:
            continue
        hi = np.percentile(x, 99)
        cen, mn, sd = binned(x, y, 0.0, hi)
        ax.plot(cen, mn, color=c, lw=2.0, label=f"{lbl} (n={len(x)})")
        ax.fill_between(cen, mn - sd, mn + sd, color=c, alpha=0.12)
    ax.set_xlabel("spn0 (NDC beacon span — bigger = closer)")
    ax.set_ylabel("out_th (NN throttle)")
    ax.set_title("distance → throttle policy  (intercept Panel B)", fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # Panel 2: outTh vs dspn (closure→throttle).
    ax = axs[1]
    for (S, c, lbl) in ((A, cA, na), (B, cB, nb)):
        x, y = gather(S, "dspn", "outTh")
        if len(x) == 0:
            continue
        lo, hi = np.percentile(x, 1), np.percentile(x, 99)
        cen, mn, sd = binned(x, y, lo, hi)
        ax.plot(cen, mn, color=c, lw=2.0, label=lbl)
        ax.fill_between(cen, mn - sd, mn + sd, color=c, alpha=0.12)
    ax.axvline(0, color="gray", lw=0.6, alpha=0.5)
    ax.set_xlabel("dspn (closure rate proxy — +ve = approaching)")
    ax.set_ylabel("out_th (NN throttle)")
    ax.set_title("closure → throttle policy  (intercept Panel C)", fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # Panel 3: encounter min-dist-to-target histogram.
    ax = axs[2]
    mdA, mdB = min_dists(A), min_dists(B)
    bins = np.linspace(0, max(mdA.max(initial=10), mdB.max(initial=10)), 36)
    ax.hist(mdA, bins=bins, alpha=0.5, color=cA, label=f"{na} (med {np.median(mdA):.1f}m)")
    ax.hist(mdB, bins=bins, alpha=0.5, color=cB, label=f"{nb} (med {np.median(mdB):.1f}m)")
    ax.axvline(1.0, color="k", ls="-.", lw=0.8, label="hull 1m")
    ax.axvline(3.048, color="tab:olive", ls="-.", lw=0.8, label="trail 3.05m")
    ax.set_xlabel("per-scenario min dist to target (m)")
    ax.set_ylabel("scenarios")
    ax.set_title("encounter envelope  (intercept Panel E)", fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    # quick numeric tell
    print(f"  min-dist median: {na} {np.median(mdA):.2f}m  vs  {nb} {np.median(mdB):.2f}m")
    print(f"  frac scenarios min-dist < hull(1m): {na} {100*np.mean(mdA<1):.0f}%  vs  {nb} {100*np.mean(mdB<1):.0f}%")
    print(f"  frac < 2m: {na} {100*np.mean(mdA<2):.0f}%  vs  {nb} {100*np.mean(mdB<2):.0f}%")


if __name__ == "__main__":
    main()
