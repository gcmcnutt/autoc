"""Per-path tracking-score distribution — a proxy for tracking generalization (M2).

For each M1 source path (path_variant 0..N-1) the tracker is evaluated against many
wind variants. Two views:
  LEFT  — box+strip of the raw per-scenario tracking `score` per path (crashes marked).
          Shows spread + crash tail, but raw score is confounded by PATH LENGTH
          (longer paths accumulate more score).
  RIGHT — one histogram PER PATH of per-step score (score / steps_taken), which nulls
          out path-length differences → comparable per-step tracking quality. Narrow &
          low (very negative) = consistent good tracking; wide / long-tailed toward 0 =
          poor generalization across winds on that path.

`score` is the tracking axis (negative = lower = better; energy/stability excluded).
Source: `dmp-dump --meta-only` YAML (per-scenario path_variant + score + crash_reason +
steps "taken/total"). M2-only (pathgen has one path).

Usage:
  ./build/dmp-dump s3://autoc-m2/<run-id>/ --meta-only -i autoc-tracker.ini > meta.yaml
  python3 src/analytics/score_by_path.py --meta meta.yaml --label t14 \
    -o specs/037-20hz-control-loop/<run>_score_by_path.png
"""
import argparse
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CRASH = {"Eval", "HullStrike", "Sim", "Boot"}   # isCrash() set; RabbitComplete/TimeLimit = clean


def parse_meta(path):
    txt = open(path, errors="replace").read()
    gm = re.search(r"\bgen:\s*(\d+)", txt)
    gen = int(gm.group(1)) if gm else None
    rows = []
    for blk in re.split(r"\n\s*-\s+idx:", txt)[1:]:
        def g(key, pat=r"([-\w./]+)"):
            m = re.search(rf"\b{key}:\s*{pat}", blk)
            return m.group(1) if m else None
        pv = g("path_variant"); sc = g("score"); cr = g("crash_reason")
        st = g("steps", r"(\d+)\s*/\s*(\d+)")  # only grabs first group; reparse below
        sm = re.search(r"\bsteps:\s*(\d+)\s*/\s*(\d+)", blk)
        taken = int(sm.group(1)) if sm else 0
        if pv is None or sc is None:
            continue
        rows.append((int(pv), float(sc), cr or "", taken))
    return gen, rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--meta", required=True, help="dmp-dump --meta-only YAML")
    p.add_argument("--label", default="run")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    gen, rows = parse_meta(args.meta)
    if not rows:
        print("no scenarios parsed"); return
    paths = sorted(set(r[0] for r in rows))
    colors = plt.cm.viridis(np.linspace(0, 0.85, len(paths)))

    # layout: box spans the left column over 2 rows; 6 per-path panels fill the right 3x2
    ncol = (len(paths) + 1) // 2
    keys = [f"p{pv}" for pv in paths]
    top = ["box"] + keys[:ncol]
    bot = ["box"] + keys[ncol:] + ["."] * (1 + ncol - 1 - len(keys[ncol:]))
    fig, axd = plt.subplot_mosaic([top, bot], figsize=(14, 6),
                                  gridspec_kw={"width_ratios": [1.5] + [1] * ncol})

    ax = axd["box"]
    print(f"=== {args.label}  gen={gen}  per-path tracking-score (negative=better) ===")
    print(f"{'path':>4} {'n':>4} {'med':>8} {'std':>7} {'crash':>5} | per-step (score/ticks): "
          f"{'med':>9} {'std':>8}")
    box_data = []
    perstep_all = {}
    for i, pv in enumerate(paths):
        grp = [r for r in rows if r[0] == pv]
        s = np.array([r[1] for r in grp])
        crashed = np.array([r[2] in CRASH for r in grp])
        ps = np.array([r[1] / r[3] if r[3] > 0 else 0.0 for r in grp])
        perstep_all[pv] = (ps, crashed)
        box_data.append(s)
        # left strip
        x = pv + np.random.RandomState(pv).uniform(-0.18, 0.18, len(s))
        ax.scatter(x[~crashed], s[~crashed], s=12, color=colors[i], alpha=0.6, edgecolors="none")
        ax.scatter(x[crashed], s[crashed], s=30, color="red", marker="x", lw=1.3,
                   label="crash" if i == 0 else None)
        print(f"{pv:>4} {len(s):>4} {np.median(s):>8.1f} {s.std():>7.1f} {int(crashed.sum()):>5} | "
              f"{'':>20} {np.median(ps):>9.4f} {ps.std():>8.4f}")

    bp = ax.boxplot(box_data, positions=paths, widths=0.5, showfliers=False,
                    patch_artist=True, zorder=0)
    for i, b in enumerate(bp["boxes"]):
        b.set(facecolor=colors[i], alpha=0.18)
    ax.set_xlabel("path_variant"); ax.set_ylabel("raw per-scenario score (negative=better)")
    ax.set_title("raw score per path\n(box=IQR, dots=wind variants, ×=crash)", fontsize=9)
    ax.legend(fontsize=8, loc="lower left"); ax.grid(alpha=0.3, axis="y")

    # right: per-step histograms, shared x for comparability
    allps = np.concatenate([perstep_all[pv][0] for pv in paths])
    lo, hi = np.percentile(allps, [1, 99])
    bins = np.linspace(lo, hi, 22)
    for i, pv in enumerate(paths):
        a = axd[f"p{pv}"]
        ps, crashed = perstep_all[pv]
        a.hist(ps[~crashed], bins=bins, color=colors[i], alpha=0.8, label="ok")
        if crashed.any():
            a.hist(ps[crashed], bins=bins, color="red", alpha=0.7, label="crash")
        a.axvline(np.median(ps), color="k", ls="--", lw=1)
        a.set_title(f"path {pv}: med {np.median(ps):.3f}, σ {ps.std():.3f}", fontsize=8)
        a.tick_params(labelsize=7); a.grid(alpha=0.25)
    fig.supxlabel("per-step tracking score (score / steps flown) — path-length-normalized", fontsize=9)

    fig.suptitle(f"{args.label} gen={gen}: per-path tracking-score distribution "
                 "(generalization across wind variants)", fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
