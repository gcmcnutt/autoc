"""Per-path tracking-score distribution — a proxy for tracking generalization (M2).

For each M1 source path (path_variant 0..N-1) the tracker is evaluated against many
wind variants. The key generalization signal is the PER-STEP score (score / steps_taken,
which nulls out path-length differences) and how WIDE its distribution is per path:
narrow & low (very negative) = consistent good tracking; wide / long-tailed = poor
generalization across winds. Reward-gradient knobs (streak threshold, crash penalty)
mostly relocate trade-offs; the threshold-invariant per-step distribution narrowing is
what shows real generalization.

Modes:
  SINGLE run (`--meta FILE --label NAME`): left = box+strip of raw per-scenario score per
    path (crashes marked); right = 2x3 per-step histograms (length-normalized) per path.
  MULTI run (`--run NAME:FILE` repeatable): left = per-path per-step SPREAD (std) bars,
    one bar per run — narrowing across runs at a glance; right = 2x3 per-path panels
    overlaying each run's per-step distribution. Use to compare tuning runs (does t15
    narrow vs t14, or just shift while staying wide?).

`score` is the tracking axis (negative=better). Source: `dmp-dump --meta-only` YAML
(per-scenario path_variant + score + crash_reason + steps "taken/total"). M2-only.

Usage:
  ./build/dmp-dump s3://autoc-m2/<run-id>/ --meta-only -i autoc-tracker.ini > meta.yaml
  python3 src/analytics/score_by_path.py --meta meta.yaml --label t14 -o out.png
  python3 src/analytics/score_by_path.py --run t11:t11.yaml --run t14:t14.yaml \
    --run t15:t15.yaml -o out.png
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
        sm = re.search(r"\bsteps:\s*(\d+)\s*/\s*(\d+)", blk)
        taken = int(sm.group(1)) if sm else 0
        if pv is None or sc is None:
            continue
        rows.append((int(pv), float(sc), cr or "", taken))
    return gen, rows


def per_step(rows, pv):
    grp = [r for r in rows if r[0] == pv]
    ps = np.array([r[1] / r[3] if r[3] > 0 else 0.0 for r in grp])
    crashed = np.array([r[2] in CRASH for r in grp])
    return ps, crashed, np.array([r[1] for r in grp])


NWIND = 49  # winds per path (6 paths x 49 = 294); path = scenario // NWIND


def parse_tick(csv_path):
    """Per-tick tracking error ‖chase − trail rabbit‖ from dmp-dump --csv-only, grouped by path.

    This is the GROUND-TRUTH tracking measure (reward-shaping-invariant): how far the chase is
    from the ideal trail point each tick. Returns {path_variant: np.array(err_meters)}.
    """
    import csv as _csv
    by_path = {}
    for r in _csv.DictReader(open(csv_path)):
        pv = int(r["scenario"]) // NWIND
        err = ((float(r["px"]) - float(r["trX"]))**2 +
               (float(r["py"]) - float(r["trY"]))**2 +
               (float(r["pz"]) - float(r["trZ"]))**2) ** 0.5
        by_path.setdefault(pv, []).append(err)
    return {pv: np.array(v) for pv, v in by_path.items()}


def plot_single(name, gen, rows, out, tick=None):
    paths = sorted(set(r[0] for r in rows))
    colors = plt.cm.viridis(np.linspace(0, 0.85, len(paths)))
    ncol = (len(paths) + 1) // 2
    keys = [f"p{pv}" for pv in paths]
    top = ["box"] + keys[:ncol]; bot = ["box"] + keys[ncol:]
    fig, axd = plt.subplot_mosaic([top, bot], figsize=(14, 6),
                                  gridspec_kw={"width_ratios": [1.5] + [1] * ncol})
    ax = axd["box"]; box_data = []; persteps = {}
    print(f"=== {name} gen={gen} per-path tracking-score (negative=better) ===")
    print(f"{'path':>4} {'n':>4} {'med':>8} {'std':>7} {'crash':>5} | per-step med/std")
    for i, pv in enumerate(paths):
        ps, crashed, s = per_step(rows, pv); persteps[pv] = (ps, crashed); box_data.append(s)
        x = pv + np.random.RandomState(pv).uniform(-0.18, 0.18, len(s))
        ax.scatter(x[~crashed], s[~crashed], s=12, color=colors[i], alpha=0.6, edgecolors="none")
        ax.scatter(x[crashed], s[crashed], s=30, color="red", marker="x", lw=1.3,
                   label="crash" if i == 0 else None)
        print(f"{pv:>4} {len(s):>4} {np.median(s):>8.1f} {s.std():>7.1f} {int(crashed.sum()):>5} | "
              f"{np.median(ps):>8.4f} {ps.std():.4f}")
    bp = ax.boxplot(box_data, positions=paths, widths=0.5, showfliers=False, patch_artist=True, zorder=0)
    for i, b in enumerate(bp["boxes"]):
        b.set(facecolor=colors[i], alpha=0.18)
    ax.set_xlabel("path_variant"); ax.set_ylabel("raw per-scenario score (negative=better)")
    ax.set_title("raw score per path\n(box=IQR, dots=wind variants, ×=crash)", fontsize=9)
    ax.legend(fontsize=8, loc="lower left"); ax.grid(alpha=0.3, axis="y")
    if tick is not None:
        # RIGHT = ground-truth per-tick tracking ERROR distance histograms (‖chase − trail rabbit‖),
        # bucketed across every tick of each path's scenarios. Reward-shaping-invariant.
        allv = np.concatenate([tick[pv] for pv in paths if pv in tick])
        hi = np.percentile(allv, 99); bins = np.linspace(0, hi, 40)
        print(f"--- per-tick tracking error ‖chase−trail‖ (m), by path (gen {gen}) ---")
        for i, pv in enumerate(paths):
            a = axd[f"p{pv}"]; e = tick.get(pv, np.array([0.0]))
            a.hist(e, bins=bins, color=colors[i], alpha=0.85)
            a.axvline(np.median(e), color="k", ls="--", lw=1.3)
            a.axvline(3.048, color="green", ls="-", lw=1.1, alpha=0.7)   # trail distance ref
            f5 = 100 * (e < 5).mean()
            a.set_title(f"path {pv}: med {np.median(e):.1f}m, %<5m {f5:.0f}", fontsize=8)
            a.tick_params(labelsize=7); a.grid(alpha=0.25)
            print(f"  path {pv}: med {np.median(e):.1f}m  mean {e.mean():.1f}m  p90 {np.percentile(e,90):.1f}m  %<5m {f5:.0f}")
        fig.supxlabel("per-tick tracking error: ‖chase − trail rabbit‖ (m)  "
                      "[green=trail dist 3.0m; 0=perfect, large=far/lost]", fontsize=9)
        fig.suptitle(f"{name} gen={gen}: raw score per path (left) + ground-truth per-tick "
                     "tracking-error distribution by path (right)", fontsize=10)
    else:
        allps = np.concatenate([persteps[pv][0] for pv in paths]); lo, hi = np.percentile(allps, [1, 99])
        bins = np.linspace(lo, hi, 22)
        for i, pv in enumerate(paths):
            a = axd[f"p{pv}"]; ps, crashed = persteps[pv]
            a.hist(ps[~crashed], bins=bins, color=colors[i], alpha=0.8)
            if crashed.any():
                a.hist(ps[crashed], bins=bins, color="red", alpha=0.7)
            a.axvline(np.median(ps), color="k", ls="--", lw=1)
            a.set_title(f"path {pv}: med {np.median(ps):.3f}, σ {ps.std():.3f}", fontsize=8)
            a.tick_params(labelsize=7); a.grid(alpha=0.25)
        fig.supxlabel("per-step tracking score (score / steps flown) — path-length-normalized", fontsize=9)
        fig.suptitle(f"{name} gen={gen}: per-path tracking-score distribution", fontsize=11)
    fig.tight_layout(); fig.savefig(out, dpi=110); print(f"\nwrote {out}")


def plot_multi(runs, out):
    # runs: list of (name, gen, rows)
    paths = sorted(set(r[0] for _, _, rows in runs for r in rows))
    rcolors = ["tab:blue", "tab:red", "tab:green", "tab:purple", "tab:orange"]
    ncol = (len(paths) + 1) // 2
    keys = [f"p{pv}" for pv in paths]
    top = ["std"] + keys[:ncol]; bot = ["std"] + keys[ncol:]
    fig, axd = plt.subplot_mosaic([top, bot], figsize=(15, 6.5),
                                  gridspec_kw={"width_ratios": [1.6] + [1] * ncol})
    # per-run per-path per-step arrays
    P = {name: {pv: per_step(rows, pv)[0] for pv in paths} for name, _, rows in runs}
    # left: spread (std) bars per path, grouped by run — lower = narrower = better generalization
    axs = axd["std"]; w = 0.8 / len(runs)
    print(f"=== per-step spread (std) per path — lower = narrower = better generalization ===")
    hdr = "path | " + " ".join(f"{n:>9}" for n, _, _ in runs); print(hdr)
    for j, pv in enumerate(paths):
        for k, (name, _, _) in enumerate(runs):
            axs.bar(pv + (k - (len(runs)-1)/2)*w, P[name][pv].std(), w,
                    color=rcolors[k % len(rcolors)], label=name if j == 0 else None)
        print(f"{pv:>4} | " + " ".join(f"{P[n][pv].std():>9.4f}" for n, _, _ in runs))
    axs.set_xlabel("path_variant"); axs.set_ylabel("per-step score std (spread)")
    axs.set_title("per-path spread by run\n(narrower = better generalization)", fontsize=9)
    axs.set_xticks(paths); axs.legend(fontsize=8); axs.grid(alpha=0.3, axis="y")
    # right: per-path overlaid per-step distributions
    allps = np.concatenate([P[n][pv] for n in P for pv in paths]); lo, hi = np.percentile(allps, [1, 99])
    bins = np.linspace(lo, hi, 24)
    for pv in paths:
        a = axd[f"p{pv}"]
        for k, (name, _, _) in enumerate(runs):
            a.hist(P[name][pv], bins=bins, histtype="step", lw=1.8, density=True,
                   color=rcolors[k % len(rcolors)], label=name)
            a.axvline(np.median(P[name][pv]), color=rcolors[k % len(rcolors)], ls="--", lw=0.9)
        a.set_title(f"path {pv}", fontsize=8); a.tick_params(labelsize=7); a.grid(alpha=0.25)
    axd[f"p{paths[0]}"].legend(fontsize=7)
    fig.supxlabel("per-step tracking score (score / steps flown) — path-length-normalized", fontsize=9)
    fig.suptitle("Per-path per-step score distribution by run — does it narrow (generalize) or just shift?  "
                 + " | ".join(f"{n} gen{g}" for n, g, _ in runs), fontsize=10)
    fig.tight_layout(); fig.savefig(out, dpi=110); print(f"\nwrote {out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--meta", help="single-run: dmp-dump --meta-only YAML")
    p.add_argument("--label", default="run")
    p.add_argument("--run", action="append", help="multi-run overlay: NAME:meta.yaml (repeatable)")
    p.add_argument("--csv", help="single-run: dmp-dump --csv-only per-tick CSV → right panels show "
                                 "ground-truth per-tick tracking-error distance by path (else per-step score)")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    if args.run:
        runs = []
        for spec in args.run:
            name, _, path = spec.partition(":")
            gen, rows = parse_meta(path)
            if rows:
                runs.append((name, gen, rows))
        if not runs:
            print("no runs parsed"); return
        plot_multi(runs, args.out)
    else:
        if not args.meta:
            print("need --meta FILE (single) or --run NAME:FILE (multi)"); return
        gen, rows = parse_meta(args.meta)
        if not rows:
            print("no scenarios parsed"); return
        tick = parse_tick(args.csv) if args.csv else None
        plot_single(args.label, gen, rows, args.out, tick=tick)


if __name__ == "__main__":
    main()
