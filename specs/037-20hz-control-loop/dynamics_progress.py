#!/usr/bin/env python3
"""037 flight-dynamics progress over generations (dmp-dump-based).

Fills the gap between the per-gen SNAPSHOT (per_axis_aggressiveness) and the
dctrl/mag time series (plot_per_axis_time_series): the de-alias + tracking
quality metrics as TRAJECTORIES across generations, so "is flight dynamics
improving" is one chart instead of N snapshots.

Panels (all per-generation, elite individual, mean-of-scenarios):
  1. lag-1 autocorr per axis     — the 037 gate metric. >=0 = smoothed;
                                   reference lines at 0 and the 035-t6 10 Hz
                                   roll baseline (-0.24).
  2. sign-flip %  per axis       — gate companion; reference lines at the
                                   <=36% target and the 56% 10 Hz baseline.
  3. saturation % per axis       — |out| >= 0.95 fraction; watches where the
                                   bang-bang muscle migrates (037 finding:
                                   roll -> throttle once the servo got real).
  4. regime occupancy            — % ticks in {tracking, intercept, patrol}
                                   (operator 2026-06-10: the fitness/streak
                                   space behind the target means stpPt >= 0.5
                                   IS tracking; below that the craft is in a
                                   different behavioral regime — closing on
                                   the target = intercept, else patrol — and
                                   they deserve separate metrics). Forward
                                   progress = occupancy shifting
                                   patrol -> intercept -> tracking.
  5. target distance (m)         — mean + p95 across ticks/scenarios; the
                                   operator's preferred tracking-competence
                                   measure (completions are a red herring per
                                   project_servo_era_progress_metrics —
                                   spiral arrives without tracking).

Each generation costs one `dmp-dump --gen N --csv-only` S3 fetch, so sample
with --stride and use --cache: computed rows persist and re-runs on a live
run fetch only the new gens (mirrors the REPORTS.md incremental run-summary
workflow).

Usage:
    python3 dynamics_progress.py --run s3://autoc-m1/<run-id>/ \
        --gens 1-157 --stride 5 -i autoc.ini \
        --cache /tmp/t6_dynamics_cache.csv \
        --label autoc-037-t6-m1-20hz -o out.png

Stdlib csv + numpy + matplotlib. Pathgen/M1 CSV contract (needs columns
scenario, out_pt, out_rl, out_th, dist).
"""
import argparse
import csv
import io
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("out_pt", "pitch", "tab:red"),
        ("out_rl", "roll", "tab:blue"),
        ("out_th", "throttle", "tab:green")]
SAT_KNEE = 0.95

# 035-t6 10 Hz converged-elite roll references (dealias_metrics.py header).
ROLL_BASELINE_AC = -0.24
ROLL_BASELINE_FLIP = 56.0
ROLL_GATE_FLIP = 36.0

# Regime classification (operator 2026-06-10): stpPt >= TRACK_THRESHOLD is
# "in tracking" (the location metric's cone behind the target); below it,
# ticks where the craft is closing on the target are "intercept", the rest
# "patrol". Closing is judged on the smoothed first-difference of dist to
# tolerate per-tick noise.
TRACK_THRESHOLD = 0.5
CLOSE_SMOOTH_TICKS = 5

CACHE_FIELDS = (["gen"]
                + [f"ac_{n}" for _, n, _ in AXES]
                + [f"flip_{n}" for _, n, _ in AXES]
                + [f"sat_{n}" for _, n, _ in AXES]
                + ["occ_track", "occ_intercept", "occ_patrol"]
                + ["dist_mean", "dist_p95"])


def per_scenario_series(rows):
    """Group the per-tick CSV rows by scenario, preserving tick order."""
    out = {}
    for r in rows:
        out.setdefault(int(r["scenario"]), []).append(r)
    return out


def lag1_autocorr(x):
    if len(x) < 3:
        return float("nan")
    a, b = x[:-1], x[1:]
    sa, sb = a.std(), b.std()
    if sa == 0.0 or sb == 0.0:
        return float("nan")
    return float(np.mean((a - a.mean()) * (b - b.mean())) / (sa * sb))


def sign_flip_pct(x):
    if len(x) < 2:
        return float("nan")
    s = np.sign(x)
    nz = (s[:-1] != 0) & (s[1:] != 0)
    if nz.sum() == 0:
        return 0.0
    return 100.0 * float(np.mean(s[:-1][nz] != s[1:][nz]))


def compute_gen_metrics(csv_text):
    """Mean-of-scenarios metrics for one generation's per-tick CSV."""
    rows = list(csv.DictReader(io.StringIO(csv_text)))
    need = {"scenario", "out_pt", "out_rl", "out_th", "dist", "stpPt"}
    if not rows or not need.issubset(rows[0].keys()):
        raise ValueError("CSV missing pathgen columns "
                         f"(have: {sorted(rows[0].keys()) if rows else 'none'})")
    by_scn = per_scenario_series(rows)

    acs = {n: [] for _, n, _ in AXES}
    flips = {n: [] for _, n, _ in AXES}
    sats = {n: [] for _, n, _ in AXES}
    occ = {"track": [], "intercept": [], "patrol": []}
    dist_means, dist_all = [], []

    for scn_rows in by_scn.values():
        for col, name, _ in AXES:
            x = np.array([float(r[col]) for r in scn_rows])
            acs[name].append(lag1_autocorr(x))
            flips[name].append(sign_flip_pct(x))
            sats[name].append(100.0 * float(np.mean(np.abs(x) >= SAT_KNEE)))
        d = np.array([float(r["dist"]) for r in scn_rows])
        dist_means.append(float(d.mean()))
        dist_all.extend(d.tolist())

        # Regime occupancy. tracking: stpPt >= threshold. Below: closing
        # (smoothed d(dist) < 0) = intercept, else patrol.
        sp = np.array([float(r["stpPt"]) for r in scn_rows])
        tracking = sp >= TRACK_THRESHOLD
        if len(d) >= 2:
            dd = np.diff(d, prepend=d[0])
            k = min(CLOSE_SMOOTH_TICKS, len(dd))
            kernel = np.ones(k) / k
            dd_s = np.convolve(dd, kernel, mode="same")
            closing = dd_s < 0.0
        else:
            closing = np.zeros(len(d), dtype=bool)
        intercept = ~tracking & closing
        patrol = ~tracking & ~closing
        n = max(len(sp), 1)
        occ["track"].append(100.0 * float(tracking.sum()) / n)
        occ["intercept"].append(100.0 * float(intercept.sum()) / n)
        occ["patrol"].append(100.0 * float(patrol.sum()) / n)

    rec = {}
    for _, name, _ in AXES:
        rec[f"ac_{name}"] = float(np.nanmean(acs[name]))
        rec[f"flip_{name}"] = float(np.nanmean(flips[name]))
        rec[f"sat_{name}"] = float(np.nanmean(sats[name]))
    rec["occ_track"] = float(np.mean(occ["track"]))
    rec["occ_intercept"] = float(np.mean(occ["intercept"]))
    rec["occ_patrol"] = float(np.mean(occ["patrol"]))
    rec["dist_mean"] = float(np.mean(dist_means))
    rec["dist_p95"] = float(np.percentile(np.array(dist_all), 95))
    return rec


def fetch_gen_csv(dmp_dump, run, gen, ini):
    cmd = [dmp_dump, run, "--gen", str(gen), "--csv-only", "-i", ini]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"dmp-dump --gen {gen} failed: {res.stderr.strip().splitlines()[-1:]}" )
    return res.stdout


def load_cache(path):
    if not path or not os.path.isfile(path):
        return {}
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                out[int(r["gen"])] = {k: float(r[k])
                                      for k in CACHE_FIELDS if k != "gen"}
            except (KeyError, TypeError, ValueError):
                # Cache row from an older schema (e.g. pre-regime-occupancy)
                # — drop it; the gen refetches.
                continue
    return out


def save_cache(path, data):
    if not path:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CACHE_FIELDS)
        w.writeheader()
        for gen in sorted(data):
            w.writerow({"gen": gen, **data[gen]})


def parse_gens(spec, stride):
    lo, hi = (int(x) for x in spec.split("-"))
    gens = list(range(lo, hi + 1, stride))
    if gens[-1] != hi:
        gens.append(hi)  # always include the newest gen
    return gens


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True, help="s3://<bucket>/<run-id>/")
    p.add_argument("--gens", required=True, help="LO-HI generation range (actualGen)")
    p.add_argument("--stride", type=int, default=5)
    p.add_argument("--dmp-dump", default="./build/dmp-dump")
    p.add_argument("-i", "--config", default="autoc.ini")
    p.add_argument("--cache", default=None,
                   help="CSV cache of computed per-gen rows (skip refetch)")
    p.add_argument("--label", default="run")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    data = load_cache(args.cache)
    gens = parse_gens(args.gens, args.stride)
    todo = [g for g in gens if g not in data]
    print(f"{len(gens)} gens requested, {len(todo)} to fetch "
          f"({len(data)} cached)")
    for i, gen in enumerate(todo):
        try:
            rec = compute_gen_metrics(fetch_gen_csv(args.dmp_dump, args.run,
                                                    gen, args.config))
        except (RuntimeError, ValueError) as e:
            print(f"  gen {gen}: SKIP ({e})", file=sys.stderr)
            continue
        data[gen] = rec
        save_cache(args.cache, data)  # persist as we go — interruptible
        print(f"  [{i+1}/{len(todo)}] gen {gen}: "
              f"roll ac {rec['ac_roll']:+.2f} flip {rec['flip_roll']:.0f}% "
              f"thr sat {rec['sat_throttle']:.0f}% dist {rec['dist_mean']:.1f}m "
              f"occ t/i/p {rec['occ_track']:.0f}/{rec['occ_intercept']:.0f}/"
              f"{rec['occ_patrol']:.0f}%")

    plot_gens = sorted(g for g in data if gens[0] <= g <= gens[-1])
    if not plot_gens:
        raise SystemExit("no generations computed")
    G = np.array(plot_gens)

    def series(key):
        return np.array([data[g][key] for g in plot_gens])

    fig, axs = plt.subplots(5, 1, figsize=(11, 16), sharex=True)
    fig.suptitle(f"{args.label} — flight-dynamics progress over generations\n"
                 "(elite, mean-of-scenarios; refs = 035-t6 10 Hz converged elite)")

    ax = axs[0]
    for _, name, c in AXES:
        ax.plot(G, series(f"ac_{name}"), color=c, label=name)
    ax.axhline(0.0, color="k", lw=0.8, ls="--", label="gate: ≥0")
    ax.axhline(ROLL_BASELINE_AC, color="tab:blue", lw=0.8, ls=":",
               label=f"10 Hz roll baseline {ROLL_BASELINE_AC}")
    ax.set_ylabel("lag-1 autocorr")
    ax.legend(fontsize=8, ncol=2)
    ax.set_title("command coherence (de-alias gate metric)", fontsize=10)

    ax = axs[1]
    for _, name, c in AXES:
        ax.plot(G, series(f"flip_{name}"), color=c, label=name)
    ax.axhline(ROLL_GATE_FLIP, color="k", lw=0.8, ls="--", label="gate: ≤36%")
    ax.axhline(ROLL_BASELINE_FLIP, color="tab:blue", lw=0.8, ls=":",
               label="10 Hz roll baseline 56%")
    ax.set_ylabel("sign-flip %")
    ax.legend(fontsize=8, ncol=2)
    ax.set_title("tick-to-tick sign flips (bang-bang detector)", fontsize=10)

    ax = axs[2]
    for _, name, c in AXES:
        ax.plot(G, series(f"sat_{name}"), color=c, label=name)
    ax.set_ylabel(f"% ticks |out| ≥ {SAT_KNEE}")
    ax.legend(fontsize=8)
    ax.set_title("saturation (watch the muscle migrate between axes)",
                 fontsize=10)

    ax = axs[3]
    ax.stackplot(G, series("occ_track"), series("occ_intercept"),
                 series("occ_patrol"),
                 labels=["tracking (stpPt ≥ 0.5)", "intercept (closing)",
                         "patrol"],
                 colors=["tab:green", "tab:orange", "tab:gray"], alpha=0.8)
    ax.set_ylabel("% ticks")
    ax.set_ylim(0, 100)
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("regime occupancy (progress = patrol → intercept → tracking)",
                 fontsize=10)

    ax = axs[4]
    ax.plot(G, series("dist_mean"), color="tab:purple", label="mean dist")
    ax.plot(G, series("dist_p95"), color="tab:purple", ls=":", alpha=0.7,
            label="p95 dist")
    ax.set_ylabel("target distance (m)")
    ax.set_xlabel("generation")
    ax.legend(fontsize=8)
    ax.set_title("tracking competence (distance-to-target — the spiral-proof "
                 "measure)", fontsize=10)

    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
