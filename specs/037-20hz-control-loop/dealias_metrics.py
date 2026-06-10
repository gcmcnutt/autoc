#!/usr/bin/env python3
"""037 de-alias gate metric tool (dmp-dump-based).

The Phase-A go/no-go for the 20 Hz control-loop feature. 035 found the
controller's roll bang-bang is largely a **10 Hz artifact**, not an objective
problem: at the historical t6 10 Hz baseline only *roll* dithers -- saturated,
sign-flipping, anti-persistent -- while pitch/throttle stay smooth. A faster
control loop should de-alias roll without hurting tracking.

This script computes, per NN output axis (roll primary; pitch + throttle too),
the four de-alias metrics straight off a `dmp-dump --csv-only` per-tick CSV:

  * lag-1 autocorr  -- Pearson autocorrelation of the per-tick command series at
                      lag 1. **Primary gate metric.** Negative => bang-bang
                      dither (anti-persistent); >=0 => smoothed/coherent.
  * sign-flip rate  -- fraction of consecutive-tick pairs whose command changes
                      sign. Reported as %. (The "56%" t6 roll figure is this.)
  * saturation %    -- fraction of ticks with |command| >= knee*max. Outputs are
                      tanh-bounded to [-1, 1] so max = 1.0; knee default 0.95,
                      CLI-tunable via --sat-knee.
  * dctrl           -- mean |dcommand| per tick (mean abs first-difference). The
                      variation-stable comparator per
                      `project_late_run_fitness_interpretation`.

GATE TARGET (Phase-A, roll axis, vs historical t6 10 Hz):
  roll lag-1 autocorr  negative -> >=0     (t6 ~ -0.24)
  AND sign-flip        ->=20 points        (t6 56% -> <=36%)
  at tracking within noise of historical t6 10 Hz (spec Clarifications Q1/Q3).

AGGREGATION
  Metrics are computed **per scenario** over that scenario's tick series, then
  reported two ways:
    * per-scenario then averaged ("mean-of-scenarios", with std + p50/p95) -- the
      headline figure; weights every scenario equally regardless of length.
    * "pooled" -- over all ticks of all scenarios concatenated *within* each
      scenario boundary (first-differences / sign-flips never cross the
      scenario seam). Pooled weights by tick count.
  The gate is read off the mean-of-scenarios row.

DATA CONTRACT (dmp-dump, pathgen/M1 mode -- see
specs/035-energy-lexicase-objective/contracts/dmp-dump-cli.md):
  CSV header (pathgen):
    scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,
    pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,dist,along,stpPt,mult,rampSc
  NN output channels consumed here: out_rl (roll), out_pt (pitch), out_th
  (throttle). `scenario` segments the per-tick series. Tracker-mode CSVs share
  the same out_* names, so this tool works on those too (extra columns ignored).

USAGE
  # historical t6 10 Hz baseline (the default reference this gate compares to):
  dmp-dump s3://autoc-m1/autoc-9223370256079660488-2026-06-06T19:45:15.319Z/ \
      --gen 800 --csv-only | python3 dealias_metrics.py - --label t6-10Hz

  # a candidate 20 Hz run, all axes, custom saturation knee:
  dmp-dump s3://autoc-m1/<run>/ --gen N --csv-only \
      | python3 dealias_metrics.py - --label 20Hz --axis all --sat-knee 0.95

  # local CSV (dev convenience):
  python3 dealias_metrics.py /tmp/t6.csv --label t6-10Hz

NOTE on the t6 reference: t6 = the 035 M1 energy-lexicase run, converged gen
800. dmp filenames encode (10000 - actual_gen) (reference_dmp_filename_gen_
encoding), so gen 800 = gen9200.dmp[.zst]. This script does NOT fetch from S3
itself -- pipe a `dmp-dump ... --csv-only` stream in, or pass a local CSV path
(configure via CLI flags, not env vars, per feedback_cli_over_env_vars).
"""

import argparse
import csv
import sys

import numpy as np

# NN output channel column -> human axis name. Outputs are tanh-bounded [-1,1].
AXES = {
    "roll": "out_rl",
    "pitch": "out_pt",
    "throttle": "out_th",
}
OUT_ABS_MAX = 1.0  # tanh saturation magnitude for every output channel


def load_series_by_scenario(src, cols):
    """Read a dmp-dump --csv-only stream/file. Return {axis_name: [np.array per
    scenario, ...]} preserving per-scenario tick order. `cols` is a dict
    {axis_name: csv_column}. Rows are grouped by the `scenario` column; within a
    scenario, ticks are kept in file order (dmp-dump emits them tick-ascending).
    """
    reader = csv.DictReader(src)
    field = reader.fieldnames or []
    if "scenario" not in field:
        sys.stderr.write(
            "error: CSV has no 'scenario' column -- is this a dmp-dump "
            f"--csv-only stream? (got {field})\n")
        sys.exit(1)
    missing = [c for c in cols.values() if c not in field]
    if missing:
        sys.stderr.write(
            f"error: CSV missing output column(s) {missing} (got {field})\n")
        sys.exit(1)

    # scenario -> axis -> list[float]
    buf = {}
    order = []  # scenario keys in first-seen order
    for row in reader:
        scn = row["scenario"]
        if scn not in buf:
            buf[scn] = {name: [] for name in cols}
            order.append(scn)
        for name, col in cols.items():
            v = row[col]
            try:
                buf[scn][name].append(float(v))
            except (TypeError, ValueError):
                buf[scn][name].append(np.nan)

    if not order:
        sys.stderr.write("error: no scenario rows parsed from CSV\n")
        sys.exit(1)

    out = {name: [] for name in cols}
    for scn in order:
        for name in cols:
            out[name].append(np.asarray(buf[scn][name], dtype=float))
    return out, len(order)


def _clean(x):
    """Drop NaNs, preserving order."""
    return x[~np.isnan(x)]


def lag1_autocorr(x):
    """Pearson autocorrelation at lag 1 of series x. NaN if <3 points or zero
    variance (a constant series has undefined correlation)."""
    x = _clean(x)
    if x.size < 3:
        return np.nan
    a, b = x[:-1], x[1:]
    am, bm = a.mean(), b.mean()
    da, db = a - am, b - bm
    denom = np.sqrt((da * da).sum() * (db * db).sum())
    if denom == 0.0:
        return np.nan  # constant (pinned) series -- no dither to measure
    return float((da * db).sum() / denom)


def sign_flip_rate(x):
    """Fraction of consecutive-tick pairs that change sign. Zero counts as no
    sign (a 0->+ transition is not a flip); only strict +/- reversals count.
    NaN if <2 valid points."""
    x = _clean(x)
    if x.size < 2:
        return np.nan
    s = np.sign(x)
    a, b = s[:-1], s[1:]
    flips = (a * b) < 0  # strictly opposite signs
    return float(flips.mean())


def saturation_frac(x, knee):
    """Fraction of ticks with |x| >= knee * OUT_ABS_MAX."""
    x = _clean(x)
    if x.size == 0:
        return np.nan
    return float((np.abs(x) >= knee * OUT_ABS_MAX).mean())


def dctrl(x):
    """Mean |dx| per tick (mean abs first-difference). NaN if <2 valid points."""
    x = _clean(x)
    if x.size < 2:
        return np.nan
    return float(np.mean(np.abs(np.diff(x))))


METRIC_FNS = {
    "lag1_autocorr": lag1_autocorr,
    "sign_flip": sign_flip_rate,   # fraction; printed as %
    "saturation": saturation_frac,  # fraction; printed as %
    "dctrl": dctrl,
}


def per_scenario_metrics(series_list, sat_knee):
    """Compute the 4 metrics per scenario. Returns {metric: np.array(values)}
    (one entry per scenario, NaNs for scenarios too short for that metric)."""
    out = {m: [] for m in METRIC_FNS}
    for x in series_list:
        out["lag1_autocorr"].append(lag1_autocorr(x))
        out["sign_flip"].append(sign_flip_rate(x))
        out["saturation"].append(saturation_frac(x, sat_knee))
        out["dctrl"].append(dctrl(x))
    return {m: np.asarray(v, dtype=float) for m, v in out.items()}


def pooled_metrics(series_list, sat_knee):
    """Tick-weighted pooled metrics: accumulate over all ticks of all scenarios,
    but first-differences / sign-flips / autocorr pairs never cross a scenario
    boundary (each scenario contributes its own consecutive-tick pairs)."""
    # lag-1 autocorr pooled: gather (a,b) lag pairs within each scenario.
    a_all, b_all, diff_all, flip_pairs, sat_hits, sat_n = [], [], [], [], 0, 0
    flip_n = 0
    for x in series_list:
        x = _clean(x)
        if x.size >= 1:
            sat_hits += int(np.sum(np.abs(x) >= sat_knee * OUT_ABS_MAX))
            sat_n += x.size
        if x.size >= 2:
            a_all.append(x[:-1])
            b_all.append(x[1:])
            diff_all.append(np.abs(np.diff(x)))
            s = np.sign(x)
            flip_pairs.append(((s[:-1] * s[1:]) < 0).astype(float))
            flip_n += x.size - 1
    res = {}
    if a_all:
        a = np.concatenate(a_all)
        b = np.concatenate(b_all)
        am, bm = a.mean(), b.mean()
        da, db = a - am, b - bm
        denom = np.sqrt((da * da).sum() * (db * db).sum())
        res["lag1_autocorr"] = float((da * db).sum() / denom) if denom else np.nan
        res["sign_flip"] = float(np.concatenate(flip_pairs).mean())
        res["dctrl"] = float(np.concatenate(diff_all).mean())
    else:
        res["lag1_autocorr"] = res["sign_flip"] = res["dctrl"] = np.nan
    res["saturation"] = float(sat_hits / sat_n) if sat_n else np.nan
    return res


def _nanstat(vals, fn):
    v = vals[~np.isnan(vals)]
    return fn(v) if v.size else np.nan


def report(label, n_scn, axis_results, sat_knee):
    """Print the per-axis metrics table to stdout."""
    print(f"=== {label}: 037 de-alias gate metrics  "
          f"({n_scn} scenarios, sat-knee={sat_knee:.2f}*max) ===")
    print(f"  output channels are tanh-bounded; max=|{OUT_ABS_MAX:.1f}|.  "
          "lag-1 autocorr <0 => bang-bang dither; >=0 => smoothed.\n")

    hdr = (f"  {'axis':9s} {'lag1_ac':>9s}  {'signflip%':>9s}  "
           f"{'sat%':>7s}  {'dctrl':>7s}   (mean-of-scenarios +/- std; "
           "[p50/p95])")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    for axis, (per, pooled) in axis_results.items():
        ac = per["lag1_autocorr"]
        sf = per["sign_flip"]
        sat = per["saturation"]
        dc = per["dctrl"]
        ac_m, ac_s = _nanstat(ac, np.mean), _nanstat(ac, np.std)
        sf_m = _nanstat(sf, np.mean)
        sf_p50, sf_p95 = _nanstat(sf, np.median), _nanstat(
            sf, lambda v: np.percentile(v, 95))
        sat_m = _nanstat(sat, np.mean)
        dc_m, dc_s = _nanstat(dc, np.mean), _nanstat(dc, np.std)
        print(f"  {axis:9s} "
              f"{ac_m:+9.3f}  {100*sf_m:8.1f}%  {100*sat_m:6.1f}%  "
              f"{dc_m:7.3f}")
        print(f"  {'':9s} {'+/-'+format(ac_s,'.3f'):>9s}  "
              f"{'['+format(100*sf_p50,'.0f')+'/'+format(100*sf_p95,'.0f')+']':>9s}  "
              f"{'':>7s}  {'+/-'+format(dc_s,'.3f'):>7s}")

    # Pooled (tick-weighted) line for cross-check / length-bias visibility.
    print("\n  pooled (tick-weighted, intra-scenario pairs only):")
    print(f"  {'axis':9s} {'lag1_ac':>9s}  {'signflip%':>9s}  "
          f"{'sat%':>7s}  {'dctrl':>7s}")
    for axis, (_per, pooled) in axis_results.items():
        print(f"  {axis:9s} {pooled['lag1_autocorr']:+9.3f}  "
              f"{100*pooled['sign_flip']:8.1f}%  "
              f"{100*pooled['saturation']:6.1f}%  {pooled['dctrl']:7.3f}")

    # Gate read-out (roll only).
    if "roll" in axis_results:
        per = axis_results["roll"][0]
        ac_m = _nanstat(per["lag1_autocorr"], np.mean)
        sf_m = 100 * _nanstat(per["sign_flip"], np.mean)
        print("\n  --- Phase-A gate (roll, vs historical t6 10 Hz) ---")
        print(f"    roll lag-1 autocorr = {ac_m:+.3f}  "
              f"(target: negative -> >=0;  t6 ~ -0.24)  "
              f"{'PASS' if ac_m >= 0 else 'below-0'}")
        print(f"    roll sign-flip      = {sf_m:.1f}%   "
              f"(target: ->=20 pts vs t6 56% => <=36%)")
        print("    NOTE: gate also requires tracking within noise of t6 -- "
              "check separately (crash %, scenario completion).")


def main():
    ap = argparse.ArgumentParser(
        description="037 de-alias gate metrics from a dmp-dump --csv-only stream.")
    ap.add_argument(
        "csv", nargs="?", default="-",
        help="dmp-dump --csv-only file, or '-' for stdin (default). "
             "For the t6 10 Hz baseline pipe: "
             "dmp-dump s3://autoc-m1/"
             "autoc-9223370256079660488-2026-06-06T19:45:15.319Z/ --gen 800 "
             "--csv-only | dealias_metrics.py -")
    ap.add_argument("--label", default="run",
                    help="label for the report header (e.g. t6-10Hz, 20Hz-cand)")
    ap.add_argument("--axis", choices=["roll", "pitch", "throttle", "all"],
                    default="all",
                    help="which axis to report (default all; roll is the gate)")
    ap.add_argument("--sat-knee", type=float, default=0.95,
                    help="saturation knee as a fraction of |max| (default 0.95)")
    args = ap.parse_args()

    if args.axis == "all":
        cols = dict(AXES)
        # roll first so the gate read-out and table lead with it
        cols = {k: AXES[k] for k in ("roll", "pitch", "throttle")}
    else:
        cols = {args.axis: AXES[args.axis]}

    fh = sys.stdin if args.csv == "-" else open(args.csv, newline="")
    try:
        series, n_scn = load_series_by_scenario(fh, cols)
    finally:
        if fh is not sys.stdin:
            fh.close()

    axis_results = {}
    for axis in cols:
        per = per_scenario_metrics(series[axis], args.sat_knee)
        pooled = pooled_metrics(series[axis], args.sat_knee)
        axis_results[axis] = (per, pooled)

    report(args.label, n_scn, axis_results, args.sat_knee)
    return 0


if __name__ == "__main__":
    sys.exit(main())
