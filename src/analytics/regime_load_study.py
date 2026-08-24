#!/usr/bin/env python3
"""041 Study A (T054-T057) — per-regime control aggressiveness and normal load.

Reads a per-tick CSV from `dmp-dump --csv-only` (optionally `--physics`) and
reports, PER REGIME {tracking, intercept, patrol} and PER AXIS {pitch, roll,
throttle}:

  * dCtrl   — mean |Δoutput| per tick (the bang-bang / aggressiveness detector)
  * <|u|>   — mean |output|          (the amplitude / saturation detector)
  * load    — normal load factor distribution AND ITS PEAK

⚠️ LOAD COMES FROM THE RECORDED NN INPUT COLUMN `acZ`, NOT FROM physicsTrace
(spec.md § Clarifications, 2026-08-10). `nz_g = -acZ x AccelScaleG` — the same
line flight uses, so sim and flight report one quantity. The physics trace is
capped at 175 ms per scenario (~1% of ticks) and is staying that way, which
makes it useless for a distribution and actively misleading for a PEAK.

⚠️ CONSEQUENCE, and it is the reason this script has two halves: `acZ` only
exists in runs baked AFTER 041 T039. The two pinned pre-break comparators
predate it, so for them this reports the CONTROL half only and says so. Every
041 load number is a single-run profile of the NEW M1, never an old-vs-new
delta — accepted by the operator 2026-08-10 ("old M1 is what it is").

Regime rule is IMPORTED from dynamics_progress, not restated: same thresholds,
same smoothing, so these numbers stay comparable with every prior report.

Usage:
  regime_load_study.py --csv <file.csv[.gz]> --label <name> [--out-dir DIR]
                       [--accel-scale-g 8.0]

Outputs (machine-readable first, per T055):
  <out-dir>/<label>-regime-axis.csv   per regime x axis: dCtrl, <|u|>, n
  <out-dir>/<label>-regime-load.csv   per regime: load mean/median/p95/PEAK, n
  <out-dir>/<label>-h2.csv            T056 correlations, with n and p
  <out-dir>/<label>-autocorr.csv      T057 load autocorrelation vs lag
  <out-dir>/<label>-summary.txt       human-readable, states exclusions
"""

import argparse
import csv
import gzip
import io
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# Single-sourced regime definition (T054): reuse dynamics_progress's rule rather
# than writing a second one, or the numbers stop being comparable with the
# reports that already exist.
from dynamics_progress import TRACK_THRESHOLD, CLOSE_SMOOTH_TICKS  # noqa: E402

AXES = [("out_pt", "pitch"), ("out_rl", "roll"), ("out_th", "throttle")]
REGIMES = ["tracking", "intercept", "patrol"]

# 20 Hz control cadence. T057 samples autocorrelation at the history lags; the
# shortest OBSERVABLE lag is one tick, so a between-tick structural peak is not
# visible here by construction. Stated rather than implied.
TICK_MSEC = 50.0
AUTOCORR_LAG_MSEC = [50, 100, 150, 200, 300, 500, 800]


def open_maybe_gz(path):
    if path.endswith(".gz"):
        return io.TextIOWrapper(gzip.open(path, "rb"), encoding="utf-8")
    return open(path, "r", encoding="utf-8")


def load_rows(path):
    with open_maybe_gz(path) as fh:
        return list(csv.DictReader(fh))


def fnum(row, key):
    """Parse a CSV cell, returning None for absent/blank/unparsable.

    Blank is MEANINGFUL here: dmp-dump emits empty physics fields when a tick
    has no joined trace row, and empty derived fields when a scenario has no
    path geometry. Treating those as 0.0 would silently invent data.
    """
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def classify_regimes(scn_rows):
    """{tracking, intercept, patrol} per tick — dynamics_progress's rule verbatim.

    tracking : stpPt >= TRACK_THRESHOLD (inside the objective's cone)
    intercept: not tracking, but closing (smoothed d(dist)/dt < 0)
    patrol   : neither
    """
    n = len(scn_rows)
    stp = np.array([fnum(r, "stpPt") if fnum(r, "stpPt") is not None else np.nan
                    for r in scn_rows])
    have_stp = not np.all(np.isnan(stp))
    tracking = (stp >= TRACK_THRESHOLD) if have_stp else np.zeros(n, dtype=bool)
    tracking = np.nan_to_num(tracking, nan=0.0).astype(bool)

    dist = distance_series(scn_rows)
    if dist is not None and len(dist) >= 2:
        dd = np.diff(dist, prepend=dist[0])
        k = min(CLOSE_SMOOTH_TICKS, len(dd))
        dd_s = np.convolve(dd, np.ones(k) / k, mode="same")
        closing = dd_s < 0.0
    else:
        closing = np.zeros(n, dtype=bool)

    return {
        "tracking": tracking,
        "intercept": (~tracking) & closing,
        "patrol": (~tracking) & (~closing),
    }, have_stp, (dist is not None)


def distance_series(scn_rows):
    """Chaser->target distance. `dist` directly (M1) or derived from tgX/Y/Z (M2)."""
    d = [fnum(r, "dist") for r in scn_rows]
    if all(x is not None for x in d):
        return np.array(d)
    out = []
    for r in scn_rows:
        p = [fnum(r, k) for k in ("px", "py", "pz")]
        t = [fnum(r, k) for k in ("tgX", "tgY", "tgZ")]
        if any(x is None for x in p + t):
            return None
        out.append(math.dist(p, t))
    return np.array(out) if out else None


def axis_metrics(scn_rows, mask):
    """dCtrl and <|u|> for the masked ticks, per axis.

    dCtrl is a FIRST DIFFERENCE, so it needs the preceding tick. A masked tick
    whose predecessor is outside the mask still uses that predecessor: the
    control change physically happened, and dropping it would bias a regime's
    aggressiveness low exactly at its boundaries — which is where regime changes
    make the biggest control moves.
    """
    out = {}
    for col, name in AXES:
        vals = np.array([fnum(r, col) if fnum(r, col) is not None else np.nan
                         for r in scn_rows])
        if np.all(np.isnan(vals)):
            out[name] = (np.nan, np.nan, 0)
            continue
        idx = np.where(mask)[0]
        if len(idx) == 0:
            out[name] = (np.nan, np.nan, 0)
            continue
        mag = np.nanmean(np.abs(vals[idx]))
        prev_idx = idx[idx > 0]
        dctrl = (np.nanmean(np.abs(vals[prev_idx] - vals[prev_idx - 1]))
                 if len(prev_idx) else np.nan)
        out[name] = (dctrl, mag, len(idx))
    return out


def pearson_with_p(x, y):
    """Pearson r plus a two-sided p-value, without scipy.

    p via the t-statistic t = r*sqrt((n-2)/(1-r^2)) and a normal approximation
    to the t distribution. That approximation is fine at the n here (thousands
    of ticks) and is stated so nobody reads more precision into it than it has.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    n = len(x)
    if n < 3 or np.std(x) == 0 or np.std(y) == 0:
        return float("nan"), float("nan"), n
    r = float(np.corrcoef(x, y)[0, 1])
    if abs(r) >= 1.0:
        return r, 0.0, n
    t = r * math.sqrt((n - 2) / (1 - r * r))
    p = math.erfc(abs(t) / math.sqrt(2.0))
    return r, p, n


def main():
    ap = argparse.ArgumentParser(description="041 Study A — per-regime aggressiveness + load")
    ap.add_argument("--csv", required=True, help="per-tick CSV from dmp-dump (.csv or .csv.gz)")
    ap.add_argument("--label", required=True, help="name for output files")
    ap.add_argument("--out-dir", default="specs/041-m2-depth/study-a")
    ap.add_argument("--accel-scale-g", type=float, default=8.0,
                    help="kAccelScale_g the run was baked with (nz_g = -acZ * this)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    rows = load_rows(args.csv)
    if not rows:
        print(f"regime_load_study: {args.csv} has no rows", file=sys.stderr)
        return 1

    # Group by scenario — regimes and first differences are per-trajectory.
    scenarios = {}
    for r in rows:
        scenarios.setdefault(r.get("scenario", "0"), []).append(r)

    have_load = any(fnum(r, "acZ") is not None for r in rows)

    per_axis = {rg: {n: [] for _, n in AXES} for rg in REGIMES}
    load = {rg: [] for rg in REGIMES}
    counts = {rg: 0 for rg in REGIMES}
    h2 = {rg: {"dctrl_bank": [], "throttle": [], "nz": []} for rg in REGIMES}
    nz_by_scn = []
    excluded_no_stppt = 0
    excluded_no_dist = 0

    for _, scn_rows in sorted(scenarios.items()):
        masks, have_stp, have_dist = classify_regimes(scn_rows)
        if not have_stp:
            excluded_no_stppt += len(scn_rows)
        if not have_dist:
            excluded_no_dist += len(scn_rows)

        nz = None
        if have_load:
            acz = np.array([fnum(r, "acZ") if fnum(r, "acZ") is not None else np.nan
                            for r in scn_rows])
            nz = -acz * args.accel_scale_g
            nz_by_scn.append(nz)

        for rg in REGIMES:
            m = masks[rg]
            counts[rg] += int(m.sum())
            am = axis_metrics(scn_rows, m)
            for _, name in AXES:
                dctrl, mag, n = am[name]
                if n:
                    per_axis[rg][name].append((dctrl, mag, n))
            if nz is not None and m.any():
                load[rg].extend(nz[m][~np.isnan(nz[m])].tolist())

            # T056 (H2): does bank-axis aggressiveness predict throttle level
            # and load, WITHIN a regime? Bank = pitch+roll grouped physically
            # (project_smoothness_axis_grouping), throttle kept separate.
            idx = np.where(m)[0]
            idx = idx[idx > 0]
            if len(idx):
                pt = np.array([fnum(r, "out_pt") for r in scn_rows], dtype=float)
                rl = np.array([fnum(r, "out_rl") for r in scn_rows], dtype=float)
                th = np.array([fnum(r, "out_th") for r in scn_rows], dtype=float)
                dbank = np.abs(pt[idx] - pt[idx - 1]) + np.abs(rl[idx] - rl[idx - 1])
                h2[rg]["dctrl_bank"].extend(dbank.tolist())
                h2[rg]["throttle"].extend(np.abs(th[idx]).tolist())
                h2[rg]["nz"].extend((nz[idx] if nz is not None
                                     else np.full(len(idx), np.nan)).tolist())

    def wmean(triples, which):
        """Sample-count-weighted mean, so long scenarios are not under-counted."""
        vals = [(v[which], v[2]) for v in triples if not math.isnan(v[which])]
        tot = sum(n for _, n in vals)
        return (sum(v * n for v, n in vals) / tot) if tot else float("nan")

    # ---- per regime x axis -------------------------------------------------
    p1 = os.path.join(args.out_dir, f"{args.label}-regime-axis.csv")
    with open(p1, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["regime", "axis", "dCtrl", "mean_abs_u", "ticks"])
        for rg in REGIMES:
            for _, name in AXES:
                t = per_axis[rg][name]
                w.writerow([rg, name, f"{wmean(t,0):.6f}", f"{wmean(t,1):.6f}",
                            sum(v[2] for v in t)])

    # ---- per regime load ---------------------------------------------------
    p2 = os.path.join(args.out_dir, f"{args.label}-regime-load.csv")
    with open(p2, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["regime", "n", "nz_mean", "nz_median", "nz_p95", "nz_peak", "nz_min"])
        for rg in REGIMES:
            v = np.array(load[rg], dtype=float)
            if have_load and len(v):
                w.writerow([rg, len(v), f"{v.mean():.4f}", f"{np.median(v):.4f}",
                            f"{np.percentile(v,95):.4f}", f"{v.max():.4f}", f"{v.min():.4f}"])
            else:
                w.writerow([rg, 0, "", "", "", "", ""])

    # ---- T056 H2 -----------------------------------------------------------
    p3 = os.path.join(args.out_dir, f"{args.label}-h2.csv")
    with open(p3, "w", newline="") as fh:
        w = csv.writer(fh)
        # ⚠️ r_squared is emitted BESIDE p_value deliberately. At n ~ 50k ticks a
        # correlation of r = 0.05 has p < 1e-20 and explains 0.25% of the
        # variance — "significant" and meaningless at once. Reporting p alone
        # would invite exactly the wrong reading, so the variance-explained
        # column travels with it and `verdict` states the effect-size call.
        w.writerow(["regime", "pair", "pearson_r", "r_squared", "p_value", "n", "verdict"])

        def verdict(r, p):
            if math.isnan(r):
                return "no data"
            if not (p < 0.05):
                return "no effect (not significant)"
            rsq = r * r
            if rsq < 0.01:
                return "negligible (<1% variance) despite tiny p"
            if rsq < 0.09:
                return "weak"
            return "material"

        for rg in REGIMES:
            r1, pv1, n1 = pearson_with_p(h2[rg]["dctrl_bank"], h2[rg]["throttle"])
            w.writerow([rg, "dCtrl_bank~|throttle|", f"{r1:.4f}", f"{r1*r1:.4f}",
                        f"{pv1:.3g}", n1, verdict(r1, pv1)])
            if have_load:
                r2, pv2, n2 = pearson_with_p(h2[rg]["dctrl_bank"], h2[rg]["nz"])
                w.writerow([rg, "dCtrl_bank~nz", f"{r2:.4f}", f"{r2*r2:.4f}",
                            f"{pv2:.3g}", n2, verdict(r2, pv2)])
            else:
                w.writerow([rg, "dCtrl_bank~nz", "", "", "", 0,
                            "no data (run predates T039 ACCEL_*)"])

    # ---- T057 load autocorrelation ----------------------------------------
    p4 = os.path.join(args.out_dir, f"{args.label}-autocorr.csv")
    with open(p4, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["lag_msec", "lag_ticks", "autocorr", "n_pairs"])
        if have_load:
            for lag_ms in AUTOCORR_LAG_MSEC:
                lag = int(round(lag_ms / TICK_MSEC))
                a, b = [], []
                for nz in nz_by_scn:
                    if len(nz) > lag:
                        a.extend(nz[:-lag].tolist())
                        b.extend(nz[lag:].tolist())
                r, _, n = pearson_with_p(a, b)
                w.writerow([lag_ms, lag, f"{r:.4f}", n])

    # ---- human summary -----------------------------------------------------
    p5 = os.path.join(args.out_dir, f"{args.label}-summary.txt")
    total = sum(counts.values())
    with open(p5, "w") as fh:
        fh.write(f"041 Study A — {args.label}\n")
        fh.write(f"source: {args.csv}\n")
        fh.write(f"scenarios: {len(scenarios)}   classified ticks: {total}\n")
        for rg in REGIMES:
            pct = 100.0 * counts[rg] / total if total else 0.0
            fh.write(f"  {rg:10s} {counts[rg]:7d} ticks ({pct:5.1f}%)\n")
        fh.write("\n")
        if have_load:
            fh.write(f"LOAD: present (from recorded acZ x {args.accel_scale_g} g)\n")
        else:
            fh.write(
                "LOAD: ABSENT — this run predates 041 T039, so it carries no\n"
                "  ACCEL_* columns and no load can be reported. This is the\n"
                "  CONTROL HALF ONLY (T058). Do NOT substitute physicsTrace:\n"
                "  it covers ~1% of ticks and would misstate a peak.\n")
        if excluded_no_stppt:
            fh.write(f"\nNOTE: {excluded_no_stppt} ticks had no stpPt — nothing was\n"
                     "  classed 'tracking' for them; they split intercept/patrol on\n"
                     "  distance alone (dynamics_progress's documented degradation).\n")
        if excluded_no_dist:
            fh.write(f"\nNOTE: {excluded_no_dist} ticks had no usable distance —\n"
                     "  'closing' could not be computed, so they fell to patrol.\n")
        fh.write(f"\nwrote: {p1}\n       {p2}\n       {p3}\n       {p4}\n")

    print(open(p5).read())
    return 0


if __name__ == "__main__":
    sys.exit(main())
