#!/usr/bin/env python3
"""038 US3 span/closure-predictor analysis (dmp-dump-based).

Answers the US3 thesis in one figure: (1) is the aux span/closure-predictor
head LEARNING, and (2) does prediction accuracy track with tracking depth?
All data comes straight from the honest per-tick recording — no C++ change:
`dmp-dump --csv-only` already emits, per tick, the realized CEP-gated
beacon-pair span (`spn0`), the predicted spans at +50/+100/+150 ms
(`spP1`/`spP2`/`spP3` = NN outputs[3..5]), the predicted closure rate
(`spdR` = output[6]), the two beacon CEPs (`blC0`/`brC0`), the time-since-
seen (`tSee`), and the in-cone tracking step score (`stpPt`).

The prediction error mirrors src/eval/fitness_decomposition.cc
computeSpanPredictionError EXACTLY: a (t, t+h) pair counts only when BOTH
ticks are CEP-visible (both beacon CEPs < kCepSentinelThreshold = 1.25);
error is |predicted_span_h[t] - realized_span[t+h]| plus a closure-rate term
|spdR[t] - (spn0[t+1]-spn0[t])/dt|.

Panels:
  (0,0) per-horizon mean |error| vs generation   — the "is it learning" plot
  (0,1) closure-rate mean |error| vs generation  + visible-pair count (twin)
  (0,2) error vs lookahead horizon (latest gen)  — accuracy-vs-anticipation
  (1,0) predicted vs realized span @+150 ms scatter (latest gen) — calibration
  (1,1) predicted vs realized closure-rate scatter (latest gen)  — calibration
  (1,2) prediction error in-streak vs out-of-streak (latest gen) — does
        accuracy correlate with tracking depth (stpPt >= FitStreakThreshold)

Each generation costs one `dmp-dump --gen N --csv-only` S3 fetch, so sample
with --stride and use --cache (mirrors dynamics_progress.py's incremental
workflow). The latest-gen detail panels read the already-materialized
per-tick CSV via --tick when given (no extra fetch), else fetch the newest
gen once.

Usage:
    python3 predictor_analysis.py --run s3://autoc-m2/<run-id>/ \
        --gens 1-30 --stride 5 -i autoc-tracker.ini \
        --cache /tmp/t6_predictor_cache.csv --tick /tmp/tick.csv \
        --tick-sec 0.05 --label autoc-038-t6-m2-predictor -o out.png

Stdlib csv + numpy + matplotlib.
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

# Perception gate: a tick is CEP-visible when BOTH beacon CEPs are below the
# sentinel (the fitness gate fires if EITHER is >= sentinel). Matches
# kCepSentinelThreshold from include/autoc/eval/camera_projection.h.
CEP_SENTINEL = 1.25

# Aux-head lookahead horizons in ms (== kSpanPredictHorizonsMsec in
# include/autoc/nn/nn_inputs.h) paired with their predicted-span CSV column.
HORIZONS_MS = [50, 100, 150]
HORIZON_COL = {50: "spP1", 100: "spP2", 150: "spP3"}
HORIZON_COLOR = {50: "tab:green", 100: "tab:orange", 150: "tab:red"}

# stpPt >= this is "in the streak/tracking band" (FitStreakThreshold, 038 P0-F
# reverted to 0.5). Used only to split the latest-gen error by tracking regime.
TRACK_THRESHOLD = 0.5

# persist{ms} = the PERSISTENCE baseline |span(t) − span(t+h)| — the "predict
# no change" bar. A predictor that can't beat persistence provides no signal.
CACHE_FIELDS = ["gen", "err50", "err100", "err150", "err_rate",
                "persist50", "persist100", "persist150",
                "n_span_pairs", "n_rate_pairs", "span_scale"]


def per_scenario_series(rows):
    out = {}
    for r in rows:
        out.setdefault(int(r["scenario"]), []).append(r)
    return out


def _col(scn_rows, name):
    return np.array([float(r[name]) for r in scn_rows])


def _visible(scn_rows):
    """Per-tick CEP visibility, matching the fitness gate exactly."""
    bl = _col(scn_rows, "blC0")
    br = _col(scn_rows, "brC0")
    return (bl < CEP_SENTINEL) & (br < CEP_SENTINEL)


def _horizon_ticks(tick_sec):
    """ms horizons -> integer tick offsets at this cadence (50 ms -> 1 tick)."""
    return {ms: int(round((ms / 1000.0) / tick_sec)) for ms in HORIZONS_MS}


def gather_pairs(rows, tick_sec):
    """Pull every valid (predicted, realized) pair from a gen's per-tick CSV.

    Returns dict of numpy arrays pooled across scenarios:
      span_pred[ms], span_real[ms]   — predicted vs realized span per horizon
      rate_pred, rate_real           — predicted vs realized closure rate
      abs_err[ms]                    — |pred - real| per horizon (per pair)
      err_tsee, err_stp              — the +150 ms abs-err aligned with the
                                       tSee and stpPt at the prediction tick
                                       (for the regime/latency detail panels)
    """
    htk = _horizon_ticks(tick_sec)
    by_scn = per_scenario_series(rows)
    span_pred = {ms: [] for ms in HORIZONS_MS}
    span_real = {ms: [] for ms in HORIZONS_MS}
    span_now = {ms: [] for ms in HORIZONS_MS}   # span(t) per pair — persistence baseline
    rate_pred, rate_real = [], []
    err_tsee, err_stp, err150 = [], [], []
    span_vals = []

    for scn in by_scn.values():
        vis = _visible(scn)
        spn0 = _col(scn, "spn0")
        spdR = _col(scn, "spdR")
        tsee = _col(scn, "tSee") if "tSee" in scn[0] else np.zeros(len(scn))
        stp = _col(scn, "stpPt") if "stpPt" in scn[0] else np.full(len(scn), np.nan)
        pred = {ms: _col(scn, HORIZON_COL[ms]) for ms in HORIZONS_MS}
        n = len(scn)
        span_vals.extend(spn0[vis].tolist())

        for t in range(n):
            if not vis[t]:
                continue
            for ms in HORIZONS_MS:
                ta = t + htk[ms]
                if ta >= n or not vis[ta]:
                    continue
                span_pred[ms].append(pred[ms][t])
                span_real[ms].append(spn0[ta])
                span_now[ms].append(spn0[t])
                if ms == 150:
                    err150.append(abs(pred[ms][t] - spn0[ta]))
                    err_tsee.append(tsee[t])
                    err_stp.append(stp[t])
            # closure rate: realized (spn0[t+1]-spn0[t])/dt when t+1 visible.
            if t + 1 < n and vis[t + 1]:
                rate_pred.append(spdR[t])
                rate_real.append((spn0[t + 1] - spn0[t]) / tick_sec)

    out = {"span_pred": {ms: np.array(span_pred[ms]) for ms in HORIZONS_MS},
           "span_real": {ms: np.array(span_real[ms]) for ms in HORIZONS_MS},
           "span_now": {ms: np.array(span_now[ms]) for ms in HORIZONS_MS},
           "rate_pred": np.array(rate_pred), "rate_real": np.array(rate_real),
           "err150": np.array(err150), "err_tsee": np.array(err_tsee),
           "err_stp": np.array(err_stp),
           "span_scale": float(np.std(span_vals)) if span_vals else float("nan")}
    return out


def compute_gen_metrics(csv_text, tick_sec):
    """Per-gen aggregate: mean |error| per horizon + closure, and pair counts."""
    rows = list(csv.DictReader(io.StringIO(csv_text)))
    need = {"scenario", "spn0", "spP1", "spP2", "spP3", "spdR", "blC0", "brC0"}
    if not rows or not need.issubset(rows[0].keys()):
        raise ValueError("CSV missing predictor columns (need a tracker/M2 dump "
                         "with spn0/spP1..3/spdR/blC0/brC0)")
    p = gather_pairs(rows, tick_sec)
    rec = {}
    for ms in HORIZONS_MS:
        e = np.abs(p["span_pred"][ms] - p["span_real"][ms])
        rec[f"err{ms}"] = float(e.mean()) if e.size else float("nan")
        pe = np.abs(p["span_now"][ms] - p["span_real"][ms])
        rec[f"persist{ms}"] = float(pe.mean()) if pe.size else float("nan")
    rate_err = np.abs(p["rate_pred"] - p["rate_real"])
    rec["err_rate"] = float(rate_err.mean()) if rate_err.size else float("nan")
    rec["n_span_pairs"] = float(p["span_pred"][150].size)
    rec["n_rate_pairs"] = float(p["rate_pred"].size)
    rec["span_scale"] = p["span_scale"]
    return rec


def fetch_gen_csv(dmp_dump, run, gen, ini):
    cmd = [dmp_dump, run, "--gen", str(gen), "--csv-only", "-i", ini]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        tail = res.stderr.strip().splitlines()[-1:] or [""]
        raise RuntimeError(f"dmp-dump --gen {gen} failed: {tail[0]}")
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
    if not gens or gens[-1] != hi:
        gens.append(hi)  # always include the newest gen
    return gens


def _identity_line(ax, a, b):
    """Draw a y=x reference spanning the pooled data range."""
    if a.size == 0 and b.size == 0:
        return
    lo = float(min(a.min() if a.size else 0.0, b.min() if b.size else 0.0))
    hi = float(max(a.max() if a.size else 0.0, b.max() if b.size else 0.0))
    ax.plot([lo, hi], [lo, hi], color="0.5", ls="--", lw=1, label="y = x")


def _note_empty(ax, msg="no CEP-visible\nprediction pairs yet"):
    ax.text(0.5, 0.5, msg, ha="center", va="center", transform=ax.transAxes,
            color="0.5", fontsize=11)


def plot(data, detail, label, tick_sec, out):
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(f"{label} — 038 US3 span/closure-predictor analysis "
                 f"(error in NDC; lower = better)", fontsize=13)

    gens = sorted(data)
    ga = np.array(gens, dtype=float)

    # (0,0) per-horizon error over generations.
    ax = axs[0, 0]
    if gens:
        for ms in HORIZONS_MS:
            y = np.array([data[g].get(f"err{ms}", np.nan) for g in gens])
            ax.plot(ga, y, color=HORIZON_COLOR[ms], marker=".", ms=4,
                    label=f"+{ms} ms")
            pb = np.array([data[g].get(f"persist{ms}", np.nan) for g in gens])
            ax.plot(ga, pb, color=HORIZON_COLOR[ms], ls="--", lw=1, alpha=0.6,
                    label=f"persistence +{ms} ms (no-change bar)")
        scale = np.array([data[g].get("span_scale", np.nan) for g in gens])
        ax.plot(ga, scale, color="0.6", ls=":", lw=1,
                label="realized span σ (error floor ref)")
    ax.set_title("per-horizon prediction error vs generation")
    ax.set_xlabel("generation"); ax.set_ylabel("mean |pred − realized| (NDC)")
    ax.set_ylim(bottom=0); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    # (0,1) closure-rate error over generations + visible-pair count.
    ax = axs[0, 1]
    if gens:
        y = np.array([data[g].get("err_rate", np.nan) for g in gens])
        ax.plot(ga, y, color="tab:purple", marker=".", ms=4,
                label="closure-rate error")
        ax.set_ylim(bottom=0)
        axn = ax.twinx()
        npr = np.array([data[g].get("n_span_pairs", np.nan) for g in gens])
        axn.plot(ga, npr, color="0.7", ls="--", lw=1, label="visible pairs")
        axn.set_ylabel("visible (t,t+h) pairs", color="0.5")
        axn.tick_params(axis="y", labelcolor="0.5")
    ax.set_title("closure-rate error + visibility vs generation")
    ax.set_xlabel("generation")
    ax.set_ylabel("mean |pred − realized| (NDC/s)", color="tab:purple")
    ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper right")

    # (0,2) error vs lookahead horizon (latest gen).
    ax = axs[0, 2]
    dgen = detail.get("gen")
    if detail.get("pairs"):
        p = detail["pairs"]
        means, errs, persists = [], [], []
        for ms in HORIZONS_MS:
            e = np.abs(p["span_pred"][ms] - p["span_real"][ms])
            means.append(e.mean() if e.size else np.nan)
            errs.append(e.std() if e.size else 0.0)
            pe = np.abs(p["span_now"][ms] - p["span_real"][ms])
            persists.append(pe.mean() if pe.size else np.nan)
        ax.bar([str(m) for m in HORIZONS_MS], means, yerr=errs, capsize=4,
               color=[HORIZON_COLOR[m] for m in HORIZONS_MS], alpha=0.85)
        ax.plot([str(m) for m in HORIZONS_MS], persists, color="k", ls="none",
                marker="D", ms=7, label="persistence (no-change bar)")
        ax.legend(fontsize=8)
    else:
        _note_empty(ax)
    ax.set_title(f"error vs horizon (gen {dgen})" if dgen else "error vs horizon")
    ax.set_xlabel("lookahead (ms)"); ax.set_ylabel("mean |error| (NDC)")
    ax.set_ylim(bottom=0); ax.grid(alpha=0.3, axis="y")

    # (1,0) predicted vs realized span @ +150 ms (latest gen).
    ax = axs[1, 0]
    if detail.get("pairs") and detail["pairs"]["span_pred"][150].size:
        pr = detail["pairs"]["span_pred"][150]
        re = detail["pairs"]["span_real"][150]
        ax.scatter(re, pr, s=6, alpha=0.25, color="tab:red")
        _identity_line(ax, re, pr)
        ax.legend(fontsize=8)
    else:
        _note_empty(ax)
    ax.set_title(f"span calibration @ +150 ms (gen {dgen})")
    ax.set_xlabel("realized span (NDC)"); ax.set_ylabel("predicted span (NDC)")
    ax.grid(alpha=0.3)

    # (1,1) predicted vs realized closure rate (latest gen).
    ax = axs[1, 1]
    if detail.get("pairs") and detail["pairs"]["rate_pred"].size:
        pr = detail["pairs"]["rate_pred"]
        re = detail["pairs"]["rate_real"]
        ax.scatter(re, pr, s=6, alpha=0.25, color="tab:purple")
        _identity_line(ax, re, pr)
        ax.legend(fontsize=8)
    else:
        _note_empty(ax)
    ax.set_title(f"closure-rate calibration (gen {dgen})")
    ax.set_xlabel("realized rate (NDC/s)"); ax.set_ylabel("predicted rate (NDC/s)")
    ax.grid(alpha=0.3)

    # (1,2) +150 ms error, in-streak vs out-of-streak (latest gen).
    ax = axs[1, 2]
    if detail.get("pairs") and detail["pairs"]["err150"].size:
        e = detail["pairs"]["err150"]
        stp = detail["pairs"]["err_stp"]
        ok = ~np.isnan(stp)
        ins = e[ok & (stp >= TRACK_THRESHOLD)]
        out_ = e[ok & (stp < TRACK_THRESHOLD)]
        labels, vals, colors = [], [], []
        if out_.size:
            labels.append(f"out-of-streak\n(n={out_.size})"); vals.append(out_.mean()); colors.append("tab:gray")
        if ins.size:
            labels.append(f"in-streak\n(n={ins.size})"); vals.append(ins.mean()); colors.append("tab:green")
        if vals:
            ax.bar(labels, vals, color=colors, alpha=0.85)
        else:
            _note_empty(ax, "no stpPt-labelled pairs")
    else:
        _note_empty(ax)
    ax.set_title(f"+150 ms error by tracking regime (gen {dgen})")
    ax.set_ylabel("mean |error| (NDC)"); ax.set_ylim(bottom=0)
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", required=True, help="s3://<bucket>/<run-id>/")
    p.add_argument("--gens", required=True, help="LO-HI generation range (actualGen)")
    p.add_argument("--stride", type=int, default=5)
    p.add_argument("--dmp-dump", default="./build/dmp-dump")
    p.add_argument("-i", "--config", default="autoc-tracker.ini")
    p.add_argument("--cache", default=None, help="CSV cache of per-gen rows")
    p.add_argument("--tick", default=None,
                   help="already-materialized current-gen per-tick CSV for the "
                        "detail panels (skips one fetch)")
    p.add_argument("--tick-sec", type=float, default=0.05,
                   help="seconds per control tick (cadence); 0.05 = 20 Hz")
    p.add_argument("--label", default="run")
    p.add_argument("-o", "--out", required=True)
    args = p.parse_args()

    data = load_cache(args.cache)
    gens = parse_gens(args.gens, args.stride)
    todo = [g for g in gens if g not in data]
    print(f"{len(gens)} gens requested, {len(todo)} to fetch ({len(data)} cached)")
    for i, gen in enumerate(todo):
        try:
            rec = compute_gen_metrics(
                fetch_gen_csv(args.dmp_dump, args.run, gen, args.config),
                args.tick_sec)
        except (RuntimeError, ValueError) as e:
            print(f"  gen {gen}: SKIP ({e})", file=sys.stderr)
            continue
        data[gen] = rec
        save_cache(args.cache, data)  # persist as we go — interruptible
        print(f"  [{i+1}/{len(todo)}] gen {gen}: "
              f"err150={rec['err150']:.4f} rate={rec['err_rate']:.4f} "
              f"pairs={int(rec['n_span_pairs'])}")

    # Detail panels use the newest gen's raw per-tick CSV: the caller's
    # materialized --tick when given (no extra fetch), else fetch it once.
    detail = {"gen": max(gens) if gens else None, "pairs": None}
    try:
        if args.tick and os.path.isfile(args.tick):
            with open(args.tick) as f:
                rows = list(csv.DictReader(f))
        else:
            rows = list(csv.DictReader(io.StringIO(
                fetch_gen_csv(args.dmp_dump, args.run, max(gens), args.config))))
        need = {"spn0", "spP1", "spP2", "spP3", "spdR", "blC0", "brC0"}
        if rows and need.issubset(rows[0].keys()):
            detail["pairs"] = gather_pairs(rows, args.tick_sec)
    except (RuntimeError, ValueError, OSError) as e:
        print(f"  detail panels: SKIP ({e})", file=sys.stderr)

    plot(data, detail, args.label, args.tick_sec, args.out)


if __name__ == "__main__":
    main()
