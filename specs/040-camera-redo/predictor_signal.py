#!/usr/bin/env python3
"""Does the span/closure predictor head carry ANY information? (040, 2026-08-01)

New script rather than an edit to `src/analytics/predictor_analysis.py`, per the
project rule that new analysis is a new script against dmp-dump. That script
plots the picture; this one puts a number on it.

WHY THIS EXISTS — the metric that was being used is the wrong one.

`fitness_decomposition.cc computeSpanPredictionError` scores the head on
mean |predicted − realized_span|, and `predictor_analysis.py` plots the same.
That statistic conflates three different failures:

  1. OFFSET/SCALE error   — fixable downstream by rescaling, so not fatal
  2. correlation with the LEVEL of span
  3. correlation with the CHANGE in span

Only (3) matters. Span is a slow quantity: over a 150 ms horizon it moves
~0.0075 rad against a ~0.049 rad level, so "assume no change" (the PERSISTENCE
baseline) is already right to within 15%. A predictor that nails the level and
knows nothing about the change cannot beat persistence — it just reproduces a
constant. So the honest test is r(prediction, Δspan), NOT r(prediction, span).

Reports, per horizon:
  * r and r² against the LEVEL      — what the calibration scatter panel shows
  * r and r² against the CHANGE     — what actually has to be non-zero
  * mean |err| raw, after an IDEAL linear rescale, and for persistence
    (the rescaled column answers "is this merely a scaling bug?")

Usage:
    ./build/dmp-dump s3://autoc-m2/<run-id>/ --csv-only -i autoc-tracker.ini > tick.csv
    python3 specs/040-camera-redo/predictor_signal.py tick.csv [--tick-sec 0.05]
"""
import argparse
import csv
import math

# camera_projection.h kCepSentinelThreshold — a (t, t+h) pair counts only when
# BOTH ticks are CEP-visible, mirroring computeSpanPredictionError.
CEP_SENTINEL = 1.25


def corr(xs, ys):
    n = len(xs)
    if n < 10:
        return 0.0, 0.0, 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((v - mx) ** 2 for v in xs) / n)
    sy = math.sqrt(sum((v - my) ** 2 for v in ys) / n)
    if sx == 0 or sy == 0:
        return 0.0, sx, sy
    cov = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / n
    return cov / (sx * sy), sx, sy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="dmp-dump --csv-only per-tick CSV")
    ap.add_argument("--tick-sec", type=float, default=0.05)
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    scen = {}
    for r in rows:
        scen.setdefault(r["scenario"], []).append(r)

    def visible(r):
        return float(r["blC0"]) < CEP_SENTINEL and float(r["brC0"]) < CEP_SENTINEL

    print(f"scenarios={len(scen)}  ticks={len(rows):,}\n")
    print(f"{'horizon':>8} {'n':>8} {'r(level)':>9} {'r(Δ)':>8} {'r²(Δ)%':>8} "
          f"{'raw|e|':>8} {'rescaled':>9} {'persist':>9} {'resc/pers':>10}")

    for h, col, label in ((1, "spP1", "+50ms"), (2, "spP2", "+100ms"), (3, "spP3", "+150ms")):
        pred, level, delta, pers = [], [], [], []
        for ticks in scen.values():
            for i, r in enumerate(ticks):
                j = i + h
                if j >= len(ticks) or not visible(r) or not visible(ticks[j]):
                    continue
                p = float(r[col])
                now, then = float(r["spn0"]), float(ticks[j]["spn0"])
                pred.append(p); level.append(then); delta.append(then - now); pers.append(now)
        if len(pred) < 10:
            print(f"{label:>8} insufficient visible pairs")
            continue
        r_lvl, sp, _ = corr(pred, level)
        r_dlt, _, _ = corr(pred, delta)
        n = len(pred)
        mp, mr = sum(pred) / n, sum(level) / n
        cov = sum((pred[i] - mp) * (level[i] - mr) for i in range(n)) / n
        slope = cov / (sp * sp) if sp > 0 else 0.0
        icept = mr - slope * mp
        raw = sum(abs(pred[i] - level[i]) for i in range(n)) / n
        resc = sum(abs(icept + slope * pred[i] - level[i]) for i in range(n)) / n
        prs = sum(abs(pers[i] - level[i]) for i in range(n)) / n
        print(f"{label:>8} {n:>8,} {r_lvl:>+9.4f} {r_dlt:>+8.4f} {r_dlt**2*100:>8.2f} "
              f"{raw:>8.4f} {resc:>9.5f} {prs:>9.5f} {resc/prs:>9.2f}x")

    # The closure-rate output is explicitly a CHANGE predictor, so it has no
    # excuse — a level correlation cannot flatter it.
    pred, real = [], []
    for ticks in scen.values():
        for i, r in enumerate(ticks):
            if i + 1 >= len(ticks) or not visible(r) or not visible(ticks[i + 1]):
                continue
            pred.append(float(r["spdR"]))
            real.append((float(ticks[i + 1]["spn0"]) - float(r["spn0"])) / args.tick_sec)
    r_, _, _ = corr(pred, real)
    print(f"\nclosure rate: n={len(pred):,}  r(spdR, realized rate) = {r_:+.4f}  "
          f"r² = {r_**2*100:.2f}%")
    print("\nREAD IT: r(Δ) is the number that matters. r(level) can look healthy while")
    print("the head has merely learned the MEAN span, which persistence already knows.")


if __name__ == "__main__":
    main()
