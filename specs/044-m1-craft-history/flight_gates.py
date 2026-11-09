#!/usr/bin/env python3
"""
flight_gates.py — the 044 flight criteria (SC-001/002/003/005) from xiao tick CSVs (044 T054).

Same Welch band share as gate_estimate.py / candidate_compare.py (20 Hz ticks, 128-sample Hann, share of the power above
0.25 Hz), times the span's RMS, per engaged span, then the median over spans. ⚠️ Two forms of "band RMS", because the 10-18
reference numbers mix them: the rate bands (97 / 56 / 51 °/s) reproduce with the RAW rate RMS (a turn's steady pitch rate
included — the 043 candidate_compare form; key suffix none), the load band (1.88 g) only with the MEAN-REMOVED RMS (else 1 g of
gravity counts as oscillation; suffix _m). Both are printed. The gate verdict is taken two ways: against the spec's absolute
lines, and — robust to the form — against the reference flight computed by THIS code (≥ 15 % below, the spec's own rule). Signals are the policy's own inputs: gyro p/q (post-scale
rad/s ÷ 6 → deg/s) and accel z (g ÷ 8). Also per span: rate RMS, band shares, command sign flips/s and % of ticks at the cap
(|out| > 0.9) per axis, |a_z| p99, median distance to the rabbit.

Usage: flight_gates.py REFLABEL:xiao.csv[,xiao2.csv] LABEL:xiao.csv [...] [--drop FILE.csv#SPAN ...]   (first set = reference;
       comma-joined CSVs pool spans, e.g. two flights of one day; --drop excludes a span by the name the table prints, e.g. a span
       flown on a faulted nav estimate — 11-08 span 1, flight-analysis-20261108.md § 6)
"""
import argparse
import collections
import csv
import math

import numpy as np

FS = 20.0
GYRO = 6.0
ACC = 8.0
# key, op, absolute line, criterion, required ratio to the same-code reference (None = absolute only)
GATES = (("roll35", "<", 82.0, "SC-001", 0.85), ("pitch35", "<", 48.0, "SC-002", 0.85),
         ("pitch23", "<=", 51.0, "SC-002", 1.0), ("az_osc_m", "<", 1.88, "SC-005", 1.0))


def band_rms(x, lo, hi, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < nseg:
        return np.nan
    w = np.hanning(nseg)
    P = np.mean([np.abs(np.fft.rfft(x[i:i + nseg] * w)) ** 2 for i in range(0, len(x) - nseg + 1, nseg // 2)], axis=0)
    f = np.fft.rfftfreq(nseg, 1 / FS)
    tot = P[f > 0.25].sum()
    return float(np.sqrt(np.mean(x ** 2) * P[(f >= lo) & (f < hi)].sum() / tot)) if tot > 0 else np.nan


def share(x, lo, hi):
    r = band_rms(x, lo, hi)
    tot = math.sqrt(np.mean((np.asarray(x) - np.mean(x)) ** 2))
    return 100 * (r / tot) ** 2 if tot > 0 else np.nan


def band_rms_raw(x, lo, hi):
    """The 043 form: raw RMS (mean included) times the square root of the band share."""
    return float(math.sqrt(np.mean(np.asarray(x, float) ** 2) * share(x, lo, hi) / 100))


def load_spans(paths, drop=()):
    spans = []
    for path in paths:
        by = collections.OrderedDict()
        with open(path) as f:
            for r in csv.DictReader(f):
                by.setdefault(r["span_id"], []).append(r)
        for sid, rows in by.items():
            name = f"{path.rsplit('/', 1)[-1]}#{sid}"
            if name not in drop:
                spans.append((name, rows))
    return spans


def span_metrics(rows):
    g = lambda k: np.array([float(r[k]) for r in rows])
    p, q = np.degrees(g("gyro_p") * GYRO), np.degrees(g("gyro_q") * GYRO)
    az = g("accel_z") * ACC
    up, ur, ut = g("out_pitch"), g("out_roll"), g("out_throttle")
    dist = np.sqrt((g("pos_n") - g("rabbit_n")) ** 2 + (g("pos_e") - g("rabbit_e")) ** 2 + (g("pos_d") - g("rabbit_d")) ** 2)
    dur = len(rows) / FS
    flips = lambda u: float(np.sum(np.sign(u[1:]) != np.sign(u[:-1])) / dur)
    cap = lambda u: 100 * float(np.mean(np.abs(u) > 0.9))
    return dict(
        ticks=len(rows), path=rows[0]["path_index"],
        roll35=band_rms_raw(p, 3, 5), pitch35=band_rms_raw(q, 3, 5), pitch23=band_rms_raw(q, 2, 3),
        roll35_m=band_rms(p, 3, 5), pitch35_m=band_rms(q, 3, 5), pitch23_m=band_rms(q, 2, 3), az_osc_m=band_rms(az, 1.5, 5),
        az_osc=band_rms_raw(az, 1.5, 5),
        prms=float(np.sqrt(np.mean(p ** 2))), qrms=float(np.sqrt(np.mean(q ** 2))),
        p_s35=share(p, 3, 5), q_s23=share(q, 2, 3), q_s35=share(q, 3, 5),
        flips_pt=flips(up), flips_rl=flips(ur), flips_th=flips(ut), cap_pt=cap(up), cap_rl=cap(ur), cap_th=cap(ut),
        az99=float(np.percentile(np.abs(az), 99)), dist=float(np.median(dist)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("sets", nargs="+", metavar="LABEL:csv[,csv]")
    ap.add_argument("--drop", action="append", default=[], metavar="FILE.csv#SPAN", help="exclude this span (repeatable)")
    args = ap.parse_args()
    cols = ("ticks", "path", "roll35", "pitch35", "pitch23", "roll35_m", "pitch35_m", "pitch23_m", "az_osc_m", "prms", "qrms", "p_s35", "q_s23", "q_s35",
            "flips_rl", "flips_pt", "flips_th", "cap_rl", "cap_pt", "cap_th", "az99", "dist")
    meds = {}
    for spec in args.sets:
        label, paths = spec.split(":", 1)
        spans = [(n, span_metrics(r)) for n, r in load_spans(paths.split(","), args.drop)]
        print(f"\n== {label}")
        print(f"{'span':34s}" + "".join(f"{c:>9s}" for c in cols))
        for n, m in spans:
            print(f"{n:34s}" + "".join(f"{m[c]:9.0f}" if c in ("ticks",) else (f"{m[c]:>9s}" if c == "path" else f"{m[c]:9.2f}")
                                       for c in cols))
        med = {c: float(np.nanmedian([m[c] for _, m in spans])) for c in cols if c != "path"}
        meds[label] = med
        print(f"{'median':34s}" + "".join(f"{med[c]:9.0f}" if c == "ticks" else ("" .rjust(9) if c == "path" else f"{med[c]:9.2f}")
                                         for c in cols))
    labels = list(meds)
    ref = labels[0]
    print(f"\n== gates (median over spans; reference = {ref})")
    for key, op, lim, sc, ratio in GATES:
        for k in (key,) if key.endswith("_m") else (key, key + "_m"):
            r = meds[ref][k]
            cells = []
            for label in labels[1:]:
                v = meds[label][k]
                ok_abs = v < lim if op == "<" else v <= lim
                rel = v / r
                ok_rel = rel < ratio if ratio < 1 else rel <= ratio
                cells.append(f"{label} {v:6.2f} (abs {'PASS' if ok_abs else 'MISS'}; {100 * (rel - 1):+5.1f} % vs ref "
                             f"{'PASS' if ok_rel else 'MISS'})")
            need = f"<= {100 * (ratio - 1):+.0f} %" if ratio < 1 else "no rise"
            print(f"  {sc} {k:10s} {op} {lim:<5} | ref {r:6.2f}, need {need:8s} | " + "   ".join(cells))
    for label, med in meds.items():
        print(f"  SC-003 median distance to rabbit {label}: {med['dist']:.1f} m (reference 9-14 m)")


if __name__ == "__main__":
    main()
