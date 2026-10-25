#!/usr/bin/env python3
"""
gate_estimate.py — predict the 044 flight criteria (SC-001/002/005) for candidate genomes before flying (044 T051 prep).

Every genome is evaluated in ONE environment (tier 0, seed 20261018, EvalVariationScale 1.0) on the 044 plant. t6 g752 —
the genome flown on 2026-10-18 — was evaluated on the same plant, seed and scale during the honesty ladder, so its
sim numbers and its measured flight numbers give a per-metric sim→flight ratio. Prediction = candidate sim × (g752 flight
/ g752 sim). This is the 043 calibration idea (candidate_compare.py used g488); it assumes the sim's error on each
metric is a constant ratio across genomes on this plant — an assumption, not a measurement. Departed scenarios
(any body rate > 15 rad/s) excluded. Per scenario, then median over scenarios:
  roll 3–5 Hz RMS, pitch 3–5 Hz RMS, pitch 2–3 Hz RMS (deg/s); 1.5–5 Hz normal-load RMS (g); in-streak share (%).

Usage:
  gate_estimate.py --ref g752:tick.csv LABEL:tick.csv [LABEL:tick.csv ...]
"""
import argparse
import collections
import csv
import sys

import numpy as np

FS = 20.0
GYRO = 6.0      # kGyroScale_radps
ACC = 8.0       # kAccelScale_g
FLIGHT_G752 = {"roll35": 97.0, "pitch35": 56.0, "pitch23": 51.0, "az_osc": 1.88}   # 2026-10-18, flight-analysis-20261018.md
GATES = {"roll35": ("<", 82.0, "SC-001"), "pitch35": ("<", 48.0, "SC-002"), "pitch23": ("<=", 51.0, "SC-002"),
         "az_osc": ("<", 1.88, "SC-005")}


def band_rms(x, lo, hi, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < nseg:
        return np.nan
    w = np.hanning(nseg)
    P = np.mean([np.abs(np.fft.rfft(x[i:i + nseg] * w)) ** 2 for i in range(0, len(x) - nseg + 1, nseg // 2)], axis=0)
    f = np.fft.rfftfreq(nseg, 1 / FS)
    tot = P[f > 0.25].sum()
    return float(np.sqrt(np.mean(x ** 2) * P[(f >= lo) & (f < hi)].sum() / tot)) if tot > 0 else np.nan


def metrics(path):
    with open(path) as f:
        r = csv.reader(f); h = next(r); first, last = {}, {}
        for i, n in enumerate(h):
            first.setdefault(n, i); last[n] = i
        sc = collections.defaultdict(list)
        for row in r:
            if len(row) < len(h):
                continue
            sc[int(row[first["scenario"]])].append((float(row[last["gyrP"]]), float(row[last["gyrQ"]]),
                                                    float(row[last["gyrR"]]), float(row[last["acZ"]]),
                                                    float(row[first["stpPt"]])))
    out = collections.defaultdict(list); dep = 0
    for s, v in sc.items():
        A = np.array(v)
        if len(A) < 256:
            continue
        if np.max(np.abs(A[:, :3] * GYRO)) > 15:
            dep += 1; continue
        p, q = np.degrees(A[:, 0] * GYRO), np.degrees(A[:, 1] * GYRO)
        out["roll35"].append(band_rms(p, 3, 5)); out["pitch35"].append(band_rms(q, 3, 5))
        out["pitch23"].append(band_rms(q, 2, 3)); out["az_osc"].append(band_rms(A[:, 3] * ACC, 1.5, 5))
        out["streak"].append(100 * float(np.mean(A[:, 4] >= 0.5)))
    return {k: float(np.nanmedian(v)) for k, v in out.items()}, dep, len(out["roll35"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", required=True); ap.add_argument("cands", nargs="+")
    a = ap.parse_args()
    rl, rp = a.ref.split(":", 1)
    ref, rdep, rn = metrics(rp)
    ratio = {k: FLIGHT_G752[k] / ref[k] for k in FLIGHT_G752}
    keys = ["roll35", "pitch35", "pitch23", "az_osc", "streak"]
    print(f"{'genome':>10} {'n':>4} {'dep':>3} | " + " ".join(f"{k:>9}" for k in keys) + "   (sim, equal environment)")
    print(f"{rl:>10} {rn:4d} {rdep:3d} | " + " ".join(f"{ref[k]:9.2f}" for k in keys))
    rows = []
    for spec in a.cands:
        lab, path = spec.split(":", 1)
        m, dep, n = metrics(path)
        rows.append((lab, m))
        print(f"{lab:>10} {n:4d} {dep:3d} | " + " ".join(f"{m[k]:9.2f}" for k in keys))
    print(f"\nsim→flight ratio from {rl} (flight / sim): " + ", ".join(f"{k} {ratio[k]:.2f}" for k in FLIGHT_G752))
    print(f"\npredicted flight values  (gate)      " + " ".join(f"{k:>9}" for k in FLIGHT_G752))
    print(f"{'flight ' + rl:>36} " + " ".join(f"{FLIGHT_G752[k]:9.2f}" for k in FLIGHT_G752))
    print(f"{'gate':>36} " + " ".join(f"{GATES[k][0] + str(GATES[k][1]):>9}" for k in FLIGHT_G752))
    for lab, m in rows:
        pred = {k: m[k] * ratio[k] for k in FLIGHT_G752}
        ok = all((pred[k] < GATES[k][1]) if GATES[k][0] == "<" else (pred[k] <= GATES[k][1]) for k in FLIGHT_G752)
        print(f"{lab:>36} " + " ".join(f"{pred[k]:9.2f}" for k in FLIGHT_G752) + f"   {'all gates' if ok else 'MISS'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
