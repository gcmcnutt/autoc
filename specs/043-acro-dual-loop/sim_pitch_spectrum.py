#!/usr/bin/env python3
"""
sim_pitch_spectrum.py — the SIM side of the flight pitch-cycle comparison (043, 2026-10-12).

Reads a dmp-dump tick CSV (`dmp-dump <run> --gen N --csv-only -i <ini>`) and reports, per path,
what the flight docs report from the blackbox: pitch-rate RMS, where the pitch-rate energy sits
(1–2 / 2–3 / 3–5 / 5–10 Hz band shares at the 20 Hz tick rate), the dominant frequency, how much
of the time the pitch COMMAND rides its cap, command sign-flips per second, and the empirical
command→gyro lag in ticks. Roll and yaw RMS are printed too, because `rotRatio` is ‖ω‖ over all
axes and the sim's excess rotation turned out to be roll-led while the flight problem is pitch.

Units: the dmp `gyrP/gyrQ/gyrR` columns are the NN inputs = rad/s ÷ kGyroScale_radps (6.0,
include/autoc/nn/nn_inputs.h:71). This script multiplies back and reports deg/s. `out_pt` is the
NN pitch output in [−1, 1] (×maxRate 240 deg/s = the rate setpoint in ACRO).

First use (gen 488, the genome flown 2026-10-11): sim pitch RMS 61–96 deg/s broadband with no 2 Hz
tone and the command at the cap 0–1.3 % of ticks; the same genome in the air: 124–153 deg/s, 31–63 %
of the energy in 2–3 Hz, setpoint p5/p95 at the ±240 cap. See tempered-control-first-principles.md § 11.

Usage:
  sim_pitch_spectrum.py tick.csv [--label NAME] [--path3 randomA]
"""
import argparse
import csv
import sys
from collections import defaultdict

import numpy as np

GYRO_SCALE = 6.0      # kGyroScale_radps
FS = 20.0             # tick rate, Hz
NAMES = ["StraightAndLevel", "SpiralClimb", "HorizFigure8", "slot3", "HighPerchSplitS", "SeededRandomB"]


def welch(x, fs=FS, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < nseg:
        nseg = max(16, (len(x) // 2) * 2)
    w = np.hanning(nseg)
    segs = [x[i:i + nseg] * w for i in range(0, len(x) - nseg + 1, nseg // 2)]
    P = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    return np.fft.rfftfreq(nseg, 1 / fs), P


def band(fr, P, a, b):
    m = (fr >= a) & (fr < b)
    tot = P[fr > 0.25].sum()
    return 100.0 * P[m].sum() / tot if tot > 0 else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--label", default="run")
    ap.add_argument("--path3", default="slot3")
    ap.add_argument("--maxrate", type=float, default=240.0, help="deg/s per unit of out_pt (ACRO pitch rate cap)")
    a = ap.parse_args()
    names = list(NAMES); names[3] = a.path3

    with open(a.csv) as f:
        r = csv.reader(f); h = next(r)
        first, last = {}, {}
        for i, n in enumerate(h):
            first.setdefault(n, i); last[n] = i
        need = ["scenario", "out_pt", "gyrP", "gyrQ", "gyrR"]
        missing = [c for c in need if c not in first]
        if missing:
            sys.exit(f"CSV lacks {missing}")
        iS, iO = first["scenario"], first["out_pt"]
        iP, iQ, iR = last["gyrP"], last["gyrQ"], last["gyrR"]   # the NN-input copies sit last
        sc = defaultdict(list)
        for row in r:
            if len(row) < len(h):
                continue
            sc[int(row[iS])].append((float(row[iO]), float(row[iP]), float(row[iQ]), float(row[iR])))

    print(f"{a.label}: sim pitch cycle view — {len(sc)} scenarios, {sum(len(v) for v in sc.values())} ticks")
    print(f"{'path':16s} {'n':>3s} {'qRMS':>6s} {'pRMS':>6s} {'rRMS':>6s} | {'|cmd|':>5s} {'cap%':>5s} {'flip/s':>6s} {'lag':>3s} | "
          f"{'q 1-2':>5s} {'2-3':>5s} {'3-5':>5s} {'5-10':>5s} | {'peakHz':>6s}")
    for p in range(6):
        rows = [v for s, v in sc.items() if s // 49 == p]
        if not rows:
            continue
        qr, pr, rr, cm, cap, fl, lag, gb, pk = [], [], [], [], [], [], [], np.zeros(4), []
        for v in rows:
            A = np.array(v)
            if len(A) < 64:
                continue
            o = A[:, 0]
            P_, Q_, R_ = np.degrees(A[:, 1] * GYRO_SCALE), np.degrees(A[:, 2] * GYRO_SCALE), np.degrees(A[:, 3] * GYRO_SCALE)
            qr.append(np.sqrt(np.mean(Q_ ** 2))); pr.append(np.sqrt(np.mean(P_ ** 2))); rr.append(np.sqrt(np.mean(R_ ** 2)))
            cm.append(np.mean(np.abs(o)) * a.maxrate); cap.append(100 * np.mean(np.abs(o) > 0.9))
            fl.append(np.sum(np.sign(o[1:]) != np.sign(o[:-1])) / (len(o) / FS))
            oc, qc = o - o.mean(), Q_ - Q_.mean()
            best = max(((L, np.corrcoef(oc[:len(oc) - L], qc[L:])[0, 1]) for L in range(0, 8)), key=lambda t: t[1])
            lag.append(best[0])
            fr, Pw = welch(Q_)
            gb += [band(fr, Pw, 1, 2), band(fr, Pw, 2, 3), band(fr, Pw, 3, 5), band(fr, Pw, 5, 10.01)]
            pk.append(fr[np.argmax(Pw[1:]) + 1])
        n = len(qr); gb /= max(1, n)
        print(f"{names[p]:16s} {n:3d} {np.median(qr):6.1f} {np.median(pr):6.1f} {np.median(rr):6.1f} | "
              f"{np.median(cm):5.0f} {np.median(cap):5.1f} {np.median(fl):6.2f} {np.median(lag):3.0f} | "
              f"{gb[0]:5.1f} {gb[1]:5.1f} {gb[2]:5.1f} {gb[3]:5.1f} | {np.median(pk):6.2f}")
    print("units: RMS and |cmd| in deg/s; cap% = ticks with |out_pt| > 0.9; lag = ticks (50 ms) of best out_pt→gyrQ correlation;"
          " band shares = % of pitch-rate power above 0.25 Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
