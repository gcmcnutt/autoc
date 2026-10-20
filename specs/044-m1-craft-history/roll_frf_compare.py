#!/usr/bin/env python3
"""
roll_frf_compare.py — closed-inner-loop ROLL frequency response, flight vs sim plants (044 T023, 2026-10-19).

The roll twin of ../043-acro-dual-loop/frf_pitch_compare.py, for the 044 honesty ladder rung 2 (research R11):
H(f) = roll rate / roll-rate setpoint — INAV's rate loop + servo + airframe as one block, estimated from the
policy's own excitation (cross-spectrum / auto-spectrum, Hann/Welch, 50 % overlap), coherence per bin.
**Recorded, not gated**: it shows where each sim plant sits between the t6 plant and the aircraft.

  flight : blackbox CSV (`blackbox_decode --stdout <log.TXT>`); `axisRate[0]` roll setpoint, `gyroADC[0]`
           filtered roll gyro, deg/s; engaged spans = `mspOverrideFlags == 2` longer than 5 s.
  sim    : one or more dmp-dump tick CSVs (20 Hz), each `LABEL:FILE` (e.g. t6:g752-t6.csv new:g752-044.csv);
           `rollCmd` × the sim roll maxRate (360 deg/s, hb1_streamer.xml Cntrl_InavFwRate) is the setpoint,
           `gyrP` × 6 rad/s the body roll rate; medians over scenarios. Scenarios that departed (any |p|, |q|
           or |r| above 15 rad/s) are excluded, as 043's tools do. Nyquist 10 Hz: ignore > 8 Hz.
  model  : first-order roll mode, corner f = |L_p| / 2π with L_p = q̄ S b² Cl_p / (2 V I_xx), for a list of
           Cl_p values (the 044 class centre and clamps by default) at V = 15 m/s.

Usage:
  roll_frf_compare.py [--flight bb.csv] [--sim LABEL:tick.csv ...] [--clp -0.36 --clp -0.60 ...]
"""
import argparse
import collections
import csv
import math
import sys

import numpy as np

FREQS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0]
RHO, S_REF, SPAN, IXX = 1.225, 0.136, 0.762, 0.0069   # hb1_streamer.xml <ref>, measured I_xx (provisional)
GYRO_SCALE = 6.0
DEPARTED_RADPS = 15.0


def frf(x, y, fs, nseg):
    x = np.asarray(x, float) - np.mean(x); y = np.asarray(y, float) - np.mean(y)
    w = np.hanning(nseg); Pxx = Pxy = Pyy = 0.0
    for i in range(0, len(x) - nseg + 1, nseg // 2):
        X = np.fft.rfft(x[i:i + nseg] * w); Y = np.fft.rfft(y[i:i + nseg] * w)
        Pxx = Pxx + np.abs(X) ** 2; Pyy = Pyy + np.abs(Y) ** 2; Pxy = Pxy + np.conj(X) * Y
    fr = np.fft.rfftfreq(nseg, 1 / fs)
    return fr, Pxy / Pxx, np.abs(Pxy) ** 2 / (Pxx * Pyy)


def fmt(fr, H, coh, freqs):
    out = []
    for f0 in freqs:
        k = int(np.argmin(np.abs(fr - f0)))
        out.append(f"{f0}:{np.abs(H[k]):.2f}({np.degrees(np.angle(H[k])):.0f}°,{coh[k]:.2f})")
    return " ".join(out)


def roll_corner_hz(V, clp, ixx=IXX):
    qb = 0.5 * RHO * V * V
    Lp = qb * S_REF * SPAN * SPAN * clp / (2 * V * ixx)
    return abs(Lp) / (2 * math.pi)


def flight_frf(path, nseg):
    want = ["time (us)", "axisRate[0]", "gyroADC[0]", "mspOverrideFlags"]
    t, sp, gy, fl = [], [], [], []
    with open(path) as f:
        r = csv.reader(f); h = [c.strip() for c in next(r)]; ix = {c: h.index(c) for c in want}
        for row in r:
            try:
                t.append(float(row[ix[want[0]]])); sp.append(float(row[ix[want[1]]]))
                gy.append(float(row[ix[want[2]]])); fl.append(int(float(row[ix[want[3]]])))
            except (ValueError, IndexError):
                pass
    t, sp, gy, fl = map(np.array, (t, sp, gy, fl))
    fs = 1e6 / np.median(np.diff(t))
    print(f"\nFLIGHT {path}: {len(t)} samples at {fs:.0f} Hz (engaged spans > 5 s)")
    mask = fl == 2
    e = np.diff(mask.astype(int)); st = list(np.where(e == 1)[0] + 1); en = list(np.where(e == -1)[0] + 1)
    if mask[0]: st = [0] + st
    if mask[-1]: en = en + [len(mask)]
    for s0, s1 in zip(st, en):
        if (t[s1 - 1] - t[s0]) > 5e6:
            fr, H, coh = frf(sp[s0:s1], gy[s0:s1], fs, nseg)
            print(f"  engaged {t[s0]/1e6:6.1f}-{t[s1-1]/1e6:6.1f}s: {fmt(fr, H, coh, FREQS)}")


def sim_frf(label, path, maxrate):
    with open(path) as f:
        r = csv.reader(f); h = next(r); first, last = {}, {}
        for i, n in enumerate(h):
            first.setdefault(n, i); last[n] = i
        sc = collections.defaultdict(list)
        for row in r:
            if len(row) < len(h):
                continue
            sc[int(row[first["scenario"]])].append((float(row[first["rollCmd"]]) * maxrate,
                                                    float(row[last["gyrP"]]) * GYRO_SCALE,
                                                    float(row[last["gyrQ"]]) * GYRO_SCALE,
                                                    float(row[last["gyrR"]]) * GYRO_SCALE))
    acc = {f0: [] for f0 in FREQS}
    used = departed = 0
    for s, rows in sc.items():
        A = np.array(rows)
        if len(A) < 256:
            continue
        if np.max(np.abs(A[:, 1:4])) > DEPARTED_RADPS:
            departed += 1
            continue
        used += 1
        fr, H, coh = frf(A[:, 0], np.degrees(A[:, 1]), 20.0, 128)
        for f0 in FREQS:
            k = int(np.argmin(np.abs(fr - f0))); acc[f0].append((np.abs(H[k]), np.degrees(np.angle(H[k])), coh[k]))
    print(f"\nSIM {label} ({path}): median over {used} scenarios, {departed} departed excluded (20 Hz; ignore > 8 Hz)")
    print("  sim: " + " ".join(
        f"{f0}:{np.median([v[0] for v in acc[f0]]):.2f}({np.median([v[1] for v in acc[f0]]):.0f}°,"
        f"{np.median([v[2] for v in acc[f0]]):.2f})" for f0 in FREQS))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", help="blackbox CSV (blackbox_decode --stdout)")
    ap.add_argument("--sim", action="append", default=[], help="LABEL:dmp-dump tick CSV (repeatable)")
    ap.add_argument("--maxrate", type=float, default=360.0, help="sim roll maxRate, deg/s")
    ap.add_argument("--nseg-flight", type=int, default=4096)
    ap.add_argument("--clp", action="append", type=float, default=[], help="Cl_p for the model rows (repeatable)")
    ap.add_argument("--V", type=float, default=15.0)
    a = ap.parse_args()

    print("f: |H| (phase, coherence) at " + ", ".join(str(f) for f in FREQS) + " Hz")
    if a.flight:
        flight_frf(a.flight, a.nseg_flight)
    for spec in a.sim:
        label, path = spec.split(":", 1)
        sim_frf(label, path, a.maxrate)
    clps = a.clp or [-0.135, -0.18, -0.36, -0.60]
    print(f"\nMODEL first-order roll mode at V = {a.V:g} m/s, I_xx = {IXX}:")
    for c in clps:
        print(f"  Cl_p = {c:6.3f}: corner {roll_corner_hz(a.V, c):.2f} Hz")
    print("  aircraft (043 flight-analysis-20261018 § airframe id): ~5 Hz;  t6 plant (Cl_p -0.135): ~1.1 Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
