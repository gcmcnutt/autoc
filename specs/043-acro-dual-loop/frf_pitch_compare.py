#!/usr/bin/env python3
"""
frf_pitch_compare.py — closed-inner-loop pitch frequency response, flight vs sim (043, 2026-10-13).

The quantity: H(f) = gyro / rate-setpoint, i.e. INAV's rate loop + servo + airframe as one block,
estimated from the policy's own excitation (cross-spectrum / auto-spectrum, Hann/Welch, 50 % overlap)
with coherence reported so low-excitation bins can be ignored.

  flight : a blackbox CSV from `blackbox_decode --stdout <log.TXT>` (966 Hz logs of 2026-10-11 used
           first); `axisRate[1]` is the pitch setpoint, `gyroADC[1]` the filtered gyro, both deg/s;
           engaged spans are `mspOverrideFlags == 2` longer than 5 s; the longest non-engaged stretch
           is printed as the pilot (MANUAL) contrast.
  sim    : a dmp-dump tick CSV (20 Hz): `pitchCmd` × maxRate (240 deg/s) is the setpoint, `gyrQ` × 6
           rad/s (kGyroScale_radps) the body pitch rate; medians over scenarios. Nyquist is 10 Hz, so
           bins above ~8 Hz are not meaningful on the sim side.
  model  : the short-period approximation from hb1_streamer.xml numbers (f_n, zeta vs airspeed), so a
           proposed aero/inertia change can be sanity-checked before a rebuild:
             M_alpha = qS c Cm_a / Iyy,  M_q = qS c^2 Cm_q /(2 V Iyy),  Z_alpha = -qS CL_a / m
             w_n^2 = Z_alpha M_q / V - M_alpha,   zeta = -(M_q + Z_alpha/V) / (2 w_n)

First result (gen 488, flown 10-11): flight and sim agree on |H| and phase to 1 Hz and on phase to
2.5 Hz; the aircraft peaks |H| 1.5-1.6 at 2.5 Hz and rolls off, the sim keeps rising to 1.54 at 6 Hz —
the model file's short period is 6.9 Hz / zeta 0.81 at 14 m/s (Iyy 0.0013) against the aircraft's
~3.0 Hz / zeta 0.2-0.3. See sim-pitch-fidelity-proposal.md.

Usage:
  frf_pitch_compare.py [--flight bb.csv] [--sim tick.csv] [--model Iyy,Cm_a,Cm_q,CL_a ...]
"""
import argparse
import collections
import csv
import math
import sys

import numpy as np

FREQS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0]
RHO, S_REF, CBAR, MASS = 1.225, 0.136, 0.178, 0.515   # hb1_streamer.xml <ref> and <mass_inertia>


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


def short_period(V, Iyy, Cma, Cmq, CLa):
    qb = 0.5 * RHO * V * V
    Ma = qb * S_REF * CBAR * Cma / Iyy
    Mq = qb * S_REF * CBAR * CBAR * Cmq / (2 * V * Iyy)
    Za = -qb * S_REF * CLa / MASS
    w2 = Za * Mq / V - Ma
    w = math.sqrt(max(w2, 1e-9))
    return w / (2 * math.pi), -(Mq + Za / V) / (2 * w)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight", help="blackbox CSV (blackbox_decode --stdout)")
    ap.add_argument("--sim", help="dmp-dump tick CSV")
    ap.add_argument("--maxrate", type=float, default=240.0)
    ap.add_argument("--nseg-flight", type=int, default=4096)
    ap.add_argument("--model", action="append", default=[], help="Iyy,Cm_a,Cm_q,CL_a (repeatable)")
    a = ap.parse_args()

    print("f: |H| (phase, coherence) at " + ", ".join(str(f) for f in FREQS) + " Hz")
    if a.flight:
        want = ["time (us)", "axisRate[1]", "gyroADC[1]", "mspOverrideFlags"]
        t, sp, gy, fl = [], [], [], []
        with open(a.flight) as f:
            r = csv.reader(f); h = [c.strip() for c in next(r)]; ix = {c: h.index(c) for c in want}
            for row in r:
                try:
                    t.append(float(row[ix[want[0]]])); sp.append(float(row[ix[want[1]]]))
                    gy.append(float(row[ix[want[2]]])); fl.append(int(float(row[ix[want[3]]])))
                except (ValueError, IndexError):
                    pass
        t, sp, gy, fl = map(np.array, (t, sp, gy, fl))
        fs = 1e6 / np.median(np.diff(t))
        print(f"\nFLIGHT {a.flight}: {len(t)} samples at {fs:.0f} Hz")
        for label, mask, minlen in (("engaged", fl == 2, 5e6), ("MANUAL", fl != 2, 10e6)):
            e = np.diff(mask.astype(int)); st = list(np.where(e == 1)[0] + 1); en = list(np.where(e == -1)[0] + 1)
            if mask[0]: st = [0] + st
            if mask[-1]: en = en + [len(mask)]
            spans = [(s0, s1) for s0, s1 in zip(st, en) if (t[s1 - 1] - t[s0]) > minlen]
            if label == "MANUAL" and spans:
                spans = [max(spans, key=lambda ab: ab[1] - ab[0])]
            for s0, s1 in spans:
                fr, H, coh = frf(sp[s0:s1], gy[s0:s1], fs, a.nseg_flight)
                print(f"  {label:8s} {t[s0]/1e6:6.1f}-{t[s1-1]/1e6:6.1f}s: {fmt(fr, H, coh, FREQS)}")
    if a.sim:
        with open(a.sim) as f:
            r = csv.reader(f); h = next(r); first, last = {}, {}
            for i, n in enumerate(h):
                first.setdefault(n, i); last[n] = i
            sc = collections.defaultdict(list)
            for row in r:
                if len(row) < len(h):
                    continue
                sc[int(row[first["scenario"]])].append((float(row[first["pitchCmd"]]) * a.maxrate,
                                                        math.degrees(float(row[last["gyrQ"]]) * 6.0)))
        acc = {f0: [] for f0 in FREQS}
        for s, rows in sc.items():
            A = np.array(rows)
            if len(A) < 256:
                continue
            fr, H, coh = frf(A[:, 0], A[:, 1], 20.0, 128)
            for f0 in FREQS:
                k = int(np.argmin(np.abs(fr - f0))); acc[f0].append((np.abs(H[k]), np.degrees(np.angle(H[k])), coh[k]))
        print(f"\nSIM {a.sim}: median over {len(sc)} scenarios (20 Hz; ignore > 8 Hz)")
        print("  sim: " + " ".join(f"{f0}:{np.median([v[0] for v in acc[f0]]):.2f}({np.median([v[1] for v in acc[f0]]):.0f}°,{np.median([v[2] for v in acc[f0]]):.2f})" for f0 in FREQS))
    models = [tuple(float(x) for x in m.split(",")) for m in a.model] or [(0.0013, -0.55, -4.2, 4.7)]
    print("\nMODEL short period (f_n Hz / zeta at V = 11 / 14 / 17 m/s):")
    for Iyy, Cma, Cmq, CLa in models:
        vals = [short_period(V, Iyy, Cma, Cmq, CLa) for V in (11, 14, 17)]
        print(f"  Iyy={Iyy:.4f} Cm_a={Cma:5.2f} Cm_q={Cmq:5.2f} CL_a={CLa:.1f}: " + "  ".join(f"{f:.1f}/{z:.2f}" for f, z in vals))
    print("  aircraft (actuator-pin § 2, 10-11 FRF): ~2.7 / ~3.0 / ~3.3 Hz rising with airspeed, zeta ~0.2-0.3; |H| peak 1.5-1.6 at 2.5 Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
