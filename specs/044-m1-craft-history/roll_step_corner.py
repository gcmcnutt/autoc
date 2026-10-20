#!/usr/bin/env python3
"""
roll_step_corner.py — roll-mode corner and steady roll rate per aileron from Cntrl_StepTest CSVs (044 T022).

Honesty ladder rung 1 (research R11). The standalone step test runs the FDM with the servo model OFF, so the
roll-rate response to an aileron step is the bare airframe's first-order roll mode:
    tau * dp/dt = K * delta_a - p          (p deg/s, delta_a the surface command, ±0.5 = full)
Each roll cell (axis 0) is fitted over its ramp + hold window by least squares on (K, tau) with the measured
command as input; the corner is 1 / (2 pi tau). K is the steady roll rate per unit aileron — the quantity
FR-006 keeps fixed across the class (Cl_da scales with Cl_p).

A second estimate uses the 043 flight method (airframe_id.py fit (c)): the FRF H = S_up / S_uu over the roll cells,
fitted by a first-order lag WITH a free pure delay, H = k / (1 + j f/fc) · e^(-j 2π f t), on 0.5–8 Hz. That is the
number to set beside the flight's ~5 Hz (same fit), and beside the Cl_p-only formula corner the class was sized with.
Corners scale with airspeed (L_p ∝ q̄/V ∝ V), so every corner is also quoted normalised to 15 m/s.

Usage:
  roll_step_corner.py LABEL:steptest.csv [LABEL:steptest.csv ...]
"""
import csv
import math
import sys

import numpy as np

RAD2DEG = 57.29577951308232


def load(path):
    rows = list(csv.DictReader(open(path)))
    return rows


def cells(rows):
    by = {}
    for r in rows:
        by.setdefault(int(r["cell"]), []).append(r)
    return by


def fit_first_order(t, u, p):
    """Fit tau*p' = K*u - p by simulating the ODE over a (K, tau) grid refined twice; returns K, tau, rms."""
    best = None
    def sim(K, tau):
        y = np.empty_like(p); y[0] = p[0]
        for i in range(1, len(t)):
            dt = t[i] - t[i - 1]
            a = math.exp(-dt / tau)
            y[i] = a * y[i - 1] + (1 - a) * K * u[i - 1]
        return y
    taus = np.geomspace(0.005, 0.5, 60)
    # K by linear least squares for each tau (response is linear in K given p0 handled by sim)
    for tau in taus:
        y1 = sim(1.0, tau) - sim(0.0, tau)        # unit-K response
        y0 = sim(0.0, tau)                          # free response from p0
        K = float(np.dot(y1, p - y0) / max(np.dot(y1, y1), 1e-12))
        r = float(np.sqrt(np.mean((y0 + K * y1 - p) ** 2)))
        if best is None or r < best[2]:
            best = (K, tau, r)
    lo, hi = best[1] / 1.15, best[1] * 1.15
    for tau in np.linspace(lo, hi, 60):
        y1 = sim(1.0, tau) - sim(0.0, tau); y0 = sim(0.0, tau)
        K = float(np.dot(y1, p - y0) / max(np.dot(y1, y1), 1e-12))
        r = float(np.sqrt(np.mean((y0 + K * y1 - p) ** 2)))
        if r < best[2]:
            best = (K, tau, r)
    return best


def frf_corner(t, u, p, V):
    """airframe_id.py fit (c): first-order + delay on the roll FRF of the concatenated roll cells."""
    fs = 1.0 / np.median(np.diff(t))
    nseg = 512
    x = u - np.mean(u); y = p - np.mean(p)
    w = np.hanning(nseg); Puu = Pup = 0.0
    for i in range(0, len(x) - nseg + 1, nseg // 2):
        X = np.fft.rfft(x[i:i + nseg] * w); Y = np.fft.rfft(y[i:i + nseg] * w)
        Puu = Puu + np.abs(X) ** 2; Pup = Pup + np.conj(X) * Y
    fr = np.fft.rfftfreq(nseg, 1 / fs)
    m = (fr >= 0.5) & (fr <= 8.0)
    f, H, W = fr[m], (Pup / Puu)[m], Puu[m]
    best = None
    for fc in np.geomspace(0.3, 20, 160):
        for d in np.linspace(0.0, 0.06, 31):
            G = 1.0 / (1 + 1j * f / fc) * np.exp(-2j * np.pi * f * d)
            k = np.sum(W * np.conj(G) * H) / np.sum(W * np.abs(G) ** 2)
            e = np.sum(W * np.abs(H - k * G) ** 2) / np.sum(W * np.abs(H) ** 2)
            if best is None or e < best[0]:
                best = (e, fc, d, abs(k))
    return best


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 2
    print(f"{'model':>10} {'cell':>4} {'V m/s':>6} {'K deg/s/unit':>13} {'tau ms':>7} {'corner Hz':>9} {'rms':>6}")
    summary = {}
    for spec in sys.argv[1:]:
        label, path = spec.split(":", 1)
        for c, rows in sorted(cells(load(path)).items()):
            if int(rows[0]["axis"]) != 0:
                continue
            live = [r for r in rows if r["phase"] in ("ramp", "hold", "step", "pulse")]
            if not live:
                live = [r for r in rows if r["phase"] not in ("trim", "settle", "recover")]
            if len(live) < 20:
                continue
            t = np.array([float(r["t_s"]) for r in live])
            u = np.array([float(r["cmd_aileron"]) for r in live])
            p = np.array([float(r["rate_p"]) * RAD2DEG for r in live])
            V = np.median([float(r["v_rel_airmass"]) * 0.3048 for r in live])
            K, tau, rms = fit_first_order(t, u, p)
            f = 1 / (2 * math.pi * tau)
            summary.setdefault(label, []).append((V, K, tau, f))
            print(f"{label:>10} {c:4d} {V:6.1f} {K:13.0f} {1000*tau:7.1f} {f:9.2f} {rms:6.1f}")
    print("\nmedian per model (step fits; corner also normalised to 15 m/s):")
    for label, v in summary.items():
        A = np.array(v)
        print(f"  {label:>10}: V {np.median(A[:,0]):5.1f} m/s  K {np.median(A[:,1]):6.0f} deg/s per unit aileron  "
              f"corner {np.median(A[:,3]):.2f} Hz (tau {1000*np.median(A[:,2]):.0f} ms), "
              f"at 15 m/s {np.median(A[:,3] * 15.0 / A[:,0]):.2f} Hz")
    print("\nflight-method FRF fit (first order + free delay, 0.5-8 Hz) over all roll cells:")
    for spec in sys.argv[1:]:
        label, path = spec.split(":", 1)
        rows = [r for r in load(path) if int(r["axis"]) == 0 and r["phase"] != "trim"]
        t = np.array([float(r["t_s"]) for r in rows]); u = np.array([float(r["cmd_aileron"]) for r in rows])
        p = np.array([float(r["rate_p"]) * RAD2DEG for r in rows])
        V = float(np.median([float(r["v_rel_airmass"]) * 0.3048 for r in rows]))
        e, fc, d, k = frf_corner(t, u, p, V)
        print(f"  {label:>10}: corner {fc:.2f} Hz at V {V:.1f} m/s (at 15 m/s {fc * 15.0 / V:.2f} Hz), delay {1000*d:.0f} ms, "
              f"|k| {k:.0f} deg/s per unit, residual {e:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
