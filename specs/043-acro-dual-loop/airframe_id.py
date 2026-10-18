#!/usr/bin/env python3
"""
airframe_id.py — BARE-AIRFRAME PITCH AND ROLL RESPONSE FROM FLIGHT, CHECKED AGAINST THE MEASURED INERTIA (043, 2026-10-18)

The 10-18 blackbox logs servo outputs (SERVOS re-enabled), so the airframe can be identified without the INAV rate loop in the way:
  elevator  δe = (servo[0] − servo[1]) / 2,  aileron δa = −(servo[0] + servo[1]) / 2     (µs from centre; mixing fitted from the data:
  servo0 = −0.80·roll + 0.83·pitch, servo1 = −0.83·roll − 0.86·pitch)
Closed-loop data biases a direct q/δ estimate (turbulence → q → controller → δ), so the airframe FRF is taken INDIRECTLY through the
setpoint r (axisRate, set by the NN): H_air = S_rq / S_rδ  (cross-spectra, Welch, pooled over engaged spans, coherence-weighted).

Then the model's linear modes are fitted to the measured shape (the µs→rad scale is unknown, so gain is free; a pure delay absorbs
servo lag + 50 Hz servo update):
  pitch short period:  α̇ = Zα α + q,  q̇ = Mα α + Mq q + Mδ δ;  Zα = −q̄ S CL_a /(m V), Mα = q̄ S c Cm_a / I_yy,
                       Mq = q̄ S c² Cm_q /(2 V I_yy)  →  q/δ = Mδ (s − Zα) / (s² − (Zα + Mq) s + Zα Mq − Mα)
  roll subsidence:     p/δ = Lδ / (s − Lp),  Lp = q̄ S b² Cl_p /(2 V I_xx)
Fits: (a) I_yy free with the model's Cm_a / Cm_q / CL_a, (b) Cm_a/I_yy and Cm_q/I_yy free (what the flight alone determines), (c) roll
corner → I_xx implied by the model's Cl_p.

Usage: airframe_id.py bb1.csv [bb2.csv ...] --V 12.5
"""
import argparse, csv, sys
import numpy as np

RHO, S, C, B, M = 1.225, 0.136, 0.178, 0.762, 0.510
CL_A, CM_A, CM_Q, CL_P = 4.7, -0.55, -1.0, -0.135
I_YY, I_XX = 0.0094, 0.0069


def engaged_segments(path):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    need = ["time (us)", "axisRate[0]", "axisRate[1]", "gyroADC[0]", "gyroADC[1]", "servo[0]", "servo[1]", "flightModeFlags (flags)"]
    ix = [h.index(c) for c in need]
    segs, cur = [], []
    for row in r:
        if len(row) < len(h): continue
        on = "MSPRCOVERRIDE" in row[ix[-1]]
        if on: cur.append([float(row[i]) for i in ix[:-1]])
        elif cur: segs.append(np.array(cur)); cur = []
    if cur: segs.append(np.array(cur))
    out = []
    for A in segs:
        if len(A) < 1500: continue
        t = (A[:, 0] - A[0, 0]) / 1e6; fs = 1 / np.median(np.diff(t))
        tu = np.arange(0, t[-1], 1 / 250.0)
        U = np.column_stack([np.interp(tu, t, A[:, j]) for j in range(1, A.shape[1])])
        out.append(U)
    return out


def xspec(x, y, nseg=512, fs=250.0):
    w = np.hanning(nseg); Sxy = Sxx = Syy = 0
    for i in range(0, len(x) - nseg + 1, nseg // 2):
        X = np.fft.rfft((x[i:i + nseg] - x[i:i + nseg].mean()) * w); Y = np.fft.rfft((y[i:i + nseg] - y[i:i + nseg].mean()) * w)
        Sxy = Sxy + np.conj(X) * Y; Sxx = Sxx + np.abs(X) ** 2; Syy = Syy + np.abs(Y) ** 2
    return np.fft.rfftfreq(nseg, 1 / fs), Sxy, Sxx, Syy


def short_period(f, V, Iyy, cma=CM_A, cmq=CM_Q, tau=0.0, gain=1.0):
    q = 0.5 * RHO * V * V; s = 2j * np.pi * f
    Za = -q * S * CL_A / (M * V); Ma = q * S * C * cma / Iyy; Mq = q * S * C * C * cmq / (2 * V * Iyy)
    return gain * (s - Za) / (s * s - (Za + Mq) * s + Za * Mq - Ma) * np.exp(-s * tau)


def sp_modes(V, Iyy, cma=CM_A, cmq=CM_Q):
    q = 0.5 * RHO * V * V
    Za = -q * S * CL_A / (M * V); Ma = q * S * C * cma / Iyy; Mq = q * S * C * C * cmq / (2 * V * Iyy)
    w2 = Za * Mq - Ma; w = np.sqrt(max(w2, 1e-9)); z = -(Za + Mq) / (2 * w)
    return w / (2 * np.pi), z


def fit_shape(f, H, W, model, grid):
    """Grid-search the model parameters; gain is solved in closed form (complex LS, weighted)."""
    best = None
    for p in grid:
        G = model(f, *p)
        k = np.sum(W * np.conj(G) * H) / np.sum(W * np.abs(G) ** 2)
        e = np.sum(W * np.abs(H - k * G) ** 2) / np.sum(W * np.abs(H) ** 2)
        if best is None or e < best[0]: best = (e, p, k)
    return best


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv", nargs="+"); ap.add_argument("--V", type=float, default=12.5)
    a = ap.parse_args()
    segs = sum((engaged_segments(p) for p in a.csv), [])
    print(f"{len(segs)} engaged segments, {sum(len(s) for s in segs) / 250:.0f} s at 250 Hz; airspeed for the model V = {a.V} m/s")
    acc = {k: 0 for k in ("rq", "rd", "rr", "rp", "ra", "rrr", "dd", "aa")}
    for U in segs:
        rR, rP, gR, gP, s0, s1 = U.T
        de = (s0 - s1) / 2; da = -(s0 + s1) / 2
        f, Srq, Srr, Sqq = xspec(rP, gP); _, Srd, _, Sdd = xspec(rP, de)
        _, Srp, SrrR, Spp = xspec(rR, gR); _, Sra, _, Saa = xspec(rR, da)
        acc["rq"] += Srq; acc["rd"] += Srd; acc["rr"] += Srr; acc["dd"] += Sdd
        acc["rp"] += Srp; acc["ra"] += Sra; acc["rrr"] += SrrR; acc["aa"] += Saa
    Hp = acc["rq"] / acc["rd"]; Hr = acc["rp"] / acc["ra"]
    coh_p = np.abs(acc["rd"]) ** 2 / (acc["rr"] * acc["dd"]); coh_r = np.abs(acc["ra"]) ** 2 / (acc["rrr"] * acc["aa"])
    band = (f >= 0.6) & (f <= 6.0)
    print("\nmeasured airframe FRF (gyro / surface, normalized at 1 Hz), coherence of setpoint→surface:")
    i1 = np.argmin(np.abs(f - 1.0))
    print(f"  {'f Hz':>5s} | {'pitch |H|':>9s} {'phase':>6s} {'coh':>5s} | {'roll |H|':>8s} {'phase':>6s} {'coh':>5s}")
    for fq in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0):
        i = np.argmin(np.abs(f - fq))
        print(f"  {f[i]:5.2f} | {abs(Hp[i]) / abs(Hp[i1]):9.2f} {np.degrees(np.angle(Hp[i] / Hp[i1])):6.0f} {coh_p[i]:5.2f} | "
              f"{abs(Hr[i]) / abs(Hr[i1]):8.2f} {np.degrees(np.angle(Hr[i] / Hr[i1])):6.0f} {coh_r[i]:5.2f}")
    fb, Hpb, Wp = f[band], Hp[band], coh_p[band]
    # (a) I_yy free, model Cm_a / Cm_q, delay free
    taus = np.arange(0.0, 0.081, 0.005)
    ga = [(a.V, I, CM_A, CM_Q, t) for I in np.geomspace(0.001, 0.05, 80) for t in taus]
    ea, pa, _ = fit_shape(fb, Hpb, Wp, lambda f, V, I, cma, cmq, t: short_period(f, V, I, cma, cmq, t), ga)
    fsp, zsp = sp_modes(a.V, pa[1])
    print(f"\n(a) pitch, I_yy free with the model's Cm_a {CM_A} / Cm_q {CM_Q}: best I_yy = {pa[1]:.4f} kg·m² (bifilar 0.0094), delay "
          f"{1000 * pa[4]:.0f} ms, residual {ea:.2f};  short period {fsp:.2f} Hz, ζ {zsp:.2f}")
    for I in (0.0013, 0.0047, 0.0094, 0.0141, 0.019):
        e1 = fit_shape(fb, Hpb, Wp, lambda f, V, I_, cma, cmq, t: short_period(f, V, I_, cma, cmq, t),
                       [(a.V, I, CM_A, CM_Q, t) for t in taus])[0]
        f1, z1 = sp_modes(a.V, I)
        print(f"     I_yy {I:.4f}: residual {e1:.2f}   (model short period {f1:.2f} Hz, ζ {z1:.2f})")
    # (b) ratios free: Cm_a and Cm_q at the bifilar I_yy (equivalently Cm_a/I_yy, Cm_q/I_yy)
    gb = [(a.V, I_YY, cma, cmq, t) for cma in np.linspace(-1.6, -0.15, 30) for cmq in np.linspace(-8.0, -0.1, 40) for t in taus[::2]]
    eb, pb, _ = fit_shape(fb, Hpb, Wp, lambda f, V, I, cma, cmq, t: short_period(f, V, I, cma, cmq, t), gb)
    fsb, zsb = sp_modes(a.V, I_YY, pb[2], pb[3])
    print(f"(b) pitch, Cm_a and Cm_q free at I_yy 0.0094: Cm_a {pb[2]:.2f}, Cm_q {pb[3]:.2f}, delay {1000 * pb[4]:.0f} ms, residual {eb:.2f}; "
          f"short period {fsb:.2f} Hz, ζ {zsb:.2f}  (Cm_a/I_yy {pb[2] / I_YY:.0f}, model {CM_A / I_YY:.0f})")
    # (c) roll subsidence
    fr, Hrb, Wr = f[band], Hr[band], coh_r[band]
    roll = lambda f, fc, t: 1.0 / (1 + 1j * f / fc) * np.exp(-2j * np.pi * f * t)
    er, prr, _ = fit_shape(fr, Hrb, Wr, roll, [(fc, t) for fc in np.geomspace(0.3, 20, 120) for t in taus])
    q = 0.5 * RHO * a.V ** 2; Lp_unit = q * S * B * B / (2 * a.V)   # Lp = Lp_unit * Cl_p / I_xx
    Ixx_impl = Lp_unit * abs(CL_P) / (2 * np.pi * prr[0])
    print(f"(c) roll, first-order fit: corner {prr[0]:.2f} Hz (τ {1000 / (2 * np.pi * prr[0]):.0f} ms), delay {1000 * prr[1]:.0f} ms, "
          f"residual {er:.2f};  model corner {Lp_unit * abs(CL_P) / I_XX / (2 * np.pi):.2f} Hz at I_xx {I_XX}, Cl_p {CL_P}; "
          f"I_xx implied by the model's Cl_p = {Ixx_impl:.4f}")


if __name__ == "__main__":
    sys.exit(main())
