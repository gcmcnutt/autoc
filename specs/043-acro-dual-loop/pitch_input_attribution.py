#!/usr/bin/env python3
"""
pitch_input_attribution.py — WHAT DOES THE POLICY LOOK AT WHEN IT DITHERS? (043 t6, 2026-10-15)

Question (operator): the sim is now honest to ~Nyquist; the NN's temporal view — which history slots it has, at
what cadence — has not been analysed against a 1–4 Hz cycle. This script answers, from a dmp tick CSV of one
genome (dmp-dump --csv-only):

  1. The policy's temporal apparatus, stated: the only history the input vector carries is the TARGET
     representation at lags {800,400,200,100,50,0} ms (tgX/Y/Z, ds); the craft block (quat, airspeed, gyro,
     accel, Es, …) is INSTANTANEOUS. Anything else about the past lives in the 16-unit recurrent state.
  2. A linear surrogate of the pitch output: out_pt ~ 45 inputs (pooled over scenarios), R² overall and R²
     of the 1.5–4 Hz band of out_pt, then group-ablation ΔR² in that band — which slot GROUP carries the
     cycle: target history (and which lags), gyro, quaternion, accel, the rest.
  3. The loop seen from the policy: lagged correlation out_pt(t) ↔ gyrQ(t+k), k = −6..+6 ticks, and the
     cross-spectral phase between out_pt and gyrQ at 1–4 Hz — how far the policy's command sits from
     anti-phase with the rate it measures (the relay signature is |phase| → 180° with gain ≥ 1).
  4. The lag-set nulls: for a sinusoid at f, the difference (slot_now − slot_lag τ) has gain |1−e^{−j2πfτ}|,
     zero at f = 1/τ: 400 ms → 2.5 Hz, 200 ms → 5 Hz, 800 ms → 1.25 Hz. Printed for the band of interest.

Usage: pitch_input_attribution.py tick.csv [--band 1.5 4] [--label NAME]
"""
import argparse, csv, sys
import numpy as np
from collections import defaultdict

FS = 20.0
INPUT_COLS = 45          # tgX-5 .. sgZ, contiguous after rbHhd
GROUPS = {
    "target dir history (tgX/Y/Z, 6 lags)": lambda n: n.startswith(("tgX", "tgY", "tgZ")),
    "  of which lag 0/50/100 ms": lambda n: n.startswith(("tgX", "tgY", "tgZ")) and n[-2:] in ("-1", "-2", "X0", "Y0", "Z0"),
    "  of which lag 200/400/800 ms": lambda n: n.startswith(("tgX", "tgY", "tgZ")) and n[-2:] in ("-3", "-4", "-5"),
    "distance history (ds, 6 lags) + dd/dt": lambda n: n.startswith("ds") or n == "dd/dt",
    "quaternion (now)": lambda n: n in ("qw", "qx", "qy", "qz"),
    "gyro (now)": lambda n: n in ("gyrP", "gyrQ", "gyrR"),
    "accel (now)": lambda n: n in ("acX", "acY", "acZ"),
    "airspeed/Es/boundary/inward/score-grad": lambda n: n in ("vel", "Es", "bClR", "dBnd", "inX", "inY", "inZ", "sgX", "sgY", "sgZ"),
}


def bandpass(x, lo, hi):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / FS)
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n=len(x))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--band", nargs=2, type=float, default=[1.5, 4.0]); ap.add_argument("--label", default="run")
    a = ap.parse_args(); lo, hi = a.band
    r = csv.reader(open(a.csv)); h = [c.strip() for c in next(r)]
    if "target_x_0" in h:
        # xiao flight log (flightlog_decode.py): same 45 slots, names differ; target_*_5 is NOW (lag index 5 = 0 ms)
        XMAP = {}
        for ax, tg in (("x", "tgX"), ("y", "tgY"), ("z", "tgZ")):
            for k in range(6): XMAP[f"target_{ax}_{k}"] = f"{tg}-{5-k}" if k < 5 else f"{tg}0"
        for k in range(6): XMAP[f"dist_{k}"] = f"ds-{5-k}" if k < 5 else "ds0"
        XMAP.update({"closing_rate": "dd/dt", "quat_w": "qw", "quat_x": "qx", "quat_y": "qy", "quat_z": "qz", "airspeed": "vel",
                     "gyro_p": "gyrP", "gyro_q": "gyrQ", "gyro_r": "gyrR", "accel_x": "acX", "accel_y": "acY", "accel_z": "acZ",
                     "specific_energy": "Es", "boundary_closure_rate": "bClR", "dist_to_boundary": "dBnd", "inward_body_x": "inX",
                     "inward_body_y": "inY", "inward_body_z": "inZ", "score_grad_x": "sgX", "score_grad_y": "sgY", "score_grad_z": "sgZ"})
        i0 = h.index("target_x_0"); names = [XMAP[c] for c in h[i0:i0 + INPUT_COLS]]
        iS, iP = h.index("span_id"), h.index("out_pitch")
    else:
        i0 = h.index("rbHhd") + 1; names = h[i0:i0 + INPUT_COLS]
        iS, iP = h.index("scenario"), h.index("out_pt")
    assert names[0] == "tgX-5" and names[-1] == "sgZ", names
    sc = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        sc[int(row[iS])].append([float(row[iP])] + [float(v) for v in row[i0:i0 + INPUT_COLS]])
    print(f"{a.label}: {len(sc)} scenarios")
    # --- pooled linear surrogate
    Y, X, Yb, Xb, lagc = [], [], [], [], defaultdict(list)
    for s, rows in sc.items():
        A = np.array(rows)
        if len(A) < 80: continue
        if np.abs(A[:, 1 + names.index("gyrP"):1 + names.index("gyrR") + 1]).max() > 2.5: continue  # departed flight (>15 rad/s)
        y, x = A[:, 0], A[:, 1:]
        Y.append(y); X.append(x)
        Yb.append(bandpass(y, lo, hi)); Xb.append(np.column_stack([bandpass(x[:, j], lo, hi) for j in range(x.shape[1])]))
        q = x[:, names.index("gyrQ")]
        yc, qc = y - y.mean(), q - q.mean(); den = np.sqrt((yc ** 2).sum() * (qc ** 2).sum()) or 1
        for k in range(-6, 7):
            if k >= 0: c = (yc[:len(yc) - k] * qc[k:]).sum() / den
            else: c = (yc[-k:] * qc[:len(qc) + k]).sum() / den
            lagc[k].append(c)
    Y = np.concatenate(Y); X = np.concatenate(X); Yb = np.concatenate(Yb); Xb = np.concatenate(Xb)

    def r2(Xm, y):
        Xm1 = np.column_stack([Xm, np.ones(len(y))]); b, *_ = np.linalg.lstsq(Xm1, y, rcond=None)
        res = y - Xm1 @ b; return 1 - res.var() / y.var()
    R_all = r2(X, Y); R_band = r2(Xb, Yb)
    print(f"linear surrogate out_pt ~ 45 inputs: R² = {R_all:.3f} (all frequencies), R² = {R_band:.3f} in {lo}-{hi} Hz band")
    print(f"ablation ΔR² (band): how much of the policy's {lo}-{hi} Hz pitch command each slot group explains")
    for g, pred in GROUPS.items():
        keep = [j for j, n in enumerate(names) if not pred(n)]
        d = R_band - r2(Xb[:, keep], Yb)
        only = r2(Xb[:, [j for j, n in enumerate(names) if pred(n)]], Yb)
        print(f"  {g:44s} drop → ΔR² {d:+.3f}   alone R² {only:.3f}")
    # --- lagged correlation and phase
    print(f"lagged corr out_pt(t) vs gyrQ(t+k) [median over scenarios], k in ticks (50 ms); +k = gyro AFTER command:")
    print("  " + "  ".join(f"{k:+d}:{np.median(lagc[k]):+.2f}" for k in range(-6, 7)))
    # cross-spectral phase per band
    def xphase(y, q, f1, f2):
        Yf = np.fft.rfft(y - y.mean()); Qf = np.fft.rfft(q - q.mean()); f = np.fft.rfftfreq(len(y), 1 / FS)
        m = (f >= f1) & (f < f2); S = (Yf[m] * np.conj(Qf[m])).sum()
        return np.degrees(np.angle(S)), np.abs(S)
    for f1, f2 in ((1, 2), (2, 3), (3, 4), (4, 6)):
        ph = [];
        for s, rows in sc.items():
            A = np.array(rows)
            if len(A) < 80: continue
            p, w = xphase(A[:, 0], A[:, 1 + names.index("gyrQ")], f1, f2); ph.append(p)
        print(f"  phase(out_pt vs gyrQ) {f1}-{f2} Hz: median {np.median(ph):+.0f}°  (−180° = command opposes measured rate exactly)")
    # --- lag-set nulls
    print("history-difference gain |1 − e^{−j2πfτ}| per lag slot (0 = that slot cannot see a cycle at f):")
    print("  f(Hz)   " + "  ".join(f"{t:>5d}ms" for t in (50, 100, 200, 400, 800)))
    for f in (1.0, 1.5, 2.0, 2.3, 2.5, 3.0, 3.5, 4.0):
        print(f"  {f:4.1f}    " + "  ".join(f"{abs(1 - np.exp(-2j * np.pi * f * t / 1000)):7.2f}" for t in (50, 100, 200, 400, 800)))


if __name__ == "__main__":
    sys.exit(main())
