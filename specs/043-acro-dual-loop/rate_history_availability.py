#!/usr/bin/env python3
"""
rate_history_availability.py — IS THE PROPOSED GYRO HISTORY ALREADY INSIDE THE NETWORK? (043 T121, 2026-10-15)

The previous command is in the recurrent state exactly (o_{t-1} = tanh(W3 h_{t-1} + b3)). The proposed slots are the craft's own
pitch rate 50 and 100 ms ago. Two places they could already be hiding:
  1. the TARGET-bearing history — it is stored in the body frame of its own tick, so the craft's rotation moves it:
     for a fixed target, d(body) rotates as d' = -omega x d, so omega_perp = -d x d' (exact geometry, needs a cross product)
  2. the recurrent state h(t-1), if the policy chose to keep it.
This script measures how much of q(t-1), q(t-2) and dq = q(t-1)-q(t-2) [the rate DERIVATIVE, the damping/lead term] is recoverable,
BEYOND what q(t) (the gyro now, already an input) explains — the partial R²: 1 - var(resid | q_now + X) / var(resid | q_now).
  X = bearing slots, linear (what one layer reads directly)
  X = geometric decode -(d_t x (d_t - d_{t-1}))/dt and the same at t-1 (best case — exact cross product; translation of the rabbit
      relative to the craft is the residual confound)
  X = h(t-1) from an exact replay of the genome (what memory keeps)
Usage: rate_history_availability.py tick.csv --nn genome.cpp [--every 12]
       (also accepts a flightlog_decode.py CSV; the genome is replayed on the logged inputs)
"""
import argparse, csv, os, sys
import numpy as np
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nn_replay_probe import Net, parse_nn_cpp

DT = 0.05


def load(path, every):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    if "target_x_0" in h:   # xiao flight log: 45 inputs from target_x_0; out_pitch is the logged command
        i0, iS, iP = h.index("target_x_0"), h.index("span_id"), h.index("out_pitch")
    else:
        i0, iS, iP = h.index("rbHhd") + 1, h.index("scenario"), h.index("out_pt")
    sc = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        s = int(float(row[iS]))
        if s % every: continue
        sc[s].append([float(row[iP])] + [float(v) for v in row[i0:i0 + 45]])
    out = {s: np.array(v, np.float32) for s, v in sc.items() if len(v) > 40}
    # departed flights (body rate > 15 rad/s = 2.5 NN units in slots 30-32) excluded — a tumble is not the controlled regime
    return {s: A for s, A in out.items() if np.abs(A[:, 1 + 30:1 + 33]).max() <= 2.5}


# slot indices in the 45-vector (PathgenInput order)
TGX, TGY, TGZ, DS = range(0, 6), range(6, 12), range(12, 18), range(18, 24)
NOW, L1, L2 = 5, 4, 3          # array index: 5 = now, 4 = 50 ms, 3 = 100 ms
GYRO_Q, GYRO_P, GYRO_R = 31, 30, 32
GS = 6.0                        # kGyroScale_radps


def partial_r2(y, base, extra, groups, ridge=0.05):
    """Partial R² of `extra` beyond `base`, two-fold by alternating scenario/span groups (fit on one fold, score on the
    other, both directions pooled), ridge on standardized features — saturated tanh units are near-collinear."""
    # within-scenario (fixed-effects) decode: remove each scenario's mean from target and features first, so per-scenario
    # saturation offsets of the tanh units cannot masquerade as (or hide) tick-to-tick information
    def demean(M):
        M = np.array(M, float, copy=True)
        for gg in np.unique(groups):
            m = groups == gg; M[m] -= M[m].mean(0)
        return M
    y = demean(y); base = demean(base); extra = demean(extra)
    g = np.unique(groups); fold = np.isin(groups, g[0::2])
    def oos(X):
        X = np.asarray(X, float); res = np.empty_like(y, dtype=float)
        for tr in (fold, ~fold):
            mu, sd = X[tr].mean(0), np.maximum(X[tr].std(0), 0.05)  # floor: units pinned at ±1 in one fold must not explode in the other
            Z = (X - mu) / sd; Z1 = np.column_stack([Z, np.ones(len(y))])
            A = Z1[tr].T @ Z1[tr] + ridge * len(y[tr]) * np.eye(Z1.shape[1]); A[-1, -1] -= ridge * len(y[tr])
            b = np.linalg.solve(A, Z1[tr].T @ y[tr]); res[~tr] = y[~tr] - Z1[~tr] @ b
        return res
    r0 = oos(base); r1 = oos(np.column_stack([base, extra]))
    return 1 - r1.var() / r0.var(), 1 - r0.var() / y.var()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--nn", required=True); ap.add_argument("--every", type=int, default=12)
    a = ap.parse_args()
    net = Net(*parse_nn_cpp(a.nn)); data = load(a.csv, a.every)
    rows = defaultdict(list); fid = []
    for s, A in data.items():
        X = A[:, 1:]; out, H = net.run(X)
        fid.append(1 - ((out[:, 0] - A[:, 0]) ** 2).mean() / max(A[:, 0].var(), 1e-9))
        q = X[:, GYRO_Q] * GS
        d = [np.column_stack([X[:, TGX[k]], X[:, TGY[k]], X[:, TGZ[k]]]) for k in range(6)]
        geo1 = -np.cross(d[NOW], d[NOW] - d[L1]) / DT      # omega_perp over (t-1, t]
        geo2 = -np.cross(d[L1], d[L1] - d[L2]) / DT        # omega_perp over (t-2, t-1]
        lin = X[:, list(TGX) + list(TGY) + list(TGZ) + list(DS)]
        for t in range(3, len(X)):
            rows["q_now"].append(q[t]); rows["q1"].append(q[t - 1]); rows["q2"].append(q[t - 2])
            rows["lin"].append(lin[t]); rows["geo"].append(np.concatenate([geo1[t], geo2[t]])); rows["h"].append(H[t - 1]); rows["grp"].append(s)
    R = {k: np.array(v) for k, v in rows.items()}
    print(f"{os.path.basename(a.csv)} with {os.path.basename(a.nn)}: {len(data)} scenarios/spans, {len(R['q1'])} ticks, "
          f"replay fidelity R² median {np.median(fid):.4f}")
    base = R["q_now"][:, None]
    targets = {"q(t-1) = rate 50 ms ago": R["q1"], "q(t-2) = rate 100 ms ago": R["q2"], "dq = q(t-1) - q(t-2), the derivative": R["q1"] - R["q2"]}
    sources = [("bearing history, linear (24 slots)", R["lin"]),
               ("bearing history, exact geometric decode", R["geo"]),
               ("recurrent state h(t-1) (16)", R["h"]),
               ("bearing linear + geometric + h(t-1)", np.column_stack([R["lin"], R["geo"], R["h"]]))]
    print(f"  partial R² beyond the gyro-now input (two-fold by scenario/span, ridge); 1.0 = fully recoverable, 0 = not there")
    print(f"  {'source':42s} " + " ".join(f"{k.split(' =')[0].split(',')[0]:>10s}" for k in targets))
    for name, Xs in sources:
        vals = [partial_r2(y, base, Xs, R["grp"])[0] for y in targets.values()]
        print(f"  {name:42s} " + " ".join(f"{v:10.3f}" for v in vals))
    print("  (for reference, R² of each target from the gyro-now input alone: " +
          ", ".join(f"{k.split(' =')[0].split(',')[0]} {partial_r2(y, base, base, R['grp'])[1]:.3f}" for k, y in targets.items()) + ")")


if __name__ == "__main__":
    sys.exit(main())
