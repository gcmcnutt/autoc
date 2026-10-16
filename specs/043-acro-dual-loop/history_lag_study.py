#!/usr/bin/env python3
"""
history_lag_study.py — WHICH SIGNALS SHOULD CARRY HISTORY, AND AT WHAT LAGS? (043, 2026-10-15)

Operator: give gyro (and accel?) a history like the target's — but what lag set, given the airframe's ~2.5–3 Hz short period
and the 20 Hz loop? And can the recurrent layer stand in for a history of the policy's own previous command?

Criterion. A command issued at tick t meets the airframe 100–150 ms later (cmd→gyro peak correlation at +2 ticks in sim,
+2–3 in flight). To damp the cycle the policy must know where the body rate WILL BE when its command lands. So each candidate
input set is scored by out-of-sample R² of a linear predictor of q(t+k) [and p(t+k)], k = 2, 3 ticks, built only from what is
available BEFORE the tick-t decision: sensor samples at t and earlier, and the policy's own commands at t−1 and earlier.
Linear R² is a floor on what the network can extract (it is nonlinear), but it ranks lag sets honestly, and it is the same
measure in sim and in flight.

Validation: flight = fit on one 10-11 flight, test on the other, both directions averaged (xiao 20 Hz logs = the actual NN
inputs). Sim = fit on even scenarios, test on odd, both directions averaged (dmp tick CSV).

Memory decode (--nn genome.cpp with a sim CSV): replay the genome exactly (nn_replay_probe.Net), then R² of decoding the
policy's command at t−j (pre-tanh) from the recurrent state h_{t−1} — the state the recurrent layer actually receives at t.
j = 1 is exact by construction (o_{t−1} = tanh(W3 h_{t−1} + b3)); j ≥ 2 is what the memory has chosen to retain.

Usage: history_lag_study.py --flight f1.csv f2.csv --sim tick.csv [--nn genome.cpp]
"""
import argparse, csv, os, sys
import numpy as np
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FLIGHT_MAP = {"span_id": "S", "gyro_p": "P", "gyro_q": "Q", "gyro_r": "R", "accel_x": "AX", "accel_y": "AY", "accel_z": "AZ",
              "out_pitch": "UPT", "out_roll": "URL"}
SIM_MAP = {"scenario": "S", "gyrP": "P", "gyrQ": "Q", "gyrR": "R", "acX": "AX", "acY": "AY", "acZ": "AZ", "out_pt": "UPT", "out_rl": "URL"}


def load(path):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    m = FLIGHT_MAP if "gyro_q" in h else SIM_MAP
    idx = {v: h.index(k) for k, v in m.items()}
    spans = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        spans[int(float(row[idx["S"]]))].append([float(row[idx[c]]) for c in ("P", "Q", "R", "AX", "AY", "AZ", "UPT", "URL")])
    cols = {c: i for i, c in enumerate(("P", "Q", "R", "AX", "AY", "AZ", "UPT", "URL"))}
    # departed flights (a tumble, body rate > 15 rad/s = 2.5 NN units) are not the regime the policy controls, and a few of them
    # dominate any least-squares fit — excluded, and counted
    out = {s: np.array(v) for s, v in spans.items() if len(v) > 60}
    keep = {s: A for s, A in out.items() if np.abs(A[:, :3]).max() <= 2.5}
    if len(keep) < len(out): print(f"  ({path.split('/')[-1]}: {len(out) - len(keep)} departed scenario(s) excluded)")
    return keep, cols


def design(spans, cols, feats, target, k):
    maxlag = max(l for _, l in feats)
    X, y = [], []
    for A in spans.values():
        n = len(A)
        if n <= maxlag + k + 1: continue
        t = np.arange(maxlag, n - k)
        X.append(np.column_stack([A[t - l, cols[c]] for c, l in feats])); y.append(A[t + k, cols[target]])
    return np.concatenate(X), np.concatenate(y)


def fit_test(tr, te, cols, feats, target, k):
    Xa, ya = design(tr, cols, feats, target, k); Xb, yb = design(te, cols, feats, target, k)
    Xa1 = np.column_stack([Xa, np.ones(len(ya))]); b, *_ = np.linalg.lstsq(Xa1, ya, rcond=None)
    pred = np.column_stack([Xb, np.ones(len(yb))]) @ b
    return 1 - ((yb - pred) ** 2).mean() / yb.var()


def cv(groups, cols, feats, target, k):
    a, b = groups
    return 0.5 * (fit_test(a, b, cols, feats, target, k) + fit_test(b, a, cols, feats, target, k))


def lagset(ch, ticks): return [(ch, l) for l in ticks]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--flight", nargs=2, required=True); ap.add_argument("--sim", required=True)
    ap.add_argument("--nn", default=None); a = ap.parse_args()
    f1, cols = load(a.flight[0]); f2, _ = load(a.flight[1])
    sim, scols = load(a.sim)
    assert cols == scols
    keys = sorted(sim); simg = ({s: sim[s] for s in keys[0::2]}, {s: sim[s] for s in keys[1::2]})
    flg = (f1, f2)
    print(f"flight: {len(f1)}+{len(f2)} spans, {sum(len(v) for v in f1.values())}+{sum(len(v) for v in f2.values())} ticks; "
          f"sim: {len(sim)} scenarios, {sum(len(v) for v in sim.values())} ticks")

    for axis, rate, acc, cmd in (("PITCH", "Q", "AZ", "UPT"), ("ROLL", "P", "AY", "URL")):
        sets = [
            ("rate now only (today)", lagset(rate, [0])),
            ("rate 0, 50", lagset(rate, [0, 1])),
            ("rate 0, 50, 100", lagset(rate, [0, 1, 2])),
            ("rate 0, 50, 100, 150", lagset(rate, [0, 1, 2, 3])),
            ("rate 0 ... 200 (5 uniform)", lagset(rate, [0, 1, 2, 3, 4])),
            ("rate 0 ... 300 (7 uniform)", lagset(rate, [0, 1, 2, 3, 4, 5, 6])),
            ("rate 0, 100, 200", lagset(rate, [0, 2, 4])),
            ("rate 0, 50, 100, 200", lagset(rate, [0, 1, 2, 4])),
            ("rate on the target's log set 0,50,100,200,400,800", lagset(rate, [0, 1, 2, 4, 8, 16])),
            ("rate now + own cmd t-1 (memory has this exactly)", lagset(rate, [0]) + lagset(cmd, [1])),
            ("rate now + own cmd t-1..t-3", lagset(rate, [0]) + lagset(cmd, [1, 2, 3])),
            ("rate 0..150 + own cmd t-1", lagset(rate, [0, 1, 2, 3]) + lagset(cmd, [1])),
            ("rate 0..150 + own cmd t-1..t-3", lagset(rate, [0, 1, 2, 3]) + lagset(cmd, [1, 2, 3])),
            (f"rate 0..150 + {'accel z' if acc == 'AZ' else 'accel y'} 0..150", lagset(rate, [0, 1, 2, 3]) + lagset(acc, [0, 1, 2, 3])),
            (f"rate 0..150 + {'accel z' if acc == 'AZ' else 'accel y'} now", lagset(rate, [0, 1, 2, 3]) + lagset(acc, [0])),
            ("rate 0..150 + all 3 rates now + all 3 accels now", lagset(rate, [0, 1, 2, 3]) + [(c, 0) for c in ("P", "Q", "R", "AX", "AY", "AZ") if c != rate]),
            ("all 3 rates 0..150 + accel z 0..150 + cmd t-1", sum((lagset(c, [0, 1, 2, 3]) for c in ("P", "Q", "R")), []) + lagset("AZ", [0, 1, 2, 3]) + lagset(cmd, [1])),
        ]
        print(f"\n{axis}: out-of-sample R² of predicting the body {('pitch' if rate == 'Q' else 'roll')} rate when a command issued now lands")
        print(f"  {'inputs available before the decision':52s} {'flight +100ms':>13s} {'flight +150ms':>13s} {'sim +100ms':>11s} {'sim +150ms':>11s}")
        for name, feats in sets:
            v = [cv(flg, cols, feats, rate, 2), cv(flg, cols, feats, rate, 3), cv(simg, cols, feats, rate, 2), cv(simg, cols, feats, rate, 3)]
            print(f"  {name:52s} {v[0]:13.3f} {v[1]:13.3f} {v[2]:11.3f} {v[3]:11.3f}")

    # Whole candidate input proposals. Every set includes the policy's command at t-1, because the recurrent layer
    # holds it exactly by construction; "today" = the six craft sensors as they are now (instantaneous).
    U3, U4, LOG = [0, 1, 2], [0, 1, 2, 3], [0, 1, 2, 4, 8, 16]
    allnow = [(c, 0) for c in ("P", "Q", "R", "AX", "AY", "AZ")]
    def hist(chs, lags): return sum((lagset(c, lags) for c in chs), [])
    props = [
        ("today: 6 sensors now", 0, allnow, [1]),
        ("A: gyro p,q,r at 0/50/100", 6, hist("PQR", U3) + [(c, 0) for c in ("AX", "AY", "AZ")], [1]),
        ("B: A + accel z at 0/50/100", 8, hist("PQR", U3) + hist(["AZ"], U3) + [("AX", 0), ("AY", 0)], [1]),
        ("C: gyro p,q,r + accel z at 0/50/100/150", 12, hist("PQR", U4) + hist(["AZ"], U4) + [("AX", 0), ("AY", 0)], [1]),
        ("D: all 6 on the target's log lags", 30, hist(["P", "Q", "R", "AX", "AY", "AZ"], LOG), [1]),
        ("E: B + explicit own cmd t-2, t-3", 12, hist("PQR", U3) + hist(["AZ"], U3) + [("AX", 0), ("AY", 0)], [1, 2, 3]),
    ]
    print("\nWHOLE PROPOSALS (each also has own cmd t-1 = what memory holds exactly); out-of-sample R\u00b2")
    print(f"  {'proposal':44s} {'+slots':>6s} | {'pitch flt':>9s} {'+150':>6s} {'pitch sim':>9s} {'+150':>6s} | {'roll flt':>8s} {'+150':>6s} {'roll sim':>8s} {'+150':>6s}")
    for name, nslots, feats, cmdlags in props:
        row = []
        for rate, cmd in (("Q", "UPT"), ("P", "URL")):
            fs = feats + lagset(cmd, cmdlags)
            row += [cv(flg, cols, fs, rate, 2), cv(flg, cols, fs, rate, 3), cv(simg, cols, fs, rate, 2), cv(simg, cols, fs, rate, 3)]
        print(f"  {name:44s} {nslots:6d} | {row[0]:9.3f} {row[1]:6.3f} {row[2]:9.3f} {row[3]:6.3f} | {row[4]:8.3f} {row[5]:6.3f} {row[6]:8.3f} {row[7]:6.3f}")

    if a.nn:
        from nn_replay_probe import Net, parse_nn_cpp
        net = Net(*parse_nn_cpp(a.nn))
        r = csv.reader(open(a.sim)); h = next(r); i0 = h.index("rbHhd") + 1; iS = h.index("scenario")
        X = defaultdict(list)
        for row in r:
            if len(row) < len(h): continue
            s = int(row[iS])
            if s % 12 == 0: X[s].append([float(v) for v in row[i0:i0 + 45]])
        Hs, Os = [], []
        for s, rows in X.items():
            out, H = net.run(np.array(rows, np.float32)); Hs.append(H); Os.append(out)
        print(f"\nMEMORY DECODE ({os.path.basename(a.nn)}, {len(Hs)} scenarios): R² of the policy's own command at t-j, read linearly from h(t-1)")
        print(f"  {'j (ticks back)':16s} " + " ".join(f"{j:>8d}" for j in (1, 2, 3, 4, 6)))
        for ax, name in ((0, "pitch cmd"), (1, "roll cmd")):
            vals = []
            for j in (1, 2, 3, 4, 6):
                Xd, yd = [], []
                for H, O in zip(Hs, Os):
                    # at tick t the recurrent layer receives h(t-1) = H[t-1]; target = pre-tanh command at t-j
                    t = np.arange(max(j, 1), len(H))
                    Xd.append(H[t - 1]); yd.append(np.arctanh(np.clip(O[t - j, ax], -0.999999, 0.999999)))
                Xd = np.concatenate(Xd); yd = np.concatenate(yd)
                half = len(yd) // 2; X1 = np.column_stack([Xd, np.ones(len(yd))])
                b, *_ = np.linalg.lstsq(X1[:half], yd[:half], rcond=None); p = X1[half:] @ b
                vals.append(1 - ((yd[half:] - p) ** 2).mean() / yd[half:].var())
            print(f"  {name:16s} " + " ".join(f"{v:8.3f}" for v in vals))


if __name__ == "__main__":
    sys.exit(main())
