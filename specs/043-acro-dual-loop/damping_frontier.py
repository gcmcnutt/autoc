#!/usr/bin/env python3
"""
damping_frontier.py — WHAT SMOOTHNESS DO THE T121 SLOTS BUY, AT EQUAL TRACKING? (043, 2026-10-15)

A pitch-plane closed-loop model built from the t6-plant sim data, used to bound what a controller can achieve with and without
the proposed gyro-history slots. It does not predict what evolution will find; it predicts how far the achievable
tracking-vs-smoothness frontier moves when the information is added.

Model (all identified or replayed from dmp tick CSVs on the t6 plant):
  plant     ARX(4,5) at 20 Hz:  q_t = Σ a_i q_{t-i} + Σ b_j u_{t-j} + c + w_t      (q = gyrQ·6 rad/s, u = out_pt ∈ [-1, 1])
  air       w_t = the ARX residual of a recorded scenario (turbulence, coupling, everything the commands do not explain)
  demand    r_t = the attitude the policy actually flew in that scenario, low-passed at 0.6 Hz (θ = ∫q dt): the task's slow
            pitch maneuvering; everything faster is the policy's own cycle and is NOT demand
  attitude  θ_{t+1} = θ_t + q_t·dt;   tracking error e_t = r_t - θ_t
  bearing   what the policy reads is e_t + n_t: n_t is white "confound" noise standing for what makes the real bearing history a
            poor rate sensor (relative translation, 3-D geometry). σ_n is calibrated so that, on the recorded data, the linear
            partial R² of q(t-1) from the bearing history beyond q(t) matches a target value (--decodability, measured by
            rate_history_availability.py: 0.27 for the t6 gen-370 elite, 0.64 for g488).
Controllers (linear, output clipped to ±1 = the ±240 °/s cap), nested so a bigger set can never do worse:
  A  today:    bearing at 0/50/100/200/400/800 ms, q now, own command t-1 (memory holds it exactly)
  B  + T121:   A + q at 50 and 100 ms
  C  + cmds:   A + own command at t-2, t-3
  D  both:     B + C
Gains: cross-entropy search on J = mean(e²)/(10°)² + λ·mean(Δu²)/0.1², common random numbers, training scenarios; B, C start from A's
optimum, D from B's. Reported on held-out scenarios: tracking RMS, pitch-rate RMS in 1.5–4 Hz, Δu RMS, flips/s, % at the cap.

Usage: damping_frontier.py tick1.csv [tick2.csv ...] [--decodability 0.27 0.64] [--lambdas ...]
"""
import argparse, csv, sys
import numpy as np
from collections import defaultdict

DT, FS, GS = 0.05, 20.0, 6.0
NA, NB = 4, 5
LAGS_E = [0, 1, 2, 4, 8, 16]


def load(paths):
    segs = []
    for path in paths:
        r = csv.reader(open(path)); h = next(r)
        iS, iU, iQ = h.index("scenario"), h.index("out_pt"), len(h) - 1 - h[::-1].index("gyrQ")
        iX, iZ = h.index("tgX0"), h.index("tgZ0")
        sc = defaultdict(list)
        for row in r:
            if len(row) < len(h): continue
            sc[int(row[iS])].append((float(row[iU]), float(row[iQ]) * GS, float(row[iX]), float(row[iZ])))
        dropped = 0
        for s, v in sc.items():
            A = np.array(v)
            if len(A) < 300: continue
            if np.abs(A[:, 1]).max() > 15.0: dropped += 1; continue   # departed flight (tumble): not the controlled regime
            segs.append(A)
        if dropped: print(f"  ({path.split('/')[-1]}: {dropped} departed scenario(s) excluded)")
    return segs


FC_DEMAND = 0.6   # Hz: the task's pitch demand = the slow part of the attitude the policy actually flew


def lowpass_trend(x, fc):
    """Zero-phase brick-wall low-pass that preserves a linear trend (spirals ramp θ without bound)."""
    n = len(x); t = np.arange(n); p = np.polyfit(t, x, 1); d = x - np.polyval(p, t)
    D = np.fft.rfft(np.concatenate([d, d[::-1]])); f = np.fft.rfftfreq(2 * n, 1 / FS); D[f > fc] = 0
    return np.fft.irfft(D, n=2 * n)[:n] + np.polyval(p, t)


def fit_arx(segs):
    X, y = [], []
    for A in segs:
        u, q = A[:, 0], A[:, 1]; m = max(NA, NB)
        t = np.arange(m, len(A))
        X.append(np.column_stack([q[t - i] for i in range(1, NA + 1)] + [u[t - j] for j in range(1, NB + 1)] + [np.ones(len(t))]))
        y.append(q[t])
    X = np.concatenate(X); y = np.concatenate(y); th, *_ = np.linalg.lstsq(X, y, rcond=None)
    return th[:NA], th[NA:NA + NB], th[-1], 1 - ((y - X @ th) ** 2).mean() / y.var()


def residuals_and_demand(segs, a, b, c):
    out = []
    for A in segs:
        u, q = A[:, 0], A[:, 1]; m = max(NA, NB); n = len(A)
        w = np.zeros(n)
        for t in range(m, n):
            w[t] = q[t] - (a @ q[t - NA:t][::-1] + b @ u[t - NB:t][::-1] + c)
        th = np.concatenate([[0.0], np.cumsum(q[:-1] * DT)])
        r = lowpass_trend(th, FC_DEMAND)
        # deterministic share: simulate the ARX from the recorded commands alone (w = 0)
        qs = q.copy()
        for t in range(m, n):
            qs[t] = a @ qs[t - NA:t][::-1] + b @ u[t - NB:t][::-1] + c
        out.append((w[m:], r[m:], q[m:], qs[m:]))
    return out


def frf(a, b, f):
    z = np.exp(-2j * np.pi * f / FS)
    return (sum(b[j] * z ** (j + 1) for j in range(NB))) / (1 - sum(a[i] * z ** (i + 1) for i in range(NA)))


def calibrate_noise(data, target):
    """σ_n such that partial R² of q(t-1) from bearing history beyond q(t) equals `target` on the recorded data."""
    def pr2(sig, seed=7):
        rng = np.random.default_rng(seed); Y, B, E = [], [], []
        for w, r, q, _ in data[:40]:
            th = np.concatenate([[0.0], np.cumsum(q[:-1] * DT)]); e = r - th + sig * rng.standard_normal(len(r))
            t = np.arange(17, len(r))
            Y.append(q[t - 1]); B.append(q[t]); E.append(np.column_stack([e[t - l] for l in LAGS_E]))
        Y, B, E = np.concatenate(Y), np.concatenate(B)[:, None], np.concatenate(E)
        def res(X):
            X1 = np.column_stack([X, np.ones(len(Y))]); bb, *_ = np.linalg.lstsq(X1, Y, rcond=None); return Y - X1 @ bb
        return 1 - res(np.column_stack([B, E])).var() / res(B).var()
    lo, hi = 1e-5, 0.5
    if pr2(lo) < target: return 0.0, pr2(lo)
    for _ in range(40):
        mid = np.sqrt(lo * hi)
        if pr2(mid) > target: lo = mid
        else: hi = mid
    return np.sqrt(lo * hi), pr2(np.sqrt(lo * hi))


FEATS = {
    "A": ["e0", "e1", "e2", "e4", "e8", "e16", "q0", "u1"],
    "B": ["e0", "e1", "e2", "e4", "e8", "e16", "q0", "u1", "q1", "q2"],
    "C": ["e0", "e1", "e2", "e4", "e8", "e16", "q0", "u1", "u2", "u3"],
    "D": ["e0", "e1", "e2", "e4", "e8", "e16", "q0", "u1", "q1", "q2", "u2", "u3"],
}
SCALE = {"e": 0.2, "q": 2.0, "u": 1.0}


def simulate(K, feats, segs, a, b, c, sig, lam, seed, metrics=False):
    """K: (P, d) gains. segs: list of (w, r) arrays. Returns J per candidate (and metrics of candidate 0 if asked)."""
    P, d = K.shape; S = len(segs); T = min(len(w) for w, r, *_ in segs); T = min(T, 600)
    W = np.stack([s[0][:T] for s in segs]); R = np.stack([s[1][:T] for s in segs])
    rng = np.random.default_rng(seed); N = sig * rng.standard_normal((S, T))
    L = P * S
    Wl, Rl, Nl = np.tile(W, (P, 1)), np.tile(R, (P, 1)), np.tile(N, (P, 1))
    Kl = np.repeat(K, S, axis=0)
    qh = np.zeros((L, NA)); uh = np.zeros((L, NB)); th = Rl[:, 0].copy(); eh = np.zeros((L, 17))
    q = np.zeros(L); J_e = np.zeros(L); J_du = np.zeros(L); uprev = np.zeros(L)
    rec_q, rec_u, rec_e = [], [], []
    for t in range(T):
        e_true = Rl[:, t] - th
        eh = np.roll(eh, 1, axis=1); eh[:, 0] = e_true + Nl[:, t]
        f = []
        for name in feats:
            k, lag = name[0], int(name[1:])
            if k == "e": f.append(eh[:, lag] / SCALE["e"])
            elif k == "q": f.append((q if lag == 0 else qh[:, lag - 1]) / SCALE["q"])
            else: f.append(uh[:, lag - 1])
        u = np.clip(np.einsum("ld,dl->l", Kl, np.array(f)), -1.0, 1.0)
        if t > 20:
            J_e += e_true ** 2; J_du += (u - uprev) ** 2
        uprev = u
        uh = np.roll(uh, 1, axis=1); uh[:, 0] = u
        qh = np.roll(qh, 1, axis=1); qh[:, 0] = q
        q = qh @ a + uh @ b + c + Wl[:, t]
        th = th + qh[:, 0] * DT
        if metrics:
            rec_q.append(q[:S].copy()); rec_u.append(u[:S].copy()); rec_e.append(e_true[:S].copy())
    n = T - 21; varr = Rl.var(axis=1) + 1e-9
    J = (J_e / n / np.radians(10.0) ** 2 + lam * J_du / n / 0.01).reshape(P, S).mean(axis=1)
    if not metrics: return J
    Q, U, E = np.array(rec_q).T, np.array(rec_u).T, np.array(rec_e).T
    def band_rms(x, lo, hi):
        X = np.fft.rfft(x - x.mean(), axis=1); f = np.fft.rfftfreq(x.shape[1], 1 / FS); X[:, (f < lo) | (f > hi)] = 0
        return np.sqrt((np.fft.irfft(X, n=x.shape[1], axis=1) ** 2).mean())
    return J, dict(track_deg=np.degrees(np.sqrt((E[:, 21:] ** 2).mean())), qband=np.degrees(band_rms(Q[:, 21:], 1.5, 4.0)),
                   du=np.sqrt((np.diff(U[:, 21:], axis=1) ** 2).mean()), flips=(np.diff(np.sign(U[:, 21:]), axis=1) != 0).sum(1).mean() / ((T - 21) * DT),
                   cap=100 * (np.abs(U[:, 21:]) > 0.9).mean())


def cem(feats, train, a, b, c, sig, lam, init=None, init_sd=None, iters=60, P=96, elite=12, seed=0):
    d = len(feats); rng = np.random.default_rng(seed)
    mu = np.zeros(d) if init is None else init.copy()
    sd = np.full(d, 0.6) if init_sd is None else init_sd.copy()
    best, bestJ = mu.copy(), np.inf
    for it in range(iters):
        K = mu + sd * rng.standard_normal((P, d)); K[0] = mu
        J = simulate(K, feats, train, a, b, c, sig, lam, seed=100 + it)
        o = np.argsort(J); el = K[o[:elite]]
        if J[o[0]] < bestJ: bestJ, best = J[o[0]], K[o[0]].copy()
        mu = 0.3 * mu + 0.7 * el.mean(0); sd = 0.3 * sd + 0.7 * el.std(0) + 0.01
    return best, bestJ


def nested(feats, train, a, b, c, sig, lam, parent, n_new, seed):
    """Best of (a) a gentle search around the parent's optimum — inherited gains spread 0.08, new gains 0.4 — and (b) a search
    from scratch. Training J decides; (a) can never be worse than the parent, so a bigger set never loses by construction."""
    kn, jn = cem(feats, train, a, b, c, sig, lam, init=np.concatenate([parent, np.zeros(n_new)]),
                 init_sd=np.concatenate([np.full(len(parent), 0.08), np.full(n_new, 0.4)]), seed=seed)
    ks, js = cem(feats, train, a, b, c, sig, lam, seed=seed + 50)
    return (kn, jn) if jn <= js else (ks, js)


def disturbance_floor(test, a, b, c):
    """1.5–4 Hz pitch-rate RMS the air alone produces through the plant with the command held at zero (no feedback at all)."""
    out = []
    for w, r in test:
        T = min(len(w), 600); q = np.zeros(T)
        for t in range(NA, T):
            q[t] = a @ q[t - NA:t][::-1] + c + w[t]
        X = np.fft.rfft(q[21:] - q[21:].mean()); f = np.fft.rfftfreq(len(q[21:]), 1 / FS); X[(f < 1.5) | (f > 4.0)] = 0
        out.append((np.fft.irfft(X, n=len(q[21:])) ** 2).mean())
    return np.degrees(np.sqrt(np.mean(out)))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv", nargs="+")
    ap.add_argument("--decodability", nargs="+", type=float, default=[0.27, 0.64])
    ap.add_argument("--lambdas", nargs="+", type=float, default=[0.0, 0.1, 0.3, 1.0, 3.0])
    a_ = ap.parse_args()
    segs = load(a_.csv)
    a, b, c, r2 = fit_arx(segs)
    data = residuals_and_demand(segs, a, b, c)
    def bp(x, lo=1.5, hi=4.0):
        X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / FS); X[(f < lo) | (f > hi)] = 0
        return np.fft.irfft(X, n=len(x))
    det = np.median([1 - ((bp(q) - bp(qs)) ** 2).mean() / bp(q).var() for w, r, q, qs in data])
    print(f"{len(segs)} scenarios. ARX({NA},{NB}) one-step R² {r2:.3f}; share of the 1.5-4 Hz pitch rate explained by the policy's own "
          f"command history alone (w = 0): {det:.2f} — the rest is the air and the airframe ringing")
    print("plant |q/setpoint| (setpoint = u·240 °/s):  " + "  ".join(f"{f:g} Hz {abs(frf(a, b, f)) / np.radians(240):.2f} ∠{np.degrees(np.angle(frf(a, b, f))):.0f}°" for f in (0.5, 1, 2, 2.5, 3, 4)))
    rng = np.random.default_rng(1); idx = rng.permutation(len(data))
    train = [(data[i][0], data[i][1]) for i in idx[:24]]; test = [(data[i][0], data[i][1]) for i in idx[24:56]]
    print(f"disturbance floor: the air alone, no feedback, gives {disturbance_floor(test, a, b, c):.1f} °/s RMS of 1.5-4 Hz pitch rate")
    for target in a_.decodability:
        sig, got = calibrate_noise(data, target)
        print(f"\n=== bearing as a rate sensor: decodability {target:.2f} (σ_n {np.degrees(sig):.2f}° → measured {got:.2f}) ===")
        print(f"  {'λ':>5s} {'set':4s} {'track °':>8s} {'q 1.5-4 Hz °/s':>15s} {'Δu rms':>7s} {'flips/s':>8s} {'cap %':>6s}")
        curves = {k: [] for k in "ABCD"}
        for lam in a_.lambdas:
            kA, _ = cem(FEATS["A"], train, a, b, c, sig, lam)
            kB, _ = nested(FEATS["B"], train, a, b, c, sig, lam, kA, 2, seed=1)
            kC, _ = nested(FEATS["C"], train, a, b, c, sig, lam, kA, 2, seed=2)
            kD, _ = nested(FEATS["D"], train, a, b, c, sig, lam, kB, 2, seed=3)
            for name, k in (("A", kA), ("B", kB), ("C", kC), ("D", kD)):
                _, m = simulate(k[None, :], FEATS[name], test, a, b, c, sig, lam, seed=999, metrics=True)
                curves[name].append((m["track_deg"], m["qband"], m["du"], m["flips"]))
                print(f"  {lam:5.2f} {name:4s} {m['track_deg']:8.2f} {m['qband']:15.1f} {m['du']:7.3f} {m['flips']:8.2f} {m['cap']:6.1f}")
            sys.stdout.flush()
        # matched tracking: interpolate each set's smoothness at common tracking errors
        lo = max(min(p[0] for p in v) for v in curves.values()); hi = min(max(p[0] for p in v) for v in curves.values())
        print(f"  at matched tracking error (interpolated along λ):")
        print(f"  {'track °':>8s} | {'q 1.5-4 Hz °/s  A / B / C / D':>32s} | {'B vs A':>7s} {'D vs A':>7s} | {'Δu rms  A / B / C / D':>26s}")
        for tr in np.linspace(lo, hi, 4):
            qb, du = {}, {}
            for k, v in curves.items():
                v = sorted(v); x = [p[0] for p in v]
                qb[k] = np.interp(tr, x, [p[1] for p in v]); du[k] = np.interp(tr, x, [p[2] for p in v])
            print(f"  {tr:8.2f} | {qb['A']:7.1f} {qb['B']:7.1f} {qb['C']:7.1f} {qb['D']:7.1f} | {100*(qb['B']/qb['A']-1):+6.1f}% {100*(qb['D']/qb['A']-1):+6.1f}% | "
                  f"{du['A']:6.3f} {du['B']:6.3f} {du['C']:6.3f} {du['D']:6.3f}")


if __name__ == "__main__":
    sys.exit(main())
