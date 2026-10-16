#!/usr/bin/env python3
"""
sensor_alias_budget.py — HOW MUCH OF THE POLICY'S 20 Hz SENSOR SAMPLES IS FOLDED-DOWN CONTENT FROM ABOVE 10 Hz? (043, 2026-10-15)

Question (operator, while choosing history slots for gyro/accel): the NN samples the FC's filtered gyro (gyroADCf, PT1 25 Hz +
dynamic notch >= 30 Hz) and accel (accADCf, biquad 15 Hz) ONCE per 50 ms over MSP — a point sample, no anti-alias filter at
20 Hz. Anything between 10 Hz and the FC filters' roll-off folds into 0–10 Hz. A history slot lets the network form differences,
which are high-pass (a one-tick difference has gain 2·sin(πf/20): 0.31 at 1 Hz, 1.41 at 5 Hz, 2.0 at 10 Hz), so folded noise
matters more for a history than for a single "now" sample. The sim has no vibration, so whatever folds in flight is a pure
sim-to-real gap.

Method, on the blackbox (≈1 kHz, same filtered signals the xiao reads) restricted to the policy-engaged spans
(flightModeFlags contains MSPRCOVERRIDE):
  - band powers of the 1 kHz signal: 0.25–1, 1–4, 4–10, 10–25, 25–100, >100 Hz
  - point sample at 20 Hz (every 50 ms, at all 50 phases, pooled) vs an IDEAL reference (brick-wall 10 Hz low-pass of the
    same 1 kHz signal, then sampled): the difference is the aliased component the policy actually receives
  - noise-to-signal of k-tick differences (k = 1, 2, 3, 4 → 50…200 ms): var(alias part of Δ_k) / var(ideal Δ_k)
  - the same for a 50 ms BOX AVERAGE then sample (what averaging on the FC over the MSP interval would give; costs 25 ms
    of group delay = 22° at 2.5 Hz)

Usage: sensor_alias_budget.py blackbox.csv [--channels gyroADC[1] gyroADC[0] accSmooth[2] ...]
"""
import argparse, csv, sys
import numpy as np

BANDS = [(0.25, 1), (1, 4), (4, 10), (10, 25), (25, 100), (100, 500)]


def spans(path, chans):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    it, ifm = h.index("time (us)"), h.index("flightModeFlags (flags)")
    idx = [h.index(c) for c in chans]
    out, cur = [], None
    for row in r:
        on = "MSPRCOVERRIDE" in row[ifm]
        if on:
            if cur is None: cur = ([], [])
            cur[0].append(float(row[it]) / 1e6); cur[1].append([float(row[i]) for i in idx])
        elif cur is not None:
            out.append((np.array(cur[0]), np.array(cur[1]))); cur = None
    if cur is not None: out.append((np.array(cur[0]), np.array(cur[1])))
    return out


def resample_uniform(t, x, fs=1000.0):
    tu = np.arange(t[0], t[-1], 1 / fs)
    return tu, np.column_stack([np.interp(tu, t, x[:, j]) for j in range(x.shape[1])])


def band_powers(x, fs):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs); P = np.abs(X) ** 2
    tot = P[f >= 0.25].sum()
    return [P[(f >= a) & (f < b)].sum() / tot for a, b in BANDS]


def brickwall(x, fs, fc):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs); X[f > fc] = 0
    return np.fft.irfft(X, n=len(x)) + x.mean()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv")
    ap.add_argument("--channels", nargs="+", default=["gyroADC[1]", "gyroADC[0]", "gyroADC[2]", "gyroRaw[1]", "accSmooth[0]", "accSmooth[1]", "accSmooth[2]"])
    a = ap.parse_args(); FS = 1000.0; STEP = 50  # samples per 50 ms tick at 1 kHz
    sp = spans(a.csv, a.channels)
    print(f"{len(sp)} engaged spans, {sum(t[-1]-t[0] for t, _ in sp):.1f} s")
    print("\nshare of signal power (>0.25 Hz) by band, 1 kHz blackbox, engaged spans:")
    print(f"  {'channel':14s} " + " ".join(f"{a_:>5g}-{b_:<5g}" for a_, b_ in BANDS))
    res = {}
    for j, ch in enumerate(a.channels):
        bp = np.zeros(len(BANDS)); w = 0
        diffs = {k: {"ps_alias": 0.0, "ideal": 0.0, "box_alias": 0.0, "box_ideal": 0.0} for k in (1, 2, 3, 4)}
        inband = {"ps": [0.0, 0.0], "box": [0.0, 0.0]}  # [alias var, ideal var] for the samples themselves
        for t, x in sp:
            tu, xu = resample_uniform(t, x); s = xu[:, j]
            if len(s) < 4000: continue
            bp += np.array(band_powers(s, FS)) * len(s); w += len(s)
            ideal = brickwall(s, FS, 10.0)
            box = np.convolve(s, np.ones(STEP) / STEP, mode="same")
            box_ideal = brickwall(box, FS, 10.0)
            for ph in range(0, STEP, 5):
                ps, idl = s[ph::STEP], ideal[ph::STEP]
                bx, bxi = box[ph::STEP], box_ideal[ph::STEP]
                inband["ps"][0] += ((ps - idl) ** 2).sum(); inband["ps"][1] += ((idl - idl.mean()) ** 2).sum()
                inband["box"][0] += ((bx - bxi) ** 2).sum(); inband["box"][1] += ((bxi - bxi.mean()) ** 2).sum()
                for k in (1, 2, 3, 4):
                    d_ps, d_id = ps[k:] - ps[:-k], idl[k:] - idl[:-k]
                    d_bx, d_bi = bx[k:] - bx[:-k], bxi[k:] - bxi[:-k]
                    diffs[k]["ps_alias"] += ((d_ps - d_id) ** 2).sum(); diffs[k]["ideal"] += (d_id ** 2).sum()
                    diffs[k]["box_alias"] += ((d_bx - d_bi) ** 2).sum(); diffs[k]["box_ideal"] += (d_bi ** 2).sum()
        bp /= max(w, 1); res[ch] = (bp, inband, diffs)
        print(f"  {ch:14s} " + " ".join(f"{100*v:10.1f}%" for v in bp))
    print("\naliased-to-true variance ratio in what the policy receives (point sample every 50 ms vs 50 ms box average):")
    print(f"  {'channel':14s} {'sample':>8s} {'Δ50ms':>8s} {'Δ100ms':>8s} {'Δ150ms':>8s} {'Δ200ms':>8s}   | box-avg: {'sample':>7s} {'Δ50':>6s} {'Δ100':>6s} {'Δ150':>6s} {'Δ200':>6s}")
    for ch, (bp, inband, diffs) in res.items():
        ps = inband["ps"][0] / max(inband["ps"][1], 1e-12); bx = inband["box"][0] / max(inband["box"][1], 1e-12)
        dps = [diffs[k]["ps_alias"] / max(diffs[k]["ideal"], 1e-12) for k in (1, 2, 3, 4)]
        dbx = [diffs[k]["box_alias"] / max(diffs[k]["box_ideal"], 1e-12) for k in (1, 2, 3, 4)]
        print(f"  {ch:14s} {ps:8.3f} " + " ".join(f"{v:8.3f}" for v in dps) + f"   |          {bx:7.3f} " + " ".join(f"{v:6.3f}" for v in dbx))
    print("\n(ratio 0.10 = folded noise carries 10 % of the variance of the true band-limited quantity; 1.0 = as large as the signal)")


if __name__ == "__main__":
    sys.exit(main())
