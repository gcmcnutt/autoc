#!/usr/bin/env python3
"""
candidate_compare.py — FLIGHT-CANDIDATE COMPARISON IN ONE ENVIRONMENT, CALIBRATED TO A FLOWN GENOME (043 T122, 2026-10-18)

Operator: the global fitness does not compare generations — later gens train in harder air (the variation ramp). So: fly every
candidate through the SAME scenarios at the SAME strength (eval tier 0 with one never-trained seed; weights copies with
variation_scale = 1.0, see set_eval_scale.py) and compare pitch control and tracking there. Then map to flight through the one genome
that has both: the flown g488 — its eval numbers here vs its 10-11 flight numbers, computed with the SAME code from the xiao logs.

Per candidate (sim) and per flight (xiao log), per scenario/span then median:
  pitch-rate RMS (°/s, gyro q), 2–3 Hz and 3–5 Hz share of pitch-rate power, pitch command sign flips/s, % of ticks at the cap
  (|out| > 0.9), command p5/p95 in °/s (×240), |a_z| p99 in g (body specific force, includes the 1 g); tracking from the meta:
  pctInStreak = Σ streak_steps / Σ steps, crashes. Sim subsets by the exact wind-speed draw (wind_draws.cc output): all, calm (≤ 3 m/s,
  like the 10-11 days at 1.8–2.9 m/s), windy (≥ 6 m/s).

Usage: candidate_compare.py --sim LABEL:tick.csv:meta.yaml ... --flight LABEL:xiao.csv ... --draws draws.txt [--ref t5-g488]
"""
import argparse, csv, re, sys
import numpy as np
from collections import defaultdict

FS, GS, AS = 20.0, 6.0, 8.0   # tick rate; gyro and accel NN scales (rad/s ÷ 6, g ÷ 8)


def share(x, lo, hi, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < 64: return np.nan
    nseg = min(nseg, (len(x) // 2) * 2); w = np.hanning(nseg)
    P = np.mean([np.abs(np.fft.rfft(x[i:i + nseg] * w)) ** 2 for i in range(0, len(x) - nseg + 1, nseg // 2)], axis=0)
    f = np.fft.rfftfreq(nseg, 1 / FS); tot = P[f > 0.25].sum()
    return 100 * P[(f >= lo) & (f < hi)].sum() / tot if tot > 0 else np.nan


def metrics(u, q_deg, az_g):
    dur = len(u) / FS
    return dict(qrms=float(np.sqrt(np.mean(q_deg ** 2))), s23=share(q_deg, 2, 3), s35=share(q_deg, 3, 5),
                flips=float(np.sum(np.sign(u[1:]) != np.sign(u[:-1])) / dur), cap=100 * float(np.mean(np.abs(u) > 0.9)),
                p5=240 * float(np.percentile(u, 5)), p95=240 * float(np.percentile(u, 95)), az99=float(np.percentile(np.abs(az_g), 99)))


def load_sim(path):
    r = csv.reader(open(path)); h = next(r)
    iS, iU = h.index("scenario"), h.index("out_pt"); iQ = len(h) - 1 - h[::-1].index("gyrQ"); iZ = len(h) - 1 - h[::-1].index("acZ")
    d = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        d[int(row[iS])].append((float(row[iU]), float(row[iQ]), float(row[iZ])))
    return {s: np.array(v) for s, v in d.items() if len(v) >= 64}


def load_flight(path):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    iS, iU, iQ, iZ = h.index("span_id"), h.index("out_pitch"), h.index("gyro_q"), h.index("accel_z")
    d = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        d[int(float(row[iS]))].append((float(row[iU]), float(row[iQ]), float(row[iZ])))
    return {s: np.array(v) for s, v in d.items() if len(v) >= 64}


def read_meta(path):
    out, cur = {}, None
    for line in open(path):
        m = re.match(r"\s*-\s*idx:\s*(\d+)", line)
        if m: cur = int(m.group(1)); out[cur] = {}; continue
        if cur is None: continue
        m = re.match(r"\s*(crash_reason|streak_steps|scenario_seed):\s*(\S+)", line)
        if m: out[cur][m.group(1)] = m.group(2)
        m = re.match(r"\s*steps:\s*(\d+)\s*/\s*(\d+)", line)
        if m: out[cur]["steps"] = float(m.group(1))
    return out


def summarize(per):
    keys = ["qrms", "s23", "s35", "flips", "cap", "p5", "p95", "az99"]
    return {k: float(np.nanmedian([p[k] for p in per])) for k in keys}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--sim", nargs="+", required=True); ap.add_argument("--flight", nargs="*", default=[])
    ap.add_argument("--draws", required=True); ap.add_argument("--ref", default="t5-g488"); a = ap.parse_args()
    speed = {int(l.split()[0]): 8.0 * float(l.split()[5]) for l in open(a.draws)}
    rows = {}
    for spec in a.sim:
        lab, tick, meta = spec.split(":")
        sc, mt = load_sim(tick), read_meta(meta)
        per = {s: metrics(A[:, 0], np.degrees(A[:, 1] * GS), A[:, 2] * AS) for s, A in sc.items()}
        res = {}
        for sub, sel in (("all", lambda s: True), ("calm", lambda s: speed.get(s, 99) <= 3.0), ("windy", lambda s: speed.get(s, 0) >= 6.0)):
            ids = [s for s in per if sel(s)]
            m = summarize([per[s] for s in ids])
            st = sum(float(mt[s].get("streak_steps", 0)) for s in ids if s in mt); stp = sum(mt[s].get("steps", 0) for s in ids if s in mt)
            m["streak"] = 100 * st / max(stp, 1); m["crash"] = sum(1 for s in ids if s in mt and mt[s].get("crash_reason") != "RabbitComplete")
            m["n"] = len(ids); res[sub] = m
        rows[lab] = res
    fl = {}
    for spec in a.flight:
        lab, path = spec.split(":")
        sp = load_flight(path)
        fl[lab] = summarize([metrics(A[:, 0], np.degrees(A[:, 1] * GS), A[:, 2] * AS) for A in sp.values()]); fl[lab]["n"] = len(sp)
    cols = [("qrms", "pitch RMS °/s", "{:.0f}"), ("s23", "2-3 Hz %", "{:.0f}"), ("s35", "3-5 Hz %", "{:.0f}"), ("flips", "flips/s", "{:.1f}"),
            ("cap", "cap %", "{:.1f}"), ("p5", "cmd p5", "{:.0f}"), ("p95", "cmd p95", "{:.0f}"), ("az99", "|az| p99 g", "{:.2f}")]
    for sub in ("all", "calm", "windy"):
        n = next(iter(rows.values()))[sub]["n"]
        print(f"\n=== sim, one seed, full-strength variations — {sub} scenarios (n = {n}) ===")
        print(f"  {'candidate':10s} " + " ".join(f"{c[1]:>13s}" for c in cols) + f" {'streak %':>9s} {'crashes':>8s}")
        for lab, res in rows.items():
            m = res[sub]
            print(f"  {lab:10s} " + " ".join(f"{c[2].format(m[c[0]]):>13s}" for c in cols) + f" {m['streak']:9.1f} {m['crash']:8d}")
    if fl:
        print("\n=== flights (xiao logs, same metric code), median over engaged spans ===")
        for lab, m in fl.items():
            print(f"  {lab:10s} " + " ".join(f"{c[2].format(m[c[0]]):>13s}" for c in cols) + f"   ({m['n']} spans)")
        ref = rows.get(a.ref)
        if ref:
            print(f"\n=== predicted flight values: flight({a.ref}) × sim(candidate) / sim({a.ref}), calm-scenario sim as the field-day proxy ===")
            fmean = {k: float(np.mean([m[k] for m in fl.values()])) for k in ("qrms", "s23", "flips", "az99")}
            print(f"  {a.ref} measured in flight (mean of the flights): pitch RMS {fmean['qrms']:.0f} °/s, 2-3 Hz {fmean['s23']:.0f} %, "
                  f"flips {fmean['flips']:.1f}/s, |az| p99 {fmean['az99']:.2f} g")
            for lab, res in rows.items():
                c, r0 = res["calm"], ref["calm"]
                pred = {k: fmean[k] * c[k] / r0[k] for k in ("qrms", "s23", "flips", "az99")}
                print(f"  {lab:10s} pitch RMS {pred['qrms']:5.0f} °/s   2-3 Hz {pred['s23']:4.0f} %   flips {pred['flips']:4.1f}/s   |az| p99 {pred['az99']:.2f} g")


if __name__ == "__main__":
    sys.exit(main())
