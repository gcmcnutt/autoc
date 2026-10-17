#!/usr/bin/env python3
"""
windiest_scenarios.py — THE WINDIEST ARENAS ON EACH PATH, AND HOW THE ELITE FLIES THEM (043 t6, 2026-10-16)

Wind is a per-scenario draw (uniform 0–8 m/s speed, turbulence ×0.5–2, gust length ×0.25–1, thermals) that the variation ramp
scales in: realized = base + s·(draw − base), base = the sim's 3.66 m/s, s = computeVariationScale(gen) = min((gen−1)//step,
N−1)/(N−1). The draw is not stored in the dmp (WindVariationConfig is RPC-only), so this ranks scenarios by the wind the elite
actually flew — median of ‖(wN, wE)‖ over the scenario's ticks, the tracking_by_wind.py definition — and projects it to full
strength: draw ≈ base + (realized − base)/s. Gusts and thermal inflow make the projection approximate.

Per path: the N windiest scenarios with their index (0–293, the renderer's arena order), wind now / projected at s = 1, gust p95,
direction FROM, and the elite's result there (crash reason, streak %, pitch-rate RMS) beside the path's median.

Exact draws (2026-10-16): build wind_draws.cc (see its header), feed it the meta's seeds, and pass --draws: the ranking then uses
the true speed draw (WindSpeedMin + (Max−Min)·uSpeed) and prints turbulence / gust-length / thermal multipliers. On t6 gen 525 the
realized-wind ranking and the exact-draw ranking pick the same windiest arena on every path.

Usage: windiest_scenarios.py --csv tick.csv --meta meta.yaml --gen N [--draws draws.txt] [--top 3]
                             [--ramp-step 40 --total-gens 800 --base 3.66 --speed-range 0 8]
"""
import argparse, csv, math, re, sys
import numpy as np
from collections import defaultdict

PATH_NAMES = ["StraightAndLevel", "SpiralClimb", "HorizFigure8", "randomA", "HighPerchSplitS", "SeededRandomB"]
GS = 6.0  # gyrQ is rad/s ÷ 6


def ramp_scale(gen, step, total):
    if step <= 0: return 1.0
    n = total // step
    return 1.0 if n <= 1 else min((gen - 1) // step, n - 1) / float(n - 1)


def read_meta(path):
    out, cur, idx = {}, None, None
    for line in open(path):
        m = re.match(r"\s*-\s*idx:\s*(\d+)", line)
        if m:
            if cur is not None: out[idx] = cur
            idx, cur = int(m.group(1)), {}; continue
        if cur is None: continue
        m = re.match(r"\s*([a-z_]+):\s*(.*)", line)
        if not m: continue
        k, v = m.group(1), m.group(2).strip()
        if k in ("path_variant", "wind_variant"): cur[k] = int(v)
        elif k == "crash_reason": cur[k] = v
        elif k == "score": cur[k] = float(v)
        elif k == "streak_steps": cur[k] = float(v)
        elif k == "steps":
            sm = re.match(r"(\d+)\s*/\s*(\d+)", v)
            if sm: cur["steps"], cur["steps_total"] = float(sm.group(1)), float(sm.group(2))
    if cur is not None: out[idx] = cur
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--csv", required=True); ap.add_argument("--meta", required=True)
    ap.add_argument("--gen", type=int, required=True); ap.add_argument("--top", type=int, default=3)
    ap.add_argument("--ramp-step", type=int, default=40); ap.add_argument("--total-gens", type=int, default=800)
    ap.add_argument("--base", type=float, default=3.66)
    ap.add_argument("--draws", default=None, help="output of wind_draws.cc: exact per-scenario uniforms")
    ap.add_argument("--speed-range", nargs=2, type=float, default=[0.0, 8.0])
    a = ap.parse_args()
    draws = {}
    if a.draws:
        for line in open(a.draws):
            f = line.split()
            draws[int(f[0])] = dict(speed=a.speed_range[0] + (a.speed_range[1] - a.speed_range[0]) * float(f[5]),
                                    turb=0.5 + 1.5 * float(f[6]), gust=0.25 + 0.75 * float(f[7]), therm=0.5 + 1.0 * float(f[8]))
    s = ramp_scale(a.gen, a.ramp_step, a.total_gens)
    meta = read_meta(a.meta)
    r = csv.reader(open(a.csv)); h = next(r)
    iS, iN, iE = h.index("scenario"), h.index("wN"), h.index("wE"); iQ = len(h) - 1 - h[::-1].index("gyrQ")
    acc = defaultdict(lambda: {"wn": [], "we": [], "q": []})
    for row in r:
        if len(row) < len(h): continue
        d = acc[int(row[iS])]; d["wn"].append(float(row[iN])); d["we"].append(float(row[iE])); d["q"].append(float(row[iQ]) * GS)
    rows = []
    for idx, d in acc.items():
        wn, we = np.array(d["wn"]), np.array(d["we"]); sp = np.hypot(wn, we)
        m = meta.get(idx, {})
        rows.append(dict(idx=idx, path=m.get("path_variant", idx // 49), wv=m.get("wind_variant", idx % 49),
                         w=float(np.median(sp)), g95=float(np.percentile(sp, 95)),
                         dirfrom=(math.degrees(math.atan2(-we.mean(), -wn.mean())) + 360) % 360,
                         proj=a.base + (float(np.median(sp)) - a.base) / s if s > 0 else float("nan"),
                         crash=m.get("crash_reason", "?"), streak=100 * m.get("streak_steps", 0) / max(m.get("steps", 1), 1),
                         qrms=math.degrees(float(np.sqrt(np.mean(np.array(d["q"]) ** 2)))), draw=draws.get(idx)))
    if draws:
        for x in rows:
            if x["draw"]: x["proj"] = x["draw"]["speed"]
    print(f"gen {a.gen}: variation scale s = {s:.3f} (full strength = 1.000 from gen {a.ramp_step * (a.total_gens // a.ramp_step - 1) + 1}); "
          f"wind now = {a.base} + s·(draw − {a.base}) m/s")
    for p, name in enumerate(PATH_NAMES):
        pr = [x for x in rows if x["path"] == p]
        if not pr: continue
        med_streak = np.median([x["streak"] for x in pr]); med_q = np.median([x["qrms"] for x in pr])
        crashes = sum(1 for x in pr if x["crash"] != "RabbitComplete")
        ws = sorted(x["w"] for x in pr)
        print(f"\n{name} (49 arenas: wind now {ws[0]:.1f}–{ws[-1]:.1f} m/s; elite median streak {med_streak:.0f} %, pitch RMS {med_q:.0f} °/s, "
              f"crashes {crashes})")
        print(f"  {'arena':>5s} {'wind#':>5s} {'now m/s':>8s} {'gust p95':>8s} {'at s=1':>7s} {'from °':>7s}  {'result':14s} {'streak %':>8s} {'pitch RMS':>9s}")
        key = (lambda x: -x["draw"]["speed"]) if draws else (lambda x: -x["w"])
        for x in sorted(pr, key=key)[:a.top]:
            print(f"  {x['idx']:5d} {x['wv']:5d} {x['w']:8.1f} {x['g95']:8.1f} {x['proj']:7.1f} {x['dirfrom']:7.0f}  {x['crash']:14s} "
                  f"{x['streak']:8.0f} {x['qrms']:9.0f}" + (f"   turb x{x['draw']['turb']:.2f} gust-L x{x['draw']['gust']:.2f} thermal x{x['draw']['therm']:.2f}" if x["draw"] else ""))


if __name__ == "__main__":
    sys.exit(main())
