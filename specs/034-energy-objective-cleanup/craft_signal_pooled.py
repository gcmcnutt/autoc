#!/usr/bin/env python3
"""Cross-generation consistency test for the craft-variation signal.

Single-gen regression (craft_signal_mv.py) is underpowered: n=36 scenarios,
6 confounded craft params, so joint R^2~0.17 sits at the noise floor and any
one gen could be a fluke of its genome.

But the craft->scenario map is IDENTICAL every generation (fixed scenarioSeed
table). So we can use the last 60 gens (741..800) as 60 independent draws of
the controller. For each craft_param -> behavior coupling we:

  1. compute the within-gen Pearson r across the 36 scenarios, per gen
  2. test whether the SIGN is consistent across the 60 gens (binomial sign
     test) and report the mean r.

A real physical coupling (e.g. more roll authority -> higher roll rate) must
hold for essentially every competent genome, so it shows up as ~60/60 gens
agreeing in sign even when each gen's |r| is modest. That is far stronger
evidence than one gen's regression, and it is immune to the 6-param/36-point
overfitting worry.

Regenerate the (dropped) extract from data.dat:
  tail -n 450000 data.dat | awk '$1 ~ /^[0-9]+$/ && $1+0>=5928741' > craft_last60.dat
  head -1 data.dat > craft_gen800.hdr
"""
import re, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
LOG = HERE.parent.parent / "logs" / "autoc-034-craft-confirm.log"
HDR = HERE / "craft_gen800.hdr"
DAT = HERE / "craft_last60.dat"

craft = {}
row_re = re.compile(
    r"<info>\s+(\d+)\s+0x[0-9a-f]+\s+"
    r"[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+[\d.%\-]+\s+[\d.\-]+\s+"
    r"[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+"
    r"(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+0x")
for line in open(LOG):
    m = row_re.search(line)
    if m:
        craft[int(m.group(1))] = [float(m.group(i)) for i in range(2, 8)]
CP = ["cgU", "drag", "trim", "thrSc", "pitEff", "rolEff"]
craftM = np.array([craft[s] for s in range(36)])

hdr = open(HDR).read().split()
col = {n: i for i, n in enumerate(hdr)}

def quat_pr(qw, qx, qy, qz):
    sinp = max(-1.0, min(1.0, 2.0*(qw*qy - qz*qx)))
    return math.asin(sinp), math.atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy))

BK = ["vel","outTh","absPt","absRl","gyrP","gyrQ","gyrR","pitch","roll","sat"]
# per (gen, scn) accumulators
data = {}   # (gen,scn) -> [sums..., n]
def blank(): return {k: 0.0 for k in BK} | {"n": 0}

for line in open(DAT):
    p = line.split()
    if len(p) <= max(col.values()):
        continue
    gen = int(p[col["Scn"]]) // 8001
    scn = int(p[col["Pth/Wnd:Step:"]].split("/")[1].split(":")[0])
    key = (gen, scn)
    a = data.get(key)
    if a is None:
        a = data[key] = blank()
    g = lambda n: float(p[col[n]])
    a["vel"] += g("vel"); a["outTh"] += g("outTh")
    a["absPt"] += abs(g("outPt")); a["absRl"] += abs(g("outRl"))
    a["gyrP"] += abs(g("gyrP")); a["gyrQ"] += abs(g("gyrQ")); a["gyrR"] += abs(g("gyrR"))
    a["sat"] += g("pidSat")
    pit, rol = quat_pr(g("qw"), g("qx"), g("qy"), g("qz"))
    a["pitch"] += pit; a["roll"] += rol
    a["n"] += 1

gens = sorted({k[0] for k in data})
def beh_matrix(bh, gen):
    out = []
    for s in range(36):
        a = data.get((gen, s))
        out.append(a[bh]/a["n"] if a and a["n"] else np.nan)
    return np.array(out)

def pear(x, y):
    mask = ~np.isnan(y)
    x, y = x[mask], y[mask]
    if len(x) < 10 or x.std() == 0 or y.std() == 0:
        return np.nan
    return float(np.corrcoef(x, y)[0, 1])

# couplings to test: (craft_param, behavior, expected_sign, rationale)
TESTS = [
    ("rolEff","gyrP", +1, "more roll authority -> higher roll rate"),
    ("rolEff","roll", +1, "more roll authority -> more held bank"),
    ("rolEff","absRl",-1, "more roll authority -> less roll command"),
    ("pitEff","gyrQ", +1, "more pitch authority -> higher pitch rate"),
    ("pitEff","absPt",-1, "more pitch authority -> less pitch command"),
    ("pitEff","sat",  -1, "more pitch authority -> less PID saturation"),
    ("thrSc", "vel",  +1, "more thrust -> higher speed"),
    ("drag",  "vel",  -1, "more drag -> lower speed"),
    ("drag",  "outTh",+1, "more drag -> more throttle"),
    ("drag",  "pitch",+1, "more drag -> higher AoA/pitch to hold"),
    ("thrSc", "outTh",-1, "more thrust -> less throttle"),
    ("trim",  "pitch",+1, "pitch-trim bias -> held pitch shift"),
    ("trim",  "roll", +1, "trim alters spiral geometry -> bank shift"),
    ("cgU",   "pitch",+1, "CG aft -> nose-up pitch"),
    ("trim",  "sat",  +1, "trim disturbance -> more saturation"),
]

def binom_p_two_sided(k, n):
    # P(X<=min or X>=max) under p=0.5, two-sided
    from math import comb
    kk = min(k, n-k)
    tail = sum(comb(n, i) for i in range(0, kk+1)) / (2.0**n)
    return min(1.0, 2*tail)

print(f"Pooled across {len(gens)} gens (gen {gens[0]}..{gens[-1]}), "
      f"36 craft scenarios each.\n")
print(f"{'coupling':>16} {'exp':>4} {'mean_r':>7} {'agree':>7} {'sign_p':>9}  verdict  rationale")
print("-"*100)
res = []
for cp, bh, sign, why in TESTS:
    xc = craftM[:, CP.index(cp)]
    rs = [pear(xc, beh_matrix(bh, gp)) for gp in gens]
    rs = [r for r in rs if not math.isnan(r)]
    nrs = len(rs)
    mean_r = sum(rs)/nrs
    agree = sum(1 for r in rs if r*sign > 0)
    p = binom_p_two_sided(agree, nrs)
    if agree/nrs >= 0.9 and p < 0.01:
        verdict = "STRONG"
    elif agree/nrs >= 0.75 and p < 0.05:
        verdict = "signal"
    elif agree/nrs >= 0.6:
        verdict = "weak"
    else:
        verdict = "none"
    res.append((verdict, mean_r, agree, nrs, cp, bh, why))
    print(f"{cp+'->'+bh:>16} {sign:>+4} {mean_r:>+7.3f} {agree:>3}/{nrs:<3} "
          f"{p:>9.1e}  {verdict:>7}  {why}")

order = {"STRONG":0,"signal":1,"weak":2,"none":3}
print("\n=== sorted by strength ===")
for v, mr, a, n, cp, bh, why in sorted(res, key=lambda t: (order[t[0]], -abs(t[1]))):
    print(f"  [{v:>6}] {cp:>6}->{bh:<6} mean_r={mr:+.3f} {a}/{n} gens agree -- {why}")
