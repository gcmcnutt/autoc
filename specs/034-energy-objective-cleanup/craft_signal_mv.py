#!/usr/bin/env python3
"""Multivariate craft-variation -> behavior signal (gen-800 elite).

Companion to craft_signal.py. Univariate Pearson is confounded because the
6 craft params co-vary inside each behavior (e.g. speed depends on thrSc
AND drag AND how hard the loop pushes throttle). Here we standardize all
inputs/outputs and fit each behavior = beta . craft_params via least
squares, reporting:
  - standardized beta per craft param (isolated, partials-out the others)
  - joint R^2 (how much of the 36-craft behavioral spread the 6 params
    explain together) -- this is the "is there ANY signal" number
  - the single expected-direction param's beta, flagged.

Controls are bang-bang (|outPt|~0.97, |outRl|~1.0) and the inner PID
integrator is unused (pidInt==0), so we read the signal off the
non-saturated channels: velocity, body rates, held attitude, throttle
fraction, saturation count, and the NN's signed rate commands.
"""
import re, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
LOG = HERE.parent.parent / "logs" / "autoc-034-craft-confirm.log"
HDR = HERE / "craft_gen800.hdr"
DAT = HERE / "craft_gen800.dat"

# ---- craft table -----------------------------------------------------------
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
assert len(craft) == 36

hdr = open(HDR).read().split()
col = {n: i for i, n in enumerate(hdr)}

def quat_pr(qw, qx, qy, qz):
    sinp = max(-1.0, min(1.0, 2.0*(qw*qy - qz*qx)))
    return math.asin(sinp), math.atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy))

BK = ["vel","outTh","outPt","outRl","absPt","absRl","gyrP","gyrQ","gyrR",
      "pitch","roll","rateCmdP","rateCmdQ","rateAchP","rateAchQ","sat","vx"]
agg = {s: {k: 0.0 for k in BK} for s in range(36)}
cnt = {s: 0 for s in range(36)}
for line in open(DAT):
    p = line.split()
    if len(p) <= max(col.values()):
        continue
    s = int(p[col["Pth/Wnd:Step:"]].split("/")[1].split(":")[0])
    a = agg[s]; cnt[s] += 1
    g = lambda n: float(p[col[n]])
    a["vel"] += g("vel"); a["outTh"] += g("outTh")
    a["outPt"] += g("outPt"); a["outRl"] += g("outRl")
    a["absPt"] += abs(g("outPt")); a["absRl"] += abs(g("outRl"))
    a["gyrP"] += abs(g("gyrP")); a["gyrQ"] += abs(g("gyrQ")); a["gyrR"] += abs(g("gyrR"))
    a["rateCmdP"] += g("rateCmdP"); a["rateCmdQ"] += g("rateCmdQ")
    a["rateAchP"] += g("rateAchP"); a["rateAchQ"] += g("rateAchQ")
    a["sat"] += g("pidSat"); a["vx"] += g("vxBdy")
    pit, rol = quat_pr(g("qw"), g("qx"), g("qy"), g("qz"))
    a["pitch"] += pit; a["roll"] += rol
beh = {s: {k: agg[s][k]/cnt[s] for k in BK} for s in range(36)}

X = np.array([craft[s] for s in range(36)])          # 36 x 6
Xz = (X - X.mean(0)) / X.std(0)
Xd = np.hstack([Xz, np.ones((36, 1))])               # design + intercept

# expected dominant driver per behavior (param, sign)
EXP = {
    "vel":      ("thrSc", +1), "outTh": ("drag", +1),
    "pitch":    ("trim", +1),  "roll":  ("rolEff", +1),
    "absPt":    ("pitEff", -1),"absRl": ("rolEff", -1),
    "gyrP":     ("rolEff", +1),"gyrQ":  ("pitEff", +1),
    "rateCmdQ": ("trim", -1),  "rateCmdP": ("rolEff", -1),
    "sat":      ("drag", +1),  "vx": ("thrSc", +1),
}

print("Multivariate standardized regression: behavior ~ 6 craft params (n=36)\n")
print(f"{'behavior':>9} {'R^2':>6} {'spread':>8} | standardized betas "
      f"({'  '.join(f'{c:>6}' for c in CP)})   expected")
print("-"*112)
rows = []
for bh in ["vel","outTh","vx","sat","roll","pitch","absRl","absPt",
           "gyrP","gyrQ","rateCmdP","rateCmdQ"]:
    y = np.array([beh[s][bh] for s in range(36)])
    spread = y.max() - y.min()
    yz = (y - y.mean()) / (y.std() if y.std() else 1.0)
    coef, *_ = np.linalg.lstsq(Xd, yz, rcond=None)
    betas = coef[:6]
    yhat = Xd @ coef
    r2 = 1.0 - np.sum((yz - yhat)**2) / np.sum((yz - yz.mean())**2) if y.std() else 0.0
    exp_p, exp_s = EXP.get(bh, (None, 0))
    bstr = "  ".join(f"{b:+6.2f}" for b in betas)
    flag = ""
    if exp_p:
        bexp = betas[CP.index(exp_p)]
        ok = "OK" if bexp*exp_s > 0.05 else ("." if abs(bexp) < 0.05 else "FLIP")
        flag = f"{exp_p}{'+' if exp_s>0 else '-'}:{bexp:+.2f} [{ok}]"
    print(f"{bh:>9} {r2:6.2f} {spread:8.3f} | {bstr}   {flag}")
    rows.append((bh, r2, betas))

print("\nLegend: betas are std-units of behavior per std-unit of that craft param,")
print("holding the other 5 fixed. R^2 = fraction of cross-craft behavioral")
print("spread explained jointly. |beta|>~0.3 with sign-correct = real signal.\n")

# strongest single (param,behavior) pairs by |beta|
print("=== strongest isolated craft->behavior couplings (|beta| ranked) ===")
pairs = []
for bh, r2, betas in rows:
    for i, c in enumerate(CP):
        pairs.append((abs(betas[i]), betas[i], c, bh, r2))
for ab, b, c, bh, r2 in sorted(pairs, reverse=True)[:12]:
    print(f"  beta={b:+.2f}  {c:>6} -> {bh:<9}  (model R^2={r2:.2f})")
