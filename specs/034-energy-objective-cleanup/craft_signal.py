#!/usr/bin/env python3
"""Find craft-variation -> behavior signal in the gen-800 elite.

034 craft-confirm run (logs/autoc-034-craft-confirm.log, seed 1780290226):
entry/wind/rabbit variations are all OFF, ONLY craft physics varies across
the 36 scenarios (cgU, drag, trim, thrSc, pitEff, rolEff). The gen-800
best individual is flown through all 36, so it is the SAME genome each
time -- any per-scenario behavioral difference is the controller (NN +
inner-loop rate PID) reacting to the craft via feedback.

We correlate each craft param against per-scenario flight aggregates and
report Pearson r vs the physically-expected sign. Throttle is bang-bang
and roll is often hard-over, so we lean on rates/velocity/integrators
(which are not saturated) for the signal.

Inputs (staged next to this script; extracts are regenerable from data.dat):
  craft_gen800.dat  -- gen-800 elite rows (Scn==6400800), all 36 scenarios
  craft_gen800.hdr  -- the data.dat header line
  ../../logs/autoc-034-craft-confirm.log -- craft variation table source

Regenerate the extracts (dropped after analysis):
  tail -n 20000 data.dat | awk '$1==6400800' > craft_gen800.dat
  head -1 data.dat > craft_gen800.hdr
"""
import re, math, statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOG = HERE.parent.parent / "logs" / "autoc-034-craft-confirm.log"
HDR = HERE / "craft_gen800.hdr"
DAT = HERE / "craft_gen800.dat"

# ---- parse craft table from log -------------------------------------------
craft = {}  # scn -> dict
row_re = re.compile(
    r"<info>\s+(\d+)\s+0x[0-9a-f]+\s+"
    r"[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+[\d.%\-]+\s+[\d.\-]+\s+"  # heading roll pitch speed wind
    r"[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+"                          # N E D
    r"(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+0x([0-9a-f]+)")
for line in open(LOG):
    m = row_re.search(line)
    if m:
        scn = int(m.group(1))
        craft[scn] = dict(cgU=float(m.group(2)), drag=float(m.group(3)),
                          trim=float(m.group(4)), thrSc=float(m.group(5)),
                          pitEff=float(m.group(6)), rolEff=float(m.group(7)))
assert len(craft) == 36, f"got {len(craft)} craft rows"

# ---- header column lookup --------------------------------------------------
hdr = open(HDR).read().split()
col = {name: i for i, name in enumerate(hdr)}
def C(name): return col[name]

# ---- accumulate per-scenario behavior -------------------------------------
keys = ["vel","outTh","outPt","outRl","absPt","absRl","pidIntP","pidIntQ",
        "gyrP","gyrQ","pitch","roll","vx","sat"]
agg = {s: dict(n=0, **{k: 0.0 for k in keys}) for s in range(36)}

def quat_to_pitch_roll(qw, qx, qy, qz):
    sinp = max(-1.0, min(1.0, 2.0*(qw*qy - qz*qx)))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    return pitch, roll

for line in open(DAT):
    p = line.split()
    if len(p) <= max(col.values()):
        continue
    scn = int(p[C("Pth/Wnd:Step:")].split("/")[1].split(":")[0])
    a = agg[scn]; a["n"] += 1
    a["vel"]     += float(p[C("vel")])
    a["outTh"]   += float(p[C("outTh")])
    a["outPt"]   += float(p[C("outPt")])
    a["outRl"]   += float(p[C("outRl")])
    a["absPt"]   += abs(float(p[C("outPt")]))
    a["absRl"]   += abs(float(p[C("outRl")]))
    a["pidIntP"] += float(p[C("pidIntP")])
    a["pidIntQ"] += float(p[C("pidIntQ")])
    a["gyrP"]    += abs(float(p[C("gyrP")]))
    a["gyrQ"]    += abs(float(p[C("gyrQ")]))
    a["vx"]      += float(p[C("vxBdy")])
    a["sat"]     += float(p[C("pidSat")])
    pit, rol = quat_to_pitch_roll(float(p[C("qw")]), float(p[C("qx")]),
                                  float(p[C("qy")]), float(p[C("qz")]))
    a["pitch"] += pit; a["roll"] += rol

beh = {s: {k: (v/agg[s]["n"] if k != "n" else v) for k, v in agg[s].items()}
       for s in range(36)}

def pearson(xs, ys):
    n = len(xs); mx, my = sum(xs)/n, sum(ys)/n
    sxy = sum((x-mx)*(y-my) for x, y in zip(xs, ys))
    sxx = sum((x-mx)**2 for x in xs); syy = sum((y-my)**2 for y in ys)
    return sxy/math.sqrt(sxx*syy) if sxx and syy else 0.0

scns = list(range(36))
tests = [
    ("thrSc",  "vel",     +1, "more thrust -> higher speed"),
    ("drag",   "vel",     -1, "more drag -> lower speed"),
    ("thrSc",  "outTh",   -1, "more thrust -> less throttle needed"),
    ("drag",   "outTh",   +1, "more drag -> more throttle"),
    ("trim",   "pidIntQ", -1, "nose trim -> pitch integrator opposes it"),
    ("trim",   "pitch",   +1, "trim bias shows in held pitch attitude"),
    ("cgU",    "pidIntQ", -1, "CG shift -> pitch trim compensation"),
    ("cgU",    "pitch",   +1, "CG shift -> pitch attitude"),
    ("pitEff", "absPt",   -1, "more pitch authority -> less pitch cmd"),
    ("pitEff", "pidIntQ", -1, "more pitch authority -> smaller integrator"),
    ("rolEff", "absRl",   -1, "more roll authority -> less roll cmd"),
    ("rolEff", "pidIntP", -1, "more roll authority -> smaller roll integrator"),
    ("rolEff", "roll",    +1, "roll-eff asymmetry -> held roll"),
    ("trim",   "absPt",   +1, "trim disturbance -> more pitch activity"),
    ("drag",   "sat",     +1, "more drag -> more PID saturation"),
]

print(f"gen-800 elite, 36 craft scenarios, ~{beh[0]['n']:.0f} ticks each\n")
print(f"{'craft':>7} {'':2} {'behavior':>9} {'exp':>4} {'r':>7}  {'match':>5}  rationale")
print("-"*80)
results = []
for cp, bh, sign, why in tests:
    xs = [craft[s][cp] for s in scns]; ys = [beh[s][bh] for s in scns]
    r = pearson(xs, ys)
    match = "OK" if (r*sign > 0 and abs(r) > 0.15) else ("weak" if abs(r) <= 0.15 else "FLIP")
    results.append((cp, bh, sign, r, match, why))
    print(f"{cp:>7} -> {bh:>9} {sign:>+4} {r:>+7.3f}  {match:>5}  {why}")

print("\n=== ranked by |r| ===")
for cp, bh, sign, r, match, why in sorted(results, key=lambda t: -abs(t[3])):
    print(f"  |r|={abs(r):.3f}  {cp:>7}->{bh:<9} {match:<5} {why}")

print("\n=== behavior spread across the 36 crafts (mean / min / max) ===")
for bh in ["vel","outTh","pidIntQ","pidIntP","pitch","roll","absPt","absRl","sat"]:
    vals = [beh[s][bh] for s in scns]
    print(f"  {bh:>8}: mean={statistics.mean(vals):+.4f}  [{min(vals):+.4f}, {max(vals):+.4f}]  spread={max(vals)-min(vals):.4f}")
