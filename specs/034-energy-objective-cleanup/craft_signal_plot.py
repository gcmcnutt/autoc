#!/usr/bin/env python3
"""Visualize the strongest craft-variation -> behavior couplings.

For each strong coupling we pool the last 60 gens. Within each gen we
z-score the behavior across its 36 scenarios (removes the genome-level
baseline so different elites overlay), then scatter z-behavior vs the
craft param (constant across gens). 60*36 = 2160 points per panel; a
clean diagonal trend = the controller responds to that craft param the
same way regardless of which genome is flying.
"""
import re, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
LOG = HERE.parent.parent / "logs" / "autoc-034-craft-confirm.log"
HDR = HERE / "craft_gen800.hdr"
DAT = HERE / "craft_last60.dat"

craft = {}
row_re = re.compile(
    r"<info>\s+(\d+)\s+0x[0-9a-f]+\s+[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+[\d.%\-]+\s+"
    r"[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+[\d.\-]+\s+"
    r"(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+0x")
for line in open(LOG):
    m = row_re.search(line)
    if m:
        craft[int(m.group(1))] = [float(m.group(i)) for i in range(2, 8)]
CP = ["cgU","drag","trim","thrSc","pitEff","rolEff"]
craftM = np.array([craft[s] for s in range(36)])

hdr = open(HDR).read().split()
col = {n:i for i,n in enumerate(hdr)}
def quat_pr(qw,qx,qy,qz):
    sinp=max(-1.0,min(1.0,2.0*(qw*qy-qz*qx)))
    return math.asin(sinp), math.atan2(2.0*(qw*qx+qy*qz),1.0-2.0*(qx*qx+qy*qy))

BK=["vel","outTh","gyrP","roll","sat"]
data={}
for line in open(DAT):
    p=line.split()
    if len(p)<=max(col.values()): continue
    gen=int(p[col["Scn"]])//8001
    scn=int(p[col["Pth/Wnd:Step:"]].split("/")[1].split(":")[0])
    a=data.setdefault((gen,scn),{k:0.0 for k in BK}|{"n":0})
    g=lambda n:float(p[col[n]])
    a["vel"]+=g("vel"); a["outTh"]+=g("outTh"); a["gyrP"]+=abs(g("gyrP"))
    a["sat"]+=g("pidSat")
    _,rol=quat_pr(g("qw"),g("qx"),g("qy"),g("qz")); a["roll"]+=rol
    a["n"]+=1
gens=sorted({k[0] for k in data})

def zpts(cp, bh):
    xs,ys=[],[]
    for gen in gens:
        col_b=np.array([ (data[(gen,s)][bh]/data[(gen,s)]["n"]) if (gen,s) in data and data[(gen,s)]["n"] else np.nan for s in range(36)])
        mask=~np.isnan(col_b)
        if mask.sum()<10 or col_b[mask].std()==0: continue
        z=(col_b-np.nanmean(col_b))/np.nanstd(col_b)
        for s in range(36):
            if mask[s]:
                xs.append(craftM[s,CP.index(cp)]); ys.append(z[s])
    return np.array(xs),np.array(ys)

PANELS=[("thrSc","vel","more thrust -> faster (60/60 gens)"),
        ("thrSc","outTh","more thrust -> throttle backed off (55/60)"),
        ("rolEff","roll","more roll authority -> more bank (55/60)"),
        ("rolEff","gyrP","more roll authority -> faster roll (46/60)"),
        ("pitEff","sat","more pitch authority -> less saturation (54/60)"),
        ("trim","roll","pitch trim reshapes the spiral bank (60/60)")]

fig,axes=plt.subplots(2,3,figsize=(15,8))
for ax,(cp,bh,title) in zip(axes.flat,PANELS):
    x,y=zpts(cp,bh)
    ax.scatter(x,y,s=6,alpha=0.25,color="#3366aa")
    b,a=np.polyfit(x,y,1)
    xs=np.linspace(x.min(),x.max(),50)
    ax.plot(xs,b*xs+a,color="#cc3322",lw=2)
    r=np.corrcoef(x,y)[0,1]
    ax.set_title(f"{title}\npooled r={r:+.3f}",fontsize=9)
    ax.set_xlabel(cp); ax.set_ylabel(f"{bh} (within-gen z)")
    ax.axhline(0,color="gray",lw=0.5)
fig.suptitle("Craft-variation signal: 60 elite genomes (gen 741-800), 36 crafts each\n"
             "same controller flies 36 craft variants; flight state shifts with craft params",
             fontsize=11)
fig.tight_layout(rect=[0,0,1,0.94])
out=HERE/"craft_signal.png"
fig.savefig(out,dpi=110)
print("wrote",out)
