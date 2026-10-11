#!/usr/bin/env python3
"""Per-span downwind bias: mean of (craft - rabbit) horizontal offset projected on INAV's wind
vector (blackbox wind[0]=N, wind[1]=E, cm/s, direction the air moves TO), as in
flight-analysis-20261011.md section 1. argv[1] xiao tick CSV, argv[2] decoded blackbox CSV."""
import csv, math, statistics as st, sys
X = list(csv.DictReader(open(sys.argv[1])))
want=["time (us)","wind[0]","wind[1]","mspOverrideFlags"]
B={k:[] for k in want}
with open(sys.argv[2]) as fh:
    rd=csv.reader(fh); hdr=[h.strip() for h in next(rd)]; ix=[hdr.index(k) for k in want]
    for r in rd:
        for k,i in zip(want,ix): B[k].append(r[i])
t=[float(v)/1e6 for v in B["time (us)"]]; N=len(t); spans=[]; start=None
for i in range(N):
    e=B["mspOverrideFlags"][i]=="2"
    if e and start is None: start=i
    elif not e and start is not None: spans.append((start,i-1)); start=None
spans=[s for s in spans if t[s[1]]-t[s[0]]>2]
print("span | wind N,E (m/s) speed dir(to) | craft-rabbit along-wind mean / med (m) | cross-wind mean | n ticks")
for si,(a,b) in enumerate(spans,1):
    wn=st.median(float(v) for v in B["wind[0]"][a:b+1])/100; we=st.median(float(v) for v in B["wind[1]"][a:b+1])/100
    sp=math.hypot(wn,we); un,ue=(wn/sp,we/sp) if sp>0 else (0,0)
    R=[r for r in X if int(r["span_id"])==si]
    along=[(float(r["pos_n"])-float(r["rabbit_n"]))*un+(float(r["pos_e"])-float(r["rabbit_e"]))*ue for r in R]
    cross=[-(float(r["pos_n"])-float(r["rabbit_n"]))*ue+(float(r["pos_e"])-float(r["rabbit_e"]))*un for r in R]
    print("  %d | %+.2f %+.2f  %.2f m/s %3.0f deg | %+6.1f / %+6.1f | %+6.1f | %d" % (si, wn, we, sp, math.degrees(math.atan2(we,wn))%360, st.mean(along), st.median(along), st.mean(cross), len(R)))
