#!/usr/bin/env python3
"""Whole-flight vertical-estimator survey straight from the blackbox, for logs that carry NAV_POS
(navPos[2] cm up, navVel[2] cm/s, navEPH/navEPV cm). Successor to f1011_xiao_z_survey.py, which had
to rebuild navPos[2] from the xiao because the 10-04 bench config dropped NAV_POS.
  argv[1] decoded blackbox CSV, argv[2] its .gps.csv.
Prints: per-sample step survey on navPos[2] (the 20/20 jump marker is navEPV reaching
inav_max_eph_epv = 1000 cm), per-segment navEPV/navEPH, est-baro and est-GPS residuals, GPS health."""
import csv, math, statistics as st, sys
path, gpath = sys.argv[1], sys.argv[2]
cols = ["time (us)","navPos[2]","navVel[2]","navEPH","navEPV","BaroAlt (cm)","mspOverrideFlags","navFlags","flightModeFlags (flags)","accVib","debug[4]"]
D = {c: [] for c in cols}
with open(path) as fh:
    rd = csv.reader(fh); hdr=[h.strip() for h in next(rd)]; ix=[hdr.index(c) for c in cols]
    for row in rd:
        for c,i in zip(cols,ix): D[c].append(row[i])
t=[float(v)/1e6 for v in D["time (us)"]]; N=len(t)
z=[float(v)/100 for v in D["navPos[2]"]]; vz=[float(v)/100 for v in D["navVel[2]"]]
eph=[float(v) for v in D["navEPH"]]; epv=[float(v) for v in D["navEPV"]]
baro=[float(v)/100 for v in D["BaroAlt (cm)"]]
def q(v,p): s=sorted(v); return s[min(len(s)-1,int(p*len(s)))]
# segments
msp=D["mspOverrideFlags"]; spans=[]; start=None
for i in range(N):
    e = msp[i]=="2"
    if e and start is None: start=i
    elif not e and start is not None: spans.append((start,i-1)); start=None
if start is not None: spans.append((start,N-1))
spans=[s for s in spans if t[s[1]]-t[s[0]]>2]
# home reference: first ARM sample
arm = next(i for i in range(N) if "ARM" in D["flightModeFlags (flags)"][i])
print("rows %d, %.1f-%.1f s, fs %.1f Hz; ARM at %.1f s: navPos[2] %.2f m, baro %.2f m" % (N, t[0], t[-1], (N-1)/(t[-1]-t[0]), t[arm], z[arm], baro[arm]))
print("navFlags values:", sorted(set(D["navFlags"])))
# step survey
steps=[(t[i], z[i]-z[i-1], (z[i]-z[i-1])/(t[i]-t[i-1])) for i in range(1,N) if t[i]>t[i-1]]
big=[s for s in steps if abs(s[1])>5]; mid=[s for s in steps if abs(s[1])>2]
print("\n--- navPos[2] per-sample steps over %d samples: |dz|>5 m: %d, |dz|>2 m: %d, max |dz| %.2f m at %.1f s, |dz/dt| p99 %.1f m/s, max %.1f m/s" % (
    len(steps), len(big), len(mid), max(abs(s[1]) for s in steps), max(steps,key=lambda s:abs(s[1]))[0], q([abs(s[2]) for s in steps],0.99), max(abs(s[2]) for s in steps)))
for s in big[:20]: print("   %.2f s  dz %+.2f m" % (s[0], s[1]))
print("whole-log navEPV: med %.0f p95 %.0f MAX %.0f cm (cap 1000) | navEPH med %.0f max %.0f" % (st.median(epv), q(epv,0.95), max(epv), st.median(eph), max(eph)))
# segments
segs=[("pre-engage", arm, spans[0][0]-1)] if spans else []
for i,(a,b) in enumerate(spans):
    segs.append(("ENGAGED %d"%(i+1), a, b)); nb = spans[i+1][0]-1 if i+1<len(spans) else N-1
    segs.append(("after %d"%(i+1), b+1, nb))
print("\nseg            dur | navEPV med/p95/max | navEPH med/max | est-baro med/p95/max (m, rel. ARM) | vz p95 | |vz| max | accVib med | accW med")
for name,a,b in segs:
    E=epv[a:b+1]; H=eph[a:b+1]; R=[(z[i]-z[arm])-(baro[i]-baro[arm]) for i in range(a,b+1)]; V=[abs(v) for v in vz[a:b+1]]
    av=[float(v) for v in D["accVib"][a:b+1]]; aw=[float(v)/1000 for v in D["debug[4]"][a:b+1]]
    print("%-12s %5.1f | %4.0f %4.0f %4.0f | %4.0f %4.0f | %+5.1f %5.1f %5.1f | %5.1f | %5.1f | %5.0f | %.2f" % (
        name, t[b]-t[a], st.median(E), q(E,0.95), max(E), st.median(H), max(H), st.median(R), q([abs(r) for r in R],0.95), max(abs(r) for r in R), q(V,0.95), max(V), st.median(av), st.median(aw)))
# GPS
G=list(csv.DictReader(open(gpath)))
gt=[float(r["time (us)"])/1e6 for r in G]; galt=[float(r["GPS_altitude"]) for r in G]
sats=[int(r["GPS_numSat"]) for r in G]; geph=[float(r["GPS_eph"]) for r in G]; gepv=[float(r["GPS_epv"]) for r in G]
# gps altitude units: cm in INAV frames; report raw range to sanity check
print("\nGPS: %d fixes over %.1f s (%.1f Hz), numSat min/med %d/%d, eph med/max %.0f/%.0f, epv med/max %.0f/%.0f, GPS_altitude raw range %.0f..%.0f" % (
    len(G), gt[-1]-gt[0], (len(G)-1)/(gt[-1]-gt[0]), min(sats), st.median(sats), st.median(geph), max(geph), st.median(gepv), max(gepv), min(galt), max(galt)))
# est - gps, engaged (nearest blackbox sample to each fix)
import bisect
g_arm = galt[bisect.bisect_left(gt, t[arm])] if gt else 0
scale = 100.0 if max(galt)-min(galt) > 2000 else 1.0   # cm vs m
for i,(a,b) in enumerate(spans):
    res=[]
    for k in range(len(G)):
        if t[a]<=gt[k]<=t[b]:
            j=bisect.bisect_left(t, gt[k]); j=min(j,N-1)
            res.append((z[j]-z[arm]) - (galt[k]-g_arm)/scale)
    if res: print("ENGAGED %d est-GPS (m): med %+.1f p95 %.1f max %.1f over %d fixes" % (i+1, st.median(res), q([abs(r) for r in res],0.95), max(abs(r) for r in res), len(res)))
