#!/usr/bin/env python3
"""Per-span blackbox read at native rate (stdlib): modes, wind, accVib, VIBE debug, g-load,
baro vs GPS, BaroAlt physicality, rate-loop stats."""
import csv, math, statistics as st, sys
path = sys.argv[1]
cols = ["time (us)","axisRate[0]","axisRate[1]","gyroADC[0]","gyroADC[1]","gyroRaw[1]","accSmooth[0]","accSmooth[1]","accSmooth[2]","accVib",
        "debug[0]","debug[1]","debug[2]","debug[3]","debug[4]","debug[5]","debug[6]","debug[7]","motor[0]","BaroAlt (cm)",
        "flightModeFlags (flags)","mspOverrideFlags","wind[0]","wind[1]","wind[2]","rcCommand[1]","rcCommand[3]","vbat (V)","navState","navFlags","loopIteration"]
data = {c: [] for c in cols}
with open(path) as fh:
    rd = csv.reader(fh); hdr = [h.strip() for h in next(rd)]
    ix = [hdr.index(c) for c in cols]
    for row in rd:
        for c, i in zip(cols, ix):
            data[c].append(row[i])
N = len(data["time (us)"]); t = [float(v)/1e6 for v in data["time (us)"]]
t0 = t[0]
def fl(c): return [float(v) for v in data[c]]
def q(v,p):
    s = sorted(v); return s[min(len(s)-1,int(p*len(s)))]
msp = data["mspOverrideFlags"]; fm = data["flightModeFlags (flags)"]
# engaged spans from mspOverrideFlags==2
spans=[]; start=None
for i in range(N):
    e = msp[i] == "2"
    if e and start is None: start=i
    elif not e and start is not None: spans.append((start,i-1)); start=None
if start is not None: spans.append((start,N-1))
spans=[s for s in spans if t[s[1]]-t[s[0]]>2]
print("rows %d, %.1f s, fs %.1f Hz, t0=%.3f s (log time)" % (N, t[-1]-t[0], (N-1)/(t[-1]-t[0]), t0))
print("mspOverrideFlags values:", sorted(set(msp)), " flightModeFlags values:", sorted(set(fm))[:8])
print("engaged spans (log s):", ["%.1f-%.1f" % (t[a], t[b]) for a,b in spans])
acc = fl("accSmooth[2]"); accx=fl("accSmooth[0]"); accvib = fl("accVib"); motor = fl("motor[0]")
gq = fl("gyroADC[1]"); gp = fl("gyroADC[0]"); sq = fl("axisRate[1]"); sp = fl("axisRate[0]")
dbg = [fl("debug[%d]"%k) for k in range(8)]
baro = fl("BaroAlt (cm)"); vbat = fl("vbat (V)")
w = [fl("wind[%d]"%k) for k in range(3)]
segs = [("pre-arm/launch", 0, spans[0][0]-1)] if spans else []
for i,(a,b) in enumerate(spans):
    segs.append(("ENGAGED %d"%(i+1), a, b))
    nb = spans[i+1][0]-1 if i+1 < len(spans) else N-1
    segs.append(("manual after %d"%(i+1), b+1, nb))
print()
print("%-16s %7s | %6s %6s | %7s %5s | %6s %6s %6s | %6s %6s %6s | %6s %6s | %6s" % ("seg","dur s","accVib","p90","motor",">1900%","|az|p99","max","|ax|p99","gq RMS","sq RMS","gp RMS","wind","dir","vbat"))
for name,a,b in segs:
    if b <= a: continue
    sl = slice(a,b+1)
    av = accvib[sl]; mo = motor[sl]; az=[abs(v)/2048 for v in acc[sl]]; ax=[abs(v)/2048 for v in accx[sl]]
    ws = [math.hypot(w[0][i], w[1][i])/100 for i in range(a,b+1)]
    wd = [ (math.degrees(math.atan2(w[1][i], w[0][i]))+180)%360 for i in range(a,b+1)]
    print("%-16s %7.1f | %6.0f %6.0f | %7.0f %5.1f | %7.2f %5.2f %7.2f | %6.1f %6.1f %6.1f | %6.2f %6.0f | %6.2f" % (
        name, t[b]-t[a], st.median(av), q(av,.9), st.median(mo), 100*sum(1 for v in mo if v>1900)/len(mo),
        q(az,.99), max(az), q(ax,.99), st.pstdev(gq[sl]), st.pstdev(sq[sl]), st.pstdev(gp[sl]),  # deg/s as logged, no /10 (flight-analysis-20261011b.md s2)
        st.median(ws), st.median(wd), st.median(vbat[sl])))
print()
print("--- VIBE debug[] per segment: med / max ---")
for name,a,b in segs:
    if b <= a: continue
    sl = slice(a,b+1)
    print("%-16s " % name + " | ".join("d%d %6.0f/%6.0f" % (k, st.median(dbg[k][sl]), max(dbg[k][sl])) for k in range(8)))
print()
print("--- accSmooth[2] (g) distribution engaged, raw sign: p1 / p5 / med / p95 / p99 / min / max ---")
for name,a,b in segs:
    if not name.startswith("ENG"): continue
    v = [x/2048 for x in acc[a:b+1]]
    print("%-12s %+6.2f %+6.2f %+6.2f %+6.2f %+6.2f %+6.2f %+6.2f" % (name, q(v,.01), q(v,.05), st.median(v), q(v,.95), q(v,.99), min(v), max(v)))
print()
print("--- BaroAlt: update rate, per-update step p95/max (cm), engaged vs not; residual RMS about 0.5 s mean ---")
def baro_stats(a,b):
    steps=[]; last=baro[a]; lt=t[a]; n=0
    for i in range(a+1,b+1):
        if baro[i] != last:
            steps.append(abs(baro[i]-last)); n+=1; last=baro[i]
    hz = n/(t[b]-t[a]) if t[b]>t[a] else 0
    # residual about moving mean of ~0.5 s
    win = int(0.5*(N-1)/(t[-1]-t[0])); res=[]
    for i in range(a+win, b+1-win, max(1,win//8)):
        m = sum(baro[i-win:i+win])/(2*win); res.append(baro[i]-m)
    rms = math.sqrt(sum(r*r for r in res)/len(res))/100 if res else float('nan')
    return hz, (q(steps,.95) if steps else 0), (max(steps) if steps else 0), rms
for name,a,b in segs:
    if b <= a: continue
    hz,p95,mx,rms = baro_stats(a,b)
    print("%-16s baro upd %.1f Hz | step p95 %5.0f max %5.0f cm | resid RMS %.2f m | baro range %.1f..%.1f m" % (name,hz,p95,mx,rms,min(baro[a:b+1])/100,max(baro[a:b+1])/100))
print()
print("--- rc/pilot inputs during engaged (rcCommand[1] pitch, [3] throttle): med / min / max ---")
rcp = fl("rcCommand[1]"); rct = fl("rcCommand[3]")
for name,a,b in segs:
    if not name.startswith("ENG"): continue
    print("%-12s rcPitch %+5.0f %+5.0f %+5.0f | rcThr %5.0f %5.0f %5.0f | navState %s" % (name, st.median(rcp[a:b+1]), min(rcp[a:b+1]), max(rcp[a:b+1]), st.median(rct[a:b+1]), min(rct[a:b+1]), max(rct[a:b+1]), sorted(set(data["navState"][a:b+1]))))
