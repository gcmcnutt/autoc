#!/usr/bin/env python3
"""Full-rate (966 Hz) Welch spectra of pitch gyro (raw and filtered) and normal accel per engaged
span, nseg=8192 (0.118 Hz bins). Answers t3 §5.5's open question: is there anything above 30 Hz
in flight, and where does the blade-pass line sit while engaged."""
import csv, cmath, math, statistics as st, sys
path = sys.argv[1]
want = ["time (us)","gyroADC[1]","gyroRaw[1]","gyroRaw[0]","accSmooth[2]","mspOverrideFlags","motor[0]"]
D = {k: [] for k in want}
with open(path) as fh:
    rd = csv.reader(fh); hdr=[h.strip() for h in next(rd)]; ix=[hdr.index(k) for k in want]
    for row in rd:
        for k,i in zip(want,ix): D[k].append(row[i])
t = [float(v)/1e6 for v in D["time (us)"]]; N=len(t); fs=(N-1)/(t[-1]-t[0])
def fft(a):
    n=len(a)
    if n==1: return a
    e,o=fft(a[0::2]),fft(a[1::2]); tw=[cmath.exp(-2j*math.pi*k/n)*o[k] for k in range(n//2)]
    return [e[k]+tw[k] for k in range(n//2)]+[e[k]-tw[k] for k in range(n//2)]
NSEG=8192; WIN=[0.5-0.5*math.cos(2*math.pi*i/(NSEG-1)) for i in range(NSEG)]
def welch(x):
    segs=[]
    for s in range(0,len(x)-NSEG+1,NSEG//2):
        seg=x[s:s+NSEG]; m=sum(seg)/NSEG
        X=fft([complex((seg[i]-m)*WIN[i]) for i in range(NSEG)]); segs.append([abs(v)**2 for v in X[:NSEG//2]])
    p=[sum(s[k] for s in segs)/len(segs) for k in range(NSEG//2)]; return [k*fs/NSEG for k in range(NSEG//2)], p
spans=[]; start=None
for i in range(N):
    e = D["mspOverrideFlags"][i]=="2"
    if e and start is None: start=i
    elif not e and start is not None: spans.append((start,i-1)); start=None
spans=[s for s in spans if t[s[1]]-t[s[0]]>2]
BANDS=[(0.3,1),(1,2),(2,3),(3,5),(5,10),(10,20),(20,50),(50,100),(100,200),(200,300),(300,480)]
for si,(a,b) in enumerate(spans,1):
    mo=[float(v) for v in D["motor[0]"][a:b+1]]
    print("== ENGAGED %d  %.1f s  motor med %.0f" % (si, t[b]-t[a], st.median(mo)))
    for key in ("gyroRaw[1]","gyroADC[1]","gyroRaw[0]","accSmooth[2]"):
        x=[float(v) for v in D[key][a:b+1]]
        fr,p=welch(x); tot=sum(p[1:])
        shares=" ".join("%g-%g:%4.1f%%"%(lo,hi,100*sum(p[k] for k in range(1,len(p)) if lo<=fr[k]<hi)/tot) for lo,hi in BANDS)
        # top peaks above 20 Hz (local maxima, separated by >= 5 Hz)
        cand=sorted([k for k in range(2,len(p)-1) if fr[k]>=20 and p[k]>p[k-1] and p[k]>=p[k+1]], key=lambda k:-p[k])
        peaks=[]
        for k in cand:
            if all(abs(fr[k]-fr[j])>5 for j in peaks): peaks.append(k)
            if len(peaks)==4: break
        # RMS above 20 Hz for gyro (deg/s)
        hi_rms = math.sqrt(sum(p[k] for k in range(1,len(p)) if fr[k]>=20)/tot)*st.pstdev(x)
        unit = 1/10 if key.startswith("gyro") else 1/2048
        print("   %-12s %s | >20Hz RMS %.2f | peaks>20Hz: %s" % (key, shares, hi_rms*unit, ", ".join("%.0f Hz(%.1f%%)"%(fr[k],100*p[k]/tot) for k in peaks)))
