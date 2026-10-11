#!/usr/bin/env python3
"""Per-span control quality from the xiao v5 tick CSV (stdlib only)."""
import csv, math, statistics as st, sys, cmath
rows = [{k: v for k, v in r.items()} for r in csv.DictReader(open(sys.argv[1]))]
def f(r, k): return float(r[k])
def q(v, p):
    s = sorted(v); return s[min(len(s)-1, int(p*len(s)))]
def fft(a):
    n = len(a)
    if n == 1: return a
    e, o = fft(a[0::2]), fft(a[1::2])
    tw = [cmath.exp(-2j*math.pi*k/n)*o[k] for k in range(n//2)]
    return [e[k]+tw[k] for k in range(n//2)] + [e[k]-tw[k] for k in range(n//2)]
def welch(x, fs, nseg=256):
    step = nseg//2; segs = []
    win = [0.5-0.5*math.cos(2*math.pi*i/(nseg-1)) for i in range(nseg)]
    for s in range(0, len(x)-nseg+1, step):
        seg = x[s:s+nseg]; m = sum(seg)/nseg
        X = fft([complex((seg[i]-m)*win[i]) for i in range(nseg)])
        segs.append([abs(v)**2 for v in X[:nseg//2]])
    if not segs: return [], []
    p = [sum(s[k] for s in segs)/len(segs) for k in range(nseg//2)]
    return [k*fs/nseg for k in range(nseg//2)], p
def bandshare(freqs, p, lo, hi):
    tot = sum(p[1:]); return 100*sum(p[k] for k in range(1,len(p)) if lo <= freqs[k] < hi)/tot if tot else float('nan')
spans = sorted(set(int(r["span_id"]) for r in rows))
print("span ticks path | trkerr med/p90/max | step_score mean | pitch rail% roll rail% thr+rail% thr mean | gnd spd med | pos_d step max | pos_d range | vel_d p95")
for s in spans:
    R = [r for r in rows if int(r["span_id"]) == s]
    path = R[0]["path_index"]
    err = [math.sqrt((f(r,"pos_n")-f(r,"rabbit_n"))**2+(f(r,"pos_e")-f(r,"rabbit_e"))**2+(f(r,"pos_d")-f(r,"rabbit_d"))**2) for r in R]
    ss = [f(r,"step_score") for r in R]
    op = [f(r,"out_pitch") for r in R]; orl = [f(r,"out_roll") for r in R]; ot = [f(r,"out_throttle") for r in R]
    rail = lambda v: 100*sum(1 for x in v if abs(x) >= 0.99)/len(v)
    gs = [math.hypot(f(r,"vel_n"), f(r,"vel_e")) for r in R]
    pd = [f(r,"pos_d") for r in R]
    dpd = [abs(pd[i+1]-pd[i]) for i in range(len(pd)-1)]
    vd = [abs(f(r,"vel_d")) for r in R]
    print("%d %4d %s | %5.1f %5.1f %5.1f | %.3f | %4.1f %4.1f %4.1f %+.2f | %5.1f | %5.2f | %6.1f..%6.1f | %4.1f" % (
        s, len(R), path, st.median(err), q(err,.9), max(err), st.mean(ss), rail(op), rail(orl),
        100*sum(1 for x in ot if x >= 0.99)/len(ot), st.mean(ot), st.median(gs), max(dpd), min(pd), max(pd), q(vd,.95)))
print()
print("--- 20 Hz spectra per span (same method as 09-13 addendum A1): peak Hz | share 2-3 Hz | share 5-10 Hz | RMS(NN units) ---")
for s in spans:
    R = [r for r in rows if int(r["span_id"]) == s]
    for key in ("gyro_q", "out_pitch", "gyro_p", "out_roll", "out_throttle", "accel_z"):
        x = [f(r,key) for r in R]
        fr, p = welch(x, 20.0, 256 if len(x) >= 400 else 128)
        if not fr: continue
        pk = max(range(1,len(p)), key=lambda k: p[k])
        print("  span %d %-12s peak %5.2f Hz | 2-3: %5.1f%% | 1-2: %5.1f%% | 3-5: %5.1f%% | 5-10: %5.1f%% | rms %.4f" % (
            s, key, fr[pk], bandshare(fr,p,2,3), bandshare(fr,p,1,2), bandshare(fr,p,3,5), bandshare(fr,p,5,10), st.pstdev(x)))
print()
print("--- throttle switching (sign changes/s of out_throttle about its mean) and pitch cmd sign changes/s ---")
for s in spans:
    R = [r for r in rows if int(r["span_id"]) == s]
    for key in ("out_throttle", "out_pitch", "out_roll"):
        x = [f(r,key) for r in R]; m = st.mean(x)
        sc = sum(1 for i in range(1,len(x)) if (x[i]-m)*(x[i-1]-m) < 0)
        print("  span %d %-12s %.2f /s" % (s, key, sc/(len(x)*0.05)))
print()
print("--- per-tick dropouts / timing: dt stats (ms) and state_valid ---")
for s in spans:
    R = [r for r in rows if int(r["span_id"]) == s]
    ts = [f(r,"timestamp_ms") for r in R]
    dt = [ts[i+1]-ts[i] for i in range(len(ts)-1)]
    print("  span %d dt med %.1f max %.1f min %.1f | state_valid=0 ticks: %d | recurrent_reset ticks: %d" % (
        s, st.median(dt), max(dt), min(dt), sum(1 for r in R if r["state_valid"] != "1"), sum(1 for r in R if r["recurrent_reset"] not in ("0","0.0"))))
