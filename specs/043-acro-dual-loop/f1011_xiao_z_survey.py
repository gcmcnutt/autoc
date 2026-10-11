#!/usr/bin/env python3
"""Whole-flight navPos[2] trace reconstructed from the xiao log (breadcrumbs pos_raw_d when
armed-not-engaged + ticks origin_d+pos_d when engaged), aligned to INAV time by the INAV_CLOCK
event pairs, then compared against BaroAlt and GPS altitude. Substitutes for the stripped NAV_POS."""
import csv, re, subprocess, sys, math, statistics as st, bisect
S = sys.argv[1]; binf = sys.argv[2]; bb60 = sys.argv[3]; gps = sys.argv[4]
out = subprocess.run(["python3", "src/analytics/flightlog_decode.py", binf, "-o", S+"/_tmp_ticks.csv"], capture_output=True, text=True); out = out.stdout + out.stderr
clk = [(int(m.group(1)), int(m.group(2))) for m in re.finditer(r"event t=\s*(\d+) INAV_CLOCK\s+value=(\d+)", out)]
orig = {int(m.group(1)): float(m.group(2)) for m in re.finditer(r"span (\d+): .*origin_ned=\(([-\d.]+),[-\d.]+,([-\d.]+)\)", out)}
orig = {int(m.group(1)): float(m.group(4)) for m in re.finditer(r"span (\d+): .*origin_ned=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", out)}
print("origins d:", orig)
print("INAV_CLOCK offsets (ms):", [b-a for a,b in clk])
xs = [a for a,b in clk]; offs = [b-a for a,b in clk]
def to_inav(t_ms):
    i = bisect.bisect_left(xs, t_ms)
    if i == 0: return t_ms + offs[0]
    if i >= len(xs): return t_ms + offs[-1]
    f = (t_ms - xs[i-1])/(xs[i]-xs[i-1]); return t_ms + offs[i-1] + f*(offs[i]-offs[i-1])
# z trace (NED down, m relative to home) from breadcrumbs + ticks
tr = []
for r in csv.DictReader(open(S+"/xiao_path.csv")):
    tr.append((to_inav(float(r["timestamp_ms"]))/1000, float(r["pos_raw_d"]), "crumb"))
eng = {}
for r in csv.DictReader(open(S+"/xiao_ticks.csv")):
    sp = int(r["span_id"]); tr.append((to_inav(float(r["timestamp_ms"]))/1000, orig[sp] + float(r["pos_d"]), "span%d"%sp))
    eng.setdefault(sp, [to_inav(float(r["timestamp_ms"]))/1000]*2); eng[sp][1] = to_inav(float(r["timestamp_ms"]))/1000
tr.sort()
print("trace samples %d, %.1f-%.1f s, crumb dt median %.0f ms" % (len(tr), tr[0][0], tr[-1][0],
      1000*st.median([tr[i+1][0]-tr[i][0] for i in range(len(tr)-1) if tr[i][2]=="crumb" and tr[i+1][2]=="crumb"])))
print("engaged (INAV s):", {k: ("%.1f-%.1f" % tuple(v)) for k,v in eng.items()})
# step survey
print("\n--- |dz| > 3 m between consecutive samples (navPos z, NED m) ---")
n5 = 0
for i in range(1, len(tr)):
    dz = tr[i][1]-tr[i-1][1]; dt = tr[i][0]-tr[i-1][0]
    if abs(dz) > 3:
        n5 += abs(dz) > 5
        print("  t=%.2f  dz=%+6.2f m over %.0f ms  (%s -> %s)  rate %.1f m/s" % (tr[i][0], dz, dt*1000, tr[i-1][2], tr[i][2], dz/dt if dt else float('inf')))
print("  events >5 m:", n5)
# max per-sample vertical rate overall
rates = [abs(tr[i][1]-tr[i-1][1])/(tr[i][0]-tr[i-1][0]) for i in range(1,len(tr)) if tr[i][0]-tr[i-1][0] > 0.02]
print("  |dz/dt| p99 %.1f max %.1f m/s" % (sorted(rates)[int(.99*len(rates))], max(rates)))
# baro + gps alignment
bb = [(float(r["time (us)"])/1e6, float(r["BaroAlt (cm)"])/100) for r in csv.DictReader(open(bb60))]
bt = [b[0] for b in bb]
g = [(float(r["time (us)"])/1e6, float(r["GPS_altitude"]), int(r["GPS_numSat"]), float(r["GPS_epv"]), float(r["GPS_eph"])) for r in csv.DictReader(open(gps))]
gt = [x[0] for x in g]
def nearest(ts, arr, t):
    i = bisect.bisect_left(ts, t); i = min(max(i,0), len(arr)-1)
    if i > 0 and abs(ts[i-1]-t) < abs(ts[i]-t): i -= 1
    return arr[i]
# navPos z is NED down relative to home; altitude_est = -z + home_alt. Use baro at arm as the home reference.
home_baro = nearest(bt, bb, tr[0][0])[1]
print("\nhome baro at first sample: %.1f m; GPS alt at first sample: %.0f m" % (home_baro, nearest(gt, g, tr[0][0])[1]))
gps0 = nearest(gt, g, tr[0][0])[1]
segs = [("pre-engage", tr[0][0], eng[1][0])]
for k in sorted(eng):
    segs.append(("ENGAGED %d"%k, eng[k][0], eng[k][1]))
    nxt = eng[k+1][0] if k+1 in eng else tr[-1][0]
    segs.append(("after %d"%k, eng[k][1], nxt))
print("\n%-12s %6s | %-24s | %-24s | %5s %6s %6s" % ("seg","n","est-baro med/p95/max(abs)","est-gps med/p95/max(abs)","sats","epv","eph"))
for name,a,b in segs:
    pts = [p for p in tr if a <= p[0] <= b]
    if len(pts) < 5: continue
    gb = [(-p[1]) - (nearest(bt,bb,p[0])[1]-home_baro) for p in pts]
    gg = [(-p[1]) - (nearest(gt,g,p[0])[1]-gps0) for p in pts]
    sats = [nearest(gt,g,p[0])[2] for p in pts]; epv=[nearest(gt,g,p[0])[3] for p in pts]; eph=[nearest(gt,g,p[0])[4] for p in pts]
    ab = sorted(abs(x) for x in gb); ag = sorted(abs(x) for x in gg)
    print("%-12s %6d | %+6.1f %6.1f %6.1f | %+6.1f %6.1f %6.1f | %5.0f %6.0f %6.0f" % (name, len(pts), st.median(gb), ab[int(.95*len(ab))], ab[-1], st.median(gg), ag[int(.95*len(ag))], ag[-1], st.median(sats), st.median(epv), st.median(eph)))

print("\n--- largest |est-baro| samples OUTSIDE engaged spans ---")
outside=[p for p in tr if p[2]=="crumb"]
gaps=sorted(((abs((-p[1])-(nearest(bt,bb,p[0])[1]-home_baro)), p[0], p[1], nearest(bt,bb,p[0])[1]) for p in outside), reverse=True)[:8]
for g_,t_,z_,b_ in gaps: print("  t=%.2f  est alt %.1f  baro %.1f  gap %.1f" % (t_, -z_, b_, g_))
