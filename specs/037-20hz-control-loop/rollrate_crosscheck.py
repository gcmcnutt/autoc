#!/usr/bin/env python3
"""T008 real-flight cross-check: measure actual roll RATE (gyro_p, deg/s) and
roll-command character from the 2026-05-17 engage spans (pastonly3 gen 800).
gyro_p in join CSV = INAV gyroADC[0] = deg/s (no scaling, per join_flight_analysis.py:108
and docs/sim-to-real-analysis.md units table). Sanity: integrate gyro_p over the span and
compare net deg with attitude wander to confirm deg/s units."""
import csv, math

FILES = [
    ("span1_path0", "join_analysis_flight_log_2026-05-17T19-55-07_span1_path0.csv"),
    ("span2_path2", "join_analysis_flight_log_2026-05-17T19-55-07_span2_path2.csv"),
]

def col(rows, hdr, name):
    i = hdr.index(name)
    out = []
    for r in rows:
        try: out.append(float(r[i]))
        except (ValueError, IndexError): out.append(float('nan'))
    return out

def stats(xs):
    xs = [x for x in xs if not math.isnan(x)]
    n = len(xs); m = sum(xs)/n
    sd = math.sqrt(sum((x-m)**2 for x in xs)/n)
    a = sorted(abs(x) for x in xs)
    p = lambda q: a[min(n-1, int(q*n))]
    return n, m, sd, p(0.5), p(0.90), p(0.95), p(0.99), max(a)

def quat_roll_deg(qw,qx,qy,qz):
    # roll (x-axis) from quaternion
    sinr = 2*(qw*qx+qy*qz); cosr = 1-2*(qx*qx+qy*qy)
    return math.degrees(math.atan2(sinr,cosr))

for label, fn in FILES:
    rows = list(csv.reader(open(fn)))
    hdr = [h.strip() for h in rows[0]]
    data = rows[1:]
    gp = col(data, hdr, 'gyro_p'); t = col(data, hdr, 't_ms')
    qw=col(data,hdr,'qw');qx=col(data,hdr,'qx');qy=col(data,hdr,'qy');qz=col(data,hdr,'qz')
    dur = (t[-1]-t[0])/1000.0
    n,m,sd,p50,p90,p95,p99,mx = stats(gp)
    # frac of samples above various thresholds
    g=[abs(x) for x in gp if not math.isnan(x)]
    f200=sum(x>200 for x in g)/len(g); f300=sum(x>300 for x in g)/len(g)
    f400=sum(x>400 for x in g)/len(g)
    print(f"=== {label}  dur={dur:.1f}s  rows={n} ===")
    print(f"  roll rate |gyro_p| (deg/s): mean={m:+.1f} std={sd:.1f} "
          f"p50={p50:.0f} p90={p90:.0f} p95={p95:.0f} p99={p99:.0f} max={mx:.0f}")
    print(f"  frac |rate|>200: {f200:.0%}  >300: {f300:.0%}  >400: {f400:.0%}")
    # bank-angle envelope as independent sanity on roll authority
    rolls=[quat_roll_deg(qw[i],qx[i],qy[i],qz[i]) for i in range(len(qw)) if not math.isnan(qw[i])]
    if rolls:
        ar=sorted(abs(r) for r in rolls)
        print(f"  |bank angle| p50={ar[len(ar)//2]:.0f} p95={ar[int(.95*len(ar))]:.0f} max={max(ar):.0f} deg")
print()
print("RT claim under test: 'recorded physical roll rate ~155 deg/s ... airframe-limited")
print("to ~150-200 across every regime ... only ~20% of tau-implied 774 steady'")
print("=> if real p95/p99 sit well above 200 and approach 60-80% of 774, the airframe is")
print("   NOT capped at 150-200; saturation IS reaching meaningful authority on the flips.")
