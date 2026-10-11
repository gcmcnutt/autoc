import csv, math, statistics as st, sys
rows = list(csv.DictReader(open(sys.argv[1])))
f = lambda r,k: float(r[k])
def pitch_deg(r):
    w,x,y,z = (f(r,"quat_w"), f(r,"quat_x"), f(r,"quat_y"), f(r,"quat_z"))
    s = 2*(w*y - z*x); s = max(-1,min(1,s)); return math.degrees(math.asin(s))
def roll_deg(r):
    w,x,y,z = (f(r,"quat_w"), f(r,"quat_x"), f(r,"quat_y"), f(r,"quat_z"))
    return math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
for s in sorted(set(int(r["span_id"]) for r in rows)):
    R = [r for r in rows if int(r["span_id"]) == s]; n = len(R)
    op = [f(r,"out_pitch") for r in R]
    up = [i for i,v in enumerate(op) if v >= 0.99]; dn = [i for i,v in enumerate(op) if v <= -0.99]
    def runs(ix):
        rs=[]; 
        for i in ix:
            if rs and i == rs[-1][1]+1: rs[-1][1]=i
            else: rs.append([i,i])
        return rs
    ru, rd = runs(up), runs(dn)
    pa = [pitch_deg(r) for r in R]; ra = [abs(roll_deg(r)) for r in R]
    fpa = [math.degrees(math.atan2(-f(r,"vel_d"), math.hypot(f(r,"vel_n"), f(r,"vel_e")))) for r in R]
    print("span %d: out_pitch mean %+.2f med %+.2f | rail +: %4.1f%% (%d runs, longest %d ticks) | rail -: %4.1f%% (%d runs, longest %d ticks) | first-2s rail %4.1f%%" % (
        s, st.mean(op), st.median(op), 100*len(up)/n, len(ru), max([b-a+1 for a,b in ru] or [0]), 100*len(dn)/n, len(rd), max([b-a+1 for a,b in rd] or [0]),
        100*sum(1 for v in op[:40] if abs(v)>=0.99)/40))
    print("         quat pitch deg: med %+5.1f p5 %+5.1f p95 %+5.1f min %+5.1f max %+5.1f | |roll| med %4.1f p95 %5.1f | flight-path angle med %+5.1f p5 %+5.1f p95 %+5.1f | airspeed(NN)*13 med %.1f" % (
        st.median(pa), sorted(pa)[int(.05*n)], sorted(pa)[int(.95*n)], min(pa), max(pa), st.median(ra), sorted(ra)[int(.95*n)], st.median(fpa), sorted(fpa)[int(.05*n)], sorted(fpa)[int(.95*n)], 13*st.median(f(r,"airspeed") for r in R)))
    # where in the span do the + rail runs occur (tick index ranges of the longest 3)
    top = sorted(ru, key=lambda ab: ab[0]-ab[1])[:3]
    print("         longest + rail runs at ticks:", [(a,b) for a,b in top], " ; longest - runs:", [(a,b) for a,b in sorted(rd, key=lambda ab: ab[0]-ab[1])[:3]])
    # out_pitch vs rc_pitch mapping check
    rp = [f(r,"rc_pitch") for r in R]
    print("         rc_pitch med %.0f min %.0f max %.0f ; corr(out_pitch, rc_pitch) sign check: out>0.5 -> rc med %.0f ; out<-0.5 -> rc med %.0f" % (
        st.median(rp), min(rp), max(rp), st.median([rp[i] for i in range(n) if op[i] > 0.5] or [0]), st.median([rp[i] for i in range(n) if op[i] < -0.5] or [0])))
