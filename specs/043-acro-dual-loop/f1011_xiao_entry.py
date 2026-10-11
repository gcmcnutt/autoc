import csv, math, statistics as st, sys
rows = list(csv.DictReader(open(sys.argv[1])))
f = lambda r,k: float(r[k])
def pitch_deg(r):
    w,x,y,z = (f(r,"quat_w"), f(r,"quat_x"), f(r,"quat_y"), f(r,"quat_z")); return math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
for s in (1,2,3,4):
    R = [r for r in rows if int(r["span_id"]) == s]
    print("span %d first 30 ticks: tick | out_pitch out_roll out_thr | pitch° | vel_d | pos_d rabbit_d | dist_0*26 | airspeed*13 | step_score" % s)
    for r in R[:30:2]:
        print("   %3s | %+5.2f %+5.2f %+5.2f | %+6.1f | %+5.1f | %+6.1f %+6.1f | %5.1f | %5.1f | %.3f" % (r["tick_counter"], f(r,"out_pitch"), f(r,"out_roll"), f(r,"out_throttle"), pitch_deg(r), f(r,"vel_d"), f(r,"pos_d"), f(r,"rabbit_d"), 26*f(r,"dist_0"), 13*f(r,"airspeed"), f(r,"step_score")))
