#!/usr/bin/env python3
"""T024 throttle-step response + servo-mixing sanity, on the 100%-manual long
flight (2026-04-03 .02, 225 s). Identifies clean throttle step-ups (motor PWM
jumps and holds) and measures the ground-speed rise to estimate a first-order
throttle->thrust/speed time constant. Also confirms servo[0]/servo[1] roles
(elevon: roll=diff, pitch=sum) by correlating against gyro_p / gyro_q.
READ-ONLY. ASCII only."""
import csv, math, glob

F = [f for f in glob.glob('flight-results/flight-20260403/blackbox_log_*.02.csv')
     if 'gps' not in f][0]


def load(f):
    r = csv.reader(open(f))
    hdr = [h.strip() for h in next(r)]
    I = {n: i for i, n in enumerate(hdr)}
    return I, list(r)


def col(rows, I, name):
    j = I.get(name)
    o = []
    for rw in rows:
        try:
            o.append(float(rw[j]))
        except (ValueError, IndexError, TypeError):
            o.append(float('nan'))
    return o


def pearson(a, b):
    p = [(x, y) for x, y in zip(a, b) if not math.isnan(x) and not math.isnan(y)]
    n = len(p)
    if n < 10:
        return float('nan')
    ax = sum(x for x, _ in p) / n
    ay = sum(y for _, y in p) / n
    num = sum((x - ax) * (y - ay) for x, y in p)
    dx = math.sqrt(sum((x - ax) ** 2 for x, _ in p))
    dy = math.sqrt(sum((y - ay) ** 2 for _, y in p))
    return num / (dx * dy) if dx > 0 and dy > 0 else float('nan')


I, rows = load(F)
t = col(rows, I, 'time (us)')
d = (t[2] - t[1]) / 1e3  # ms
motor = col(rows, I, 'motor[0]')
sv0 = col(rows, I, 'servo[0]')
sv1 = col(rows, I, 'servo[1]')
gp = col(rows, I, 'gyroADC[0]')
gq = col(rows, I, 'gyroADC[1]')
vn = col(rows, I, 'navVel[0]')
ve = col(rows, I, 'navVel[1]')
vu = col(rows, I, 'navVel[2]')
spd = [math.sqrt(vn[i]**2 + ve[i]**2 + vu[i]**2) / 100.0
       if not (math.isnan(vn[i]) or math.isnan(ve[i]) or math.isnan(vu[i])) else float('nan')
       for i in range(len(rows))]

print('Flight: 2026-04-03 .02  dur=%.0fs  dt=%.1fms (%.0f Hz)  100%% manual' %
      ((t[-1] - t[0]) / 1e6, d, 1000 / d))

# servo role sanity
print()
print('Servo-role sanity (Pearson):')
diff = [sv0[i] - sv1[i] for i in range(len(rows))]
summ = [sv0[i] + sv1[i] for i in range(len(rows))]
print('  corr(servo0-servo1, gyro_p roll) = %.2f' % pearson(diff, gp))
print('  corr(servo0+servo1, gyro_q pitch) = %.2f' % pearson(summ, gq))
print('  corr(servo0, gyro_p)=%.2f  corr(servo1, gyro_p)=%.2f' %
      (pearson(sv0, gp), pearson(sv1, gp)))

# motor command distribution + dynamics
mvals = sorted(x for x in motor if not math.isnan(x))
print()
print('motor[0] PWM pctiles (5/25/50/75/95): %s' %
      [round(mvals[int(p * len(mvals))]) for p in (.05, .25, .5, .75, .95)])
dm = [abs(motor[i + 1] - motor[i]) for i in range(len(motor) - 1)
      if not math.isnan(motor[i]) and not math.isnan(motor[i + 1])]
print('  frac |dmotor|>100us/sample = %.3f  max step = %.0f us' %
      (sum(1 for x in dm if x > 100) / len(dm), max(dm)))

# smoothed speed (0.25 s moving average) to suppress wind/GPS jitter
sm = int(250 / d) | 1
sspd = [float('nan')] * len(spd)
for i in range(len(spd)):
    seg = [spd[i + k] for k in range(-(sm // 2), sm // 2 + 1)
           if 0 <= i + k < len(spd) and not math.isnan(spd[i + k])]
    if seg:
        sspd[i] = sum(seg) / len(seg)

# throttle step detection: motor rises by >200us over 5 samples from a low base and holds
print()
print('Throttle step-ups (motor +200us over ~80ms from base<1500, hold >0.8s):')
W = int(900 / d)
steps = []
i = 5
while i < len(rows) - W:
    if any(math.isnan(motor[k]) for k in range(i - 3, i + W)):
        i += 1
        continue
    pre = sum(motor[i - 4:i]) / 4
    post = motor[i + 5]
    if post - pre > 200 and pre < 1500:
        if min(motor[i + 5:i + W]) > pre + 120:
            steps.append(i)
            i += W
            continue
    i += 1
print('  found %d step-ups' % len(steps))
taus = []
for s in steps:
    v0 = sspd[s - 1]
    seg = [sspd[s + k] for k in range(0, W) if not math.isnan(sspd[s + k])]
    if not seg or math.isnan(v0):
        continue
    vinf = max(seg)
    target = v0 + 0.63 * (vinf - v0)
    tr = None
    for k in range(0, W):
        if not math.isnan(sspd[s + k]) and sspd[s + k] >= target:
            tr = k * d
            break
    if tr is not None and vinf - v0 > 1.0:
        taus.append(tr)
        print('    t=%.1fs  V %.1f->%.1f m/s  rise63%%=%.0f ms' %
              ((t[s] - t[0]) / 1e6, v0, vinf, tr))
if taus:
    taus.sort()
    print('  --> speed rise (63%%) median=%.0f ms  range %.0f..%.0f ms' %
          (taus[len(taus) // 2], taus[0], taus[-1]))
    print('      (this lumps thrust build-up + airframe accel/drag; thrust tau is shorter)')
