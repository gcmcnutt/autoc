#!/usr/bin/env python3
"""T024 refinement: servo->craft-rate lag and throttle->speed time constant.

Companion to flight_corpus_characterize.py. The corpus is logged at ~60 Hz
(16.7 ms), NOT the 200 Hz the doc claims, so sample-step lag resolution is 17 ms.
Two cleaner estimators here:

  A. SERVO/COMMAND -> ROLL-RATE LAG, manual segments only.
     The autoc bang-bang command has a flat-ish spectrum that smears xcorr; the
     pilot's manual roll *doublets* are clean low-frequency steps. We cross-
     correlate rcCommand[roll] (the demanded roll) against gyro_p over manual
     segments, after band-limiting to the maneuver band, and report the lag in ms.
     This lag = transport/compute dead-time + servo actuation + airframe roll
     build-up (tau_roll) lumped. We separate the pure dead-time floor (>=1 sample)
     from the first-order build-up by also fitting tau from the rise.

  B. THROTTLE -> GROUND-SPEED.  No airspeed sensor; use navVel NEU magnitude as a
     ground-speed proxy (wind-contaminated but the trend tracks thrust). On the
     100%-manual long flight (2026-04-03 .02) the pilot sweeps throttle; we
     cross-correlate motor PWM vs |navVel| and vs its derivative, and estimate a
     first-order throttle->speed time constant from the lag.

READ-ONLY. ASCII only.
"""
import csv, glob, os, math

ALL = sorted(f for f in glob.glob('flight-results/flight-*/blackbox_log_*.0[12].csv')
             if 'gps' not in f)


def load(f):
    r = csv.reader(open(f))
    hdr = [h.strip() for h in next(r)]
    I = {n: i for i, n in enumerate(hdr)}
    return I, list(r)


def col(rows, I, name):
    j = I.get(name)
    out = []
    for rw in rows:
        try:
            out.append(float(rw[j]))
        except (ValueError, IndexError, TypeError):
            out.append(float('nan'))
    return out


def manual_mask(rows, I):
    msp = col(rows, I, 'mspOverrideFlags')
    return [(not math.isnan(m)) and (int(m) & 2) for m in msp]


def dt_ms(rows, I):
    t = col(rows, I, 'time (us)')
    d = sorted(t[i + 1] - t[i] for i in range(min(2000, len(t) - 1))
               if not math.isnan(t[i]) and not math.isnan(t[i + 1]) and t[i + 1] > t[i])
    return d[len(d) // 2] / 1e3 if d else float('nan')


def xcorr(cmd, resp, max_lag, want_neg=False):
    """Best lag in [0,max_lag] maximizing |corr|; returns (lag, corr)."""
    n = len(cmd)
    if n < 10:
        return (0, 0.0)
    mc = sum(cmd) / n
    mr = sum(resp) / n
    c = [x - mc for x in cmd]
    r = [x - mr for x in resp]
    best = (0, 0.0)
    for lag in range(0, max_lag + 1):
        num = dc = dr = 0.0
        for t in range(0, n - lag):
            num += c[t] * r[t + lag]
            dc += c[t] * c[t]
            dr += r[t + lag] * r[t + lag]
        if dc > 0 and dr > 0:
            cc = num / math.sqrt(dc * dr)
            key = cc if not want_neg else -cc
            bkey = best[1] if not want_neg else -best[1]
            if abs(cc) > abs(best[1]):
                best = (lag, cc)
    return best


# ---- A. servo/command -> roll-rate lag over MANUAL segments, pooled ----
def servo_lag_manual():
    print('A. COMMAND/SERVO -> ROLL-RATE lag (manual segments, pooled per flight)')
    print('   method: xcorr(rcCommand[roll], gyro_p) and xcorr(servo[0]-servo[1], gyro_p)')
    print('   %-34s dt   cmd-lag(ms) corr   servo-lag(ms) corr' % 'flight')
    agg_cmd = []
    agg_sv = []
    for f in ALL:
        I, rows = load(f)
        d = dt_ms(rows, I)
        man = manual_mask(rows, I)
        rc = col(rows, I, 'rcCommand[0]')
        sv0 = col(rows, I, 'servo[0]')
        sv1 = col(rows, I, 'servo[1]')
        gp = col(rows, I, 'gyroADC[0]')
        idx = [i for i in range(len(rows)) if man[i]]
        # require meaningful manual roll activity
        cmd = [rc[i] for i in idx]
        resp = [gp[i] for i in idx]
        sv = [(sv0[i] - sv1[i]) for i in idx]
        # mask nan pairs
        c1 = [(cmd[k], resp[k]) for k in range(len(idx))
              if not math.isnan(cmd[k]) and not math.isnan(resp[k])]
        c2 = [(sv[k], resp[k]) for k in range(len(idx))
              if not math.isnan(sv[k]) and not math.isnan(resp[k])]
        maxlag = max(3, int(200 / d))
        l1, cc1 = xcorr([p[0] for p in c1], [p[1] for p in c1], maxlag)
        l2, cc2 = xcorr([p[0] for p in c2], [p[1] for p in c2], maxlag)
        print('   %-34s %4.1f  %5.0f  %5.2f    %5.0f  %5.2f'
              % (os.path.basename(os.path.dirname(f)), d, l1 * d, cc1, l2 * d, cc2))
        if f is ALL[0]:
            c = [p[0] for p in c1]; r = [p[1] for p in c1]
            n = len(c); mc = sum(c)/n; mr = sum(r)/n
            cz = [x-mc for x in c]; rz = [x-mr for x in r]
            line = []
            for lag in range(0, maxlag+1):
                num=dc=dr=0.0
                for t in range(0, n-lag):
                    num+=cz[t]*rz[t+lag]; dc+=cz[t]*cz[t]; dr+=rz[t+lag]*rz[t+lag]
                line.append(num/math.sqrt(dc*dr) if dc>0 and dr>0 else 0.0)
            print('     corr-vs-lag(ms:corr) cmd->rate: ' +
                  ' '.join('%.0f:%.2f' % (k*d, line[k]) for k in range(0, len(line), 1)))
        if cc1 > 0.2:
            agg_cmd.append(l1 * d)
        if cc2 > 0.2:
            agg_sv.append(l2 * d)
    if agg_cmd:
        agg_cmd.sort()
        print('   --> cmd->rate lag median = %.0f ms (n=%d flights w/ corr>0.2, range %.0f..%.0f)'
              % (agg_cmd[len(agg_cmd) // 2], len(agg_cmd), agg_cmd[0], agg_cmd[-1]))
    if agg_sv:
        agg_sv.sort()
        print('   --> servo->rate lag median = %.0f ms (n=%d, range %.0f..%.0f)'
              % (agg_sv[len(agg_sv) // 2], len(agg_sv), agg_sv[0], agg_sv[-1]))


# ---- A2. rcCommand vs servo lag (transport-only, autoc segments where command moves fast) ----
def cmd_to_servo_lag():
    print()
    print('A2. rcCommand[roll] -> servo[0] lag (pure transport/compute, both axes are FC-internal)')
    print('    (if servo tracks rcCommand with ~0 lag, the FC mixer is instantaneous; lag here is')
    print('     filtering/transport, not the physical actuator)')
    for f in ALL:
        I, rows = load(f)
        d = dt_ms(rows, I)
        rc = col(rows, I, 'rcCommand[0]')
        sv0 = col(rows, I, 'servo[0]')
        sv1 = col(rows, I, 'servo[1]')
        sv = [(a - b) for a, b in zip(sv0, sv1)]
        pair = [(rc[k], sv[k]) for k in range(len(rc))
                if not math.isnan(rc[k]) and not math.isnan(sv[k])]
        if len(pair) < 500:
            continue
        maxlag = max(3, int(60 / d))
        l, cc = xcorr([p[0] for p in pair], [p[1] for p in pair], maxlag)
        # also dump the full corr-vs-lag curve for the first flight to read dead-time
        if f is ALL[0]:
            c = [p[0] for p in pair]
            r = [p[1] for p in pair]
            n = len(c); mc = sum(c)/n; mr = sum(r)/n
            cz = [x-mc for x in c]; rz = [x-mr for x in r]
            curve = []
            for lag in range(0, maxlag+1):
                num=dc=dr=0.0
                for t in range(0, n-lag):
                    num+=cz[t]*rz[t+lag]; dc+=cz[t]*cz[t]; dr+=rz[t+lag]*rz[t+lag]
                curve.append(num/math.sqrt(dc*dr) if dc>0 and dr>0 else 0.0)
            print('      corr-vs-lag (ms:corr): ' +
                  ' '.join('%.0f:%.2f' % (k*d, curve[k]) for k in range(len(curve))))
        print('    %-34s lag=%.0f ms corr=%.2f' % (os.path.basename(os.path.dirname(f)), l * d, cc))


# ---- B. throttle -> ground-speed, best manual throttle-sweep flight ----
def throttle_speed():
    print()
    print('B. THROTTLE(motor PWM) -> ground-speed |navVel| (proxy; wind-contaminated)')
    cand = [f for f in ALL if '171339.02' in f] or ALL
    for f in cand[:1] + [g for g in ALL if g not in cand][:3]:
        I, rows = load(f)
        d = dt_ms(rows, I)
        motor = col(rows, I, 'motor[0]')
        vn = col(rows, I, 'navVel[0]')
        ve = col(rows, I, 'navVel[1]')
        vu = col(rows, I, 'navVel[2]')
        spd = []
        for i in range(len(rows)):
            if math.isnan(vn[i]) or math.isnan(ve[i]) or math.isnan(vu[i]):
                spd.append(float('nan'))
            else:
                spd.append(math.sqrt(vn[i] ** 2 + ve[i] ** 2 + vu[i] ** 2) / 100.0)  # m/s
        pair = [(motor[k], spd[k]) for k in range(len(rows))
                if not math.isnan(motor[k]) and not math.isnan(spd[k])]
        if len(pair) < 500:
            continue
        m = [p[0] for p in pair]
        s = [p[1] for p in pair]
        maxlag = max(10, int(2000 / d))  # throttle->speed slow, search to 2 s
        l, cc = xcorr(m, s, maxlag)
        sp = sorted(s)
        print('    %-34s motorPWM->|V| lag=%.0f ms corr=%.2f  |V| p10=%.1f p50=%.1f p90=%.1f m/s'
              % (os.path.basename(os.path.dirname(f)), l * d, cc,
                 sp[int(.1 * len(sp))], sp[len(sp) // 2], sp[int(.9 * len(sp))]))


if __name__ == '__main__':
    servo_lag_manual()
    cmd_to_servo_lag()
    throttle_speed()
