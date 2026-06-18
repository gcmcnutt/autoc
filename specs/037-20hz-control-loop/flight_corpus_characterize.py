#!/usr/bin/env python3
"""T024 real-airframe characterization across the full flight corpus.

Refines specs/037-20hz-control-loop/rollrate_crosscheck.py (which used only the
2026-05-17 autoc-engage spans) by mining EVERY raw INAV blackbox CSV in
flight-results/, INCLUDING manual (pilot) segments, to build cleaner physical
models for the FDM 20 Hz retrain:

  1. INVENTORY: per-flight duration, sample rate, manual vs autoc-engaged split.
  2. ROLL AUTHORITY: |gyro_p| (deg/s) envelope max/p95/p99 + command->rate gain.
  3. SERVO LAG: cross-correlate servo output / rcCommand-roll vs gyro_p to find
     the lead time (samples) that maximizes correlation = actuator+airframe lag.
  4. THROTTLE->POWER: motor PWM / rcCommand-throttle vs vertical accel + climb,
     and a coarse first-order time-constant from throttle step responses.

Units (docs/INAV_BLACKBOX.md, docs/sim-to-real-analysis.md):
  gyroADC[0..2] = deg/s (roll p, pitch q, yaw r) raw, post-filter.
  rcCommand[0]=roll [-500,500], rcCommand[3]=throttle. servo[N]=PWM us 1000-2000.
  motor[0]=PWM us ~1000-2000. mspOverrideFlags bit1 (val&2) = MSP/autoc override.
  Main frame ~200 Hz. BaroAlt cm. navVel NEU cm/s (navVel[2]=Up).
READ-ONLY. ASCII only.
"""
import csv, glob, os, math

FILES = sorted(f for f in glob.glob('flight-results/flight-*/blackbox_log_*.0[12].csv')
               if 'gps' not in f)


def load(f):
    r = csv.reader(open(f))
    hdr = [h.strip() for h in next(r)]
    I = {n: i for i, n in enumerate(hdr)}
    rows = list(r)
    return I, rows


def colf(rows, I, name):
    j = I.get(name)
    out = []
    for rw in rows:
        try:
            out.append(float(rw[j]))
        except (ValueError, IndexError, TypeError):
            out.append(float('nan'))
    return out


def absstats(xs):
    a = sorted(abs(x) for x in xs if not math.isnan(x))
    if not a:
        return None
    n = len(a)
    p = lambda q: a[min(n - 1, int(q * n))]
    return dict(n=n, p50=p(.5), p90=p(.9), p95=p(.95), p99=p(.99), mx=a[-1])


def std(xs):
    xs = [x for x in xs if not math.isnan(x)]
    if len(xs) < 2:
        return float('nan')
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def segments(over):
    """Return list of (start,end,is_autoc) contiguous runs over the boolean list."""
    segs = []
    i = 0
    n = len(over)
    while i < n:
        j = i
        while j < n and over[j] == over[i]:
            j += 1
        segs.append((i, j, over[i]))
        i = j
    return segs


def xcorr_lag(cmd, resp, dt_ms, max_lag_samp):
    """Lag (samples, ms) that maximizes corr(cmd[t], resp[t+lag]); resp lags cmd.
    Positive lag => response delayed behind command. Uses mean-removed signals."""
    c = [x for x in cmd]
    r = [x for x in resp]
    n = len(c)
    mc = sum(c) / n
    mr = sum(r) / n
    c = [x - mc for x in c]
    r = [x - mr for x in r]
    best = (None, -2.0)
    for lag in range(0, max_lag_samp + 1):
        num = 0.0
        dc = 0.0
        dr = 0.0
        for t in range(0, n - lag):
            num += c[t] * r[t + lag]
            dc += c[t] * c[t]
            dr += r[t + lag] * r[t + lag]
        if dc > 0 and dr > 0:
            cc = num / math.sqrt(dc * dr)
            if cc > best[1]:
                best = (lag, cc)
    lag = best[0]
    return lag, lag * dt_ms, best[1]


def analyze_file(f):
    I, rows = load(f)
    t = colf(rows, I, 'time (us)')
    n = len(rows)
    dur = (t[-1] - t[0]) / 1e6
    dts = sorted(t[i + 1] - t[i] for i in range(min(2000, n - 1))
                 if not math.isnan(t[i]) and not math.isnan(t[i + 1]) and t[i+1] > t[i])
    dt_ms = dts[len(dts) // 2] / 1e3 if dts else float('nan')
    hz = 1000.0 / dt_ms if dt_ms else float('nan')

    gp = colf(rows, I, 'gyroADC[0]')
    rc_roll = colf(rows, I, 'rcCommand[0]')
    rc_thr = colf(rows, I, 'rcCommand[3]')
    sv0 = colf(rows, I, 'servo[0]')
    sv1 = colf(rows, I, 'servo[1]')
    motor = colf(rows, I, 'motor[0]')
    baro = colf(rows, I, 'BaroAlt (cm)')
    navvz = colf(rows, I, 'navVel[2]')  # Up cm/s
    msp = colf(rows, I, 'mspOverrideFlags')

    over = [(not math.isnan(m)) and (int(m) & 2) for m in msp]
    n_over = sum(over)
    n_man = n - n_over

    name = os.path.basename(os.path.dirname(f)) + '/' + os.path.basename(f)[14:]
    print('=' * 78)
    print('%s' % name)
    print('  dur=%.1fs  n=%d  dt=%.2fms (%.0f Hz)  autoc=%.0f%%  manual=%.0f%%'
          % (dur, n, dt_ms, hz, 100 * n_over / n, 100 * n_man / n))

    # per-segment durations
    segs = segments(over)
    for (a, b, isa) in segs:
        sd = (t[b - 1] - t[a]) / 1e6 if b > a else 0
        if sd >= 2.0:
            print('     seg %-6s %6.1fs  rows %d..%d'
                  % ('AUTOC' if isa else 'MANUAL', sd, a, b))

    # Build manual and autoc index masks (only segments >= 2s to avoid edges)
    man_idx = []
    aut_idx = []
    for (a, b, isa) in segs:
        sd = (t[b - 1] - t[a]) / 1e6 if b > a else 0
        if sd < 2.0:
            continue
        (aut_idx if isa else man_idx).extend(range(a, b))

    for label, idx in (('MANUAL', man_idx), ('AUTOC', aut_idx)):
        if len(idx) < 50:
            continue
        g = [gp[i] for i in idx]
        s = absstats(g)
        sdv = std(g)
        # command->rate gain: regress |gyro_p| on |rcCommand_roll| via robust ratio
        # gain = p95(|gyro_p|) / p95(|rc_roll|/500)  (deg/s per full deflection)
        rr = [abs(rc_roll[i]) / 500.0 for i in idx if not math.isnan(rc_roll[i])]
        rr.sort()
        gp95cmd = (rr[min(len(rr) - 1, int(.95 * len(rr)))] if rr else float('nan'))
        gain = s['p95'] / gp95cmd if gp95cmd and gp95cmd > 0.05 else float('nan')
        print('  [%s] |roll rate| deg/s: std=%.0f p50=%.0f p90=%.0f p95=%.0f p99=%.0f max=%.0f'
              % (label, sdv, s['p50'], s['p90'], s['p95'], s['p99'], s['mx']))
        f200 = sum(1 for x in g if abs(x) > 200) / len(g)
        f300 = sum(1 for x in g if abs(x) > 300) / len(g)
        f400 = sum(1 for x in g if abs(x) > 400) / len(g)
        print('         frac>200=%.0f%% >300=%.0f%% >400=%.0f%%  cmd->rate gain~%.0f deg/s/full'
              % (100 * f200, 100 * f300, 100 * f400, gain))

        # servo lag: prefer servo[0]/servo[1] (actual actuator), fall back to rc_roll
        # roll servo: elevons differential. Use servo[0]-servo[1] as roll actuator proxy.
        sv_roll = []
        ok_sv = all(not math.isnan(sv0[i]) for i in idx[:200]) and \
                all(not math.isnan(sv1[i]) for i in idx[:200])
        for i in idx:
            sv_roll.append((sv0[i] - sv1[i]) if ok_sv else rc_roll[i])
        gpi = [gp[i] for i in idx]
        # clean nans
        pair = [(sv_roll[k], gpi[k]) for k in range(len(idx))
                if not math.isnan(sv_roll[k]) and not math.isnan(gpi[k])]
        if len(pair) > 400 and not math.isnan(dt_ms):
            cmd = [p[0] for p in pair]
            resp = [p[1] for p in pair]
            maxlag = max(2, int(60 / dt_ms))  # search up to 60 ms
            lag, lag_ms, cc = xcorr_lag(cmd, resp, dt_ms, maxlag)
            src = 'servo[0-1]' if ok_sv else 'rcCmd_roll'
            print('         servo-lag (%s vs gyro_p): lag=%d samp = %.1f ms (corr=%.2f)'
                  % (src, lag, lag_ms, cc))

    # THROTTLE -> POWER  (use whole flight; need throttle dynamic range)
    # vertical accel proxy: d(navVel_up)/dt; climb = navVel_up.  motor PWM normalized.
    mvalid = [x for x in motor if not math.isnan(x)]
    if mvalid:
        mn, mx = min(mvalid), max(mvalid)
        # throttle command normalized 0..1 from motor PWM range
        print('  throttle: motor[0] PWM range %.0f..%.0f (span %.0f us)' % (mn, mx, mx - mn))
        # correlate motor PWM (lead) vs vertical accel (climb-rate derivative)
        if not math.isnan(dt_ms):
            vz = navvz  # Up cm/s
            az = [float('nan')] * n
            for i in range(1, n):
                if not math.isnan(vz[i]) and not math.isnan(vz[i - 1]):
                    az[i] = (vz[i] - vz[i - 1]) / (dt_ms / 1000.0) / 100.0  # m/s^2
            pair = [(motor[i], az[i]) for i in range(n)
                    if not math.isnan(motor[i]) and not math.isnan(az[i])]
            if len(pair) > 500:
                cmd = [p[0] for p in pair]
                resp = [p[1] for p in pair]
                maxlag = max(4, int(500 / dt_ms))  # throttle->climb is slow, up to 500ms
                lag, lag_ms, cc = xcorr_lag(cmd, resp, dt_ms, maxlag)
                print('         motor PWM -> vertical accel: lag=%d samp = %.0f ms (corr=%.2f)'
                      % (lag, lag_ms, cc))


def main():
    print('Flight corpus: %d raw blackbox CSVs' % len(FILES))
    for f in FILES:
        analyze_file(f)


if __name__ == '__main__':
    main()
