#!/usr/bin/env python3
# 3x3 cmd->response scatter from sim data.dat (024 variant).
#
# Semantic note (spec 026, ACRO delegation): outPt/outRl are *rate commands*
# (NN [-1,+1] scaled by ACRO_MAX_RATE_*) not direct surface deflections. The
# inner ACRO PID converts them to surfaces in CRRCSim. cmd→gyro correlation
# is now mediated by the rate PID, so expect tighter/faster correlation than
# pre-026 raw-MANUAL scatter. data.dat carries rateCmd*/rateAch* if the run
# captured PidInternals (elite reeval); use those for direct rate tracking.

#
# Row 1: cmd(t) vs gyro-rate(t)                     (instantaneous, lag=0)
# Row 2: cmd(t) vs gyro-rate(t + best_lag_per_axis) (per-axis best lag, gyro)
# Row 3: cmd(t) vs quat-rate(t + best_lag_per_axis) (per-axis best lag, quat-d/dt)
#
# DIFFERENCE FROM 023: rows 2 and 3 use PER-AXIS best-lag, not a unified
# lag. At cadence7's 100ms step with a faster trained controller, pitch
# response peaks at ~100ms while roll peaks at ~200ms. Forcing a unified
# lag (e.g. 023's 234ms) lands past the pitch direct-response peak and
# shows the oscillatory rebound (negative r) instead of the direct
# response (positive r). The per-axis best-lag view is the honest one
# for judging convention sign.
#
# Independent of rate-gyro channel: row 3 rates come from finite-differencing
# attitude angles computed via q_EB -> body-axis world projection. In sim,
# gyr* and quat-diff should agree closely (CRRCSim integrates quat from
# omega_body) but via different log columns; a mismatch flags a serialization
# issue. In flight, they're truly independent: gyro from MEMS, quat from AHRS.

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def parse(path):
    with open(path) as f:
        header = f.readline().split()
        col = {name: i for i, name in enumerate(header)}
        rows = []
        for line in f:
            parts = line.split()
            if len(parts) < len(header):
                continue
            path_wind = parts[col['Pth/Wnd:Step:']].split(':')[0]
            rows.append((
                path_wind,
                int(parts[col['Time']]),
                float(parts[col['gyrP']]),
                float(parts[col['gyrQ']]),
                float(parts[col['outPt']]),
                float(parts[col['outRl']]),
                float(parts[col['outTh']]),
                float(parts[col['vel']]),
                float(parts[col['qw']]),
                float(parts[col['qx']]),
                float(parts[col['qy']]),
                float(parts[col['qz']]),
            ))
    return rows


def group_by_scenario(rows):
    by = {}
    for r in rows:
        by.setdefault(r[0], []).append(r)
    for k in by:
        by[k].sort(key=lambda r: r[1])
    return by


def attitudes_from_quat_rows(rows):
    w = np.array([r[8] for r in rows])
    x = np.array([r[9] for r in rows])
    y = np.array([r[10] for r in rows])
    z = np.array([r[11] for r in rows])
    nose_z = np.clip(2 * (x*z - w*y), -1.0, 1.0)
    rw_z = np.clip(2 * (y*z + w*x), -1.0, 1.0)
    pitch_att = np.arcsin(-nose_z)
    roll_att = np.arcsin(rw_z)
    return pitch_att, roll_att


def build_arrays(per_scn):
    """Per-scenario arrays: (t, cmd_pt, cmd_rl, cmd_th, vel, gyrP, gyrQ,
    p_quat, q_quat)."""
    out = {}
    for scn, rows in per_scn.items():
        if len(rows) < 3:
            continue
        t = np.array([r[1] for r in rows])
        t_s = t / 1000.0
        cmd_pt = np.array([r[4] for r in rows])
        cmd_rl = np.array([r[5] for r in rows])
        cmd_th = np.array([r[6] for r in rows])
        vel = np.array([r[7] for r in rows])
        gyrP = np.array([r[2] for r in rows])
        gyrQ = np.array([r[3] for r in rows])
        pitch_att, roll_att = attitudes_from_quat_rows(rows)
        # Central difference for interior points, forward/backward for ends
        q_quat = np.zeros_like(pitch_att)
        p_quat = np.zeros_like(roll_att)
        dt = np.diff(t_s)
        q_quat[1:-1] = (pitch_att[2:] - pitch_att[:-2]) / (t_s[2:] - t_s[:-2])
        p_quat[1:-1] = (roll_att[2:] - roll_att[:-2]) / (t_s[2:] - t_s[:-2])
        q_quat[0] = (pitch_att[1] - pitch_att[0]) / max(dt[0], 1e-6)
        q_quat[-1] = (pitch_att[-1] - pitch_att[-2]) / max(dt[-1], 1e-6)
        p_quat[0] = (roll_att[1] - roll_att[0]) / max(dt[0], 1e-6)
        p_quat[-1] = (roll_att[-1] - roll_att[-2]) / max(dt[-1], 1e-6)
        out[scn] = dict(t=t, cmd_pt=cmd_pt, cmd_rl=cmd_rl, cmd_th=cmd_th,
                        vel=vel, gyrP=gyrP, gyrQ=gyrQ,
                        p_quat=p_quat, q_quat=q_quat)
    return out


def lag_pairs(arrs, cmd_key, rate_key, lag_steps):
    xs, ys = [], []
    for d in arrs.values():
        a = d[cmd_key]; b = d[rate_key]; n = len(a)
        if lag_steps > 0:
            if n <= lag_steps: continue
            xs.extend(a[:-lag_steps]); ys.extend(b[lag_steps:])
        elif lag_steps < 0:
            k = -lag_steps
            if n <= k: continue
            xs.extend(a[k:]); ys.extend(b[:-k])
        else:
            xs.extend(a); ys.extend(b)
    return np.array(xs), np.array(ys)


def best_lag(arrs, cmd_key, rate_key, lag_range_steps):
    best = (0, 0.0)
    for L in lag_range_steps:
        xs, ys = lag_pairs(arrs, cmd_key, rate_key, L)
        if len(xs) < 10:
            continue
        c = np.corrcoef(xs, ys)[0, 1]
        if abs(c) > abs(best[1]):
            best = (L, c)
    return best


def main():
    data = sys.argv[1] if len(sys.argv) > 1 else '/tmp/gen400_p0_p2.dat'
    out_path = sys.argv[2] if len(sys.argv) > 2 else \
        '/home/gmcnutt/autoc/specs/024-sim-real-fidelity/cmd_response_scatter_cadence7.png'
    label = sys.argv[3] if len(sys.argv) > 3 else 'cadence7 (path 0 & 2, wind 00)'

    rows = parse(data)
    print(f"Rows: {len(rows)}")
    per_scn = group_by_scenario(rows)
    arrs = build_arrays(per_scn)

    dts = []
    for d in arrs.values():
        diffs = np.diff(d['t'])
        dts.extend(diffs[diffs > 0].tolist())
    step_ms = int(np.median(dts)) if dts else 100
    print(f"Median sim step: {step_ms} ms")

    # Lag search ranges. Pitch/roll need a modest window either side of zero;
    # throttle dynamics are slow (prop+drag settling), so expand forward.
    rot_lags = (np.arange(-300, 301, step_ms) / step_ms).astype(int)
    thr_lags = (np.arange(0, 1501, step_ms) / step_ms).astype(int)

    def ms(L): return L * step_ms

    def r_at(cmd_key, rate_key, L):
        x, y = lag_pairs(arrs, cmd_key, rate_key, L)
        return np.corrcoef(x, y)[0, 1] if len(x) > 2 else float('nan')

    # Per-axis best lag, for ALL SIX rate channels (gyro + quat-derived).
    best_gQ = best_lag(arrs, 'cmd_pt', 'gyrQ',   rot_lags)
    best_gP = best_lag(arrs, 'cmd_rl', 'gyrP',   rot_lags)
    best_v  = best_lag(arrs, 'cmd_th', 'vel',    thr_lags)
    best_qQ = best_lag(arrs, 'cmd_pt', 'q_quat', rot_lags)
    best_qP = best_lag(arrs, 'cmd_rl', 'p_quat', rot_lags)
    best_vQ = best_v  # quat row throttle panel reuses airspeed

    r_gQ_0 = r_at('cmd_pt', 'gyrQ', 0)
    r_gP_0 = r_at('cmd_rl', 'gyrP', 0)
    r_v_0  = r_at('cmd_th', 'vel',  0)

    print(f"\nBest per-axis lags (rows 2 & 3):")
    print(f"  gyro:  pitch->Q @ {ms(best_gQ[0]):+d} ms  r={best_gQ[1]:+.3f}")
    print(f"         roll ->P @ {ms(best_gP[0]):+d} ms  r={best_gP[1]:+.3f}")
    print(f"         thr  ->V @ {ms(best_v [0]):+d} ms  r={best_v [1]:+.3f}")
    print(f"  quat:  pitch->q @ {ms(best_qQ[0]):+d} ms  r={best_qQ[1]:+.3f}")
    print(f"         roll ->p @ {ms(best_qP[0]):+d} ms  r={best_qP[1]:+.3f}")
    print(f"\nRow 1 (instantaneous, lag=0):")
    print(f"  pitch->Q r={r_gQ_0:+.3f}   roll->P r={r_gP_0:+.3f}   thr->V r={r_v_0:+.3f}")

    cmap = plt.get_cmap('tab10')
    scn_color = {k: cmap(i) for i, k in enumerate(arrs.keys())}

    def panel(ax, cmd_key, rate_key, lag, title, xlabel, ylabel, rate_to_deg):
        for k, d in arrs.items():
            a = d[cmd_key]; b = d[rate_key]; n = len(a)
            if lag > 0:
                if n <= lag: continue
                x, y = a[:-lag], b[lag:]
            elif lag < 0:
                L = -lag
                if n <= L: continue
                x, y = a[L:], b[:-L]
            else:
                x, y = a, b
            if rate_to_deg:
                y = np.degrees(y)
            ax.scatter(x, y, s=10, alpha=0.6, color=scn_color[k],
                       label=f'{k} (n={len(x)})')
        ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)
        ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.set_title(title, fontsize=10)
        ax.set_xlim(-1.1, 1.1); ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='best')

    fig, axes = plt.subplots(3, 3, figsize=(18, 15))

    # Row 1: gyro, lag=0 (instantaneous)
    panel(axes[0,0], 'cmd_pt', 'gyrQ', 0,
          f'Pitch cmd vs gyrQ  (lag = 0, r={r_gQ_0:+.3f})',
          'NN pitch cmd', 'gyrQ, deg/s', True)
    panel(axes[0,1], 'cmd_rl', 'gyrP', 0,
          f'Roll cmd vs gyrP  (lag = 0, r={r_gP_0:+.3f})',
          'NN roll cmd', 'gyrP, deg/s', True)
    panel(axes[0,2], 'cmd_th', 'vel', 0,
          f'Throttle cmd vs airspeed  (lag = 0, r={r_v_0:+.3f})',
          'NN throttle cmd', 'airspeed, m/s', False)

    # Row 2: gyro, PER-AXIS best lag
    panel(axes[1,0], 'cmd_pt', 'gyrQ', best_gQ[0],
          f'Pitch vs gyrQ  (gyro, lag = {ms(best_gQ[0]):+d} ms, r={best_gQ[1]:+.3f})',
          'NN pitch cmd', 'gyrQ, deg/s', True)
    panel(axes[1,1], 'cmd_rl', 'gyrP', best_gP[0],
          f'Roll vs gyrP  (gyro, lag = {ms(best_gP[0]):+d} ms, r={best_gP[1]:+.3f})',
          'NN roll cmd', 'gyrP, deg/s', True)
    panel(axes[1,2], 'cmd_th', 'vel', best_v[0],
          f'Throttle vs airspeed  (lag = {ms(best_v[0]):+d} ms, r={best_v[1]:+.3f})',
          'NN throttle cmd', 'airspeed, m/s', False)

    # Row 3: quat-derived, PER-AXIS best lag
    panel(axes[2,0], 'cmd_pt', 'q_quat', best_qQ[0],
          f'Pitch vs q_quat  (quat-d/dt, lag = {ms(best_qQ[0]):+d} ms, r={best_qQ[1]:+.3f})',
          'NN pitch cmd', 'd(pitch_att)/dt, deg/s', True)
    panel(axes[2,1], 'cmd_rl', 'p_quat', best_qP[0],
          f'Roll vs p_quat  (quat-d/dt, lag = {ms(best_qP[0]):+d} ms, r={best_qP[1]:+.3f})',
          'NN roll cmd', 'd(roll_att)/dt, deg/s', True)
    panel(axes[2,2], 'cmd_th', 'vel', best_v[0],
          f'Throttle vs airspeed  (same as row 2, lag = {ms(best_v[0]):+d} ms)',
          'NN throttle cmd', 'airspeed, m/s', False)

    fig.suptitle(
        f'Command -> body response, sim (per-axis best lag)\n{label}\n'
        f'Row 1: lag=0 instantaneous.  Row 2: gyro at best lag per axis.  Row 3: quat-d/dt at best lag per axis.',
        fontsize=11)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"\nSaved: {out_path}")


if __name__ == '__main__':
    main()
