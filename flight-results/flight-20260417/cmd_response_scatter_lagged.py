#!/usr/bin/env python3
# 3x3 cmd->response scatter from xiao flight log (autoc engage spans).
# Row 1: cmd(t) vs gyro-rate(t)                  (instantaneous, from NN line g=)
# Row 2: cmd(t) vs gyro-rate(t+L)                (per-axis best lag L from gyro)
# Row 3: cmd(t) vs quat-derived-rate(t+L)        (SAME per-axis lag L as row 2)
#
# Mirrors specs/023-ood-and-engage-fixes/cmd_response_scatter_sim_lagged.py.
# In flight, gyro comes from INAV's gyroADC (post-neg for pitch/yaw) and quat
# comes from INAV's AHRS fusion — truly independent channels, so row 3 vs row 2
# tests whether AHRS attitude tracking agrees with raw gyro integration.

import re
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# NN line has as=..., g=[p,q,r], out=[pt,rl,th], q=[w,x,y,z]
NN = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+NN:.*?'
    r'q=\[([^\]]+)\].*?'
    r'as=([-\d.]+).*?'
    r'g=\[([^\]]+)\].*?'
    r'out=\[([^\]]+)\]'
)
NAV = re.compile(r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+Nav State:.*autoc=(\w)')


def find_spans(path):
    spans = []
    in_span = False
    start = None
    last_t = None
    with open(path) as f:
        for line in f:
            m = NAV.search(line)
            if not m:
                continue
            t = int(m.group(2))
            autoc = m.group(4) == 'Y'
            if autoc and not in_span:
                start = t; in_span = True
            if autoc:
                last_t = t
            elif in_span:
                spans.append((start, last_t)); in_span = False
    if in_span:
        spans.append((start, last_t))
    return spans


def parse_nn(path, spans):
    rows = []
    with open(path) as f:
        for line in f:
            m = NN.search(line)
            if not m:
                continue
            t = int(m.group(2))
            span_id = None
            for i, (s, e) in enumerate(spans):
                if s <= t <= e: span_id = i; break
            if span_id is None: continue
            q = [float(x) for x in m.group(4).split(',')]
            airspeed = float(m.group(5))
            g = [float(x) for x in m.group(6).split(',')]
            out = [float(x) for x in m.group(7).split(',')]
            rows.append(dict(span=span_id, t=t, p=g[0], q_rate=g[1],
                             pitch_cmd=out[0], roll_cmd=out[1], thr_cmd=out[2],
                             vel=airspeed,
                             qw=q[0], qx=q[1], qy=q[2], qz=q[3]))
    return rows


def group_by_span(rows):
    by = {}
    for r in rows:
        by.setdefault(r['span'], []).append(r)
    for k in by:
        by[k].sort(key=lambda r: r['t'])
    return by


def build_arrays(per_span):
    out = {}
    for span, rows in per_span.items():
        if len(rows) < 3:
            continue
        t = np.array([r['t'] for r in rows])
        t_s = t / 1000.0
        cmd_pt = np.array([r['pitch_cmd'] for r in rows])
        cmd_rl = np.array([r['roll_cmd'] for r in rows])
        cmd_th = np.array([r['thr_cmd'] for r in rows])
        vel = np.array([r['vel'] for r in rows])
        gyrP = np.array([r['p'] for r in rows])
        gyrQ = np.array([r['q_rate'] for r in rows])
        w = np.array([r['qw'] for r in rows])
        x = np.array([r['qx'] for r in rows])
        y = np.array([r['qy'] for r in rows])
        z = np.array([r['qz'] for r in rows])
        nose_z = np.clip(2 * (x*z - w*y), -1.0, 1.0)
        rw_z = np.clip(2 * (y*z + w*x), -1.0, 1.0)
        pitch_att = np.arcsin(-nose_z)
        roll_att = np.arcsin(rw_z)
        q_quat = np.zeros_like(pitch_att)
        p_quat = np.zeros_like(roll_att)
        dt = np.diff(t_s)
        q_quat[1:-1] = (pitch_att[2:] - pitch_att[:-2]) / (t_s[2:] - t_s[:-2])
        p_quat[1:-1] = (roll_att[2:] - roll_att[:-2]) / (t_s[2:] - t_s[:-2])
        q_quat[0] = (pitch_att[1] - pitch_att[0]) / max(dt[0], 1e-6)
        q_quat[-1] = (pitch_att[-1] - pitch_att[-2]) / max(dt[-1], 1e-6)
        p_quat[0] = (roll_att[1] - roll_att[0]) / max(dt[0], 1e-6)
        p_quat[-1] = (roll_att[-1] - roll_att[-2]) / max(dt[-1], 1e-6)
        out[span] = dict(t=t, cmd_pt=cmd_pt, cmd_rl=cmd_rl, cmd_th=cmd_th,
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
        if len(xs) < 10: continue
        c = np.corrcoef(xs, ys)[0, 1]
        if abs(c) > abs(best[1]):
            best = (L, c)
    return best


def main():
    log = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/gmcnutt/autoc/flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt'
    out_path = sys.argv[2] if len(sys.argv) > 2 else \
        '/home/gmcnutt/autoc/flight-results/flight-20260417/cmd_response_scatter_lagged.png'

    spans = find_spans(log)
    print(f"Engage spans: {len(spans)}")
    for i, (s, e) in enumerate(spans):
        print(f"  span {i+1}: xiao_ms {s} .. {e}  ({(e-s)/1000:.1f}s)")
    rows = parse_nn(log, spans)
    print(f"NN samples in spans: {len(rows)}")
    per_span = group_by_span(rows)
    arrs = build_arrays(per_span)

    dts = []
    for d in arrs.values():
        diffs = np.diff(d['t'])
        dts.extend(diffs[diffs > 0].tolist())
    step_ms = int(np.median(dts)) if dts else 100
    print(f"Median NN sample step: {step_ms} ms")

    fast = (np.arange(-300, 301, step_ms) / step_ms).astype(int)
    slow = (np.arange(0, 1501, step_ms) / step_ms).astype(int)

    # UNIFIED LAG across sim and flight: 234 ms target, quantized to nearest
    # whole step. Applied to all axes on rows 2 and 3 for apples-to-apples.
    # Quantizes to exactly 2 steps in both sim (117 ms) and flight (~101 ms).
    UNIFIED_LAG_MS = 234
    lag_steps = int(round(UNIFIED_LAG_MS / step_ms))
    lag_pitch = lag_roll = lag_thr = lag_steps

    def ms(L): return L * step_ms

    def r_at(cmd_key, rate_key, L):
        x, y = lag_pairs(arrs, cmd_key, rate_key, L)
        return np.corrcoef(x, y)[0, 1] if len(x) > 2 else float('nan')

    best_gQ = best_lag(arrs, 'cmd_pt', 'gyrQ', fast)
    best_gP = best_lag(arrs, 'cmd_rl', 'gyrP', fast)
    best_v  = best_lag(arrs, 'cmd_th', 'vel',  slow)

    r_gQ_0 = r_at('cmd_pt', 'gyrQ', 0)
    r_gP_0 = r_at('cmd_rl', 'gyrP', 0)
    r_v_0  = r_at('cmd_th', 'vel',  0)
    r_gQ_L = r_at('cmd_pt', 'gyrQ', lag_steps)
    r_gP_L = r_at('cmd_rl', 'gyrP', lag_steps)
    r_v_L  = r_at('cmd_th', 'vel',  lag_steps)
    r_qQ_L = r_at('cmd_pt', 'q_quat', lag_steps)
    r_qP_L = r_at('cmd_rl', 'p_quat', lag_steps)

    print(f"\nUnified lag: {UNIFIED_LAG_MS} ms target -> {lag_steps} step = {ms(lag_steps)} ms actual")
    print(f"(best-lag, for reference: pitch {ms(best_gQ[0]):+d} ms r={best_gQ[1]:+.3f},  "
          f"roll {ms(best_gP[0]):+d} ms r={best_gP[1]:+.3f},  "
          f"thr {ms(best_v[0]):+d} ms r={best_v[1]:+.3f})")
    print(f"\n{'':>18}{'row 1 lag=0':>15}{f'row 2 gyro @{ms(lag_steps)}':>20}{f'row 3 quat @{ms(lag_steps)}':>20}")
    print(f"{'pitch->Q':>18}{r_gQ_0:+15.3f}{r_gQ_L:+20.3f}{r_qQ_L:+20.3f}")
    print(f"{'roll->P':>18}{r_gP_0:+15.3f}{r_gP_L:+20.3f}{r_qP_L:+20.3f}")
    print(f"{'throttle->vel':>18}{r_v_0:+15.3f}{r_v_L:+20.3f}{r_v_L:+20.3f}")

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

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
            ax.scatter(x, y, s=14, alpha=0.6, color=colors[k % len(colors)],
                       label=f'span {k+1} (n={len(x)})')
        ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)
        ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.set_title(title, fontsize=10)
        ax.set_xlim(-1.1, 1.1); ax.grid(alpha=0.3)
        ax.legend(fontsize=9, loc='best')

    fig, axes = plt.subplots(3, 3, figsize=(18, 15))

    # Row 1: gyro, lag=0
    panel(axes[0,0], 'cmd_pt', 'gyrQ', 0,
          f'Pitch cmd vs gyrQ  (gyro, lag = 0, r={r_gQ_0:+.3f})',
          'NN pitch cmd', 'gyrQ, deg/s', True)
    panel(axes[0,1], 'cmd_rl', 'gyrP', 0,
          f'Roll cmd vs gyrP  (gyro, lag = 0, r={r_gP_0:+.3f})',
          'NN roll cmd', 'gyrP, deg/s', True)
    panel(axes[0,2], 'cmd_th', 'vel', 0,
          f'Throttle cmd vs airspeed  (lag = 0, r={r_v_0:+.3f})',
          'NN throttle cmd', 'airspeed, m/s', False)

    L_ms = ms(lag_steps)

    # Row 2: gyro, unified lag
    panel(axes[1,0], 'cmd_pt', 'gyrQ', lag_steps,
          f'Pitch vs gyrQ  (gyro, lag = {L_ms:+d} ms, r={r_gQ_L:+.3f})',
          'NN pitch cmd', 'gyrQ, deg/s', True)
    panel(axes[1,1], 'cmd_rl', 'gyrP', lag_steps,
          f'Roll vs gyrP  (gyro, lag = {L_ms:+d} ms, r={r_gP_L:+.3f})',
          'NN roll cmd', 'gyrP, deg/s', True)
    panel(axes[1,2], 'cmd_th', 'vel', lag_steps,
          f'Throttle vs airspeed  (lag = {L_ms:+d} ms, r={r_v_L:+.3f})',
          'NN throttle cmd', 'airspeed, m/s', False)

    # Row 3: quat-derived, SAME unified lag
    panel(axes[2,0], 'cmd_pt', 'q_quat', lag_steps,
          f'Pitch vs q_quat  (quat-d/dt, lag = {L_ms:+d} ms, r={r_qQ_L:+.3f})',
          'NN pitch cmd', 'd(pitch_att)/dt, deg/s', True)
    panel(axes[2,1], 'cmd_rl', 'p_quat', lag_steps,
          f'Roll vs p_quat  (quat-d/dt, lag = {L_ms:+d} ms, r={r_qP_L:+.3f})',
          'NN roll cmd', 'd(roll_att)/dt, deg/s', True)
    panel(axes[2,2], 'cmd_th', 'vel', lag_steps,
          f'Throttle vs airspeed  (same lag as row 2)',
          'NN throttle cmd', 'airspeed, m/s', False)

    fig.suptitle(
        f'Command -> body response, flight (autoc spans only)\n{log.split("/")[-1]}\n'
        f'Row 1: gyro lag=0.  Row 2: gyro best-lag.  Row 3: quat-derived rate at SAME lag.',
        fontsize=11)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"\nSaved: {out_path}")


if __name__ == '__main__':
    main()
