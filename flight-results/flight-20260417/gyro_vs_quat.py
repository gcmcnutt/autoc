#!/usr/bin/env python3
# gyro vs quat-derived rate scatter, flight.
# Mirrors specs/024-sim-real-fidelity/gyro_vs_quat_sim.py.
#
# Body-frame (p, q, r) computed from consecutive q_EB samples via
#   q_delta = q_prev.inverse() * q_curr
#   (p, q, r) = 2 * q_delta.vec / dt
# This is the exact kinematics relation (q̇ = 0.5·q⊗ω_body) — no Euler gymnastics.
# Scatter against logged gyro. Expected: y=x diagonal (slope=1).
# Sign flip = convention bug. Scale ≠ 1 in flight = AHRS filter contribution.

import re
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

NN = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+NN:.*?'
    r'q=\[([^\]]+)\].*?'
    r'g=\[([^\]]+)\]'
)
NAV = re.compile(r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+Nav State:.*autoc=(\w)')


def find_spans(path):
    spans = []; in_span = False; start = None; last_t = None
    with open(path) as f:
        for line in f:
            m = NAV.search(line)
            if not m: continue
            t = int(m.group(2))
            autoc = m.group(4) == 'Y'
            if autoc and not in_span:
                start = t; in_span = True
            if autoc: last_t = t
            elif in_span:
                spans.append((start, last_t)); in_span = False
    if in_span: spans.append((start, last_t))
    return spans


def parse_nn(path, spans):
    rows = []
    with open(path) as f:
        for line in f:
            m = NN.search(line)
            if not m: continue
            t = int(m.group(2))
            span_id = None
            for i, (s, e) in enumerate(spans):
                if s <= t <= e: span_id = i; break
            if span_id is None: continue
            q = [float(x) for x in m.group(4).split(',')]
            g = [float(x) for x in m.group(5).split(',')]
            rows.append((span_id, t, g[0], g[1], g[2], q[0], q[1], q[2], q[3]))
    return rows


def rate_from_quat_pairs(w, x, y, z, dt):
    """Body-frame (p, q, r) from consecutive q_EB samples via q_delta."""
    wp, xp, yp, zp = w[:-1], -x[:-1], -y[:-1], -z[:-1]  # q_prev.inverse()
    wc, xc, yc, zc = w[1:], x[1:], y[1:], z[1:]
    wd = wp*wc - xp*xc - yp*yc - zp*zc
    xd = wp*xc + xp*wc + yp*zc - zp*yc
    yd = wp*yc - xp*zc + yp*wc + zp*xc
    zd = wp*zc + xp*yc - yp*xc + zp*wc
    neg = wd < 0
    xd = np.where(neg, -xd, xd)
    yd = np.where(neg, -yd, yd)
    zd = np.where(neg, -zd, zd)
    p = 2.0 * xd / dt
    q = 2.0 * yd / dt
    r = 2.0 * zd / dt
    return p, q, r


def build(rows):
    by = {}
    for r in rows:
        by.setdefault(r[0], []).append(r)
    for k in by:
        by[k].sort(key=lambda r: r[1])
    out = {}
    for k, rs in by.items():
        if len(rs) < 3:
            continue
        t = np.array([r[1] for r in rs]) / 1000.0
        gyrP = np.array([r[2] for r in rs])
        gyrQ = np.array([r[3] for r in rs])
        gyrR = np.array([r[4] for r in rs])
        w = np.array([r[5] for r in rs])
        x = np.array([r[6] for r in rs])
        y = np.array([r[7] for r in rs])
        z = np.array([r[8] for r in rs])
        dt = t[1:] - t[:-1]

        p_quat, q_quat, r_quat = rate_from_quat_pairs(w, x, y, z, dt)

        gyrP_m = 0.5 * (gyrP[:-1] + gyrP[1:])
        gyrQ_m = 0.5 * (gyrQ[:-1] + gyrQ[1:])
        gyrR_m = 0.5 * (gyrR[:-1] + gyrR[1:])

        out[k] = dict(gyrP=gyrP_m, gyrQ=gyrQ_m, gyrR=gyrR_m,
                      p_quat=p_quat, q_quat=q_quat, r_quat=r_quat)
    return out


def main():
    log = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/gmcnutt/autoc/flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt'
    out_path = sys.argv[2] if len(sys.argv) > 2 else \
        '/home/gmcnutt/autoc/flight-results/flight-20260417/gyro_vs_quat.png'

    spans = find_spans(log)
    rows = parse_nn(log, spans)
    arrs = build(rows)
    print(f"spans={len(spans)}, samples={len(rows)}")

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    def panel(ax, gyro_key, quat_key, title, axis_label_deg):
        xs_all, ys_all = [], []
        for k, d in arrs.items():
            x = np.degrees(d[gyro_key])
            y = np.degrees(d[quat_key])
            ax.scatter(x, y, s=14, alpha=0.6, color=colors[k % len(colors)],
                       label=f'span {k+1} (n={len(x)})')
            xs_all.extend(x); ys_all.extend(y)
        xs_all = np.array(xs_all); ys_all = np.array(ys_all)
        lo = min(xs_all.min(), ys_all.min())
        hi = max(xs_all.max(), ys_all.max())
        ax.plot([lo, hi], [lo, hi], 'k--', lw=0.8, alpha=0.5, label='y=x')
        if len(xs_all) > 2:
            slope, intercept = np.polyfit(xs_all, ys_all, 1)
            r = np.corrcoef(xs_all, ys_all)[0, 1]
            xx = np.array([lo, hi])
            ax.plot(xx, slope*xx + intercept, 'r-', lw=1.2, alpha=0.7,
                    label=f'fit: slope={slope:+.3f}  r={r:+.3f}')
        ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)
        ax.set_xlabel(f'gyro {axis_label_deg}, deg/s')
        ax.set_ylabel(f'd/dt({axis_label_deg}_att from quat), deg/s')
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='best')
        ax.set_aspect('equal', adjustable='datalim')
        return slope, r

    sp_roll, r_roll = panel(axes[0], 'gyrP', 'p_quat', 'Roll plane: gyrP vs d(roll)/dt', 'roll')
    sp_pitch, r_pitch = panel(axes[1], 'gyrQ', 'q_quat', 'Pitch plane: gyrQ vs d(pitch)/dt', 'pitch')
    sp_yaw, r_yaw = panel(axes[2], 'gyrR', 'r_quat', 'Yaw plane: gyrR vs d(yaw)/dt', 'yaw')

    print(f"\n{'plane':>8} {'slope':>10} {'r':>10}")
    print(f"{'roll':>8} {sp_roll:>+10.3f} {r_roll:>+10.3f}")
    print(f"{'pitch':>8} {sp_pitch:>+10.3f} {r_pitch:>+10.3f}")
    print(f"{'yaw':>8} {sp_yaw:>+10.3f} {r_yaw:>+10.3f}")

    fig.suptitle(
        f'Gyro vs quat-derived rate (flight)\n{log.split("/")[-1]}\n'
        f'Expected: points on y=x (slope=1). Sign flip or scale = convention / AHRS filter issue.',
        fontsize=11)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"\nSaved: {out_path}")


if __name__ == '__main__':
    main()
