#!/usr/bin/env python3
# gyro vs quat-derived rate scatter, sim.
# X axis: logged gyro rate (gyrP / gyrQ / gyrR) rad/s
# Y axis: body-frame angular rate implied by consecutive quaternions:
#           q_delta = q_prev.inverse() * q_curr
#           (p, q, r) = 2 * q_delta.vec / dt
#         This is the exact kinematics relationship (q̇ = 0.5·q⊗ω_body)
#         and avoids Euler-angle coupling during banked flight.
# Expected: points on y=x diagonal, slope=1. Sign flip = convention bug
#           (e.g. scalar-first vs scalar-last, q_EB vs q_BE).
# Zero lag expected: both signals represent rate at the same instant.
#
# Quaternion convention used throughout: scalar-first (w,x,y,z), Hamilton,
# q_EB (earth-to-body) per docs/COORDINATE_CONVENTIONS.md.

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
                float(parts[col['gyrR']]),
                float(parts[col['qw']]),
                float(parts[col['qx']]),
                float(parts[col['qy']]),
                float(parts[col['qz']]),
            ))
    return rows


def rate_from_quat_pairs(w, x, y, z, dt):
    """Body-frame (p, q, r) rate from consecutive q_EB samples.
    Returns arrays of length N-1, valued at midpoints between samples.
    """
    # q_delta = q_prev.inverse() * q_curr
    # q_prev.inverse() = (w_p, -x_p, -y_p, -z_p)  (unit quat)
    wp, xp, yp, zp = w[:-1], -x[:-1], -y[:-1], -z[:-1]  # conjugate (inverse)
    wc, xc, yc, zc = w[1:], x[1:], y[1:], z[1:]
    # Hamilton product (wp, xp, yp, zp) * (wc, xc, yc, zc)
    wd = wp*wc - xp*xc - yp*yc - zp*zc
    xd = wp*xc + xp*wc + yp*zc - zp*yc
    yd = wp*yc - xp*zc + yp*wc + zp*xc
    zd = wp*zc + xp*yc - yp*xc + zp*wc
    # Shortest-path disambiguation: if delta has wd<0, negate (same rotation)
    neg = wd < 0
    xd = np.where(neg, -xd, xd)
    yd = np.where(neg, -yd, yd)
    zd = np.where(neg, -zd, zd)
    # Body rate approximation: for small rotations, qd ≈ (1, p*dt/2, q*dt/2, r*dt/2)
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

        # Pair gyro at midpoints: average of adjacent samples
        gyrP_m = 0.5 * (gyrP[:-1] + gyrP[1:])
        gyrQ_m = 0.5 * (gyrQ[:-1] + gyrQ[1:])
        gyrR_m = 0.5 * (gyrR[:-1] + gyrR[1:])

        out[k] = dict(gyrP=gyrP_m, gyrQ=gyrQ_m, gyrR=gyrR_m,
                      p_quat=p_quat, q_quat=q_quat, r_quat=r_quat)
    return out


def main():
    data = sys.argv[1] if len(sys.argv) > 1 else '/tmp/gen400_p0_p2.dat'
    out_path = sys.argv[2] if len(sys.argv) > 2 else \
        '/home/gmcnutt/autoc/specs/024-sim-real-fidelity/gyro_vs_quat_sim.png'
    label = sys.argv[3] if len(sys.argv) > 3 else 'gen 400, path 0 & 2, variation 0 (test4-data.dat)'

    rows = parse(data)
    arrs = build(rows)
    print(f"Scenarios: {len(arrs)}")

    cmap = plt.get_cmap('tab10')
    scn_color = {k: cmap(i) for i, k in enumerate(arrs.keys())}

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    def panel(ax, gyro_key, quat_key, title, axis_label_deg):
        xs_all, ys_all = [], []
        for k, d in arrs.items():
            x = np.degrees(d[gyro_key])
            y = np.degrees(d[quat_key])
            ax.scatter(x, y, s=8, alpha=0.5, color=scn_color[k], label=f'{k} (n={len(x)})')
            xs_all.extend(x); ys_all.extend(y)
        xs_all = np.array(xs_all); ys_all = np.array(ys_all)
        # y=x reference line
        lo = min(xs_all.min(), ys_all.min())
        hi = max(xs_all.max(), ys_all.max())
        ax.plot([lo, hi], [lo, hi], 'k--', lw=0.8, alpha=0.5, label='y=x')
        # Linear fit
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
        f'Gyro vs quat-derived rate (sim)\n{label}\n'
        f'Expected: points on y=x (slope=1). Sign flip or scale = convention / filter issue.',
        fontsize=11)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"\nSaved: {out_path}")


if __name__ == '__main__':
    main()
