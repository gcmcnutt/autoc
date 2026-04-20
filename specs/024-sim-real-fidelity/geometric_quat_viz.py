#!/usr/bin/env python3
"""Break the gyro↔quat circular-argument trap.

`gyrP/Q/R` in data.dat come from EOM01's `getOmegaBody()`; `qw/qx/qy/qz`
come from integrating THOSE SAME rates via `q̇ = 0.5·q⊗ω`. So gyro↔quat-delta
slope ≈ 1 is tautological — it tells us the integrator is self-consistent,
not that the conventions map to standard aerospace.

What IS an independent witness: position/velocity integration. The FDM
projects body forces to world via the quat to produce motion. If the quat
has a convention bug, the world-frame motion wouldn't match reality.

This script produces three independent geometric checks:

1. `nose_world = q_EB.inverse() · (1,0,0)_body` vs `vel_world / ||vel||` —
   in coordinated flight these must align. Different physics chain:
   nose comes from rotating body-X by quat; velocity comes from integrating
   body forces projected by quat.

2. `belly_world = q_EB.inverse() · (0,0,1)_body` vs `(0,0,+1)` (world-down) —
   must be near-parallel when upright. Shows which direction body-Z "is"
   in world. Separate sanity on Z-axis convention.

3. `nose_world[Z]` vs altitude rate — if nose-Z is negative (climbing), the
   aircraft should be gaining altitude (pos_Z decreasing in NED).

Plot these overlays and scatters. Visually, for aero-correct sim data:
- Trajectory in N-E plane should have nose-arrows aligned with motion.
- `belly_world[Z]` near +1 during level flight.
- Climbing segments have nose-Z<0 AND altitude rate positive.

If ANY of these diverges, there's a real convention problem (not circular).
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Import our lib
sys.path.insert(0, '/home/gmcnutt/autoc/flight-results/flight-20260417')
from sensor_self_check_lib import (
    read_sim_data_dat, read_blackbox_csv,
    rotate_body_to_world, quat_to_euler_zyx,
)


def compute_witnesses(d):
    """Compute three independent geometric quantities from canonical data."""
    # nose direction in world (body +X rotated)
    body_x = np.tile(np.array([1.0, 0.0, 0.0]), (len(d.quat), 1))
    nose_world = rotate_body_to_world(d.quat, body_x)
    # belly direction in world (body +Z rotated)
    body_z = np.tile(np.array([0.0, 0.0, 1.0]), (len(d.quat), 1))
    belly_world = rotate_body_to_world(d.quat, body_z)
    # right-wing direction in world (body +Y rotated)
    body_y = np.tile(np.array([0.0, 1.0, 0.0]), (len(d.quat), 1))
    right_world = rotate_body_to_world(d.quat, body_y)
    return nose_world, belly_world, right_world


def render(d, out_png):
    """Render a 3×2 diagnostic figure for one dataset."""
    nose, belly, right = compute_witnesses(d)

    # Velocity direction (world)
    vel = d.vel if d.vel is not None else np.zeros_like(d.pos)
    speed = np.linalg.norm(vel, axis=1)
    vel_dir = np.where(speed[:, None] > 1.0, vel / np.maximum(speed[:, None], 1e-9), 0.0)
    t_s = d.t_ms.astype(float) / 1000.0
    t_rel = t_s - t_s[0]

    fig = plt.figure(figsize=(16, 14))
    title = f"Geometric quat viz — {d.source}  {os.path.basename(d.meta.get('path', '?'))}"
    fig.suptitle(title, fontsize=12)

    # Panel 1: Top-down trajectory with nose arrows
    ax = plt.subplot(3, 2, 1)
    if d.pos is not None:
        ax.plot(d.pos[:, 1], d.pos[:, 0], 'b-', lw=0.6, alpha=0.7, label='trajectory')
        # Downsample to ~40 arrows
        step = max(1, len(d.pos) // 40)
        E = d.pos[::step, 1]
        N = d.pos[::step, 0]
        dN = nose[::step, 0]
        dE = nose[::step, 1]
        scale = 0.04 * (max(np.ptp(d.pos[:, 0]), np.ptp(d.pos[:, 1])) + 1e-3)
        for i in range(len(E)):
            ax.arrow(E[i], N[i], scale * dE[i], scale * dN[i],
                     head_width=scale * 0.4, head_length=scale * 0.4,
                     fc='red', ec='red', alpha=0.6, lw=0.5)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Top-down trajectory + nose arrows (red)\n'
                 'Arrows should point along motion for coordinated flight',
                 fontsize=10)
    ax.grid(alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    # Panel 2: Side view (N vs altitude = -D)
    ax = plt.subplot(3, 2, 2)
    if d.pos is not None:
        alt = -d.pos[:, 2]  # NED D > 0 = below; altitude = -D
        ax.plot(d.pos[:, 0], alt, 'b-', lw=0.6, alpha=0.7, label='trajectory')
        step = max(1, len(d.pos) // 40)
        N = d.pos[::step, 0]
        A = alt[::step]
        dN = nose[::step, 0]
        d_alt = -nose[::step, 2]  # nose_world.D > 0 = nose points down → altitude rate negative
        scale = 0.04 * (max(np.ptp(d.pos[:, 0]), np.ptp(alt)) + 1e-3)
        for i in range(len(N)):
            ax.arrow(N[i], A[i], scale * dN[i], scale * d_alt[i],
                     head_width=scale * 0.4, head_length=scale * 0.4,
                     fc='red', ec='red', alpha=0.6, lw=0.5)
    ax.set_xlabel('North (m)')
    ax.set_ylabel('Altitude (= -Z_NED, m)')
    ax.set_title('Side view + nose pitch arrows\n'
                 'Nose up arrow ⇒ altitude should be increasing',
                 fontsize=10)
    ax.grid(alpha=0.3)

    # Panel 3: belly-Z over time (should be near +1 upright)
    ax = plt.subplot(3, 2, 3)
    ax.plot(t_rel, belly[:, 2], 'k-', lw=0.8, label='belly_world.D')
    ax.axhline(1.0, color='green', lw=0.5, linestyle=':', label='level upright')
    ax.axhline(-1.0, color='red', lw=0.5, linestyle=':', label='inverted')
    ax.axhline(0.0, color='gray', lw=0.3)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('belly_world[Z] (NED D)')
    ax.set_title('Belly direction in world Z\n'
                 '+1 = upright (belly-down), -1 = inverted',
                 fontsize=10)
    ax.set_ylim(-1.2, 1.2)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='best')

    # Panel 4: nose-Z vs altitude rate
    ax = plt.subplot(3, 2, 4)
    if d.pos is not None:
        # Compute altitude rate (world) from pos
        dt = np.diff(t_s)
        alt_rate = -np.diff(d.pos[:, 2]) / np.maximum(dt, 1e-6)
        nose_z_mid = 0.5 * (nose[:-1, 2] + nose[1:, 2])
        # Expected: nose_z_mid negative = nose up = altitude_rate positive
        # So plot nose_z_mid (x) vs altitude_rate (y); expected NEGATIVE slope
        mask = np.abs(alt_rate) < 50  # filter outliers
        ax.scatter(nose_z_mid[mask], alt_rate[mask], s=4, alpha=0.5, c='purple')
        if mask.sum() > 3:
            slope, intercept = np.polyfit(nose_z_mid[mask], alt_rate[mask], 1)
            r = np.corrcoef(nose_z_mid[mask], alt_rate[mask])[0, 1]
            xs = np.array([nose_z_mid[mask].min(), nose_z_mid[mask].max()])
            ax.plot(xs, slope*xs + intercept, 'r-', lw=1,
                    label=f'slope={slope:+.2f}  r={r:+.3f}')
            ax.legend(fontsize=9)
    ax.axhline(0, color='gray', lw=0.3)
    ax.axvline(0, color='gray', lw=0.3)
    ax.set_xlabel('nose_world[Z] (>0 = nose points down)')
    ax.set_ylabel('altitude rate (m/s, + = climbing)')
    ax.set_title('Nose-pitch vs altitude rate\n'
                 'Aero-correct: NEGATIVE slope (nose-up ⇒ climbing)',
                 fontsize=10)
    ax.grid(alpha=0.3)

    # Panel 5: nose_world vs vel_dir (alignment)
    ax = plt.subplot(3, 2, 5)
    mask = speed > 5.0
    # Dot product of nose and vel_dir per sample
    align = np.sum(nose * vel_dir, axis=1)
    ax.plot(t_rel[mask], align[mask], 'k-', lw=0.5, alpha=0.7)
    ax.axhline(1.0, color='green', lw=0.5, linestyle=':', label='perfect alignment')
    ax.axhline(0.0, color='gray', lw=0.3)
    ax.axhline(-1.0, color='red', lw=0.5, linestyle=':', label='opposite')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('nose_world · vel_dir')
    ax.set_title('Nose ↔ velocity alignment\n'
                 'Aero-correct coordinated flight: near +1',
                 fontsize=10)
    ax.set_ylim(-1.2, 1.2)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='best')

    # Panel 6: right-wing-Y component (Earth-E) and belly-Y (should be 0)
    ax = plt.subplot(3, 2, 6)
    ax.plot(t_rel, right[:, 0], 'b-', lw=0.5, alpha=0.7, label='right.N')
    ax.plot(t_rel, right[:, 1], 'g-', lw=0.5, alpha=0.7, label='right.E')
    ax.plot(t_rel, right[:, 2], 'r-', lw=0.5, alpha=0.7, label='right.D')
    ax.axhline(0, color='gray', lw=0.3)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('right-wing in world')
    ax.set_title('Right-wing direction in world\n'
                 'Bank sign: right.D > 0 = right wing down (positive roll)',
                 fontsize=10)
    ax.set_ylim(-1.2, 1.2)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='best')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_png, dpi=110, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {out_png}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    out_png = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + '_geometric_quat.png'

    low = path.lower()
    if low.endswith('.dat'):
        d = read_sim_data_dat(path)
    elif low.endswith('.csv'):
        d = read_blackbox_csv(path)
    else:
        raise SystemExit(f"unsupported format: {path}")

    print(f"Read {len(d.t_ms)} samples from {path} ({d.source})")
    render(d, out_png)

    # Print summary statistics
    nose, belly, right = compute_witnesses(d)
    speed = np.linalg.norm(d.vel, axis=1) if d.vel is not None else np.zeros(len(d.quat))
    mask = speed > 5.0
    print(f"\nGeometric summary ({mask.sum()} samples with speed > 5 m/s):")
    if mask.any():
        vel_dir = d.vel[mask] / np.linalg.norm(d.vel[mask], axis=1, keepdims=True)
        align = np.sum(nose[mask] * vel_dir, axis=1)
        print(f"  nose·vel_dir:  mean={np.mean(align):+.3f}  median={np.median(align):+.3f}  (+1 = aligned)")
    print(f"  belly.D:       mean={np.mean(belly[:,2]):+.3f}  median={np.median(belly[:,2]):+.3f}  (+1 = upright)")
    print(f"  right.D:       mean={np.mean(right[:,2]):+.3f}  median={np.median(right[:,2]):+.3f}  (+ = right wing down)")
    if d.pos is not None:
        # Check sign of nose-Z vs altitude-rate
        t_s = d.t_ms.astype(float) / 1000.0
        dt = np.diff(t_s)
        alt_rate = -np.diff(d.pos[:, 2]) / np.maximum(dt, 1e-6)
        nose_z_mid = 0.5 * (nose[:-1, 2] + nose[1:, 2])
        m = np.abs(alt_rate) < 50
        if m.sum() > 3:
            slope, _ = np.polyfit(nose_z_mid[m], alt_rate[m], 1)
            r = np.corrcoef(nose_z_mid[m], alt_rate[m])[0, 1]
            print(f"  nose.Z ↔ alt_rate:  slope={slope:+.2f}  r={r:+.3f}  (aero-correct: slope NEGATIVE)")


if __name__ == '__main__':
    main()
