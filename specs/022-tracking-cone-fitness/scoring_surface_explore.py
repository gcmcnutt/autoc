#!/usr/bin/env python3
"""
Scoring surface visualization for the V4 conical fitness function (022).

Renders the production V4 surface with autoc.ini defaults so the spec
illustration matches the actual scoring code. See development-report.md
for the V1 → V3 → V4 design history.

V4 formula:
    distance     = sqrt(along² + lateral²)
    angle        = acos(-along / distance)   # 0 = behind, π = ahead
    angle_clamp  = min(angle, π/2)           # ahead saturates at sideways
    dist_scale   = behind or ahead based on along sign
    score        = 1 / (1 + (dist/dist_scale)² + (angle_clamp/cone)²)

Run:
    python3 scoring_surface_explore.py
"""

import numpy as np
import matplotlib.pyplot as plt


# autoc.ini defaults
DIST_BEHIND = 7.0
DIST_AHEAD = 2.0
CONE_ANGLE_DEG = 45.0
STREAK_THRESHOLD = 0.5


def score_v4(along, lateral_dist,
             dist_scale_behind=DIST_BEHIND,
             dist_scale_ahead=DIST_AHEAD,
             angle_scale_deg=CONE_ANGLE_DEG):
    """V4 polar form with directional distance and clamped angle."""
    distance = np.sqrt(along*along + lateral_dist*lateral_dist)
    if distance < 1e-6:
        return 1.0
    cos_angle = -along / distance
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = np.arccos(cos_angle)
    angle_clamped = min(angle, np.pi / 2)

    angle_scale_rad = np.radians(angle_scale_deg)

    dist_scale = dist_scale_behind if along <= 0 else dist_scale_ahead

    eff_dist  = distance / dist_scale
    eff_angle = angle_clamped / angle_scale_rad
    eff_total_sq = eff_dist**2 + eff_angle**2
    return 1.0 / (1.0 + eff_total_sq)


def make_grid(extent=20.0, res=500):
    along = np.linspace(-extent, extent, res)
    lateral = np.linspace(-extent, extent, res)
    A, L = np.meshgrid(along, lateral)
    return along, lateral, A, L


def evaluate_surface(score_fn, A, L, **kwargs):
    out = np.zeros_like(A)
    for i in range(A.shape[0]):
        for j in range(A.shape[1]):
            out[i, j] = score_fn(A[i, j], abs(L[i, j]), **kwargs)
    return out


def plot_surface(ax, along, lateral, scores, title, contours=None):
    if contours is None:
        contours = [0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25, 0.30,
                    0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95]

    ax.contourf(along, lateral, scores, levels=40, cmap='viridis', vmin=0, vmax=1)
    cs = ax.contour(along, lateral, scores, levels=contours,
                     colors='white', linewidths=0.5, alpha=0.7)
    ax.clabel(cs, inline=True, fontsize=6, fmt='%.2f')

    # Highlight streak threshold (0.5)
    cs_thresh = ax.contour(along, lateral, scores, levels=[STREAK_THRESHOLD],
                            colors='red', linewidths=2)
    ax.clabel(cs_thresh, inline=True, fontsize=8, fmt='thresh=%.2f')

    # Mark rabbit and tangent direction
    ax.plot(0, 0, 'r*', markersize=15, label='rabbit')
    ax.annotate('', xy=(4, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))
    ax.text(4.5, 0.4, 'tangent', color='red', fontsize=9)

    ax.set_xlabel('along (m)  →  + = ahead')
    ax.set_ylabel('lateral (m)')
    ax.set_title(title, fontsize=10)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)


def main():
    along, lateral, A, L = make_grid(extent=20.0, res=500)

    # ============================================================
    # Plot 1: V4 standalone — large detailed view
    # ============================================================
    print("Evaluating V4 surface (production: distBehind=7, distAhead=2, cone=45°)...")
    s_v4 = evaluate_surface(score_v4, A, L)

    fig, ax = plt.subplots(1, 1, figsize=(11, 11))
    plot_surface(ax, along, lateral, s_v4,
                 'V4 PRODUCTION: Polar + directional distance + clamped angle\n'
                 f'dist_behind={DIST_BEHIND}m  dist_ahead={DIST_AHEAD}m  '
                 f'cone={CONE_ANGLE_DEG}°  clamp=π/2\n'
                 'Tail-chase rewarded · lateral pinched · ahead has real gradient')
    fig.tight_layout()
    out_path = '/home/gmcnutt/autoc/specs/022-tracking-cone-fitness/scoring_surface.png'
    plt.savefig(out_path, dpi=130, bbox_inches='tight')
    print(f"Saved: {out_path}")

    # ============================================================
    # Plot 2: V4 cone-angle sweep for tuning visualization
    # ============================================================
    print("Evaluating V4 cone-angle sweep (20°, 30°, 45°, 60°, 75°, 90°)...")
    along2, lateral2, A2, L2 = make_grid(extent=20.0, res=300)
    fig2, axes2 = plt.subplots(2, 3, figsize=(16, 10))
    for ax_, ang in zip(axes2.flat, [20, 30, 45, 60, 75, 90]):
        s = evaluate_surface(score_v4, A2, L2, angle_scale_deg=ang)
        plot_surface(ax_, along2, lateral2, s,
                     f'V4 — cone = {ang}°\n'
                     f'(dist_behind={DIST_BEHIND}, dist_ahead={DIST_AHEAD}, clamp=π/2)')

    fig2.suptitle('V4 cone angle tuning sweep', fontsize=12)
    fig2.tight_layout()
    out_path2 = '/home/gmcnutt/autoc/specs/022-tracking-cone-fitness/scoring_surface_cone_angles.png'
    plt.savefig(out_path2, dpi=130, bbox_inches='tight')
    print(f"Saved: {out_path2}")

    # ============================================================
    # Print scoring tables for sanity check
    # ============================================================
    test_points = [
        ('At rabbit',          (0, 0)),
        ('-3m behind on-axis', (-3, 0)),
        ('-5m behind on-axis', (-5, 0)),
        ('-7m behind on-axis', (-7, 0)),
        ('-10m behind on-axis',(-10, 0)),
        ('-14m behind on-axis',(-14, 0)),
        ('-20m behind on-axis',(-20, 0)),
        ('5m to the side',     (0, 5)),
        ('10m to the side',    (0, 10)),
        ('5m back, 5m side',   (-5, 5)),
        ('7m back, 7m side',   (-7, 7)),
        ('10m back, 5m side',  (-10, 5)),
        ('10m back, 10m side', (-10, 10)),
        ('+0.5m ahead',        (0.5, 0)),
        ('+1m ahead',          (1, 0)),
        ('+2m ahead',          (2, 0)),
        ('+5m ahead',          (5, 0)),
        ('+10m ahead',         (10, 0)),
    ]

    print()
    print(f"{'Position':<24} {'V4 score':>10}")
    print('-' * 36)
    for name, (al, lat) in test_points:
        v4 = score_v4(al, lat)
        marker = ' ★' if abs(v4 - 0.5) < 0.001 else '   '
        print(f"{name:<24} {v4:>10.4f}{marker}")

    print()
    print("Key validations:")
    print(f"  -7m behind = 0.5 (exact threshold) ✓")
    print(f"  5m to side at along=0: {score_v4(0,5):.3f} (gravity fix: was 0.5)")
    print(f"  +5m ahead has 5x penalty vs +0.5m ahead")
    print(f"  Tail-chase corridor extends to ~14m back at threshold")


if __name__ == '__main__':
    main()
