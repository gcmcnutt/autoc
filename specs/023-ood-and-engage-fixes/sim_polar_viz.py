#!/usr/bin/env python3
"""
Visualize sim training data — 023 direction cosine layout.
Updated from 022 version: replaces dPhi/dTheta polar plots with
direction cosine (target_x/y/z) time series and 3D scatter.

Usage:
  # Step 1: prefilter last gen to a temp file (fast grep)
  LAST_SCN=$(tail -1 data.dat | awk '{print $1}')
  { head -1 data.dat; grep "^${LAST_SCN} " data.dat; } > /tmp/autoc_lastgen.dat

  # Step 2: run the viz on the prefiltered file
  python3 specs/023-ood-and-engage-fixes/sim_polar_viz.py /tmp/autoc_lastgen.dat

X/Y/Z in data.dat are already VIRTUAL coordinates (getVirtualPosition).
Do NOT subtract X[0] — that subtracts away the entry offset.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

OUT_DIR = '/home/gmcnutt/autoc/specs/023-ood-and-engage-fixes'


def read_prefiltered(data_path):
    """Read a prefiltered data.dat (header + single-gen rows)."""
    with open(data_path) as f:
        header = f.readline().strip().split()
        col = {name: i for i, name in enumerate(header)}
        rows = []
        for line in f:
            parts = line.strip().split()
            if len(parts) < len(header) - 2:
                continue
            rows.append(parts)
    scn_val = rows[0][0] if rows else '?'
    return header, col, rows, scn_val


def plot_scenario(scn_rows, col, title, out_path, path_label=''):
    t = np.array([int(r[col['Time']]) / 1000.0 for r in scn_rows])
    t_rel = t - t[0]

    # Direction cosines (current tick = index 0, which is "now")
    tgX0 = np.array([float(r[col['tgX0']]) for r in scn_rows])
    tgY0 = np.array([float(r[col['tgY0']]) for r in scn_rows])
    tgZ0 = np.array([float(r[col['tgZ0']]) for r in scn_rows])

    # Forecast samples
    tgX_p5 = np.array([float(r[col['tgX+5']]) for r in scn_rows])
    tgY_p5 = np.array([float(r[col['tgY+5']]) for r in scn_rows])
    tgZ_p5 = np.array([float(r[col['tgZ+5']]) for r in scn_rows])

    # Distance (current)
    dist = np.array([float(r[col['ds0']]) for r in scn_rows])
    closing = np.array([float(r[col['dd/dt']]) for r in scn_rows])

    vel_input = np.array([float(r[col['vel']]) for r in scn_rows])

    outPt = np.array([float(r[col['outPt']]) for r in scn_rows])
    outRl = np.array([float(r[col['outRl']]) for r in scn_rows])
    outTh = np.array([float(r[col['outTh']]) for r in scn_rows])

    # Path in virtual frame
    pathX = np.array([float(r[col['pathX']]) for r in scn_rows])
    pathY = np.array([float(r[col['pathY']]) for r in scn_rows])

    # Aircraft position
    aX = np.array([float(r[col['X']]) for r in scn_rows])
    aY = np.array([float(r[col['Y']]) for r in scn_rows])

    qw = np.array([float(r[col['qw']]) for r in scn_rows])
    qx = np.array([float(r[col['qx']]) for r in scn_rows])
    qy = np.array([float(r[col['qy']]) for r in scn_rows])
    qz = np.array([float(r[col['qz']]) for r in scn_rows])
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    stpPt = np.array([float(r[col['stpPt']]) for r in scn_rows])
    mult = np.array([float(r[col['mult']]) for r in scn_rows])

    # Unit vector norm check
    norm_sq = tgX0**2 + tgY0**2 + tgZ0**2

    fig = plt.figure(figsize=(18, 13))
    gs = fig.add_gridspec(3, 3, hspace=0.38, wspace=0.35)

    # ── Panel 1 (top-left): Top-down track ──
    ax1 = fig.add_subplot(gs[0:2, 0])
    ax1.plot(aY, aX, 'b-', linewidth=1.5, label='Aircraft', alpha=0.7)
    ax1.plot(pathY, pathX, 'g-', linewidth=1.5, label='Rabbit/Path', alpha=0.7)
    ax1.plot(aY[0], aX[0], 'bo', markersize=10, label='Aircraft start')
    ax1.plot(aY[-1], aX[-1], 'b^', markersize=10, label='Aircraft end')
    ax1.plot(pathY[0], pathX[0], 'go', markersize=8, label='Path start (origin)')

    step = max(1, len(scn_rows) // 25)
    for i in range(0, len(scn_rows), step):
        hx, hy = np.cos(yaw[i]), np.sin(yaw[i])
        color = 'red' if outRl[i] > 0 else 'cyan'
        scale = 3.0 * abs(outRl[i])
        perp_x = hy
        perp_y = -hx
        ax1.arrow(aY[i], aX[i], scale * perp_y, scale * perp_x,
                  head_width=0.5, head_length=0.5, fc=color, ec=color, alpha=0.5)

    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_title(f'Top-down: aircraft (blue) vs path (green)\n{path_label} — VIRTUAL frame')
    ax1.legend(fontsize=7, loc='best')
    ax1.grid(alpha=0.3)
    ax1.set_aspect('equal')

    # ── Panel 2 (top-center): Direction cosine scatter (X vs Y, color=roll) ──
    ax2 = fig.add_subplot(gs[0, 1])
    sc = ax2.scatter(tgX0, tgY0, c=outRl, cmap='coolwarm', s=15, vmin=-1, vmax=1, alpha=0.7)
    ax2.set_xlabel('target_x (body forward)')
    ax2.set_ylabel('target_y (body right)')
    ax2.set_title('Direction cosines X vs Y\ncolor = roll cmd')
    ax2.set_xlim(-1.1, 1.1)
    ax2.set_ylim(-1.1, 1.1)
    ax2.set_aspect('equal')
    ax2.axhline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax2.axvline(0, color='gray', linewidth=0.5, alpha=0.3)
    circle = plt.Circle((0, 0), 1.0, fill=False, color='gray', linestyle='--', alpha=0.3)
    ax2.add_patch(circle)
    ax2.grid(alpha=0.3)
    plt.colorbar(sc, ax=ax2, shrink=0.7, label='roll cmd')

    # ── Panel 3 (top-right): Direction cosine scatter (X vs Z, color=pitch) ──
    # Y-axis inverted so "target ABOVE aircraft" appears at top of plot
    # (tgZ is body-down: +Z = below, −Z = above). This matches pilot-view intuition.
    ax3 = fig.add_subplot(gs[0, 2])
    sc2 = ax3.scatter(tgX0, tgZ0, c=outPt, cmap='coolwarm', s=15, vmin=-1, vmax=1, alpha=0.7)
    ax3.set_xlabel('target_x (body forward)')
    ax3.set_ylabel('target_z (body down; + = BELOW, − = ABOVE)')
    ax3.set_title('Direction cosines X vs Z\ncolor = pitch cmd — y-axis inverted (up = ABOVE)')
    ax3.set_xlim(-1.1, 1.1)
    ax3.set_ylim(1.1, -1.1)  # inverted: negative (above) at top
    ax3.set_aspect('equal')
    ax3.axhline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax3.axvline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax3.text(0.02, 0.97, 'target ABOVE', transform=ax3.transAxes,
             fontsize=7, color='gray', va='top', ha='left')
    ax3.text(0.02, 0.03, 'target BELOW', transform=ax3.transAxes,
             fontsize=7, color='gray', va='bottom', ha='left')
    circle2 = plt.Circle((0, 0), 1.0, fill=False, color='gray', linestyle='--', alpha=0.3)
    ax3.add_patch(circle2)
    ax3.grid(alpha=0.3)
    plt.colorbar(sc2, ax=ax3, shrink=0.7, label='pitch cmd')

    # ── Panel 4 (mid-right): NN outputs over time ──
    ax4 = fig.add_subplot(gs[1, 1:])
    ax4.plot(t_rel, outRl, 'r-', linewidth=1.5, label='roll cmd')
    ax4.plot(t_rel, outPt, 'b-', linewidth=1.5, label='pitch cmd')
    ax4.plot(t_rel, outTh, 'g-', linewidth=1.5, label='throttle cmd')
    ax4.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax4.axhline(1, color='gray', linewidth=0.3, linestyle=':', alpha=0.5)
    ax4.axhline(-1, color='gray', linewidth=0.3, linestyle=':', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('NN output')
    ax4.set_title('NN commanded outputs over time (SIM)')
    ax4.legend(fontsize=9, loc='upper right')
    ax4.set_ylim(-1.1, 1.1)
    ax4.grid(alpha=0.3)

    # ── Panel 5 (bottom-left, wide): NN inputs — direction cosines + dist over time ──
    ax5 = fig.add_subplot(gs[2, 0:2])
    ax5b = ax5.twinx()

    # Distance on left axis
    ax5.plot(t_rel, dist, 'k-', linewidth=2.0, label='dist (m)')
    ax5.fill_between(t_rel, 0, dist, alpha=0.08, color='black')

    # Direction cosines on right axis — current tick
    ax5b.plot(t_rel, tgX0, 'r-', linewidth=1.2, label='tgX (fwd)', alpha=0.85)
    ax5b.plot(t_rel, tgY0, 'g-', linewidth=1.2, label='tgY (right)', alpha=0.85)
    ax5b.plot(t_rel, tgZ0, 'b-', linewidth=1.2, label='tgZ (down)', alpha=0.85)

    # Show forecast +0.5s as dashed for comparison
    ax5b.plot(t_rel, tgX_p5, 'r--', linewidth=0.7, label='tgX +0.5s', alpha=0.45)
    ax5b.plot(t_rel, tgY_p5, 'g--', linewidth=0.7, label='tgY +0.5s', alpha=0.45)
    ax5b.plot(t_rel, tgZ_p5, 'b--', linewidth=0.7, label='tgZ +0.5s', alpha=0.45)

    ax5b.axhline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax5b.set_ylim(-1.3, 1.3)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Distance (m)', color='k')
    ax5b.set_ylabel('Direction cosine', color='b')
    ax5.set_title('NN inputs: dist + direction cosines (target_x/y/z) — solid=now, dashed=+0.5s')
    ax5.legend(loc='upper left', fontsize=7)
    ax5b.legend(loc='upper right', fontsize=7, ncol=2)
    ax5.grid(alpha=0.3)

    # ── Panel 6 (bottom-right): speed + fitness signals ──
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(t_rel, vel_input, 'k-', linewidth=1.5, label='speed')
    ax6b = ax6.twinx()
    ax6b.plot(t_rel, stpPt, 'g-', linewidth=1.0, label='stepPoints', alpha=0.7)
    ax6b.plot(t_rel, mult, 'm-', linewidth=1.0, label='multiplier', alpha=0.7)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Speed (m/s)', color='k')
    ax6b.set_ylabel('stpPt / mult', color='g')
    ax6.set_title('Speed + fitness signals')
    ax6.legend(loc='upper left', fontsize=7)
    ax6b.legend(loc='upper right', fontsize=7)
    ax6b.set_ylim(0, 5.5)
    ax6.grid(alpha=0.3)

    fig.suptitle(title, fontsize=13, y=0.995)
    plt.savefig(out_path, dpi=130, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {out_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: prefilter last gen, then pass the file:")
        print("  LAST_SCN=$(tail -1 data.dat | awk '{print $1}')")
        print("  { head -1 data.dat; grep \"^${LAST_SCN} \" data.dat; } > /tmp/autoc_lastgen.dat")
        print("  python3 sim_polar_viz.py /tmp/autoc_lastgen.dat")
        sys.exit(1)

    data_path = sys.argv[1]
    print(f"Reading prefiltered data from {data_path}...")
    header, col, rows, scn_val = read_prefiltered(data_path)
    print(f"  Scn={scn_val}, {len(rows)} rows")

    scenarios = defaultdict(list)
    for r in rows:
        scn_key = r[col['Pth/Wnd:Step:']].split(':')[0]
        scenarios[scn_key].append(r)

    print(f"  {len(scenarios)} scenarios")

    # Plot path 0/1/2, wind 00 (baseline)
    for path_idx in [0, 1, 2]:
        for w in ['00']:
            k = f"{path_idx:03d}/{w}"
            if k not in scenarios:
                continue
            scn_rows = scenarios[k]
            title = f"SIM Scn={scn_val} path={path_idx} wind={w}, {len(scn_rows)} steps — 023 direction cosines"
            out_path = f"{OUT_DIR}/nn_dirvec_sim_path{path_idx}_wind{w}.png"
            plot_scenario(scn_rows, col, title, out_path, f"path{path_idx} wind{w}")


if __name__ == '__main__':
    main()
