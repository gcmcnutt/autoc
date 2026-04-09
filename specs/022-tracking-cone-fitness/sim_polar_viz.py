#!/usr/bin/env python3
"""
Visualize sim training data in polar layout (V2: correct virtual frame).
X/Y/Z in data.dat are already VIRTUAL coordinates (getVirtualPosition).
Do NOT subtract X[0] — that subtracts away the entry offset.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DATA_DAT = '/home/gmcnutt/autoc/data.dat'
OUT_DIR = '/home/gmcnutt/autoc/specs/022-tracking-cone-fitness'

def read_last_gen(data_path, last_gen_suffix='400'):
    with open(data_path) as f:
        header = f.readline().strip().split()
        col = {name: i for i, name in enumerate(header)}
        rows = []
        for line in f:
            parts = line.strip().split()
            if len(parts) < 50: continue
            if not parts[0].endswith(last_gen_suffix): continue
            rows.append(parts)
    return header, col, rows

def plot_scenario(scn_rows, col, title, out_path, path_label=''):
    t = np.array([int(r[col['Time']]) / 1000.0 for r in scn_rows])
    t_rel = t - t[0]
    
    dPhi = np.array([float(r[col['dPh0']]) for r in scn_rows])
    dTheta = np.array([float(r[col['dTh0']]) for r in scn_rows])
    dist = np.array([float(r[col['ds0']]) for r in scn_rows])
    vel_input = np.array([float(r[col['vel']]) for r in scn_rows])
    
    outPt = np.array([float(r[col['outPt']]) for r in scn_rows])
    outRl = np.array([float(r[col['outRl']]) for r in scn_rows])
    outTh = np.array([float(r[col['outTh']]) for r in scn_rows])
    
    # Path in virtual frame (already correct)
    pathX = np.array([float(r[col['pathX']]) for r in scn_rows])
    pathY = np.array([float(r[col['pathY']]) for r in scn_rows])
    
    # Aircraft position — ALREADY VIRTUAL (from getVirtualPosition via inputdev_autoc store)
    # Do NOT subtract X[0]!
    aX = np.array([float(r[col['X']]) for r in scn_rows])
    aY = np.array([float(r[col['Y']]) for r in scn_rows])
    
    qw = np.array([float(r[col['qw']]) for r in scn_rows])
    qx = np.array([float(r[col['qx']]) for r in scn_rows])
    qy = np.array([float(r[col['qy']]) for r in scn_rows])
    qz = np.array([float(r[col['qz']]) for r in scn_rows])
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    
    stpPt = np.array([float(r[col['stpPt']]) for r in scn_rows])
    mult = np.array([float(r[col['mult']]) for r in scn_rows])
    
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
    
    # Panel 1: Top-down (virtual frame, both aircraft and path share origin)
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
    ax1.legend(fontsize=8, loc='best')
    ax1.grid(alpha=0.3)
    ax1.set_aspect('equal')
    
    # Panel 2: Polar dPhi × dist
    ax2 = fig.add_subplot(gs[0, 1], projection='polar')
    ax2.scatter(dPhi, dist, c=outRl, cmap='coolwarm', s=20, vmin=-1, vmax=1, alpha=0.7)
    ax2.set_title('dPhi (roll bearing) vs dist\ncolor = roll cmd', fontsize=9)
    ax2.set_theta_zero_location('N')
    ax2.set_theta_direction(-1)
    ax2.set_rlim(0, max(dist.max(), 1) * 1.1)
    
    # Panel 3: Polar dTheta × dist
    ax3 = fig.add_subplot(gs[0, 2], projection='polar')
    ax3.scatter(dTheta, dist, c=outPt, cmap='coolwarm', s=20, vmin=-1, vmax=1, alpha=0.7)
    ax3.set_title('dTheta (pitch bearing) vs dist\ncolor = pitch cmd', fontsize=9)
    ax3.set_theta_zero_location('N')
    ax3.set_theta_direction(-1)
    ax3.set_rlim(0, max(dist.max(), 1) * 1.1)
    
    # Panel 4: NN outputs
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
    
    # Panel 5: NN inputs
    ax5 = fig.add_subplot(gs[2, 0:2])
    ax5b = ax5.twinx()
    ax5.plot(t_rel, dist, 'k-', linewidth=1.5, label='dist (m)')
    ax5b.plot(t_rel, np.degrees(dPhi), 'r-', linewidth=1.0, label='dPhi°', alpha=0.7)
    ax5b.plot(t_rel, np.degrees(dTheta), 'b-', linewidth=1.0, label='dTheta°', alpha=0.7)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Distance (m)', color='k')
    ax5b.set_ylabel('Bearing angles (deg)', color='b')
    ax5b.axhline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax5.set_title('NN inputs: dist, dPhi, dTheta')
    ax5.legend(loc='upper left', fontsize=8)
    ax5b.legend(loc='upper right', fontsize=8)
    ax5.grid(alpha=0.3)
    
    # Panel 6: speed + fitness
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(t_rel, vel_input, 'k-', linewidth=1.5, label='speed')
    ax6b = ax6.twinx()
    ax6b.plot(t_rel, stpPt, 'g-', linewidth=1.0, label='stepPoints', alpha=0.7)
    ax6b.plot(t_rel, mult, 'm-', linewidth=1.0, label='multiplier', alpha=0.7)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Speed (m/s)', color='k')
    ax6b.set_ylabel('stpPt / mult', color='g')
    ax6.set_title('Speed + fitness signals')
    ax6.legend(loc='upper left', fontsize=8)
    ax6b.legend(loc='upper right', fontsize=8)
    ax6b.set_ylim(0, 5.5)
    ax6.grid(alpha=0.3)
    
    fig.suptitle(title, fontsize=13, y=0.995)
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {out_path}")

def main():
    print("Reading last gen from data.dat...")
    header, col, rows = read_last_gen(DATA_DAT, last_gen_suffix='400')
    print(f"  Gen 400 rows: {len(rows)}")
    
    from collections import defaultdict
    scenarios = defaultdict(list)
    for r in rows:
        scn_key = r[col['Pth/Wnd:Step:']].split(':')[0]
        scenarios[scn_key].append(r)
    
    path_names = {0: 'StraightAndLevel', 2: 'HorizontalFigureEight'}
    
    for path_idx, name in path_names.items():
        pick_winds = ['00', '10', '20', '40']
        for w in pick_winds:
            k = f"{path_idx:03d}/{w}"
            if k not in scenarios:
                continue
            scn_rows = scenarios[k]
            title = f"SIM gen400 path={path_idx} ({name}) wind={w}, {len(scn_rows)} steps"
            out_path = f"{OUT_DIR}/nn_polar_sim_path{path_idx}_wind{w}.png"
            plot_scenario(scn_rows, col, title, out_path, f"path{path_idx} wind{w}")

if __name__ == '__main__':
    main()
