#!/usr/bin/env python3
"""
Visualize NN perception and commands during flight autoc spans.
Reads xiao flight log NN: lines, shows per-span:
  - Top-down map: aircraft path, rabbit path, arrows for commanded roll
  - Polar plot: dPhi (bearing to rabbit) vs dist, colored by commanded roll
  - Time series: out_roll, out_pitch, out_throttle, dist, dPhi, dTheta
"""
import re
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

nn_pat = re.compile(
    r'#(\d+) (\d+) (\d+) i NN: idx=(\d+) '
    r'in=\[([^\]]+)\] out=\[([^\]]+)\] rc=\[([^\]]+)\] rabbit=\[([^\]]+)\]'
)
nav_pat = re.compile(
    r'#(\d+) (\d+) (\d+) i Nav State: '
    r'pos_raw=\[([^\]]+)\] pos=\[([^\]]+)\] vel=\[([^\]]+)\] '
    r'quat=\[([^\]]+)\] gyro_raw=\[([^\]]+)\] '
    r'armed=(\w) fs=(\w) servo=(\w) autoc=(\w) rabbit=(\w) path=(\d+)'
)

def parse(path):
    nn_rows = []  # {t_ms, idx, inputs[27], out[3], rc[3], rabbit[3]}
    nav_rows = []  # {t_ms, pos_virt[3], vel[3], quat[4], autoc, path}
    with open(path) as f:
        for line in f:
            m = nn_pat.search(line)
            if m:
                nn_rows.append({
                    't_ms': int(m.group(2)),
                    'idx': int(m.group(4)),
                    'inputs': [float(x) for x in m.group(5).split(',')],
                    'out': [float(x) for x in m.group(6).split(',')],
                    'rc': [int(x) for x in m.group(7).split(',')],
                    'rabbit': [float(x) for x in m.group(8).split(',')],
                })
                continue
            m = nav_pat.search(line)
            if m:
                nav_rows.append({
                    't_ms': int(m.group(2)),
                    'pos': [float(x) for x in m.group(5).split(',')],
                    'vel': [float(x) for x in m.group(6).split(',')],
                    'quat': [float(x) for x in m.group(7).split(',')],
                    'autoc': m.group(12) == 'Y',
                    'path': int(m.group(14)),
                })
    return nn_rows, nav_rows

def find_autoc_spans(nav_rows):
    spans = []
    in_span = False
    start_idx = None
    for i, r in enumerate(nav_rows):
        if r['autoc'] and not in_span:
            start_idx = i
            in_span = True
        elif not r['autoc'] and in_span:
            spans.append({
                'start_t': nav_rows[start_idx]['t_ms'],
                'end_t': nav_rows[i-1]['t_ms'],
                'path': nav_rows[start_idx]['path'],
            })
            in_span = False
    if in_span:
        spans.append({
            'start_t': nav_rows[start_idx]['t_ms'],
            'end_t': nav_rows[-1]['t_ms'],
            'path': nav_rows[start_idx]['path'],
        })
    return spans

def quat_to_yaw(q):
    qw, qx, qy, qz = q
    return np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

def plot_span(nn_rows, nav_rows, span, title, out_path):
    """Create a 4-panel visualization for one autoc span."""
    # Filter NN and nav rows to span
    nn_span = [r for r in nn_rows if span['start_t'] <= r['t_ms'] <= span['end_t']]
    nav_span = [r for r in nav_rows if span['start_t'] <= r['t_ms'] <= span['end_t']]
    
    if not nn_span or not nav_span:
        print(f"  skipping {title}: no data")
        return
    
    # Match NN rows to nav rows by time (nearest)
    nav_t = np.array([r['t_ms'] for r in nav_rows])
    
    # Extract aircraft position (virtual frame from Nav State)
    t_nav = np.array([r['t_ms'] for r in nav_span]) / 1000.0
    t_nav_rel = t_nav - t_nav[0]
    pos = np.array([r['pos'] for r in nav_span])  # NED virtual
    vel = np.array([r['vel'] for r in nav_span])
    quat = np.array([r['quat'] for r in nav_span])
    yaw = np.array([quat_to_yaw(q) for q in quat])
    
    # Extract NN data
    t_nn = np.array([r['t_ms'] for r in nn_span]) / 1000.0
    t_nn_rel = t_nn - t_nn[0]
    nn_out = np.array([r['out'] for r in nn_span])  # [pitch, roll, throttle]
    nn_rabbit = np.array([r['rabbit'] for r in nn_span])  # virtual NED
    
    # inputs: [dPhi-9,-3,-1,0,+1,+5, dTheta*6, dist*6, dDist/dt, qw,qx,qy,qz, vel, gyrP,gyrQ,gyrR]
    # dPhi[3]=now is index 3, dTheta[3]=now is index 9, dist[3]=now is index 15
    dPhi_now = np.array([r['inputs'][3] for r in nn_span])    # radians
    dTheta_now = np.array([r['inputs'][9] for r in nn_span])  # radians
    dist_now = np.array([r['inputs'][15] for r in nn_span])   # meters
    vel_input = np.array([r['inputs'][23] for r in nn_span])  # m/s (groundspeed)
    
    # Commanded outputs
    out_pitch = nn_out[:, 0]  # -1 to +1 (+ = nose up)
    out_roll  = nn_out[:, 1]  # -1 to +1 (+ = right wing down)
    out_thr   = nn_out[:, 2]  # -1 to +1 (+ = more power)
    
    # ==== FIGURE ====
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)
    
    # ---- Panel 1: Top-down map (NED XY plane) ----
    ax1 = fig.add_subplot(gs[0:2, 0])
    ax1.plot(pos[:,1], pos[:,0], 'b-', linewidth=1.5, label='Aircraft', alpha=0.7)
    ax1.plot(nn_rabbit[:,1], nn_rabbit[:,0], 'g-', linewidth=1.5, label='Rabbit', alpha=0.7)
    ax1.plot(pos[0,1], pos[0,0], 'bo', markersize=10, label='Start')
    ax1.plot(pos[-1,1], pos[-1,0], 'b^', markersize=10, label='End')
    
    # Draw commanded roll as small arrows perpendicular to velocity
    # Downsample to ~25 arrows across the span
    step = max(1, len(nav_span) // 25)
    for i in range(0, len(nav_span), step):
        if i >= len(yaw): continue
        N, E = pos[i,0], pos[i,1]
        # heading direction
        hx, hy = np.cos(yaw[i]), np.sin(yaw[i])  # N, E
        # Match NN sample at this time
        if len(nn_span) == 0: continue
        nn_idx = min(range(len(t_nn)), key=lambda k: abs(t_nn[k] - t_nav[i]))
        roll_cmd = out_roll[nn_idx]
        # Perpendicular to heading, colored by roll command
        color = 'red' if roll_cmd > 0 else 'cyan'
        scale = 3.0 * abs(roll_cmd)
        perp_x = hy  # N component of right-hand perp
        perp_y = -hx  # E component
        ax1.arrow(E, N, scale * perp_y, scale * perp_x,
                  head_width=0.5, head_length=0.5, fc=color, ec=color, alpha=0.5)
    
    ax1.set_xlabel('East (m)')
    ax1.set_ylabel('North (m)')
    ax1.set_title('Top-down: aircraft (blue) vs rabbit (green)\narrows = roll command (red=right, cyan=left)')
    ax1.legend(fontsize=8, loc='best')
    ax1.grid(alpha=0.3)
    ax1.set_aspect('equal')
    
    # ---- Panel 2: Polar — dPhi (yaw bearing) vs dist, colored by roll cmd ----
    ax2 = fig.add_subplot(gs[0, 1], projection='polar')
    # dPhi is bearing to rabbit in body YZ plane. 0 = directly below, π/2 = right.
    # Actually it's the roll bearing — angle from body -Z axis
    # Let's just plot it in body frame polar
    ax2.scatter(dPhi_now, dist_now, c=out_roll, cmap='coolwarm', s=20, vmin=-1, vmax=1, alpha=0.7)
    ax2.set_title('dPhi (roll bearing to rabbit) vs dist\ncolor = commanded roll', fontsize=9)
    ax2.set_theta_zero_location('N')  # 0 at top
    ax2.set_theta_direction(-1)
    ax2.set_rlim(0, max(dist_now.max(), 1) * 1.1)
    
    # ---- Panel 3: Polar — dTheta (pitch bearing) vs dist, colored by pitch cmd ----
    ax3 = fig.add_subplot(gs[0, 2], projection='polar')
    ax3.scatter(dTheta_now, dist_now, c=out_pitch, cmap='coolwarm', s=20, vmin=-1, vmax=1, alpha=0.7)
    ax3.set_title('dTheta (pitch bearing to rabbit) vs dist\ncolor = commanded pitch', fontsize=9)
    ax3.set_theta_zero_location('N')
    ax3.set_theta_direction(-1)
    ax3.set_rlim(0, max(dist_now.max(), 1) * 1.1)
    
    # ---- Panel 4: NN outputs over time ----
    ax4 = fig.add_subplot(gs[1, 1:])
    ax4.plot(t_nn_rel, out_roll, 'r-', linewidth=1.5, label='roll cmd')
    ax4.plot(t_nn_rel, out_pitch, 'b-', linewidth=1.5, label='pitch cmd')
    ax4.plot(t_nn_rel, out_thr, 'g-', linewidth=1.5, label='throttle cmd')
    ax4.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax4.axhline(1, color='gray', linewidth=0.3, linestyle=':', alpha=0.5)
    ax4.axhline(-1, color='gray', linewidth=0.3, linestyle=':', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('NN output')
    ax4.set_title('NN commanded outputs over time')
    ax4.legend(fontsize=9, loc='upper right')
    ax4.set_ylim(-1.1, 1.1)
    ax4.grid(alpha=0.3)
    
    # ---- Panel 5: NN inputs (distance, dPhi, dTheta) over time ----
    ax5 = fig.add_subplot(gs[2, 0:2])
    ax5b = ax5.twinx()
    ax5.plot(t_nn_rel, dist_now, 'k-', linewidth=1.5, label='dist (m)')
    ax5b.plot(t_nn_rel, np.degrees(dPhi_now), 'r-', linewidth=1.0, label='dPhi°', alpha=0.7)
    ax5b.plot(t_nn_rel, np.degrees(dTheta_now), 'b-', linewidth=1.0, label='dTheta°', alpha=0.7)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Distance to rabbit (m)', color='k')
    ax5b.set_ylabel('Bearing angles (deg)', color='b')
    ax5b.axhline(0, color='gray', linewidth=0.5, alpha=0.3)
    ax5.set_title('NN inputs: distance (black), dPhi (red), dTheta (blue)')
    ax5.legend(loc='upper left', fontsize=8)
    ax5b.legend(loc='upper right', fontsize=8)
    ax5.grid(alpha=0.3)
    
    # ---- Panel 6: Aircraft speed and attitude rate ----
    ax6 = fig.add_subplot(gs[2, 2])
    speed = np.sqrt((vel**2).sum(axis=1))
    ax6.plot(t_nav_rel, speed, 'k-', linewidth=1.5, label='3D speed')
    ax6.plot(t_nn_rel, vel_input, 'm--', linewidth=1.0, label='NN vel input', alpha=0.7)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Speed (m/s)')
    ax6.set_title('Groundspeed')
    ax6.legend(fontsize=8)
    ax6.grid(alpha=0.3)
    
    fig.suptitle(title, fontsize=13, y=0.995)
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {out_path}")

def main():
    flight_dir = '/home/gmcnutt/autoc/flight-results/flight-20260407'
    logs = [
        ('flight_log_2026-04-08T02-16-40.txt', 'Flight1'),
        ('flight_log_2026-04-08T02-22-51.txt', 'Flight2'),
    ]
    
    for log_name, flight_label in logs:
        log_path = f"{flight_dir}/{log_name}"
        print(f"\nParsing {log_name}...")
        nn_rows, nav_rows = parse(log_path)
        print(f"  NN rows: {len(nn_rows)}, Nav rows: {len(nav_rows)}")
        
        spans = find_autoc_spans(nav_rows)
        print(f"  Autoc spans: {len(spans)}")
        
        path_names = ['StraightAndLevel', 'SpiralClimb', 'HorizontalFigureEight',
                      'FortyFiveDegreeAngledLoop', 'HighPerchSplitS', 'SeededRandomB']
        
        for i, span in enumerate(spans, 1):
            dur = (span['end_t'] - span['start_t']) / 1000.0
            path_name = path_names[span['path']] if 0 <= span['path'] < 6 else f"path{span['path']}"
            title = f"{flight_label} Span {i}: path={span['path']} {path_name}, dur={dur:.1f}s"
            out_file = f"{flight_dir}/nn_polar_{flight_label}_span{i}.png"
            plot_span(nn_rows, nav_rows, span, title, out_file)

if __name__ == '__main__':
    main()
