#!/usr/bin/env python3
"""Correlate xiao Nav State with INAV blackbox CSV at matching timestamps."""
import csv
import re
import sys

# Parse INAV blackbox CSV into dict keyed by time_ms (rounded to ms)
inav = {}
with open(sys.argv[1]) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_us = int(row['time (us)'])
        t_ms = t_us // 1000  # convert to ms for xiao correlation
        inav[t_ms] = {
            'pos': [float(row['navPos[0]'])/100.0, float(row['navPos[1]'])/100.0, float(row['navPos[2]'])/100.0],  # cm->m
            'vel': [float(row['navVel[0]'])/100.0, float(row['navVel[1]'])/100.0, float(row['navVel[2]'])/100.0],  # cm/s->m/s
            'quat': [float(row['quaternion[0]'])/10000.0, float(row['quaternion[1]'])/10000.0,
                     float(row['quaternion[2]'])/10000.0, float(row['quaternion[3]'])/10000.0],
            'att': [float(row['attitude[0]'])/10.0, float(row['attitude[1]'])/10.0, float(row['attitude[2]'])/10.0],  # decideg->deg
            'rc': [row['rcData[0]'], row['rcData[1]'], row['rcData[2]'], row['rcData[3]']],
            'msp_override': row['mspOverrideFlags'],
        }

# Parse xiao flight log Nav State lines
# Format: #NNNNN xiao_ms inav_ms i Nav State: pos_raw=[x,y,z] pos=[x,y,z] vel=[vx,vy,vz] quat=[w,x,y,z] ...
nav_re = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+Nav State:\s+'
    r'pos_raw=\[([-\d.]+),([-\d.]+),([-\d.]+)\]\s+'
    r'pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\]\s+'
    r'vel=\[([-\d.]+),([-\d.]+),([-\d.]+)\]\s+'
    r'quat=\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\]'
)

nn_re = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+NN:\s+idx=(\d+)\s+in=\[([^\]]+)\]\s+out=\[([^\]]+)\]\s+rc=\[([^\]]+)\]'
)

xiao_states = []
xiao_nn = []
with open(sys.argv[2]) as f:
    for line in f:
        m = nav_re.match(line)
        if m:
            xiao_states.append({
                'seq': int(m.group(1)),
                'xiao_ms': int(m.group(2)),
                'inav_ms': int(m.group(3)),
                'pos_raw': [float(m.group(4)), float(m.group(5)), float(m.group(6))],
                'vel': [float(m.group(10)), float(m.group(11)), float(m.group(12))],
                'quat': [float(m.group(13)), float(m.group(14)), float(m.group(15)), float(m.group(16))],
            })
        m = nn_re.match(line)
        if m:
            xiao_nn.append({
                'seq': int(m.group(1)),
                'xiao_ms': int(m.group(2)),
                'inav_ms': int(m.group(3)),
                'idx': int(m.group(4)),
                'inputs': [float(x) for x in m.group(5).split(',')],
                'outputs': [float(x) for x in m.group(6).split(',')],
                'rc': [int(x) for x in m.group(7).split(',')],
            })

# Correlate: for each xiao Nav State, find nearest INAV blackbox row
print("=== Nav State correlation (xiao pos_raw vs INAV navPos) ===")
print(f"{'xiao_seq':>8} {'inav_ms':>9} {'xiao_px':>8} {'inav_px':>8} {'dp_x':>7} {'xiao_py':>8} {'inav_py':>8} {'dp_y':>7} {'xiao_pz':>8} {'inav_pz':>8} {'dp_z':>7}")

matches = 0
misses = 0
pos_errs = []
vel_errs = []
quat_errs = []

for xs in xiao_states[::10]:  # sample every 10th for readability
    t = xs['inav_ms']
    # Find nearest INAV row (within ±50ms)
    best = None
    for dt in range(0, 50):
        if t + dt in inav:
            best = inav[t + dt]
            break
        if t - dt in inav:
            best = inav[t - dt]
            break
    if best is None:
        misses += 1
        continue
    matches += 1
    dp = [xs['pos_raw'][i] - best['pos'][i] for i in range(3)]
    dv = [xs['vel'][i] - best['vel'][i] for i in range(3)]
    dq = [xs['quat'][i] - best['quat'][i] for i in range(4)]
    pos_errs.append(dp)
    vel_errs.append(dv)
    quat_errs.append(dq)
    print(f"{xs['seq']:>8} {t:>9} {xs['pos_raw'][0]:>8.2f} {best['pos'][0]:>8.2f} {dp[0]:>7.2f} "
          f"{xs['pos_raw'][1]:>8.2f} {best['pos'][1]:>8.2f} {dp[1]:>7.2f} "
          f"{xs['pos_raw'][2]:>8.2f} {best['pos'][2]:>8.2f} {dp[2]:>7.2f}")

print(f"\nMatched: {matches}, Missed: {misses}")
if pos_errs:
    import statistics
    for axis, label in enumerate(['X(N)', 'Y(E)', 'Z(D)']):
        vals = [e[axis] for e in pos_errs]
        print(f"Pos {label}: mean={statistics.mean(vals):.3f}m  std={statistics.stdev(vals):.3f}m  max={max(abs(v) for v in vals):.3f}m")
    for axis, label in enumerate(['VN', 'VE', 'VD']):
        vals = [e[axis] for e in vel_errs]
        print(f"Vel {label}: mean={statistics.mean(vals):.3f}m/s  std={statistics.stdev(vals):.3f}m/s  max={max(abs(v) for v in vals):.3f}m/s")
    for axis, label in enumerate(['qw', 'qx', 'qy', 'qz']):
        vals = [e[axis] for e in quat_errs]
        print(f"Quat {label}: mean={statistics.mean(vals):.4f}  std={statistics.stdev(vals):.4f}  max={max(abs(v) for v in vals):.4f}")

# Show NN outputs during active control spans
print("\n=== NN output samples during active control ===")
print(f"{'seq':>6} {'inav_ms':>9} {'idx':>4} {'out0(pit)':>9} {'out1(rol)':>9} {'out2(thr)':>9} {'rc0':>5} {'rc1':>5} {'rc2':>5}")
for nn in xiao_nn[:30]:
    print(f"{nn['seq']:>6} {nn['inav_ms']:>9} {nn['idx']:>4} {nn['outputs'][0]:>9.3f} {nn['outputs'][1]:>9.3f} {nn['outputs'][2]:>9.3f} {nn['rc'][0]:>5} {nn['rc'][1]:>5} {nn['rc'][2]:>5}")
