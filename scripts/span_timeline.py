#!/usr/bin/env python3
"""
Trace the activation sequence for each test span:
1. RC arm channel (ch9) goes high → visible in INAV rcData
2. mspOverrideFlags transitions
3. First xiao NN Control enable
4. First xiao NN eval / RC command sent
5. First INAV rcData change reflecting xiao commands
6. Servo/motor response
"""
import csv
import re
import sys

# Parse INAV blackbox
inav_timeline = []
with open(sys.argv[1]) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_ms = int(row['time (us)']) // 1000
        inav_timeline.append({
            't_ms': t_ms,
            'rc': [int(row[f'rcData[{i}]']) for i in range(4)],
            'msp': int(row['mspOverrideFlags']),
            'servo0': int(row['servo[0]']),
            'servo1': int(row['servo[1]']),
            'motor0': int(row['motor[0]']),
            'rcCmd': [int(row[f'rcCommand[{i}]']) for i in range(4)],
        })

# Build INAV lookup by ms
inav_by_ms = {}
for row in inav_timeline:
    inav_by_ms[row['t_ms']] = row

# Parse xiao log
xiao_events = []
with open(sys.argv[2]) as f:
    for line in f:
        m = re.match(r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+(.*)', line)
        if m:
            seq, xiao_ms, inav_ms, msg = int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4)
            xiao_events.append({'seq': seq, 'xiao_ms': xiao_ms, 'inav_ms': inav_ms, 'msg': msg})

# Find xiao enable/disable events
enables = []
disables = []
nn_evals = []
for ev in xiao_events:
    if 'NN Control: Switch enabled' in ev['msg']:
        enables.append(ev)
    elif 'Nav Control: Switch disabled' in ev['msg']:
        disables.append(ev)
    elif ev['msg'].startswith('NN:'):
        m = re.match(r'NN:\s+idx=(\d+)\s+in=\[([^\]]+)\]\s+out=\[([^\]]+)\]\s+rc=\[([^\]]+)\]', ev['msg'])
        if m:
            nn_evals.append({
                **ev,
                'rc': [int(x) for x in m.group(4).split(',')],
                'out': [float(x) for x in m.group(3).split(',')],
            })

# For each span, trace the activation sequence
for span_idx in range(len(enables)):
    en = enables[span_idx]
    dis = disables[span_idx] if span_idx < len(disables) else None
    
    print(f"\n{'='*80}")
    print(f"SPAN {span_idx+1}")
    print(f"{'='*80}")
    
    # Search window: 2s before enable to 2s after
    t_start = en['inav_ms'] - 2000
    t_end = en['inav_ms'] + 2000
    
    events = []
    
    # 1. Find RC arm channel activation (look for rcData going high on channel that triggers)
    # We look at mspOverrideFlags transitions near this span
    prev_msp = None
    prev_rc = [1500]*4
    first_rc_change = None
    first_servo_change = None
    servo_baseline = None
    motor_baseline = None
    
    for row in inav_timeline:
        if row['t_ms'] < t_start or row['t_ms'] > t_end:
            if row['t_ms'] > t_end:
                break
            continue
        
        # mspOverrideFlags transitions
        if prev_msp is not None and row['msp'] != prev_msp:
            events.append((row['t_ms'], f"INAV mspOverrideFlags: {prev_msp} → {row['msp']}"))
        prev_msp = row['msp']
        
        # rcData changes (first significant deviation from 1500 center)
        if first_rc_change is None:
            for ch in range(3):
                if abs(row['rc'][ch] - 1500) > 20 and row['t_ms'] > en['inav_ms']:
                    first_rc_change = row['t_ms']
                    events.append((row['t_ms'], f"INAV rcData first change: ch{ch}={row['rc'][ch]} (was ~1500)"))
                    break
        
        # Servo/motor baseline
        if servo_baseline is None and row['t_ms'] < en['inav_ms']:
            servo_baseline = (row['servo0'], row['servo1'])
            motor_baseline = row['motor0']
        
        # First servo change after enable
        if first_servo_change is None and servo_baseline and row['t_ms'] > en['inav_ms']:
            if abs(row['servo0'] - servo_baseline[0]) > 30 or abs(row['servo1'] - servo_baseline[1]) > 30:
                first_servo_change = row['t_ms']
                events.append((row['t_ms'], f"INAV servo first change: s0={row['servo0']} s1={row['servo1']} (was {servo_baseline[0]},{servo_baseline[1]})"))
            if abs(row['motor0'] - motor_baseline) > 50:
                events.append((row['t_ms'], f"INAV motor first change: {row['motor0']} (was {motor_baseline})"))
                motor_baseline = None  # only report once
    
    # 2. Xiao enable event
    events.append((en['inav_ms'], f"XIAO NN Control enabled (seq={en['seq']}, xiao_ms={en['xiao_ms']})"))
    
    # 3. First NN eval in this span
    span_nn = [nn for nn in nn_evals if en['inav_ms'] <= nn['inav_ms'] <= (dis['inav_ms'] if dis else en['inav_ms'] + 30000)]
    if span_nn:
        first_nn = span_nn[0]
        events.append((first_nn['inav_ms'], f"XIAO first NN eval: rc={first_nn['rc']} out=[{first_nn['out'][0]:.2f},{first_nn['out'][1]:.2f},{first_nn['out'][2]:.2f}] (seq={first_nn['seq']})"))
    
    # 4. Find when INAV rcData first matches a xiao RC command
    for nn in span_nn[:20]:
        target = nn['rc']
        for dt in range(0, 500):
            t = nn['inav_ms'] + dt
            if t in inav_by_ms:
                row = inav_by_ms[t]
                if abs(row['rc'][0] - target[0]) < 30 and abs(row['rc'][1] - target[1]) < 30:
                    events.append((t, f"INAV rcData matches xiao rc=[{target[0]},{target[1]},{target[2]}] (dt=+{dt}ms from xiao eval at {nn['inav_ms']})"))
                    break
        else:
            continue
        break  # only need first match
    
    # 5. Xiao disable event  
    if dis:
        events.append((dis['inav_ms'], f"XIAO NN Control disabled (seq={dis['seq']}, duration={(dis['xiao_ms']-en['xiao_ms'])/1000:.1f}s)"))
    
    # Sort and print
    events.sort(key=lambda x: x[0])
    
    print(f"\n{'t_ms':>10}  {'dt_from_enable':>14}  Event")
    print(f"{'-'*10}  {'-'*14}  {'-'*60}")
    for t, desc in events:
        dt = t - en['inav_ms']
        print(f"{t:>10}  {dt:>+14}ms  {desc}")
    
    # Summary
    if first_rc_change:
        print(f"\n  Enable → first rcData change: {first_rc_change - en['inav_ms']}ms")
    if first_servo_change:
        print(f"  Enable → first servo change:  {first_servo_change - en['inav_ms']}ms")
    print(f"  Span NN evals: {len(span_nn)}")

