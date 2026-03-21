#!/usr/bin/env python3
"""
Per-span latency chain analysis with clock drift compensation.

For each span, measures the pipeline latency at each step:
  A. INAV sensor → xiao receives (MSP transport)
  B. xiao receives → NN eval complete
  C. NN eval → MSP RC send (send ticker)
  D. MSP RC send → INAV rcData reflects command
  E. INAV rcData → rcCommand (processing)
  F. rcCommand → servo/motor output (PID + mixer)

Clock drift: xiao_ms and inav_ms use different oscillators.
Per-span offset = inav_ms - xiao_ms at enable point.
"""
import csv
import re
import sys
import statistics

# Parse INAV
inav = {}
with open(sys.argv[1]) as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_ms = int(row['time (us)']) // 1000
        inav[t_ms] = {
            'rc': [int(row[f'rcData[{i}]']) for i in range(4)],
            'rcCmd': [int(row[f'rcCommand[{i}]']) for i in range(4)],
            'servo0': int(row['servo[0]']),
            'servo1': int(row['servo[1]']),
            'motor0': int(row['motor[0]']),
            'msp': int(row['mspOverrideFlags']),
        }

# Parse xiao
xiao_events = []
with open(sys.argv[2]) as f:
    for line in f:
        m = re.match(r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+(.*)', line)
        if m:
            xiao_events.append({
                'seq': int(m.group(1)),
                'xiao_ms': int(m.group(2)),
                'inav_ms': int(m.group(3)),
                'msg': m.group(4),
            })

# Extract enables, disables, NN evals, nav states
enables = [e for e in xiao_events if 'NN Control: Switch enabled' in e['msg']]
disables = [e for e in xiao_events if 'Nav Control: Switch disabled' in e['msg']]

nn_evals = []
nav_states = []
for ev in xiao_events:
    if ev['msg'].startswith('NN:'):
        m = re.match(r'NN:\s+idx=(\d+)\s+in=\[([^\]]+)\]\s+out=\[([^\]]+)\]\s+rc=\[([^\]]+)\]', ev['msg'])
        if m:
            nn_evals.append({**ev, 'rc': [int(x) for x in m.group(4).split(',')]})
    elif ev['msg'].startswith('Nav State:'):
        nav_states.append(ev)

# MSP send stats from xiao log
send_stats = [e for e in xiao_events if 'MSP TX stats:' in e['msg']]
latency_stats = [e for e in xiao_events if 'MSP latency:' in e['msg']]

print("="*80)
print("MSP OVERRIDE FRESHNESS GUARD ANALYSIS")
print("="*80)
print(f"\nINAV MSP override recovery: PERIOD_RXDATA_RECOVERY(200ms) + failsafe_recovery_delay(default 5) * 100ms = 700ms")
print(f"rxSignalTimeout: DELAY_5_HZ = 200ms (xiao sends at 50ms = 4x safety margin)")
print(f"Expected initial delay: ~700-800ms (observed consistently ~790ms)")

for span_idx in range(len(enables)):
    en = enables[span_idx]
    dis = disables[span_idx] if span_idx < len(disables) else None
    
    # Clock offset for this span
    clock_offset = en['inav_ms'] - en['xiao_ms']
    
    en_inav = en['inav_ms']
    dis_inav = dis['inav_ms'] if dis else en_inav + 30000
    
    print(f"\n{'='*80}")
    print(f"SPAN {span_idx+1}: clock_offset={clock_offset}ms")
    print(f"{'='*80}")
    
    # Get span NN evals
    span_nn = [nn for nn in nn_evals if en_inav <= nn['inav_ms'] <= dis_inav]
    span_nav = [ns for ns in nav_states if en_inav <= ns['inav_ms'] <= dis_inav]
    
    if not span_nn:
        print("  No NN evals in this span")
        continue
    
    # Skip first 10 ticks (inside freshness guard) and analyze steady-state
    steady_nn = span_nn[10:] if len(span_nn) > 15 else span_nn[5:]
    
    if not steady_nn:
        print("  Span too short for steady-state analysis")
        continue
    
    # For each steady-state NN eval, measure:
    # D. xiao RC → INAV rcData: find when INAV rcData matches xiao's command
    rc_latencies = []
    rcCmd_latencies = []
    servo_latencies = []
    
    for nn in steady_nn:
        target_rc = nn['rc']
        nn_t = nn['inav_ms']
        
        # D. Find first INAV rcData match
        for dt in range(0, 300):
            t = nn_t + dt
            if t in inav:
                row = inav[t]
                if abs(row['rc'][0] - target_rc[0]) < 30 and abs(row['rc'][1] - target_rc[1]) < 30:
                    rc_latencies.append(dt)
                    
                    # E. From that rcData match, find when rcCommand is non-trivially different
                    # (rcCommand is derived from rcData through expo/deadband/rates)
                    # Just record the rcCommand at the same point
                    # F. Find servo response - check if servo changes significantly within 200ms
                    servo_baseline = row['servo0']
                    for dt2 in range(1, 200):
                        t2 = t + dt2
                        if t2 in inav:
                            if abs(inav[t2]['servo0'] - servo_baseline) > 20:
                                servo_latencies.append(dt + dt2)
                                break
                    break
    
    print(f"\n  Steady-state analysis ({len(steady_nn)} ticks, after freshness guard):")
    
    # B. Nav State → NN eval latency (within xiao)
    nn_lats = []
    nav_by_inav = {ns['inav_ms']: ns for ns in span_nav}
    for nn in steady_nn:
        # Find nav state with same inav_ms
        if nn['inav_ms'] in nav_by_inav:
            ns = nav_by_inav[nn['inav_ms']]
            lat = nn['xiao_ms'] - ns['xiao_ms']
            if 0 <= lat <= 50:
                nn_lats.append(lat)
    
    if nn_lats:
        print(f"  B. Nav→NN eval:    mean={statistics.mean(nn_lats):.1f}ms  std={statistics.stdev(nn_lats) if len(nn_lats)>1 else 0:.1f}ms  range=[{min(nn_lats)},{max(nn_lats)}]ms")
    
    # C. Inter-NN interval (should be ~100ms)
    nn_intervals = [steady_nn[i+1]['xiao_ms'] - steady_nn[i]['xiao_ms'] for i in range(len(steady_nn)-1)]
    if nn_intervals:
        print(f"  C. NN interval:    mean={statistics.mean(nn_intervals):.1f}ms  std={statistics.stdev(nn_intervals) if len(nn_intervals)>1 else 0:.1f}ms  range=[{min(nn_intervals)},{max(nn_intervals)}]ms")
    
    if rc_latencies:
        print(f"  D. NN→rcData:      mean={statistics.mean(rc_latencies):.1f}ms  std={statistics.stdev(rc_latencies) if len(rc_latencies)>1 else 0:.1f}ms  range=[{min(rc_latencies)},{max(rc_latencies)}]ms  ({len(rc_latencies)}/{len(steady_nn)} matched)")
    else:
        print(f"  D. NN→rcData:      NO MATCHES (override may not be applying to all channels)")
    
    if servo_latencies:
        print(f"  F. NN→servo:       mean={statistics.mean(servo_latencies):.1f}ms  std={statistics.stdev(servo_latencies) if len(servo_latencies)>1 else 0:.1f}ms  range=[{min(servo_latencies)},{max(servo_latencies)}]ms")
    
    # Show sample time series for first 5 steady-state ticks
    print(f"\n  Sample pipeline trace (first 5 steady-state ticks):")
    print(f"  {'xiao_ms':>8} {'inav_ms':>8} {'xiao_rc0':>8} | {'dt_rcData':>9} {'inav_rc0':>8} {'rcCmd0':>6} | {'dt_servo':>8} {'servo0':>6} {'servo1':>6} {'motor':>6}")
    for nn in steady_nn[:5]:
        dt_rc = "?"
        inav_rc0 = "?"
        rcCmd0 = "?"
        dt_srv = "?"
        srv0 = srv1 = mtr = "?"
        
        for dt in range(0, 300):
            t = nn['inav_ms'] + dt
            if t in inav:
                row = inav[t]
                if abs(row['rc'][0] - nn['rc'][0]) < 30:
                    dt_rc = f"+{dt}ms"
                    inav_rc0 = row['rc'][0]
                    rcCmd0 = row['rcCmd'][0]
                    srv0 = row['servo0']
                    srv1 = row['servo1']
                    mtr = row['motor0']
                    break
        
        print(f"  {nn['xiao_ms']:>8} {nn['inav_ms']:>8} {nn['rc'][0]:>8} | {dt_rc:>9} {inav_rc0:>8} {rcCmd0:>6} | {dt_srv:>8} {srv0:>6} {srv1:>6} {mtr:>6}")

# Print MSP TX stats from xiao log
print(f"\n{'='*80}")
print("XIAO MSP TX STATS (per-span)")
print("="*80)
for stat in send_stats:
    print(f"  {stat['msg']}")
for stat in latency_stats:
    print(f"  {stat['msg']}")

