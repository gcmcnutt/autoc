#!/usr/bin/env python3
"""Extract response curves from INAV blackbox CSV.

Usage:
    python3 scripts/flight_response.py --axis throttle --csv <path>
    python3 scripts/flight_response.py --axis roll --csv <path>
    python3 scripts/flight_response.py --axis pitch --csv <path>

Outputs ascii art response curves: command magnitude vs response metric,
binned by airspeed regime (slow <12, cruise 12-16, fast >16 m/s).

Uses MSPRCOVERRIDE spans (NN control) and pilot recovery segments.
Based on 018 analysis patterns (correlate_flight.py, span_timeline.py).
"""
import argparse
import csv
import math
import sys


def parse_blackbox(csv_path):
    """Parse blackbox CSV, return list of dicts with converted units."""
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t_us = int(row['time (us)'])
                rc_roll = int(row['rcCommand[0]'])       # -500..+500
                rc_pitch = int(row['rcCommand[1]'])      # -500..+500
                # motor[0] is the actual throttle output PWM (1000-2000).
                # rcCommand[2] is 0 in MANUAL mode, rcData[2] is the MSP
                # override value (often stuck at 1500). motor[0] is closest
                # to actual power demand after mixer.
                motor = int(row['motor[0]'])              # 1000-2000 PWM
                motor_norm = (motor - 1000) / 1000.0      # 0.0-1.0
                # servo[1] is the one logged elevon (bug: servo[0] is unused)
                # elevon mixing: servo[1] = -roll + pitch
                servo1 = int(row['servo[1]'])             # 1000-2000 PWM
                att_roll = float(row['attitude[0]']) / 10.0   # decideg -> deg
                att_pitch = float(row['attitude[1]']) / 10.0
                att_yaw = float(row['attitude[2]']) / 10.0
                vel_n = float(row['navVel[0]']) / 100.0       # cm/s -> m/s
                vel_e = float(row['navVel[1]']) / 100.0
                vel_d = float(row['navVel[2]']) / 100.0
                pos_d = float(row['navPos[2]']) / 100.0       # cm -> m
                flags = row.get('flightModeFlags (flags)', '')
                msp = 'MSPRCOVERRIDE' in flags
            except (ValueError, KeyError):
                continue

            speed = math.sqrt(vel_n**2 + vel_e**2 + vel_d**2)
            rows.append({
                't_us': t_us,
                'rc_roll': rc_roll, 'rc_pitch': rc_pitch,
                'motor': motor, 'motor_norm': motor_norm, 'servo1': servo1,
                'att_roll': att_roll, 'att_pitch': att_pitch, 'att_yaw': att_yaw,
                'vel_n': vel_n, 'vel_e': vel_e, 'vel_d': vel_d,
                'speed': speed, 'pos_d': pos_d,
                'msp': msp,
            })
    return rows


def compute_rates(rows, dt_us=50000):
    """Add attitude rates (deg/s) from finite differences, sampled at ~dt_us intervals."""
    out = []
    prev = None
    for r in rows:
        if prev is not None:
            actual_dt = (r['t_us'] - prev['t_us']) / 1e6
            if 0.01 < actual_dt < 0.2:  # 10ms to 200ms window
                r['roll_rate'] = (r['att_roll'] - prev['att_roll']) / actual_dt
                r['pitch_rate'] = (r['att_pitch'] - prev['att_pitch']) / actual_dt
                # Handle yaw wraparound
                dyaw = r['att_yaw'] - prev['att_yaw']
                if dyaw > 180: dyaw -= 360
                if dyaw < -180: dyaw += 360
                r['yaw_rate'] = dyaw / actual_dt
                r['climb_rate'] = -(r['pos_d'] - prev['pos_d']) / actual_dt  # NED: -D = up
                out.append(r)
        prev = r
    return out


def speed_bin(speed):
    if speed < 12: return 'slow'
    if speed < 16: return 'cruise'
    return 'fast'


def ascii_histogram(bins, label, unit, width=50):
    """Print ascii bar chart for a dict of {bin_label: [values]}."""
    print(f"\n  {label}:")
    for bname in ['slow', 'cruise', 'fast']:
        vals = bins.get(bname, [])
        if not vals:
            print(f"    {bname:>6} (<12/12-16/>16 m/s): no data")
            continue
        mean = sum(vals) / len(vals)
        vmin = min(vals)
        vmax = max(vals)
        p25 = sorted(vals)[len(vals)//4]
        p75 = sorted(vals)[3*len(vals)//4]
        n = len(vals)
        print(f"    {bname:>6}: mean={mean:7.2f} {unit}  "
              f"[{vmin:7.2f}, {p25:7.2f}, {p75:7.2f}, {vmax:7.2f}]  n={n}")


def throttle_analysis(rows):
    """Throttle response: motor[0] output vs speed, climb rate.

    motor[0] is the actual throttle PWM (1000-2000) after mixer.
    This is the closest we have to power demand without direct ESC telemetry.
    Note: servo smoothing filters add some lag between command and actuator.
    """
    print("=" * 70)
    print("FLIGHT THROTTLE RESPONSE (motor[0] = actual throttle output)")
    print("=" * 70)

    # Bin by motor output: low <0.3, mid 0.3-0.7, high >0.7
    speed_by_thr = {'low': [], 'mid': [], 'high': []}
    climb_by_thr = {'low': [], 'mid': [], 'high': []}
    speed_bins = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        mn = r['motor_norm']
        if mn < 0.3:
            thr_bin = 'low'
        elif mn < 0.7:
            thr_bin = 'mid'
        else:
            thr_bin = 'high'
        speed_by_thr[thr_bin].append(r['speed'])
        if 'climb_rate' in r:
            climb_by_thr[thr_bin].append(r['climb_rate'])
        sb = speed_bin(r['speed'])
        speed_bins[sb].append(r['speed'])

    print("\n  Speed by motor output (motor[0] normalized 0-1):")
    for thr_bin in ['low', 'mid', 'high']:
        vals = speed_by_thr[thr_bin]
        if vals:
            mean = sum(vals) / len(vals)
            print(f"    motor={thr_bin:>4}: speed mean={mean:.1f} m/s  "
                  f"[{min(vals):.1f}, {max(vals):.1f}]  n={len(vals)}")
        else:
            print(f"    motor={thr_bin:>4}: no data")

    print("\n  Climb rate by motor output (m/s, positive=up):")
    for thr_bin in ['low', 'mid', 'high']:
        vals = climb_by_thr[thr_bin]
        if vals:
            mean = sum(vals) / len(vals)
            print(f"    motor={thr_bin:>4}: climb mean={mean:.2f} m/s  "
                  f"[{min(vals):.2f}, {max(vals):.2f}]  n={len(vals)}")
        else:
            print(f"    motor={thr_bin:>4}: no data")

    print("\n  Speed distribution across flight:")
    for sb in ['slow', 'cruise', 'fast']:
        vals = speed_bins[sb]
        if vals:
            print(f"    {sb:>6}: n={len(vals)} ({100*len(vals)/len(rows):.0f}%)  "
                  f"mean={sum(vals)/len(vals):.1f} m/s")

    # Steady-state Vmax: high motor output, roughly level
    level_full_thr = [r for r in rows if r['motor_norm'] > 0.8
                      and abs(r['att_pitch']) < 15 and abs(r['att_roll']) < 30]
    if level_full_thr:
        speeds = [r['speed'] for r in level_full_thr]
        print(f"\n  Vmax estimate (motor>0.8, |pitch|<15°, |roll|<30°):")
        print(f"    mean={sum(speeds)/len(speeds):.1f} m/s  "
              f"max={max(speeds):.1f} m/s  n={len(speeds)}")

    # Idle descent: low motor, coasting
    idle_coast = [r for r in rows if r['motor_norm'] < 0.15
                  and 'climb_rate' in r and abs(r['att_roll']) < 45]
    if idle_coast:
        climbs = [r['climb_rate'] for r in idle_coast]
        print(f"\n  Idle descent (motor<0.15, |roll|<45°):")
        print(f"    mean={sum(climbs)/len(climbs):.2f} m/s  n={len(climbs)}")


def roll_analysis(rows):
    """Roll response: rate vs command, binned by airspeed."""
    print("=" * 70)
    print("FLIGHT ROLL RESPONSE")
    print("=" * 70)

    rate_bins = {'slow': [], 'cruise': [], 'fast': []}
    # Collect (|rc_roll/500|, |roll_rate|) pairs
    cmd_rate_pairs = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        if 'roll_rate' not in r:
            continue
        sb = speed_bin(r['speed'])
        rate_bins[sb].append(abs(r['roll_rate']))
        cmd_mag = abs(r['rc_roll']) / 500.0
        if cmd_mag > 0.05:  # ignore near-zero commands
            cmd_rate_pairs[sb].append((cmd_mag, abs(r['roll_rate'])))

    ascii_histogram(rate_bins, "|Roll rate| by airspeed", "deg/s")

    # Rate gain: deg/s per unit command
    print("\n  Roll rate gain (deg/s per unit rcCommand/500):")
    for sb in ['slow', 'cruise', 'fast']:
        pairs = cmd_rate_pairs[sb]
        if len(pairs) > 10:
            # Simple: mean(rate) / mean(cmd)
            mean_cmd = sum(p[0] for p in pairs) / len(pairs)
            mean_rate = sum(p[1] for p in pairs) / len(pairs)
            gain = mean_rate / mean_cmd if mean_cmd > 0.01 else 0
            print(f"    {sb:>6}: gain={gain:.2f} deg/s/unit  "
                  f"(mean_cmd={mean_cmd:.2f}, mean_rate={mean_rate:.1f} deg/s, n={len(pairs)})")


def pitch_analysis(rows):
    """Pitch response: rate vs command, binned by airspeed."""
    print("=" * 70)
    print("FLIGHT PITCH RESPONSE")
    print("=" * 70)

    rate_bins = {'slow': [], 'cruise': [], 'fast': []}
    cmd_rate_pairs = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        if 'pitch_rate' not in r:
            continue
        sb = speed_bin(r['speed'])
        rate_bins[sb].append(abs(r['pitch_rate']))
        cmd_mag = abs(r['rc_pitch']) / 500.0
        if cmd_mag > 0.05:
            cmd_rate_pairs[sb].append((cmd_mag, abs(r['pitch_rate'])))

    ascii_histogram(rate_bins, "|Pitch rate| by airspeed", "deg/s")

    print("\n  Pitch rate gain (deg/s per unit rcCommand/500):")
    for sb in ['slow', 'cruise', 'fast']:
        pairs = cmd_rate_pairs[sb]
        if len(pairs) > 10:
            mean_cmd = sum(p[0] for p in pairs) / len(pairs)
            mean_rate = sum(p[1] for p in pairs) / len(pairs)
            gain = mean_rate / mean_cmd if mean_cmd > 0.01 else 0
            print(f"    {sb:>6}: gain={gain:.2f} deg/s/unit  "
                  f"(mean_cmd={mean_cmd:.2f}, mean_rate={mean_rate:.1f} deg/s, n={len(pairs)})")


def main():
    parser = argparse.ArgumentParser(description='Extract flight response curves from blackbox CSV')
    parser.add_argument('--axis', required=True, choices=['throttle', 'roll', 'pitch', 'all'])
    parser.add_argument('--csv', required=True, help='Path to blackbox CSV')
    parser.add_argument('--msp-only', action='store_true', help='Only use MSPRCOVERRIDE spans')
    args = parser.parse_args()

    print(f"Loading {args.csv}...")
    rows = parse_blackbox(args.csv)
    print(f"  {len(rows)} rows parsed")

    if args.msp_only:
        rows = [r for r in rows if r['msp']]
        print(f"  {len(rows)} rows with MSPRCOVERRIDE")

    rows = compute_rates(rows)
    print(f"  {len(rows)} rows with rates computed")

    msp_count = sum(1 for r in rows if r['msp'])
    print(f"  {msp_count} MSPRCOVERRIDE rows, {len(rows)-msp_count} pilot/idle rows")

    if args.axis in ('throttle', 'all'):
        throttle_analysis(rows)
    if args.axis in ('roll', 'all'):
        roll_analysis(rows)
    if args.axis in ('pitch', 'all'):
        pitch_analysis(rows)


if __name__ == '__main__':
    main()
