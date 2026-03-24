#!/usr/bin/env python3
"""Extract response curves from CRRCSim data.dat training output.

Usage:
    python3 scripts/sim_response.py --axis throttle --data 20260323-data.dat --gen-range 1-5
    python3 scripts/sim_response.py --axis roll --data 20260323-data.dat --gen-range 1-5
    python3 scripts/sim_response.py --axis pitch --data 20260323-data.dat --gen-range 1-5

Outputs ascii art response curves: command magnitude vs response metric,
binned by airspeed regime (slow <12, cruise 12-16, fast >16 m/s).

data.dat fields (1-indexed):
  F1=Scn F2=Bake F3=Pth/Wnd:Step: F4=Time F5=Idx
  F6-F11=dPhi  F12-F17=dTheta  F18-F23=dist  F24=dddt
  F25=qw F26=qx F27=qy F28=qz F29=vel F30=alpha F31=beta
  F32=outPt F33=outRl F34=outTh F35=cmdP F36=cmdR F37=cmdT
  F38=pathX F39=pathY F40=pathZ F41=X F42=Y F43=Z
  F44=vxBody F45=vyBody F46=vzBody F47=dhome F48=dist
  F49=attDlt F50=rabVl F51=intSc F52=rampSc
"""
import argparse
import math
import sys


def quat_to_euler(qw, qx, qy, qz):
    """Quaternion (earth->body) to roll, pitch, yaw in degrees."""
    # Roll (x-axis)
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))
    # Pitch (y-axis)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # Yaw (z-axis)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def parse_data_dat(path, gen_min=1, gen_max=999):
    """Parse data.dat, return list of step dicts for requested generation range.

    Data.dat has a header line per generation starting with 'Scn', followed
    by data rows. We use the generation counter from the .stc file convention:
    each header increments the gen count.
    """
    rows = []
    gen = 0
    with open(path) as f:
        for line in f:
            if line.startswith('Scn'):
                gen += 1
                continue
            if gen < gen_min:
                continue
            if gen > gen_max:
                break

            parts = line.split()
            if len(parts) < 52:
                continue
            try:
                scn = parts[0]
                step_field = parts[2]  # e.g. "000/00:0001:"
                time_ms = int(parts[3])
                qw = float(parts[24])
                qx = float(parts[25])
                qy = float(parts[26])
                qz = float(parts[27])
                vel = float(parts[28])
                alpha = float(parts[29])
                beta = float(parts[30])
                outPt = float(parts[31])
                outRl = float(parts[32])
                outTh = float(parts[33])
                x = float(parts[40])
                y = float(parts[41])
                z = float(parts[42])

                roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)

                # Throttle in sim: outTh is -1..+1, maps to 0..1 via (outTh/2+0.5)
                thr_norm = outTh / 2.0 + 0.5

                # Track scenario+path for grouping consecutive steps
                # step_field format: "PPP/WW:SSSS:" where P=path, W=wind, S=step
                path_wind = step_field.split(':')[0]  # "PPP/WW"

                rows.append({
                    'gen': gen,
                    'scn': scn,
                    'path_wind': path_wind,
                    'time_ms': time_ms,
                    'vel': vel,
                    'alpha': alpha,
                    'beta': beta,
                    'outPt': outPt, 'outRl': outRl, 'outTh': outTh,
                    'thr_norm': thr_norm,
                    'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
                    'roll': roll, 'pitch': pitch, 'yaw': yaw,
                    'x': x, 'y': y, 'z': z,
                })
            except (ValueError, IndexError):
                continue

    return rows


def compute_rates(rows):
    """Add attitude rates from consecutive steps in same scenario/path."""
    out = []
    prev = None
    for r in rows:
        if prev is not None and r['scn'] == prev['scn'] and r['path_wind'] == prev['path_wind']:
            dt = (r['time_ms'] - prev['time_ms']) / 1000.0
            if 0.05 < dt < 0.5:  # 50-500ms window
                r['roll_rate'] = (r['roll'] - prev['roll']) / dt
                r['pitch_rate'] = (r['pitch'] - prev['pitch']) / dt
                dyaw = r['yaw'] - prev['yaw']
                if dyaw > 180: dyaw -= 360
                if dyaw < -180: dyaw += 360
                r['yaw_rate'] = dyaw / dt
                r['climb_rate'] = -(r['z'] - prev['z']) / dt  # NED: -dZ = up
                out.append(r)
        prev = r
    return out


def speed_bin(speed):
    if speed < 12: return 'slow'
    if speed < 16: return 'cruise'
    return 'fast'


def ascii_histogram(bins, label, unit):
    print(f"\n  {label}:")
    for bname in ['slow', 'cruise', 'fast']:
        vals = bins.get(bname, [])
        if not vals:
            print(f"    {bname:>6} (<12/12-16/>16 m/s): no data")
            continue
        mean = sum(vals) / len(vals)
        vmin = min(vals)
        vmax = max(vals)
        s = sorted(vals)
        p25 = s[len(s)//4]
        p75 = s[3*len(s)//4]
        print(f"    {bname:>6}: mean={mean:7.2f} {unit}  "
              f"[{vmin:7.2f}, {p25:7.2f}, {p75:7.2f}, {vmax:7.2f}]  n={len(vals)}")


def throttle_analysis(rows):
    """Throttle response: outTh command vs speed, climb rate."""
    print("=" * 70)
    print("SIM THROTTLE RESPONSE (outTh mapped to 0-1)")
    print("=" * 70)

    speed_by_thr = {'low': [], 'mid': [], 'high': []}
    climb_by_thr = {'low': [], 'mid': [], 'high': []}
    speed_bins = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        tn = r['thr_norm']
        if tn < 0.3:
            thr_bin = 'low'
        elif tn < 0.7:
            thr_bin = 'mid'
        else:
            thr_bin = 'high'
        speed_by_thr[thr_bin].append(r['vel'])
        if 'climb_rate' in r:
            climb_by_thr[thr_bin].append(r['climb_rate'])
        sb = speed_bin(r['vel'])
        speed_bins[sb].append(r['vel'])

    print("\n  Speed by throttle output (outTh mapped 0-1):")
    for thr_bin in ['low', 'mid', 'high']:
        vals = speed_by_thr[thr_bin]
        if vals:
            mean = sum(vals) / len(vals)
            print(f"    thr={thr_bin:>4}: speed mean={mean:.1f} m/s  "
                  f"[{min(vals):.1f}, {max(vals):.1f}]  n={len(vals)}")
        else:
            print(f"    thr={thr_bin:>4}: no data")

    print("\n  Climb rate by throttle output (m/s, positive=up):")
    for thr_bin in ['low', 'mid', 'high']:
        vals = climb_by_thr[thr_bin]
        if vals:
            mean = sum(vals) / len(vals)
            print(f"    thr={thr_bin:>4}: climb mean={mean:.2f} m/s  "
                  f"[{min(vals):.2f}, {max(vals):.2f}]  n={len(vals)}")
        else:
            print(f"    thr={thr_bin:>4}: no data")

    print("\n  Speed distribution:")
    for sb in ['slow', 'cruise', 'fast']:
        vals = speed_bins[sb]
        if vals:
            print(f"    {sb:>6}: n={len(vals)} ({100*len(vals)/len(rows):.0f}%)  "
                  f"mean={sum(vals)/len(vals):.1f} m/s")

    # Vmax at full throttle, roughly level
    level_full = [r for r in rows if r['thr_norm'] > 0.8
                  and abs(r['pitch']) < 15 and abs(r['roll']) < 30]
    if level_full:
        speeds = [r['vel'] for r in level_full]
        print(f"\n  Vmax estimate (thr>0.8, |pitch|<15°, |roll|<30°):")
        print(f"    mean={sum(speeds)/len(speeds):.1f} m/s  "
              f"max={max(speeds):.1f} m/s  n={len(speeds)}")

    idle = [r for r in rows if r['thr_norm'] < 0.15
            and 'climb_rate' in r and abs(r['roll']) < 45]
    if idle:
        climbs = [r['climb_rate'] for r in idle]
        print(f"\n  Idle descent (thr<0.15, |roll|<45°):")
        print(f"    mean={sum(climbs)/len(climbs):.2f} m/s  n={len(climbs)}")


def roll_analysis(rows):
    """Roll response: rate vs command, binned by airspeed."""
    print("=" * 70)
    print("SIM ROLL RESPONSE")
    print("=" * 70)

    rate_bins = {'slow': [], 'cruise': [], 'fast': []}
    cmd_rate_pairs = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        if 'roll_rate' not in r:
            continue
        sb = speed_bin(r['vel'])
        rate_bins[sb].append(abs(r['roll_rate']))
        cmd_mag = abs(r['outRl'])
        if cmd_mag > 0.05:
            cmd_rate_pairs[sb].append((cmd_mag, abs(r['roll_rate'])))

    ascii_histogram(rate_bins, "|Roll rate| by airspeed", "deg/s")

    print("\n  Roll rate gain (deg/s per unit outRl):")
    for sb in ['slow', 'cruise', 'fast']:
        pairs = cmd_rate_pairs[sb]
        if len(pairs) > 10:
            mean_cmd = sum(p[0] for p in pairs) / len(pairs)
            mean_rate = sum(p[1] for p in pairs) / len(pairs)
            gain = mean_rate / mean_cmd if mean_cmd > 0.01 else 0
            print(f"    {sb:>6}: gain={gain:.2f} deg/s/unit  "
                  f"(mean_cmd={mean_cmd:.2f}, mean_rate={mean_rate:.1f} deg/s, n={len(pairs)})")


def pitch_analysis(rows):
    """Pitch response: rate vs command, binned by airspeed."""
    print("=" * 70)
    print("SIM PITCH RESPONSE")
    print("=" * 70)

    rate_bins = {'slow': [], 'cruise': [], 'fast': []}
    cmd_rate_pairs = {'slow': [], 'cruise': [], 'fast': []}

    for r in rows:
        if 'pitch_rate' not in r:
            continue
        sb = speed_bin(r['vel'])
        rate_bins[sb].append(abs(r['pitch_rate']))
        cmd_mag = abs(r['outPt'])
        if cmd_mag > 0.05:
            cmd_rate_pairs[sb].append((cmd_mag, abs(r['pitch_rate'])))

    ascii_histogram(rate_bins, "|Pitch rate| by airspeed", "deg/s")

    print("\n  Pitch rate gain (deg/s per unit outPt):")
    for sb in ['slow', 'cruise', 'fast']:
        pairs = cmd_rate_pairs[sb]
        if len(pairs) > 10:
            mean_cmd = sum(p[0] for p in pairs) / len(pairs)
            mean_rate = sum(p[1] for p in pairs) / len(pairs)
            gain = mean_rate / mean_cmd if mean_cmd > 0.01 else 0
            print(f"    {sb:>6}: gain={gain:.2f} deg/s/unit  "
                  f"(mean_cmd={mean_cmd:.2f}, mean_rate={mean_rate:.1f} deg/s, n={len(pairs)})")


def main():
    parser = argparse.ArgumentParser(description='Extract sim response curves from data.dat')
    parser.add_argument('--axis', required=True, choices=['throttle', 'roll', 'pitch', 'all'])
    parser.add_argument('--data', required=True, help='Path to data.dat')
    parser.add_argument('--gen-range', default='1-5', help='Generation range, e.g. 1-5 or 400-400')
    args = parser.parse_args()

    gen_min, gen_max = [int(x) for x in args.gen_range.split('-')]
    print(f"Loading {args.data} gens {gen_min}-{gen_max}...")
    rows = parse_data_dat(args.data, gen_min, gen_max)
    print(f"  {len(rows)} steps parsed")

    rows = compute_rates(rows)
    print(f"  {len(rows)} steps with rates computed")

    if args.axis in ('throttle', 'all'):
        throttle_analysis(rows)
    if args.axis in ('roll', 'all'):
        roll_analysis(rows)
    if args.axis in ('pitch', 'all'):
        pitch_analysis(rows)


if __name__ == '__main__':
    main()
