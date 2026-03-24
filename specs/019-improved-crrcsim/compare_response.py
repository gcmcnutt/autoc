#!/usr/bin/env python3
"""Compare flight vs sim response curves side-by-side.

Usage:
    python3 specs/019-improved-crrcsim/compare_response.py \
        --axis all \
        --csv flight-results/flight-20260322/blackbox_log_2026-03-22_103857.01.csv \
        --data 20260323-data.dat --gen-range 1-5

Runs both flight_response and sim_response extraction, then prints a
side-by-side comparison with ratios.

Note on units: flight uses rcCommand/500 as command unit (roll/pitch),
motor[0] normalized 0-1 (throttle). Sim uses outPt/outRl/outTh directly
(-1..1, throttle mapped to 0-1). Both represent full-scale commands.
The flight path has INAV mixer + servo actuator between command and
control surface; sim applies directly to FDM. Ratios reflect the
full-chain response difference.
"""
import argparse
import sys
import os

# Import from same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_response import parse_blackbox, compute_rates as flight_compute_rates, speed_bin
from sim_response import parse_data_dat, compute_rates as sim_compute_rates


def gain_from_pairs(pairs):
    """Compute rate gain from (cmd_mag, rate_mag) pairs."""
    if len(pairs) < 10:
        return None, None, None, 0
    mean_cmd = sum(p[0] for p in pairs) / len(pairs)
    mean_rate = sum(p[1] for p in pairs) / len(pairs)
    gain = mean_rate / mean_cmd if mean_cmd > 0.01 else 0
    return gain, mean_cmd, mean_rate, len(pairs)


def compare_throttle(flight_rows, sim_rows):
    print("\n" + "=" * 78)
    print("THROTTLE COMPARISON (flight motor[0] vs sim outTh)")
    print("=" * 78)

    # Vmax: high throttle, roughly level
    f_level = [r for r in flight_rows if r['motor_norm'] > 0.8
               and abs(r['att_pitch']) < 15 and abs(r['att_roll']) < 30]
    s_level = [r for r in sim_rows if r['thr_norm'] > 0.8
               and abs(r['pitch']) < 15 and abs(r['roll']) < 30]

    f_vmax = max(r['speed'] for r in f_level) if f_level else 0
    s_vmax = max(r['vel'] for r in s_level) if s_level else 0
    f_vmean = sum(r['speed'] for r in f_level) / len(f_level) if f_level else 0
    s_vmean = sum(r['vel'] for r in s_level) / len(s_level) if s_level else 0

    print(f"\n  {'':>20} {'Flight':>10} {'Sim':>10} {'Ratio':>10}")
    print(f"  {'':>20} {'------':>10} {'------':>10} {'------':>10}")
    if f_vmean > 0:
        print(f"  {'Vmax mean (level)':>20} {f_vmean:10.1f} {s_vmean:10.1f} {s_vmean/f_vmean:10.2f}x")
    if f_vmax > 0:
        print(f"  {'Vmax peak':>20} {f_vmax:10.1f} {s_vmax:10.1f} {s_vmax/f_vmax:10.2f}x")

    # Climb rate at high throttle
    f_climb = [r['climb_rate'] for r in flight_rows
               if 'climb_rate' in r and r['motor_norm'] > 0.8]
    s_climb = [r['climb_rate'] for r in sim_rows
               if 'climb_rate' in r and r['thr_norm'] > 0.8]
    if f_climb and s_climb:
        fc = sum(f_climb) / len(f_climb)
        sc = sum(s_climb) / len(s_climb)
        print(f"  {'Climb (high thr)':>20} {fc:10.2f} {sc:10.2f} {'n/a' if abs(fc)<0.01 else f'{sc/fc:10.2f}x':>10}")

    # Speed distribution
    print(f"\n  Speed distribution:")
    for sb in ['slow', 'cruise', 'fast']:
        fn = sum(1 for r in flight_rows if speed_bin(r['speed']) == sb)
        sn = sum(1 for r in sim_rows if speed_bin(r['vel']) == sb)
        fp = 100 * fn / len(flight_rows) if flight_rows else 0
        sp = 100 * sn / len(sim_rows) if sim_rows else 0
        print(f"    {sb:>6}: flight {fp:4.0f}%  sim {sp:4.0f}%")


def compare_roll(flight_rows, sim_rows):
    print("\n" + "=" * 78)
    print("ROLL COMPARISON (flight rcCommand/500 vs sim outRl)")
    print("=" * 78)

    print(f"\n  {'Speed bin':>10} {'Flight gain':>14} {'Sim gain':>14} {'Ratio':>10}")
    print(f"  {'':>10} {'(deg/s/unit)':>14} {'(deg/s/unit)':>14} {'(sim/flt)':>10}")
    print(f"  {'----------':>10} {'-----------':>14} {'-----------':>14} {'------':>10}")

    for sb in ['slow', 'cruise', 'fast']:
        f_pairs = [(abs(r['rc_roll'])/500.0, abs(r['roll_rate']))
                   for r in flight_rows if 'roll_rate' in r
                   and speed_bin(r['speed']) == sb and abs(r['rc_roll']) > 25]
        s_pairs = [(abs(r['outRl']), abs(r['roll_rate']))
                   for r in sim_rows if 'roll_rate' in r
                   and speed_bin(r['vel']) == sb and abs(r['outRl']) > 0.05]

        fg, _, _, fn = gain_from_pairs(f_pairs)
        sg, _, _, sn = gain_from_pairs(s_pairs)

        if fg and sg:
            ratio = sg / fg
            print(f"  {sb:>10} {fg:14.1f} {sg:14.1f} {ratio:10.2f}x")
        elif fg:
            print(f"  {sb:>10} {fg:14.1f} {'no data':>14} {'':>10}")
        elif sg:
            print(f"  {sb:>10} {'no data':>14} {sg:14.1f} {'':>10}")


def compare_pitch(flight_rows, sim_rows):
    print("\n" + "=" * 78)
    print("PITCH COMPARISON (flight rcCommand/500 vs sim outPt)")
    print("=" * 78)

    print(f"\n  {'Speed bin':>10} {'Flight gain':>14} {'Sim gain':>14} {'Ratio':>10}")
    print(f"  {'':>10} {'(deg/s/unit)':>14} {'(deg/s/unit)':>14} {'(sim/flt)':>10}")
    print(f"  {'----------':>10} {'-----------':>14} {'-----------':>14} {'------':>10}")

    for sb in ['slow', 'cruise', 'fast']:
        f_pairs = [(abs(r['rc_pitch'])/500.0, abs(r['pitch_rate']))
                   for r in flight_rows if 'pitch_rate' in r
                   and speed_bin(r['speed']) == sb and abs(r['rc_pitch']) > 25]
        s_pairs = [(abs(r['outPt']), abs(r['pitch_rate']))
                   for r in sim_rows if 'pitch_rate' in r
                   and speed_bin(r['vel']) == sb and abs(r['outPt']) > 0.05]

        fg, _, _, fn = gain_from_pairs(f_pairs)
        sg, _, _, sn = gain_from_pairs(s_pairs)

        if fg and sg:
            ratio = sg / fg
            print(f"  {sb:>10} {fg:14.1f} {sg:14.1f} {ratio:10.2f}x")
        elif fg:
            print(f"  {sb:>10} {fg:14.1f} {'no data':>14} {'':>10}")
        elif sg:
            print(f"  {sb:>10} {'no data':>14} {sg:14.1f} {'':>10}")


def main():
    parser = argparse.ArgumentParser(description='Compare flight vs sim response curves')
    parser.add_argument('--axis', required=True, choices=['throttle', 'roll', 'pitch', 'all'])
    parser.add_argument('--csv', required=True, help='Flight blackbox CSV path')
    parser.add_argument('--data', required=True, help='Sim data.dat path')
    parser.add_argument('--gen-range', default='1-5', help='Sim generation range')
    args = parser.parse_args()

    gen_min, gen_max = [int(x) for x in args.gen_range.split('-')]

    print(f"Flight: {args.csv}")
    print(f"Sim:    {args.data} gens {gen_min}-{gen_max}")

    flight_rows = parse_blackbox(args.csv)
    flight_rows = flight_compute_rates(flight_rows)
    print(f"  Flight: {len(flight_rows)} rows with rates")

    sim_rows = parse_data_dat(args.data, gen_min, gen_max)
    sim_rows = sim_compute_rates(sim_rows)
    print(f"  Sim:    {len(sim_rows)} steps with rates")

    if args.axis in ('throttle', 'all'):
        compare_throttle(flight_rows, sim_rows)
    if args.axis in ('roll', 'all'):
        compare_roll(flight_rows, sim_rows)
    if args.axis in ('pitch', 'all'):
        compare_pitch(flight_rows, sim_rows)

    print("\n" + "-" * 78)
    print("NOTE: Flight commands go through INAV mixer + servo actuator (~22ms to 90%).")
    print("Sim applies commands directly to FDM (with slew limiter + 50ms latency).")
    print("Ratios reflect the full-chain response difference, not just aero mismatch.")


if __name__ == '__main__':
    main()
