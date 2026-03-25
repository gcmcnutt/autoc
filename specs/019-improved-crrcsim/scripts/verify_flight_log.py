#!/usr/bin/env python3
"""Verify xiao flight log NN inputs against independently computed values.

Reconstructs the StraightAndLevel path, computes expected dist/dPhi/dTheta
at each tick based on Nav State position + quaternion, and compares against
the logged NN input vector. Also verifies the history buffer chain.

Usage: python3 scripts/verify_flight_log.py <flight_log.txt>
"""

import sys
import re
import math
import numpy as np
from dataclasses import dataclass

# Constants matching autoc
SIM_RABBIT_VELOCITY = 13.0  # m/s (matches autoc.ini RabbitSpeedNominal) — used for odometer advancement
SIM_TIME_STEP_MSEC = 100    # ms
HIST_PAST = [9, 3, 1, 0]   # history indices for inputs [0-3], [6-9], [12-15]
FORECAST_OFFSETS = [1.0, 5.0]  # steps for inputs [4-5], [10-11], [16-17]

# ============================================================================
# Path generation: StraightAndLevel racetrack
# ============================================================================

@dataclass
class PathPoint:
    pos: np.ndarray   # [x, y, z]
    dist: float       # distance from start

def generate_straight_and_level():
    """Generate the StraightAndLevel racetrack path (matches embedded_pathgen_selector.h)."""
    points = []
    total_dist = 0.0
    step = 0.4  # meters

    def add_straight(start, direction, distance):
        nonlocal total_dist
        d = np.array(direction, dtype=float)
        d = d / np.linalg.norm(d)
        start_d = step if len(points) > 0 else 0.0
        dvals = np.arange(start_d, distance + step/2, step)
        for dd in dvals:
            pt = np.array(start) + d * dd
            if len(points) > 0:
                seg_dist = np.linalg.norm(pt - points[-1].pos)
                total_dist += seg_dist
            points.append(PathPoint(pos=pt.copy(), dist=total_dist))

    def add_horizontal_turn(start, radius, angle_radians, clockwise):
        nonlocal total_dist
        turn_step = 0.02  # radians

        # Determine heading from last two points
        if len(points) >= 2:
            heading = points[-1].pos - points[-2].pos
            heading = heading / np.linalg.norm(heading)
        else:
            heading = np.array([-1.0, 0.0, 0.0])

        heading_xy = np.array([heading[0], heading[1], 0.0])
        heading_xy = heading_xy / np.linalg.norm(heading_xy)

        # Right perpendicular: rotate (x,y) by -90° = (y, -x)
        right_perp = np.array([-heading_xy[1], heading_xy[0], 0.0])

        sign = 1.0 if clockwise else -1.0
        center = np.array(start) + right_perp * sign * radius

        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        angle_sign = sign

        angles = np.arange(turn_step, angle_radians + turn_step/2, turn_step)
        for angle in angles:
            total_angle = start_angle + angle_sign * angle
            pt = np.array([
                center[0] + radius * math.cos(total_angle),
                center[1] + radius * math.sin(total_angle),
                start[2]
            ])
            if len(points) > 0:
                seg_dist = np.linalg.norm(pt - points[-1].pos)
                total_dist += seg_dist
            points.append(PathPoint(pos=pt.copy(), dist=total_dist))

    # Racetrack: south 20m, 180° right, north 40m, 180° right, south 20m
    entry = [0.0, 0.0, 0.0]
    add_straight(entry, [-1, 0, 0], 20.0)

    turn1_start = points[-1].pos.tolist()
    add_horizontal_turn(turn1_start, 20.0, math.pi, True)

    north_start = points[-1].pos.tolist()
    add_straight(north_start, [1, 0, 0], 40.0)

    turn2_start = points[-1].pos.tolist()
    add_horizontal_turn(turn2_start, 20.0, math.pi, True)

    return_start = points[-1].pos.tolist()
    add_straight(return_start, [-1, 0, 0], 20.0)

    return points

def get_interpolated_position(path, odometer, offset_meters=0.0):
    """Match getInterpolatedTargetPosition from sensor_math.cc (odometer-based)"""
    if len(path) == 0:
        return np.zeros(3)

    goal_dist = odometer + offset_meters

    # Clamp to path bounds
    if goal_dist <= path[0].dist:
        return path[0].pos.copy()
    if goal_dist >= path[-1].dist:
        return path[-1].pos.copy()

    # Linear scan
    lo = 0
    for i in range(len(path) - 1):
        if path[i + 1].dist > goal_dist:
            lo = i
            break

    p0 = path[lo]
    p1 = path[lo + 1]
    dd = p1.dist - p0.dist
    if dd > 0:
        frac = (goal_dist - p0.dist) / dd
        frac = max(0.0, min(1.0, frac))
    else:
        frac = 0.0

    return p0.pos + frac * (p1.pos - p0.pos)

# ============================================================================
# Quaternion helpers
# ============================================================================

def quat_inverse(q):
    """Conjugate of unit quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_rotate(q, v):
    """Rotate vector v by quaternion q = [w, x, y, z]."""
    w, x, y, z = q
    # q * v * q^-1 via expanded formula
    t = 2.0 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)

def quat_to_euler(q):
    """Convert [w,x,y,z] to [roll, pitch, yaw] in degrees."""
    w, x, y, z = q
    # Roll
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # Yaw
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ============================================================================
# Sensor computation (matching sensor_math.cc)
# ============================================================================

def fast_atan2(y, x):
    """Python std atan2 — close enough for verification (LUT has ~0.001 rad error)."""
    if abs(x) < 1e-6 and abs(y) < 1e-6:
        return 0.0
    return math.atan2(y, x)

def compute_dphi(target_pos, aircraft_pos, quat):
    """dPhi: roll-axis error angle (body YZ plane, from -Z axis)."""
    craft_to_target = target_pos - aircraft_pos
    q_inv = quat_inverse(quat)
    target_local = quat_rotate(q_inv, craft_to_target)
    return fast_atan2(target_local[1], -target_local[2])

def compute_dtheta(target_pos, aircraft_pos, quat):
    """dTheta: pitch-axis error angle."""
    craft_to_target = target_pos - aircraft_pos
    q_inv = quat_inverse(quat)
    target_local = quat_rotate(q_inv, craft_to_target)
    return fast_atan2(-target_local[2], target_local[0])

def compute_dist(target_pos, aircraft_pos):
    """Euclidean distance to target."""
    return np.linalg.norm(target_pos - aircraft_pos)

# ============================================================================
# Log parsing
# ============================================================================

def parse_log(filename):
    """Parse flight log into Nav State and NN entries."""
    nav_states = []
    nn_entries = []

    with open(filename) as f:
        for line in f:
            # Nav State
            m = re.search(r'Nav State:.*pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\].*vel=\[([-\d.]+),([-\d.]+),([-\d.]+)\].*quat=\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\].*autoc=(\w)', line)
            if m:
                nav_states.append({
                    'pos': np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))]),
                    'vel': np.array([float(m.group(4)), float(m.group(5)), float(m.group(6))]),
                    'quat': np.array([float(m.group(7)), float(m.group(8)), float(m.group(9)), float(m.group(10))]),
                    'autoc': m.group(11) == 'Y',
                })

            # NN entry
            m = re.search(r'NN: idx=(\d+) in=\[([^\]]+)\] out=\[([^\]]+)\] rc=\[([^\]]+)\]', line)
            if m:
                nn_entries.append({
                    'idx': int(m.group(1)),
                    'inputs': [float(x) for x in m.group(2).split(',')],
                    'outputs': [float(x) for x in m.group(3).split(',')],
                    'rc': [int(x) for x in m.group(4).split(',')],
                })

    return nav_states, nn_entries

# ============================================================================
# Verification
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <flight_log.txt>")
        sys.exit(1)

    nav_states, nn_entries = parse_log(sys.argv[1])
    path = generate_straight_and_level()

    print(f"Path: {len(path)} points, length {path[-1].dist:.1f}m")
    print(f"Nav states: {len(nav_states)}, NN entries: {len(nn_entries)}")

    # Find autoc-enabled nav states (pos is relative after enable)
    autoc_states = [s for s in nav_states if s['autoc']]
    print(f"Autoc-enabled states: {len(autoc_states)}")

    # Decode quaternion orientation at start
    if autoc_states:
        q = autoc_states[0]['quat']
        roll, pitch, yaw = quat_to_euler(q)
        print(f"\nQuaternion at autoc enable: w={q[0]:.3f} x={q[1]:.3f} y={q[2]:.3f} z={q[3]:.3f}")
        print(f"Euler (deg): roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}")
        print(f"(INAV reports heading 179°, pitch 0.4°, roll 1.7°)")

    print(f"\n{'Tick':>4} {'idx':>4} | {'dist_exp':>8} {'dist_log':>8} {'d_err':>6} | {'dPhi_exp':>8} {'dPhi_log':>8} {'p_err':>6} | {'dTh_exp':>8} {'dTh_log':>8} {'t_err':>6} | {'dDdt_log':>7}")
    print("-" * 130)

    # Track history for buffer verification
    dphi_history = []
    dtheta_history = []
    dist_history = []

    history_errors = 0

    for tick, nn in enumerate(nn_entries):
        idx = nn['idx']
        inputs = nn['inputs']

        # Rabbit odometer: idx tells us path index, use path distance
        if idx < len(path):
            rabbit_odo = path[idx].dist
        else:
            rabbit_odo = path[-1].dist

        # Get aircraft state from corresponding autoc nav state
        if tick < len(autoc_states):
            pos = autoc_states[tick]['pos']
            quat = autoc_states[tick]['quat']
            vel = autoc_states[tick]['vel']
        else:
            continue

        # Compute expected target position (offset=0 = current rabbit)
        target = get_interpolated_position(path, rabbit_odo, 0.0)

        # Expected sensor values
        exp_dist = compute_dist(target, pos)
        exp_dphi = compute_dphi(target, pos, quat)
        exp_dtheta = compute_dtheta(target, pos, quat)

        # Logged values: inputs[3]=dPhi_now, inputs[9]=dTheta_now, inputs[15]=dist_now
        log_dphi = inputs[3]
        log_dtheta = inputs[9]
        log_dist = inputs[15]
        log_ddist_dt = inputs[18]

        # Errors
        dist_err = exp_dist - log_dist
        dphi_err = exp_dphi - log_dphi
        dtheta_err = exp_dtheta - log_dtheta

        if tick < 30 or tick % 20 == 0 or tick >= len(nn_entries) - 5:
            print(f"{tick:4d} {idx:4d} | {exp_dist:8.2f} {log_dist:8.2f} {dist_err:6.2f} | {exp_dphi:8.3f} {log_dphi:8.3f} {dphi_err:6.3f} | {exp_dtheta:8.3f} {log_dtheta:8.3f} {dtheta_err:6.3f} | {log_ddist_dt:7.2f}")

        # Record for history verification
        dphi_history.append(log_dphi)  # what was recorded as t-0
        dtheta_history.append(log_dtheta)
        dist_history.append(log_dist)

        # Verify history buffer: inputs[0]=dPhi at t-9, inputs[1]=t-3, inputs[2]=t-1
        if tick >= 10:  # enough history
            for hist_slot, hist_idx in enumerate(HIST_PAST):
                if hist_idx <= tick:
                    expected_val = dphi_history[tick - hist_idx]  # use the value from hist_idx ticks ago
                    logged_val = inputs[hist_slot]
                    if abs(expected_val - logged_val) > 0.01:
                        history_errors += 1
                        if history_errors <= 5:
                            print(f"  ** HISTORY MISMATCH tick={tick} dPhi slot[{hist_slot}] (t-{hist_idx}): expected={expected_val:.3f} logged={logged_val:.3f}")

            # Same for dTheta
            for hist_slot, hist_idx in enumerate(HIST_PAST):
                if hist_idx <= tick:
                    expected_val = dtheta_history[tick - hist_idx]
                    logged_val = inputs[6 + hist_slot]
                    if abs(expected_val - logged_val) > 0.01:
                        history_errors += 1
                        if history_errors <= 5:
                            print(f"  ** HISTORY MISMATCH tick={tick} dTheta slot[{hist_slot}] (t-{hist_idx}): expected={expected_val:.3f} logged={logged_val:.3f}")

            # And dist
            for hist_slot, hist_idx in enumerate(HIST_PAST):
                if hist_idx <= tick:
                    expected_val = dist_history[tick - hist_idx]
                    logged_val = inputs[12 + hist_slot]
                    if abs(expected_val - logged_val) > 0.1:
                        history_errors += 1
                        if history_errors <= 5:
                            print(f"  ** HISTORY MISMATCH tick={tick} dist slot[{hist_slot}] (t-{hist_idx}): expected={expected_val:.2f} logged={logged_val:.2f}")

    # Verify forecast slots: inputs[16]=dist at +0.1s, inputs[17]=dist at +0.5s
    print(f"\n--- Forecast verification (sample) ---")
    for tick in [5, 20, 50, 100]:
        if tick >= len(nn_entries):
            break
        nn = nn_entries[tick]
        idx = nn['idx']
        if idx < len(path):
            rabbit_odo = path[idx].dist
        else:
            rabbit_odo = path[-1].dist
        if tick < len(autoc_states):
            pos = autoc_states[tick]['pos']
        else:
            continue
        for fi, offset in enumerate(FORECAST_OFFSETS):
            offset_meters = offset * (SIM_TIME_STEP_MSEC / 1000.0) * SIM_RABBIT_VELOCITY
            future_target = get_interpolated_position(path, rabbit_odo, offset_meters)
            exp_dist = compute_dist(future_target, pos)
            log_dist = nn['inputs'][16 + fi]
            err = exp_dist - log_dist
            print(f"  tick={tick:3d} idx={idx:3d} forecast +{offset:.1f}s: expected={exp_dist:.2f} logged={log_dist:.2f} err={err:.2f}")

    print(f"\nTotal history buffer mismatches: {history_errors}")

if __name__ == '__main__':
    main()
