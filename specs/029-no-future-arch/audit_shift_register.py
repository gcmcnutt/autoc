#!/usr/bin/env python3
"""029 US1 audit — verify the past-only sensor input arrays behave as a
shift register and that the "now" slot reconstructs to the rabbit position.

Two checks:

(A) Shift-register property: across consecutive ticks within one trajectory,
    the value of tgX[k] at tick (t+1) equals the value of tgX[k+1] at tick t.
    Same for tgY, tgZ, ds. Tolerance accounts for the "%6.3f" precision in
    data.dat (~ 1e-3).

(B) Now-slot reconstruction: at tick t, body-frame direction-cosines × distance
    rotated to world frame and added to aircraft position should equal the
    rabbit position (pathX, pathY, pathZ) for that tick — within ~ 1 m of
    float-precision noise.

Default: read the last ~50 trajectories from data.dat (covers ~latest 1-2 gens
worth of data) and audit the longest one.

Usage:
    python3 audit_shift_register.py [--in data.dat] [--tolerance 0.01] [--show-rows N]
"""

import argparse
import math
from collections import defaultdict
from pathlib import Path

# Column layout per src/autoc.cc data.dat header (029 US1 past-only).
# 5 leading tokens: Scn Bake "Pth/Wnd:Step:" Time Idx
# Then 6+6+6+6 floats: tgX[-5..0] tgY[-5..0] tgZ[-5..0] ds[-5..0]
# Then dd/dt qw qx qy qz vel gyrP gyrQ gyrR
#      outPt outRl outTh
#      pathX pathY pathZ
#      X Y Z
#      vxBdy vyBdy vzBdy
#      dhome dist along rabVl stpPt mult rampSc
#      rateCmd... pidFF... pidIntP pidIntQ pidSat
N_LEAD = 5
TGX_OFF = N_LEAD              # 6 cols
TGY_OFF = N_LEAD + 6
TGZ_OFF = N_LEAD + 12
DS_OFF  = N_LEAD + 18
DDT_OFF = N_LEAD + 24
QUAT_OFF = N_LEAD + 25        # qw qx qy qz
VEL_OFF  = N_LEAD + 29
GYRO_OFF = N_LEAD + 30        # gyrP gyrQ gyrR
OUT_OFF  = N_LEAD + 33        # outPt outRl outTh
PATH_OFF = N_LEAD + 36        # pathX pathY pathZ
POS_OFF  = N_LEAD + 39        # X Y Z (aircraft world position)


def parse_row(line):
    """Parse one data.dat row. Returns dict with parsed fields, or None on
    header / blank lines."""
    parts = line.strip().split()
    if len(parts) < POS_OFF + 3:
        return None
    try:
        # Reject header line (non-numeric)
        int(parts[0])
    except ValueError:
        return None
    # Trajectory key: (Scn, Bake, "Pth/Wnd:Step:" — just the path/wind portion)
    pwsi = parts[2]              # "001/02:0123:"
    pwsi = pwsi.rstrip(":")
    pw, step = pwsi.rsplit(":", 1)
    traj_key = (parts[0], parts[1], pw)
    return {
        "traj": traj_key,
        "step": int(step),
        "tgX": [float(x) for x in parts[TGX_OFF:TGX_OFF+6]],
        "tgY": [float(x) for x in parts[TGY_OFF:TGY_OFF+6]],
        "tgZ": [float(x) for x in parts[TGZ_OFF:TGZ_OFF+6]],
        "ds":  [float(x) for x in parts[DS_OFF:DS_OFF+6]],
        "quat": [float(x) for x in parts[QUAT_OFF:QUAT_OFF+4]],
        "path": [float(x) for x in parts[PATH_OFF:PATH_OFF+3]],
        "pos":  [float(x) for x in parts[POS_OFF:POS_OFF+3]],
    }


def tail_trajectories(path: Path, max_bytes=20_000_000, want_traj=50):
    """Stream data.dat from the end, decode the last `max_bytes`, parse rows,
    return groups keyed by (Scn, Bake, Pth/Wnd) -> list of rows sorted by step.
    Skips a partial leading row (truncated by the byte boundary)."""
    size = path.stat().st_size
    start = max(0, size - max_bytes)
    with open(path, "rb") as f:
        f.seek(start)
        text = f.read().decode("utf-8", errors="replace")
    lines = text.splitlines()
    if start > 0:
        lines = lines[1:]  # drop possibly-truncated first line
    groups = defaultdict(list)
    for line in lines:
        row = parse_row(line)
        if row is None:
            continue
        groups[row["traj"]].append(row)
    # Sort each group by step
    for k in groups:
        groups[k].sort(key=lambda r: r["step"])
    # Keep only complete-ish ones (>= 6 ticks needed for the deepest shift check)
    groups = {k: v for k, v in groups.items() if len(v) >= 6}
    # Take the last `want_traj` distinct trajectories by Pth/Wnd insertion order
    # (the last gen's worth typically lands at the tail).
    keys = list(groups.keys())[-want_traj:]
    return {k: groups[k] for k in keys}


def quat_rotate_body_to_world(q, v):
    """Rotate vector v from body frame to world frame using unit quaternion
    q = (w, x, y, z). Standard Hamilton product formula."""
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q * v * q_conj
    # Equivalent expansion (Rodrigues):
    cx = 2 * (y * vz - z * vy)
    cy = 2 * (z * vx - x * vz)
    cz = 2 * (x * vy - y * vx)
    rx = vx + w * cx + (y * cz - z * cy)
    ry = vy + w * cy + (z * cx - x * cz)
    rz = vz + w * cz + (x * cy - y * cx)
    return (rx, ry, rz)


def audit_shift_register(rows, tolerance=0.01):
    """For each consecutive-tick pair (t, t+1), verify shift register property:
    rows[t+1].tgX[k] == rows[t].tgX[k+1] for k in [0..4]
    Returns (n_checks, n_passed, max_err_per_axis).
    """
    n_checks = 0
    n_passed = 0
    max_err = {"tgX": 0.0, "tgY": 0.0, "tgZ": 0.0, "ds": 0.0}
    for i in range(len(rows) - 1):
        a, b = rows[i], rows[i + 1]
        if b["step"] != a["step"] + 1:
            continue
        for axis in ("tgX", "tgY", "tgZ", "ds"):
            for k in range(5):  # idx 0..4 shifts to 1..5
                expected = a[axis][k + 1]
                actual = b[axis][k]
                err = abs(expected - actual)
                max_err[axis] = max(max_err[axis], err)
                n_checks += 1
                if err <= tolerance:
                    n_passed += 1
    return n_checks, n_passed, max_err


def audit_now_reconstruction(rows, tolerance_m=2.0):
    """For each row, reconstruct rabbit world position from (tgX0, tgY0, tgZ0,
    ds0, quat, X/Y/Z). Compare to (pathX, pathY, pathZ).

    Reconstruction:
      target_local_body = (tgX0, tgY0, tgZ0) * ds0
      target_world = R(quat) * target_local_body + aircraft_pos
    """
    n_checks = 0
    n_passed = 0
    max_err = 0.0
    err_samples = []
    for r in rows:
        # Skip rows where ds0 ~ 0 (near-zero distance, reconstruction is noisy)
        ds_now = r["ds"][5]
        if ds_now < 0.5:
            continue
        body_vec = (r["tgX"][5] * ds_now, r["tgY"][5] * ds_now, r["tgZ"][5] * ds_now)
        world_vec = quat_rotate_body_to_world(r["quat"], body_vec)
        recon_x = world_vec[0] + r["pos"][0]
        recon_y = world_vec[1] + r["pos"][1]
        recon_z = world_vec[2] + r["pos"][2]
        dx = recon_x - r["path"][0]
        dy = recon_y - r["path"][1]
        dz = recon_z - r["path"][2]
        err_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        max_err = max(max_err, err_m)
        n_checks += 1
        if err_m <= tolerance_m:
            n_passed += 1
        err_samples.append(err_m)
    return n_checks, n_passed, max_err, err_samples


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--in", dest="inp", default="data.dat")
    p.add_argument("--tolerance", type=float, default=0.01,
                   help="shift-register tolerance (per-component, ~ float32 of %6.3f)")
    p.add_argument("--reconstruction-tol-m", type=float, default=2.0,
                   help="now-slot reconstruction tolerance in metres")
    p.add_argument("--show-rows", type=int, default=4,
                   help="show sample rows from the chosen trajectory")
    args = p.parse_args()

    path = Path(args.inp)
    print(f"Reading tail of {path} ({path.stat().st_size / 1e9:.2f} GB)...")
    groups = tail_trajectories(path)
    print(f"Found {len(groups)} complete trajectories in tail buffer.")

    if not groups:
        print("ERROR: no parseable trajectories at end of file.")
        return 1

    # Pick the longest trajectory for richest shift-register check
    best_key = max(groups.keys(), key=lambda k: len(groups[k]))
    rows = groups[best_key]
    print(f"\nAuditing trajectory: Scn={best_key[0]} Bake={best_key[1]} Pth/Wnd={best_key[2]}")
    print(f"  {len(rows)} rows, steps {rows[0]['step']}..{rows[-1]['step']}")

    if args.show_rows > 0:
        print("\nSample rows (showing tgX[-5..0] and ds[-5..0]):")
        sample_idx = list(range(min(args.show_rows, len(rows))))
        if len(rows) > args.show_rows + 2:
            sample_idx += [len(rows) // 2, len(rows) - 1]
        for i in sample_idx:
            r = rows[i]
            tgx_str = " ".join(f"{x:+.3f}" for x in r["tgX"])
            ds_str  = " ".join(f"{x:6.1f}" for x in r["ds"])
            print(f"  step={r['step']:4d}  tgX=[{tgx_str}]  ds=[{ds_str}]")

    print(f"\n=== (A) Shift register check (tolerance {args.tolerance}) ===")
    n, p_, max_err = audit_shift_register(rows, args.tolerance)
    pct = 100.0 * p_ / max(n, 1)
    print(f"  {p_}/{n} checks pass ({pct:.1f}%)")
    for axis, err in max_err.items():
        status = "PASS" if err <= args.tolerance else "FAIL"
        print(f"    {axis}: max abs err = {err:.5f}  [{status}]")

    print(f"\n=== (B) Now-slot reconstruction check (tolerance {args.reconstruction_tol_m} m) ===")
    n, p_, max_err_m, err_samples = audit_now_reconstruction(
        rows, args.reconstruction_tol_m)
    pct = 100.0 * p_ / max(n, 1)
    err_samples.sort()
    median = err_samples[len(err_samples) // 2] if err_samples else float("nan")
    p95 = err_samples[int(len(err_samples) * 0.95)] if err_samples else float("nan")
    status = "PASS" if max_err_m <= args.reconstruction_tol_m else "FAIL"
    print(f"  {p_}/{n} checks pass ({pct:.1f}%)")
    print(f"  median err = {median:.3f} m, p95 = {p95:.3f} m, max = {max_err_m:.3f} m  [{status}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
