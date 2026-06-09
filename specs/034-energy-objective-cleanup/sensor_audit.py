#!/usr/bin/env python3
"""034 sensor cross-check — verify data.dat sensor consistency tick-by-tick.

Picks the most recent complete trajectory from data.dat tail and walks it
row-by-row, running physical-consistency checks. If the sensors all agree
the NN is being fed coherent input; if any check fails consistently, the
cleanup has broken sensor wiring.

Checks per consecutive tick pair (t-1, t):
  1. Quat unit-norm: |q| ≈ 1
  2. Position integration: Δpos ≈ R(quat) · bodyVel × dt
     (world velocity = body velocity rotated by orientation)
  3. Gyro vs quat-rate: gyro rad/s should match angular rate from Δquat/dt
  4. dhome ≈ sqrt(X² + Y² + Z²)
  5. NN forward-vel (single scalar) == body-vel-X (vxBdy)
  6. Lookahead distances ds[0..5] are monotonically increasing

Usage:
    python3 sensor_audit.py [--in data.dat] [--max-bytes 20000000]
                            [--print-ticks 20] [--dt 0.1]
"""

import argparse
import math
from pathlib import Path


# data.dat layout (header-resolved by name to survive column shifts).
# The NN-input block (33 inputs) sits at columns after Idx:
#   tgX[6] tgY[6] tgZ[6] ds[6] dddt[1] qw qx qy qz vel gyrP gyrQ gyrR
# Then NN outputs (outPt/Rl/Th), then world state (pathX/Y/Z, X/Y/Z,
# vxBdy/vyBdy/vzBdy), then derived (dhome, dist, along, rabVl, ...).


def build_header_offsets(path: Path):
    """First non-blank line of data.dat → {column-name: index}."""
    with open(path, "r", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            toks = s.split()
            try:
                int(toks[0])
                return {}  # first line was data, not a header
            except ValueError:
                return {name: i for i, name in enumerate(toks)}
    return {}


def quat_to_R(qw, qx, qy, qz):
    """Hamilton quaternion (body→world) to 3x3 rotation matrix.
    Convention: vec_world = R · vec_body."""
    return [
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ]


def matvec(R, v):
    return [R[i][0]*v[0] + R[i][1]*v[1] + R[i][2]*v[2] for i in range(3)]


def quat_mul(a, b):
    """Hamilton quat product a * b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_to_body_rates(q_prev, q_curr, dt):
    """Body-frame angular rate (P, Q, R) from two consecutive quats.
    q_delta = q_prev^-1 * q_curr; small-angle rate = 2 * q_delta_vec / dt."""
    qd = quat_mul(quat_conj(q_prev), q_curr)
    # ensure shortest-arc
    if qd[0] < 0:
        qd = tuple(-x for x in qd)
    return (2.0 * qd[1] / dt, 2.0 * qd[2] / dt, 2.0 * qd[3] / dt)


def tail_window(path: Path, max_bytes: int):
    """Return last `max_bytes` of the file as decoded lines (skipping a
    possibly-truncated first line)."""
    with open(path, "rb") as f:
        f.seek(0, 2)
        sz = f.tell()
        f.seek(max(0, sz - max_bytes))
        buf = f.read()
    lines = buf.decode("utf-8", errors="replace").splitlines()
    if not lines:
        return []
    return lines[1:]  # drop possibly-truncated first line


def parse_traj(lines, hdr):
    """Stream lines, return list of dict-rows for the LAST complete
    trajectory (one Scn × Pth/Wnd) seen in the buffer."""
    col = {k: hdr[k] for k in (
        "Scn", "Bake", "Pth/Wnd:Step:", "Time", "Idx",
        "qw", "qx", "qy", "qz",
        "outPt", "outRl", "outTh",
        "X", "Y", "Z",
        "vxBdy", "vyBdy", "vzBdy",
        "dhome",
        # NN-input single-value sensors (offsets known: vel @ +29, gyro @ +30..32)
    )}
    # We need to also pull the NN-input vel + gyro columns by index since
    # their names in the header are mode-specific (kPathgenInputMeta /
    # kTrackerInputMeta). Use the audit_shift_register convention: VEL is
    # at N_LEAD+29 = 34, GYRO at +30..+32 = 35..37.
    N_LEAD = 5
    nn_vel = N_LEAD + 29
    nn_gyrP = N_LEAD + 30
    nn_gyrQ = N_LEAD + 31
    nn_gyrR = N_LEAD + 32
    nn_qw = N_LEAD + 25
    nn_qx = N_LEAD + 26
    nn_qy = N_LEAD + 27
    nn_qz = N_LEAD + 28
    # NN lookahead ds at +18..+23
    nn_ds = [N_LEAD + 18 + k for k in range(6)]

    trajectories = {}
    for raw in lines:
        parts = raw.split()
        if len(parts) <= max(max(col.values()), nn_gyrR, *nn_ds):
            continue
        try:
            scn = int(parts[col["Scn"]])
            pw_field = parts[col["Pth/Wnd:Step:"]]
            row = dict(
                scn=scn,
                pw=pw_field.split(":")[0],
                step=int(pw_field.split(":")[1]),
                time_ms=int(parts[col["Time"]]),
                qw=float(parts[col["qw"]]),
                qx=float(parts[col["qx"]]),
                qy=float(parts[col["qy"]]),
                qz=float(parts[col["qz"]]),
                X=float(parts[col["X"]]),
                Y=float(parts[col["Y"]]),
                Z=float(parts[col["Z"]]),
                vxBdy=float(parts[col["vxBdy"]]),
                vyBdy=float(parts[col["vyBdy"]]),
                vzBdy=float(parts[col["vzBdy"]]),
                dhome=float(parts[col["dhome"]]),
                outPt=float(parts[col["outPt"]]),
                outRl=float(parts[col["outRl"]]),
                outTh=float(parts[col["outTh"]]),
                nn_qw=float(parts[nn_qw]),
                nn_qx=float(parts[nn_qx]),
                nn_qy=float(parts[nn_qy]),
                nn_qz=float(parts[nn_qz]),
                nn_vel=float(parts[nn_vel]),
                nn_gyrP=float(parts[nn_gyrP]),
                nn_gyrQ=float(parts[nn_gyrQ]),
                nn_gyrR=float(parts[nn_gyrR]),
                nn_ds=[float(parts[i]) for i in nn_ds],
            )
        except (ValueError, IndexError):
            continue
        key = (scn, row["pw"])
        trajectories.setdefault(key, []).append(row)
    if not trajectories:
        return None, None
    # pick the LAST complete trajectory (highest scn, latest pw)
    keys_sorted = sorted(trajectories.keys())
    last_key = keys_sorted[-1]
    traj = sorted(trajectories[last_key], key=lambda r: r["step"])
    return last_key, traj


def check_traj(traj, dt, print_n):
    n = len(traj)
    print(f"\nTrajectory has {n} ticks (dt = {dt}s)")
    if n < 2:
        print("  not enough ticks for cross-check")
        return

    headers = (
        "step",
        "|q|",
        "|q|-1",
        "Δpos_err",
        "gyro-quat_err",
        "dhome_err",
        "nn_vel-vxBdy",
        "ds_mono",
    )
    print("  " + " ".join(f"{h:>13}" for h in headers))

    norm_err_max = 0.0
    pos_err_max = 0.0
    gyro_err_max = 0.0
    dhome_err_max = 0.0
    vel_err_max = 0.0
    ds_mono_violations = 0

    for i in range(n):
        r = traj[i]
        # 1. Quat norm
        q = (r["qw"], r["qx"], r["qy"], r["qz"])
        qn = math.sqrt(sum(x*x for x in q))
        qne = abs(qn - 1.0)
        norm_err_max = max(norm_err_max, qne)

        # 4. dhome
        actual_dhome = math.sqrt(r["X"]**2 + r["Y"]**2 + r["Z"]**2)
        dh_err = abs(actual_dhome - r["dhome"])
        dhome_err_max = max(dhome_err_max, dh_err)

        # 5. NN vel == vxBdy?
        vel_err = abs(r["nn_vel"] - r["vxBdy"])
        vel_err_max = max(vel_err_max, vel_err)

        # 6. NN ds monotonic?
        ds = r["nn_ds"]
        mono_ok = all(ds[k+1] >= ds[k] for k in range(5))
        if not mono_ok:
            ds_mono_violations += 1

        # 2. Position integration (needs prev)
        pos_err = float("nan")
        gyro_err = float("nan")
        if i > 0:
            prev = traj[i-1]
            # World velocity = R(quat_prev) · bodyVel_prev (forward Euler)
            R = quat_to_R(prev["qw"], prev["qx"], prev["qy"], prev["qz"])
            vbody = (prev["vxBdy"], prev["vyBdy"], prev["vzBdy"])
            vworld = matvec(R, vbody)
            expected_dpos = tuple(v * dt for v in vworld)
            actual_dpos = (
                r["X"] - prev["X"],
                r["Y"] - prev["Y"],
                r["Z"] - prev["Z"],
            )
            err = math.sqrt(
                sum((a-e)**2 for a, e in zip(actual_dpos, expected_dpos)))
            pos_err = err
            pos_err_max = max(pos_err_max, err)

            # 3. Gyro vs quat rate
            qp = (prev["qw"], prev["qx"], prev["qy"], prev["qz"])
            qc = q
            rates = quat_to_body_rates(qp, qc, dt)
            err_gyro = math.sqrt(
                (rates[0] - prev["nn_gyrP"])**2
                + (rates[1] - prev["nn_gyrQ"])**2
                + (rates[2] - prev["nn_gyrR"])**2)
            gyro_err = err_gyro
            gyro_err_max = max(gyro_err_max, err_gyro)

        if i < print_n:
            print(f"  {r['step']:>13} "
                  f"{qn:>13.4f} {qne:>13.2e} "
                  f"{pos_err if not math.isnan(pos_err) else 0.0:>13.3f} "
                  f"{gyro_err if not math.isnan(gyro_err) else 0.0:>13.3f} "
                  f"{dh_err:>13.3f} "
                  f"{vel_err:>13.3f} "
                  f"{'OK' if mono_ok else 'BAD':>13}")

    print("\nSummary (max errors over the trajectory):")
    print(f"  Quat norm error      : {norm_err_max:.2e}")
    print(f"  Position-int error   : {pos_err_max:.3f} m  (dt × v ≈ "
          f"{dt * abs(traj[0]['vxBdy']):.2f} m, so this is the "
          f"single-step displacement scale)")
    print(f"  Gyro vs quat-rate    : {gyro_err_max:.3f} rad/s")
    print(f"  dhome consistency    : {dhome_err_max:.3f} m")
    print(f"  NN vel vs vxBdy      : {vel_err_max:.3f}")
    print(f"  ds non-monotonic     : {ds_mono_violations}/{n} ticks")

    # Verdicts
    print("\nVerdicts:")
    print(f"  Quat unit       : {'OK' if norm_err_max < 1e-3 else 'BAD'}")
    print(f"  Position integ  : {'OK' if pos_err_max < 0.05 else 'WARN' if pos_err_max < 0.5 else 'BAD'}")
    print(f"  Gyro matches    : {'OK' if gyro_err_max < 0.05 else 'WARN' if gyro_err_max < 0.5 else 'BAD'}")
    print(f"  dhome           : {'OK' if dhome_err_max < 0.01 else 'BAD'}")
    print(f"  NN vel = vxBdy  : {'OK' if vel_err_max < 1e-3 else 'BAD'}")
    print(f"  ds monotonic    : {'OK' if ds_mono_violations == 0 else 'BAD'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, default=Path("data.dat"))
    ap.add_argument("--max-bytes", type=int, default=20_000_000,
                    help="bytes from end of file to scan (default 20MB)")
    ap.add_argument("--print-ticks", type=int, default=20,
                    help="how many per-tick rows to print (default 20)")
    ap.add_argument("--dt", type=float, default=0.1,
                    help="sim tick period in seconds (default 0.1)")
    args = ap.parse_args()

    if not args.inp.is_file():
        raise SystemExit(f"input not found: {args.inp}")

    hdr = build_header_offsets(args.inp)
    if not hdr:
        raise SystemExit("could not parse header from data.dat")

    print(f"Reading tail of {args.inp} ({args.inp.stat().st_size / 1e9:.2f} GB)…")
    lines = tail_window(args.inp, args.max_bytes)
    key, traj = parse_traj(lines, hdr)
    if traj is None:
        raise SystemExit("no complete trajectory in tail buffer "
                         "(try --max-bytes larger)")
    print(f"Audit target: scn={key[0]} pw={key[1]} ({len(traj)} ticks)")
    check_traj(traj, args.dt, args.print_ticks)


if __name__ == "__main__":
    main()
