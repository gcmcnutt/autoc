#!/usr/bin/env python3
"""AHRS independence check via gravity projection, per engage span.

The renderer's projected points / magenta spheres landing where expected
is not an AHRS accuracy check — it's a tautology, because we reconstruct
the rabbit from the same quat we're trying to validate. This script
does two things that ARE independent of the self-referential projection:

1. **Gravity projection residual.** If AHRS q_EB is accurate, then
   rotating the world gravity vector (+Z down, 9.81 m/s²) into the body
   frame via q_EB should equal the steady-state body-frame accel
   reading. Any large maneuver load (>2g total) on top of that will
   ride as additional linear-acc, but the *direction* of the AHRS-
   projected gravity should still line up with the accel vector under
   the assumption that gravity dominates a 1g+ portion of the reading
   across many frames. Time-series plot shows the component-wise
   residual; persistent offsets indicate AHRS bias, trending offsets
   indicate AHRS drift.

2. **Gyro-integrated attitude vs AHRS attitude.** Gyro integration
   drifts linearly (gyro bias). AHRS corrects via accel/mag. Plotting
   both Euler tracks over a span reveals the AHRS correction amount:
   ideally pure gyro and AHRS agree on short timescales and diverge on
   long ones (AHRS pulls back toward gravity/magnetic). If they
   diverge rapidly or disagree on sign during maneuvers, the AHRS
   filter is overcorrecting.

Reads INAV blackbox CSV. Emits one PNG per engage span (derived from
the xiao log's "NN Control: Switch enabled" … "Switch disabled" times).

Usage:
    python3 plot_gravity_check.py <blackbox.csv> <xiao_log> [out-dir]
"""

import csv
import re
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# INAV blackbox units. acc_1G is per-device (header line "H acc_1G:NNNN" in
# the raw .TXT blackbox file). Default 256 matches common ICM-series IMUs;
# this FC logs 2048 (ICM42688P at ±16g full scale).
DEFAULT_ACC_1G_LSB = 256.0
G = 9.80665


def detect_acc_1g_from_txt(csv_path: Path) -> float:
    """Read acc_1G from the sibling .TXT blackbox binary file header.
    Fall back to 256 if not found."""
    txt_path = csv_path.with_suffix(".TXT")
    if not txt_path.exists():
        # Try stripping ".01" suffix
        base = csv_path.name
        for ext in (".01.csv", ".02.csv", ".03.csv"):
            if base.endswith(ext):
                txt_path = csv_path.parent / (base[: -len(ext)] + ".TXT")
                if txt_path.exists():
                    break
    if not txt_path.exists():
        print(f"NOTE: no .TXT sibling; using ACC_1G_LSB = {DEFAULT_ACC_1G_LSB:.0f}", file=sys.stderr)
        return DEFAULT_ACC_1G_LSB
    # Binary scan for "H acc_1G:NNNN" header
    data = txt_path.read_bytes()
    import re as _re
    m = _re.search(rb"H acc_1G:(\d+)", data)
    if not m:
        print(f"NOTE: no acc_1G in {txt_path.name}; using {DEFAULT_ACC_1G_LSB:.0f}", file=sys.stderr)
        return DEFAULT_ACC_1G_LSB
    val = float(m.group(1))
    print(f"acc_1G from header: {val:.0f} LSB/g", file=sys.stderr)
    return val

SWITCH_ENABLED = re.compile(r"Switch enabled", re.I)
SWITCH_DISABLED = re.compile(r"Switch disabled|Autoc disabled", re.I)
PATH_RE = re.compile(r"\bpath=(\d+)")


def parse_xiao_spans(xiao_path: Path):
    """Return [(span_idx, inav_ms_start, inav_ms_end, path_idx), ...]."""
    spans = []
    span_start_inav = None
    cur_path = -1
    span_idx = 0
    last_inav_ms = None
    for line in xiao_path.read_text().splitlines():
        if not line.startswith("#"):
            continue
        parts = line.split(maxsplit=4)
        if len(parts) < 5:
            continue
        inav_ms = int(parts[2])
        rest = parts[4]
        last_inav_ms = inav_ms

        pm = PATH_RE.search(rest)
        if pm and span_start_inav is not None:
            cur_path = int(pm.group(1))

        if SWITCH_ENABLED.search(rest):
            if span_start_inav is not None:
                # unclosed — close at the inav_ms of this line
                spans.append((span_idx, span_start_inav, inav_ms, cur_path))
                span_idx += 1
            span_start_inav = inav_ms
            cur_path = -1
            continue

        if SWITCH_DISABLED.search(rest) and span_start_inav is not None:
            spans.append((span_idx, span_start_inav, inav_ms, cur_path))
            span_idx += 1
            span_start_inav = None
            cur_path = -1
            continue

    if span_start_inav is not None and last_inav_ms is not None:
        spans.append((span_idx, span_start_inav, last_inav_ms, cur_path))
    return spans


def load_blackbox(path: Path, acc_1g_lsb: float = DEFAULT_ACC_1G_LSB):
    """Return (t_ms, quat_static, quat_raw, accel_body, gyr_raw).

    Canonicalized per flight-results/flight-20260417/sensor_self_check_lib.py:
    - quaternion[0..3] are ×10000 ints. Divide by 10000, normalize.
    - quat_raw = INAV internal frame (no flip). Matches raw gyro for rate work.
    - quat_static = qz-flip of quat_raw. Used for static-lookup rotations
      (world gravity → body frame is a static lookup).
    - gyroADC is in deg/s. Converted to rad/s. No axis flip for raw path.
    - accSmooth is ×256 per g → m/s² via /256 * 9.80665.
    """
    t_list, q_list, a_list, gyr_list = [], [], [], []

    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        header = [h.strip() for h in header]
        col = {name: i for i, name in enumerate(header)}

        def need(name):
            if name not in col:
                raise SystemExit(f"blackbox CSV missing column: {name}")
            return col[name]

        it = need("time (us)")
        iax, iay, iaz = need("accSmooth[0]"), need("accSmooth[1]"), need("accSmooth[2]")
        igp, igq, igr = need("gyroADC[0]"), need("gyroADC[1]"), need("gyroADC[2]")
        iq = [need(f"quaternion[{i}]") for i in range(4)]

        for row in reader:
            try:
                t_us = int(row[it])
            except (ValueError, IndexError):
                continue
            ax = float(row[iax]); ay = float(row[iay]); az = float(row[iaz])
            gp = float(row[igp]); gq = float(row[igq]); gr = float(row[igr])
            qw = float(row[iq[0]]) / 10000.0
            qx = float(row[iq[1]]) / 10000.0
            qy = float(row[iq[2]]) / 10000.0
            qz = float(row[iq[3]]) / 10000.0

            t_list.append(t_us / 1000.0)  # ms
            q_list.append((qw, qx, qy, qz))
            a_list.append((ax / acc_1g_lsb * G, ay / acc_1g_lsb * G, az / acc_1g_lsb * G))
            # gyroADC in deg/s → rad/s, raw (no axis flip; stays consistent
            # with quat_raw for integration).
            gyr_list.append((gp * np.pi / 180.0,
                             gq * np.pi / 180.0,
                             gr * np.pi / 180.0))

    q_raw = np.array(q_list, dtype=float)
    norms = np.linalg.norm(q_raw, axis=1, keepdims=True)
    q_raw = q_raw / np.maximum(norms, 1e-9)
    q_static = q_raw.copy()
    q_static[:, 3] *= -1.0  # qz flip for NEU→NED static-lookup use
    return (np.array(t_list),
            q_static,
            q_raw,
            np.array(a_list),
            np.array(gyr_list))


def rotate_world_to_body(q, v_world):
    """Apply q_EB (body→world) INVERSE to v_world → v_body."""
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    vx, vy, vz = v_world[..., 0], v_world[..., 1], v_world[..., 2]
    # v_body = q_EB^{-1} * v_world * q_EB ; for unit q, inverse = conj (w, -x, -y, -z)
    cx = 2.0 * (-qy * vz + qz * vy)
    cy = 2.0 * (-qz * vx + qx * vz)
    cz = 2.0 * (-qx * vy + qy * vx)
    bx = vx + qw * cx + (-qy * cz + qz * cy)
    by = vy + qw * cy + (-qz * cx + qx * cz)
    bz = vz + qw * cz + (-qx * cy + qy * cx)
    return np.stack([bx, by, bz], axis=-1)


def integrate_gyro_quat(q0, gyr, t_ms):
    """Pure gyro integration starting from q0. Used to compare against AHRS
    quat — divergence shows the AHRS correction magnitude."""
    q = np.zeros_like(gyr[:, :1]) if False else np.empty((len(gyr), 4))
    q[0] = q0
    for i in range(1, len(gyr)):
        dt = (t_ms[i] - t_ms[i - 1]) / 1000.0
        wx, wy, wz = gyr[i]
        # q_dot = 0.5 * q ⊗ [0, w]
        qw, qx, qy, qz = q[i - 1]
        dw = 0.5 * (-qx * wx - qy * wy - qz * wz)
        dx = 0.5 * (qw * wx + qy * wz - qz * wy)
        dy = 0.5 * (qw * wy - qx * wz + qz * wx)
        dz = 0.5 * (qw * wz + qx * wy - qy * wx)
        q[i] = (qw + dw * dt, qx + dx * dt, qy + dy * dt, qz + dz * dt)
        # Normalize
        n = np.sqrt(np.sum(q[i] ** 2))
        if n > 1e-9:
            q[i] = q[i] / n
    return q


def quat_to_euler_ypr(q):
    """Z-Y-X intrinsic (yaw, pitch, roll) in rad. Matches aerospace RHR:
    +roll=right wing down, +pitch=nose up, +yaw=nose right."""
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    # pitch = asin(2(wy - xz))   [standard body→world]
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # roll = atan2(2(wx + yz), 1 - 2(x^2 + y^2))
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return yaw, pitch, roll


def plot_span(span_idx, path_idx, t_ms, q_static, q_raw, accel_body, gyr, out_dir, stem):
    if len(t_ms) < 5:
        return

    t_s = (t_ms - t_ms[0]) / 1000.0

    # Expected body-frame gravity from AHRS. q_static is the qz-flipped
    # static-lookup quaternion per docs/COORDINATE_CONVENTIONS.md.
    g_world = np.tile(np.array([0.0, 0.0, G]), (len(t_ms), 1))
    g_body_expected = rotate_world_to_body(q_static, g_world)

    residual = accel_body - g_body_expected
    residual_mag = np.linalg.norm(residual, axis=1)
    accel_mag = np.linalg.norm(accel_body, axis=1)

    # Pure gyro integration using q_raw frame (both are in INAV's raw frame,
    # so comparing them is fair). The aerospace-convention Euler extraction
    # below is just for visualization; the drift trace is what matters.
    q_gyro = integrate_gyro_quat(q_raw[0], gyr, t_ms)
    yaw_ahrs, pitch_ahrs, roll_ahrs = quat_to_euler_ypr(q_raw)
    yaw_gyro, pitch_gyro, roll_gyro = quat_to_euler_ypr(q_gyro)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # 1. Accel body-frame components vs AHRS-projected gravity
    ax = axes[0, 0]
    ax.plot(t_s, accel_body[:, 0], "-", color="tab:blue", lw=0.6, alpha=0.7, label="accX (meas)")
    ax.plot(t_s, g_body_expected[:, 0], "--", color="tab:blue", lw=0.8, alpha=0.9, label="accX (g→body from q)")
    ax.plot(t_s, accel_body[:, 1], "-", color="tab:orange", lw=0.6, alpha=0.7, label="accY (meas)")
    ax.plot(t_s, g_body_expected[:, 1], "--", color="tab:orange", lw=0.8, alpha=0.9, label="accY (g→body from q)")
    ax.plot(t_s, accel_body[:, 2], "-", color="tab:green", lw=0.6, alpha=0.7, label="accZ (meas)")
    ax.plot(t_s, g_body_expected[:, 2], "--", color="tab:green", lw=0.8, alpha=0.9, label="accZ (g→body from q)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m/s²")
    ax.set_title("Body accel vs AHRS-rotated gravity (if AHRS right, dashed fits solid under 1g)")
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=7, ncol=2, loc="upper right")

    # 2. Residual magnitude (linear acc) + accel_mag-1g
    ax = axes[0, 1]
    ax.plot(t_s, residual_mag, color="tab:red", lw=0.6, label="|accel_body - AHRS·g_world|")
    ax.plot(t_s, accel_mag - G, color="tab:purple", lw=0.6, alpha=0.7, label="|accel| − 1g (load - 1)")
    ax.axhline(0, color="gray", lw=0.4, alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m/s²")
    ax.set_title("Linear-accel residual (close to 0 in steady flight)")
    ax.grid(True, alpha=0.4)
    ax.legend(fontsize=8)

    # 3. AHRS Euler vs gyro-integrated Euler — divergence = AHRS correction
    def deg(x): return np.degrees(x)

    ax = axes[1, 0]
    ax.plot(t_s, deg(roll_ahrs), "-", color="tab:blue", lw=1, label="roll (AHRS)")
    ax.plot(t_s, deg(roll_gyro), "--", color="tab:blue", lw=0.8, alpha=0.7, label="roll (pure gyro)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("degrees")
    ax.set_title("Roll: AHRS vs pure-gyro integration")
    ax.grid(True, alpha=0.4); ax.legend(fontsize=8)

    ax = axes[1, 1]
    ax.plot(t_s, deg(pitch_ahrs), "-", color="tab:orange", lw=1, label="pitch (AHRS)")
    ax.plot(t_s, deg(pitch_gyro), "--", color="tab:orange", lw=0.8, alpha=0.7, label="pitch (pure gyro)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("degrees")
    ax.set_title("Pitch: AHRS vs pure-gyro integration")
    ax.grid(True, alpha=0.4); ax.legend(fontsize=8)

    ax = axes[2, 0]
    # Unwrap yaw for sensible visualization
    yaw_ahrs_u = np.unwrap(yaw_ahrs)
    yaw_gyro_u = np.unwrap(yaw_gyro)
    ax.plot(t_s, deg(yaw_ahrs_u), "-", color="tab:green", lw=1, label="yaw (AHRS)")
    ax.plot(t_s, deg(yaw_gyro_u), "--", color="tab:green", lw=0.8, alpha=0.7, label="yaw (pure gyro)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("degrees")
    ax.set_title("Yaw: AHRS vs pure-gyro integration")
    ax.grid(True, alpha=0.4); ax.legend(fontsize=8)

    # 4. Euler drift summary — rolling gyro-vs-AHRS difference
    ax = axes[2, 1]
    ax.plot(t_s, deg(roll_ahrs - roll_gyro), color="tab:blue", lw=0.8, label="roll drift (AHRS − gyro)")
    ax.plot(t_s, deg(pitch_ahrs - pitch_gyro), color="tab:orange", lw=0.8, label="pitch drift")
    # yaw handled with unwrap
    yaw_drift = np.unwrap(yaw_ahrs) - np.unwrap(yaw_gyro)
    ax.plot(t_s, deg(yaw_drift), color="tab:green", lw=0.8, label="yaw drift")
    ax.axhline(0, color="gray", lw=0.4, alpha=0.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("degrees")
    ax.set_title("Euler divergence (AHRS − gyro-integrated, drift = AHRS correction)")
    ax.grid(True, alpha=0.4); ax.legend(fontsize=8)

    qsteady = np.linalg.norm(gyr, axis=1) < 0.3  # rad/s — "quasi-steady"
    qsfrac = 100.0 * np.mean(qsteady)
    qsres = residual_mag[qsteady].mean() if np.any(qsteady) else float("nan")

    fig.suptitle(
        f"{stem} — span {span_idx+1}  path={path_idx}  n={len(t_ms)}  "
        f"Δt={(t_ms[-1]-t_ms[0])/1000.0:.1f}s\n"
        f"Quasi-steady fraction: {qsfrac:.1f}%   mean |residual| when quasi-steady: "
        f"{qsres:.2f} m/s² ({qsres/G:.2f}g)",
        fontsize=11,
    )
    fig.tight_layout()
    out = out_dir / f"gravity_check_{stem}_span{span_idx+1}_path{path_idx}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")


def main():
    if len(sys.argv) < 3:
        print("Usage: plot_gravity_check.py <blackbox.csv> <xiao_log> [out-dir]", file=sys.stderr)
        sys.exit(2)
    bb = Path(sys.argv[1])
    xiao = Path(sys.argv[2])
    out_dir = Path(sys.argv[3]) if len(sys.argv) > 3 else bb.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = xiao.stem
    spans = parse_xiao_spans(xiao)
    print(f"{xiao}: {len(spans)} engage spans")

    acc_1g_lsb = detect_acc_1g_from_txt(bb)
    t_ms, q_static, q_raw, a, gyr = load_blackbox(bb, acc_1g_lsb=acc_1g_lsb)
    print(f"{bb}: {len(t_ms)} blackbox rows  (acc_1G={acc_1g_lsb:.0f})")

    for span_idx, t_start, t_end, path_idx in spans:
        mask = (t_ms >= t_start) & (t_ms <= t_end)
        if not np.any(mask):
            print(f"span {span_idx+1}: no blackbox rows in [{t_start}, {t_end}] ms — skip")
            continue
        idx = np.where(mask)[0]
        print(f"span {span_idx+1} path={path_idx}: {len(idx)} rows in engage")
        # Gravity projection uses q_static (qz-flipped); gyro drift comparison
        # uses q_raw (matches gyro integration in INAV's raw frame).
        plot_span(
            span_idx, path_idx,
            t_ms[idx], q_static[idx], q_raw[idx], a[idx], gyr[idx],
            out_dir, stem,
        )


if __name__ == "__main__":
    main()
