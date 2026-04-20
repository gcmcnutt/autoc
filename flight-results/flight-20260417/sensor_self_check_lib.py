"""Sensor self-consistency audit library (024 WI1/WI2).

Reads INAV blackbox CSV, xiao flight log, or sim data.dat; converts each to
a canonical NED / q_EB / aerospace-RHR representation; runs 8 cross-checks
to verify conventions end-to-end.

Pass/fail policy (per spec 024 clarification #3): sign-inversion only.
Correlation strength is reported but not gated.

Conventions reference:
- docs/COORDINATE_CONVENTIONS.md (stack-wide)
- docs/INAV_BLACKBOX.md (source-cited INAV side)
- specs/024-sim-real-fidelity/contracts/*_contract.md (per-format)

Quaternion convention used throughout: (w, x, y, z) scalar-first, Hamilton.
q_EB (earth->body) rotates world vectors to body via q.inverse() * v_world.
"""
import csv
import re
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ACC_1G_LSB = 256        # INAV acc_1G constant (typical ICM-series IMUs)
G_MS2 = 9.81            # gravitational acceleration (m/s²)
DEG_PER_SEC_TO_RAD = np.pi / 180.0
DECIDEG_TO_RAD = 0.1 * DEG_PER_SEC_TO_RAD

# INAV column-name aliases (blackbox field names vary across configs)
BLACKBOX_VEL_NAMES = ['navVel', 'navRealVel']
BLACKBOX_ACC_NAMES = ['accSmooth', 'accADC']


# ---------------------------------------------------------------------------
# Canonical data container
# ---------------------------------------------------------------------------

@dataclass
class CanonicalData:
    """Per-sample arrays in canonical conventions.
    Arrays are all the same length N. Missing sensors are None (not logged)."""
    source: str                     # 'blackbox' | 'xiao' | 'sim'
    t_ms: np.ndarray               # timestamps in milliseconds (monotonic)
    pos: Optional[np.ndarray] = None     # (N, 3) NED m
    vel: Optional[np.ndarray] = None     # (N, 3) NED m/s
    quat: Optional[np.ndarray] = None    # (N, 4) q_EB (w, x, y, z)
    gyro: Optional[np.ndarray] = None    # (N, 3) body rad/s aerospace RHR
    accel: Optional[np.ndarray] = None   # (N, 3) body m/s² FRD
    euler: Optional[np.ndarray] = None   # (N, 3) (roll, pitch, yaw) rad aerospace
    mag: Optional[np.ndarray] = None     # (N, 3) body unit vector
    # Xiao/sim-only (optional, None for blackbox-only sources):
    cmd: Optional[np.ndarray] = None     # (N, 3) (pitch, roll, thr) ∈ [-1, 1]
    # Metadata:
    meta: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Quaternion utilities (Hamilton, scalar-first)
# ---------------------------------------------------------------------------

def quat_conj(q):
    """(w, -x, -y, -z)."""
    out = np.asarray(q, dtype=float).copy()
    out[..., 1:] *= -1.0
    return out


def quat_mul(a, b):
    """Hamilton product of two quaternions (scalar-first)."""
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], axis=-1)


def rotate_body_to_world(q_EB, v_body):
    """Apply q_EB.inverse() to a body-frame vector to get world frame."""
    # For q_EB (earth->body), v_world = q_EB^{-1} * v_body * q_EB.
    # Implement via unit-quat conjugate + explicit rotation formula for speed.
    qw, qx, qy, qz = q_EB[..., 0], q_EB[..., 1], q_EB[..., 2], q_EB[..., 3]
    vx, vy, vz = v_body[..., 0], v_body[..., 1], v_body[..., 2]
    # Using EOM01-consistent LocalToBody layout; .inverse() = transpose
    # world = R_BE * body = R_EB^T * body. We compute rows of R_EB:
    # row 0: (w²+x²-y²-z², 2(xy+wz), 2(xz-wy))
    # row 1: (2(xy-wz), w²-x²+y²-z², 2(yz+wx))
    # row 2: (2(xz+wy), 2(yz-wx), w²-x²-y²+z²)
    # world = R_EB^T * body => world[i] = sum_j R_EB[j][i] * body[j]
    ww = qw*qw
    xx = qx*qx
    yy = qy*qy
    zz = qz*qz
    wx = qw*qx
    wy = qw*qy
    wz = qw*qz
    xy = qx*qy
    xz = qx*qz
    yz = qy*qz
    # R_EB rows (per EOM01 LocalToBody convention, matching our contract)
    r00 = ww + xx - yy - zz
    r01 = 2*(xy + wz)
    r02 = 2*(xz - wy)
    r10 = 2*(xy - wz)
    r11 = ww - xx + yy - zz
    r12 = 2*(yz + wx)
    r20 = 2*(xz + wy)
    r21 = 2*(yz - wx)
    r22 = ww - xx - yy + zz
    # world = R_EB^T * body
    wx_out = r00*vx + r10*vy + r20*vz
    wy_out = r01*vx + r11*vy + r21*vz
    wz_out = r02*vx + r12*vy + r22*vz
    return np.stack([wx_out, wy_out, wz_out], axis=-1)


def rotate_world_to_body(q_EB, v_world):
    """Apply q_EB to a world-frame vector to get body frame."""
    qw, qx, qy, qz = q_EB[..., 0], q_EB[..., 1], q_EB[..., 2], q_EB[..., 3]
    vx, vy, vz = v_world[..., 0], v_world[..., 1], v_world[..., 2]
    ww = qw*qw; xx = qx*qx; yy = qy*qy; zz = qz*qz
    wx = qw*qx; wy = qw*qy; wz = qw*qz
    xy = qx*qy; xz = qx*qz; yz = qy*qz
    r00 = ww + xx - yy - zz
    r01 = 2*(xy + wz)
    r02 = 2*(xz - wy)
    r10 = 2*(xy - wz)
    r11 = ww - xx + yy - zz
    r12 = 2*(yz + wx)
    r20 = 2*(xz + wy)
    r21 = 2*(yz - wx)
    r22 = ww - xx - yy + zz
    # body = R_EB * world
    bx = r00*vx + r01*vy + r02*vz
    by = r10*vx + r11*vy + r12*vz
    bz = r20*vx + r21*vy + r22*vz
    return np.stack([bx, by, bz], axis=-1)


def quat_to_euler_zyx(q_EB):
    """Extract (roll, pitch, yaw) in radians from q_EB using ZYX intrinsic.

    Uses the identity derived from EOM01's LocalToBody layout:
      pitch θ = asin(-nose_world_Z) = asin(2(wy - xz))
      roll  φ = atan2(rw_world_Z, world_Z_body) — simpler: atan2(2(yz+wx), ww-xx-yy+zz)
      yaw   ψ = atan2(nose_world_Y, nose_world_X) = atan2(2(xy+wz), ww+xx-yy-zz)
    """
    qw = q_EB[..., 0]
    qx = q_EB[..., 1]
    qy = q_EB[..., 2]
    qz = q_EB[..., 3]
    ww = qw*qw; xx = qx*qx; yy = qy*qy; zz = qz*qz
    # pitch: asin clamped
    sin_pitch = np.clip(2*(qw*qy - qx*qz), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    # roll: atan2 of (body-Y-projected-to-world-Z) / (body-Z-projected-to-world-Z)
    roll = np.arctan2(2*(qy*qz + qw*qx), ww - xx - yy + zz)
    # yaw
    yaw = np.arctan2(2*(qx*qy + qw*qz), ww + xx - yy - zz)
    return np.stack([roll, pitch, yaw], axis=-1)


def rate_from_quat_pairs(q, t_ms):
    """Body-rate (rad/s) from consecutive q_EB samples via
       q_delta = q_prev.inverse() * q_curr;  rate = 2 * q_delta.vec / dt.

    Returns (rates (N-1, 3), midpoint_t_ms (N-1,))."""
    dt = np.diff(t_ms) / 1000.0  # seconds
    q_inv = quat_conj(q[:-1])
    q_d = quat_mul(q_inv, q[1:])
    # Shortest-path: flip if scalar < 0
    neg = q_d[:, 0] < 0
    q_d[neg] = -q_d[neg]
    rates = 2.0 * q_d[:, 1:] / dt[:, None]
    mid_t = 0.5 * (t_ms[:-1] + t_ms[1:])
    return rates, mid_t


# ---------------------------------------------------------------------------
# Source readers
# ---------------------------------------------------------------------------

def _find_col(header, candidates):
    """Return the first matching column name from candidates, or None."""
    for c in candidates:
        if c in header:
            return c
    return None


def read_blackbox_csv(path) -> CanonicalData:
    """Read INAV blackbox CSV and convert to canonical conventions.

    Transforms applied (per contracts/blackbox_csv_contract.md):
    - navPos / navVel: NEU cm -> NED m (divide 100, negate Z)
    - quaternion: ×10000 int -> unit quat, apply INAV->aerospace transform.
      Pre-T044: simple conjugate. Post-T044: bench-derived transform.
      Using simple conjugate currently; this is THE audit target.
    - gyroADC: deg/s INAV -> rad/s aerospace (negate pitch, yaw).
    - accSmooth: (acc_1G=256) LSB -> m/s² body FRD.
    - attitude: decideg -> rad aerospace (if present; negate pitch).
    - magADC: raw -> unit vector direction.
    """
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
    header = [h.strip() for h in header]
    ncols = len(header)
    # Build column index map
    col = {name: i for i, name in enumerate(header)}

    # Required:
    assert 'time (us)' in col, "blackbox CSV missing 'time (us)' column"
    assert 'quaternion[0]' in col, (
        "blackbox CSV missing 'quaternion[0]'. This likely means the log is "
        "from mainline INAV (no quat support). Need the custom gcmcnutt/inav "
        "fork — see docs/INAV_BLACKBOX.md."
    )

    # Velocity column name varies
    vel_name = _find_col(col, [n + '[0]' for n in BLACKBOX_VEL_NAMES])
    vel_base = vel_name[:-3] if vel_name else None  # strip '[0]'
    # Accel column name varies
    acc_name = _find_col(col, [n + '[0]' for n in BLACKBOX_ACC_NAMES])
    acc_base = acc_name[:-3] if acc_name else None
    # Attitude (optional — not always logged)
    has_attitude = 'attitude[0]' in col
    has_mag = 'magADC[0]' in col

    # Stream-read rows into arrays
    t_us_list = []
    pos_list = []
    vel_list = []
    quat_list = []
    gyro_list = []
    accel_list = []
    euler_list = []
    mag_list = []

    with open(path, newline='') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            if len(row) < ncols:
                continue  # skip malformed lines
            try:
                t_us_list.append(int(row[col['time (us)']]))
                pos_list.append([
                    float(row[col['navPos[0]']]),
                    float(row[col['navPos[1]']]),
                    float(row[col['navPos[2]']]),
                ])
                if vel_base:
                    vel_list.append([
                        float(row[col[f'{vel_base}[0]']]),
                        float(row[col[f'{vel_base}[1]']]),
                        float(row[col[f'{vel_base}[2]']]),
                    ])
                quat_list.append([
                    float(row[col['quaternion[0]']]),
                    float(row[col['quaternion[1]']]),
                    float(row[col['quaternion[2]']]),
                    float(row[col['quaternion[3]']]),
                ])
                gyro_list.append([
                    float(row[col['gyroADC[0]']]),
                    float(row[col['gyroADC[1]']]),
                    float(row[col['gyroADC[2]']]),
                ])
                if acc_base:
                    accel_list.append([
                        float(row[col[f'{acc_base}[0]']]),
                        float(row[col[f'{acc_base}[1]']]),
                        float(row[col[f'{acc_base}[2]']]),
                    ])
                if has_attitude:
                    euler_list.append([
                        float(row[col['attitude[0]']]),
                        float(row[col['attitude[1]']]),
                        float(row[col['attitude[2]']]),
                    ])
                if has_mag:
                    mag_list.append([
                        float(row[col['magADC[0]']]),
                        float(row[col['magADC[1]']]),
                        float(row[col['magADC[2]']]),
                    ])
            except (ValueError, IndexError):
                continue

    t_us = np.array(t_us_list, dtype=np.int64)
    t_ms = t_us // 1000

    # Apply transforms to canonical
    pos_raw = np.array(pos_list, dtype=float)
    pos = pos_raw / 100.0  # cm -> m
    pos[:, 2] *= -1.0      # NEU Up -> NED Down

    vel = None
    if vel_list:
        vel_raw = np.array(vel_list, dtype=float)
        vel = vel_raw / 100.0
        vel[:, 2] *= -1.0

    # Quaternion: /10000, then INAV->aerospace transform.
    # CURRENT working transform (pre-T044 fix): simple conjugate as msplink does.
    # This is what we AUDIT. If the audit fails, the fix lands here (and in
    # msplink) per T044.
    quat_raw = np.array(quat_list, dtype=float) / 10000.0
    # Simple conjugate (the transform we think is correct, pending WI5 bench)
    quat = quat_raw.copy()
    quat[:, 1:] *= -1.0
    # Re-normalize
    quat_norm = np.linalg.norm(quat, axis=1, keepdims=True)
    quat = quat / np.maximum(quat_norm, 1e-9)

    # Gyro: deg/s -> rad/s; negate pitch and yaw (aerospace RHR)
    gyro_raw = np.array(gyro_list, dtype=float)
    gyro = gyro_raw * DEG_PER_SEC_TO_RAD
    gyro[:, 1] *= -1.0
    gyro[:, 2] *= -1.0

    accel = None
    if accel_list:
        acc_raw = np.array(accel_list, dtype=float)
        accel = acc_raw * (G_MS2 / ACC_1G_LSB)

    euler = None
    if euler_list:
        eu_raw = np.array(euler_list, dtype=float)
        # attitude[] is decideg: [roll, pitch, yaw]; pitch is INAV nose-down-positive
        euler = eu_raw * DECIDEG_TO_RAD
        euler[:, 1] *= -1.0  # negate pitch -> aerospace nose-up-positive

    mag = None
    if mag_list:
        mag_raw = np.array(mag_list, dtype=float)
        norms = np.linalg.norm(mag_raw, axis=1, keepdims=True)
        mag = mag_raw / np.maximum(norms, 1e-9)

    return CanonicalData(
        source='blackbox', t_ms=t_ms,
        pos=pos, vel=vel, quat=quat, gyro=gyro,
        accel=accel, euler=euler, mag=mag,
        meta={'path': path, 'rows': len(t_ms), 'vel_col': vel_base,
              'acc_col': acc_base, 'has_attitude': has_attitude,
              'has_mag': has_mag})


# Xiao log parsing: Nav State and NN lines
_XIAO_NAV = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+Nav State:.*?'
    r'pos=\[([^\]]+)\].*?vel=\[([^\]]+)\].*?quat=\[([^\]]+)\]')
_XIAO_NN = re.compile(
    r'#(\d+)\s+(\d+)\s+(\d+)\s+i\s+NN:.*?'
    r'q=\[([^\]]+)\].*?g=\[([^\]]+)\].*?out=\[([^\]]+)\]')


def read_xiao_log(path) -> CanonicalData:
    """Read xiao flight log (text), canonical quantities assumed already
    aerospace-RHR q_EB / NED m (xiao emits post-msplink-transform)."""
    t_ms_list = []
    pos_list = []
    vel_list = []
    quat_list = []
    gyro_list = []
    cmd_list = []

    with open(path) as f:
        for line in f:
            mn = _XIAO_NN.search(line)
            if mn:
                t_ms_list.append(int(mn.group(2)))  # xiao_ms
                q = [float(x) for x in mn.group(4).split(',')]
                g = [float(x) for x in mn.group(5).split(',')]
                o = [float(x) for x in mn.group(6).split(',')]
                quat_list.append(q)
                gyro_list.append(g)
                cmd_list.append(o)
                # Xiao NN line doesn't have pos/vel directly — those come
                # from the preceding Nav State line. For now, leave pos/vel
                # None for xiao-only source; use blackbox for those.
                continue
            mnav = _XIAO_NAV.search(line)
            if mnav:
                # Nav State has its own timeline — we could build a
                # separate pos/vel series. Keep minimal for now.
                pass

    n = len(t_ms_list)
    if n == 0:
        raise ValueError(f"No NN lines parsed from xiao log: {path}")

    return CanonicalData(
        source='xiao',
        t_ms=np.array(t_ms_list, dtype=np.int64),
        quat=np.array(quat_list, dtype=float),
        gyro=np.array(gyro_list, dtype=float),
        cmd=np.array(cmd_list, dtype=float),
        meta={'path': path, 'nn_samples': n})


def read_sim_data_dat(path) -> CanonicalData:
    """Read sim data.dat. All columns already in canonical stack conventions."""
    with open(path) as f:
        header = f.readline().split()
    col = {name: i for i, name in enumerate(header)}

    t_ms_list = []
    pos_list = []
    quat_list = []
    gyro_list = []
    cmd_list = []
    vel_body_list = []

    with open(path) as f:
        f.readline()  # skip header
        for line in f:
            parts = line.split()
            if len(parts) < len(header):
                continue
            try:
                t_ms_list.append(int(parts[col['Time']]))
                pos_list.append([
                    float(parts[col['X']]),
                    float(parts[col['Y']]),
                    float(parts[col['Z']]),
                ])
                quat_list.append([
                    float(parts[col['qw']]),
                    float(parts[col['qx']]),
                    float(parts[col['qy']]),
                    float(parts[col['qz']]),
                ])
                gyro_list.append([
                    float(parts[col['gyrP']]),
                    float(parts[col['gyrQ']]),
                    float(parts[col['gyrR']]),
                ])
                cmd_list.append([
                    float(parts[col['outPt']]),
                    float(parts[col['outRl']]),
                    float(parts[col['outTh']]),
                ])
                vel_body_list.append([
                    float(parts[col['vxBdy']]),
                    float(parts[col['vyBdy']]),
                    float(parts[col['vzBdy']]),
                ])
            except (ValueError, KeyError, IndexError):
                continue

    n = len(t_ms_list)
    t_ms = np.array(t_ms_list, dtype=np.int64)
    pos = np.array(pos_list, dtype=float)
    quat = np.array(quat_list, dtype=float)
    gyro = np.array(gyro_list, dtype=float)
    cmd = np.array(cmd_list, dtype=float)

    # Sim logs velocity in body frame; rotate to world NED for canonical
    vel_body = np.array(vel_body_list, dtype=float)
    vel = rotate_body_to_world(quat, vel_body)

    return CanonicalData(
        source='sim', t_ms=t_ms,
        pos=pos, vel=vel, quat=quat, gyro=gyro, cmd=cmd,
        meta={'path': path, 'rows': n})


# ---------------------------------------------------------------------------
# Fusion join (blackbox x xiao)
# ---------------------------------------------------------------------------

def fuse_blackbox_xiao(bb: CanonicalData, xl: CanonicalData, max_skew_ms=50):
    """Align xiao samples to blackbox samples by timestamp.

    Uses blackbox time (ms) and xiao's inav_ms (we use xiao_ms here as a
    proxy; future work: parse the third-column inav_ms from the log directly).
    Returns indices into bb for each matching xiao sample, plus diagnostics.
    """
    bb_t = bb.t_ms
    xl_t = xl.t_ms
    # Binary search each xiao timestamp in bb
    idx = np.searchsorted(bb_t, xl_t)
    idx = np.clip(idx, 1, len(bb_t) - 1)
    # Choose closer of idx-1 vs idx
    left = bb_t[idx - 1]
    right = bb_t[idx]
    pick = np.where(np.abs(xl_t - left) <= np.abs(xl_t - right), idx - 1, idx)
    skew_ms = bb_t[pick] - xl_t
    matched = np.abs(skew_ms) <= max_skew_ms
    return {
        'xiao_idx': np.arange(len(xl_t))[matched],
        'bb_idx': pick[matched],
        'skew_ms': skew_ms[matched],
        'total_xiao': len(xl_t),
        'matched': int(matched.sum()),
        'median_skew_ms': float(np.median(np.abs(skew_ms[matched]))) if matched.any() else float('nan'),
        'max_skew_over_threshold': int(np.sum(np.abs(skew_ms) > max_skew_ms)),
    }


# ---------------------------------------------------------------------------
# Cross-check computations
# ---------------------------------------------------------------------------

def _sloperr(x, y):
    """Return (slope, intercept, pearson_r, n). NaN if insufficient data."""
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 3:
        return float('nan'), float('nan'), float('nan'), len(x)
    slope, intercept = np.polyfit(x, y, 1)
    r = np.corrcoef(x, y)[0, 1]
    return float(slope), float(intercept), float(r), int(len(x))


def check_position_vs_velocity(d: CanonicalData):
    """Integrate vel over time, compare to pos delta."""
    if d.pos is None or d.vel is None:
        return {'skipped': 'no pos or vel'}
    t_s = d.t_ms.astype(float) / 1000.0
    # Trapezoidal integration of velocity
    dt = np.diff(t_s)
    mid_vel = 0.5 * (d.vel[:-1] + d.vel[1:])
    pos_integrated = np.cumsum(mid_vel * dt[:, None], axis=0)
    pos_integrated = np.vstack([[0, 0, 0], pos_integrated])
    pos_delta = d.pos - d.pos[0]
    results = {}
    for i, axis in enumerate(['N', 'E', 'D']):
        s, b, r, n = _sloperr(pos_delta[:, i], pos_integrated[:, i])
        results[axis] = {'slope': s, 'intercept': b, 'r': r, 'n': n}
    return results


def check_gyro_vs_quat_delta(d: CanonicalData):
    """q_delta-derived rate vs logged gyro (midpoint of sample pairs)."""
    if d.gyro is None or d.quat is None:
        return {'skipped': 'no gyro or quat'}
    rates, mid_t = rate_from_quat_pairs(d.quat, d.t_ms)
    # Pair gyro at midpoints
    gyro_mid = 0.5 * (d.gyro[:-1] + d.gyro[1:])
    results = {}
    for i, axis in enumerate(['p', 'q', 'r']):
        s, b, r, n = _sloperr(gyro_mid[:, i], rates[:, i])
        results[axis] = {'slope': s, 'intercept': b, 'r': r, 'n': n}
    return results


def check_euler_quat_vs_attitude(d: CanonicalData):
    """Our Euler extraction from canonical q_EB vs logged canonical attitude[]."""
    if d.euler is None or d.quat is None:
        return {'skipped': 'no euler or quat'}
    ours = quat_to_euler_zyx(d.quat)
    results = {}
    # Yaw wraps at ±π; unwrap before comparing
    ours_u = ours.copy()
    theirs_u = d.euler.copy()
    for i in range(3):
        ours_u[:, i] = np.unwrap(ours_u[:, i])
        theirs_u[:, i] = np.unwrap(theirs_u[:, i])
    for i, axis in enumerate(['roll', 'pitch', 'yaw']):
        s, b, r, n = _sloperr(theirs_u[:, i], ours_u[:, i])
        results[axis] = {'slope': s, 'intercept': b, 'r': r, 'n': n}
    return results


def check_accel_vs_gravity(d: CanonicalData):
    """At quasi-steady flight, body-Z accel ≈ g×cos(θ)×cos(φ).
    Expected gravity in body = q_EB * (0, 0, g) in world NED (Z-down)."""
    if d.accel is None or d.quat is None:
        return {'skipped': 'no accel or quat'}
    # Expected gravity in body (NED Z-down, so world gravity is (0, 0, +g))
    g_world = np.tile(np.array([0.0, 0.0, G_MS2]), (len(d.quat), 1))
    g_body_expected = rotate_world_to_body(d.quat, g_world)
    # Filter quasi-steady samples (low gyro)
    if d.gyro is None:
        mask = np.ones(len(d.accel), dtype=bool)
    else:
        gyro_mag = np.linalg.norm(d.gyro, axis=1)
        mask = gyro_mag < 0.2  # rad/s
    results = {}
    for i, axis in enumerate(['ax', 'ay', 'az']):
        s, b, r, n = _sloperr(g_body_expected[mask, i], d.accel[mask, i])
        results[axis] = {'slope': s, 'intercept': b, 'r': r, 'n': n}
    results['filtered_samples'] = int(mask.sum())
    return results


def check_heading_vs_track(d: CanonicalData):
    """Quat-derived heading vs ground-track heading from vel (NE plane)."""
    if d.quat is None or d.vel is None:
        return {'skipped': 'no quat or vel'}
    # Quat heading
    euler = quat_to_euler_zyx(d.quat)
    psi_quat = euler[:, 2]
    # Ground track
    psi_track = np.arctan2(d.vel[:, 1], d.vel[:, 0])
    # Filter fast-enough samples
    speed = np.linalg.norm(d.vel[:, :2], axis=1)
    mask = speed > 5.0
    s, b, r, n = _sloperr(
        np.unwrap(psi_track[mask]),
        np.unwrap(psi_quat[mask]),
    )
    return {'yaw': {'slope': s, 'intercept': b, 'r': r, 'n': n},
            'filtered_samples': int(mask.sum())}


def check_mag_vs_heading(d: CanonicalData):
    """Rotate mag body vector to world via q_EB; horizontal direction should
    point to magnetic north (offset by declination)."""
    if d.mag is None or d.quat is None:
        return {'skipped': 'no mag or quat'}
    mag_world = rotate_body_to_world(d.quat, d.mag)
    # Horizontal mag heading (ignore Z)
    psi_mag = np.arctan2(mag_world[:, 1], mag_world[:, 0])
    # Quat yaw
    euler = quat_to_euler_zyx(d.quat)
    psi_quat = euler[:, 2]
    # Difference should be ~constant (the declination)
    diff = np.mod(psi_mag - psi_quat + np.pi, 2*np.pi) - np.pi
    # Report mean offset + stability
    return {
        'mean_offset_deg': float(np.degrees(np.nanmean(diff))),
        'stddev_deg': float(np.degrees(np.nanstd(diff))),
        'n': len(diff),
    }


def check_attitude_vs_velocity_direction(d: CanonicalData):
    """Body-forward rotated to world via q_EB vs normalized velocity direction.
    During coordinated flight, these should agree modulo wind/sideslip."""
    if d.quat is None or d.vel is None:
        return {'skipped': 'no quat or vel'}
    nose_body = np.tile(np.array([1.0, 0.0, 0.0]), (len(d.quat), 1))
    nose_world = rotate_body_to_world(d.quat, nose_body)
    vel_norm = np.linalg.norm(d.vel, axis=1, keepdims=True)
    vel_dir = d.vel / np.maximum(vel_norm, 1e-9)
    # Filter fast samples
    mask = vel_norm.flatten() > 5.0
    # Correlate per axis
    results = {}
    for i, axis in enumerate(['N', 'E', 'D']):
        s, b, r, n = _sloperr(vel_dir[mask, i], nose_world[mask, i])
        results[axis] = {'slope': s, 'intercept': b, 'r': r, 'n': n}
    results['filtered_samples'] = int(mask.sum())
    return results


def check_cmd_vs_attitude_change(d: CanonicalData, lag_ms=200):
    """NN command at t vs body rate at t+lag. For sim (single source, has cmd).
    For flight, use fuse_blackbox_xiao() to bring cmd in."""
    if d.cmd is None or d.quat is None:
        return {'skipped': 'no cmd or quat'}
    rates, mid_t = rate_from_quat_pairs(d.quat, d.t_ms)
    # For each cmd at time t, find rate at time t+lag
    target_t = d.t_ms[:-1] + lag_ms  # shift by lag (drop last cmd sample since no rate for it)
    idx = np.searchsorted(mid_t, target_t)
    idx = np.clip(idx, 0, len(mid_t) - 1)
    # Command axes map: cmd[0]=pitch->q, cmd[1]=roll->p, cmd[2]=throttle (no attitude rate)
    results = {}
    # Pitch cmd vs q (rate[1])
    s, b, r, n = _sloperr(d.cmd[:-1, 0], rates[idx, 1])
    results['pitch->q'] = {'slope': s, 'intercept': b, 'r': r, 'n': n, 'lag_ms': lag_ms}
    # Roll cmd vs p (rate[0])
    s, b, r, n = _sloperr(d.cmd[:-1, 1], rates[idx, 0])
    results['roll->p'] = {'slope': s, 'intercept': b, 'r': r, 'n': n, 'lag_ms': lag_ms}
    return results


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

CHECKS = [
    ('1. Position ↔ velocity integration', check_position_vs_velocity),
    ('2. Gyro ↔ quat-delta',               check_gyro_vs_quat_delta),
    ('3. Euler(quat) ↔ attitude[]',        check_euler_quat_vs_attitude),
    ('4. Accel ↔ gravity (quasi-steady)',  check_accel_vs_gravity),
    ('5. Heading ↔ ground track',          check_heading_vs_track),
    ('6. Mag ↔ heading',                   check_mag_vs_heading),
    ('7. Attitude vector ↔ velocity dir',  check_attitude_vs_velocity_direction),
    ('8. Cmd ↔ attitude change (rate)',    check_cmd_vs_attitude_change),
]


def run_all_checks(d: CanonicalData):
    """Run all applicable checks, return report dict with per-check results."""
    return {name: fn(d) for name, fn in CHECKS}


def format_report(d: CanonicalData, report: dict) -> str:
    """Render a text report; mark pass/fail based on sign-inversion gate."""
    lines = []
    lines.append(f"Sensor self-consistency audit — source={d.source}")
    lines.append(f"  path: {d.meta.get('path', '?')}")
    lines.append(f"  samples: {len(d.t_ms)}, span: "
                 f"{(d.t_ms[-1] - d.t_ms[0]) / 1000:.1f}s, "
                 f"median dt: {np.median(np.diff(d.t_ms)):.0f}ms")
    lines.append("")
    lines.append("Pass/fail policy: sign-inversion gate (slope sign wrong = FAIL).")
    lines.append("Correlation 'r' reported but not gated.")
    lines.append("")
    for name, result in report.items():
        if 'skipped' in result:
            lines.append(f"  {name}: SKIP ({result['skipped']})")
            continue
        # Extract per-axis slope/r; FAIL if any axis slope < 0
        fail = False
        warn_notes = []
        axis_lines = []
        for axis, d_ax in result.items():
            if isinstance(d_ax, dict) and 'slope' in d_ax:
                s = d_ax['slope']
                r = d_ax['r']
                n = d_ax['n']
                sign = '+' if s >= 0 else '-'
                if s < 0:
                    fail = True
                axis_lines.append(f"      {axis:>12}  slope={s:+.3f}  r={r:+.3f}  n={n}")
        status = 'FAIL' if fail else 'PASS'
        lines.append(f"  {name}: {status}")
        lines.extend(axis_lines)
        # Special-case free-form results (mean_offset_deg, etc.)
        for key, val in result.items():
            if not isinstance(val, dict):
                lines.append(f"      {key}: {val}")
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Self-test (T013)
# ---------------------------------------------------------------------------

def _self_test():
    """Run quick sanity tests on the math library."""
    print("self-test: start")

    # 1. Identity quaternion gives zero rate
    q_id = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
    t_ms = np.array([0, 100, 200], dtype=np.int64)
    rates, _ = rate_from_quat_pairs(q_id, t_ms)
    assert np.allclose(rates, 0.0, atol=1e-9), f"identity rate nonzero: {rates}"
    print("  [ok] identity quat -> zero rate")

    # 2. Pure roll: q(t) = (cos(t/2), sin(t/2), 0, 0), d/dt should be (1, 0, 0) rad/s
    dt = 0.1  # seconds
    ts = np.arange(5) * dt
    half = 1.0 * ts / 2.0
    q_roll = np.stack([np.cos(half), np.sin(half), np.zeros_like(ts), np.zeros_like(ts)], axis=1)
    t_ms = (ts * 1000).astype(np.int64)
    rates, _ = rate_from_quat_pairs(q_roll, t_ms)
    assert np.allclose(rates[:, 0], 1.0, atol=1e-3), f"roll rate: {rates[:, 0]}"
    assert np.allclose(rates[:, 1:], 0.0, atol=1e-3), f"roll cross-axis: {rates[:, 1:]}"
    print("  [ok] pure roll -> (1, 0, 0) rad/s")

    # 3. quat_to_euler_zyx on identity gives (0, 0, 0)
    eu = quat_to_euler_zyx(np.array([[1.0, 0.0, 0.0, 0.0]]))
    assert np.allclose(eu, 0.0, atol=1e-9), f"identity euler: {eu}"
    print("  [ok] identity quat -> (0, 0, 0) euler")

    # 4. Nose-up 45° aerospace: q_EB = (cos22.5, 0, sin22.5, 0); extract pitch
    q45 = np.array([[np.cos(np.pi/8), 0.0, np.sin(np.pi/8), 0.0]])
    eu = quat_to_euler_zyx(q45)
    assert np.isclose(eu[0, 1], np.pi/4, atol=1e-6), f"nose-up pitch: {eu}"
    print(f"  [ok] nose-up-45° -> pitch={np.degrees(eu[0,1]):.2f}°")

    # 5. rotate_body_to_world: body +X for nose-up-45° should be (cos45, 0, -sin45)
    #    (nose points up-forward in world NED; Z negative = up)
    nose_w = rotate_body_to_world(q45[0], np.array([1.0, 0.0, 0.0]))
    expected = np.array([np.cos(np.pi/4), 0.0, -np.sin(np.pi/4)])
    assert np.allclose(nose_w, expected, atol=1e-6), \
        f"nose_world: {nose_w}, expected {expected}"
    print(f"  [ok] nose-up-45° body+X -> world {nose_w}")

    # 6. Right-wing-down 45°: q_EB = (cos22.5, sin22.5, 0, 0); right-wing world
    q45r = np.array([[np.cos(np.pi/8), np.sin(np.pi/8), 0.0, 0.0]])
    eu = quat_to_euler_zyx(q45r)
    assert np.isclose(eu[0, 0], np.pi/4, atol=1e-6), f"roll: {eu}"
    print(f"  [ok] right-wing-down-45° -> roll={np.degrees(eu[0,0]):.2f}°")
    rw_w = rotate_body_to_world(q45r[0], np.array([0.0, 1.0, 0.0]))
    # Right wing at 45° bank: world = (0, cos45, sin45) (E positive, D positive = down)
    expected = np.array([0.0, np.cos(np.pi/4), np.sin(np.pi/4)])
    assert np.allclose(rw_w, expected, atol=1e-6), f"rw_w: {rw_w}"
    print(f"  [ok] right-wing-down-45° body+Y -> world {rw_w}")

    print("self-test: all checks passed")


if __name__ == '__main__':
    _self_test()
