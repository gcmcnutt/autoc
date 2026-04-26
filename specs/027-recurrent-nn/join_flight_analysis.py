#!/usr/bin/env python3
"""027-recurrent-nn variant of join_flight_analysis.

Forked from specs/023-ood-and-engage-fixes/join_flight_analysis.py for the
04-26 RNN flight onward.  Differences vs. the 023 original:

  - B (quat consistency): tries both INAV-as-body→world and INAV-as-world→body
    and picks whichever yields lower RMS.  RMS verdict thresholds relaxed to
    account for nearest-neighbour sample-time slop (xiao 100ms vs INAV 2ms)
    — only flags ✗ at >0.4, "~ ok" between 0.1 and 0.4.
  - C0 (target acquisition): mean dtX/dt within ±0.03/s reads as "steady
    (rabbit-chase equilibrium)" instead of false-alarming on noise-floor
    numbers.
  - D (control chain): each stage labelled with this build's *expected*
    polarity in [brackets].  msplink convertPitchToMSPChannel intentionally
    inverts pitch, and INAV's elevon mix inverts both servos relative to
    rcData — so NN→rc pitch and rcData→servo* are EXPECTED negative.
    Verdict panel uses signed expectations.

The 023 copy stays untouched for pre-027 flights; use this one for 027+.

Join INAV blackbox CSV + xiao flight log on INAV timestamp, segment by
engage spans (mspOverrideFlags), and produce first-principles diagnostic
plots and an anchor report to locate sign/polarity/convention bugs.

Anchors (per span):
  A. GRAVITY        : accSmooth[2] sign tells FRD vs FLU body Z.
  B. QUAT XIAO↔INAV : RMS diff between xiao quat and INAV quat.
  C. FORWARD FLIGHT : cos(nose_world, vel_world) should be ≈ +1.
  D. CONTROL CHAIN  : for each of roll/pitch/throttle, measure corr of
                       NN output ↔ rcData ↔ gyro/motor at short lag.
                       Any negative correlation pinpoints the broken link.
  E. POLICY         : mean NN commands vs. resulting attitude/climb.

Conventions (from src/nn/evaluator.cc + include/autoc/eval/aircraft_state.h):
  World: NED (X=North, Y=East, Z=Down).
  Body:  FRD (X=forward, Y=right wing, Z=down).
  quat:  body→world rotation, [w, x, y, z] order.
  INAV:  navPos/navVel in cm; quaternion stored × 10000.

Usage:
  join_flight_analysis.py <inav_csv> <xiao_log> [--out-dir DIR]
"""
import argparse, csv, os, re, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ---------- parsing ----------

def parse_inav_csv(path):
    """Read INAV blackbox CSV into dict of numpy arrays.  Returns t_ms plus
    raw arrays and unit-converted arrays where appropriate."""
    wanted_cols = [
        'time (us)', 'mspOverrideFlags', 'motor[0]', 'servo[0]', 'servo[1]',
        'gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]',
        'accSmooth[0]', 'accSmooth[1]', 'accSmooth[2]',
        'quaternion[0]', 'quaternion[1]', 'quaternion[2]', 'quaternion[3]',
        'navPos[0]', 'navPos[1]', 'navPos[2]',
        'navVel[0]', 'navVel[1]', 'navVel[2]',
        'navTgtPos[0]', 'navTgtPos[1]', 'navTgtPos[2]',
        'navTgtVel[0]', 'navTgtVel[1]', 'navTgtVel[2]',
        'rcData[0]', 'rcData[1]', 'rcData[2]', 'rcData[3]',
        'rcCommand[0]', 'rcCommand[1]', 'rcCommand[2]', 'rcCommand[3]',
    ]
    data = {c: [] for c in wanted_cols}
    with open(path) as f:
        # INAV CSV has space-padded headers and values ("  time (us) ").
        reader = csv.reader(f, skipinitialspace=True)
        header = [h.strip() for h in next(reader)]
        col_idx = {c: i for i, c in enumerate(header)}
        for row in reader:
            for c in wanted_cols:
                i = col_idx.get(c)
                if i is None or i >= len(row):
                    data[c].append(float('nan'))
                    continue
                try:
                    data[c].append(float(row[i].strip()))
                except ValueError:
                    data[c].append(float('nan'))
    arr = {c: np.array(v) for c, v in data.items()}
    t_ms = arr['time (us)'] / 1000.0
    # INAV blackbox uses NEU (North-East-Up) for navPos/navVel in cm / cm/s.
    # Convert to NED (flip Z sign) to stay consistent with xiao's internal
    # frame (msplink.cpp:neuVecToNed does the same when ingesting MSP).
    navPos = np.stack([arr['navPos[0]'], arr['navPos[1]'], -arr['navPos[2]']], axis=1) / 100.0
    navVel = np.stack([arr['navVel[0]'], arr['navVel[1]'], -arr['navVel[2]']], axis=1) / 100.0
    navTgtPos = np.stack([arr['navTgtPos[0]'], arr['navTgtPos[1]'], -arr['navTgtPos[2]']], axis=1) / 100.0
    navTgtVel = np.stack([arr['navTgtVel[0]'], arr['navTgtVel[1]'], -arr['navTgtVel[2]']], axis=1) / 100.0
    quat = np.stack([arr['quaternion[0]'], arr['quaternion[1]'],
                     arr['quaternion[2]'], arr['quaternion[3]']], axis=1) / 10000.0
    gyro = np.stack([arr['gyroADC[0]'], arr['gyroADC[1]'], arr['gyroADC[2]']], axis=1)
    acc = np.stack([arr['accSmooth[0]'], arr['accSmooth[1]'], arr['accSmooth[2]']], axis=1)
    rcData = np.stack([arr['rcData[0]'], arr['rcData[1]'],
                       arr['rcData[2]'], arr['rcData[3]']], axis=1)
    rcCommand = np.stack([arr['rcCommand[0]'], arr['rcCommand[1]'],
                          arr['rcCommand[2]'], arr['rcCommand[3]']], axis=1)
    return dict(
        t_ms=t_ms,
        msp=arr['mspOverrideFlags'],
        motor=arr['motor[0]'],
        servo=np.stack([arr['servo[0]'], arr['servo[1]']], axis=1),
        gyro=gyro, acc=acc, quat=quat,
        navPos=navPos, navVel=navVel,
        navTgtPos=navTgtPos, navTgtVel=navTgtVel,
        rcData=rcData, rcCommand=rcCommand,
    )


NAV_RE = re.compile(
    r'#\d+\s+(\d+)\s+(\d+)\s+i\s+Nav State:\s+'
    r'pos_raw=\[([-0-9.]+),([-0-9.]+),([-0-9.]+)\]\s+'
    r'pos=\[([-0-9.]+),([-0-9.]+),([-0-9.]+)\]\s+'
    r'vel=\[([-0-9.]+),([-0-9.]+),([-0-9.]+)\]\s+'
    r'quat=\[([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+)\]'
    r'.*autoc=(\w).*path=(\d+)'
)
NN_RE = re.compile(
    r'#\d+\s+(\d+)\s+(\d+)\s+i\s+NN:\s+idx=\d+\s+'
    r'tX=\[([^\]]+)\]\s+tY=\[([^\]]+)\]\s+tZ=\[([^\]]+)\]\s+d=\[([^\]]+)\]\s+'
    r'cr=([-0-9.eE+]+)\s+q=\[([^\]]+)\]\s+as=([-0-9.eE+]+)\s+g=\[([^\]]+)\]\s+'
    r'out=\[([^\]]+)\]\s+rc=\[(\d+),(\d+),(\d+)\]'
)


def parse_xiao_log(path):
    nav, nn = [], []
    with open(path) as f:
        for line in f:
            m = NAV_RE.search(line)
            if m:
                # Group layout: 1=xiao_ms, 2=inav_ms,
                #   3,4,5=pos_raw, 6,7,8=pos, 9,10,11=vel,
                #   12,13,14,15=quat w,x,y,z, 16=autoc (Y/N), 17=path
                nav.append(dict(
                    xiao_ms=int(m.group(1)), inav_ms=int(m.group(2)),
                    pos=[float(m.group(i)) for i in (6, 7, 8)],
                    vel=[float(m.group(i)) for i in (9, 10, 11)],
                    quat=[float(m.group(i)) for i in (12, 13, 14, 15)],
                    autoc=m.group(16), path=int(m.group(17)),
                ))
                continue
            m = NN_RE.search(line)
            if m:
                flst = lambda s: [float(x) for x in s.split(',')]
                nn.append(dict(
                    xiao_ms=int(m.group(1)), inav_ms=int(m.group(2)),
                    tX=flst(m.group(3)), tY=flst(m.group(4)),
                    tZ=flst(m.group(5)), d=flst(m.group(6)),
                    cr=float(m.group(7)), q=flst(m.group(8)),
                    airspeed=float(m.group(9)), g=flst(m.group(10)),
                    out=flst(m.group(11)),  # [pitch, roll, throttle] tanh
                    rc=[int(m.group(i)) for i in (12, 13, 14)],  # roll, pitch, throttle MSP
                ))
    return nav, nn


# ---------- span detection ----------

def detect_engage_spans(t_ms, msp, flag_values=(2, 3)):
    """Return list of (t_start_ms, t_end_ms) for contiguous runs where msp is
    in flag_values.  Merge adjacent runs separated by < 100ms."""
    active = np.isin(msp, flag_values)
    spans = []
    i = 0
    n = len(active)
    while i < n:
        if active[i]:
            j = i
            while j + 1 < n and active[j + 1]:
                j += 1
            spans.append((t_ms[i], t_ms[j]))
            i = j + 1
        else:
            i += 1
    # merge near-adjacent
    merged = []
    for s, e in spans:
        if merged and s - merged[-1][1] < 100.0:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    return merged


# ---------- quaternion utilities ----------

def quat_rotate_v(q, v):
    """v_out = q * v * q_conj, where q = [w, x, y, z] rotates body→world."""
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    # Rodrigues expansion
    cx = 2.0 * (qy * vz - qz * vy)
    cy = 2.0 * (qz * vx - qx * vz)
    cz = 2.0 * (qx * vy - qy * vx)
    ox = vx + qw * cx + (qy * cz - qz * cy)
    oy = vy + qw * cy + (qz * cx - qx * cz)
    oz = vz + qw * cz + (qx * cy - qy * cx)
    return np.stack([ox, oy, oz], axis=-1)


def quat_inverse(q):
    """Conjugate (for unit quats, inverse = conjugate)."""
    qo = q.copy()
    qo[..., 1:] *= -1
    return qo


def quat_to_euler(q):
    """Return (roll, pitch, yaw) in radians for body→world quat [w,x,y,z]."""
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sinp = np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


# ---------- join ----------

def slice_inav_to_span(inav, t_start_ms, t_end_ms):
    t = inav['t_ms']
    mask = (t >= t_start_ms) & (t <= t_end_ms)
    out = {'t_ms': t[mask]}
    for k, v in inav.items():
        if k == 't_ms':
            continue
        out[k] = v[mask]
    return out


def slice_xiao_to_span(xiao_nn, xiao_nav, t_start_ms, t_end_ms):
    """Return arrays of xiao nn/nav entries whose inav_ms falls in [t_start, t_end]."""
    def filt(rows, key='inav_ms'):
        return [r for r in rows if t_start_ms <= r[key] <= t_end_ms]
    return filt(xiao_nn), filt(xiao_nav)


def nn_arrays(nn_rows):
    """Convert list of NN dicts to array-of-arrays."""
    if not nn_rows:
        return None
    return dict(
        inav_ms=np.array([r['inav_ms'] for r in nn_rows], dtype=float),
        out=np.array([r['out'] for r in nn_rows]),  # (N, 3) [pitch, roll, throttle]
        rc=np.array([r['rc'] for r in nn_rows], dtype=float),  # (N, 3) [roll, pitch, throttle] μs
        q=np.array([r['q'] for r in nn_rows]),  # (N, 4) [w, x, y, z]
        tXYZ_now=np.stack([
            np.array([r['tX'][3] for r in nn_rows]),
            np.array([r['tY'][3] for r in nn_rows]),
            np.array([r['tZ'][3] for r in nn_rows]),
        ], axis=1),
        dist_now=np.array([r['d'][3] for r in nn_rows]),
        cr=np.array([r['cr'] for r in nn_rows]),
        airspeed=np.array([r['airspeed'] for r in nn_rows]),
        gyro=np.array([r['g'] for r in nn_rows]),
    )


# ---------- anchors ----------

def max_corr_at_lag(a, b, lag_range):
    """Return (best_corr, best_lag) for Pearson corr of a vs b shifted by lag
    samples in lag_range.  Positive lag means b is shifted later (b[t+lag] vs
    a[t]) — i.e. response lags command."""
    if len(a) < 10 or len(b) < 10:
        return float('nan'), 0
    best = (0.0, 0)
    for lag in lag_range:
        if lag >= 0:
            aa = a[:len(a) - lag] if lag > 0 else a
            bb = b[lag:]
        else:
            aa = a[-lag:]
            bb = b[:len(b) + lag]
        n = min(len(aa), len(bb))
        if n < 10:
            continue
        aa, bb = aa[:n], bb[:n]
        if np.std(aa) < 1e-9 or np.std(bb) < 1e-9:
            continue
        c = np.corrcoef(aa, bb)[0, 1]
        if abs(c) > abs(best[0]):
            best = (c, lag)
    return best


def compute_anchors(inav_slc, nn, inav_rate_hz=500, span_label=""):
    """Return anchor dict and printable report string."""
    lines = []
    a = {}
    # Rate (for lag conversion)
    dt_ms = 1000.0 / inav_rate_hz

    # A) GRAVITY
    if len(inav_slc['acc']):
        acc = inav_slc['acc']
        a['acc_mean'] = np.nanmean(acc, axis=0)
        a['acc_std'] = np.nanstd(acc, axis=0)
        lines.append("A. GRAVITY ANCHOR (body-frame accel, INAV raw units):")
        lines.append(f"   mean accSmooth = [{a['acc_mean'][0]:+8.1f}, {a['acc_mean'][1]:+8.1f}, {a['acc_mean'][2]:+8.1f}]"
                     f"  σ=[{a['acc_std'][0]:.1f}, {a['acc_std'][1]:.1f}, {a['acc_std'][2]:.1f}]")
        sign_z = "FRD  (gravity → body +Z)" if a['acc_mean'][2] > 0 else "FLU  (gravity → body -Z) ← REVERSED?"
        lines.append(f"   Z polarity → {sign_z}")

    # B) QUAT CONSISTENCY
    # xiao stores body→world (per aircraft_state.h / evaluator.cc).  INAV's
    # *blackbox* quat is empirically world→body on this firmware (verified by
    # anchor C: cos(nose, vel) is +0.9 with conj, ~+0.1 without).  So the
    # honest comparison conjugates one of them first.  Try both and report
    # the better — pinpoints whether INAV blackbox flipped convention.
    if nn is not None and len(inav_slc['quat']):
        q_inav_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['quat'], nn['inav_ms'])
        q_xiao = nn['q']

        def _rms(qa, qb):
            dot = np.sum(qa * qb, axis=1)
            qb_aln = qb.copy(); qb_aln[dot < 0] *= -1
            return float(np.sqrt(np.mean((qa - qb_aln) ** 2)))

        rms_direct = _rms(q_inav_at_nn, q_xiao)
        rms_conj = _rms(quat_inverse(q_inav_at_nn), q_xiao)
        if rms_conj <= rms_direct:
            a['quat_rms_diff'] = rms_conj
            a['quat_inav_conv'] = 'world→body'
            label = f"world→body (RMS={rms_conj:.3f}; direct {rms_direct:.3f})"
        else:
            a['quat_rms_diff'] = rms_direct
            a['quat_inav_conv'] = 'body→world'
            label = f"body→world (RMS={rms_direct:.3f}; conj {rms_conj:.3f})"
        rms = a['quat_rms_diff']
        # Threshold tuned for nearest-neighbour matching: xiao samples at 100ms,
        # INAV at 2ms.  During fast maneuvers, even sign-correct quats drift
        # ~0.1–0.3 in RMS purely from the sample-time mismatch.  Real
        # convention divergence shows up at >0.5 (and the OTHER-convention
        # comparison would be tighter — already shown in the [direct/conj]
        # tuple above).
        if rms < 0.10:
            verdict = "✓ tight"
        elif rms < 0.40:
            verdict = "~ ok (sample-time slop)"
        else:
            verdict = "✗ DIVERGENT"
        lines.append(f"B. QUAT xiao↔INAV consistency: INAV blackbox = {label}  {verdict}")

    # C0) TARGET ACQUISITION  (control→response check that matters most)
    # If xiao's NN is doing its job — given target direction cosines, commanding
    # rates that pull the nose toward the target — then tX[3] ("now" slot, body
    # forward component of target unit vector) should trend TOWARD +1 over the
    # span. If it trends away, the NN is actively driving the aircraft away
    # from the target even though it knows where the target is.
    if nn is not None and len(nn['tXYZ_now']) > 5:
        tX = nn['tXYZ_now'][:, 0]
        tY = nn['tXYZ_now'][:, 1]
        tZ = nn['tXYZ_now'][:, 2]
        t_nn = nn['inav_ms']
        dt_s = np.diff(t_nn) / 1000.0
        dtX_dt = np.diff(tX) / np.maximum(dt_s, 1e-3)
        dtY_dt = np.diff(tY) / np.maximum(dt_s, 1e-3)
        dtZ_dt = np.diff(tZ) / np.maximum(dt_s, 1e-3)
        a['mean_dtX_dt'] = float(np.mean(dtX_dt))
        a['mean_dtY_dt'] = float(np.mean(dtY_dt))
        a['mean_dtZ_dt'] = float(np.mean(dtZ_dt))
        a['mean_tX'] = float(np.mean(tX))
        a['mean_dist'] = float(np.mean(nn['dist_now']))
        # Correlate NN commands with their intended effects:
        # pitch_up (+) should produce +dtZ (target Z moves toward 0/+) when target is above (-Z);
        # more generally, positive out_pitch should correlate with positive d(tZ)/dt if
        # physics and training conventions agree.
        out_pitch = nn['out'][:-1, 0]
        out_roll = nn['out'][:-1, 1]
        a['corr_outpitch_dtZ'], _ = max_corr_at_lag(out_pitch, dtZ_dt, range(0, 4))
        a['corr_outroll_dtY'], _ = max_corr_at_lag(out_roll, dtY_dt, range(0, 4))
        lines.append("C0. TARGET ACQUISITION (the core question):")
        lines.append(f"     mean tX  = {a['mean_tX']:+.3f}   (closer to +1 = target directly ahead)")
        # dtX/dt threshold: noise floor is ~0.01/s over a 20s span, so
        # anything in [-0.03, +0.03] is "neutral tracking" not a real signal.
        if a['mean_dtX_dt'] > 0.03:
            dt_msg = "✓ target coming toward nose"
        elif a['mean_dtX_dt'] < -0.03:
            dt_msg = "✗ TARGET MOVING AWAY from nose"
        else:
            dt_msg = "~ steady (rabbit-chase equilibrium)"
        lines.append(f"     mean dtX/dt = {a['mean_dtX_dt']:+.3f} /s   {dt_msg}")
        lines.append(f"     mean dist  = {a['mean_dist']:+.1f} m")
        lines.append(f"     NN pitch → dtZ/dt  corr = {a['corr_outpitch_dtZ']:+.2f}  "
                     + ("(NN pitch drives tZ in expected direction)" if a['corr_outpitch_dtZ'] > 0.2
                        else ""))
        lines.append(f"     NN roll  → dtY/dt  corr = {a['corr_outroll_dtY']:+.2f}  "
                     + ("(NN roll drives tY in expected direction)" if a['corr_outroll_dtY'] > 0.2
                        else ""))

    # C) FORWARD FLIGHT  (empirically pin down INAV quat convention)
    # Try both: INAV quat as body→world (direct) AND as world→body (conjugate).
    # Whichever yields cos(nose, vel) closer to +1 in sustained flight is the
    # true convention for this log. Also run the test with xiao's stored quat
    # (which msplink.cpp already normalized to body→world).
    if len(inav_slc['quat']) and len(inav_slc['navVel']):
        body_fwd = np.tile([1.0, 0, 0], (len(inav_slc['quat']), 1))
        nose_direct = quat_rotate_v(inav_slc['quat'], body_fwd)
        nose_conj = quat_rotate_v(quat_inverse(inav_slc['quat']), body_fwd)
        vel = inav_slc['navVel']
        vel_norm = np.linalg.norm(vel, axis=1)
        ok = vel_norm > 1.0
        if np.any(ok):
            vel_u = vel[ok] / vel_norm[ok, None]
            cos_direct = float(np.mean(np.sum(nose_direct[ok] * vel_u, axis=1)))
            cos_conj = float(np.mean(np.sum(nose_conj[ok] * vel_u, axis=1)))
            a['cos_nv_inav_direct'] = cos_direct
            a['cos_nv_inav_conj'] = cos_conj
            lines.append(f"C. FORWARD FLIGHT  cos(nose, vel):")
            lines.append(f"     INAV quat as body→world : {cos_direct:+.3f}")
            lines.append(f"     INAV quat as world→body : {cos_conj:+.3f}")
            better = "body→world" if abs(cos_direct - 1) < abs(cos_conj - 1) else "world→body"
            lines.append(f"     → INAV blackbox convention: {better}")
            a['cos_nose_vel_mean'] = max(cos_direct, cos_conj)
    # Also check with xiao's own logged quat against INAV velocity.
    if nn is not None and len(nn['q']) and len(inav_slc['navVel']):
        t_nn = nn['inav_ms']
        vel_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['navVel'], t_nn)
        vn = np.linalg.norm(vel_at_nn, axis=1)
        ok = vn > 1.0
        if np.any(ok):
            body_fwd = np.tile([1.0, 0, 0], (len(nn['q']), 1))
            nose_xiao = quat_rotate_v(nn['q'], body_fwd)
            vu = vel_at_nn[ok] / vn[ok, None]
            cos_xiao = float(np.mean(np.sum(nose_xiao[ok] * vu, axis=1)))
            a['cos_nv_xiao'] = cos_xiao
            lines.append(f"     xiao quat (body→world)  : {cos_xiao:+.3f}  "
                         + ("✓ forward" if cos_xiao > 0.85 else "✗ off"))

    # D) CONTROL CHAIN
    # Build conventions on this airframe:
    #   - msplink convertPitchToMSPChannel intentionally inverts pitch
    #     (NN +1 → MSP 1000 μs) "to match CRRCSim".  So NN→rc pitch = -.
    #   - INAV elevon mix has servo0/servo1 inverted relative to rcData
    #     (observed every flight on this build).  So rcData→servo* = -.
    #   - Everything else is direct (+).  End-to-end physics rcData→gyro
    #     must be +; that's the only assertion that pins reality.
    lines.append("D. CONTROL CHAIN (per-stage expected polarity in [brackets]):")
    if nn is not None and len(nn['out']):
        # NN output convention (from msplink.cpp): out = [pitch, roll, throttle], tanh ∈ [-1, +1].
        nn_pitch = nn['out'][:, 0]
        nn_roll = nn['out'][:, 1]
        nn_thr = nn['out'][:, 2]
        # rc[] at NN: [roll, pitch, throttle] in μs (MSP channels).
        nn_rc_roll = nn['rc'][:, 0]
        nn_rc_pitch = nn['rc'][:, 1]
        nn_rc_thr = nn['rc'][:, 2]
        # Lag range in NN samples: NN runs ~20 Hz (50ms tick), INAV 500 Hz.
        # Short lag (0..3 samples of NN) ≈ 0..150ms.  Use NN sample time base.

        # NN out → NN rc (this is what msplink converted and sent) — pure function test
        a['corr_nn_pitch_rc'], _ = max_corr_at_lag(nn_pitch, nn_rc_pitch, range(0, 2))
        a['corr_nn_roll_rc'], _ = max_corr_at_lag(nn_roll, nn_rc_roll, range(0, 2))
        a['corr_nn_thr_rc'], _ = max_corr_at_lag(nn_thr, nn_rc_thr, range(0, 2))
        lines.append(f"   NN out  → MSP rc : pitch={a['corr_nn_pitch_rc']:+.2f}[-]  "
                     f"roll={a['corr_nn_roll_rc']:+.2f}[+]  thr={a['corr_nn_thr_rc']:+.2f}[+]"
                     "   [msplink convertXToMSP: pitch inverted]")

        # NN rc → INAV rcData (what INAV received via MSP override) — transport test
        rcData_pitch_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['rcData'][:, 1], nn['inav_ms'])
        rcData_roll_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['rcData'][:, 0], nn['inav_ms'])
        rcData_thr_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['rcData'][:, 2], nn['inav_ms'])
        a['corr_rc_rcData_pitch'], _ = max_corr_at_lag(nn_rc_pitch, rcData_pitch_at_nn, range(0, 2))
        a['corr_rc_rcData_roll'], _ = max_corr_at_lag(nn_rc_roll, rcData_roll_at_nn, range(0, 2))
        a['corr_rc_rcData_thr'], _ = max_corr_at_lag(nn_rc_thr, rcData_thr_at_nn, range(0, 2))
        lines.append(f"   MSP rc  → rcData : pitch={a['corr_rc_rcData_pitch']:+.2f}[+]  "
                     f"roll={a['corr_rc_rcData_roll']:+.2f}[+]  thr={a['corr_rc_rcData_thr']:+.2f}[+]"
                     "   [MSP transport — all direct]")

        # INAV rcData → servo/motor — INAV mix test
        servo0_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['servo'][:, 0], nn['inav_ms'])
        servo1_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['servo'][:, 1], nn['inav_ms'])
        motor_at_nn = nearest_rows(inav_slc['t_ms'], inav_slc['motor'], nn['inav_ms'])
        a['corr_rcData_servo0'], _ = max_corr_at_lag(rcData_roll_at_nn, servo0_at_nn, range(0, 3))
        a['corr_rcData_servo1'], _ = max_corr_at_lag(rcData_pitch_at_nn, servo1_at_nn, range(0, 3))
        a['corr_rcData_motor'], _ = max_corr_at_lag(rcData_thr_at_nn, motor_at_nn, range(0, 3))
        lines.append(f"   rcData  → servo  : servo0(roll←rcData[0])={a['corr_rcData_servo0']:+.2f}[-]  "
                     f"servo1(pitch←rcData[1])={a['corr_rcData_servo1']:+.2f}[-]  "
                     f"motor(thr←rcData[2])={a['corr_rcData_motor']:+.2f}[+]"
                     "  [INAV elevon mix inverts both servos]")

        # Final physics: rcData → body gyro rate (lag 50–300ms = ~25-150 INAV samples).
        # Use INAV's own samples for this since INAV is at 500 Hz.
        gyro_p = inav_slc['gyro'][:, 0]
        gyro_q = inav_slc['gyro'][:, 1]
        a['corr_rcData_p'], lag_p = max_corr_at_lag(inav_slc['rcData'][:, 0], gyro_p, range(0, 200))
        a['corr_rcData_q'], lag_q = max_corr_at_lag(inav_slc['rcData'][:, 1], gyro_q, range(0, 200))
        lines.append(f"   rcData  → gyro   : rcData[0]→p={a['corr_rcData_p']:+.2f}[+]@lag{lag_p*2:.0f}ms  "
                     f"rcData[1]→q={a['corr_rcData_q']:+.2f}[+]@lag{lag_q*2:.0f}ms"
                     "  [end-to-end physics — must be +]")

    # E) POLICY
    if nn is not None and len(nn['out']):
        mean_pitch_cmd = float(np.mean(nn['out'][:, 0]))
        mean_roll_cmd = float(np.mean(nn['out'][:, 1]))
        mean_thr_cmd = float(np.mean(nn['out'][:, 2]))
        a['mean_cmd'] = (mean_pitch_cmd, mean_roll_cmd, mean_thr_cmd)
        lines.append(f"E. POLICY  mean NN cmd  pitch={mean_pitch_cmd:+.2f}  "
                     f"roll={mean_roll_cmd:+.2f}  thr={mean_thr_cmd:+.2f}  (each in [-1,+1])")
    if len(inav_slc['quat']):
        _, pitch, _ = quat_to_euler(inav_slc['quat'])
        a['mean_pitch_deg'] = float(np.degrees(np.mean(pitch)))
        lines.append(f"   mean pitch attitude = {a['mean_pitch_deg']:+.1f}°")
    if len(inav_slc['navVel']):
        climb = -inav_slc['navVel'][:, 2]  # NED Z=down → climb = -Z vel
        a['mean_climb'] = float(np.mean(climb))
        lines.append(f"   mean climb rate = {a['mean_climb']:+.2f} m/s  "
                     f"(+ up, - down)")
    return a, "\n".join(lines)


def nearest_rows(t_src, values_src, t_query):
    """For each t in t_query (ms), return values_src at nearest t_src index."""
    idx = np.searchsorted(t_src, t_query)
    idx = np.clip(idx, 1, len(t_src) - 1)
    # pick closer of idx-1 or idx
    left = t_src[idx - 1]; right = t_src[idx]
    chose_left = (t_query - left) < (right - t_query)
    idx_final = np.where(chose_left, idx - 1, idx)
    return values_src[idx_final]


# ---------- plotting ----------

def plot_span(inav_slc, nn, nav_rows, anchors, out_path, title):
    fig = plt.figure(figsize=(17, 13))
    gs = fig.add_gridspec(4, 3, hspace=0.45, wspace=0.30)

    t0 = inav_slc['t_ms'][0] if len(inav_slc['t_ms']) else 0.0
    t = (inav_slc['t_ms'] - t0) / 1000.0  # seconds since span start

    # Panel 1: top-down XY (NED: X=North, Y=East; show East horizontal, North vertical)
    ax = fig.add_subplot(gs[0:2, 0])
    ax.plot(inav_slc['navPos'][:, 1], inav_slc['navPos'][:, 0], 'b-', lw=1.5, label='aircraft', alpha=0.8)
    ax.plot(inav_slc['navTgtPos'][:, 1], inav_slc['navTgtPos'][:, 0], 'g--', lw=1.2, label='navTgtPos', alpha=0.7)
    # Nose direction (red) and velocity direction (cyan) arrows every ~0.5s
    step = max(1, len(inav_slc['quat']) // 30)
    nose_w = quat_rotate_v(inav_slc['quat'], np.tile([1.0, 0, 0], (len(inav_slc['quat']), 1)))
    for i in range(0, len(inav_slc['quat']), step):
        n = inav_slc['navPos'][i]
        nd = nose_w[i]
        ax.arrow(n[1], n[0], 2.5 * nd[1], 2.5 * nd[0],
                 head_width=0.6, fc='red', ec='red', alpha=0.6)
        v = inav_slc['navVel'][i]
        vn = np.linalg.norm(v[:2]) + 1e-9
        ax.arrow(n[1], n[0], 2.5 * v[1]/vn, 2.5 * v[0]/vn,
                 head_width=0.6, fc='cyan', ec='cyan', alpha=0.6)
    if len(inav_slc['navPos']):
        ax.plot(inav_slc['navPos'][0, 1], inav_slc['navPos'][0, 0], 'bo', ms=10, label='start')
        ax.plot(inav_slc['navPos'][-1, 1], inav_slc['navPos'][-1, 0], 'b^', ms=10, label='end')
    ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
    ax.set_title('Top-down NED  red=nose  cyan=vel\n(red and cyan should overlap if flying forward)')
    ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(alpha=0.3)

    # Panel 2: altitude + pitch attitude
    ax = fig.add_subplot(gs[0, 1:])
    alt = -inav_slc['navPos'][:, 2]  # NED Z=down → altitude = -Z
    ax.plot(t, alt, 'b-', lw=1.5, label='aircraft altitude')
    ax.plot(t, -inav_slc['navTgtPos'][:, 2], 'g--', lw=1.2, label='navTgt altitude', alpha=0.7)
    ax.set_xlabel('time (s)'); ax.set_ylabel('altitude (m, + up)')
    ax.grid(alpha=0.3); ax.legend(fontsize=8, loc='upper left')
    axp = ax.twinx()
    _, pitch, _ = quat_to_euler(inav_slc['quat'])
    axp.plot(t, np.degrees(pitch), 'r-', lw=1.0, label='pitch attitude', alpha=0.8)
    if nn is not None and len(nn['out']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        axp.plot(t_nn, nn['out'][:, 0] * 45.0, 'm--', lw=1.0, label='NN pitch cmd ×45°', alpha=0.8)
    axp.set_ylabel('pitch (deg)  / NN cmd (×45°)', color='r')
    axp.legend(fontsize=8, loc='upper right')
    axp.axhline(0, color='gray', lw=0.5, alpha=0.5)
    ax.set_title('Altitude & pitch  (NN +pitch cmd should raise altitude)')

    # Panel 3: roll polarity
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t, inav_slc['rcData'][:, 0], 'k-', lw=0.8, label='rcData[0]', alpha=0.6)
    if nn is not None and len(nn['rc']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        ax.plot(t_nn, nn['rc'][:, 0], 'b.', ms=4, label='nn rc roll')
    ax.set_ylabel('μs (RC)', color='k'); ax.set_xlabel('time (s)')
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc='upper left')
    axg = ax.twinx()
    axg.plot(t, inav_slc['gyro'][:, 0], 'r-', lw=0.8, label='gyro p', alpha=0.6)
    axg.set_ylabel('gyro p (raw)', color='r')
    axg.legend(fontsize=7, loc='upper right')
    axg.axhline(0, color='gray', lw=0.5)
    ax.set_title(f"roll:  NN→rc={anchors.get('corr_nn_roll_rc', float('nan')):+.2f}  "
                 f"rc→rcData={anchors.get('corr_rc_rcData_roll', float('nan')):+.2f}  "
                 f"rcData→p={anchors.get('corr_rcData_p', float('nan')):+.2f}")

    # Panel 4: pitch polarity
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(t, inav_slc['rcData'][:, 1], 'k-', lw=0.8, label='rcData[1]', alpha=0.6)
    if nn is not None and len(nn['rc']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        ax.plot(t_nn, nn['rc'][:, 1], 'b.', ms=4, label='nn rc pitch')
    ax.set_ylabel('μs (RC)', color='k'); ax.set_xlabel('time (s)')
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc='upper left')
    axg = ax.twinx()
    axg.plot(t, inav_slc['gyro'][:, 1], 'r-', lw=0.8, label='gyro q', alpha=0.6)
    axg.set_ylabel('gyro q (raw)', color='r')
    axg.legend(fontsize=7, loc='upper right')
    axg.axhline(0, color='gray', lw=0.5)
    ax.set_title(f"pitch:  NN→rc={anchors.get('corr_nn_pitch_rc', float('nan')):+.2f}  "
                 f"rc→rcData={anchors.get('corr_rc_rcData_pitch', float('nan')):+.2f}  "
                 f"rcData→q={anchors.get('corr_rcData_q', float('nan')):+.2f}")

    # Panel 5: throttle + servo traces
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, inav_slc['motor'], 'k-', lw=1.0, label='motor[0]')
    ax.plot(t, inav_slc['rcData'][:, 2], 'b-', lw=0.6, label='rcData[2] (thr)', alpha=0.5)
    if nn is not None and len(nn['out']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        axo = ax.twinx()
        axo.plot(t_nn, nn['out'][:, 2], 'm.', ms=4, label='NN thr cmd')
        axo.set_ylabel('NN out', color='m')
        axo.set_ylim(-1.1, 1.1)
        axo.legend(fontsize=7, loc='upper right')
    ax.set_xlabel('time (s)'); ax.set_ylabel('PWM (μs)')
    ax.set_title(f"throttle:  NN→rc={anchors.get('corr_nn_thr_rc', float('nan')):+.2f}  "
                 f"rcData→motor={anchors.get('corr_rcData_motor', float('nan')):+.2f}")
    ax.legend(fontsize=7, loc='upper left'); ax.grid(alpha=0.3)

    # Panel 6: servos
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(t, inav_slc['servo'][:, 0], 'b-', lw=0.8, label='servo[0]')
    ax.plot(t, inav_slc['servo'][:, 1], 'r-', lw=0.8, label='servo[1]')
    ax.set_xlabel('time (s)'); ax.set_ylabel('servo PWM')
    ax.set_title(f"servos  rcData→servo0={anchors.get('corr_rcData_servo0', float('nan')):+.2f}  "
                 f"→servo1={anchors.get('corr_rcData_servo1', float('nan')):+.2f}")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # Panel 7: body-frame accel (gravity anchor)
    ax = fig.add_subplot(gs[2, 2])
    ax.plot(t, inav_slc['acc'][:, 0], label='acc X', lw=0.7)
    ax.plot(t, inav_slc['acc'][:, 1], label='acc Y', lw=0.7)
    ax.plot(t, inav_slc['acc'][:, 2], label='acc Z (FRD: + if gravity→body+Z)', lw=1.2)
    ax.axhline(0, color='gray', lw=0.5)
    ax.set_xlabel('time (s)'); ax.set_ylabel('accSmooth (raw)')
    ax.set_title(f"accel body-frame  mean Z={anchors.get('acc_mean', [0,0,0])[2]:+.0f}")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # Panel 8: NN inputs sanity — direction cosines now
    ax = fig.add_subplot(gs[3, 0])
    if nn is not None and len(nn['tXYZ_now']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        ax.plot(t_nn, nn['tXYZ_now'][:, 0], label='tX (fwd=+1)', lw=1.0)
        ax.plot(t_nn, nn['tXYZ_now'][:, 1], label='tY (right=+1)', lw=1.0)
        ax.plot(t_nn, nn['tXYZ_now'][:, 2], label='tZ (down=+1)', lw=1.0)
        ax.set_ylim(-1.2, 1.2); ax.axhline(0, color='gray', lw=0.5)
    ax.set_xlabel('time (s)'); ax.set_ylabel('dir cosine (body)')
    ax.set_title('NN input: target direction cosines (body FRD)')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # Panel 9: NN outputs time series
    ax = fig.add_subplot(gs[3, 1])
    if nn is not None and len(nn['out']):
        t_nn = (nn['inav_ms'] - t0) / 1000.0
        ax.plot(t_nn, nn['out'][:, 0], 'r-', label='pitch', lw=1.2)
        ax.plot(t_nn, nn['out'][:, 1], 'b-', label='roll', lw=1.2)
        ax.plot(t_nn, nn['out'][:, 2], 'g-', label='throttle', lw=1.2)
        ax.set_ylim(-1.2, 1.2); ax.axhline(0, color='gray', lw=0.5)
    ax.set_xlabel('time (s)'); ax.set_ylabel('NN output (tanh)')
    ax.set_title('NN commanded outputs')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # Panel 10: anchor verdict text
    ax = fig.add_subplot(gs[3, 2])
    ax.axis('off')
    verdict = anchor_verdict(anchors)
    ax.text(0, 1, verdict, family='monospace', fontsize=8, va='top', ha='left')

    fig.suptitle(title, fontsize=13)
    plt.savefig(out_path, dpi=120, bbox_inches='tight')
    plt.close()


def anchor_verdict(a):
    """Compact verdict based on anchor values."""
    lines = []
    # Gravity
    if 'acc_mean' in a:
        lines.append(f"GRAVITY Z: {a['acc_mean'][2]:+.0f}  "
                     + ("✓ FRD" if a['acc_mean'][2] > 0 else "✗ FLU"))
    if 'quat_rms_diff' in a:
        lines.append(f"QUAT diff: {a['quat_rms_diff']:.3f}  "
                     + ("✓" if a['quat_rms_diff'] < 0.05 else "✗"))
    if 'cos_nose_vel_mean' in a:
        lines.append(f"cos(nose,vel): {a['cos_nose_vel_mean']:+.2f}  "
                     + ("✓ fwd" if a['cos_nose_vel_mean'] > 0.5 else "✗"))
    lines.append("")
    # Per-stage expected polarity for this build (see compute_anchors D notes).
    # ✓ if observed corr matches expected sign with |corr|>0.2.
    lines.append("CONTROL CHAIN (✓ = matches build's expected polarity)")
    for label, key, exp in [
        ('NN→rc pitch',      'corr_nn_pitch_rc',     -1),
        ('NN→rc roll',       'corr_nn_roll_rc',      +1),
        ('NN→rc thr',        'corr_nn_thr_rc',       +1),
        ('rc→rcData pitch',  'corr_rc_rcData_pitch', +1),
        ('rc→rcData roll',   'corr_rc_rcData_roll',  +1),
        ('rc→rcData thr',    'corr_rc_rcData_thr',   +1),
        ('rcData→servo0',    'corr_rcData_servo0',   -1),
        ('rcData→servo1',    'corr_rcData_servo1',   -1),
        ('rcData→motor',     'corr_rcData_motor',    +1),
        ('rcData→gyro p',    'corr_rcData_p',        +1),
        ('rcData→gyro q',    'corr_rcData_q',        +1),
    ]:
        v = a.get(key, float('nan'))
        if abs(v) < 0.2:
            mark = "?"
        elif (v > 0) == (exp > 0):
            mark = "✓"
        else:
            mark = "✗"
        lines.append(f"  {label:<18s} {v:+.2f} [{'+' if exp>0 else '-'}] {mark}")
    lines.append("")
    if 'mean_cmd' in a:
        p, r, th = a['mean_cmd']
        lines.append(f"mean NN cmd: p={p:+.2f} r={r:+.2f} th={th:+.2f}")
    if 'mean_pitch_deg' in a:
        lines.append(f"mean pitch attitude: {a['mean_pitch_deg']:+.1f}°")
    if 'mean_climb' in a:
        lines.append(f"mean climb: {a['mean_climb']:+.2f} m/s")
    return "\n".join(lines)


# ---------- CSV export ----------

def export_span_csv(inav_slc, nn, out_path):
    """Per-span joined CSV for custom analysis."""
    with open(out_path, 'w') as f:
        cols = [
            't_ms', 'msp', 'motor', 'servo0', 'servo1',
            'gyro_p', 'gyro_q', 'gyro_r', 'acc_x', 'acc_y', 'acc_z',
            'qw', 'qx', 'qy', 'qz',
            'pos_n', 'pos_e', 'pos_d', 'vel_n', 'vel_e', 'vel_d',
            'tgtpos_n', 'tgtpos_e', 'tgtpos_d',
            'rcData0', 'rcData1', 'rcData2', 'rcData3',
        ]
        f.write(','.join(cols) + '\n')
        for i in range(len(inav_slc['t_ms'])):
            row = [
                inav_slc['t_ms'][i], inav_slc['msp'][i], inav_slc['motor'][i],
                inav_slc['servo'][i, 0], inav_slc['servo'][i, 1],
                *inav_slc['gyro'][i], *inav_slc['acc'][i], *inav_slc['quat'][i],
                *inav_slc['navPos'][i], *inav_slc['navVel'][i],
                *inav_slc['navTgtPos'][i],
                *inav_slc['rcData'][i],
            ]
            f.write(','.join(f"{x:.4f}" for x in row) + '\n')


# ---------- main ----------

def main():
    p = argparse.ArgumentParser()
    p.add_argument('inav_csv')
    p.add_argument('xiao_log')
    p.add_argument('--out-dir', default=None,
                   help='output dir (default: next to xiao_log)')
    p.add_argument('--engage-flags', default='2,3',
                   help='mspOverrideFlags values that count as engage (default 2,3)')
    args = p.parse_args()

    out_dir = Path(args.out_dir) if args.out_dir else Path(args.xiao_log).parent
    out_dir.mkdir(exist_ok=True)
    stem = Path(args.xiao_log).stem

    print(f"Parsing INAV CSV {args.inav_csv}...")
    inav = parse_inav_csv(args.inav_csv)
    print(f"  {len(inav['t_ms'])} rows, {inav['t_ms'][0]:.1f} → {inav['t_ms'][-1]:.1f} ms")

    print(f"Parsing xiao log {args.xiao_log}...")
    nav_rows, nn_rows = parse_xiao_log(args.xiao_log)
    print(f"  {len(nav_rows)} Nav State, {len(nn_rows)} NN lines")

    flags = tuple(int(x) for x in args.engage_flags.split(','))
    spans = detect_engage_spans(inav['t_ms'], inav['msp'], flag_values=flags)
    print(f"Engage spans (mspOverrideFlags in {flags}): {len(spans)}")
    for s, e in spans:
        print(f"  {s/1000.0:.2f}s → {e/1000.0:.2f}s  ({(e-s)/1000.0:.2f}s)")

    for i, (s, e) in enumerate(spans):
        inav_slc = slice_inav_to_span(inav, s, e)
        nn_rows_span, nav_rows_span = slice_xiao_to_span(nn_rows, nav_rows, s, e)
        nn = nn_arrays(nn_rows_span)
        path_id = nav_rows_span[0]['path'] if nav_rows_span else -1
        title = f"Span {i+1}  path={path_id}  {s/1000.0:.1f}s→{e/1000.0:.1f}s  ({(e-s)/1000.0:.2f}s)"
        print(f"\n{'='*78}\n{title}\n{'='*78}")
        anchors, report = compute_anchors(inav_slc, nn)
        print(report)
        out_png = out_dir / f"join_analysis_{stem}_span{i+1}_path{path_id}.png"
        plot_span(inav_slc, nn, nav_rows_span, anchors, str(out_png), title)
        print(f"  plot → {out_png}")
        out_csv = out_dir / f"join_analysis_{stem}_span{i+1}_path{path_id}.csv"
        export_span_csv(inav_slc, nn, str(out_csv))
        print(f"  csv  → {out_csv}")


if __name__ == '__main__':
    main()
