#!/usr/bin/env python3
# 043 T005 — verify the T004 041-t7 extract INDEPENDENTLY of any dmp loader.
#
# Reads only the per-tick CSV (no cereal, no dmp). Reproduces:
#   1. row counts / scenario count
#   2. per-path airframe rotation rate (deg/s), the shared-code method in
#      tools/dmp_dump.cc::computePerPathRates — must match the preserved
#      run-summary row (t7-gen800-runsummary.csv)
#   3. 3-5 Hz / 5-10 Hz roll/pitch-rate band-power fractions (engaged-only)
#
# Method mirrors dmp_dump.cc exactly: quat->(roll,pitch) aerospace ZYX,
# droll = sum|unwrap(dRoll)|, dpitch = sum|dPitch|, dur = n_ticks * dt,
# dt = SIM_TIME_STEP_MSEC/1000 = 0.05 s (20 Hz). path = scenario // 49.
import sys, math
import numpy as np

CSV = sys.argv[1] if len(sys.argv) > 1 else "t7-gen800-per-tick.csv"
DT = 0.050            # SIM_TIME_STEP_MSEC/1000
WINDS = 49            # 6 paths x 49 winds = 294
NYQ = 0.5 / DT        # 10 Hz

# Column positions (CSV has duplicate 'qw' names, so index by position).
# header: scenario,tick,px,py,pz,qw,qx,qy,qz,vx,...
C_SCEN, C_TICK, C_QW, C_QX, C_QY, C_QZ = 0, 1, 5, 6, 7, 8

rows = np.genfromtxt(CSV, delimiter=",", skip_header=1, usecols=(C_SCEN, C_TICK, C_QW, C_QX, C_QY, C_QZ))
scen = rows[:, 0].astype(int)
qw, qx, qy, qz = rows[:, 2], rows[:, 3], rows[:, 4], rows[:, 5]

def quat_to_rp(qw, qx, qy, qz):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    roll = np.degrees(np.arctan2(sinr, cosr))
    pitch = np.degrees(np.arcsin(sinp))
    return roll, pitch

def unwrap(d):
    d = np.where(d > 180.0, d - 360.0, d)
    d = np.where(d <= -180.0, d + 360.0, d)
    return d

roll_all, pitch_all = quat_to_rp(qw, qx, qy, qz)

scen_ids = np.unique(scen)
n_scen = len(scen_ids)
n_rows = len(scen)

# --- per-path rotation rate (shared-code method) ---
sr = np.zeros(6); sp = np.zeros(6); npc = np.zeros(6, int)
# band-power accumulators (pooled over engaged ticks per band, power-weighted)
band_roll = {"3-5": 0.0, "5-10": 0.0, "tot": 0.0}
band_pitch = {"3-5": 0.0, "5-10": 0.0, "tot": 0.0}

def bandpower(series):
    x = np.asarray(series, float)
    if len(x) < 8:
        return 0.0, 0.0, 0.0
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    P = (np.abs(X) ** 2)
    f = np.fft.rfftfreq(len(x), d=DT)
    tot = P.sum()
    b35 = P[(f >= 3.0) & (f < 5.0)].sum()
    b510 = P[(f >= 5.0) & (f <= 10.0)].sum()
    return b35, b510, tot

for s in scen_ids:
    m = scen == s
    r = roll_all[m]; p = pitch_all[m]
    if len(r) < 2:
        continue
    path = int(s) // WINDS
    droll = np.abs(unwrap(np.diff(r))).sum()
    dpitch = np.abs(np.diff(p)).sum()
    dur = len(r) * DT
    if 0 <= path < 6:
        sr[path] += droll / dur; sp[path] += dpitch / dur; npc[path] += 1
    # rate time-series for band-power (deg/s)
    rr = unwrap(np.diff(r)) / DT
    pr = np.diff(p) / DT
    b35, b510, tot = bandpower(rr)
    band_roll["3-5"] += b35; band_roll["5-10"] += b510; band_roll["tot"] += tot
    b35, b510, tot = bandpower(pr)
    band_pitch["3-5"] += b35; band_pitch["5-10"] += b510; band_pitch["tot"] += tot

roll_rate = np.where(npc > 0, sr / np.maximum(npc, 1), 0.0)
pitch_rate = np.where(npc > 0, sp / np.maximum(npc, 1), 0.0)

print(f"rows={n_rows}  scenarios={n_scen}  (expected 294)")
print("per-path ROLL rate deg/s :", ", ".join(f"{v:.3f}" for v in roll_rate),
      f"| mean={roll_rate.mean():.3f}")
print("per-path PITCH rate deg/s:", ", ".join(f"{v:.3f}" for v in pitch_rate),
      f"| mean={pitch_rate.mean():.3f}")
print("scenarios per path       :", ", ".join(str(int(v)) for v in npc))
def frac(b):
    return (100.0 * b["3-5"] / b["tot"], 100.0 * b["5-10"] / b["tot"]) if b["tot"] > 0 else (0, 0)
r35, r510 = frac(band_roll)
p35, p510 = frac(band_pitch)
print(f"ROLL-rate band-power  : 3-5 Hz {r35:.1f}%   5-10 Hz {r510:.1f}%  (Nyquist {NYQ:.0f} Hz)")
print(f"PITCH-rate band-power : 3-5 Hz {p35:.1f}%   5-10 Hz {p510:.1f}%")
