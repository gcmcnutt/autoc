#!/usr/bin/env python3
"""span_attitude.py — 045 research R3/R4/R7: per engaged span, gyro-only attitude from engage vs INAV attitude, rotation totals,
rate/load envelope, and a gyroRaw spectrum (research.md R3/R4/R7 tables). Run from the repo root: python3 specs/045-m2-tracking/span_attitude.py"""
import csv, sys
import numpy as np

SPANS = {
    "1108": ("flight-results/flight-20261108/blackbox_log_2026-11-08_160651.01.csv",
             [(373.07, 389.78), (406.18, 428.07), (455.55, 476.61), (493.05, 514.30)]),
    "1018": ("flight-results/flight-20261018/blackbox_log_2026-10-18_125223.01.csv",
             [(399.81, 406.08), (423.15, 439.69), (460.04, 482.18), (499.91, 521.45)]),
}
COLS = ["time (us)", "gyroADC[0]", "gyroADC[1]", "gyroADC[2]", "gyroRaw[0]", "gyroRaw[1]", "gyroRaw[2]",
        "accSmooth[0]", "accSmooth[1]", "accSmooth[2]", "attitude[0]", "attitude[1]", "attitude[2]"]


def load(path):
    r = csv.reader(open(path)); h = [c.strip() for c in next(r)]
    idx = [h.index(c) for c in COLS]
    rows = []
    for row in r:
        try: rows.append([float(row[i]) for i in idx])
        except (ValueError, IndexError): pass
    A = np.array(rows)
    return A[:, 0] / 1e6, A[:, 1:4], A[:, 4:7], A[:, 7:10] / 2048.0, np.radians(A[:, 10:13] / 10.0)


def euler_to_R(roll, pitch, yaw):  # FRD body -> NED, ZYX
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                     [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                     [-sp, sr * cp, cr * cp]])


def expm_so3(w):
    th = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-12: return np.eye(3) + K
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K


def ang(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def propagate(t, w, R0, i0, i1):  # body rates rad/s FRD, right-multiply
    R = R0.copy(); out = [R.copy()]
    for k in range(i0, i1):
        dt = t[k + 1] - t[k]
        R = R @ expm_so3(0.5 * (w[k] + w[k + 1]) * dt)
        out.append(R.copy())
    return out


def inav_R(att, k, rs, ps):
    return euler_to_R(rs * att[k, 0], ps * att[k, 1], att[k, 2])


def main():
    np.set_printoptions(precision=2, suppress=True)
    for key, (path, spans) in SPANS.items():
        t, gadc, graw, acc, att = load(path)
        # sign search on 1 s windows inside spans: euler (rs, ps) x gyro FLU->FRD signs
        best = None
        for rs in (1, -1):
            for ps in (1, -1):
                for sx in (1, -1):
                    for sy in (1, -1):
                        for sz in (1, -1):
                            w = np.radians(gadc) * np.array([sx, sy, sz])
                            errs = []
                            for (a, b) in spans:
                                for s0 in np.arange(a + 1, b - 1.5, 2.0):
                                    i0 = np.searchsorted(t, s0); i1 = np.searchsorted(t, s0 + 1.0)
                                    Rs = propagate(t, w, inav_R(att, i0, rs, ps), i0, i1)
                                    errs.append(ang(Rs[-1], inav_R(att, i1, rs, ps)))
                            e = np.median(errs)
                            if best is None or e < best[0]: best = (e, rs, ps, sx, sy, sz)
        e, rs, ps, sx, sy, sz = best
        print(f"== {key}: sign fit median 1 s gyro-vs-INAV error {e:.2f} deg  (rs {rs} ps {ps} gyro {sx},{sy},{sz})")
        sg = np.array([sx, sy, sz])
        for name, G in (("gyroADC", gadc), ("gyroRaw", graw)):
            w = np.radians(G) * sg
            for n, (a, b) in enumerate(spans, 1):
                i0, i1 = np.searchsorted(t, a), np.searchsorted(t, b)
                Rs = propagate(t, w, inav_R(att, i0, rs, ps), i0, i1)
                def at(sec):
                    j = min(np.searchsorted(t, a + sec), i1) - i0
                    return ang(Rs[j], inav_R(att, i0 + j, rs, ps))
                if name == "gyroADC":
                    seg = G[i0:i1] * sg; dt = np.diff(t[i0:i1 + 1])[:, None]
                    tot = np.sum(np.abs(seg) * dt, axis=0); net = np.sum(seg * dt, axis=0)
                    f = np.linalg.norm(acc[i0:i1], axis=1)
                    print(f"  span {n} {b - a:5.1f}s  |rot| r/p/y {tot.round(0)} deg  net {net.round(0)}  max|rate| {np.abs(seg).max(0).round(0)} dps"
                          f"  |f| mean {f.mean():.2f} g p95 {np.percentile(f, 95):.2f} >2g {np.mean(f > 2) * 100:.0f}%")
                print(f"    {name:8s} gyro-only vs INAV: 5 s {at(5):5.1f}  10 s {at(10):5.1f}  end {at(b - a):5.1f} deg")
        # spectrum of gyroRaw over engaged spans
        fs = 1 / np.median(np.diff(t))
        for ax in range(3):
            P = None
            for (a, b) in spans:
                i0, i1 = np.searchsorted(t, a), np.searchsorted(t, b)
                x = graw[i0:i1, ax]; n = 512
                for s in range(0, len(x) - n, n // 2):
                    seg = (x[s:s + n] - x[s:s + n].mean()) * np.hanning(n)
                    p = np.abs(np.fft.rfft(seg)) ** 2
                    P = p if P is None else P + p
            f = np.fft.rfftfreq(512, 1 / fs)
            m = f > 15
            top = np.argsort(P[m])[-4:][::-1]
            print(f"  gyroRaw axis {ax} spectrum (fs {fs:.0f} Hz) top peaks > 15 Hz: {f[m][top].round(1)} Hz")


if __name__ == "__main__":
    main()
