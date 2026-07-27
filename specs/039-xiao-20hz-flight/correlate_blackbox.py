#!/usr/bin/env python3
"""039 — correlate a v3 xiao flight log against the INAV blackbox.

The v3 flight log carries INAV_CLOCK anchor events (EventRecord.timestamp_ms =
xiao clock ms, value = INAV clock ms sampled from MSP2_AUTOC_STATE). This
script:

  1. fits xiao_ms -> inav_ms as a linear map from the anchors (reports offset,
     drift ppm, residuals — residual scale ~ one 50 ms tick of sampling jitter);
  2. maps every FlightState breadcrumb + engaged TickRecord onto the blackbox
     timebase and interpolates the blackbox series there;
  3. compares position (NED m), velocity (NED m/s), and attitude (quat angle)
     between what the xiao recorded and what INAV logged.

Frames: breadcrumbs are raw INAV home-frame NED m; ticks are virtual
(engage-relative) NED and are re-anchored via their span's EngageHeader
origin. Blackbox navPos is home-frame NEU cm -> NED m; quaternion[] is INAV
convention (pitch nose-down+, yaw N->W+) and gets the same qy/qz flip the
firmware applies (include/autoc/imu/inav_quat_convention.h) before comparison.

Usage:
  correlate_blackbox.py flight.bin blackbox.01.csv [--plot out.png]
"""

import argparse
import csv
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DECODER = os.path.join(HERE, "..", "..", "src", "analytics", "flightlog_decode.py")

spec = importlib.util.spec_from_file_location("flightlog_decode", DECODER)
fld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fld)


def fit_clock(anchors):
    """Least-squares inav_ms = a * xiao_ms + b. Returns (a, b, residuals)."""
    n = len(anchors)
    if n < 2:
        sys.exit("need >=2 INAV_CLOCK anchors to fit the clock map")
    sx = sum(x for x, _ in anchors)
    sy = sum(y for _, y in anchors)
    sxx = sum(x * x for x, _ in anchors)
    sxy = sum(x * y for x, y in anchors)
    denom = n * sxx - sx * sx
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    resid = [y - (a * x + b) for x, y in anchors]
    return a, b, resid


def load_blackbox(path):
    """Return sorted list of (t_ms, pos_ned[3] m, vel_ned[3] m/s, quat_aero[4])."""
    rows = []
    with open(path) as f:
        reader = csv.reader(f)
        hdr = [h.strip() for h in next(reader)]
        idx = {name: i for i, name in enumerate(hdr)}

        def col(name):
            if name not in idx:
                sys.exit(f"blackbox CSV missing column '{name}'")
            return idx[name]

        c_t = col("time (us)")
        c_q = [col(f"quaternion[{i}]") for i in range(4)]
        c_p = [col(f"navPos[{i}]") for i in range(3)]
        c_v = [col(f"navVel[{i}]") for i in range(3)]
        for r in reader:
            try:
                t_ms = float(r[c_t]) / 1000.0
                q = [float(r[c]) for c in c_q]
                p = [float(r[c]) for c in c_p]
                v = [float(r[c]) for c in c_v]
            except (ValueError, IndexError):
                continue
            # navPos/navVel: NEU cm -> NED m. quaternion: INAV wxyz -> aerospace
            # q_EB via the firmware's static-lookup flip (qy, qz negated).
            pos = [p[0] / 100.0, p[1] / 100.0, -p[2] / 100.0]
            vel = [v[0] / 100.0, v[1] / 100.0, -v[2] / 100.0]
            norm = math.sqrt(sum(c * c for c in q)) or 1.0
            quat = [q[0] / norm, q[1] / norm, -q[2] / norm, -q[3] / norm]
            rows.append((t_ms, pos, vel, quat))
    rows.sort(key=lambda r: r[0])
    return rows


def interp(rows, t):
    """Linear interp of (pos, vel) and nearest-neighbor quat at blackbox time t.
    Returns None outside the logged range or across a gap > 500 ms."""
    lo, hi = 0, len(rows) - 1
    if t < rows[0][0] or t > rows[hi][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, p0, v0, q0 = rows[lo]
    t1, p1, v1, q1 = rows[hi]
    if t1 - t0 > 500.0:
        return None
    w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    pos = [p0[i] + w * (p1[i] - p0[i]) for i in range(3)]
    vel = [v0[i] + w * (v1[i] - v0[i]) for i in range(3)]
    quat = q0 if w < 0.5 else q1
    return pos, vel, quat


def quat_angle_deg(qa, qb):
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    d = min(1.0, d)
    return math.degrees(2.0 * math.acos(d))


def stats(vals):
    if not vals:
        return "n=0"
    s = sorted(vals)
    n = len(s)
    mean = sum(s) / n
    return (f"n={n} mean={mean:.3f} p50={s[n // 2]:.3f} "
            f"p95={s[int(n * 0.95)]:.3f} max={s[-1]:.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("flight_bin")
    ap.add_argument("blackbox_csv")
    ap.add_argument("--plot", metavar="PNG", help="write a comparison plot")
    args = ap.parse_args()

    with open(args.flight_bin, "rb") as f:
        blob = f.read()
    header, spans, events, warnings, flight_states = fld.decode(blob)
    print(f"flight log v{header['format_version']}  program: {header['program']}")

    anchors = [(ev["timestamp_ms"], ev["value"]) for ev in events
               if ev["code"] == 11]
    a, b, resid = fit_clock(anchors)
    drift_ppm = (a - 1.0) * 1e6
    print(f"clock map: inav_ms = {a:.6f} * xiao_ms + {b:.1f}  "
          f"(drift {drift_ppm:+.0f} ppm, {len(anchors)} anchors, "
          f"residuals ms: {['%.0f' % r for r in resid]})")

    bb = load_blackbox(args.blackbox_csv)
    print(f"blackbox: {len(bb)} rows, t = {bb[0][0]:.0f}..{bb[-1][0]:.0f} ms "
          f"({(bb[-1][0] - bb[0][0]) / 1000.0:.1f} s)")

    # Sample set: breadcrumbs are already raw-frame; ticks add their span origin.
    samples = [(fs["timestamp_ms"],
                [fs["pos_raw_n"], fs["pos_raw_e"], fs["pos_raw_d"]],
                [fs["vel_n"], fs["vel_e"], fs["vel_d"]],
                [fs["quat_w"], fs["quat_x"], fs["quat_y"], fs["quat_z"]],
                "crumb") for fs in flight_states]
    for s in spans:
        e = s["engage"]
        org = [e["origin_n"], e["origin_e"], e["origin_d"]]
        for r in s["ticks"]:
            samples.append((r["timestamp_ms"],
                            [r["pos_n"] + org[0], r["pos_e"] + org[1],
                             r["pos_d"] + org[2]],
                            [r["vel_n"], r["vel_e"], r["vel_d"]],
                            [r["quat_w"], r["quat_x"], r["quat_y"], r["quat_z"]],
                            "tick"))
    samples.sort(key=lambda s: s[0])

    pos_err, vel_err, att_err = [], [], []
    matched, skipped = 0, 0
    series = []  # (t_inav, xiao_pos, bb_pos) for plotting
    for t_x, pos, vel, quat, kind in samples:
        t_i = a * t_x + b
        hit = interp(bb, t_i)
        if hit is None:
            skipped += 1
            continue
        bpos, bvel, bquat = hit
        matched += 1
        pos_err.append(math.dist(pos, bpos))
        vel_err.append(math.dist(vel, bvel))
        att_err.append(quat_angle_deg(quat, bquat))
        series.append((t_i, pos, bpos))

    print(f"samples: {matched} matched, {skipped} outside blackbox coverage "
          f"({len(flight_states)} crumbs + {sum(len(s['ticks']) for s in spans)} ticks)")
    print(f"  pos err (m):      {stats(pos_err)}")
    print(f"  vel err (m/s):    {stats(vel_err)}")
    print(f"  attitude err (deg): {stats(att_err)}")

    if args.plot and series:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        t0 = series[0][0]
        ts = [(s[0] - t0) / 1000.0 for s in series]
        for ax, i, name in zip(axes, range(3), ["North", "East", "Down"]):
            ax.plot(ts, [s[1][i] for s in series], ".", ms=2, label="xiao flight log")
            ax.plot(ts, [s[2][i] for s in series], ".", ms=2, label="INAV blackbox")
            ax.set_ylabel(f"{name} (m)")
            ax.grid(True, alpha=0.3)
        axes[0].legend(loc="upper right")
        axes[0].set_title("xiao flight log vs INAV blackbox — home-frame NED position")
        axes[2].set_xlabel("time since first matched sample (s)")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"wrote {args.plot}")


if __name__ == "__main__":
    main()
