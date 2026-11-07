#!/usr/bin/env python3
"""044 T052 — join a v6 xiao flight log to the INAV blackbox and run the bench gates.

The 039 correlate_blackbox.py needs quaternion[] in the blackbox; the 043 flight procedure logs ATTI, not
QUAT, so this joins on the INAV_CLOCK anchors and checks what the bench cycle gates on (043 T069 / T123):

  - clock map inav_ms = a * xiao_ms + b from the anchors (refit per log; the ppm is per FC pairing)
  - every engaged tick matched to the nearest blackbox row (skew)
  - flight-mode flags at each tick (pure ACRO = ARM|MSPRCOVERRIDE; the non-pure ticks should be one
    contiguous run from tick 0, the engage priming window)
  - on pure-ACRO ticks: out_pitch -> axisRate[1], out_roll -> axisRate[0], rc_throttle -> rcCommand[3]
    (r, slope), yaw axisRate[2] == 0, and a lag scan of out_roll vs axisRate[0] (peak near 0 ms says the
    clock map is right)
  - position / velocity agreement, xiao (engage origin added back) vs navPos / navVel (NEU cm -> NED m)

Usage: bench_join.py <flight_NNN.bin> <blackbox .01.csv>
"""
import argparse
import bisect
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

INAV_CLOCK = 11  # flight-log event code
PURE_ACRO = "ARM|MSPRCOVERRIDE"
MATCH_MS = 30.0


def fit_clock(anchors):
    n = len(anchors)
    if n < 2:
        sys.exit("need >=2 INAV_CLOCK anchors to fit the clock map")
    sx = sum(x for x, _ in anchors)
    sy = sum(y for _, y in anchors)
    sxx = sum(x * x for x, _ in anchors)
    sxy = sum(x * y for x, y in anchors)
    a = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    b = (sy - a * sx) / n
    return a, b, [y - (a * x + b) for x, y in anchors]


def load_blackbox(path):
    with open(path) as f:
        reader = csv.reader(f)
        hdr = [h.strip() for h in next(reader)]
        idx = {name: i for i, name in enumerate(hdr)}
        need = ["time (us)", "flightModeFlags (flags)", "axisRate[0]", "axisRate[1]", "axisRate[2]",
                "rcCommand[0]", "rcCommand[1]", "rcCommand[3]",
                "navPos[0]", "navPos[1]", "navPos[2]", "navVel[0]", "navVel[1]", "navVel[2]"]
        for name in need:
            if name not in idx:
                sys.exit(f"blackbox CSV missing column '{name}'")
        rows = []
        for r in reader:
            try:
                row = {k: float(r[idx[k]]) for k in need if k != "flightModeFlags (flags)"}
            except (ValueError, IndexError):
                continue
            row["t_ms"] = row["time (us)"] / 1000.0
            row["flags"] = r[idx["flightModeFlags (flags)"]].strip()
            rows.append(row)
    rows.sort(key=lambda r: r["t_ms"])
    return rows


def pearson_slope(xs, ys):
    n = len(xs)
    if n < 3:
        return float("nan"), float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0 or syy == 0:
        return float("nan"), float("nan")
    return sxy / math.sqrt(sxx * syy), sxy / sxx


def pct(vals, q):
    s = sorted(vals)
    return s[min(len(s) - 1, int(len(s) * q))]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("flight_bin")
    ap.add_argument("blackbox_csv")
    args = ap.parse_args()

    with open(args.flight_bin, "rb") as f:
        header, spans, events, _warnings, _states = fld.decode(f.read())
    print(f"flight log v{header['format_version']}  program: {header['program']}")

    anchors = [(ev["timestamp_ms"], ev["value"]) for ev in events if ev["code"] == INAV_CLOCK]
    a, b, resid = fit_clock(anchors)
    print(f"clock map: inav_ms = {a:.6f} * xiao_ms + {b:.1f}  (drift {(a - 1) * 1e6:+.0f} ppm, "
          f"{len(anchors)} anchors, residuals ms {['%.1f' % r for r in resid]})")

    bb = load_blackbox(args.blackbox_csv)
    bt = [r["t_ms"] for r in bb]
    print(f"blackbox: {len(bb)} rows, inav t = {bt[0]:.0f}..{bt[-1]:.0f} ms")

    def nearest(t):
        i = bisect.bisect_left(bt, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(bb)]
        j = min(cands, key=lambda j: abs(bt[j] - t))
        return bb[j], bt[j] - t

    for s in spans:
        e = s["engage"]
        org = (e["origin_n"], e["origin_e"], e["origin_d"])
        ticks = s["ticks"]
        joined, skews = [], []
        for k, r in enumerate(ticks):
            t = a * r["timestamp_ms"] + b
            row, skew = nearest(t)
            if abs(skew) <= MATCH_MS:
                joined.append((k, r, row, t))
                skews.append(abs(skew))
        print(f"\nspan {s['span_id'] if 'span_id' in s else 1}: {len(ticks)} ticks, "
              f"{len(joined)} matched within {MATCH_MS:.0f} ms "
              f"(|skew| p50 {pct(skews, .5):.1f} ms, max {max(skews):.1f} ms)")

        counts = {}
        for _, _, row, _ in joined:
            counts[row["flags"]] = counts.get(row["flags"], 0) + 1
        print("flight-mode flags at the engaged ticks:")
        for fl, n in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f"  {fl:28s} {n:4d}  {100.0 * n / len(joined):5.1f} %")
        impure = [k for k, _, row, _ in joined if row["flags"] != PURE_ACRO]
        if impure:
            contiguous = impure == list(range(impure[0], impure[0] + len(impure)))
            t0 = ticks[impure[0]]["timestamp_ms"] - ticks[0]["timestamp_ms"]
            t1 = ticks[impure[-1]]["timestamp_ms"] - ticks[0]["timestamp_ms"]
            print(f"  non-pure ticks {impure[0]}..{impure[-1]} ({len(impure)}), "
                  f"{'contiguous' if contiguous else 'NOT contiguous'}, "
                  f"{t0 / 1000:.2f}-{t1 / 1000:.2f} s after engage")

        pure = [(r, row) for _, r, row, _ in joined if row["flags"] == PURE_ACRO]
        print(f"pure-ACRO ticks n={len(pure)}:")
        for xk, yk in (("out_pitch", "axisRate[1]"), ("out_roll", "axisRate[0]"),
                       ("rc_throttle", "rcCommand[3]"), ("rc_roll", "rcCommand[0]"),
                       ("rc_pitch", "rcCommand[1]")):
            xs = [r[xk] for r, _ in pure]
            ys = [row[yk] for _, row in pure]
            rr, sl = pearson_slope(xs, ys)
            sd = math.sqrt(sum((x - sum(xs) / len(xs)) ** 2 for x in xs) / len(xs))
            print(f"  {xk:11s} -> {yk:12s} r {rr:+.3f}  slope {sl:+9.2f}  (x sd {sd:.3f})")
        yaw = [abs(row["axisRate[2]"]) for _, row in pure]
        print(f"  yaw axisRate[2] max |.| = {max(yaw):.1f}")

        # Lag scan on the pass-through channels (PWM sent -> rcCommand): shift the xiao clock by L ms and
        # re-correlate. Roll is too slow-moving to locate a lag; pitch / throttle steps are not. ⚠️ A policy
        # that zigzags tick to tick makes r periodic in L (one tick = 50 ms), so the scan stays causal-side.
        pass_through = (("rc_roll", "rcCommand[0]"), ("rc_pitch", "rcCommand[1]"),
                        ("rc_throttle", "rcCommand[3]"))

        def shifted(lag):
            out = []
            for r in ticks:
                row, skew = nearest(a * r["timestamp_ms"] + b + lag)
                if abs(skew) <= MATCH_MS and row["flags"] == PURE_ACRO:
                    out.append((r, row))
            return out

        scan = []
        for lag in range(0, 141, 10):
            pairs = shifted(lag)
            rs = [pearson_slope([r[x] for r, _ in pairs], [row[y] for _, row in pairs])[0]
                  for x, y in pass_through]
            scan.append((lag, sum(rs) / len(rs), rs))
        print("  lag scan, pass-through r (roll / pitch / throttle) by xiao-clock shift:")
        for lag, mean_r, rs in scan:
            print(f"    {lag:+4d} ms  " + "  ".join(f"{v:.3f}" for v in rs))
        best = max(scan, key=lambda s: s[1])
        print(f"  best common lag {best[0]:+d} ms (tick timestamp -> FC rcCommand); at that lag:")
        for xk, yk in (("out_pitch", "axisRate[1]"), ("out_roll", "axisRate[0]")) + pass_through:
            pairs = shifted(best[0])
            rr, sl = pearson_slope([r[xk] for r, _ in pairs], [row[yk] for _, row in pairs])
            print(f"    {xk:11s} -> {yk:12s} r {rr:+.3f}  slope {sl:+9.2f}")

        dp, dv = [], []
        for _, r, row, _ in joined:
            xp = (r["pos_n"] + org[0], r["pos_e"] + org[1], r["pos_d"] + org[2])
            bp = (row["navPos[0]"] / 100, row["navPos[1]"] / 100, -row["navPos[2]"] / 100)
            xv = (r["vel_n"], r["vel_e"], r["vel_d"])
            bv = (row["navVel[0]"] / 100, row["navVel[1]"] / 100, -row["navVel[2]"] / 100)
            dp.append(math.dist(xp, bp))
            dv.append(math.dist(xv, bv))
        print(f"position |xiao - navPos|  p50 {pct(dp, .5):.2f} m  p95 {pct(dp, .95):.2f}  max {max(dp):.2f}")
        print(f"velocity |xiao - navVel|  p50 {pct(dv, .5):.2f} m/s  p95 {pct(dv, .95):.2f}  max {max(dv):.2f}")


if __name__ == "__main__":
    main()
