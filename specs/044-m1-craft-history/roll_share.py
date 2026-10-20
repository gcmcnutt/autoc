#!/usr/bin/env python3
"""
roll_share.py — the roll side of an equal-environment eval (044 T024, honesty ladder rung 3).

For each sim tick CSV (dmp-dump --csv-only), per scenario: roll-rate RMS (deg/s), the share of roll-rate power in
3–5 Hz (Welch at the 20 Hz tick rate, as pitch_cycle_progress.py does for pitch) and the absolute 3–5 Hz roll-rate
RMS (the SC-001 quantity); medians over scenarios, for all scenarios and the calm subset (wind ≤ 3 m/s by the
scenario's own speed draw). Departed scenarios (any |p|, |q|, |r| > 15 rad/s) are excluded, as 043's tools do.

Wind draws come from the eval's console log (the 044 scenario-table column `windMps`; --log), or from a 043
`wind_draws.cc` file (--draws, column 6 = uSpeed, speed = 8·u). Each sim gets its own source, since the 044 build
draws different scenarios from the same seed.

Usage:
  roll_share.py --sim LABEL:tick.csv:(log:console.log | draws:draws.txt) [...]
"""
import argparse
import collections
import csv
import math
import re
import sys

import numpy as np

FS = 20.0
GYRO_SCALE = 6.0
DEPARTED = 15.0


def welch(x, lo, hi, nseg=128):
    x = np.asarray(x, float) - np.mean(x)
    if len(x) < 64:
        return float("nan"), float("nan")
    nseg = min(nseg, (len(x) // 2) * 2)
    w = np.hanning(nseg)
    segs = [x[i:i + nseg] * w for i in range(0, len(x) - nseg + 1, nseg // 2)]
    P = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    fr = np.fft.rfftfreq(nseg, 1 / FS)
    tot = P[fr > 0.25].sum()
    band = P[(fr >= lo) & (fr < hi)].sum()
    share = 100.0 * band / tot if tot > 0 else float("nan")
    rms_band = math.sqrt(np.mean(x ** 2) * band / tot) if tot > 0 else float("nan")
    return share, rms_band


def speeds_from_log(path):
    sp, on = {}, False
    for line in open(path, errors="replace"):
        body = re.sub(r"^\S+ \S+: <info> ", "", line.rstrip("\n"))
        if "ScenarioSeed" in body and "windMps" in body:
            on = True; continue
        if not on:
            continue
        f = body.split()
        if not f:
            break
        if f[0].startswith("---"):
            continue
        if not f[0].isdigit():
            break
        try:
            sp[int(f[0])] = float(f[-4])
        except ValueError:
            sp[int(f[0])] = float("nan")
    return sp


def speeds_from_draws(path):
    return {int(l.split()[0]): 8.0 * float(l.split()[5]) for l in open(path)}


def summarize(tick_csv, speed):
    with open(tick_csv) as f:
        r = csv.reader(f); h = next(r); first, last = {}, {}
        for i, n in enumerate(h):
            first.setdefault(n, i); last[n] = i
        sc = collections.defaultdict(list)
        for row in r:
            if len(row) < len(h):
                continue
            sc[int(row[first["scenario"]])].append((float(row[last["gyrP"]]), float(row[last["gyrQ"]]),
                                                    float(row[last["gyrR"]])))
    out = {"all": [], "calm": []}
    departed = 0
    for s, rows in sc.items():
        A = np.array(rows) * GYRO_SCALE
        if len(A) < 64:
            continue
        if np.max(np.abs(A)) > DEPARTED:
            departed += 1; continue
        p = np.degrees(A[:, 0])
        share, rms35 = welch(p, 3.0, 5.0)
        rec = (float(np.sqrt(np.mean(p ** 2))), share, rms35)
        out["all"].append(rec)
        if speed.get(s, float("nan")) <= 3.0:
            out["calm"].append(rec)
    return out, departed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="append", required=True)
    a = ap.parse_args()
    print(f"{'label':>16} {'subset':>6} {'n':>4} {'roll RMS':>9} {'3-5 Hz %':>9} {'3-5 Hz RMS':>11}  (deg/s; medians)")
    for spec in a.sim:
        label, csvp, src = spec.split(":", 2)
        kind, path = src.split(":", 1)
        speed = speeds_from_log(path) if kind == "log" else speeds_from_draws(path)
        res, dep = summarize(csvp, speed)
        for sub in ("all", "calm"):
            A = np.array(res[sub])
            if len(A) == 0:
                print(f"{label:>16} {sub:>6}    0"); continue
            print(f"{label:>16} {sub:>6} {len(A):4d} {np.median(A[:,0]):9.0f} {np.nanmedian(A[:,1]):9.0f} {np.nanmedian(A[:,2]):11.0f}")
        print(f"{'':>16} departed excluded: {dep}")
    print("flight reference (2026-10-18, t6 g752): roll 3-5 Hz share ~50 %, 3-5 Hz roll RMS 97 deg/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
