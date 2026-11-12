#!/usr/bin/env python3
"""
nav_span_scan.py — find nav-estimate faults in a xiao tick CSV (045 PRE-TASK N1 / N3) or, per engaged span, in the INAV blackbox.

Xiao mode: per engaged span, ticks where horizontal velocity is exactly zero (INAV's XY estimate invalid ⇒ frozen position, zero
velocity), position steps that velocity does not explain (residual of Δpos − mean(vel)·Δt), and the largest tick-to-tick steps in
velocity, rabbit, attitude and distance. Pure stdlib, so it runs anywhere the decoder does. First found the 11-08 span 1 faults
(044 flight-analysis-20261108.md § 6): 46 frozen ticks, a 106 m residual at tick 215 → 216, 25 m at 332 → 333.

Blackbox mode (045 T026 inventory, T053 flags): per engaged span (mspOverrideFlags == 2, >= --min-span s), resampled to the xiao's
20 Hz tick so the definitions match data-model.md § Estimator fault flags (thresholds pinned in drift-report.md § Pinned values):
  INVALID_XY   navEPH >= --eph-cap-cm (the flown inav_max_eph_epv; 1000 cm on 11-08 / 10-18, 3000 cm for the re-fly)
  INVALID_Z    navEPV >= --eph-cap-cm
  UNTRUSTED_XY navFlags bit 2 clear (estPosStatus not TRUSTED);  UNTRUSTED_Z  bit 0 clear (estAltStatus not TRUSTED)
  FROZEN       navVel x = y = 0 exactly and navPos xy unchanged vs the previous tick, for >= --frozen-min ticks (default 2)
  STEP_XY / Z  |Δpos − v̄·Δt| > --step-xy / --step-z m in one tick (the INAV estimate jumped; default 5 / 5 m)
  LOW_SATS     GPS numSat < --min-sats (default 6);  NO_FIX  GPS fixType < 2 (no 3D fix)
  GPS_GAP      interval between GPS frames carrying new data > --gps-gap s (default 0.2 = GPS_STALE), from <stem>.gps.csv
               (INAV re-writes an unchanged G frame with each periodic home frame; those copies do not end a gap)
  FAILSAFE     slow-frame failsafePhase not IDLE (skipped, with a note, when the decode has no failsafePhase column)
Prints the faults per span, a flag summary, and the `flight_gates.py --drop FILE.csv#SPAN` arguments for the flagged spans
(FR-013). Blackbox spans are mapped to the xiao CSV's spans (flight_gates.py's names: <xiao csv basename>#<span_id>) by order and
verified: the durations must agree within --match-tol s and the blackbox-minus-xiao start offset must be the same for every span
within --match-tol s (the xiao span starts ~0.7–1.3 s before INAV's override flag sets; the clocks are independent). Spans that
do not verify get no --drop argument and are reported. Writes the events to --out and the clean-tick residual distribution (the
threshold evidence) to --resid-out: by default n1/faults.csv and n1/nav_residuals.csv when all known flights are scanned, and
nothing for a --flight subset or a --bb log unless --out / --resid-out are given (so neither overwrites the N1 record). numpy is
needed in this mode only.

Usage (paths are repo-relative or absolute; run from anywhere):
  nav_span_scan.py xiao.csv [--step 5] [--top 5]          (xiao.csv from src/analytics/flightlog_decode.py -o)
  nav_span_scan.py --blackbox [--flight 1108 ...]         the known flights (default all: 1108, 1018a, 1018b), xiao CSVs paired
  nav_span_scan.py --blackbox --bb path/blackbox_log_X.01.csv --xiao path/flight_log_Y.csv --eph-cap-cm 3000
                   (--bb / --xiao repeatable: the i-th --xiao pairs with the i-th --bb; <stem>.gps.csv is read beside the log)
  options: [--eph-cap-cm 1000] [--step-xy 5] [--step-z 5] [--frozen-min 2] [--min-sats 6] [--gps-gap 0.2] [--min-span 3]
           [--match-tol 1] [--tick 0.05] [--out faults.csv] [--resid-out nav_residuals.csv]
"""
import argparse
import collections
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
KNOWN = {  # key -> (decoded blackbox stem, xiao tick CSV), repo-relative; the stems are span_attitude.FLIGHTS'
    "1108": ("flight-results/flight-20261108/blackbox_log_2026-11-08_160651.01",
             "flight-results/flight-20261108/flight_log_2026-11-09T00-14-08_flight_001.csv"),
    "1018a": ("flight-results/flight-20261018/blackbox_log_2026-10-18_125223.01",
              "flight-results/flight-20261018/flight_log_2026-10-18T19-56-02_flight_001.csv"),
    "1018b": ("flight-results/flight-20261018/blackbox_log_2026-10-18_125223.02",
              "flight-results/flight-20261018/flight_log_2026-10-18T20-20-32_flight_002.csv"),
}
BB_COLS = ["time (us)", "navPos[0]", "navPos[1]", "navPos[2]", "navVel[0]", "navVel[1]", "navVel[2]",
           "navEPH", "navEPV", "navFlags", "mspOverrideFlags"]
FAILSAFE_COL = "failsafePhase (flags)"  # a slow-frame field, carried forward onto every main row by blackbox_decode
KINDS = ("INVALID_XY", "INVALID_Z", "UNTRUSTED_XY", "UNTRUSTED_Z", "FROZEN", "STEP_XY", "STEP_Z",
         "LOW_SATS", "NO_FIX", "GPS_GAP", "FAILSAFE")
UNITS = {"INVALID_XY": "EPH max {} cm", "INVALID_Z": "EPV max {} cm", "STEP_XY": "{} m", "STEP_Z": "{} m",
         "LOW_SATS": "min {} sats", "NO_FIX": "fixType min {}", "GPS_GAP": "{} s between frames", "FAILSAFE": "phase {}"}
NEAR_KINDS = ("INVALID_XY", "INVALID_Z", "UNTRUSTED_XY", "UNTRUSTED_Z", "FROZEN", "NO_FIX", "FAILSAFE")  # ±1 s not "clean"


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def scan(sid, rows, step, top):
    g = lambda r, k: float(r[k])
    t = [g(r, "timestamp_ms") for r in rows]
    P = [(g(r, "pos_n"), g(r, "pos_e"), g(r, "pos_d")) for r in rows]
    V = [(g(r, "vel_n"), g(r, "vel_e"), g(r, "vel_d")) for r in rows]
    R = [(g(r, "rabbit_n"), g(r, "rabbit_e"), g(r, "rabbit_d")) for r in rows]
    Q = [(g(r, "quat_w"), g(r, "quat_x"), g(r, "quat_y"), g(r, "quat_z")) for r in rows]
    n = len(rows) - 1
    series = {
        "pos resid m": [norm([P[i + 1][k] - P[i][k] - 0.5 * (V[i][k] + V[i + 1][k]) * (t[i + 1] - t[i]) / 1000 for k in range(3)])
                        for i in range(n)],
        "dvel m/s": [norm([b - a for a, b in zip(V[i], V[i + 1])]) for i in range(n)],
        "rabbit step m": [norm([b - a for a, b in zip(R[i], R[i + 1])]) for i in range(n)],
        "att step deg": [math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(Q[i], Q[i + 1])))))) for i in range(n)],
    }
    D = [norm([a - b for a, b in zip(P[i], R[i])]) for i in range(len(rows))]
    series["dist step m"] = [abs(D[i + 1] - D[i]) for i in range(n)]
    frozen = [i for i, v in enumerate(V) if v[0] == 0 and v[1] == 0]
    steps = [i + 1 for i, x in enumerate(series["pos resid m"]) if x > step]
    flag = "  ⚠️ FAULT" if frozen or steps else ""
    print(f"\n== span {sid}: {len(rows)} ticks, {(t[-1] - t[0]) / 1000:.1f} s, path {rows[0]['path_index']}{flag}")
    if frozen:
        print(f"  frozen XY (vel_n = vel_e = 0): {len(frozen)} ticks, first {frozen[0]} last {frozen[-1]}"
              f" ({(t[frozen[0]] - t[0]) / 1000:.2f}–{(t[frozen[-1]] - t[0]) / 1000:.2f} s)")
    for i in steps:
        d = [b - a for a, b in zip(P[i - 1], P[i])]
        print(f"  pos step at tick {i - 1} → {i} ({(t[i] - t[0]) / 1000:.2f} s): Δn {d[0]:+.1f} Δe {d[1]:+.1f} Δd {d[2]:+.1f} m")
    for name, a in series.items():
        s = sorted(a)
        worst = sorted(range(len(a)), key=lambda j: -a[j])[:top]
        print(f"  {name:14s} p50 {s[len(s) // 2]:7.3f} p99 {s[int(0.99 * len(s))]:7.3f}  top (tick, value, s): "
              + ", ".join(f"({j + 1}, {a[j]:.2f}, {(t[j + 1] - t[0]) / 1000:.2f})" for j in worst))


def runs(mask):
    """[(i_start, i_end_inclusive)] of True runs."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((start, i - 1)); start = None
    if start is not None:
        out.append((start, len(mask) - 1))
    return out


# ------------------------------------------------------------------------------------------------------------ blackbox mode
def repo_path(p):
    """p as given if it exists from here (or is absolute), else repo-relative."""
    return p if os.path.isabs(p) or os.path.exists(p) or os.path.exists(p + ".csv") else os.path.join(REPO, p)


def stem_of(path):
    return path[:-4] if path.endswith(".csv") else path


def load_nav(stem):
    """<stem>.csv (blackbox_decode output, or a fixture with only these columns) -> nav-estimate arrays.
    navPos m and navVel m/s (NEU, as logged), EPH / EPV cm, failsafe phase names (None when the column is absent)."""
    import numpy as np
    with open(stem + ".csv") as f:
        r = csv.reader(f)
        h = [c.strip() for c in next(r)]
        missing = [c for c in BB_COLS if c not in h]
        if missing:
            raise SystemExit(f"{stem}.csv: missing columns {missing}")
        ix = [h.index(c) for c in BB_COLS]
        jf = h.index(FAILSAFE_COL) if FAILSAFE_COL in h else None
        rows, phase = [], []
        for row in r:
            try:
                vals = [float(row[i]) for i in ix]
                ph = row[jf].strip() if jf is not None else ""
            except (ValueError, IndexError):
                continue
            rows.append(vals)
            phase.append(ph)
    A = np.array(rows)
    return {"t": A[:, 0] / 1e6, "pos": A[:, 1:4] / 100.0, "vel": A[:, 4:7] / 100.0, "eph": A[:, 7], "epv": A[:, 8],
            "flags": A[:, 9].astype(int), "msp": A[:, 10].astype(int),
            "phase": np.array(phase) if jf is not None else None}


def load_gps_frames(stem):
    """<stem>.gps.csv -> every G frame's time, fixType, numSat, and whether it carries new data (differs from the frame before
    in any field but time). None when the file is absent."""
    import numpy as np
    path = stem + ".gps.csv"
    if not os.path.exists(path):
        return None
    t, fix, nsat, new, prev = [], [], [], [], None
    with open(path) as f:
        r = csv.reader(f)
        h = [c.strip() for c in next(r)]
        it, jf, js = h.index("time (us)"), h.index("GPS_fixType"), h.index("GPS_numSat")
        for row in r:
            try:
                tt, fx, ns = int(row[it]), int(row[jf]), int(row[js])
            except (ValueError, IndexError):
                continue
            body = tuple(c.strip() for k, c in enumerate(row) if k != it)
            t.append(tt / 1e6); fix.append(fx); nsat.append(ns); new.append(body != prev)
            prev = body
    return {"t": np.array(t), "fix": np.array(fix), "nsat": np.array(nsat), "new": np.array(new, bool)}


def bb_spans(t, msp, min_len):
    """[(t_start, t_end)] of mspOverrideFlags == 2 runs at least min_len s long (span_attitude.engaged_spans' rule)."""
    import numpy as np
    on = (msp == 2).astype(int)
    d = np.diff(np.r_[0, on, 0])
    starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0] - 1
    return [(t[a], t[b]) for a, b in zip(starts, ends) if t[b] - t[a] >= min_len]


def r2(x):
    return round(float(x), 2)


def phase_bad(p):
    return p not in ("IDLE", "0", "")


def flag_log(stem, args):
    """Scan one decoded blackbox log. Returns (spans, notes): spans = [dict(n, t0, t1, ticks, events, rh, rv, clean)], events =
    [(kind, t_start_s, t_end_s, duration_s, magnitude_str)] with times from the span's start; notes = what could not be checked."""
    import numpy as np
    bb, gps = load_nav(stem), load_gps_frames(stem)
    notes = []
    if gps is None:
        notes.append(f"no {os.path.basename(stem)}.gps.csv: LOW_SATS / NO_FIX / GPS_GAP not checked")
    if bb["phase"] is None:
        notes.append(f"no '{FAILSAFE_COL}' column: FAILSAFE not checked")
    t, out = bb["t"], []
    for n, (a, b) in enumerate(bb_spans(t, bb["msp"], args.min_span), 1):
        tk = np.arange(a, b, args.tick)
        ix = np.clip(np.searchsorted(t, tk, side="right") - 1, 0, len(t) - 1)
        P, V = bb["pos"][ix], bb["vel"][ix]
        eph, epv, fl = bb["eph"][ix], bb["epv"][ix], bb["flags"][ix]
        dt = np.diff(tk)[:, None]
        r = P[1:] - P[:-1] - 0.5 * (V[1:] + V[:-1]) * dt
        rh, rv = np.hypot(r[:, 0], r[:, 1]), np.abs(r[:, 2])
        frozen = np.r_[False, (V[1:, 0] == 0) & (V[1:, 1] == 0) & np.all(P[1:, :2] == P[:-1, :2], axis=1)]
        masks = {"INVALID_XY": eph >= args.eph_cap_cm, "INVALID_Z": epv >= args.eph_cap_cm,
                 "UNTRUSTED_XY": (fl & 4) == 0, "UNTRUSTED_Z": (fl & 1) == 0,
                 "FROZEN": frozen, "STEP_XY": np.r_[False, rh > args.step_xy], "STEP_Z": np.r_[False, rv > args.step_z]}
        mag = {"INVALID_XY": ("max", eph), "INVALID_Z": ("max", epv), "STEP_XY": ("max", np.r_[0, rh]),
               "STEP_Z": ("max", np.r_[0, rv])}
        if gps is not None:
            gix = np.clip(np.searchsorted(gps["t"], tk, side="right") - 1, 0, len(gps["t"]) - 1)
            masks["LOW_SATS"], mag["LOW_SATS"] = gps["nsat"][gix] < args.min_sats, ("min", gps["nsat"][gix])
            masks["NO_FIX"], mag["NO_FIX"] = gps["fix"][gix] < 2, ("min", gps["fix"][gix])
        if bb["phase"] is not None:
            ph = bb["phase"][ix]
            masks["FAILSAFE"] = np.array([phase_bad(p) for p in ph], bool)
            mag["FAILSAFE"] = ("set", ph)
        events, near = [], np.zeros(len(tk), bool)  # near: ticks near a validity / freeze event, excluded from "clean"
        for kind in KINDS:
            if kind not in masks:
                continue
            for i0, i1 in runs(masks[kind]):
                if kind == "FROZEN" and i1 - i0 + 1 < args.frozen_min:
                    continue
                how, x = mag.get(kind, (None, None))
                seg = x[i0:i1 + 1] if x is not None else None
                val = ("" if how is None else f"{seg.max():.2f}" if kind.startswith("STEP") else f"{seg.max():.0f}"
                       if how == "max" else f"{seg.min():.0f}" if how == "min" else "|".join(sorted(set(seg))))
                events.append((kind, r2(tk[i0] - a), r2(tk[i1] - a + args.tick), r2((i1 - i0 + 1) * args.tick), val))
                if kind in NEAR_KINDS:
                    lo, hi = max(i0 - int(1 / args.tick), 0), min(i1 + int(1 / args.tick), len(tk) - 1)
                    near[lo:hi + 1] = True
                if kind in ("STEP_XY", "STEP_Z"):  # the step ticks themselves are faults, not clean residuals
                    near[i0:i1 + 1] = True
        if gps is not None:  # GPS_GAP from the frame times themselves, not the 20 Hz resample
            tn = gps["t"][gps["new"]]
            for g0, g1 in zip(tn[:-1], tn[1:]):
                if g1 - g0 > args.gps_gap and g1 > a and g0 < b:
                    events.append(("GPS_GAP", r2(g0 - a), r2(g1 - a), r2(g1 - g0), f"{g1 - g0:.3f}"))
            if len(tn) and b - tn[-1] > args.gps_gap and tn[-1] < b:  # no new frame after the last one before the span end
                events.append(("GPS_GAP", r2(tn[-1] - a), r2(b - a), r2(b - tn[-1]), f">{b - tn[-1]:.3f}"))
        events.sort(key=lambda e: (KINDS.index(e[0]), e[1]))
        out.append({"n": n, "t0": a, "t1": b, "ticks": len(tk), "events": events, "rh": rh, "rv": rv, "clean": ~near[1:]})
    return out, notes


def xiao_spans(path):
    """[(name, t_start_s, duration_s)] in file order; name as flight_gates.py prints it: <csv basename>#<span_id>."""
    by = collections.OrderedDict()
    with open(path) as f:
        for r in csv.DictReader(f):
            by.setdefault(r["span_id"], []).append(float(r["timestamp_ms"]))
    return [(f"{os.path.basename(path)}#{sid}", ts[0] / 1000.0, (ts[-1] - ts[0]) / 1000.0) for sid, ts in by.items()]


def match_spans(bb, xs, tol):
    """Pair blackbox spans [(t0, dur)] with xiao spans [(name, t0, dur)] by order. Returns ([(name|None, ok, note)] per blackbox
    span, [problems]). ok needs |Δduration| <= tol and the start offset within tol of the median offset."""
    import numpy as np
    problems = []
    if len(bb) != len(xs):
        problems.append(f"{len(bb)} blackbox spans vs {len(xs)} xiao spans: paired by order, each pair checked")
    k = min(len(bb), len(xs))
    off = [bb[i][0] - xs[i][1] for i in range(k)]
    med = float(np.median(off)) if k else 0.0
    res = []
    for i in range(len(bb)):
        if i >= k:
            res.append((None, False, "no xiao span at this position")); continue
        dd, do = bb[i][1] - xs[i][2], off[i] - med
        ok = abs(dd) <= tol and abs(do) <= tol
        res.append((xs[i][0], ok, f"Δdur {dd:+.2f} s, offset {off[i]:+.2f} s (median {med:+.2f})"))
        if not ok:
            problems.append(f"blackbox span {i + 1} ↔ {xs[i][0]}: {res[-1][2]} exceeds ±{tol} s")
    return res, problems


def blackbox_mode(args):
    import numpy as np
    if args.bb:
        if args.xiao and len(args.xiao) != len(args.bb):
            raise SystemExit("--xiao must be given once per --bb (paired by order), or not at all")
        logs = [(os.path.basename(stem_of(p)), stem_of(repo_path(p)), repo_path(args.xiao[i]) if args.xiao else None)
                for i, p in enumerate(args.bb)]
        out, resid_out = args.out, args.resid_out
    else:
        logs = [(k, os.path.join(REPO, KNOWN[k][0]), os.path.join(REPO, KNOWN[k][1])) for k in (args.flight or sorted(KNOWN))]
        n1 = not args.flight  # the full N1 set: the default writes the record of n1/; a subset writes only where told
        out = args.out or (os.path.join(HERE, "n1", "faults.csv") if n1 else None)
        resid_out = args.resid_out or (os.path.join(HERE, "n1", "nav_residuals.csv") if n1 else None)
    rows, resid, summary, drops, unmatched = [], [], [], [], []
    print(f"thresholds: eph/epv cap {args.eph_cap_cm:g} cm, step xy {args.step_xy:g} / z {args.step_z:g} m per {args.tick:g} s"
          f" tick, frozen >= {args.frozen_min} ticks, sats < {args.min_sats}, GPS gap > {args.gps_gap:g} s,"
          f" spans >= {args.min_span:g} s")
    for key, stem, xiao in logs:
        spans, notes = flag_log(stem, args)
        for note in notes:
            print(f"  note ({key}): {note}")
        names = [(None, False, "no xiao CSV given")] * len(spans)
        if xiao:
            if os.path.exists(xiao):
                names, problems = match_spans([(s["t0"], s["t1"] - s["t0"]) for s in spans], xiao_spans(xiao), args.match_tol)
                for p in problems:
                    print(f"  ⚠️ span mapping ({key}): {p}")
            else:
                names = [(None, False, f"xiao CSV not found: {xiao}")] * len(spans)
        for s, (name, ok, note) in zip(spans, names):
            ev = s["events"]
            kinds = sorted({e[0] for e in ev}, key=KINDS.index)
            print(f"== {key} span {s['n']}: {s['t1'] - s['t0']:.1f} s, {s['ticks']} ticks"
                  + (f"  ↔ {name}{'' if ok else ' (UNVERIFIED)'}: {note}" if name else f"  ({note})")
                  + ("  ⚠️ " + ", ".join(kinds) if kinds else ""))
            for e in ev:
                print(f"  {e[0]:12s} {e[1]:6.2f}–{e[2]:6.2f} s ({e[3]:.2f} s)" + (f"  {UNITS[e[0]].format(e[4])}" if e[4] else ""))
            rows += [[key, s["n"], *e] for e in ev]
            summary.append((key, s["n"], s["t1"] - s["t0"], name if ok else None, kinds))
            if kinds:
                (drops if ok else unmatched).append(name if ok else f"{key} span {s['n']} ({note})")
            resid += [(s["rh"], s["rv"], s["clean"])]
    print("\n== flag summary")
    for key, n, dur, name, kinds in summary:
        print(f"  {key:>28s} span {n}  {dur:5.1f} s  {name or '-':>52s}  {' '.join(kinds) if kinds else 'clean'}")
    print("\nflight_gates.py --drop arguments for the flagged spans:")
    print("  " + (" ".join(f"--drop {d}" for d in drops) if drops else "(none)"))
    for u in unmatched:
        print(f"  ⚠️ flagged but not mapped to a xiao span: {u}")
    H = np.concatenate([x[0] for x in resid]) if resid else np.zeros(0)
    Z = np.concatenate([x[1] for x in resid]) if resid else np.zeros(0)
    C = np.concatenate([x[2] for x in resid]) if resid else np.zeros(0, bool)
    stats = []
    for name, m in (("clean", C), ("near_faults", ~C), ("all", np.ones(len(C), bool))):
        for axis, X in (("horizontal", H), ("vertical", Z)):
            x = X[m]
            if len(x):
                row = [name, axis, len(x), *(f"{np.percentile(x, q):.2f}" for q in (50, 99, 99.9)), f"{x.max():.2f}"]
                stats.append(row)
                print(f"  residual {name:11s} {axis:10s} n {len(x):5d}  p50 {row[3]}  p99 {row[4]}  p99.9 {row[5]}  max {row[6]} m")
    if out:
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["flight", "span", "kind", "t_start_s", "t_end_s", "duration_s", "magnitude"])
            w.writerows([[k, n, kind, f"{t0:.2f}", f"{t1:.2f}", f"{d:.2f}", m] for k, n, kind, t0, t1, d, m in rows])
        print(f"wrote {out}")
    if resid_out:
        with open(resid_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["set", "axis", "n_ticks", "p50_m", "p99_m", "p99_9_m", "max_m"])
            w.writerows(stats)
        print(f"wrote {resid_out}")
    return summary, drops


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("csv", nargs="?", help="xiao tick CSV (xiao mode)")
    ap.add_argument("--blackbox", action="store_true", help="scan INAV blackbox logs per engaged span (T026 / T053)")
    ap.add_argument("--flight", action="append", choices=sorted(KNOWN), help="blackbox mode: a known flight (default all)")
    ap.add_argument("--bb", action="append", metavar="LOG.csv", help="blackbox mode: a decoded blackbox log (repeatable)")
    ap.add_argument("--xiao", action="append", metavar="XIAO.csv", help="blackbox mode: the xiao CSV of the i-th --bb")
    ap.add_argument("--tick", type=float, default=0.05, help="blackbox mode: resample tick, s (default 0.05 = the xiao's 20 Hz)")
    ap.add_argument("--eph-cap-cm", type=float, default=1000.0,
                    help="blackbox mode: inav_max_eph_epv of the flown config, cm (default 1000 = 11-08; the re-fly: 3000)")
    ap.add_argument("--step-xy", type=float, default=5.0, help="blackbox mode: STEP_XY threshold, m per tick (default 5)")
    ap.add_argument("--step-z", type=float, default=5.0, help="blackbox mode: STEP_Z threshold, m per tick (default 5)")
    ap.add_argument("--frozen-min", type=int, default=2, help="blackbox mode: FROZEN needs this many consecutive ticks (default 2)")
    ap.add_argument("--min-sats", type=int, default=6, help="blackbox mode: LOW_SATS below this many satellites (default 6)")
    ap.add_argument("--gps-gap", type=float, default=0.2, help="blackbox mode: GPS_GAP above this frame interval, s (default 0.2)")
    ap.add_argument("--min-span", type=float, default=3.0, help="blackbox mode: shortest engaged span scanned, s (default 3)")
    ap.add_argument("--match-tol", type=float, default=1.0, help="blackbox mode: span-mapping tolerance, s (default 1)")
    ap.add_argument("--out", help="blackbox mode: events CSV (default n1/faults.csv for all known flights, else none)")
    ap.add_argument("--resid-out", help="blackbox mode: residual CSV (default n1/nav_residuals.csv for all known flights, else none)")
    ap.add_argument("--step", type=float, default=5.0, help="xiao mode: position residual (m) reported as a step (default 5)")
    ap.add_argument("--top", type=int, default=5, help="xiao mode: worst ticks listed per series (default 5)")
    return ap


def main():
    ap = build_parser()
    args = ap.parse_args()
    if args.blackbox:
        blackbox_mode(args)
        return
    if not args.csv:
        ap.error("a xiao CSV is required unless --blackbox")
    by = collections.OrderedDict()
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            by.setdefault(r["span_id"], []).append(r)
    for sid, rows in by.items():
        scan(sid, rows, args.step, args.top)


if __name__ == "__main__":
    main()
