#!/usr/bin/env python3
"""pendulum_analyze.py — T077: score the tracker against pendulum truth.

Takes `pendulum_truth.py`'s CSV and `beacon_trackd --emit json:-` output over the SAME clip, and answers
the three questions the pendulum rig was built to answer:

  --decay      how the swing wound down: apparent rate, amplitude, beacon peak, exposure per block.
               Also the battery witness — a clean stop at full peak amplitude is the pod's 3.48 V UVLO,
               a fade is something else.
  --rate       decode rate vs apparent angular rate. Reported per TIME BLOCK, not per rate bin, because
               a pendulum passes through low rate instantly: an instantaneous-rate bin mixes "sustained
               slow" with "just arrived from fast", and those score very differently. Decay makes time a
               proxy for SUSTAINED rate, which is the honest axis. (Rate bins are still printed, marked
               as the contaminated view, because the contrast between the two is itself the lesson.)
  --reacquire  recovery after each pole occlusion: occlusion end -> next MEASURED fix, against the
               400 ms relock bar of spec §3. NB this counts any measured fix, including a false one —
               cross-read it against the false share that --rate reports.

MEASURED FIX vs PRESENT is the distinction that matters and the reason this tool exists. `n>=1` only
means the tracker is REPORTING a track; it may be coasting on a stale velocity with no fresh decode at
all. BCN_F_MEASURED_FIX (0x40) means this tick carried a real correlation. On the 2026-08-25 30 cm clip
the tracker was present 49% of ticks and measured on 11% — reading `present` as `tracking` would have
overstated it by 4.5x.

Usage:
  pendulum_analyze.py --truth truth.csv [--track trk.json] [--decay] [--rate] [--reacquire]
"""
import argparse
import bisect
import csv
import json
import math

DEG_PER_M2_PX = 0.304
MEASURED_FIX = 0x40
OCCLUSION_FRAMES = 8      # >=8 missing frames (28 ms) is longer than any code-dark run at 288 fps


def load_truth(path):
    rows = []
    for d in csv.DictReader(open(path)):
        rows.append(dict(frame=int(d["frame"]), t_us=int(d["t_us"]),
                         x=float(d["x_native"]) / 2 - 160, y=float(d["y_native"]) / 2 - 100,
                         peak=int(d["peak"]), expo=int(d["exposure_us"])))
    if not rows:
        raise SystemExit("empty truth CSV")
    return rows


def rates_of(tr, half=4):
    """Central-difference apparent rate (deg/s) keyed by t_us, skipping across occlusion gaps."""
    out = {}
    for i in range(len(tr)):
        a, b = max(0, i - half), min(len(tr) - 1, i + half)
        if tr[b]["frame"] - tr[a]["frame"] > 30:      # spans a gap -> the difference is not a rate
            continue
        dt = (tr[b]["t_us"] - tr[a]["t_us"]) / 1e6
        if dt <= 0:
            continue
        out[tr[i]["t_us"]] = math.hypot(tr[b]["x"] - tr[a]["x"], tr[b]["y"] - tr[a]["y"]) / dt * DEG_PER_M2_PX
    return out


def occlusions(tr):
    return [(tr[i - 1]["t_us"], tr[i]["t_us"], tr[i]["frame"] - tr[i - 1]["frame"] - 1)
            for i in range(1, len(tr)) if tr[i]["frame"] - tr[i - 1]["frame"] - 1 >= OCCLUSION_FRAMES]


def load_ticks(path):
    ticks = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        t = json.loads(line)
        trk = t["tracks"][0] if t["n"] >= 1 else None
        ticks.append(dict(t_us=t["t_us"], present=t["n"] >= 1,
                          measured=bool(trk and trk["flags"] & MEASURED_FIX),
                          x=trk["x"] if trk else None, y=trk["y"] if trk else None,
                          cep=trk["cep"] if trk else None, q=trk["q"] if trk else None))
    return ticks


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def do_decay(tr, rate, block):
    t0 = tr[0]["t_us"]
    span = (tr[-1]["t_us"] - t0) / 1e6
    rs = sorted(rate.values())
    print("truth: %d samples over %.1f s" % (len(tr), span))
    print("apparent rate over the clip: p50 %.1f  p90 %.1f  max %.1f deg/s"
          % (rs[len(rs) // 2], rs[int(len(rs) * .9)], rs[-1]))
    print("\n  block       rate p50  rate p90   x span   ampl@3m   peak p50  peak p10   expo")
    for b in range(int(math.ceil(span / block))):
        lo, hi = b * block, (b + 1) * block
        seg = sorted(r for t, r in ((p["t_us"], rate.get(p["t_us"])) for p in tr)
                     if r is not None and lo <= (t - t0) / 1e6 < hi)
        pts = [p for p in tr if lo <= (p["t_us"] - t0) / 1e6 < hi]
        if not seg or not pts:
            continue
        xs = [p["x"] for p in pts]
        pk = sorted(p["peak"] for p in pts)
        xspan = (max(xs) - min(xs)) * DEG_PER_M2_PX
        print("  %4d-%4ds  %8.1f  %8.1f  %6.2f deg  %5.1f cm   %8d  %8d  %5d"
              % (lo, hi, seg[len(seg) // 2], seg[int(len(seg) * .9)], xspan,
                 100 * 3.0 * math.radians(xspan / 2), pk[len(pk) // 2], pk[len(pk) // 10],
                 sorted(p["expo"] for p in pts)[len(pts) // 2]))


def _match(tr, tt, rate, ts):
    j = bisect.bisect_left(tt, ts)
    cand = [tt[k] for k in (j - 1, j) if 0 <= k < len(tt) and abs(tt[k] - ts) < 40000 and tt[k] in rate]
    if not cand:
        return None
    key = min(cand, key=lambda c: abs(c - ts))
    return key, rate[key], tr[bisect.bisect_left(tt, key)]


def do_rate(tr, tt, rate, ticks, block, limit_s, max_err):
    """Decode rate per time block, splitting MEASURED fixes into on-beacon and FALSE.

    A MEASURED fix only means a correlation passed — it does NOT mean the correlation passed ON THE
    BEACON. Scoring against truth is the only way to tell, and the difference is not small: on the
    30 cm clip 25% of measured fixes land >20 M2 px away, almost all of them on the mains-flicker
    window (120 Hz — the same rate as our chip clock). Counting those as decodes overstates the
    tracker by a third. `on-beacon` below is the number to quote.
    """
    t0 = tr[0]["t_us"]
    print("\n=== decode rate vs SUSTAINED rate (time blocks — the honest axis) ===")
    print("  block       rate p50   ticks   on-beacon fix    false    present   bearing err p50")
    blocks = {}
    for t in ticks:
        el = (t["t_us"] - t0) / 1e6
        if el < 0 or (limit_s and el > limit_s):
            continue
        m = _match(tr, tt, rate, t["t_us"])
        if not m:
            continue
        _key, r, tp = m
        d = blocks.setdefault(int(el // block), dict(n=0, good=0, false=0, pres=0, err=[], rs=[]))
        d["n"] += 1
        d["rs"].append(r)
        d["pres"] += t["present"]
        if t["measured"]:
            e = math.hypot(t["x"] - tp["x"], t["y"] - tp["y"])
            if e <= max_err:
                d["good"] += 1
                d["err"].append(e)
            else:
                d["false"] += 1
    for b in sorted(blocks):
        d = blocks[b]
        d["rs"].sort()
        d["err"].sort()
        e = ("%.2f M2 px = %.2f deg" % (d["err"][len(d["err"]) // 2], d["err"][len(d["err"]) // 2] * DEG_PER_M2_PX)
             if d["err"] else "--")
        print("  %4d-%4ds  %8.1f   %5d   %4d (%3.0f%%)     %4d     %3.0f%%      %s"
              % (b * block, (b + 1) * block, d["rs"][len(d["rs"]) // 2], d["n"],
                 d["good"], pct(d["good"], d["n"]), d["false"], pct(d["pres"], d["n"]), e))
    tot_g = sum(d["good"] for d in blocks.values())
    tot_f = sum(d["false"] for d in blocks.values())
    print("  TOTAL measured fixes: %d on-beacon, %d false (>%.0f M2 px) — false share %.1f%%"
          % (tot_g, tot_f, max_err, pct(tot_f, tot_g + tot_f)))
    print("\n=== the same data binned by INSTANTANEOUS rate (contaminated — see the docstring) ===")
    BINS = [(0, 1), (1, 2), (2, 3), (3, 5), (5, 8), (8, 1e9)]
    st = {b: dict(n=0, meas=0) for b in BINS}
    for t in ticks:
        el = (t["t_us"] - t0) / 1e6
        if el < 0 or (limit_s and el > limit_s):
            continue
        m = _match(tr, tt, rate, t["t_us"])
        if not m:
            continue
        r = m[1]
        for b in BINS:
            if b[0] <= r < b[1]:
                st[b]["n"] += 1
                st[b]["meas"] += t["measured"]
                break
    for b in BINS:
        d = st[b]
        if d["n"] < 20:
            continue
        lab = "%d-%d deg/s" % b if b[1] < 1e9 else ">=8 deg/s"
        print("  %-14s  %5d ticks   %4d (%3.0f%%)" % (lab, d["n"], d["meas"], pct(d["meas"], d["n"])))


def do_reacquire(tr, ticks):
    occ = occlusions(tr)
    span = (tr[-1]["t_us"] - tr[0]["t_us"]) / 1e6
    print("\n=== reacquisition after pole occlusion ===")
    print("occlusions: %d over %.0f s (%.2f/s), duty %.1f%%"
          % (len(occ), span, len(occ) / span if span else 0,
             pct(sum(b - a for a, b, _ in occ), tr[-1]["t_us"] - tr[0]["t_us"])))
    if not occ:
        return
    ds = sorted(n for _, _, n in occ)
    print("  duration: p50 %d fr (%.0f ms)  p90 %d fr (%.0f ms)  max %d fr (%.0f ms)"
          % (ds[len(ds) // 2], ds[len(ds) // 2] / 288 * 1000, ds[int(len(ds) * .9)],
             ds[int(len(ds) * .9)] / 288 * 1000, ds[-1], ds[-1] / 288 * 1000))
    tk = [t["t_us"] for t in ticks]
    lat, lost = [], 0
    for _s, e, _n in occ:
        j = bisect.bisect_left(tk, e)
        hit = next((t for t in ticks[j:] if t["measured"]), None)
        if hit is None or hit["t_us"] - e > 5_000_000:
            lost += 1
            continue
        lat.append((hit["t_us"] - e) / 1000.0)
    lat.sort()
    if lat:
        print("  occlusion end -> next MEASURED fix: p50 %.0f ms  p90 %.0f ms  max %.0f ms"
              % (lat[len(lat) // 2], lat[int(len(lat) * .9)], lat[-1]))
        print("  within the 400 ms relock bar: %d of %d (%.0f%%);  never recovered: %d"
              % (sum(1 for x in lat if x <= 400), len(lat),
                 pct(sum(1 for x in lat if x <= 400), len(lat)), lost))
    n = len(ticks)
    print("  overall: present %.0f%%, MEASURED fix %.0f%% of %d ticks"
          % (pct(sum(t["present"] for t in ticks), n), pct(sum(t["measured"] for t in ticks), n), n))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", required=True)
    ap.add_argument("--track", help="beacon_trackd --emit json:- output over the same clip")
    ap.add_argument("--block", type=float, default=20.0, help="seconds per reporting block")
    ap.add_argument("--limit", type=float, default=0.0,
                    help="ignore ticks after this many seconds (e.g. where the pod hit UVLO)")
    ap.add_argument("--max-err", type=float, default=20.0,
                    help="M2 px beyond which a MEASURED fix is counted as FALSE, not a decode")
    ap.add_argument("--decay", action="store_true")
    ap.add_argument("--rate", action="store_true")
    ap.add_argument("--reacquire", action="store_true")
    a = ap.parse_args()
    if not (a.decay or a.rate or a.reacquire):
        a.decay = a.rate = a.reacquire = True
    tr = load_truth(a.truth)
    tt = [p["t_us"] for p in tr]
    rate = rates_of(tr)
    if a.decay:
        do_decay(tr, rate, a.block)
    if a.rate or a.reacquire:
        if not a.track:
            raise SystemExit("--rate/--reacquire need --track")
        ticks = load_ticks(a.track)
        if a.rate:
            do_rate(tr, tt, rate, ticks, a.block, a.limit, a.max_err)
        if a.reacquire:
            do_reacquire(tr, ticks)


if __name__ == "__main__":
    main()
