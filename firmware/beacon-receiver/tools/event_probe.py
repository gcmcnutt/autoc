#!/usr/bin/env python3
"""event_probe.py — how much does event detection actually reduce the search space? (offline, on a .bcnr)

THE IDEA (operator, 2026-08-27): *"basically any illumination delta greater than N flips the bit for that
pixel ... the first pass greatly reduces the search space -- we have a list of events by coordinate --
then using those events we run through the correlation and then spatial isn't too bad."*

So the cost model is two-stage and only the FIRST stage is full-field:

    stage 1   O(pixels)  one subtract + compare per pixel per frame  -> a sparse event list
    stage 2   O(events)  correlate only the coordinates that fired

Stage 2 is only cheap if stage 1 is genuinely sparse, and that is an empirical question about the scene,
not something to assume. This measures it.

WHY THE BANDPASS MAKES AN AGGRESSIVE THRESHOLD SAFE. Measured on this rig with the 850 nm filter: frame
mean 0.02 ADU, and shot noise at zero background is ~1 ADU/frame. So N=5 is already 5 sigma. Without the
filter the background ran 4.24 ADU mean with p99 52, where the same threshold would fire everywhere. The
filter is what buys the low threshold, and the low threshold is what reaches a far-field target.

WHAT IT REPORTS, per threshold:
  - events per frame, and as a fraction of the field (the search-space reduction)
  - how many events land ON the beacon vs elsewhere (signal vs noise in the event list)
  - the split between fast and slow parts of the clip, because a moving target generates events on BOTH
    edges of its motion while a stationary one only blinks

Usage:
  event_probe.py <clip.bcnr> [--start S] [--frames N] [--thresholds 3,5,10,20]
                 [--beacon-roi X0,X1,Y0,Y1]
"""
import argparse
import struct
import sys

import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40


def frames(path, start, count):
    f = open(path, "rb")
    magic, ver, hdr_b, w, h, bpp, _m = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    if magic != MAGIC or bpp != 8:
        raise SystemExit("not an 8bpp BCNR")
    fsz = FRAME_HDR + w * h
    f.seek(HDR + start * fsz)
    for _ in range(count):
        d = f.read(fsz)
        if len(d) < fsz:
            return
        _r, _s, t_us, _e, _g, _fl = struct.unpack_from("<IIQIHH", d, 0)
        yield t_us, np.frombuffer(d[FRAME_HDR:], dtype=np.uint8).reshape(h, w)


CODE_B = "0100011001100111100101001011110"
CODE_A = "0000000100011011000011001110011"


def decode_from_events(clip, start, count, thr, code, chip_hz, top):
    """STAGE 2: correlate the EVENT STREAM, per coordinate, against the code.

    This is the whole point of the two-stage model. Stage 1 gives a sparse list of (coord, sign); the
    only thing left is to ask which coordinates carry sign flips that line up with chip transitions.

    The matched quantity is the TRANSITION, not the level: at a chip boundary the expected change is
    code[k] - code[k-1], which is -1, 0 or +1. Frames inside a chip expect 0 and carry no information, so
    they are skipped -- that is the same fact T082 records as "5 of 12 frames straddle a boundary", seen
    from the detector side. Score = sum(observed_sign * expected_delta) over transition frames only.

    Phase is searched over all 31 here because this is a COLD prototype. In the receiver phase would be
    receiver-global (T081) and this collapses to a single evaluation.
    """
    n = len(code)
    bits = [1 if c == "1" else 0 for c in code]
    acc = {}                                   # coord -> per-phase score
    seen = {}                                  # coord -> event count
    prev = None
    t0 = None
    nf = 0
    for t_us, fr in frames(clip, start, count + 1):
        f = fr.astype(np.int16)
        if prev is None:
            prev, t0 = f, t_us
            continue
        d = f - prev
        prev = f
        ys, xs = np.nonzero(np.abs(d) >= thr)
        if len(xs):
            sgn = np.sign(d[ys, xs])
            # chip index at this frame and the previous one, per candidate phase
            c_now = (t_us - t0) * chip_hz / 1e6
            c_prv = c_now - chip_hz / 288.0
            for ph in range(n):
                k1 = int((c_now + ph) % n)
                k0 = int((c_prv + ph) % n)
                if k1 == k0:
                    continue                   # no boundary crossed at this phase: no information
                exp = bits[k1] - bits[k0]      # -1, 0 or +1
                if exp == 0:
                    continue
                for x, y, sg in zip(xs, ys, sgn):
                    key = (int(x), int(y))
                    a = acc.get(key)
                    if a is None:
                        a = acc[key] = [0] * n
                        seen[key] = 0
                    a[ph] += int(sg) * exp
            for x, y in zip(xs, ys):
                seen[(int(x), int(y))] = seen.get((int(x), int(y)), 0) + 1
        nf += 1
    rows = []
    for k, a in acc.items():
        best = max(range(n), key=lambda i: a[i])
        rows.append((a[best], best, k[0], k[1], seen[k]))
    rows.sort(reverse=True)

    # STAGE 3 -- cluster the point cloud (operator, 2026-08-27: "we do need to figure out how to find
    # near neighbors when close in as a point cloud").
    #
    # A target is not one pixel. At range it is a handful; close in the bloom spreads it over many, and
    # the operator's judgement is that bloom is ACCEPTABLE -- an event is a DELTA, and a saturated core
    # still swings 255->0 when the code goes dark, so saturation costs nothing here the way it wrecked
    # the centroid-based photometry. That is a real simplification: it is an argument for dropping
    # exposure control from this path entirely.
    #
    # Cluster in (x, y, PHASE) rather than (x, y) alone. Phase is the strong axis: every pixel of one
    # emitter votes the same phase, while noise pixels scatter. Two beacons close together but running
    # different codes/phases separate cleanly on it, where a purely spatial clustering would merge them.
    good = [r for r in rows if r[0] > 0]
    used, clusters = set(), []
    bysc = sorted(good, reverse=True)
    for sc, ph, x, y, ev in bysc:
        if (x, y) in used:
            continue
        member = [(sc, ph, x, y, ev)]
        used.add((x, y))
        grew = True
        while grew:                       # flood-fill: adjacency in space AND agreement in phase
            grew = False
            for s2, p2, x2, y2, e2 in bysc:
                if (x2, y2) in used or abs(p2 - ph) > 1:
                    continue
                if any(abs(x2 - mx) <= 2 and abs(y2 - my) <= 2 for _, _, mx, my, _ in member):
                    member.append((s2, p2, x2, y2, e2))
                    used.add((x2, y2))
                    grew = True
        w = sum(m[0] for m in member)
        cx = sum(m[0] * m[2] for m in member) / w
        cy = sum(m[0] * m[3] for m in member) / w
        clusters.append((w, ph, cx, cy, len(member), sum(m[4] for m in member)))
    clusters.sort(reverse=True)

    print("STAGE 2 -- code correlation on the event stream (%d frames, thr %d, %d coords touched)"
          % (nf, thr, len(acc)))
    print("   score  phase  native(x,y)   events   M2 centre-rel")
    for sc, ph, x, y, ev in rows[:top]:
        print("   %5d   %3d   (%4d,%4d)   %6d   (%+.1f,%+.1f)" % (sc, ph, x, y, ev, x/2-320, y/2-200))
    if len(rows) > top:
        w = sorted(r[0] for r in rows)
        print("   ... %d more; score distribution p50 %d p90 %d max %d"
              % (len(rows)-top, w[len(w)//2], w[int(len(w)*.9)], w[-1]))
    print()
    print("STAGE 3 -- clustered into targets (adjacency + phase agreement): %d cluster(s)" % len(clusters))
    print("   weight  phase   centroid native(x,y)   px   events    M2 centre-rel")
    for w_, ph, cx, cy, npx_, ev in clusters[:top]:
        print("   %6d   %3d    (%7.2f,%7.2f)  %4d  %6d    (%+.2f,%+.2f)"
              % (w_, ph, cx, cy, npx_, ev, cx/2-320, cy/2-200))
    return rows, clusters


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--start", type=int, default=0, help="first frame index")
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--thresholds", default="3,5,10,20,40")
    ap.add_argument("--decode", action="store_true", help="run STAGE 2: correlate events against the code")
    ap.add_argument("--decode-thr", type=int, default=5)
    ap.add_argument("--code", default="B")
    ap.add_argument("--chip-hz", type=float, default=120.0)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--beacon-roi", default=None,
                    help="X0,X1,Y0,Y1 native — events inside are 'on beacon', outside are 'elsewhere'")
    a = ap.parse_args()
    thr = [int(t) for t in a.thresholds.split(",")]
    roi = tuple(int(v) for v in a.beacon_roi.split(",")) if a.beacon_roi else None

    if a.decode:
        decode_from_events(a.clip, a.start, a.frames, a.decode_thr,
                           CODE_B if a.code.upper() == "B" else CODE_A, a.chip_hz, a.top)
        return 0

    prev = None
    npx = None
    stats = {t: dict(ev=0, on=0, off=0, frames=0, maxev=0) for t in thr}
    nf = 0
    for t_us, fr in frames(a.clip, a.start, a.frames + 1):
        f = fr.astype(np.int16)
        if prev is None:
            prev, npx = f, f.size
            continue
        d = np.abs(f - prev)
        prev = f
        nf += 1
        for t in thr:
            m = d >= t
            n = int(m.sum())
            s = stats[t]
            s["ev"] += n
            s["frames"] += 1
            s["maxev"] = max(s["maxev"], n)
            if roi and n:
                x0, x1, y0, y1 = roi
                inn = int(m[y0:y1, x0:x1].sum())
                s["on"] += inn
                s["off"] += n - inn
    if not nf:
        raise SystemExit("no frames")

    print("event-detector search-space reduction  (%d frames of %d px)" % (nf, npx))
    print("  thr   events/frame   fraction of field   max/frame   on-beacon   elsewhere")
    for t in thr:
        s = stats[t]
        per = s["ev"] / s["frames"]
        line = "  %3d   %10.1f     %13s   %9d" % (t, per, "1 in %-8.0f" % (npx / per) if per else "-",
                                                  s["maxev"])
        if roi:
            tot = s["on"] + s["off"]
            line += "   %6.1f%%   %6.1f%%" % (100.0 * s["on"] / tot if tot else 0,
                                              100.0 * s["off"] / tot if tot else 0)
        print(line)
    print()
    print("  stage 2 cost is proportional to events/frame; stage 1 is one subtract+compare per pixel")
    print("  regardless. If events/frame is in the hundreds, the correlation is effectively free.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
