#!/usr/bin/env python3
"""trail_decode.py — the physics ceiling of decode-along-track, measured with ORACLE velocity.

THE QUESTION THIS SETTLES (2026-08-30). Three attempts at improving the correlator's motion handling all
failed and located the real limiter: a chicken-and-egg on VELOCITY. The aperture follows the prediction,
the prediction needs a velocity, and the velocity comes from fixes that are failing at high rate. The
tracker can only REFINE a velocity it already has. Hypothesis-based detection (T050 / the event path)
would test (position, velocity, phase) candidates instead — but before building any of that, one number
decides whether it can work at all:

    IF THE TRAJECTORY WERE KNOWN PERFECTLY, does the code decode along the trail at the rates where the
    correlator fails?

This tool answers that by cheating with truth. It samples the clip along the fiducial trajectory — a
small box that follows the beacon exactly, frame by frame — builds the along-track amplitude series, and
correlates it against the Gold code in sliding one-word windows. That is decode-along-track with an
oracle for the velocity search. Two outcomes:

  - along-track decode is HIGH at 8-26 deg/s  ->  the coherence limit is pure velocity acquisition, and a
    hypothesis search (T050 or event-trail matching) recovers it. Build it.
  - along-track decode is ALSO LOW            ->  something else limits (exposure sampling, LED edges,
    blur) and no amount of velocity search will fix it. Do not build T050 for this.

THE CONTROL is the same correlation with the sampling box FROZEN at each window's start position — what a
static aperture sees over one word. The gap between the two columns is exactly what motion compensation
is worth, measured on the same photons.

Usage:
  trail_decode.py <clip.bcnr> --truth truth.csv [--chip-hz 120.75] [--box 4] [--q-bar 0.55]
                  [--deg-per-px 0.548] [--step 15] [--limit 210]
"""
import argparse
import csv
import struct
import sys

import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40
CODE_A = 0b0000000100011011000011001110011
CODE_B = 0b0100011001100111100101001011110
N = 31


def template(bits):
    return np.array([1.0 if (bits >> (N - 1 - k)) & 1 else -1.0 for k in range(N)])


def load_truth(path):
    fr, xs, ys = [], [], []
    for d in csv.DictReader(open(path)):
        fr.append(int(d["frame"]))
        xs.append(float(d["x_native"]))
        ys.append(float(d["y_native"]))
    return np.array(fr), np.array(xs), np.array(ys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--truth", required=True)
    ap.add_argument("--chip-hz", type=float, default=120.75)
    ap.add_argument("--box", type=int, default=4, help="half-size of the sampling box, native px")
    ap.add_argument("--q-bar", type=float, default=0.55, help="decode bar, same as the tracker's q_lock")
    ap.add_argument("--deg-per-px", type=float, default=0.548)
    ap.add_argument("--step", type=int, default=15, help="window slide in frames (~one 20 Hz tick)")
    ap.add_argument("--limit", type=float, default=1e9, help="stop at this many seconds (UVLO tail)")
    ap.add_argument("--max-gap", type=int, default=30,
                    help="interpolate truth across gaps up to this many frames; longer = occlusion, skip")
    a = ap.parse_args()

    tfr, txs, tys = load_truth(a.truth)

    f = open(a.clip, "rb")
    magic, ver, hdr_b, w, h, bpp, _m = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    if magic != MAGIC:
        sys.exit("not a BCNR clip")
    fsz = FRAME_HDR + w * h

    # ---- pass over the clip: timestamps + the two amplitude series -------------------------------
    # Truth positions exist only on lit-chip frames (~50%); a pendulum is smooth, so interpolate
    # inside gaps <= max_gap and mark anything longer (pole occlusions) invalid.
    ts = []
    n_frames = 0
    pos = 0
    while True:
        d = f.read(fsz)
        if len(d) < fsz:
            break
        _r, _s, t, _e, _g, _fl = struct.unpack_from("<IIQIHH", d, 0)
        ts.append(t)
        n_frames += 1
    ts = np.array(ts, dtype=np.float64)
    t_s = (ts - ts[0]) / 1e6

    all_idx = np.arange(n_frames)
    ix = np.interp(all_idx, tfr, txs)
    iy = np.interp(all_idx, tfr, tys)
    # validity: distance to the nearest truth sample
    near = np.searchsorted(tfr, all_idx)
    near = np.clip(near, 0, len(tfr) - 1)
    prev = np.clip(near - 1, 0, len(tfr) - 1)
    gap = np.minimum(np.abs(tfr[near] - all_idx), np.abs(tfr[prev] - all_idx))
    valid = gap <= a.max_gap
    valid &= t_s <= a.limit

    # apparent rate per frame (deg/s) from the interpolated trajectory, central difference over ~29 ms
    HALF = 4
    rate = np.zeros(n_frames)
    d_m2 = np.hypot(np.roll(ix, -HALF) - np.roll(ix, HALF), np.roll(iy, -HALF) - np.roll(iy, HALF)) / 2.0
    dt = np.roll(t_s, -HALF) - np.roll(t_s, HALF)
    with np.errstate(divide="ignore", invalid="ignore"):
        rate = np.where(dt > 0, d_m2 / dt * a.deg_per_px, 0.0)
    rate[:HALF] = rate[HALF]
    rate[-HALF:] = rate[-HALF - 1]

    # ---- second pass: sample along the trail and at the frozen control positions ------------------
    f.seek(HDR)
    B = a.box
    trail = np.zeros(n_frames)
    for i in range(n_frames):
        d = f.read(fsz)
        frame = np.frombuffer(d[FRAME_HDR:], dtype=np.uint8).reshape(h, w)
        cx, cy = int(round(ix[i])), int(round(iy[i]))
        x0, x1 = max(0, cx - B), min(w, cx + B + 1)
        y0, y1 = max(0, cy - B), min(h, cy + B + 1)
        trail[i] = float(frame[y0:y1, x0:x1].sum())
    print("clip %s: %d frames, %.1f s, box +-%d px, chip %.3f Hz" %
          (a.clip, n_frames, t_s[-1], B, a.chip_hz), file=sys.stderr)

    # The static control needs samples at a FROZEN position per window, i.e. a second set of reads.
    # Doing that per window would re-read the clip many times; instead reuse the trail series and note
    # that a frozen box differs from the moving one only once the target leaves it — so emulate the
    # control by zeroing the contribution once the true position is more than `box` px from the window's
    # start position. That is exactly what a frozen box measures (background here is ~0 through the
    # bandpass, so "target outside the box" reads as dark, not as background).
    ta, tb = template(CODE_A), template(CODE_B)
    word_frames = int(round(N * 287.88 / a.chip_hz))          # one code word in frames (~74)

    rows = []
    for s0 in range(0, n_frames - word_frames, a.step):
        s1 = s0 + word_frames
        if not valid[s0:s1].all():
            continue
        tt = t_s[s0:s1]
        chips = np.floor((tt - tt[0]) * a.chip_hz).astype(np.int64)

        def q_of(series):
            # per-chip means, mean-removed, best |corr|/energy over 31 phases x 2 codes
            nb = chips[-1] + 1
            sums = np.bincount(chips, weights=series, minlength=nb)
            cnts = np.bincount(chips, minlength=nb)
            m = cnts > 0
            dev = np.zeros(nb)
            dev[m] = sums[m] / cnts[m]
            dev[m] -= dev[m].mean()
            energy = np.abs(dev[m]).sum() or 1.0
            best_q, best_code = 0.0, None
            for code, tm in (("A", ta), ("B", tb)):
                for ph in range(N):
                    pol = tm[(np.arange(nb) + ph) % N]
                    c = abs(float(np.dot(dev[m], pol[m])))
                    if c / energy > best_q:
                        best_q, best_code = c / energy, code
            return best_q, best_code

        q_trail, c_trail = q_of(trail[s0:s1])

        # frozen control: distance of the true position from the window-start position
        dx = ix[s0:s1] - ix[s0]
        dy = iy[s0:s1] - iy[s0]
        inside = (np.hypot(dx, dy) <= B).astype(float)
        q_stat, c_stat = q_of(trail[s0:s1] * inside)

        rows.append((tt[0], float(np.median(rate[s0:s1])),
                     q_trail, c_trail == "B", q_stat, c_stat == "B"))

    rows = np.array(rows, dtype=object)
    print("\n%d windows scored (%.0f%% of the clip usable)" %
          (len(rows), 100.0 * len(rows) * a.step / max(n_frames - word_frames, 1)))

    def decode(qs, ok):
        return 100.0 * np.mean([(q >= a.q_bar) and o for q, o in zip(qs, ok)])

    print("\n=== decode vs SUSTAINED rate (20 s blocks) — ORACLE trail vs FROZEN control ===")
    print("  block       rate p50   windows   TRAIL decode  q p50    FROZEN decode  q p50")
    t0s = np.array([r[0] for r in rows], dtype=float)
    for lo in range(0, int(t0s.max()) + 1, 20):
        m = (t0s >= lo) & (t0s < lo + 20)
        if m.sum() < 5:
            continue
        sub = rows[m]
        qs_t = [r[2] for r in sub]; ok_t = [r[3] for r in sub]
        qs_s = [r[4] for r in sub]; ok_s = [r[5] for r in sub]
        print("  %3d-%3ds     %5.1f      %4d      %5.1f%%      %.2f       %5.1f%%      %.2f" %
              (lo, lo + 20, np.median([r[1] for r in sub]), m.sum(),
               decode(qs_t, ok_t), np.median(qs_t), decode(qs_s, ok_s), np.median(qs_s)))

    print("\n=== the same, binned by instantaneous rate ===")
    print("  rate bin       windows   TRAIL decode  q p50    FROZEN decode  q p50")
    rr = np.array([r[1] for r in rows], dtype=float)
    for lo, hi in ((0, 2), (2, 5), (5, 8), (8, 12), (12, 18), (18, 40)):
        m = (rr >= lo) & (rr < hi)
        if m.sum() < 5:
            continue
        sub = rows[m]
        qs_t = [r[2] for r in sub]; ok_t = [r[3] for r in sub]
        qs_s = [r[4] for r in sub]; ok_s = [r[5] for r in sub]
        print("  %2d-%2d deg/s    %4d      %5.1f%%      %.2f       %5.1f%%      %.2f" %
              (lo, hi, m.sum(), decode(qs_t, ok_t), np.median(qs_t),
               decode(qs_s, ok_s), np.median(qs_s)))


if __name__ == "__main__":
    main()
