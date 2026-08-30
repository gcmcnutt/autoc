#!/usr/bin/env python3
"""trail_search_proto.py — the velocity-hypothesis search WITHOUT the oracle: does it find the trail?

trail_decode.py measured the ceiling: given the true trajectory, decode is 100.0 % at q = 1.00 out to
40 deg/s. This is the other half — a self-sustaining trail tracker that must FIND that trajectory with
the search the C implementation would run:

  every ~1 tick:
    nominate a few candidate positions (brightest pixels in a max-of-12-frames crop around the
      prediction — 12 frames covers the longest dark run of the Gold code, so a lit chip is guaranteed)
    for each candidate x each velocity hypothesis on a coarse grid:
      sample the ring of past frames along the hypothesised trail, bin to chips, correlate all 31 phases
    take the best q; if it clears the bar it IS the new (position, velocity)

Truth is used ONLY to initialise once and to score. If this holds lock through pend4's fast swing at
near-ceiling decode, the parameters are validated and the C build is justified.

Usage:  trail_search_proto.py <clip.bcnr> --truth truth.csv [--frames 12000] [--step 15] ...
"""
import argparse
import csv
import struct
import sys
from collections import deque

import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40
CODE_B = 0b0100011001100111100101001011110
N = 31


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--truth", required=True)
    ap.add_argument("--frames", type=int, default=12000)
    ap.add_argument("--start", type=int, default=100)
    ap.add_argument("--step", type=int, default=15, help="search cadence in frames (~one 20 Hz tick)")
    ap.add_argument("--ring", type=int, default=80)
    ap.add_argument("--chip-hz", type=float, default=120.75)
    ap.add_argument("--crop", type=int, default=48, help="candidate-nomination half-window, native px")
    ap.add_argument("--vmax", type=float, default=64.0, help="velocity grid extent, M2 px/s")
    ap.add_argument("--vstep", type=float, default=8.0, help="velocity grid step, M2 px/s")
    ap.add_argument("--cands", type=int, default=5)
    ap.add_argument("--box", type=int, default=2, help="trail sampling half-box, native px")
    ap.add_argument("--q-bar", type=float, default=0.55)
    ap.add_argument("--deg-per-px", type=float, default=0.548)
    a = ap.parse_args()

    # truth for init + scoring only
    tfr, txs, tys = [], [], []
    for d in csv.DictReader(open(a.truth)):
        tfr.append(int(d["frame"])); txs.append(float(d["x_native"])); tys.append(float(d["y_native"]))
    tfr = np.array(tfr); txs = np.array(txs); tys = np.array(tys)

    f = open(a.clip, "rb")
    magic, ver, hb, w, h, bpp, _m = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    fsz = FRAME_HDR + w * h

    tmpl = np.array([1.0 if (CODE_B >> (N - 1 - k)) & 1 else -1.0 for k in range(N)])
    tmpl_rot = np.stack([np.roll(tmpl, -ph) for ph in range(N)])       # (31 phases, 31 chips)

    # velocity grid, M2 px/s -> native px/s (x2)
    vs = np.arange(-a.vmax, a.vmax + 1e-9, a.vstep)
    VX, VY = np.meshgrid(vs, vs)
    vgrid = np.stack([VX.ravel(), VY.ravel()], axis=1) * 2.0           # native px/s
    nv = len(vgrid)

    ring = deque(maxlen=a.ring)      # (t_us, integral_image i64 with zero pad, raw frame)
    est = None                       # (x_native, y_native, vx_native/s, vy_native/s)
    B = a.box
    results = []

    for i in range(a.frames):
        d = f.read(fsz)
        if len(d) < fsz:
            break
        _r, _s, t_us, _e, _g, _fl = struct.unpack_from("<IIQIHH", d, 0)
        fr = np.frombuffer(d[FRAME_HDR:], dtype=np.uint8).reshape(h, w)
        ii = np.zeros((h + 1, w + 1), np.int64)
        ii[1:, 1:] = fr.astype(np.int64).cumsum(0).cumsum(1)
        ring.append((t_us, ii, fr))

        if i == a.start:                                              # one-time init from truth
            j = np.searchsorted(tfr, i)
            j = min(j, len(tfr) - 1)
            est = [txs[j], tys[j], 0.0, 0.0]
            est_t = t_us
        if est is None or i < a.start or (i - a.start) % a.step or len(ring) < a.ring:
            continue

        # ---- predict forward to now ----
        dt = (t_us - est_t) / 1e6
        px, py = est[0] + est[2] * dt, est[1] + est[3] * dt

        # ---- nominate candidates: local maxima of max-over-last-12 inside the crop ----
        x0, x1 = max(0, int(px) - a.crop), min(w, int(px) + a.crop)
        y0, y1 = max(0, int(py) - a.crop), min(h, int(py) + a.crop)
        recent = np.max([r[2][y0:y1, x0:x1] for r in list(ring)[-12:]], axis=0).astype(np.int16)
        cands = [(px, py)]
        tmp = recent.copy()
        for _ in range(a.cands - 1):
            cy, cx = np.unravel_index(int(tmp.argmax()), tmp.shape)
            if tmp[cy, cx] < 100:
                break
            cands.append((x0 + cx, y0 + cy))
            tmp[max(0, cy - 2):cy + 3, max(0, cx - 2):cx + 3] = 0

        # ---- chip binning fixed for this ring composition ----
        ts_ring = np.array([r[0] for r in ring], np.float64)
        rel = (ts_ring - ts_ring[0]) / 1e6
        chips = np.floor(rel * a.chip_hz).astype(np.int64)
        nb = int(chips[-1]) + 1
        cnts = np.bincount(chips, minlength=nb).astype(np.float64)
        assign = np.zeros((len(ring), nb))
        assign[np.arange(len(ring)), chips] = 1.0
        fold = np.zeros((nb, N))                                       # chip index -> mod-31 bin
        fold[np.arange(nb), np.arange(nb) % N] = 1.0

        # ---- evaluate all (candidate, velocity) hypotheses ----
        best = (0.0, None)
        back = (ts_ring - ts_ring[-1]) / 1e6                           # <= 0
        for (cx0, cy0) in cands:
            # trail positions per ring frame per velocity: pos - v*(t_now - t_i)
            xs = np.clip((cx0 + np.outer(vgrid[:, 0], back)).round().astype(int), B, w - B - 1)
            ys = np.clip((cy0 + np.outer(vgrid[:, 1], back)).round().astype(int), B, h - B - 1)
            samples = np.empty((nv, len(ring)))
            for k, (_t, ii_k, _f) in enumerate(ring):                  # box sums via integral image
                X, Y = xs[:, k], ys[:, k]
                samples[:, k] = (ii_k[Y + B + 1, X + B + 1] - ii_k[Y - B, X + B + 1]
                                 - ii_k[Y + B + 1, X - B] + ii_k[Y - B, X - B])
            binned = samples @ assign                                  # (nv, nb) chip sums
            means = binned / np.maximum(cnts, 1)
            m31 = means @ fold / np.maximum(cnts @ fold, 1)            # not exact mean, close enough
            dev = m31 - m31.mean(axis=1, keepdims=True)
            energy = np.abs(dev).sum(axis=1) + 1e-9
            corr = np.abs(dev @ tmpl_rot.T).max(axis=1)                # best phase per hypothesis
            q = corr / energy
            k = int(q.argmax())
            if q[k] > best[0]:
                best = (float(q[k]), (cx0, cy0, vgrid[k, 0], vgrid[k, 1]))

        q, hyp = best
        if hyp and q >= a.q_bar:
            est = [hyp[0], hyp[1], hyp[2], hyp[3]]
            est_t = t_us
            locked = True
        else:
            locked = False                                             # coast on the old estimate

        # ---- score vs truth ----
        j = np.searchsorted(tfr, i)
        j = min(max(j, 1), len(tfr) - 1)
        jj = j if abs(tfr[j] - i) < abs(tfr[j - 1] - i) else j - 1
        if abs(tfr[jj] - i) <= 15:
            ex = (est[0] + est[2] * (t_us - est_t) / 1e6 - txs[jj]) / 2.0     # M2 px
            ey = (est[1] + est[3] * (t_us - est_t) / 1e6 - tys[jj]) / 2.0
            k0 = max(0, jj - 4); k1 = min(len(tfr) - 1, jj + 4)
            dtp = (tfr[k1] - tfr[k0]) / 287.88
            rate = np.hypot(txs[k1] - txs[k0], tys[k1] - tys[k0]) / 2.0 / max(dtp, 1e-6) * a.deg_per_px
            results.append((i / 287.88, rate, q, locked, np.hypot(ex, ey)))

    res = np.array(results)
    print("%d searches scored" % len(res))
    print("\n  rate bin      n     LOCKED   on-beacon(<=5 M2px)   err p50   q p50")
    for lo, hi in ((0, 2), (2, 5), (5, 8), (8, 12), (12, 18), (18, 40)):
        m = (res[:, 1] >= lo) & (res[:, 1] < hi)
        if m.sum() < 5:
            continue
        sub = res[m]
        onb = (sub[:, 3] > 0) & (sub[:, 4] <= 5.0)
        print("  %2d-%2d deg/s %5d    %5.1f%%       %5.1f%%           %5.2f     %.2f" %
              (lo, hi, m.sum(), 100 * sub[:, 3].mean(), 100 * onb.mean(),
               np.median(sub[:, 4]), np.median(sub[:, 2])))


if __name__ == "__main__":
    main()
