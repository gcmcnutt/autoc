#!/usr/bin/env python3
"""pendulum_truth.py — T077: where the beacon ACTUALLY was, per frame, on the pendulum rig.

WHY NOT oracle.py. `oracle.py` is the right truth for a STATIC target: it correlates every pixel's time
series against the Gold code, so nothing but the beacon can win. But that requires the beacon to sit in
one pixel for a whole code word (258 ms), which is precisely what a moving target does not do — at
20 deg/s it crosses 17.7 M2 px in a word. The oracle therefore cannot measure the regime T077 exists to
measure. This tool trades the code discriminator for a geometric one and works through motion.

WHY BRIGHTEST-BLOB IS LEGITIMATE HERE, given oracle.py's explicit warning against it. That warning is
real and it fired on this very rig: the unrestricted per-frame maximum lands on the CEILING LIGHT, not
the beacon. The fix is the ROI below — the measured swept arc, padded. Inside it the beacon is the only
saturating source (the scene is otherwise near-black; the nearest competing blinker is the window 135 M2
px away). Outside that ROI this method is NOT trustworthy, which is why the ROI is required, not optional.

Run `--arc` first on any new rig geometry to MEASURE the swept region, then pass it as --roi. Do not
guess the ROI: a too-generous one re-admits the ceiling light and the failure is silent.

The beacon is dark on ~half the chips, so frames with no bright blob are skipped rather than
interpolated — at 288 fps that still leaves ~144 Hz of position samples against a ~1.8 s period.

VALIDATION (2026-08-25, `pend_static.bcnr`): this tool gives M2 (-7.74, +36.15) where `oracle.py`'s
full-field 31-phase matched filter gives (-8.00, +36.00) — agreement to 0.30 M2 px = 0.09 deg. Re-run
that check whenever the rig or the ROI changes; it is the only thing standing between this tool and the
brightest-pixel failure mode above.

Usage:
  pendulum_truth.py <clip.bcnr> --arc [--step N]          # map the swept arc, print + write arc.png
  pendulum_truth.py <clip.bcnr> --out truth.csv [--roi X0,X1,Y0,Y1] [--min-peak N]
"""
import argparse
import csv
import struct
import sys

import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40                 # <IIQIHH: rec_bytes, seq, t_us, exposure_us, gain_q8, flags
# Angular scale of the M2 grid. MEASURED per lens, not assumed: 0.304 for the 2.31 mm
# lens every pendulum clip was shot on (three independent methods agreed to 1.4%), and
# 0.548 for the 1.26 mm IR-bandpass lens. Override with --deg-per-px; a clip analysed
# under the wrong constant reports every rate wrong by the ratio, silently.
DEG_PER_M2_PX = 0.304

# Default ROI = the swept arc measured on the 2026-08-25 rig (beacon at rest native ~305,272),
# padded. Excludes the ceiling light (native y~47) and the window/door (native x>500).
DEFAULT_ROI = (230, 400, 225, 300)


def open_clip(path):
    f = open(path, "rb")
    magic, ver, hdr_b, w, h, bpp, _mode = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    if magic != MAGIC:
        raise SystemExit("%#x is not BCNR" % magic)
    if bpp != 8:
        raise SystemExit("expected 8bpp, got %d" % bpp)
    return f, w, h, FRAME_HDR + w * h


def frames(f, w, h, fsz):
    """Yield (index, t_us, exposure_us, gain_q8, frame) for every frame in the clip."""
    i = 0
    while True:
        d = f.read(fsz)
        if len(d) < fsz:
            return
        _rec, _seq, t_us, expo, gain, _flags = struct.unpack_from("<IIQIHH", d, 0)
        yield i, t_us, expo, gain, np.frombuffer(d[FRAME_HDR:], dtype=np.uint8).reshape(h, w)
        i += 1


def map_arc(path, step):
    """Max/min projection over the clip -> the swept arc, which is the ROI to use."""
    import cv2
    f, w, h, fsz = open_clip(path)
    mx = np.zeros((h, w), np.uint8)
    mn = np.full((h, w), 255, np.uint8)
    n = 0
    for i, _t, _e, _g, fr in frames(f, w, h, fsz):
        if i % step == 0:
            np.maximum(mx, fr, out=mx)
            np.minimum(mn, fr, out=mn)
            n += 1
    pp = mx.astype(np.int16) - mn.astype(np.int16)
    arc = ((mx >= 250) & (pp >= 150)).astype(np.uint8)   # saturating AND modulating
    ys, xs = np.nonzero(arc)
    print("scanned %d frames (every %d)" % (n, step))
    if not len(xs):
        print("no swept arc found — is the beacon saturating?")
        return
    print("swept arc: native x %d..%d, y %d..%d" % (xs.min(), xs.max(), ys.min(), ys.max()))
    print("  angular span %.1f deg horizontal, %.1f deg vertical"
          % ((xs.max() - xs.min()) / 2 * DEG_PER_M2_PX, (ys.max() - ys.min()) / 2 * DEG_PER_M2_PX))
    print("  suggested --roi %d,%d,%d,%d (padded by 20 px)"
          % (max(0, xs.min() - 20), min(w, xs.max() + 20), max(0, ys.min() - 20), min(h, ys.max() + 20)))
    print("  NB the arc mask also catches any saturating flicker (windows, lamps) — LOOK at arc.png")
    v = mx.astype(np.float32)
    img = (np.log1p(v) * 255.0 / np.log1p(max(v.max(), 1))).clip(0, 255).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img[arc > 0] = (0, 0, 255)
    cv2.imwrite("arc.png", img)
    print("  wrote arc.png (swept arc in red over a log-stretched max projection)")


def extract(path, out, roi, min_peak):
    x0r, x1r, y0r, y1r = roi
    f, w, h, fsz = open_clip(path)
    rows = []
    n = dark = 0
    for i, t_us, expo, gain, full in frames(f, w, h, fsz):
        n += 1
        fr = full[y0r:y1r, x0r:x1r]
        pk = int(fr.max())
        if pk < min_peak:
            dark += 1
            continue
        cy, cx = divmod(int(fr.argmax()), x1r - x0r)
        a, b = max(0, cy - 12), min(y1r - y0r, cy + 13)
        c, d = max(0, cx - 12), min(x1r - x0r, cx + 13)
        win = fr[a:b, c:d].astype(np.float32)
        m = win >= pk * 0.5                       # half-max centroid: robust to the saturated core
        ys, xs = np.nonzero(m)
        wts = win[m]
        rows.append((i, t_us,
                     (xs * wts).sum() / wts.sum() + c + x0r,
                     (ys * wts).sum() / wts.sum() + a + y0r,
                     pk, int(m.sum()), expo, gain))
    print("frames %d, position samples %d (%.0f%%), dark/occluded %d"
          % (n, len(rows), 100.0 * len(rows) / max(n, 1), dark))
    if rows:
        span = (rows[-1][1] - rows[0][1]) / 1e6
        print("  span %.1f s; peak amplitude p50 %d (a CLEAN STOP at full amplitude = pod UVLO, not a fade)"
              % (span, sorted(r[4] for r in rows)[len(rows) // 2]))
    with open(out, "w", newline="") as fh:
        cw = csv.writer(fh)
        cw.writerow(["frame", "t_us", "x_native", "y_native", "peak", "area", "exposure_us", "gain_q8"])
        cw.writerows(rows)
    print("  wrote %s" % out)


def main():
    global DEG_PER_M2_PX
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--arc", action="store_true", help="map the swept arc and exit (do this first)")
    ap.add_argument("--step", type=int, default=3, help="frame stride for --arc")
    ap.add_argument("--out", help="CSV of per-frame truth")
    ap.add_argument("--roi", default=",".join(str(v) for v in DEFAULT_ROI),
                    help="X0,X1,Y0,Y1 in native px — MEASURE it with --arc, do not guess")
    ap.add_argument("--min-peak", type=int, default=100)
    ap.add_argument("--deg-per-px", type=float, default=DEG_PER_M2_PX,
                    help="M2 angular scale of the lens this clip was shot on: 0.304 = 2.31 mm, "
                         "0.548 = 1.26 mm IR bandpass. Wrong value => every rate wrong by the ratio")
    a = ap.parse_args()
    DEG_PER_M2_PX = a.deg_per_px
    if a.arc:
        map_arc(a.clip, a.step)
        return
    if not a.out:
        ap.error("--out is required unless --arc")
    roi = tuple(int(v) for v in a.roi.split(","))
    if len(roi) != 4:
        ap.error("--roi wants X0,X1,Y0,Y1")
    extract(a.clip, a.out, roi, a.min_peak)


if __name__ == "__main__":
    sys.exit(main())
