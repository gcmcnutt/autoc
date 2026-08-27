#!/usr/bin/env python3
"""beacon_spot.py — locate the emitter in a short capture, for verifying lens geometry through an IR filter.

THE PROBLEM THIS SOLVES. An 850 nm bandpass makes the calibration target invisible: paper and plastic
reflect almost nothing at 850, and the filter blocks the visible light that would otherwise show the grid.
So the usual "detect the board, fit the geometry" loop cannot run at all on the flight lens. The emitter
can — it is a bright point source IN BAND — but only if you can put it at known positions.

THE METHOD. Leave the camera and the target where they are across the lens swap. The VISIBLE-lens frame
then serves as the coordinate reference: it says where each target feature sits in pixels. Put the emitter
on a known feature, capture through the IR lens, and the difference between where it lands and where the
visible frame says that feature is IS the geometry change. Nothing needs the board to be visible in the
IR frame.

WHY A BURST, NOT A FRAME. The emitter is transmitting a Gold code, so it is DARK on roughly half of all
chips. A single grab has about even odds of catching it off, and a "the beacon is not visible" conclusion
drawn from one dark frame is wrong in a way that looks convincing. This max-projects over N frames, which
also renders the spot at full amplitude regardless of which chips were lit.

Usage (on the Pi):
    beacon_spot.py --seconds 3 [--shutter 2000] [--roi X0,X1,Y0,Y1] [--save out.png]
"""
import argparse
import subprocess
import sys

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--shutter", type=int, default=0, help="0 = auto; force it once IR lighting is set")
    ap.add_argument("--gain", type=float, default=0.0)
    ap.add_argument("--roi", default=None, help="X0,X1,Y0,Y1 to exclude other IR sources")
    ap.add_argument("--min-peak", type=int, default=60)
    ap.add_argument("--steady", action="store_true",
                    help="the emitter is running STEADY, not transmitting a code. The default detector "
                         "scores max-minus-min so it REJECTS anything constant -- which is precisely a "
                         "steady emitter. This switches to plain brightness. Use it for alignment; drop "
                         "it once the pod is coded, because then rejecting steady lamps is the point.")
    ap.add_argument("--save", default=None, help="write the max-projection here")
    a = ap.parse_args()

    cmd = ["rpicam-vid", "-n", "-t", str(int(a.seconds * 1000)), "--codec", "mjpeg",
           "--framerate", str(a.fps), "--width", str(a.width), "--height", str(a.height),
           "--denoise", "off", "-o", "-"]
    if a.shutter:
        cmd += ["--shutter", str(a.shutter)]
    if a.gain:
        cmd += ["--gain", str(a.gain)]
    raw = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout

    frames, i = [], 0
    while True:
        s = raw.find(b"\xff\xd8", i)
        e = raw.find(b"\xff\xd9", s + 2) if s >= 0 else -1
        if s < 0 or e < 0:
            break
        f = cv2.imdecode(np.frombuffer(raw[s:e + 2], np.uint8), cv2.IMREAD_GRAYSCALE)
        if f is not None:
            frames.append(f)
        i = e + 2
    if not frames:
        print("no frames decoded", file=sys.stderr)
        return 1
    stack = np.stack(frames)
    mx = stack.max(axis=0)                    # the emitter is dark on ~half the chips; max fixes that
    mn = stack.min(axis=0)
    print("frames %d  max-projection: mean %.2f  peak %d" % (len(frames), mx.mean(), mx.max()))

    view = mx.copy()
    if a.roi:
        x0, x1, y0, y1 = (int(v) for v in a.roi.split(","))
        m = np.zeros_like(view)
        m[y0:y1, x0:x1] = view[y0:y1, x0:x1]
        view = m
    if int(view.max()) < a.min_peak:
        print("NO SOURCE above %d — emitter off, out of frame, or the filter is blocking it"
              % a.min_peak)
        return 2

    if a.steady:
        blink = view.astype(np.int16)                 # brightness: the source is constant by assumption
        label = "bright sources (steady mode -- lamps are NOT rejected)"
    else:
        # blinking = present in max, absent in min. A steady lamp scores ~0; a coded emitter scores high.
        blink = view.astype(np.int16) - mn.astype(np.int16)
        label = "blinking sources (max - min, so steady lamps are rejected)"
    thr = max(a.min_peak // 2, int(blink.max() * 0.4))
    n, lab, st, cent = cv2.connectedComponentsWithStats((blink >= thr).astype(np.uint8), 8)
    rows = []
    for k in range(1, n):
        area = int(st[k, cv2.CC_STAT_AREA])
        if area < 2:
            continue
        rows.append((int(blink[lab == k].max()), area, float(cent[k][0]), float(cent[k][1])))
    rows.sort(reverse=True)
    print(label + ":")
    for pk, area, x, y in rows[:5]:
        print("   amp %3d  area %4d  native (%.2f, %.2f)   M2 centre-rel (%+.2f, %+.2f)"
              % (pk, area, x, y, x / 2 - 320, y / 2 - 200))
    if not rows:
        print("   none — if the pod is running steady, re-run with --steady")
    if a.save:
        img = cv2.cvtColor(mx, cv2.COLOR_GRAY2BGR)
        for pk, area, x, y in rows[:5]:
            cv2.circle(img, (int(x), int(y)), 18, (0, 0, 255), 1)
        cv2.imwrite(a.save, img)
        print("wrote %s" % a.save)
    return 0


if __name__ == "__main__":
    sys.exit(main())
