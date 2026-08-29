#!/usr/bin/env python3
"""sky_station.py — measure the sky background and the event-noise floor, one orientation per run.

WHY THIS IS THE MEASUREMENT THAT MATTERS. Everything measured indoors ran a background of 0.02 to 4.3
ADU, against beacon deltas of 46 to 255 -- enormous margin everywhere. Sky is the case where the
background is large AND structureless, and it has never been measured. The event detector's limit is
physics, not tuning: an event needs the beacon's frame-to-frame delta to clear the background's shot
noise (sqrt(2) worse for a difference of two frames). At 5 sigma:

    beacon delta 255 ADU  ->  tolerates ~1300 ADU of background
                  96      ->              184
                  46      ->               42
                  10      ->                2

So a single number -- the sky background at the flight exposure -- decides whether the far field works
outdoors. This reads it, and converts it straight into "what beacon delta would survive here".

RUN IT PER ORIENTATION. Appends to CSV so the orientations compare directly.

⚠️ SUN SAFETY. This lens is f/2 at 1.26 mm, so the sun images to about 4 pixels with a concentration
factor of order 1000x even after the 850 nm bandpass removes most of the spectrum. Do NOT dwell with the
sun centred. Keep sun-in-frame captures SHORT (this defaults to 3 s) and put it near the field EDGE
rather than the middle. The sensor is more replaceable than a burned-in column, but neither is free.

Usage:
  sky_station.py --label "north-45" [--seconds 3] [--csv /data/sky.csv]
"""
import argparse
import csv
import os
import subprocess
import sys

import numpy as np


def capture(shutter, gain, w, h, seconds, fps=60):
    cmd = ["rpicam-vid", "-n", "-t", str(int(seconds * 1000)), "--codec", "mjpeg",
           "--framerate", str(fps), "--width", str(w), "--height", str(h), "--denoise", "off",
           "--shutter", str(shutter), "--gain", str(gain), "-o", "-"]
    raw = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout
    import cv2
    fr, i = [], 0
    while True:
        s = raw.find(b"\xff\xd8", i)
        e = raw.find(b"\xff\xd9", s + 2) if s >= 0 else -1
        if s < 0 or e < 0:
            break
        im = cv2.imdecode(np.frombuffer(raw[s:e + 2], np.uint8), cv2.IMREAD_GRAYSCALE)
        if im is not None:
            fr.append(im)
        i = e + 2
    return np.stack(fr).astype(np.int16) if len(fr) > 2 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True, help="orientation, e.g. north-45 / zenith / sun-edge")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--shutter", type=int, default=53, help="the flight exposure")
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--csv", default="/data/sky.csv")
    a = ap.parse_args()

    st = capture(a.shutter, a.gain, a.width, a.height, a.seconds)
    if st is None:
        print("no frames")
        return 1
    npx = st.shape[1] * st.shape[2]
    mean = float(st.mean())
    p50, p99, p999 = (float(np.percentile(st, q)) for q in (50, 99, 99.9))
    mx = int(st.max())
    sat = float((st >= 255).sum()) / st.shape[0]

    d = np.abs(np.diff(st, axis=0))
    nfr = d.shape[0]
    rates = {t: float((d >= t).sum()) / nfr for t in (3, 5, 8, 10, 20, 40, 60)}

    # the threshold that lands near 30 events/frame -- the usable operating point
    op = next((t for t in (3, 5, 8, 10, 20, 40, 60) if rates[t] <= 30), None)
    # minimum beacon delta that clears this background at 5 sigma
    need = 5.0 * (2.0 ** 0.5) * (max(mean, 0.25) ** 0.5)

    print("SKY STATION  %s   (%d frames at %d us, gain %.1f)" % (a.label, st.shape[0], a.shutter, a.gain))
    print("  background: mean %.2f  p50 %.0f  p99 %.0f  p99.9 %.0f  max %d  saturated %.0f px/frame"
          % (mean, p50, p99, p999, mx, sat))
    print("  events/frame vs threshold:")
    print("     " + "  ".join("thr%d %8.1f" % (t, rates[t]) for t in (3, 5, 8, 10)))
    print("     " + "  ".join("thr%d %8.1f" % (t, rates[t]) for t in (20, 40, 60)))
    print("  usable threshold (<=30 ev/frame): %s" % (op if op else "NONE below 60 -- background too high"))
    print("  minimum beacon delta to clear this at 5 sigma: %.1f ADU" % need)
    for lbl, dlt, rng in (("2.5 m", 255, "close"), ("8 m", 46, "mid"), ("17 m", 96, "far-indoor")):
        print("     vs measured delta %3d (%s, %s): %s" % (dlt, lbl, rng, "OK" if dlt > need else "LOST"))

    row = dict(label=a.label, shutter=a.shutter, gain=a.gain, frames=st.shape[0], mean=round(mean, 3),
               p50=p50, p99=p99, p999=p999, max=mx, sat_px=round(sat, 1),
               need_delta=round(need, 1), usable_thr=op if op else -1)
    row.update({("ev_thr%d" % t): round(v, 1) for t, v in rates.items()})
    new = not os.path.exists(a.csv)
    with open(a.csv, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=sorted(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print("  -> appended to %s" % a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
