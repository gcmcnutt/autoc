#!/usr/bin/env python3
"""range_station.py — one command per range station: photometry AND tracking, appended to a CSV.

RUNS ON THE PI. The camera is single-access, so the two measurements are taken in sequence:

  1. PHOTOMETRY — a short raw burst at the tracker's own exposure. Reports peak, blob area and
     background-subtracted SUM. The sum is the flux measure that anchors 1/r^2, but ONLY while the peak
     is unsaturated: a clipped core makes the sum a lower bound, and on the optical bench a 50 mA emitter
     at 0.28 m stayed clipped even at 53 us with the current turned down 60x. So the tool states outright
     whether the point is usable rather than leaving a saturated number looking like a measurement.

  2. TRACKING — beacon_trackd live, reporting present vs MEASURED. Those differ enormously (49% vs 11%
     on the pendulum clips) and only MEASURED means a fresh decode; `present` includes coasting.

Run it once per station. The CSV accumulates, so the falloff plots itself:

    range_station.py --range 1.0 --csv /data/range.csv
    range_station.py --range 3.0 --csv /data/range.csv     # ... etc

WHAT TO WATCH FOR. Two numbers matter more than the rest: the range at which the peak first comes off
255 (the point the current exposure policy starts behaving at all), and the range at which MEASURED
collapses (the actual detection limit). They are not the same range and the gap between them is the
headroom the exposure policy is wasting.
"""
import argparse
import csv
import json
import os
import subprocess
import sys

import numpy as np


def photometry(shutter, gain, w, h, seconds, at=None):
    """`at` = (x, y) in NATIVE px, from the tracker. Without it this falls back to the frame maximum,
    which is right indoors on a black background and WRONG outdoors: the brightest blob in a sunlit
    scene is the ground, not the beacon. Measured at 10 m against foliage, the fallback reported peak
    255 for the driveway while the beacon was a dim source the tracker could barely hold. Same class of
    error as oracle.py's brightest-pixel warning, which finds the ceiling light."""
    cmd = ["rpicam-vid", "-n", "-t", str(int(seconds * 1000)), "--codec", "mjpeg", "--framerate", "10",
           "--width", str(w), "--height", str(h), "--denoise", "off",
           "--shutter", str(shutter), "--gain", str(gain), "-o", "-"]
    raw = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout
    import cv2
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
        return None
    a = np.stack(frames).astype(np.int16)
    mx = a.max(axis=0)                      # a coded emitter is dark on ~half its chips
    bg = float(np.median(a))
    peak = int(mx.max())
    # Take ONLY the connected blob containing the peak. A global threshold looks fine on a black
    # hallway and fails silently the moment house lights raise the floor: at 16.69 m a 0.15*peak cut
    # fell below the max-projection noise and selected all 256000 pixels, reporting the whole frame as
    # the emitter. Flood-filling from the peak cannot do that.
    import cv2 as _cv
    if at is not None:
        # Measure a fixed aperture AT THE TRACKER'S POSITION. No thresholding, no blob search: a dim
        # beacon does not win a brightest-blob contest against sunlit ground, and searching would find
        # the ground instead. peak/sum here are the beacon's, whatever else is in frame.
        ax, ay = int(round(at[0])), int(round(at[1]))
        y0, y1 = max(0, ay - 12), min(mx.shape[0], ay + 13)
        x0, x1 = max(0, ax - 12), min(mx.shape[1], ax + 13)
        m = np.zeros_like(mx, bool)
        m[y0:y1, x0:x1] = True
        peak = int(mx[y0:y1, x0:x1].max())
        bg = float(np.median(a[:, y0:y1, x0:x1]))
    else:
        thr = max(peak * 0.35, bg + 6)
        lab = (mx >= thr).astype(np.uint8)
        nlab, comp = _cv.connectedComponents(lab, 8)
        py, px_ = np.unravel_index(int(mx.argmax()), mx.shape)
        m = comp == comp[py, px_] if nlab > 1 else lab.astype(bool)
    ys, xs = np.nonzero(m)
    cx = float((xs * (mx[m] - bg)).sum() / max((mx[m] - bg).sum(), 1e-9)) if m.sum() else 0.0
    cy = float((ys * (mx[m] - bg)).sum() / max((mx[m] - bg).sum(), 1e-9)) if m.sum() else 0.0
    return dict(peak=peak, bg=bg, area=int(m.sum()), total=float((mx[m] - bg).sum()),
                sat=int((mx >= 255).sum()), frame_mean=float(a.mean()), cx=cx, cy=cy)


def tracking(trackd, config, seconds):
    out = subprocess.run(["timeout", str(int(seconds)), trackd, "--config", config, "--emit", "json:-"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout.decode()
    ticks = [json.loads(l) for l in out.splitlines() if l.strip().startswith("{") and '"tracks"' in l]
    if not ticks:
        return None
    pres = [t for t in ticks if t["n"] >= 1]
    meas = [t for t in pres if t["tracks"][0]["flags"] & 0x40]
    first = next(((t["t_us"] - ticks[0]["t_us"]) / 1e6 for t in ticks if t["n"] >= 1), None)
    q = sorted(t["tracks"][0]["q"] for t in pres) or [0]
    cep = sorted(t["tracks"][0]["cep"] for t in pres) or [0]
    xs = [t["tracks"][0]["x"] for t in pres]
    ys = [t["tracks"][0]["y"] for t in pres]
    return dict(x=(sum(xs) / len(xs)) if xs else None, y=(sum(ys) / len(ys)) if ys else None,
                ticks=len(ticks), present=100.0 * len(pres) / len(ticks),
                measured=100.0 * len(meas) / len(ticks),
                first_lock=first if first is not None else -1.0,
                q50=q[len(q) // 2], cep50=cep[len(cep) // 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--range", type=float, required=True, help="metres to the emitter")
    ap.add_argument("--csv", default="/data/range.csv")
    ap.add_argument("--config", default="beacon-bench.ini")
    ap.add_argument("--trackd", default="../../build/firmware/beacon-receiver/beacon_trackd")
    ap.add_argument("--shutter", type=int, default=53, help="the tracker's operating exposure")
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--seconds", type=float, default=15.0, help="tracking dwell")
    ap.add_argument("--note", default="")
    a = ap.parse_args()

    print("station %.2f m -- tracking %.0f s ..." % (a.range, a.seconds))
    t = tracking(a.trackd, a.config, a.seconds)
    if t:
        print("   first lock %5.2f s   present %3.0f%%   MEASURED %3.0f%%   q50 %.2f   cep50 %.2f"
              % (t["first_lock"], t["present"], t["measured"], t["q50"], t["cep50"]))
    else:
        print("   NO TRACK AT ALL")
    at = None
    if t and t.get("x") is not None:
        at = ((t["x"] + 320) * (a.width / 640.0), (t["y"] + 200) * (a.height / 400.0))
        print("station %.2f m -- photometry AT the tracked position (%.0f,%.0f) ..." % (a.range, *at))
    else:
        print("station %.2f m -- photometry (no track; falling back to brightest blob) ..." % a.range)
    p = photometry(a.shutter, a.gain, a.width, a.height, 2.0, at)
    if p:
        state = "SATURATED (sum is a LOWER BOUND)" if p["sat"] else "clean"
        print("   peak %3d  area %5d  sum %10.0f  bg %.2f  frame mean %.2f   %s"
              % (p["peak"], p["area"], p["total"], p["bg"], p["frame_mean"], state))
    else:
        print("   no frames")
    row = dict(range_m=a.range, note=a.note)
    row.update({("ph_" + k): v for k, v in (p or {}).items()})
    row.update({("tr_" + k): v for k, v in (t or {}).items()})
    new = not os.path.exists(a.csv)
    with open(a.csv, "a", newline="") as fh:
        wcsv = csv.DictWriter(fh, fieldnames=sorted(row))
        if new:
            wcsv.writeheader()
        wcsv.writerow(row)
    print("   -> appended to %s" % a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
