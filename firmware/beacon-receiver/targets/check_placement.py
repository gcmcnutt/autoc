#!/usr/bin/env python3
"""check_placement.py — live ArUco readout while you physically place the targets.

Pulls frames from pi/preview.py's MJPEG stream and prints, once a second, which marker ids are detected
and how big they are in native pixels. Move a sheet, watch the line change. Ctrl-C to stop.

Runs on the DEV BOX, not the Pi: it needs OpenCV, and the Pi deliberately has nothing installed on it.
    python3 -m venv /tmp/cv && /tmp/cv/bin/pip install opencv-contrib-python-headless
    /tmp/cv/bin/python check_placement.py [--url http://100.97.242.96:8888/stream]

WHY SIZE IS THE WHOLE GAME. A DICT_4X4_50 marker is 6x6 cells (4x4 data + a 1-cell border), and the
detector needs roughly 5-6 px per cell to read it — so the black square wants **>= ~40 native px**, and
50+ to be comfortable. Measured on this rig 2026-08-21: the lower pair read at 37 px (marginal, detected),
the upper pair measured ~27 px and did NOT detect. Focus was ruled out — the upper region was the sharpest
in the frame (Laplacian variance 2025 vs 1277). It is purely angular size: 0.152 deg per native px means
160 mm subtends ~40 px only out to about 2 m.
"""
import argparse, sys, time
import numpy as np
import cv2

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


def frames(url):
    import urllib.request
    r = urllib.request.urlopen(url, timeout=10)
    buf = b""
    while True:
        chunk = r.read(65536)
        if not chunk:
            return
        buf += chunk
        while True:
            i = buf.find(SOI)
            j = buf.find(EOI, i + 2) if i >= 0 else -1
            if i < 0 or j < 0:
                break
            yield np.frombuffer(buf[i:j + 2], np.uint8)
            buf = buf[j + 2:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://100.97.242.96:8888/stream")
    ap.add_argument("--want", default="0,1,2,3")
    ap.add_argument("--period", type=float, default=1.0)
    a = ap.parse_args()
    want = {int(x) for x in a.want.split(",") if x.strip() != ""}

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    last = 0.0
    for jpg in frames(a.url):
        if time.time() - last < a.period:
            continue
        last = time.time()
        img = cv2.imdecode(jpg, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        h, w = img.shape
        corners, ids, _rej = det.detectMarkers(img)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                q = c.reshape(4, 2)
                side = float(np.mean([np.linalg.norm(q[k] - q[(k + 1) % 4]) for k in range(4)]))
                found[int(i)] = (q[:, 0].mean(), q[:, 1].mean(), side)
        parts = []
        for i in sorted(want):
            if i in found:
                cx, cy, s = found[i]
                mark = "OK " if s >= 40 else "sml"          # 40 px = the detector's comfort floor
                parts.append(f"id{i}:{mark}{s:4.0f}px M2({cx/2 - w/4:+6.0f},{cy/2 - h/4:+5.0f})")
            else:
                parts.append(f"id{i}:MISSING            ")
        n_ok = sum(1 for i in want if i in found and found[i][2] >= 40)
        print(f"\r[{n_ok}/{len(want)} solid] " + " | ".join(parts), end="", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
