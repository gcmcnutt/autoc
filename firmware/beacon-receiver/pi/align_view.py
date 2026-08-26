#!/usr/bin/env python3
"""align_view.py — live camera view for LENS/FOV BASELINING: reticles, angular rings, zone sharpness.

RUNS ON THE PI, streams MJPEG to ffplay anywhere:

    ssh pi@<pi> 'cd ~/autoc-beacon/firmware/beacon-receiver && ~/cv/bin/python pi/align_view.py' \\
      | ffplay -f mjpeg -i -

MJPEG, not raw. live_view.py sends raw BGR because it runs over the gigabit link; this one is for the
alignment bench, which is often on WiFi, where 1280x800x3 at 10 fps is ~30 MB/s and simply will not fit.
JPEG puts it near 1 MB/s.

WHY NOT pi/focus_view.py. That tool measures Laplacian variance on a CENTRE 160x160 crop only. For a
~95-103 deg lens the centre is never the problem -- field curvature means the CORNERS go soft first, and
corner sharpness is exactly what a wide-angle calibration depends on, because the distortion coefficients
are fitted from corner detections out there. A centre-only number will read "sharp" while the data you
are about to calibrate from is mush. This measures five zones and holds each one's peak, so you can turn
the barrel and see which zone is being traded against which.

WHY AUTO-EXPOSURE. The tracker runs at 53 us, where paper under room light is BLACK. A calibration board
has to be seen, so this lets the ISP choose -- the same reason pi/preview.py exists. Nothing here writes
to /data and nothing touches the tracker's config.

THE ANGULAR RINGS are drawn from --deg-per-px, default 0.304 (the spec's assumed uniform M2 scale, which
the pendulum independently measured at 0.2999 on the 1.8 mm lens). They are the ASSUMPTION made visible:
put a straight edge or a board of known angular width in the frame and the rings show you directly where
the uniform-scale model starts to disagree with the lens. That disagreement is what T075 exists to
quantify, so being able to SEE it before calibrating is worth the four circles.
"""
import argparse
import subprocess
import sys

import cv2
import numpy as np

C_RETICLE = (60, 200, 60)
C_RING    = (200, 160, 60)
C_ZONE    = (200, 200, 200)
C_PEAK    = (80, 255, 255)
C_DIM     = (110, 110, 110)


def sharpness(gray):
    """Laplacian variance — the standard focus metric. Higher is sharper."""
    return float(cv2.Laplacian(gray, cv2.CV_32F, ksize=3).var())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--deg-per-px", type=float, default=0.304,
                    help="assumed M2 deg/px, for the angular rings (native px = half this)")
    ap.add_argument("--zone", type=int, default=120, help="sharpness patch size, px")
    ap.add_argument("--rings", default="10,20,30,40", help="ring radii in degrees; empty to disable")
    ap.add_argument("--shutter", type=int, default=0, help="0 = auto exposure (what you want for a board)")
    a = ap.parse_args()

    cmd = ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg", "--framerate", str(a.fps),
           "--width", str(a.width), "--height", str(a.height), "--denoise", "off", "-o", "-"]
    if a.shutter:
        cmd += ["--shutter", str(a.shutter)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=1 << 22)

    W, H = a.width, a.height
    cx, cy = W // 2, H // 2
    z = a.zone
    # centre + four corners, inset by half a patch
    zones = [("C", cx - z // 2, cy - z // 2), ("TL", 4, 4), ("TR", W - z - 4, 4),
             ("BL", 4, H - z - 4), ("BR", W - z - 4, H - z - 4)]
    peak = {n: 0.0 for n, _, _ in zones}
    rings = [float(r) for r in a.rings.split(",") if r.strip()] if a.rings.strip() else []
    # native px per degree: the M2 grid is half-resolution, so a native px is half an M2 px in angle
    px_per_deg = 1.0 / (a.deg_per_px / 2.0)

    buf = b""
    out = sys.stdout.buffer
    try:
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(b"\xff\xd8")
                e = buf.find(b"\xff\xd9", s + 2) if s >= 0 else -1
                if s < 0 or e < 0:
                    break
                jpg, buf = buf[s:e + 2], buf[e + 2:]
                fr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
                if fr is None:
                    continue
                if fr.shape[1] != W or fr.shape[0] != H:
                    fr = cv2.resize(fr, (W, H))
                img = cv2.cvtColor(fr, cv2.COLOR_GRAY2BGR)

                for ri, r in enumerate(rings):       # angular rings: the uniform-scale model, drawn
                    rp = int(round(r * px_per_deg))
                    if rp < max(W, H):
                        cv2.circle(img, (cx, cy), rp, C_RING, 1, cv2.LINE_AA)
                        # stagger the labels around the ring, else they stack on one horizontal line
                        ang = -0.6 + 0.35 * ri
                        lx = int(cx + rp * np.cos(ang)) + 4
                        ly = int(cy + rp * np.sin(ang))
                        cv2.putText(img, "%g" % r, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                                    C_RING, 1, cv2.LINE_AA)
                cv2.line(img, (0, cy), (W, cy), C_RETICLE, 1)
                cv2.line(img, (cx, 0), (cx, H), C_RETICLE, 1)
                for f in (1 / 3.0, 2 / 3.0):         # thirds, for squaring the board up
                    cv2.line(img, (int(W * f), 0), (int(W * f), H), C_DIM, 1)
                    cv2.line(img, (0, int(H * f)), (W, int(H * f)), C_DIM, 1)

                y = 20
                for name, x0, y0 in zones:
                    v = sharpness(fr[y0:y0 + z, x0:x0 + z])
                    if v > peak[name]:
                        peak[name] = v
                    at_peak = v >= peak[name] * 0.97
                    col = C_PEAK if at_peak else C_ZONE
                    cv2.rectangle(img, (x0, y0), (x0 + z, y0 + z), col, 1)
                    cv2.putText(img, "%s %7.1f" % (name, v), (x0 + 3, y0 + z - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44, col, 1, cv2.LINE_AA)
                    cv2.putText(img, "%-3s %8.1f  peak %8.1f%s" % (name, v, peak[name],
                                                                   "  <" if at_peak else ""),
                                (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, col, 1, cv2.LINE_AA)
                    y += 18
                cv2.putText(img, "turn the barrel for MAX -- corners go soft first on a wide lens",
                            (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_DIM, 1, cv2.LINE_AA)

                ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
                if ok:
                    try:
                        out.write(enc.tobytes())
                        out.flush()
                    except BrokenPipeError:
                        return 0
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
