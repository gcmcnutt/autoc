#!/usr/bin/env python3
"""fiducial_truth.py — per-frame ground truth from the printed ArUco constellation (T044, the good half).

WHAT PROBLEM THIS SOLVES. oracle.py finds the beacon by its code, which is honest but *static*: it is a
fixed-pixel matched filter, so it fails in exactly the fast-motion case we most need to measure
(stage1-pan1-analysis.md — coherence dies at 1–3 °/s). Truth therefore depended on decode-along-track, the
very algorithm it was meant to score. Fiducials cut that circle: four markers rigidly attached near the
emitter give a transform PER FRAME, from geometry alone, at the full 288 Hz and at any slew rate.

HOW. Pick a reference frame where the beacon position is known independently (oracle.py, on a slow or
stationary stretch). Every later frame's marker corners give a homography back to that reference, so the
emitter's reference position maps forward into every frame — including frames where the beacon is DARK,
which is half of them, and frames where it is smeared beyond recognition, which is the whole point.

>= 3 markers gives a comfortable homography (12+ point correspondences); 2 markers (8 points) still solves
one and is accepted with a flag, because during a pan markers leave the field and a degraded estimate that
says so beats no estimate. Coverage is REPORTED, never assumed — on this rig at rest it is 100 % of frames
with >= 3 markers, but motion blur will erode that and the number must travel with the envelope.

Needs OpenCV, so it runs on the dev box, not the Pi:
    python3 -m venv /tmp/cv && /tmp/cv/bin/pip install opencv-contrib-python-headless
    /tmp/cv/bin/python fiducial_truth.py <clip.bcnr> --csv truth.csv [--ref-xy X,Y] [--every N]
"""
import argparse, struct, sys
import numpy as np
import cv2

MAGIC = 0x42434E52
HDR, FRAME_HDR = 52, 40
DEG_PER_M2_PX = 0.304


def read_header(f):
    b = f.read(HDR)
    magic, ver, hdr_b, w, h = struct.unpack_from("<IHHHH", b, 0)
    if magic != MAGIC:
        raise SystemExit(f"{magic:#x} is not BCNR")
    if ver != 1:
        raise SystemExit(f"container format_version {ver}, this tool implements 1 — refusing")
    if hdr_b != HDR:
        f.seek(hdr_b)
    return w, h


def frames(f, w, h, every=1):
    npx = w * h
    i = 0
    while True:
        head = f.read(FRAME_HDR)
        if len(head) < FRAME_HDR:
            return
        rec_b, seq, t_us, _e, _g, flags = struct.unpack_from("<IIQIHH", head, 0)
        payload = f.read(rec_b - FRAME_HDR)
        if len(payload) < npx:
            return
        if i % every == 0:
            yield i, seq, t_us, flags, np.frombuffer(payload[:npx], np.uint8).reshape(h, w)
        i += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--csv")
    ap.add_argument("--every", type=int, default=1, help="sample every Nth frame (1 = all)")
    ap.add_argument("--ref-xy", default=None,
                    help="emitter position in the REFERENCE frame, native px 'x,y'. "
                         "Default: the brightest high-pass pixel in the reference frame.")
    a = ap.parse_args()

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    ref_pts = None          # id -> 4x2 corners in the reference frame
    ref_xy = None
    rows = [("i", "t_us", "n_markers", "emitter_x_native", "emitter_y_native",
             "emitter_x_m2", "emitter_y_m2", "deg_per_s", "degraded")]
    prev = None
    n_seen = cov3 = cov4 = 0

    with open(a.clip, "rb") as f:
        w, h = read_header(f)
        for i, _seq, t_us, _flags, img in frames(f, w, h, a.every):
            corners, ids, _ = det.detectMarkers(img)
            found = {}
            if ids is not None:
                for c, k in zip(corners, ids.flatten()):
                    found[int(k)] = c.reshape(4, 2).astype(np.float64)
            n_seen += 1
            cov3 += len(found) >= 3
            cov4 += len(found) >= 4

            if ref_pts is None:
                if len(found) < 3:
                    continue                       # wait for a solid reference
                ref_pts = found
                if a.ref_xy:
                    ref_xy = np.array([float(v) for v in a.ref_xy.split(",")], np.float64)
                else:
                    p = img.astype(np.int32)
                    s = sum(p[1 + dy:h - 1 + dy, 1 + dx:w - 1 + dx]
                            for dy in (-1, 0, 1) for dx in (-1, 0, 1))
                    hp = 9 * p[1:-1, 1:-1] - s
                    yy, xx = divmod(int(np.argmax(hp)), hp.shape[1])
                    ref_xy = np.array([xx + 1.0, yy + 1.0])
                    print(f"reference frame {i}: emitter taken as brightest high-pass px "
                          f"({ref_xy[0]:.0f},{ref_xy[1]:.0f}) — override with --ref-xy if wrong",
                          file=sys.stderr)
                continue

            shared = sorted(set(ref_pts) & set(found))
            if len(shared) < 2:
                rows.append((i, t_us, len(found), "", "", "", "", "", 1))
                continue
            src = np.vstack([ref_pts[k] for k in shared])
            dst = np.vstack([found[k] for k in shared])
            H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if H is None:
                rows.append((i, t_us, len(found), "", "", "", "", "", 1))
                continue
            p = cv2.perspectiveTransform(ref_xy.reshape(1, 1, 2), H).reshape(2)
            m2 = (p[0] / 2.0 - w / 4.0, p[1] / 2.0 - h / 4.0)
            rate = ""
            if prev is not None:
                dt = (t_us - prev[2]) / 1e6
                if dt > 0:
                    d = float(np.hypot(m2[0] - prev[0], m2[1] - prev[1]))
                    rate = f"{d * DEG_PER_M2_PX / dt:.1f}"
            prev = (m2[0], m2[1], t_us)
            rows.append((i, t_us, len(found), f"{p[0]:.2f}", f"{p[1]:.2f}",
                         f"{m2[0]:.2f}", f"{m2[1]:.2f}", rate, int(len(shared) < 3)))

    print(f"\ntruth coverage: >=3 markers on {cov3}/{n_seen} = {100*cov3/max(1,n_seen):.1f}% of frames, "
          f"all four on {100*cov4/max(1,n_seen):.1f}%", file=sys.stderr)
    text = "\n".join(",".join(str(v) for v in r) for r in rows)
    if a.csv:
        open(a.csv, "w").write(text + "\n")
        print(f"{len(rows)-1} frames -> {a.csv}", file=sys.stderr)
    else:
        print(text)


if __name__ == "__main__":
    main()
