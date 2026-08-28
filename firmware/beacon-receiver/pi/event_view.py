#!/usr/bin/env python3
"""event_view.py — watch the event detector run live: events, clusters, phase votes.

    ssh pi@<pi> 'cd ~/autoc-beacon/firmware/beacon-receiver && ~/cv/bin/python pi/event_view.py' \\
      | ffplay -f mjpeg -i -

WHAT IT SHOWS, which is the whole three-stage chain from event-camera-emulation.md:

    stage 1   per-pixel frame-to-frame delta over a threshold  -> events, drawn as dots
              green = brightening, red = darkening (the code's polarity, visible directly)
    stage 2   per-coordinate correlation of event signs against the Gold code, all 31 phases
    stage 3   clustering on ADJACENCY AND PHASE AGREEMENT     -> circles, labelled with phase

HONEST ABOUT ITS RATE. The real detector runs in C at the frame clock; this is Python and cannot. It
therefore works in BURSTS: it reads as many consecutive frames as it can, processes that window properly,
draws the result, and repeats. The HUD reports the achieved capture rate and the window length so you can
see what the analysis was actually computed over -- a viewer that quietly analysed every third frame would
show plausible-looking events computed across chip boundaries, which is worse than showing nothing.

Deltas are taken between CONSECUTIVE DECODED frames. If the decode rate falls below capture, that spans
more than one frame interval and the correlation degrades; the HUD says so rather than hiding it.
"""
import argparse
import subprocess
import sys
import time

import cv2
import numpy as np

CODE = {"B": "0100011001100111100101001011110",
        "A": "0000000100011011000011001110011"}
C_POS = (120, 255, 120)     # brightening event
C_NEG = (90, 90, 255)       # darkening event
C_CLU = (255, 200, 60)
C_HUD = (210, 210, 210)
C_DIM = (120, 120, 120)


def cluster(scores, n_phase):
    """(x,y)->(score,phase) into clusters by adjacency AND phase agreement. See event-camera-emulation §5."""
    pts = sorted(((v[0], v[1], k[0], k[1]) for k, v in scores.items() if v[0] > 0), reverse=True)
    used, out = set(), []
    for sc, ph, x, y in pts:
        if (x, y) in used:
            continue
        mem = [(sc, ph, x, y)]
        used.add((x, y))
        grew = True
        while grew:
            grew = False
            for s2, p2, x2, y2 in pts:
                if (x2, y2) in used or min(abs(p2 - ph), n_phase - abs(p2 - ph)) > 1:
                    continue
                if any(abs(x2 - mx) <= 2 and abs(y2 - my) <= 2 for _, _, mx, my in mem):
                    mem.append((s2, p2, x2, y2))
                    used.add((x2, y2))
                    grew = True
        w = sum(m[0] for m in mem)
        out.append((w, ph, sum(m[0] * m[2] for m in mem) / w, sum(m[0] * m[3] for m in mem) / w, len(mem)))
    out.sort(reverse=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--fps", type=int, default=200, help="requested capture rate")
    ap.add_argument("--shutter", type=int, default=53)
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--thr", type=int, default=5, help="event threshold in ADU of frame-to-frame delta")
    ap.add_argument("--window", type=int, default=90, help="frames per analysis burst")
    ap.add_argument("--code", default="B")
    ap.add_argument("--chip-hz", type=float, default=120.0)
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("--quality", type=int, default=88)
    a = ap.parse_args()

    bits = [1 if c == "1" else 0 for c in CODE[a.code.upper()]]
    n = len(bits)
    W, H = a.width * a.scale, a.height * a.scale

    cmd = ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg", "--framerate", str(a.fps),
           "--width", str(a.width), "--height", str(a.height), "--denoise", "off",
           "--shutter", str(a.shutter), "--gain", str(a.gain), "-o", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=1 << 22)
    out = sys.stdout.buffer
    buf = b""
    prev = None
    scores = {}
    seen = {}
    nev = 0
    nfr = 0
    t_start = time.time()
    last_frame = None
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
                last_frame = fr
                f = fr.astype(np.int16)
                if prev is None:
                    prev = f
                    continue
                d = f - prev
                prev = f
                ys, xs = np.nonzero(np.abs(d) >= a.thr)
                nfr += 1
                if len(xs):
                    nev += len(xs)
                    sg = np.sign(d[ys, xs])
                    # chip index now and one frame ago, per candidate phase
                    c_now = nfr * a.chip_hz / max(a.fps, 1)
                    c_prv = c_now - a.chip_hz / max(a.fps, 1)
                    for ph in range(n):
                        k1, k0 = int((c_now + ph) % n), int((c_prv + ph) % n)
                        if k1 == k0:
                            continue
                        exp = bits[k1] - bits[k0]
                        if exp == 0:
                            continue
                        for x, y, g in zip(xs, ys, sg):
                            key = (int(x), int(y))
                            v = scores.get(key)
                            if v is None:
                                v = scores[key] = [0] * n
                            v[ph] += int(g) * exp
                    for x, y in zip(xs, ys):
                        seen[(int(x), int(y))] = seen.get((int(x), int(y)), 0) + 1

                if nfr < a.window:
                    continue

                # ---- render the burst ----
                el = time.time() - t_start
                best = {k: (max(v), int(np.argmax(v))) for k, v in scores.items()}
                clus = cluster(best, n)
                img = cv2.cvtColor(cv2.resize(np.clip(last_frame.astype(np.float32) * 3, 0, 255).astype(np.uint8),
                                              (W, H), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
                img = (img * 0.35).astype(np.uint8)          # dim the scene; events are the subject
                for (x, y), cnt in seen.items():
                    col = C_POS if best[(x, y)][0] >= 0 else C_NEG
                    cv2.circle(img, (x * a.scale, y * a.scale), max(1, a.scale), col, -1)
                for i, (w_, ph, cx, cy, npx) in enumerate(clus[:6]):
                    p = (int(cx * a.scale), int(cy * a.scale))
                    col = C_CLU if i == 0 else C_DIM
                    cv2.circle(img, p, 16 + 3 * a.scale, col, 2, cv2.LINE_AA)
                    cv2.putText(img, "w%d ph%d n%d" % (w_, ph, npx), (p[0] + 20, p[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
                cap_fps = nfr / el if el else 0
                cv2.putText(img, "events %.1f/frame   coords %d   clusters %d"
                            % (nev / max(nfr, 1), len(seen), len(clus)), (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_HUD, 1, cv2.LINE_AA)
                cv2.putText(img, "burst %d frames   capture %.0f fps (asked %d)   thr %d ADU"
                            % (nfr, cap_fps, a.fps, a.thr), (8, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            C_HUD if cap_fps > a.fps * 0.6 else C_NEG, 1, cv2.LINE_AA)
                cv2.putText(img, "green = brightening   red = darkening   circle = cluster (weight, phase, px)",
                            (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_DIM, 1, cv2.LINE_AA)
                ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
                if ok:
                    try:
                        out.write(enc.tobytes())
                        out.flush()
                    except BrokenPipeError:
                        return 0
                scores, seen, nev, nfr = {}, {}, 0, 0
                t_start = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
