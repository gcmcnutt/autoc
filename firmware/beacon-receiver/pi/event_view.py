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
    ap.add_argument("--thr", type=int, default=5, help="fixed event threshold, ADU of frame-to-frame delta")
    ap.add_argument("--target-events", type=float, default=0.0,
                    help="ADAPTIVE threshold: aim for this many events/frame and let the threshold find "
                         "itself. A FIXED threshold cannot serve two scenes -- measured, thr=3 gives 19 "
                         "events/frame in a dark room and 12500 in a lit one. Setting the RATE is the "
                         "stable control; the threshold is just where that lands.")
    ap.add_argument("--lock-weight", type=float, default=40.0,
                    help="minimum leader weight before crosshairs are drawn")
    ap.add_argument("--lock-margin", type=float, default=3.0,
                    help="leader must beat the runner-up by this factor")
    ap.add_argument("--max-events", type=int, default=400,
                    help="cap events per frame, keeping the strongest by magnitude")
    ap.add_argument("--window", type=int, default=90, help="frames per analysis burst")
    ap.add_argument("--code", default="B")
    ap.add_argument("--chip-hz", type=float, default=120.0)
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("--quality", type=int, default=88)
    a = ap.parse_args()

    bits = [1 if c == "1" else 0 for c in CODE[a.code.upper()]]
    thr = a.thr
    n = len(bits)
    W, H = a.width * a.scale, a.height * a.scale

    cmd = ["rpicam-vid", "-n", "-t", "0", "--codec", "mjpeg", "--framerate", str(a.fps),
           "--width", str(a.width), "--height", str(a.height), "--denoise", "off",
           "--shutter", str(a.shutter), "--gain", str(a.gain), "-o", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=1 << 22)
    out = sys.stdout.buffer
    buf = b""
    prev = None
    score = np.zeros((n, a.width * a.height), np.int32)
    seen_arr = np.zeros(a.width * a.height, np.int32)
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
                ad = np.abs(d)
                ys, xs = np.nonzero(ad >= thr)
                nfr += 1
                nraw = len(xs)
                if nraw > a.max_events:
                    # Cap the per-frame event count. This is not just a speed guard: 12500 events in a
                    # lit room is not 12500 candidates, it is a threshold that has fallen below the
                    # noise, and the strongest few hundred are where any real target lives. Keeping the
                    # top-N by magnitude bounds the work AND discards the part that was never signal.
                    keep = np.argpartition(ad[ys, xs], -a.max_events)[-a.max_events:]
                    ys, xs = ys[keep], xs[keep]
                if len(xs):
                    nev += len(xs)
                    sg = np.sign(d[ys, xs]).astype(np.int16)
                    flat = ys.astype(np.int64) * a.width + xs
                    # Vectorised over phases. The Python inner loop this replaces was O(events x phases)
                    # and stalled the viewer outright at high event counts -- 388000 iterations a frame.
                    c_now = nfr * a.chip_hz / max(a.fps, 1)
                    c_prv = c_now - a.chip_hz / max(a.fps, 1)
                    for ph in range(n):
                        k1, k0 = int((c_now + ph) % n), int((c_prv + ph) % n)
                        if k1 == k0:
                            continue
                        exp = bits[k1] - bits[k0]
                        if exp == 0:
                            continue
                        np.add.at(score[ph], flat, sg * exp)
                    np.add.at(seen_arr, flat, 1)

                if a.target_events > 0:
                    # Servo the threshold on the ACHIEVED event rate. Gain is deliberately not a control
                    # here: it scales signal and noise together, so it only moves where the threshold has
                    # to sit (measured: gain 1->4 doubles the event rate at a fixed threshold for no
                    # detection benefit). Exposure is the real lever and it points SHORT -- background
                    # accumulates with it while the beacon, a bright pulsed source, has huge headroom.
                    err = nraw - a.target_events
                    if err > 0 and thr < 200:
                        thr += 1 if err < a.target_events else 3
                    elif err < 0 and thr > 2:
                        thr -= 1

                if nfr < a.window:
                    continue

                # ---- render the burst ----
                el = time.time() - t_start
                fired = np.nonzero(seen_arr)[0]
                best = {}
                if len(fired):
                    sub = score[:, fired]
                    bph = np.argmax(sub, axis=0)
                    bsc = sub[bph, np.arange(len(fired))]
                    for i, fi in enumerate(fired):
                        best[(int(fi % a.width), int(fi // a.width))] = (int(bsc[i]), int(bph[i]))
                clus = cluster(best, n)
                img = cv2.cvtColor(cv2.resize(np.clip(last_frame.astype(np.float32) * 3, 0, 255).astype(np.uint8),
                                              (W, H), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
                img = (img * 0.35).astype(np.uint8)          # dim the scene; events are the subject
                for (x, y), bv in best.items():
                    col = C_POS if bv[0] >= 0 else C_NEG
                    cv2.circle(img, (x * a.scale, y * a.scale), max(1, a.scale), col, -1)
                # Full-span crosshairs on the DOMINANT cluster, same meaning as live_view.py's: these
                # are the coordinates the detector would emit. Drawn only when the leader clearly wins --
                # weight above a floor AND a margin over the runner-up -- so their ABSENCE means the
                # detector has candidates but no confident pick, which is the state worth seeing.
                if clus:
                    w0, ph0, cx0, cy0, npx0 = clus[0]
                    w1 = clus[1][0] if len(clus) > 1 else 0
                    if w0 >= a.lock_weight and w0 >= a.lock_margin * max(w1, 1):
                        px0, py0 = int(cx0 * a.scale), int(cy0 * a.scale)
                        ov = img.copy()
                        cv2.line(ov, (0, py0), (W, py0), C_POS, 1, cv2.LINE_AA)
                        cv2.line(ov, (px0, 0), (px0, H), C_POS, 1, cv2.LINE_AA)
                        cv2.addWeighted(ov, 0.55, img, 0.45, 0, img)
                        cv2.putText(img, "y %+.2f" % (cy0 / (640.0 / a.width) / 2 - 200),
                                    (W - 104, max(14, py0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                    C_POS, 1, cv2.LINE_AA)
                        cv2.putText(img, "x %+.2f" % (cx0 / (640.0 / a.width) / 2 - 320),
                                    (min(px0 + 6, W - 92), H - 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                    C_POS, 1, cv2.LINE_AA)
                        cv2.putText(img, "LOCK  w%d ph%d  %dx margin" % (w0, ph0, int(w0 / max(w1, 1))),
                                    (8, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_POS, 1, cv2.LINE_AA)
                    else:
                        cv2.putText(img, "no confident pick (leader w%d vs runner-up w%d)" % (w0, w1),
                                    (8, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_DIM, 1, cv2.LINE_AA)

                for i, (w_, ph, cx, cy, npx) in enumerate(clus[:6]):
                    p = (int(cx * a.scale), int(cy * a.scale))
                    col = C_CLU if i == 0 else C_DIM
                    cv2.circle(img, p, 16 + 3 * a.scale, col, 2, cv2.LINE_AA)
                    cv2.putText(img, "w%d ph%d n%d" % (w_, ph, npx), (p[0] + 20, p[1] - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
                cap_fps = nfr / el if el else 0
                cv2.putText(img, "events %.1f/frame   coords %d   clusters %d"
                            % (nev / max(nfr, 1), len(best), len(clus)), (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_HUD, 1, cv2.LINE_AA)
                cv2.putText(img, "burst %d frames   capture %.0f fps (asked %d)   thr %d ADU%s"
                            % (nfr, cap_fps, a.fps, thr,
                               "  [auto -> %.0f ev/frame]" % a.target_events if a.target_events else ""),
                            (8, 44),
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
                score[:] = 0; seen_arr[:] = 0; nev = 0; nfr = 0
                t_start = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
