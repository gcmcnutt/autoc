#!/usr/bin/env python3
"""live_view.py — live camera + tracker overlay, streamed as raw video for ffplay.

RUNS ON THE PI. The camera is single-access and `beacon_trackd` owns it, so a live view cannot come from
a second capture process (that is why pi/preview.py and the tracker can never run together). Instead this
spawns the tracker itself with `--preview`, which emits the M2 plane as its own JSON line beside each
record, composites the two, and writes raw BGR frames to stdout. Watch it from anywhere:

    ssh pi@<pi> 'cd ~/autoc-beacon/firmware/beacon-receiver &&
                 tools/live_view.py --config beacon-bench.ini' \\
      | ffplay -f rawvideo -pixel_format bgr24 -video_size 960x600 -framerate 20 -i -

960x600 is the default (M2 320x200 at --scale 3); pass --print-cmd to get the matching ffplay line with
the geometry filled in, because a wrong -video_size shears the image rather than erroring.

WHY THE COORDINATES ARE SAFE HERE. The preview plane is at M2 resolution and track x/y are M2
centre-relative, so a marker is just (x+160, y+100) — one addition, no scale factor to get wrong. That is
deliberate: track_overlay.py had a marker bug (a stray /2) that put every marker in the wrong quadrant and
produced no error at all, which cost a false "the tracker locked onto a reflection" conclusion.

WHAT TO LOOK FOR. Same convention as track_overlay.py: a track is a circle of radius `cep`, drawn SOLID
when that tick carries a MEASURED fix and HOLLOW when it is coasting on a stale velocity with no fresh
decode. On a moving target most circles are hollow, and that is the thing worth watching.

COST. --preview costs one pass over the frame per tick, the same as --field-map (measured ~3.2 ms against
~39 ms of margin). Fine for looking at; do not leave it on for an envelope run where the deadline margin
is itself the measurement.
"""
import argparse
import base64
import json
import os
import signal
import subprocess
import sys

import cv2
import numpy as np

MEASURED_FIX = 0x40
TRUTH_C = (80, 255, 80)
TRACK_C = (80, 80, 255)
PRED_C = (255, 200, 60)
HUD_C = (210, 210, 210)
DIM_C = (120, 120, 120)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="beacon-bench.ini")
    ap.add_argument("--trackd", default="../../build/firmware/beacon-receiver/beacon_trackd")
    ap.add_argument("--source", default="live", help="live | replay:<file>")
    ap.add_argument("--scale", type=int, default=3, help="upscale of the 320x200 M2 plane")
    ap.add_argument("--gamma", type=float, default=0.45,
                    help="display gamma; the scene runs ~0.02-4 ADU against a 255 beacon, so a linear "
                         "stretch shows a black frame with one dot")
    ap.add_argument("--print-cmd", action="store_true")
    a = ap.parse_args()

    W, H = 320 * a.scale, 200 * a.scale
    if a.print_cmd:
        print("ssh pi@<pi> 'cd ~/autoc-beacon/firmware/beacon-receiver && tools/live_view.py "
              "--config %s --scale %d' \\\n  | ffplay -f rawvideo -pixel_format bgr24 "
              "-video_size %dx%d -framerate 20 -i -" % (a.config, a.scale, W, H))
        return 0

    lut = np.array([min(255, int((v / 255.0) ** a.gamma * 255.0 + 0.5)) for v in range(256)], np.uint8)

    cmd = [a.trackd, "--config", a.config, "--source", a.source, "--emit", "json:-", "--preview"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=sys.stderr, bufsize=1 << 20)
    out = sys.stdout.buffer
    plane = None
    rec = None
    nframes = 0
    try:
        for line in proc.stdout:
            line = line.strip()
            if not line.startswith(b"{"):
                continue
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if "preview_b64" in d:
                pw, ph = d["preview_w"], d["preview_h"]
                raw = base64.b64decode(d["preview_b64"])
                if len(raw) != pw * ph:
                    continue
                plane = np.frombuffer(raw, np.uint8).reshape(ph, pw)
            elif "tracks" in d:
                rec = d
                continue                          # the preview line follows; composite when it lands
            else:
                continue

            if plane is None:
                continue
            img = cv2.cvtColor(cv2.resize(cv2.LUT(plane, lut), (W, H),
                                          interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
            # field centre
            cv2.drawMarker(img, (W // 2, H // 2), (45, 45, 45), cv2.MARKER_CROSS, 18, 1)

            y = 20
            if rec:
                for t in rec.get("tracks", []):
                    # M2 centre-relative -> preview pixel: one addition, no scale factor (see docstring)
                    px = int(round((t["x"] + 160) * a.scale))
                    py = int(round((t["y"] + 100) * a.scale))
                    meas = bool(t["flags"] & MEASURED_FIX)
                    r = max(4, int(round(t["cep"] * a.scale)))
                    cv2.circle(img, (px, py), r, TRACK_C, -1 if meas else 2, cv2.LINE_AA)
                    cv2.drawMarker(img, (int(round((t["xp"] + 160) * a.scale)),
                                         int(round((t["yp"] + 100) * a.scale))),
                                   PRED_C, cv2.MARKER_TILTED_CROSS, 11, 1, cv2.LINE_AA)
                    cv2.putText(img, "%s %-8s q %.2f lh %.2f cep %.2f sc %d  %.0fHz"
                                % (t["code"], "MEASURED" if meas else "coast", t["q"], t["lh"],
                                   t["cep"], t["scale"], t["chip_hz"]),
                                (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                TRACK_C if meas else DIM_C, 1, cv2.LINE_AA)
                    y += 20
                if not rec.get("tracks"):
                    cv2.putText(img, "NO TRACK  (slots %d)" % rec.get("slots", 0), (8, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, DIM_C, 1, cv2.LINE_AA)
                    y += 20
                cv2.putText(img, "tick %d   margin %.1f ms" % (rec.get("tick", 0),
                                                               rec.get("deadline_us", 0) / 1000.0),
                            (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, HUD_C, 1, cv2.LINE_AA)
            cv2.putText(img, "solid = MEASURED fix   hollow = coasting", (8, H - 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, DIM_C, 1, cv2.LINE_AA)
            try:
                out.write(img.tobytes())
                out.flush()
            except BrokenPipeError:
                break
            nframes += 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        print("live_view: %d frames composited" % nframes, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
