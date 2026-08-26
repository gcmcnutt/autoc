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

THE VISUAL LANGUAGE. Colour is the lifecycle state, because that is what you need at a glance while the
numbers sit in the HUD:

    green   LOCKED  - CONFIRMED and carrying a MEASURED fix THIS tick (a fresh decode)
    amber   coast   - CONFIRMED but extrapolating; still inside the §3.1 hold bound
    red     cand    - a candidate, not a confirmed track
    magenta mirror  - MULTIPATH_SUSPECT, the lower member of a same-code pair (spec §9)

    ring    the CEP, at true scale — the tracker's own claimed accuracy
    box     the APERTURE the correlator actually integrated over
    square  drawn around a track whose peak is SATURATED (flat-top centroid in use)
    cross   the prediction (xp, yp)

An INSET in the top-right magnifies the primary track (default x8 over 32 M2 px). At scale 2 the
aperture is 3 M2 px and cep is often under 0.5, so at 1:1 the ring and box are a few pixels and show
nothing — which is precisely the detail worth looking at. INTER_NEAREST throughout, so a pixel stays a
pixel and you are looking at data rather than at interpolation.

The aperture box is the one to watch during development: §2.2's ladder widens the aperture when q is
weak, binning costs sqrt(k) in background, and that cuts q further. A track death-spiralling out through
the scales (T084) is visible directly as the box growing while the ring loses its green.

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

# BcnRecord flags (src/core/record.h) — kept as literals so this tool stays standard-library-plus-cv2.
F_VALID, F_LOCK, F_HOLD, F_EXTRAP = 0x01, 0x02, 0x04, 0x08
F_MULTIPATH, F_SATURATED, F_MEASURED, F_AGC = 0x10, 0x20, 0x40, 0x80

# One colour per lifecycle state, because the state is what you need at a glance and the numbers are
# already in the HUD. BGR.
C_LOCKED   = (120, 255, 120)   # green   — CONFIRMED and decoding THIS tick
C_COAST    = (80, 200, 255)    # amber   — CONFIRMED but extrapolating; still inside the §3.1 bound
C_STALE    = (80, 80, 255)     # red     — HOLD expired / no lock: a candidate, not a track
C_MIRROR   = (255, 120, 255)   # magenta — MULTIPATH_SUSPECT: the mirror rule kept it but flagged it
C_APERTURE = (70, 70, 70)      # the correlator's actual footprint
PRED_C     = (255, 200, 60)
HUD_C      = (210, 210, 210)
DIM_C      = (120, 120, 120)

# Scale ladder: config scale_extents[] is in PLANE px at that scale, and a plane px is 4/2/1 native px
# for coarse/medium/fine — so the aperture in M2 px is extent * factor/2. Getting this wrong would draw
# a believable circle of the wrong size, which is worse than drawing none.
SCALE_NATIVE_PER_PLANE = (4.0, 2.0, 1.0)


def track_state(flags):
    """(colour, short label). Order matters: mirror and lock-loss dominate the healthy case."""
    if flags & F_MULTIPATH:
        return C_MIRROR, "MIRROR"
    if flags & F_LOCK:
        if flags & F_MEASURED:
            return C_LOCKED, "LOCKED"
        return C_COAST, "HOLD" if flags & F_HOLD else "COAST"
    return C_STALE, "CAND"


def flag_names(flags):
    out = []
    for bit, name in ((F_LOCK, "LOCK"), (F_HOLD, "HOLD"), (F_EXTRAP, "EXTRAP"),
                      (F_MULTIPATH, "MIRROR"), (F_SATURATED, "SAT"),
                      (F_MEASURED, "MEAS"), (F_AGC, "AGC")):
        if flags & bit:
            out.append(name)
    return "|".join(out) if out else "-"


def read_scale_extents(path):
    """scale_extents from the ini, so the aperture ring matches what the correlator actually used."""
    try:
        for raw in open(path):
            k, _, v = raw.partition("=")
            if k.strip() == "scale_extents":
                return [int(x) for x in v.split(";")[0].split("#")[0].split(",")]
    except OSError:
        pass
    return [24, 12, 6]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="beacon-bench.ini")
    ap.add_argument("--trackd", default="../../build/firmware/beacon-receiver/beacon_trackd")
    ap.add_argument("--source", default="live", help="live | replay:<file>")
    ap.add_argument("--scale", type=int, default=3, help="upscale of the 320x200 M2 plane")
    ap.add_argument("--gamma", type=float, default=0.45,
                    help="display gamma; the scene runs ~0.02-4 ADU against a 255 beacon, so a linear "
                         "stretch shows a black frame with one dot")
    ap.add_argument("--inset", type=int, default=8, help="magnification of the track inset (1 = off)")
    ap.add_argument("--inset-m2", type=int, default=32, help="inset crop size in M2 px")
    ap.add_argument("--print-cmd", action="store_true")
    a = ap.parse_args()

    W, H = 320 * a.scale, 200 * a.scale
    if a.print_cmd:
        print("ssh pi@<pi> 'cd ~/autoc-beacon/firmware/beacon-receiver && tools/live_view.py "
              "--config %s --scale %d' \\\n  | ffplay -f rawvideo -pixel_format bgr24 "
              "-video_size %dx%d -framerate 20 -i -" % (a.config, a.scale, W, H))
        return 0

    lut = np.array([min(255, int((v / 255.0) ** a.gamma * 255.0 + 0.5)) for v in range(256)], np.uint8)
    extents = read_scale_extents(a.config)

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
                    flags = t["flags"]
                    colour, label = track_state(flags)

                    # aperture the correlator actually integrated over — watch this widen and you are
                    # watching the scale ladder death-spiral (T084) happen live
                    sc = min(t["scale"], len(extents) - 1)
                    ap_m2 = extents[sc] * SCALE_NATIVE_PER_PLANE[min(sc, 2)] / 2.0
                    cv2.rectangle(img,
                                  (int(px - ap_m2 * a.scale / 2), int(py - ap_m2 * a.scale / 2)),
                                  (int(px + ap_m2 * a.scale / 2), int(py + ap_m2 * a.scale / 2)),
                                  C_APERTURE, 1)

                    # CEP ring: the tracker's own claimed accuracy, at true scale
                    cep_r = max(3, int(round(t["cep"] * a.scale)))
                    cv2.circle(img, (px, py), cep_r, colour, 2, cv2.LINE_AA)
                    cv2.circle(img, (px, py), 2, colour, -1, cv2.LINE_AA)   # the estimate itself
                    if flags & F_SATURATED:      # flat-top centroid in use — worth seeing, not an error
                        cv2.drawMarker(img, (px, py), colour, cv2.MARKER_SQUARE, cep_r * 2 + 8, 1)
                    cv2.drawMarker(img, (int(round((t["xp"] + 160) * a.scale)),
                                         int(round((t["yp"] + 100) * a.scale))),
                                   PRED_C, cv2.MARKER_TILTED_CROSS, 11, 1, cv2.LINE_AA)

                    cv2.putText(img, "%s %-6s q %.2f lh %.2f cep %.2f  sc%d ap%.0f  %.1fHz  %s"
                                % (t["code"], label, t["q"], t["lh"], t["cep"], t["scale"], ap_m2,
                                   t["chip_hz"], flag_names(flags)),
                                (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, colour, 1, cv2.LINE_AA)
                    y += 19
                if not rec.get("tracks"):
                    cv2.putText(img, "NO TRACK  (slots %d)" % rec.get("slots", 0), (8, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_STALE, 1, cv2.LINE_AA)
                    y += 19
                cv2.putText(img, "tick %d   margin %.1f ms" % (rec.get("tick", 0),
                                                               rec.get("deadline_us", 0) / 1000.0),
                            (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, HUD_C, 1, cv2.LINE_AA)
            # Magnified inset on the primary track. At scale 2 the aperture is 3 M2 px and cep is often
            # under 0.5, so at 1:1 the ring and the box are a few pixels and tell you nothing — which is
            # exactly the detail worth looking at. INTER_NEAREST so a pixel stays a pixel.
            if rec and rec.get("tracks") and a.inset > 1:
                t = rec["tracks"][0]
                half = a.inset_m2 // 2
                cx, cy = int(round(t["x"] + 160)), int(round(t["y"] + 100))
                x0, y0 = max(0, min(320 - a.inset_m2, cx - half)), max(0, min(200 - a.inset_m2, cy - half))
                crop = cv2.LUT(plane, lut)[y0:y0 + a.inset_m2, x0:x0 + a.inset_m2]
                if crop.size:
                    z = a.inset
                    ins = cv2.cvtColor(cv2.resize(crop, (a.inset_m2 * z, a.inset_m2 * z),
                                                  interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
                    colour, _lab = track_state(t["flags"])
                    ipx = int(round((t["x"] + 160 - x0) * z))
                    ipy = int(round((t["y"] + 100 - y0) * z))
                    sc = min(t["scale"], len(extents) - 1)
                    ap = extents[sc] * SCALE_NATIVE_PER_PLANE[min(sc, 2)] / 2.0
                    cv2.rectangle(ins, (int(ipx - ap * z / 2), int(ipy - ap * z / 2)),
                                  (int(ipx + ap * z / 2), int(ipy + ap * z / 2)), C_APERTURE, 1)
                    cv2.circle(ins, (ipx, ipy), max(2, int(round(t["cep"] * z))), colour, 1, cv2.LINE_AA)
                    cv2.drawMarker(ins, (ipx, ipy), colour, cv2.MARKER_CROSS, 9, 1, cv2.LINE_AA)
                    cv2.drawMarker(ins, (int(round((t["xp"] + 160 - x0) * z)),
                                         int(round((t["yp"] + 100 - y0) * z))),
                                   PRED_C, cv2.MARKER_TILTED_CROSS, 9, 1, cv2.LINE_AA)
                    ih, iw = ins.shape[:2]
                    if ih < H and iw < W:
                        img[8:8 + ih, W - 8 - iw:W - 8] = ins
                        cv2.rectangle(img, (W - 9 - iw, 7), (W - 8, 8 + ih), (90, 90, 90), 1)
                        cv2.putText(img, "x%d  %d M2 px" % (z, a.inset_m2), (W - 8 - iw, 8 + ih + 14),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, DIM_C, 1, cv2.LINE_AA)

            # legend
            lx = 8
            for c, txt in ((C_LOCKED, "LOCKED"), (C_COAST, "coast"), (C_STALE, "cand"),
                           (C_MIRROR, "mirror")):
                cv2.circle(img, (lx + 5, H - 36), 5, c, 2, cv2.LINE_AA)
                cv2.putText(img, txt, (lx + 14, H - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1, cv2.LINE_AA)
                lx += 14 + 8 * len(txt) + 10
            cv2.putText(img, "ring = CEP   box = aperture", (lx + 6, H - 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, DIM_C, 1, cv2.LINE_AA)
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
