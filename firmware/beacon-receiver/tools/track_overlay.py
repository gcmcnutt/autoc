#!/usr/bin/env python3
"""track_overlay.py — render a .bcnr clip with tracker state drawn on it, for subjective analysis.

The numbers in `pendulum_analyze.py` say WHAT the tracker did; this says what it LOOKED like. Built for
the workflow of "develop a tracker model, replay it, watch it" — so it takes any number of tracker runs
over the SAME clip and draws them together, which is how a change gets judged against its baseline:

    beacon_trackd --config beacon-bench.ini --source replay:clip.bcnr --emit json:- > base.json
    # ... make a change, rebuild ...
    beacon_trackd --config beacon-bench.ini --source replay:clip.bcnr --emit json:- > fixed.json
    track_overlay.py clip.bcnr --track base=base.json --track fixed=fixed.json \
                     --truth truth.csv --out compare.mp4

WHAT IT DRAWS, and the one thing worth looking for. A track is drawn as a circle of radius `cep`, and its
COLOUR carries the state that the JSON hides:

    solid  = this tick has a MEASURED fix (a fresh coherent decode)
    hollow = present but COASTING — dead reckoning on a stale velocity, no decode

On the pendulum clips the tracker is present on ~49% of ticks and measured on ~11%, so most of what looks
like tracking is hollow. Watching the circles go hollow as the swing speeds up, and the estimate drift off
the beacon while they are hollow, is the whole point of this tool.

Truth (from `pendulum_truth.py`) is a green cross. The strip charts below the image show x, y and cep with
truth overlaid, over a rolling window, so divergence is visible before the circle visibly leaves the dot.

Frames are seeked, not streamed — only the rendered ones are read, so a 17 GB clip renders in about the
time it takes to write the video.

Usage:
  track_overlay.py <clip.bcnr> --out v.mp4 [--track NAME=file.json ...] [--truth truth.csv]
                   [--speed 1.0] [--fps 30] [--start S] [--duration S] [--gain 8] [--window 10]
"""
import argparse
import bisect
import csv
import json
import struct
import sys

import cv2
import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40
SRC_FPS = 288.0
# BcnRecord flags (src/core/record.h) — same set and same colours as live_view.py, deliberately: a
# convention that differs between the live view and the replay view is worse than no convention.
F_LOCK, F_HOLD, F_EXTRAP = 0x02, 0x04, 0x08
F_MULTIPATH, F_SATURATED, F_MEASURED, F_AGC = 0x10, 0x20, 0x40, 0x80
MEASURED_FIX = F_MEASURED

C_LOCKED = (120, 255, 120)   # green   — CONFIRMED and decoding THIS tick
C_COAST  = (80, 200, 255)    # amber   — CONFIRMED but extrapolating
C_STALE  = (80, 80, 255)     # red     — candidate, not a confirmed track
C_MIRROR = (255, 120, 255)   # magenta — MULTIPATH_SUSPECT
C_APERTURE = (70, 70, 70)
PRED_C = (255, 200, 60)      # prediction cross — same as live_view.py
SCALE_NATIVE_PER_PLANE = (4.0, 2.0, 1.0)


def track_state(flags):
    if flags & F_MULTIPATH:
        return C_MIRROR, "MIRROR"
    if flags & F_LOCK:
        if flags & F_MEASURED:
            return C_LOCKED, "LOCKED"
        return C_COAST, "HOLD" if flags & F_HOLD else "COAST"
    return C_STALE, "CAND"


def flag_names(flags):
    out = [n for b, n in ((F_LOCK, "LOCK"), (F_HOLD, "HOLD"), (F_EXTRAP, "EXTRAP"),
                          (F_MULTIPATH, "MIRROR"), (F_SATURATED, "SAT"),
                          (F_MEASURED, "MEAS"), (F_AGC, "AGC")) if flags & b]
    return "|".join(out) if out else "-"


def read_scale_extents(path):
    try:
        for raw in open(path):
            k, _, v = raw.partition("=")
            if k.strip() == "scale_extents":
                return [int(x) for x in v.split(";")[0].split("#")[0].split(",")]
    except (OSError, TypeError):
        pass
    return [24, 12, 6]
DEG_PER_M2_PX = 0.304

# BGR. First colour is the first --track, and so on.
PALETTE = [(80, 80, 255), (255, 200, 60), (120, 255, 120), (255, 120, 255)]
TRUTH_C = (80, 255, 80)
CHART_H = 78
PAD = 34


def open_clip(path):
    f = open(path, "rb")
    magic, ver, hdr_b, w, h, bpp, _m = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    if magic != MAGIC:
        raise SystemExit("%#x is not BCNR" % magic)
    if bpp != 8:
        raise SystemExit("expected 8bpp, got %d" % bpp)
    return f, w, h, FRAME_HDR + w * h


def read_frame(f, w, h, fsz, i):
    f.seek(HDR + i * fsz)
    d = f.read(fsz)
    if len(d) < fsz:
        return None, None
    _r, _s, t_us, _e, _g, _fl = struct.unpack_from("<IIQIHH", d, 0)
    return t_us, np.frombuffer(d[FRAME_HDR:], dtype=np.uint8).reshape(h, w)


def load_track(path):
    out = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        t = json.loads(line)
        trk = t["tracks"][0] if t["n"] >= 1 else None
        out.append(dict(t_us=t["t_us"], trk=trk,
                        measured=bool(trk and trk["flags"] & MEASURED_FIX)))
    return out


def load_truth(path):
    rows = []
    for d in csv.DictReader(open(path)):
        rows.append(dict(t_us=int(d["t_us"]),
                         x=float(d["x_native"]) / 2 - 160, y=float(d["y_native"]) / 2 - 100))
    return rows


def m2_to_px(x, y, scale):
    """Tracker/truth M2 centre-relative -> pixel in the upscaled image.

    Two conversions, and dropping either one puts the marker in the wrong quadrant with no error:
    M2 is centre-relative and half-resolution, so native = (m2 + centre) * 2; then the render upscales
    by `scale`. Validated by eye against the beacon on pend60.bcnr.
    """
    return int(round((x + 160) * 2 * scale)), int(round((y + 100) * 2 * scale))


def draw_chart(canvas, y0, label, series, window, t_now, lo=None, hi=None):
    """series: list of (name, colour, [(t, v)], dashed)."""
    h, w = CHART_H, canvas.shape[1]
    cv2.rectangle(canvas, (0, y0), (w, y0 + h), (24, 24, 24), -1)
    cv2.line(canvas, (0, y0), (w, y0), (60, 60, 60), 1)
    vals = [v for _n, _c, pts, _d in series for t, v in pts if t_now - window <= t <= t_now]
    if not vals:
        return
    lo = min(vals) if lo is None else lo
    hi = max(vals) if hi is None else hi
    if hi - lo < 1e-6:
        hi = lo + 1.0
    pad = (hi - lo) * 0.12
    lo, hi = lo - pad, hi + pad
    for _name, colour, pts, dashed in series:
        prev = None
        for k, (t, v) in enumerate(pts):
            if not (t_now - window <= t <= t_now):
                continue
            px = int((t - (t_now - window)) / window * (w - 1))
            py = int(y0 + h - 1 - (v - lo) / (hi - lo) * (h - 2))
            if prev is not None and (not dashed or k % 2 == 0):
                cv2.line(canvas, prev, (px, py), colour, 1, cv2.LINE_AA)
            prev = (px, py)
    cv2.putText(canvas, "%s  [%.1f .. %.1f]" % (label, lo, hi), (6, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--out", required=True)
    ap.add_argument("--track", action="append", default=[], metavar="NAME=FILE")
    ap.add_argument("--truth")
    ap.add_argument("--speed", type=float, default=1.0, help="1.0 = real time, 0.25 = quarter speed")
    ap.add_argument("--fps", type=float, default=30.0, help="output video frame rate")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = to the end")
    ap.add_argument("--gain", type=float, default=1.0, help="linear gain applied before the gamma stretch")
    ap.add_argument("--gamma", type=float, default=0.45,
                    help="display gamma; the scene mean is ~4 ADU against a 255 beacon, so a LINEAR "
                         "stretch either hides the background or blows out every window. 0.45 shows both.")
    ap.add_argument("--scale", type=int, default=2, help="image upscale factor")
    ap.add_argument("--window", type=float, default=10.0, help="strip-chart window, seconds")
    ap.add_argument("--config", default=None,
                    help="ini to read scale_extents from, so the aperture box matches the run")
    a = ap.parse_args()

    f, w, h, fsz = open_clip(a.clip)
    f.seek(0, 2)
    n_frames = (f.tell() - HDR) // fsz
    t_first, _ = read_frame(f, w, h, fsz, 0)

    tracks = []
    for i, spec in enumerate(a.track):
        name, _, path = spec.partition("=")
        if not path:
            name, path = "track%d" % (i + 1), spec
        tracks.append(dict(name=name, colour=PALETTE[i % len(PALETTE)], ticks=load_track(path)))
        tracks[-1]["tt"] = [t["t_us"] for t in tracks[-1]["ticks"]]
    if len(tracks) == 1:
        # With one run the image ring is coloured BY STATE, so leaving the chart trace red would make
        # red mean two different things on the same screen. Neutral grey for the single-run case.
        tracks[0]["colour"] = (220, 220, 220)
    extents = read_scale_extents(a.config)
    truth = load_truth(a.truth) if a.truth else []
    truth_t = [p["t_us"] for p in truth]

    decim = max(1, int(round(SRC_FPS * a.speed / a.fps)))
    i0 = int(a.start * SRC_FPS)
    i1 = n_frames if not a.duration else min(n_frames, i0 + int(a.duration * SRC_FPS))
    idx = list(range(i0, i1, decim))
    ow, oh = w * a.scale, h * a.scale
    n_charts = 3
    canvas_h = oh + PAD + n_charts * CHART_H
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (ow, canvas_h))
    if not vw.isOpened():
        raise SystemExit("could not open VideoWriter for %s" % a.out)
    print("clip %d frames; rendering %d (every %dth, speed %.2fx, %.0f fps out) -> %dx%d"
          % (n_frames, len(idx), decim, a.speed, a.fps, ow, canvas_h))

    hist = {t["name"]: dict(x=[], y=[], cep=[]) for t in tracks}
    hist_truth = dict(x=[], y=[])
    written = 0
    for k, i in enumerate(idx):
        t_us, fr = read_frame(f, w, h, fsz, i)
        if fr is None:
            break
        el = (t_us - t_first) / 1e6
        img = np.clip(fr.astype(np.float32) * a.gain, 0, 255) / 255.0
        img = np.clip(np.power(img, a.gamma) * 255.0, 0, 255).astype(np.uint8)
        img = cv2.cvtColor(cv2.resize(img, (ow, oh), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)

        if truth:
            j = bisect.bisect_left(truth_t, t_us)
            cand = [truth[m] for m in (j - 1, j) if 0 <= m < len(truth) and abs(truth_t[m] - t_us) < 8000]
            if cand:
                p = min(cand, key=lambda q: abs(q["t_us"] - t_us))
                px, py = m2_to_px(p["x"], p["y"], a.scale)
                cv2.drawMarker(img, (px, py), TRUTH_C, cv2.MARKER_CROSS, 22, 1, cv2.LINE_AA)
                hist_truth["x"].append((el, p["x"]))
                hist_truth["y"].append((el, p["y"]))

        y_text = 22
        for t in tracks:
            j = bisect.bisect_right(t["tt"], t_us) - 1     # hold the most recent tick
            tick = t["ticks"][j] if j >= 0 and t_us - t["tt"][j] < 200000 else None
            trk = tick["trk"] if tick else None
            if trk:
                px, py = m2_to_px(trk["x"], trk["y"], a.scale)
                flags = trk["flags"]
                st_colour, label = track_state(flags)
                # With ONE run, colour carries the lifecycle state (same language as live_view.py).
                # With several, colour has to identify the run instead — that is the point of an A/B —
                # so the state moves to the fill and the label.
                colour = st_colour if len(tracks) == 1 else t["colour"]
                sc = min(trk["scale"], len(extents) - 1)
                ap_m2 = extents[sc] * SCALE_NATIVE_PER_PLANE[min(sc, 2)] / 2.0
                apx = ap_m2 * 2 * a.scale / 2.0          # M2 px -> native -> image
                cv2.rectangle(img, (int(px - apx), int(py - apx)), (int(px + apx), int(py + apx)),
                              C_APERTURE, 1)
                if label == "LOCKED":
                    # Full-span crosshairs on a measured fix -- these ARE the coordinates being
                    # emitted. Absence means coasting. Same rule as live_view.py.
                    ov = img.copy()
                    cv2.line(ov, (0, py), (ow, py), C_LOCKED, 1, cv2.LINE_AA)
                    cv2.line(ov, (px, 0), (px, oh), C_LOCKED, 1, cv2.LINE_AA)
                    cv2.addWeighted(ov, 0.55, img, 0.45, 0, img)
                    cv2.putText(img, "y %+.2f" % trk["y"], (ow - 96, max(14, py - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_LOCKED, 1, cv2.LINE_AA)
                    cv2.putText(img, "x %+.2f" % trk["x"], (min(px + 6, ow - 86), oh - 46),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_LOCKED, 1, cv2.LINE_AA)
                r = max(3, int(round(trk["cep"] * 2 * a.scale)))
                cv2.circle(img, (px, py), r, colour, 2 if tick["measured"] else 1, cv2.LINE_AA)
                cv2.circle(img, (px, py), 2, colour, -1, cv2.LINE_AA)
                if flags & F_SATURATED:
                    cv2.drawMarker(img, (px, py), colour, cv2.MARKER_SQUARE, r * 2 + 8, 1)
                qx, qy = m2_to_px(trk["xp"], trk["yp"], a.scale)
                cv2.drawMarker(img, (qx, qy), PRED_C, cv2.MARKER_TILTED_CROSS, 9, 1, cv2.LINE_AA)
                hist[t["name"]]["x"].append((el, trk["x"]))
                hist[t["name"]]["y"].append((el, trk["y"]))
                hist[t["name"]]["cep"].append((el, trk["cep"]))
                txt = "%-8s %-6s q %.2f lh %.2f cep %.2f sc%d ap%.0f  %s" % (
                    t["name"], label, trk["q"], trk["lh"], trk["cep"], trk["scale"], ap_m2,
                    flag_names(flags))
            else:
                colour = C_STALE if len(tracks) == 1 else t["colour"]
                txt = "%-8s %-6s --" % (t["name"], "NOTRK")
            cv2.putText(img, txt, (8, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.46, colour, 1, cv2.LINE_AA)
            y_text += 19

        cv2.putText(img, "t %6.2fs   frame %6d   %.2fx" % (el, i, a.speed),
                    (8, oh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)
        lx = 8
        if len(tracks) == 1:
            for c, lab in ((C_LOCKED, "LOCKED"), (C_COAST, "coast"), (C_STALE, "cand"),
                           (C_MIRROR, "mirror")):
                cv2.circle(img, (lx + 5, oh - 34), 5, c, 2, cv2.LINE_AA)
                cv2.putText(img, lab, (lx + 14, oh - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1, cv2.LINE_AA)
                lx += 14 + 8 * len(lab) + 10
            cv2.putText(img, "ring = CEP   box = aperture", (lx + 6, oh - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
        else:
            cv2.putText(img, "colour = run (A/B)   bold ring = MEASURED fix   box = aperture",
                        (8, oh - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1, cv2.LINE_AA)

        canvas = np.zeros((canvas_h, ow, 3), np.uint8)
        canvas[:oh] = img
        y0 = oh + PAD
        for label, key in (("x  (M2 px)", "x"), ("y  (M2 px)", "y"), ("cep  (M2 px)", "cep")):
            series = [(t["name"], t["colour"], hist[t["name"]][key], False) for t in tracks]
            if key in hist_truth:
                series.append(("truth", TRUTH_C, hist_truth[key], True))
            draw_chart(canvas, y0, label, series, a.window, el,
                       lo=0.0 if key == "cep" else None)
            y0 += CHART_H
        cv2.putText(canvas, "green dashed = truth", (ow - 190, oh + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, TRUTH_C, 1, cv2.LINE_AA)
        vw.write(canvas)
        written += 1
        if written % 200 == 0:
            print("  %d/%d" % (written, len(idx)))
    vw.release()
    print("wrote %s (%d frames, %.1f s of video)" % (a.out, written, written / a.fps))


if __name__ == "__main__":
    sys.exit(main())
