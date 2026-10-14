#!/usr/bin/env python3
"""
bifilar_period.py — moment of inertia from a bifilar-pendulum video (043 T116, 2026-10-13).

Extracts the torsional period from a phone video with ffmpeg (no OpenCV needed): frames are decoded to
small grayscale arrays, the horizontal centroid of the darkest pixels (the black fuselage / motor pod)
is tracked per frame, the trace is detrended, and the period comes from the autocorrelation peak,
cross-checked against the FFT peak and against direct zero-crossing counting. Then the inertia:

  general (strings may converge):  I_total = m_total · g · r_t · r_b · T² / (4 π² h)
      r_t = d_top/2, r_b = d_bot/2, h = sqrt(L_s² − (r_b − r_t)²)   (vertical height of the strings)
  parallel strings (d_top = d_bot = D, h = L) reduces to the textbook  I = m g D² T² / (16 π² L).

  I_craft = I_total − I_rig,  I_rig = m_rig · d_rig² / 12  (uniform bar, strings at its ends; add any
  other attached mass explicitly with --rig-extra "m,r" terms, each m·r²).

Article 1 rig of 2026-10-13: d_top = 2 × 6 3/8 in, d_bot = 34 1/4 in, L_s = 20.4 in, craft 510 g bare.

Usage:
  bifilar_period.py VIDEO --m-craft 0.510 --m-rig 0.030 --d-top-in 12.75 --d-bot-in 34.25 --ls-in 20.4
      [--crop x,y,w,h] [--fps F] [--start S --end E] [--width 320] [--dump trace.csv]
"""
import argparse
import json
import math
import subprocess
import sys

import numpy as np

PIXFMT = "gray"   # set from --track in main(); frames() reads it


def probe(video):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height,r_frame_rate,avg_frame_rate,duration,nb_frames",
                          "-of", "json", video], capture_output=True, text=True, check=True).stdout
    s = json.loads(out)["streams"][0]
    num, den = s.get("avg_frame_rate", s["r_frame_rate"]).split("/")
    fps = float(num) / float(den) if float(den) else float(num)
    return int(s["width"]), int(s["height"]), fps, float(s.get("duration", 0) or 0)


def frames(video, width, crop, start, end):
    vf = []
    if crop:
        x, y, w, h = crop; vf.append(f"crop={w}:{h}:{x}:{y}")
    vf.append(f"scale={width}:-2")
    cmd = ["ffmpeg", "-v", "error"]
    if start is not None: cmd += ["-ss", str(start)]
    if end is not None: cmd += ["-to", str(end)]
    cmd += ["-i", video, "-vf", ",".join(vf), "-pix_fmt", PIXFMT, "-f", "rawvideo", "-"]
    p = subprocess.run(cmd, capture_output=True, check=True)
    raw = np.frombuffer(p.stdout, dtype=np.uint8)
    # height after scale: need it — probe one frame's size via ffprobe of the same filter is awkward; infer
    # from the rawvideo length and a second ffprobe of width/height ratio.
    return raw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--m-craft", type=float, required=True, help="kg, as flown (bare)")
    ap.add_argument("--m-rig", type=float, default=0.0, help="kg of the bar the strings attach to")
    ap.add_argument("--d-rig-in", type=float, default=None, help="bar length between string points, inches (default d_bot)")
    ap.add_argument("--rig-extra", action="append", default=[], help="extra attached mass as 'm_kg,r_m' (adds m r^2)")
    ap.add_argument("--d-top-in", type=float, required=True)
    ap.add_argument("--d-bot-in", type=float, required=True)
    ap.add_argument("--ls-in", type=float, required=True, help="string length, inches")
    ap.add_argument("--crop", default=None, help="x,y,w,h in source pixels (restrict to the craft)")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--fps", type=float, default=None, help="override probed frame rate")
    ap.add_argument("--start", type=float, default=None); ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--dark-frac", type=float, default=0.02, help="fraction of darkest pixels tracked")
    ap.add_argument("--band", default="1.0,2.5", help="Hz band the twist is read in (sway sits below it)")
    ap.add_argument("--track", default="dark", choices=["dark", "yellow", "bright"],
                    help="what to follow: darkest pixels (black pod), yellow tape/elevon, or brightest pixels")
    ap.add_argument("--dump", default=None, help="write the per-frame trace CSV")
    a = ap.parse_args()

    W, H, fps_p, dur = probe(a.video)
    fps = a.fps or fps_p
    # phone videos carry a rotation tag and ffmpeg auto-rotates on decode: find the DISPLAYED size by
    # decoding one frame, rather than trusting the stored width/height
    one = subprocess.run(["ffmpeg", "-v", "error", "-i", a.video, "-frames:v", "1", "-pix_fmt", "gray", "-f", "rawvideo", "-"],
                         capture_output=True).stdout
    if len(one) == W * H:
        # ambiguous between WxH and HxW; use the rotation side data
        rot = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                              "stream_side_data=rotation:stream_tags=rotate", "-of", "default=nw=1:nk=1", a.video],
                             capture_output=True, text=True).stdout.strip().split()
        if any(abs(int(float(r))) % 180 == 90 for r in rot if r.lstrip("-").replace(".", "").isdigit()):
            W, H = H, W
    crop = tuple(int(v) for v in a.crop.split(",")) if a.crop else None
    cw, ch = (crop[2], crop[3]) if crop else (W, H)
    sw = a.width; sh = int(round(ch * sw / cw / 2)) * 2
    global PIXFMT
    PIXFMT = "gray" if a.track == "dark" or a.track == "bright" else "rgb24"
    ch = 1 if PIXFMT == "gray" else 3
    raw = frames(a.video, sw, crop, a.start, a.end)
    n = len(raw) // (sw * sh * ch)
    if n < 30:
        sys.exit(f"only {n} frames decoded (expected ~{sw}x{sh}x{ch}) — check --crop/--width")
    F = raw[: n * sw * sh * ch].reshape(n, sh, sw, ch).astype(np.float32)
    print(f"{a.video}: {W}x{H} @ {fps:.2f} fps, {dur:.1f} s; decoded {n} frames at {sw}x{sh}, tracking '{a.track}'")

    # per-frame centroid (x and y) of the tracked pixels; the twist may show on either image axis
    # depending on where the camera is, so take the axis whose estimators agree (see below)
    k = max(20, int(a.dark_frac * sw * sh))
    cx = np.empty(n); cy = np.empty(n)
    for i in range(n):
        if a.track == "yellow":
            R, G, B = F[i, :, :, 0], F[i, :, :, 1], F[i, :, :, 2]
            score = -(R + G - 2.0 * B)            # most negative = most yellow
        elif a.track == "bright":
            score = -F[i, :, :, 0]
        else:
            score = F[i, :, :, 0]
        f = score.ravel()
        idx = np.argpartition(f, k)[:k]
        cx[i] = np.mean(idx % sw); cy[i] = np.mean(idx // sw)
    t = np.arange(n) / fps
    win = max(5, int(fps * 2.0) | 1)
    kern = np.ones(win) / win
    def detrend(v):
        base = np.convolve(np.pad(v, win // 2, mode="edge"), kern, mode="valid")
        return v - base[:n]
    dx, dy = detrend(cx), detrend(cy)
    use_y = np.std(dy) > np.std(dx)
    x = dy if use_y else dx
    trace = cy if use_y else cx
    print(f"tracking darkest {k} px per frame: detrended std x {np.std(dx):.2f} px, y {np.std(dy):.2f} px → using {'y' if use_y else 'x'}")
    if a.dump:
        np.savetxt(a.dump, np.c_[t, cx, cy, dx, dy], delimiter=",", header="t_s,cx_px,cy_px,dx_px,dy_px", comments="")

    # The hanging rig also SWAYS as a plain pendulum (T = 2π√(h/g), ~1.3 s at h = 0.44 m), usually with more
    # amplitude than the twist. Separate them: the twist is read in a band around the expected torsional
    # frequency (--band, default 1.0–2.5 Hz), on BOTH image axes, three estimators each; the sway is reported
    # for the sanity check. Article-1 I_yy (2026-10-13): twist 0.70–0.72 s on all estimators/axes, sway 1.25 s.
    lo_f, hi_f = (float(v) for v in a.band.split(","))
    def bandpass(v, lo, hi):
        X = np.fft.rfft(v - v.mean()); fr = np.fft.rfftfreq(len(v), 1 / fps); X[(fr < lo) | (fr > hi)] = 0
        return np.fft.irfft(X, len(v))
    def estimates(v):
        s = np.sign(v); zc = np.where((s[:-1] <= 0) & (s[1:] > 0))[0]
        Tz = (zc[-1] - zc[0]) / fps / (len(zc) - 1) if len(zc) > 2 else float("nan")
        ac = np.correlate(v, v, "full")[n - 1:]; ac /= ac[0]
        l0 = int(0.3 / hi_f * fps) + 1; h0 = int(min(n // 2, 1.5 / lo_f * fps))
        fm = l0 + int(np.argmin(ac[l0:h0])); pk = fm + int(np.argmax(ac[fm:h0])); Ta = pk / fps
        sp = np.abs(np.fft.rfft(v * np.hanning(n))) ** 2; fr = np.fft.rfftfreq(n, 1 / fps)
        m = (fr >= lo_f) & (fr <= hi_f); k = np.where(m)[0][int(np.argmax(sp[m]))]
        if 0 < k < len(sp) - 1:
            la, lb, lc = np.log(sp[k - 1:k + 2] + 1e-12); p = 0.5 * (la - lc) / (la - 2 * lb + lc)
            fpk = fr[k] + p * (fr[1] - fr[0])
        else:
            fpk = fr[k]
        return Tz, (len(zc) - 1 if len(zc) > 2 else 0), Ta, 1 / fpk, np.percentile(np.abs(v), 95)
    results = []
    for name, v in (("x", dx), ("y", dy)):
        Tz, nc, Ta, Tf, amp = estimates(bandpass(v, lo_f, hi_f))
        print(f"twist, axis {name}, band {lo_f}-{hi_f} Hz: zero-cross {Tz:.4f} s over {nc} cycles | autocorr {Ta:.4f} s | FFT-interp {Tf:.4f} s | amp95 {amp:.1f} px")
        results.append((name, amp, [Tz, Ta, Tf]))
    sway_hi = 0.9 * lo_f
    sway = estimates(bandpass(dx if np.std(dx) > np.std(dy) else dy, 0.4, sway_hi))
    print(f"sway (0.4-{sway_hi:.2f} Hz): FFT-interp {sway[3]:.3f} s   (plain pendulum 2π√(h/g) is printed with the geometry below)")
    # Use the axis whose three estimators AGREE (the other axis usually carries sway leakage — article 1's x axis
    # read 0.69/0.68/0.59 while y read 0.703/0.717/0.721); amplitude only breaks ties. T = median, spread = range.
    def spread(r): Ts = r[2]; return (max(Ts) - min(Ts)) / np.median(Ts)
    name_best, amp_best, Ts = min(results, key=lambda r: (round(spread(r), 2), -r[1]))
    T = float(np.median(Ts)); T_lo, T_hi = min(Ts), max(Ts)
    print(f"→ twist period T = {T:.4f} s from axis {name_best} (estimator range {T_lo:.4f}–{T_hi:.4f})")
    if (T_hi - T_lo) / T > 0.08:
        print("⚠️ estimators spread by >8 % — inspect --dump, try a tighter --crop, --start/--end past the release transient, or adjust --band")

    g = 9.81; inch = 0.0254
    rt = a.d_top_in * inch / 2; rb = a.d_bot_in * inch / 2; Ls = a.ls_in * inch
    h = math.sqrt(max(Ls ** 2 - (rb - rt) ** 2, 1e-9))
    m_tot = a.m_craft + a.m_rig
    I_tot = m_tot * g * rt * rb * T ** 2 / (4 * math.pi ** 2 * h)
    d_rig = (a.d_rig_in or a.d_bot_in) * inch
    I_rig = a.m_rig * d_rig ** 2 / 12
    for e in a.rig_extra:
        m_e, r_e = (float(v) for v in e.split(",")); I_rig += m_e * r_e ** 2; m_tot += m_e
    I_craft = I_tot - I_rig
    print(f"geometry: r_t {rt:.4f} m, r_b {rb:.4f} m, L_s {Ls:.4f} m, h {h:.4f} m (plain-pendulum sway 2π√(h/g) = {2*math.pi*math.sqrt(h/g):.3f} s); m_total {m_tot:.3f} kg")
    I_lo = m_tot * g * rt * rb * T_lo ** 2 / (4 * math.pi ** 2 * h) - I_rig
    I_hi = m_tot * g * rt * rb * T_hi ** 2 / (4 * math.pi ** 2 * h) - I_rig
    print(f"I_total {I_tot:.5f} kg·m²  −  I_rig {I_rig:.5f}  =  I_craft {I_craft:.5f} kg·m²  [{I_lo:.5f}–{I_hi:.5f} over the estimator spread]   (radius of gyration {math.sqrt(max(I_craft,0)/a.m_craft)*100:.1f} cm)")
    # sensitivity
    print(f"sensitivity: dT ±1 frame → I ×{((T+1/fps)/T)**2:.3f}/{((T-1/fps)/T)**2:.3f}; d_bot ±2 mm → ±{2*0.002/(2*rb)*100:.1f} %; L_s ±2 mm → ∓{0.002/h*100:.1f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
