#!/usr/bin/env python3
"""fps_probe.py -- what frame rate does the OV9281 actually sustain on this Pi, per mode?

Runs rpicam-raw bursts to tmpfs and counts delivered frames (raw = no ISP/encoder in the loop --
the only honest path on Pi 3-class hardware). Prints a table; exits with the best sustained fps.

Usage (on the Pi):  python3 fps_probe.py [--modes 640:400:8,320:200:8] [--fps 200,250,300,...] [--secs 3]
Note: the mainline ov9281 driver defines ONLY 1280x800 / 1280x720 / 640x400 (v4l2-ctl
--list-subdev-framesizes). Requesting 320x200 maps to 640x400 and over-clocks it -> frontend timeout.
A real 320x200 mode needs a driver mode-table entry (the InnoMaker Bullseye driver route).
"""
import argparse, os, subprocess, sys, time

def burst(mode, fps, secs, out="/dev/shm/fps_probe.raw"):
    w, h, bits = (int(x) for x in mode.split(":"))
    if os.path.exists(out): os.remove(out)
    t0 = time.time()
    r = subprocess.run(["rpicam-raw", "-n", "-t", str(secs*1000), "--mode", mode, "--framerate", str(fps),
                        "-o", out], capture_output=True, text=True)
    dt = time.time() - t0
    log = (r.stdout + r.stderr).lower()
    errs = log.count("timed out") + log.count("error")
    # HONESTY: the driver may map an undefined size (e.g. 320x200) to its nearest real mode (640x400)
    # and rpicam-raw then writes REAL-mode-sized frames. Divide by the requested size and the count
    # inflates 4x. Detect that: if the file isn't an integer multiple of the requested frame size but IS
    # of 640x400, report the real mode. (Verified 2026-08-16: 320:200 request -> 640x400 frames.)
    fsz = w*h*(1 if bits == 8 else 2)
    size = os.path.getsize(out) if os.path.exists(out) else 0
    real = f"{w}x{h}"
    if size and size % fsz == 0 and (w, h) != (640, 400) and size % (640*400) == 0 and size//(640*400) < size//fsz:
        fsz = 640*400; real = "640x400(mapped)"
    n = size//fsz if size else 0
    if os.path.exists(out): os.remove(out)
    return n, errs, dt, real

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modes", default="640:400:8,320:200:8")
    ap.add_argument("--fps", default="200,250,270,290,310,400,453")
    ap.add_argument("--secs", type=int, default=3)
    a = ap.parse_args()
    best = 0
    print(f"{'mode':>11} {'req':>5} {'frames':>7} {'fps':>6} {'errs':>5}  verdict")
    for mode in a.modes.split(","):
        for fps in (int(x) for x in a.fps.split(",")):
            n, errs, dt, real = burst(mode, fps, a.secs)
            got = n / a.secs
            ok = errs == 0 and got > 0.9*fps
            if ok: best = max(best, got)
            print(f"{mode:>11} {fps:>5} {n:>7} {got:>6.0f} {errs:>5}  {'SUSTAINED' if ok else ('FAIL' if errs or n < 5 else 'short')}  [{real}]")
    print(f"BEST SUSTAINED ~{best:.0f} fps")
    return 0
if __name__ == "__main__":
    sys.exit(main())
