#!/usr/bin/env python3
"""focus_view.py -- live remote camera view on the DGX for focusing (a few fps is plenty).

Pi side is started FOR you over ssh: rpicam-vid streams MJPEG over TCP; this window decodes + shows it,
with a live sharpness number (Laplacian variance of the center crop) -- turn the lens for the MAX.

Usage:  python3 focus_view.py [--fps 5] [--shutter 2000] [--gain 4] [--pi pi@100.87.61.53] [--full]
Keys: q quit.  The number in the title is the focus metric (higher = sharper); it's what you optimize.
"""
import argparse, io, socket, subprocess, sys, threading, time
import numpy as np
from PIL import Image, ImageFilter, ImageStat
import matplotlib; matplotlib.use("GTK3Agg")
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", default="pi@100.87.61.53"); ap.add_argument("--port", type=int, default=8554)
    ap.add_argument("--fps", type=int, default=5); ap.add_argument("--shutter", type=int, default=2000)
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--full", action="store_true", help="1280x800 full-res (default 640x400 mode)")
    a = ap.parse_args()
    host = a.pi.split("@")[-1]
    mode = "1280:800:10" if a.full else "640:400:8"
    W, H = (1280, 800) if a.full else (640, 400)
    # start the streamer on the Pi (listens, then sends MJPEG frames on connect)
    # NB: no "pkill -f rpicam-vid" in this command line -- it would match (and kill) the shell running it.
    subprocess.run(["ssh", a.pi, "pkill -x rpicam-vid"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    cmd = (f"exec rpicam-vid -n -t 0 --codec mjpeg --framerate {a.fps} "
           f"--mode {mode} --width {W} --height {H} --shutter {a.shutter} --gain {a.gain} "
           f"--listen -o tcp://0.0.0.0:{a.port} 2>/dev/null")
    ssh = subprocess.Popen(["ssh", "-o", "ServerAliveInterval=10", a.pi, cmd],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sock = None
    for _ in range(40):
        try:
            sock = socket.create_connection((host, a.port), timeout=3); break
        except OSError: time.sleep(0.5)
    if sock is None: print("could not connect to the Pi stream"); ssh.kill(); sys.exit(1)

    state = {"img": None, "metric": 0.0, "n": 0}
    def reader():
        buf = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk: break
            buf += chunk
            while True:
                s_ = buf.find(b"\xff\xd8"); e = buf.find(b"\xff\xd9", s_ + 2) if s_ >= 0 else -1
                if s_ < 0 or e < 0: break
                jpg = buf[s_:e+2]; buf = buf[e+2:]
                try:
                    im = Image.open(io.BytesIO(jpg)).convert("L")
                    c = im.crop((im.width//2-80, im.height//2-80, im.width//2+80, im.height//2+80))
                    lap = c.filter(ImageFilter.Kernel((3,3), [0,1,0,1,-4,1,0,1,0], scale=1))
                    state["metric"] = ImageStat.Stat(lap).var[0]
                    state["img"] = np.asarray(im); state["n"] += 1
                except Exception: pass
    threading.Thread(target=reader, daemon=True).start()
    plt.ion()
    fig = plt.figure(figsize=(18, 11)); fig.canvas.manager.set_window_title("focus view")
    try: fig.canvas.manager.full_screen_toggle()          # go fullscreen if the backend allows
    except Exception: pass
    # layout: big number strip on top (readable across the room), image below
    ax_num = fig.add_axes([0.0, 0.72, 1.0, 0.28]); ax_num.set_axis_off()
    ax     = fig.add_axes([0.0, 0.0, 1.0, 0.72]); ax.set_axis_off()
    art = ax.imshow(np.zeros((H, W), dtype=np.uint8), cmap="gray", vmin=0, vmax=255, aspect="auto")
    ax.add_patch(plt.Rectangle((W//2-80, H//2-80), 160, 160, fill=False, ec="lime", lw=3))
    big  = ax_num.text(0.5, 0.62, "----", ha="center", va="center", fontsize=140, fontweight="bold",
                       family="monospace", color="lime", transform=ax_num.transAxes)
    sub  = ax_num.text(0.5, 0.12, "connecting...", ha="center", va="center", fontsize=28,
                       family="monospace", color="white", transform=ax_num.transAxes)
    ax_num.set_facecolor("black"); fig.patch.set_facecolor("black")
    fig.canvas.mpl_connect("key_press_event", lambda e: plt.close(fig) if e.key == "q" else None)
    title = None
    best = 0.0
    try:
        while plt.fignum_exists(fig.number):
            im = state["img"]
            if im is not None:
                art.set_data(im); m = state["metric"]; best = max(best, m)
                big.set_text(f"{m:7.0f}")
                # colour cue: green near best, yellow mid, red far
                r = m / best if best > 0 else 0
                big.set_color("lime" if r > 0.9 else ("yellow" if r > 0.6 else "tomato"))
                sub.set_text(f"best {best:7.0f}   |   turn lens for MAX   |   frame {state['n']}   {a.fps} fps   sh={a.shutter}us  g={a.gain}   |   q quits")
            fig.canvas.draw_idle(); plt.pause(0.15)
    finally:
        try: sock.close()
        except Exception: pass
        ssh.kill(); subprocess.run(["ssh", a.pi, "pkill -x rpicam-vid"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

if __name__ == "__main__": main()
