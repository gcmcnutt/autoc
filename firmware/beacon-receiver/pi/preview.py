#!/usr/bin/env python3
"""preview.py — a live browser view of the camera, for physically placing things in the scene.

Runs ON THE PI. Spawns rpicam-vid as an MJPEG source and re-serves it as multipart/x-mixed-replace, which
every browser renders natively, plus an overlay page showing where the field centre and edges are.

WHY NOT THE ASCII --field-map SCOPE. That is a 64x20 contrast map: enough to answer "is the beacon on the
sensor", useless for aiming a 160 mm printed marker. Placement needs real pixels.

WHY rpicam-vid RATHER THAN beacon_trackd. Two reasons. The camera is single-access, so a preview cannot run
alongside the tracker anyway — and you do not want the tracker's exposure here. The rig runs at 53 us for a
bright IR LED, where paper under room light is BLACK; rpicam-vid's auto-exposure is exactly what makes the
printed targets visible. It requests the same 640x400 mode, so the framing matches what the tracker sees.

    ./preview.py [--port 8888] [--fps 15] [--shutter US]     # then open http://<pi>:8888/

Ctrl-C to stop. Nothing here writes to /data and nothing touches the tracker's config.
"""
import argparse, socket, subprocess, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOI, EOI = b"\xff\xd8", b"\xff\xd9"

PAGE = """<!doctype html><meta charset=utf-8><title>beacon camera</title>
<style>
 body{{background:#111;color:#ddd;font:13px system-ui,sans-serif;margin:0;padding:12px}}
 .wrap{{position:relative;display:inline-block;line-height:0}}
 img{{width:{w}px;height:{h}px;image-rendering:pixelated;display:block}}
 svg{{position:absolute;inset:0;pointer-events:none}}
 h1{{font-size:14px;font-weight:600;margin:0 0 8px}}
 p{{max-width:{w}px;line-height:1.5;color:#999}}
 b{{color:#ddd}}
</style>
<h1>beacon camera — live ({w}&times;{h}, auto-exposure)</h1>
<div class=wrap>
  <img src="/stream">
  <svg viewBox="0 0 {w} {h}">
    <!-- field edge: the outer 15% is where an ultra-wide lens distorts most and where 0.304 deg/px
         is least trustworthy. Keep calibration targets inside the inner box where you can. -->
    <rect x="{ix}" y="{iy}" width="{iw}" height="{ih}" fill="none" stroke="#3a7" stroke-width="1"
          stroke-dasharray="6 4" opacity=".8"/>
    <line x1="{cx}" y1="0" x2="{cx}" y2="{h}" stroke="#e55" stroke-width="1" opacity=".7"/>
    <line x1="0" y1="{cy}" x2="{w}" y2="{cy}" stroke="#e55" stroke-width="1" opacity=".7"/>
    <circle cx="{cx}" cy="{cy}" r="4" fill="none" stroke="#e55" stroke-width="1"/>
    <text x="{ix2}" y="{iy2}" fill="#3a7" font-size="11" font-family="monospace">inner 70% - least distortion</text>
  </svg>
</div>
<p><b>Red cross</b> = boresight, M2 (0,0). <b>Green box</b> = inner 70% of the field.
Full field is 97.3&deg; &times; 60.8&deg;, so 1 px here &asymp; 0.152&deg;.
An ArUco marker wants &ge;40 native px across to detect &mdash; that is 160 mm at about 2 m.
Auto-exposure is ON so paper is visible; the tracker itself runs at 53 &micro;s where paper is black.</p>
"""


class Broker:
    """One rpicam-vid process, many viewers. Latest-frame-wins: a slow browser gets stale frames, never
    backpressure onto the capture."""
    def __init__(self, cmd):
        self.frame = None
        self.cv = threading.Condition()
        self.seq = 0
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        buf = b""
        while True:
            chunk = self.proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                i = buf.find(SOI)
                j = buf.find(EOI, i + 2) if i >= 0 else -1
                if i < 0 or j < 0:
                    break
                with self.cv:
                    self.frame = buf[i:j + 2]
                    self.seq += 1
                    self.cv.notify_all()
                buf = buf[j + 2:]
        with self.cv:                       # source died: wake viewers so they do not hang forever
            self.frame = None
            self.cv.notify_all()

    def next(self, last):
        with self.cv:
            while self.seq == last and self.proc.poll() is None:
                self.cv.wait(timeout=2.0)
            return self.frame, self.seq


def make_handler(broker, w, h):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass                            # a browser polling at 15 Hz would drown the console

        def do_GET(self):
            if self.path == "/":
                ix, iy = int(w * .15), int(h * .15)
                body = PAGE.format(w=w, h=h, cx=w // 2, cy=h // 2, ix=ix, iy=iy,
                                   iw=w - 2 * ix, ih=h - 2 * iy, ix2=ix + 4, iy2=iy + 14).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while True:
                    frame, last = broker.next(last)
                    if frame is None:
                        return
                    self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return                      # viewer closed the tab; normal
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--shutter", type=int, default=0, help="fixed shutter in us (0 = auto, the default)")
    a = ap.parse_args()

    cmd = ["rpicam-vid", "-t", "0", "--codec", "mjpeg", "-o", "-", "-n",
           "--width", str(a.width), "--height", str(a.height), "--framerate", str(a.fps)]
    if a.shutter:
        cmd += ["--shutter", str(a.shutter), "--gain", "2"]

    broker = Broker(cmd)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(broker, a.width, a.height))
    srv.daemon_threads = True
    host = socket.gethostname()
    print(f"preview: http://{host}:{a.port}/   (Ctrl-C to stop)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        broker.proc.terminate()


if __name__ == "__main__":
    main()
