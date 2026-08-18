#!/usr/bin/env python3
"""beacon_display.py -- DGX-side live display for beacon_track.py --json (run over ssh).

Renders the M2 camera grid: 320x200 px at 0.304 deg/px (97.3 x 60.8 deg), CENTRE = 0, image-right = +x,
image-down = +y (041 camera_projection.h convention: pixel centres at (i-(n-1)/2)*deg_per_px). The tracked
beacon is drawn as a spot; sidebar shows the live tuple. Terminal cell aspect ~2:1 -> grid drawn 80x25 chars.

Usage:  ssh pi@100.87.61.53 python3 beacon_track.py --json --fps 250 --chip 121 --shutter 200 | python3 beacon_display.py
"""
import curses, json, sys, time, select

PXW, PXH = 320, 200
DEG = 0.304
GW, GH = 80, 25                     # character grid (≈ 4 px per column, 8 px per row)

def main(stdscr):
    curses.curs_set(0); stdscr.nodelay(True); stdscr.timeout(100)
    last = None; trail = []
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            line = sys.stdin.readline()
            if not line: break
            try: last = json.loads(line)
            except ValueError: continue
            if last.get("lock"): trail = (trail + [(last["x"], last["y"])])[-12:]
        stdscr.erase()
        stdscr.addstr(0, 0, f" M2 camera grid {PXW}x{PXH} @ {DEG} deg/px  ({PXW*DEG:.1f} x {PXH*DEG:.1f} deg)   centre=(0,0)  +x right, +y down", curses.A_BOLD)
        # frame
        for i in range(GW+2): stdscr.addch(2, i, '-'); stdscr.addch(3+GH, i, '-')
        for j in range(GH): stdscr.addch(3+j, 0, '|'); stdscr.addch(3+j, GW+1, '|')
        # crosshair at centre
        cx, cy = 1+GW//2, 3+GH//2
        for j in range(GH): stdscr.addch(3+j, cx, ':')
        for i in range(GW): stdscr.addch(cy, 1+i, '.')
        stdscr.addch(cy, cx, '+')
        # tick labels (deg)
        stdscr.addstr(3+GH+1, 1, f"-{PXW*DEG/2:.0f}deg"); stdscr.addstr(3+GH+1, GW-6, f"+{PXW*DEG/2:.0f}deg")
        stdscr.addstr(3, GW+3, f"-{PXH*DEG/2:.0f}deg"); stdscr.addstr(3+GH-1, GW+3, f"+{PXH*DEG/2:.0f}deg")
        # trail + spot
        for (tx, ty) in trail[:-1]:
            gx = 1 + int(tx / PXW * GW); gy = 3 + int(ty / PXH * GH)
            if 1 <= gx <= GW and 3 <= gy < 3+GH: stdscr.addch(gy, gx, '*')
        if last:
            x, y = last["x"], last["y"]
            gx = 1 + int(x / PXW * GW); gy = 3 + int(y / PXH * GH)
            ch = '@' if last.get("lock") else '?'
            if 1 <= gx <= GW and 3 <= gy < 3+GH: stdscr.addch(gy, gx, ch, curses.A_BOLD | curses.A_REVERSE)
            xc = x - (PXW-1)/2; yc = y - (PXH-1)/2          # centred pixel coords (041 convention)
            side = {"A": "code A", "B": "code B (shown as R/starboard slot -- 041 assigns L/R by wavelength, not code)"}[last["code"]]
            info = [
                f" {'LOCK' if last.get('lock') else 'SEARCH'}   mode={last['mode']}   {side}",
                f" x={xc:+7.1f} px  y={yc:+7.1f} px   (raw {x:6.1f},{y:6.1f})",
                f" bearing_x={xc*DEG:+6.2f} deg  bearing_y={yc*DEG:+6.2f} deg   cep={last['cep']:.2f} px = {last['cep']*DEG:.2f} deg",
                f" q={last['q']:.3f}  peak={last['peak']}  chip={last['chip']} Hz  fps={last['fps']}  corr={last['corr_ms']} ms  t={last['t']}",
            ]
            for k, s in enumerate(info): stdscr.addstr(3+GH+3+k, 0, s[:curses.COLS-1])
        else:
            stdscr.addstr(3+GH+3, 0, " waiting for collector...")
        stdscr.refresh()
        try:
            if stdscr.getch() in (ord('q'), 27): break
        except curses.error: pass

if __name__ == "__main__":
    curses.wrapper(main)
