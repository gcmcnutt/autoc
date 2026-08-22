#!/usr/bin/env python3
"""ascii_scope — T064: the 10 Hz terminal display. THE STAGE 1 EXIT CRITERION is this animating at
10 Hz on real beacons over SSH, no X.

    ascii_scope --source tcp:<host>:<port> | json:- | binary:<path>  [--hz 10]

Renders the M2 grid (320x200, centre origin): beacon glyphs A (PORT, red) / B (STARBOARD, green),
prediction ghost '+', lock state, CEP ring at large CEP, and a status line per track. Standard library
only — it must run on anything with a terminal.
"""
import argparse, json, socket, struct, sys, time

WIRE = 64 + 48 * 16          # BcnRecord v1: header + 16 tracks
MAGIC = 0x42434E31

def parse_binary(buf):
    (magic, ver, hdr_b) = struct.unpack_from("<IHH", buf, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic:#x} — not a BCN1 record stream")
    if ver != 1:
        raise ValueError(f"record format_version {ver}, this scope implements 1 — refusing (Constitution V)")
    (t_us, seq, tick) = struct.unpack_from("<QII", buf, 8)
    n_tracks, n_slots = struct.unpack_from("<BB", buf, 24)
    (deadline,) = struct.unpack_from("<i", buf, 28)
    rec = {"t_us": t_us, "seq": seq, "tick": tick, "n": n_tracks, "slots": n_slots,
           "deadline_us": deadline, "tracks": []}
    for k in range(n_tracks):
        o = hdr_b + 48 * k
        x, y, vx, vy, xp, yp = struct.unpack_from("<iiiiii", buf, o)
        chip_hz, = struct.unpack_from("<I", buf, o + 24)
        cep, q, lh, extent, scint, flags, age = struct.unpack_from("<HHHHHHH", buf, o + 28)
        code, phase, tint, scale = struct.unpack_from("<BBBB", buf, o + 42)
        rec["tracks"].append({"code": "B" if code else "A", "x": x/256, "y": y/256,
            "vx": vx/256, "vy": vy/256, "xp": xp/256, "yp": yp/256, "cep": cep/256,
            "q": q/256, "lh": lh/256, "chip_hz": chip_hz/256, "scale": scale,
            "tint": tint, "age_ms": age, "flags": flags})
    return rec

def source_records(spec):
    if spec.startswith("tcp:"):
        host, port = spec[4:].rsplit(":", 1)
        while True:
            try:
                s = socket.create_connection((host, int(port)), timeout=5)
                s.settimeout(2.0)
                buf = b""
                while True:
                    chunk = s.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while len(buf) >= WIRE:
                        yield parse_binary(buf[:WIRE]); buf = buf[WIRE:]
            except (ConnectionRefusedError, socket.timeout, OSError):
                time.sleep(0.5)      # daemon not up yet / restarting — keep dialing
    elif spec == "json:-":
        for line in sys.stdin:
            try: yield json.loads(line)
            except json.JSONDecodeError: pass
    elif spec.startswith("binary:"):
        with open(spec[7:], "rb") as f:
            while True:
                b = f.read(WIRE)
                if len(b) < WIRE: return
                yield parse_binary(b)
    else:
        raise SystemExit(f"unknown source {spec}")

GRID_W, GRID_H = 96, 30       # chars; maps the WHOLE M2 grid: x in [-160,160], y in [-100,100]
# The M2 grid is 320x200 CENTRE-ORIGIN (data-model §2), so the field is +-160 x +-100 px = 97.3 x 60.8 deg.
# This was +-80 x +-50 until 2026-08-21, i.e. the scope silently drew only the middle QUARTER of the field
# by area and a beacon out toward the edge simply never appeared -- indistinguishable, on screen, from a
# tracker that had lost it. Fixed together with the --field-map viewfinder, which made the mismatch
# obvious: the field map covered the whole frame while the glyphs covered the centre.
X_SPAN, Y_SPAN = 320.0, 200.0
RED, GRN, DIM, YEL, RST = "\x1b[31m", "\x1b[32m", "\x1b[2m", "\x1b[33m", "\x1b[0m"

# --field-map viewfinder. beacon_trackd --field-map writes a second JSON line per tick carrying a coarse
# max-minus-mean CONTRAST map of the frame (see engine.h). It answers the one question the track glyphs
# cannot: "is the beacon even on the sensor?" -- which is what you need while aiming, precisely when
# there is no track to draw. Rendered as a dim ramp UNDER the glyphs, so a lock always wins the cell.
FIELD_RAMP = " .:-=+*#%@"
# The floor is derived from the map itself, never a constant: scene texture sets the background contrast
# (a synthetic noise field sits at ~17, a dark room near 0), so a fixed threshold either paints the whole
# field or hides a real beacon. Same self-normalising principle acquire_pass() uses on the blink plane.
FIELD_FLOOR_K = 3             # a cell must beat the field median by this many median-absolute-deviations
# PEAK-HOLD. The beacon is DARK on roughly half the chips, so a single frame catches it only ~47 % of
# ticks (measured on the golden clip: peak 96-103 lit, 19 dark). Without a hold the viewfinder strobes and
# is useless for aiming. Decay 3/4 per tick keeps a real source visible for ~5 ticks (250 ms) while still
# letting it fade once you pan off it.
FIELD_DECAY_NUM, FIELD_DECAY_DEN = 3, 4

def field_floor(cells):
    """Robust background level: median + K*MAD. Beacons are a handful of cells out of 1280, so they do
    not move the median, and MAD is unmoved by them either -- that is the point of using it over sigma."""
    srt = sorted(cells)
    med = srt[len(srt) // 2]
    mad = sorted(abs(c - med) for c in cells)[len(cells) // 2]
    return med + FIELD_FLOOR_K * max(mad, 1)


def cell(x, y):
    cx = int((x + X_SPAN/2) / X_SPAN * GRID_W)
    cy = int((y + Y_SPAN/2) / Y_SPAN * GRID_H)
    return (cx, cy) if 0 <= cx < GRID_W and 0 <= cy < GRID_H else None

def render(rec, fps_est, field=None):
    if field:
        # field cells -> grid cells by nearest sample; scale the ramp to this frame's own peak so a dim
        # far beacon still shows (it is a viewfinder, not a photometer).
        fw, fh, cells = field
        floor = field_floor(cells)
        peak = max(max(cells), floor + 1)
        grid = []
        for y in range(GRID_H):
            row = []
            for x in range(GRID_W):
                v = cells[(y * fh // GRID_H) * fw + (x * fw // GRID_W)]
                if v <= floor:
                    row.append(f"{DIM}.{RST}" if (x % 8 == 0 and y % 6 == 0) else " ")
                else:
                    k = 1 + (v - floor) * (len(FIELD_RAMP) - 2) // max(1, peak - floor)
                    row.append(f"{DIM}{FIELD_RAMP[min(k, len(FIELD_RAMP) - 1)]}{RST}")
            grid.append(row)
    else:
        grid = [[f"{DIM}.{RST}" if (x % 8 == 0 and y % 6 == 0) else " "
                 for x in range(GRID_W)] for y in range(GRID_H)]
    # centre crosshair
    c = cell(0, 0)
    if c: grid[c[1]][c[0]] = f"{DIM}+{RST}"
    lines = []
    for t in rec["tracks"]:
        col = GRN if t["code"] == "B" else RED
        hold = t["flags"] & 0x4
        glyph = t["code"].lower() if hold else t["code"]
        p = cell(t["xp"], t["yp"])
        if p: grid[p[1]][p[0]] = f"{DIM}+{RST}"
        c = cell(t["x"], t["y"])
        if c: grid[c[1]][c[0]] = f"{col}{glyph}{RST}"
        state = "HOLD" if hold else ("LOCK" if t["flags"] & 0x2 else "????")
        extra = ("SAT " if t["flags"] & 0x20 else "") + ("MP " if t["flags"] & 0x10 else "") + \
                ("FIX " if t["flags"] & 0x40 else "")
        lines.append(f"  {col}{t['code']}{RST} {state} ({t['x']:7.2f},{t['y']:7.2f}) "
                     f"v({t['vx']:6.1f},{t['vy']:6.1f}) q={t['q']:.2f} lh={t['lh']:.2f} "
                     f"cep={t['cep']:.2f} chip={t['chip_hz']:6.2f}Hz scale={t['scale']} "
                     f"K={t['tint']} age={t['age_ms']}ms {YEL}{extra}{RST}")
    out = ["\x1b[H\x1b[2J"]
    dl = rec.get("deadline_us", 0)
    dl_s = f"{dl/1000.0:+.1f}ms" if dl else "n/a"
    vf = ""
    if field:
        hot, floor = max(field[2]), field_floor(field[2])
        vf = f"  view {'ON-SENSOR' if hot > floor else 'nothing'} (peak {hot:3d} vs floor {floor:3d})"
    out.append(f" beacon scope  tick {rec['tick']:>7}  tracks {rec['n']}  slots {rec['slots']}"
               f"  deadline {dl_s}  {fps_est:4.1f} rec/s{vf}   (A=PORT red, B=STARBOARD green)")
    out.append(" +" + "-" * GRID_W + "+")
    for row in grid: out.append(" |" + "".join(row) + "|")
    out.append(" +" + "-" * GRID_W + "+")
    out.extend(lines if lines else [f"  {DIM}no tracks — acquiring...{RST}"])
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--hz", type=float, default=10.0)
    a = ap.parse_args()
    period = 1.0 / a.hz
    last_draw = 0.0
    n_since, t_rate, fps_est = 0, time.time(), 0.0
    field = None
    hold = None
    for rec in source_records(a.source):
        if "field" in rec:                      # viewfinder side channel, not a record
            cur = rec["field"]
            if hold is None or len(hold) != len(cur):
                hold = list(cur)
            else:
                hold = [c if c > h * FIELD_DECAY_NUM // FIELD_DECAY_DEN
                        else h * FIELD_DECAY_NUM // FIELD_DECAY_DEN for c, h in zip(cur, hold)]
            field = (rec["field_w"], rec["field_h"], hold)
            continue
        n_since += 1
        now = time.time()
        if now - t_rate >= 1.0:
            fps_est = n_since / (now - t_rate)
            n_since, t_rate = 0, now
        if now - last_draw >= period:      # records arrive at 20 Hz; draw at --hz
            render(rec, fps_est, field)
            last_draw = now

if __name__ == "__main__":
    main()
