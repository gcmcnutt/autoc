#!/usr/bin/env python3
"""oracle.py — T044: where the beacon ACTUALLY was, measured independently of the tracker.

The tracker's own output cannot measure the tracker, so this re-derives the beacon position from the raw
.bcnr by the one property nothing else in the room has: it blinks the Gold code. Per burst it runs the
FULL-FIELD matched filter the spec (§1) keeps as the weak-signal fallback -- spatial high-pass, then
correlate every pixel's time series against the code over all 31 phases, take the peak. No ROI, no
aperture, no ladder, no lifecycle: nothing the tracker does can bias it.

WHY NOT BRIGHTEST-PIXEL. The first version of this tool took the brightest spatial-high-pass response per
frame. On a real bench scene that does NOT find the beacon: it hops between room clutter and the beacon,
and because the beacon is DARK on half the chips it must hop. Measured on pan1.bcnr it produced apparent
rates of 6500-19000 deg/s -- physically impossible, and the tell that the detector was jumping between
fixed objects. The code is the only discriminator that works here; that is the same fact the tracker is
built on (spec §2.4, "the code kills false candidates").

Correlating ACROSS a burst boundary is forbidden by the container contract (seq gaps are real
discontinuities), so every measurement here is within one burst. Each burst is split into halves so a
within-burst apparent RATE falls out: two positions, one half-burst apart.

Usage:  oracle.py <clip.bcnr> [--csv out.csv] [--code B|A] [--chip-hz 120.0] [--halves N]
"""
import argparse, struct, sys
import numpy as np

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40                 # 24 bytes of fields + the inav/gps trailer, present even when zero (R10)
CODE = {"A": "0000000100011011000011001110011",
        "B": "0100011001100111100101001011110"}
N_CHIPS = 31
DEG_PER_M2_PX = 0.304


def read_header(f):
    b = f.read(HDR)
    magic, ver, hdr_b, w, h, bpp, _mode_e = struct.unpack_from("<IHHHHHH", b, 0)
    if magic != MAGIC:
        raise SystemExit(f"{magic:#x} is not BCNR")
    if ver != 1:
        raise SystemExit(f"container format_version {ver}, this oracle implements 1 — refusing")
    if hdr_b != HDR:
        f.seek(hdr_b)
    return w, h


def bursts(f, w, h):
    """Yield one burst at a time as (t_us list, uint8 array [n,h,w]). Splits on the burst_start flag."""
    npx = w * h
    cur_t, cur_f = [], []
    while True:
        head = f.read(FRAME_HDR)
        if len(head) < FRAME_HDR:
            break
        rec_b, _seq, t_us, _e, _g, flags = struct.unpack_from("<IIQIHH", head, 0)
        payload = f.read(rec_b - FRAME_HDR)
        if len(payload) < npx:
            break                                   # truncated tail: stop, never fabricate
        if (flags & 1) and cur_f:
            yield cur_t, np.array(cur_f, dtype=np.uint8)
            cur_t, cur_f = [], []
        cur_t.append(t_us)
        cur_f.append(np.frombuffer(payload[:npx], dtype=np.uint8).reshape(h, w))
    if cur_f:
        yield cur_t, np.array(cur_f, dtype=np.uint8)


def m2_hipass(stack):
    """native -> M2 (2x2 box) -> spatial high-pass (9*centre - 3x3), the receiver's own front end."""
    n, h, w = stack.shape
    m2 = (stack[:, 0::2, 0::2].astype(np.int32) + stack[:, 0::2, 1::2] +
          stack[:, 1::2, 0::2] + stack[:, 1::2, 1::2])
    s = np.zeros_like(m2)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            s[:, 1:-1, 1:-1] += m2[:, 1 + dy:m2.shape[1] - 1 + dy, 1 + dx:m2.shape[2] - 1 + dx]
    hp = np.zeros_like(m2)
    hp[:, 1:-1, 1:-1] = 9 * m2[:, 1:-1, 1:-1] - s[:, 1:-1, 1:-1]
    return hp


def locate(ts, stack, tmpl, chip_hz):
    """Full-field code match over all 31 phases. Returns (x_m2, y_m2, q, peak) in CENTRE-ORIGIN M2 px."""
    hp = m2_hipass(stack).astype(np.float32)
    n, mh, mw = hp.shape
    hp -= hp.mean(axis=0, keepdims=True)             # per-pixel temporal DC removal
    chips = np.floor((np.array(ts, dtype=np.float64) - ts[0]) * chip_hz / 1e6).astype(np.int64)
    best = None
    flat = hp.reshape(n, -1)
    for phase in range(N_CHIPS):
        signs = tmpl[(chips + phase) % N_CHIPS].astype(np.float32)
        corr = signs @ flat                          # [mh*mw]
        i = int(np.argmax(np.abs(corr)))
        v = abs(float(corr[i]))
        if best is None or v > best[0]:
            best = (v, i, phase, corr)
    v, i, phase, corr = best
    y, x = divmod(i, mw)
    denom = float(np.abs(corr).mean()) or 1.0
    return x - mw / 2.0, y - mh / 2.0, v / denom, v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--csv")
    ap.add_argument("--code", default="B", choices=["A", "B"])
    ap.add_argument("--chip-hz", type=float, default=120.0)
    ap.add_argument("--halves", type=int, default=2,
                    help="sub-windows per burst; 2 gives one rate estimate per burst")
    a = ap.parse_args()

    bits = CODE[a.code]
    tmpl = np.array([1 if c == "1" else -1 for c in bits], dtype=np.int8)

    out = [("burst", "sub", "t_us", "x_m2", "y_m2", "q_rel", "deg_per_s")]
    with open(a.clip, "rb") as f:
        w, h = read_header(f)
        for bi, (ts, stack) in enumerate(bursts(f, w, h), 1):
            k = len(ts) // a.halves
            if k < N_CHIPS // 4:
                print(f"burst {bi}: only {len(ts)} frames — skipped", file=sys.stderr)
                continue
            prev = None
            for s in range(a.halves):
                sl = slice(s * k, (s + 1) * k)
                x, y, q, _pk = locate(ts[sl], stack[sl], tmpl, a.chip_hz)
                tmid = (ts[sl][0] + ts[sl][-1]) // 2
                rate = ""
                if prev is not None:
                    dt = (tmid - prev[2]) / 1e6
                    if dt > 0:
                        d = ((x - prev[0]) ** 2 + (y - prev[1]) ** 2) ** 0.5
                        rate = f"{d * DEG_PER_M2_PX / dt:.1f}"
                prev = (x, y, tmid)
                out.append((bi, s, tmid, f"{x:.2f}", f"{y:.2f}", f"{q:.1f}", rate))
            print(f"burst {bi:3d}: {len(ts)} frames", file=sys.stderr, end="\r")

    text = "\n".join(",".join(str(v) for v in r) for r in out)
    if a.csv:
        open(a.csv, "w").write(text + "\n")
        print(f"\n{len(out)-1} sub-windows -> {a.csv}", file=sys.stderr)
    else:
        print(text)


if __name__ == "__main__":
    main()
