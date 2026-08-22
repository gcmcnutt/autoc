#!/usr/bin/env python3
"""Printable bench targets for the 042 ground-truth work — STDLIB ONLY, no OpenCV, no PIL.

WHY THE BITS ARE EMBEDDED. These were first generated with cv2.aruco, but a bench fixture that cannot be
re-cut in five years because a contrib module moved its API is a liability, and this repo already prefers
self-contained tools. The 6x6 cell grids below (4x4 data + the mandatory 1-cell black border) are the
DICT_4X4_50 markers for ids 0-3, extracted once from OpenCV 5.0.0 and verified byte-identical to its
generateImageMarker output. Any ArUco detector set to DICT_4X4_50 reads them.

WHY THESE SIZES. Sizing is forced by the optics, not by taste. The OV9281 at 640x400 over a 97.3 deg field
is 0.152 deg per NATIVE pixel = 2.65 mrad, so a feature of physical size S at range d spans

    px = S / (d * 0.00265)

ArUco wants >= ~40 px across the black square to detect reliably, which at 2 m is 21 cm. Hence 160 mm
markers, one per A4 sheet. On this ultra-wide lens, printed targets must be BIG or CLOSE.

    ./gen_targets.py [outdir]        # default: alongside this file

⚠️ PRINT AT 100% / ACTUAL SIZE. "Fit to page" silently rescales and every measurement downstream inherits
the error. Measure a printed feature with a ruler before trusting a run.
"""
import os, struct, sys, zlib

DPI = 300
MM_PER_IN = 25.4
A4_W_MM, A4_H_MM = 210.0, 297.0

# DICT_4X4_50, ids 0-3, as 6x6 cells: '1' = white, '0' = black. Border ring is all black by construction.
ARUCO_4X4_50 = {
    0: ['000000', '010110', '001010', '000110', '000100', '000000'],
    1: ['000000', '000000', '011110', '010010', '010100', '000000'],
    2: ['000000', '000110', '000110', '000100', '011010', '000000'],
    3: ['000000', '010010', '010010', '001000', '001100', '000000'],
}


def px(mm):
    return int(round(mm / MM_PER_IN * DPI))


def sheet():
    w, h = px(A4_W_MM), px(A4_H_MM)
    return bytearray(b"\xff" * (w * h)), w, h


def blit_cells(buf, W, cells, x_mm, y_mm, side_mm):
    """Draw an N x N cell grid as a solid block — nearest-neighbour by construction, so no resampling
    softens the edges the corner refiner depends on."""
    n = len(cells)
    x0, y0, side = px(x_mm), px(y_mm), px(side_mm)
    for yy in range(side):
        cy = cells[yy * n // side]
        row = (y0 + yy) * W
        for xx in range(side):
            if cy[xx * n // side] == '0':
                buf[row + x0 + xx] = 0


def rect(buf, W, H, x_mm, y_mm, w_mm, h_mm, val=0):
    """Bounds-CHECKED. Without this the first checkerboard silently ran off the page: 11 cols x 22 mm =
    242 mm against A4's 210 mm, so each row wrapped into the next and the board looked plausible at a
    glance while findChessboardCorners refused it. A fixture generator must fail loudly, not draw
    nonsense."""
    x0, y0, w, h = px(x_mm), px(y_mm), px(w_mm), px(h_mm)
    if x0 < 0 or y0 < 0 or x0 + w > W or y0 + h > H:
        raise SystemExit(f"rect({x_mm:.1f},{y_mm:.1f},{w_mm:.1f},{h_mm:.1f}) mm runs off the "
                         f"{A4_W_MM:.0f}x{A4_H_MM:.0f} mm sheet — reduce the square size or the count")
    for yy in range(h):
        row = (y0 + yy) * W + x0
        buf[row:row + w] = bytes([val]) * w


def write_png(path, buf, W, H):
    """Minimal 8-bit greyscale PNG. zlib is stdlib; this is the whole reason no imaging library is needed."""
    raw = bytearray()
    for y in range(H):
        raw.append(0)                                  # filter type 0 (None) per scanline
        raw += buf[y * W:(y + 1) * W]

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", W, H, 8, 0, 0, 0, 0)   # 8-bit, greyscale
    phys = struct.pack(">IIB", int(DPI / 0.0254), int(DPI / 0.0254), 1)   # px/metre, so viewers print right
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"pHYs", phys) +
                chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out, exist_ok=True)
    made = []

    SIDE = 160.0
    for i, cells in sorted(ARUCO_4X4_50.items()):
        buf, W, H = sheet()
        blit_cells(buf, W, cells, (A4_W_MM - SIDE) / 2, 45.0, SIDE)
        p = os.path.join(out, f"aruco_4x4_50_id{i}_160mm.png")
        write_png(p, buf, W, H)
        made.append(p)

    # Checkerboard: LENS CALIBRATION, a different job from pose (see README). 20 mm x 8x11 squares fits A4
    # with a margin, and the 7x10 INNER-corner grid is deliberately asymmetric (odd x even) so the board's
    # orientation is unambiguous — a symmetric grid leaves a 180-degree flip the calibrator cannot resolve.
    SQ, COLS, ROWS = 20.0, 8, 11
    buf, W, H = sheet()
    ox, oy = (A4_W_MM - COLS * SQ) / 2, 45.0
    for r in range(ROWS):
        for c in range(COLS):
            if (r + c) % 2 == 0:
                rect(buf, W, H, ox + c * SQ, oy + r * SQ, SQ, SQ, 0)
    p = os.path.join(out, f"checkerboard_{COLS}x{ROWS}sq_{int(SQ)}mm_inner{COLS-1}x{ROWS-1}.png")
    write_png(p, buf, W, H)
    made.append(p)

    for p in made:
        print(os.path.basename(p))
    print(f"\n{len(made)} sheets in {out}", file=sys.stderr)
    print("PRINT AT 100% / ACTUAL SIZE — then measure a feature and confirm.", file=sys.stderr)


if __name__ == "__main__":
    main()
