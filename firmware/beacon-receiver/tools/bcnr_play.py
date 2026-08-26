#!/usr/bin/env python3
"""bcnr_play.py — stream a .bcnr clip as raw gray8 on stdout, so ffplay/ffmpeg can read it.

A .bcnr is not a video file and no player will open it: it is a 52-byte container header followed, per
frame, by a 40-byte frame header and w*h bytes of uncompressed gray8. This strips those headers and
emits nothing but pixels, which is exactly what `-f rawvideo` wants.

Two things it unlocks, neither of which needs OpenCV (so they work on any box with ffmpeg):

    # WATCH IT, no intermediate file, scrubbable in ffplay
    bcnr_play.py clip.bcnr --speed 0.1 | ffplay -f rawvideo -pixel_format gray \\
        -video_size 640x400 -framerate 29 -vf eq=gamma=0.45 -i -

    # CONVERT to mp4
    bcnr_play.py clip.bcnr --speed 0.1 | ffmpeg -f rawvideo -pixel_format gray \\
        -video_size 640x400 -framerate 29 -i - -vf eq=gamma=0.45 -c:v libx264 -crf 20 out.mp4

`--print-cmd` prints the matching command line for you with the geometry and framerate filled in, since
getting `-video_size` wrong yields a sheared image rather than an error.

ON GAMMA. These scenes run a mean of ~0.02–4 ADU against a beacon that saturates at 255, so a linear
display shows a black frame with one dot and nothing else. `eq=gamma=0.45` in the ffmpeg filter chain
brings the background up without clipping the beacon. It is a DISPLAY transform only — this tool never
alters the pixels it emits.

For tracker state drawn over the video (position, cep, MEASURED vs coasting, strip charts) use
`track_overlay.py` instead; that needs OpenCV and currently runs on the Pi.
"""
import argparse
import struct
import sys

MAGIC = 0x42434E52
HDR = 52
FRAME_HDR = 40
SRC_FPS = 288.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback speed vs real time (1.0 = 288 fps, 0.1 = ten times slower)")
    ap.add_argument("--start", type=float, default=0.0, help="seconds into the clip")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to emit (0 = to the end)")
    ap.add_argument("--print-cmd", action="store_true", help="print the ffplay/ffmpeg command and exit")
    a = ap.parse_args()

    f = open(a.clip, "rb")
    magic, ver, hdr_b, w, h, bpp, _mode = struct.unpack_from("<IHHHHHH", f.read(HDR), 0)
    if magic != MAGIC:
        raise SystemExit("%s: magic %#x is not BCNR" % (a.clip, magic))
    if bpp != 8:
        raise SystemExit("%s: expected 8bpp gray, got %d" % (a.clip, bpp))
    fsz = FRAME_HDR + w * h
    fps = max(1.0, SRC_FPS * a.speed)

    if a.print_cmd:
        base = ("-f rawvideo -pixel_format gray -video_size %dx%d -framerate %.3f" % (w, h, fps))
        me = "%s %s --speed %g" % (sys.argv[0], a.clip, a.speed)
        print("# watch:")
        print("%s | ffplay %s -vf eq=gamma=0.45 -i -" % (me, base))
        print("# convert:")
        print("%s | ffmpeg %s -i - -vf eq=gamma=0.45 -c:v libx264 -crf 20 -pix_fmt yuv420p out.mp4"
              % (me, base))
        return 0

    i0 = int(a.start * SRC_FPS)
    f.seek(HDR + i0 * fsz)
    n_max = int(a.duration * SRC_FPS) if a.duration else -1
    out = sys.stdout.buffer
    n = 0
    try:
        while n_max < 0 or n < n_max:
            d = f.read(fsz)
            if len(d) < fsz:
                break
            out.write(d[FRAME_HDR:])
            n += 1
    except BrokenPipeError:
        pass                                  # the player was closed; that is a normal exit here
    finally:
        try:
            out.flush()
        except BrokenPipeError:
            pass
    print("emitted %d frames (%dx%d gray8) at %.1f fps" % (n, w, h, fps), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
