#!/usr/bin/env python3
"""045 T056 bench tool: watch the flight controller over its USB MSP port, read-only, at ~20 Hz, and print every change in arm
state, FAILSAFE / MSP RC OVERRIDE / MANUAL / ANGLE / NAV POSHOLD / NAV RTH modes, the RC_LINK / FAILSAFE_SYSTEM arming flags, and the merged RC channels
(roll, pitch, throttle, yaw, RC6 mode, RC8 nav switch, RC9 override switch). Every sample also goes to a CSV, so hold windows and hand-backs can be
timed afterwards. Never writes a setting.

    python3 specs/045-m2-tracking/fc_watch.py [--port /dev/ttyACM0] [--seconds 300] [--csv out.csv]

Box positions come from MSP_BOXIDS by permanent id (ARM 0, ANGLE 1, MANUAL 12, FAILSAFE 27, MSP RC OVERRIDE 50, NAV POSHOLD 11, NAV RTH 10;
CSVs from before 2026-11-13 17:30 lack the last two modes and rc8), not fixed bits
(the bench found the xiao's fixed FAILSAFE bit 19 is NAV ALTHOLD on article 1). MSP_RC reports the merged channels: while the
override is active, channels 1–4 and 6 are the xiao's; after the override drops, the pilot's (or the last held values in a
receiver failsafe). Close INAV Configurator first; only one program can hold the FC's port.
"""
import argparse
import csv
import struct
import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from msp_bench_read import MSP_BOXIDS, MSP2_INAV_STATUS, find_fc_port, req  # noqa: E402

MSP_RC = 105
PERM = {"ARM": 0, "ANGLE": 1, "MANUAL": 12, "FAILSAFE": 27, "OVERRIDE": 50, "POSHOLD": 11, "RTH": 10}
AF_ARMED, AF_FS_SYSTEM, AF_RC_LINK = 1 << 2, 1 << 7, 1 << 18
CH = {"roll": 0, "pitch": 1, "yaw": 2, "thr": 3, "rc6": 5, "rc8": 7, "rc9": 8}  # INAV internal order: ROLL, PITCH, YAW, THROTTLE


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=300)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--chan-step", type=int, default=40, help="print a channel change larger than this many µs (default 40)")
    ap.add_argument("--print-chans", default=",".join(CH), help="channels whose changes are printed (default all; the CSV has all)")
    args = ap.parse_args()

    s = serial.Serial(args.port or find_fc_port(), 115200, timeout=0.02)
    ids = req(s, MSP_BOXIDS)
    if not ids:
        sys.exit("fc_watch: no MSP_BOXIDS reply (Configurator open? wrong port?)")
    pos = {name: (list(ids).index(pid) if pid in ids else None) for name, pid in PERM.items()}
    print("box positions:", pos, flush=True)
    w = csv.writer(open(args.csv, "w", newline="")) if args.csv else None
    if w:
        # column order = the order the values are written below (flags, then modes in PERM order without ARM)
        w.writerow(["t_s", "armed", "rc_link_lost", "fs_system"] + [n for n in PERM if n != "ARM"] + list(CH))

    t0 = time.time(); prev = None; prevch = None
    while time.time() - t0 < args.seconds:
        st = req(s, MSP2_INAV_STATUS, timeout=0.2)
        rc = req(s, MSP_RC, timeout=0.2)
        t = time.time() - t0
        if not st or not rc or len(st) < 13:
            continue
        af = struct.unpack("<I", st[9:13])[0]
        boxbits = int.from_bytes(st[13:-1], "little")
        modes = {n: int(p is not None and bool(boxbits >> p & 1)) for n, p in pos.items() if n != "ARM"}
        flags = {"armed": int(bool(af & AF_ARMED)), "rc_link_lost": int(bool(af & AF_RC_LINK)),
                 "fs_system": int(bool(af & AF_FS_SYSTEM))}
        chans = struct.unpack("<%dH" % (len(rc) // 2), rc)
        ch = {k: (chans[i] if i < len(chans) else 0) for k, i in CH.items()}
        state = {**flags, **modes}
        if w:
            w.writerow([f"{t:.3f}"] + list(state.values()) + list(ch.values()))
        if state != prev:
            print(f"{time.strftime('%H:%M:%S')} t={t:7.2f}s  " + " ".join(f"{k}={v}" for k, v in state.items())
                  + "   ch " + " ".join(f"{k}={v}" for k, v in ch.items()), flush=True)
            prev = state; prevch = ch
        elif prevch and any(abs(ch[k] - prevch[k]) > args.chan_step for k in args.print_chans.split(",")):
            print(f"{time.strftime('%H:%M:%S')} t={t:7.2f}s  ch " + " ".join(f"{k}={v}" for k, v in ch.items()), flush=True)
            prevch = ch


if __name__ == "__main__":
    main()
