#!/usr/bin/env python3
"""045 T056 bench tool: read the flight controller's blackbox (onboard dataflash) over its USB MSP port, read-only, the way INAV
Configurator does (MSP_DATAFLASH_SUMMARY, then MSP_DATAFLASH_READ in blocks), and save the used part as a raw blackbox file that
`blackbox_decode` reads. Never erases and never writes a setting.

    python3 specs/045-m2-tracking/fc_blackbox_dump.py -o out.TXT [--port /dev/ttyACM0] [--block 4096] [--summary]

INAV 8.0 replies with [u32 address][data] and no compression (fc/fc_msp.c serializeDataflashReadReply); the block is capped by
the MSP port's dataflash buffer. Close INAV Configurator first; only one program can hold the FC's port. Disarmed only (the FC
refuses nothing, but reading while logging would race the writer).
"""
import argparse
import struct
import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from msp_bench_read import find_fc_port, req  # noqa: E402

MSP_DATAFLASH_SUMMARY, MSP_DATAFLASH_READ = 70, 71
MSP2_INAV_STATUS = 0x2000
AF_ARMED = 1 << 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", help="raw blackbox output file (e.g. blackbox_<date>.TXT)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--block", type=int, default=4096, help="bytes per read request (default 4096)")
    ap.add_argument("--summary", action="store_true", help="print the flash summary and exit")
    args = ap.parse_args()

    s = serial.Serial(args.port or find_fc_port(), 115200, timeout=0.05)
    st = req(s, MSP2_INAV_STATUS)
    if st and len(st) >= 13 and struct.unpack("<I", st[9:13])[0] & AF_ARMED:
        sys.exit("fc_blackbox_dump: the FC is ARMED; disarm first")
    summ = req(s, MSP_DATAFLASH_SUMMARY)
    if not summ or len(summ) < 13:
        sys.exit("fc_blackbox_dump: no MSP_DATAFLASH_SUMMARY reply (Configurator open? wrong port? no flash?)")
    ready, sectors, total, used = struct.unpack("<BIII", summ[:13])
    print(f"flash: ready={ready} sectors={sectors} total={total} B used={used} B ({100.0 * used / max(total, 1):.1f} %)",
          flush=True)
    if args.summary:
        return
    if not args.out:
        sys.exit("fc_blackbox_dump: -o is required unless --summary")
    if not ready:
        sys.exit("fc_blackbox_dump: flash not ready")

    t0 = time.time(); addr = 0; last = 0.0
    with open(args.out, "wb") as f:
        while addr < used:
            n = min(args.block, used - addr)
            for attempt in range(5):
                r = req(s, MSP_DATAFLASH_READ, struct.pack("<IH", addr, n), timeout=1.0)
                if r and len(r) > 4 and struct.unpack("<I", r[:4])[0] == addr:
                    break
            else:
                sys.exit(f"fc_blackbox_dump: read failed at {addr} after 5 attempts")
            data = r[4:]
            f.write(data); addr += len(data)
            if time.time() - last > 2:
                el = time.time() - t0
                print(f"  {addr}/{used} B  {addr / el / 1024:.1f} kB/s", flush=True); last = time.time()
    el = time.time() - t0
    print(f"saved {addr} B to {args.out} in {el:.1f} s ({addr / el / 1024:.1f} kB/s)", flush=True)


if __name__ == "__main__":
    main()
