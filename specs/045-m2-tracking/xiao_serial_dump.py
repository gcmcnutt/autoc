#!/usr/bin/env python3
"""045 F2 (tasks.md T016): read a xiao flight log over the USB serial console — the wired byte reference for checking a BLE
download (T023), and a field fallback with a laptop.

    python3 specs/045-m2-tracking/xiao_serial_dump.py --port /dev/ttyACM0 --list
    python3 specs/045-m2-tracking/xiao_serial_dump.py --port /dev/ttyACM0 --file flight_001.bin -o flight_001.wired.bin

The xiao must be disarmed. Identify its port by `udevadm info -q property -n /dev/ttyACMx | grep ID_MODEL` (the ACM numbers
swap between sessions). Don't open the port during the xiao's 8 s boot window; wait for its boot banner. The firmware frames
the dump as "DUMP_BEGIN:<name>:<size>", <size> raw bytes, "DUMP_END:<crc32>:<bytes>"; this script checks the size and the
CRC32 and writes nothing on a mismatch.
"""
import argparse
import sys
import time
import zlib

import serial  # pyserial


def read_line(port, deadline):
    buf = bytearray()
    while time.monotonic() < deadline:
        b = port.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode(errors="replace").rstrip("\r")
        buf += b
    raise TimeoutError("timed out waiting for a line from the xiao")


def wait_for(port, prefixes, timeout):
    """Skip console log lines until one starts with any of `prefixes`."""
    deadline = time.monotonic() + timeout
    while True:
        line = read_line(port, deadline)
        if line.startswith(prefixes):
            return line


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="the xiao's serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="list the flight logs on the xiao")
    ap.add_argument("--file", help="flight log to dump, e.g. flight_001.bin")
    ap.add_argument("-o", "--out", help="output path (default: <file> in the current directory)")
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds allowed for the whole dump (default 60)")
    args = ap.parse_args()
    if not args.list and not args.file:
        ap.error("give --list or --file")

    with serial.Serial(args.port, 115200, timeout=0.2) as port:
        port.reset_input_buffer()
        if args.list:
            port.write(b"LIST\n")
            deadline = time.monotonic() + 10
            while True:
                line = read_line(port, deadline)
                if line.startswith("FILE:"):
                    print(line[5:])
                elif line in ("LIST_DONE",) or line.startswith("CONSOLE_ERR"):
                    if line != "LIST_DONE":
                        sys.exit(line)
                    return

        port.write(f"DUMP:{args.file}\n".encode())
        head = wait_for(port, ("DUMP_BEGIN:", "DUMP_ERR:", "CONSOLE_ERR:"), 10)
        if not head.startswith("DUMP_BEGIN:"):
            sys.exit(f"xiao refused: {head}")
        size = int(head.rsplit(":", 1)[1])
        data = bytearray()
        t0 = time.monotonic()
        deadline = t0 + args.timeout
        while len(data) < size:
            if time.monotonic() > deadline:
                sys.exit(f"timed out after {len(data)} of {size} bytes")
            data += port.read(size - len(data))
        secs = time.monotonic() - t0
        tail = wait_for(port, ("DUMP_END:", "DUMP_ERR:"), 10)
        if not tail.startswith("DUMP_END:"):
            sys.exit(f"dump failed: {tail}")
        _, crc_hex, n = tail.split(":")
        crc = zlib.crc32(bytes(data)) & 0xFFFFFFFF
        if int(n) != size or crc != int(crc_hex, 16):
            sys.exit(f"MISMATCH: size {len(data)} vs {n}, crc {crc:08X} vs {crc_hex}; nothing written")
        out = args.out or args.file
        with open(out, "wb") as f:
            f.write(data)
        print(f"wrote {out}: {size} bytes, CRC32 {crc:08X} OK, {secs:.2f} s ({size / 1024 / max(secs, 1e-6):.1f} kB/s)")


if __name__ == "__main__":
    main()
