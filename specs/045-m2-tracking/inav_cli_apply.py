#!/usr/bin/env python3
"""inav_cli_apply.py — apply an explicit INAV change set over the CLI for the 045 bench: `get` before, `set`, `get` after for each
setting; `save` (reboots) only if every value reads back, otherwise `exit` without saving. Everything is logged.
Close INAV Configurator first. Usage: python3 inav_cli_apply.py /dev/ttyACM0 log.txt name=value [name=value ...]
Used for the 2026-11-11 GPS / calibration change set and the 2026-11-12 alignment change set (bench-results-20261111.md)."""
import sys, time
import serial

FORBIDDEN = {"defaults", "dfu", "bl", "flash_erase", "load", "reboot", "save", "set", "exit"}


def cmd(s, line, wait=8):
    s.write((line + "\r\n").encode()); buf, idle = b"", 0
    while idle < wait:
        c = s.read(65536)
        if c: buf += c; idle = 0
        else: idle += 1
    return buf.decode(errors="replace")


def main():
    port, log_path, pairs = sys.argv[1], sys.argv[2], sys.argv[3:]
    changes = []
    for p in pairs:
        k, _, v = p.partition("=")
        assert k and v and k not in FORBIDDEN and " " not in k, f"bad setting: {p!r}"
        changes.append((k.strip(), v.strip()))
    if not changes: raise SystemExit("no name=value pairs given")
    s = serial.Serial(port, 115200, timeout=0.2); time.sleep(0.3)
    s.write(b"#"); time.sleep(1.0); s.read(65536)
    log = []
    for k, v in changes:
        log += [cmd(s, f"get {k}"), cmd(s, f"set {k} = {v}"), cmd(s, f"get {k}")]
    text = "".join(log).lower().replace("\r", "")
    ok = all(f"{k} = {v}".lower() in text for k, v in changes)
    log.append(f"### read-back check: {'PASS' if ok else 'FAIL'}")
    log.append(cmd(s, "save" if ok else "exit", wait=4))
    s.close()
    open(log_path, "w").write("\n".join(log))
    print("read-back", "PASS -> saved (FC reboots)" if ok else "FAIL -> exited without save", "; log:", log_path)


if __name__ == "__main__":
    main()
