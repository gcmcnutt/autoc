#!/usr/bin/env python3
"""inav_cli_capture.py — read-only INAV CLI capture for the 045 bench (version, status, aux, serial, feature, blackbox,
diff all). Never sends set/save; `exit` at the end reboots the FC. Close INAV Configurator first.
Usage: python3 inav_cli_capture.py /dev/ttyACM0 out.txt"""
import sys, time
import serial

PORT, OUT = sys.argv[1], sys.argv[2]
CMDS = ["version", "status", "aux", "serial", "feature", "blackbox", "diff all"]
FORBIDDEN = {"set", "save", "defaults", "dfu", "bl", "flash_erase", "load", "reboot"}
assert not any(c.split()[0] in FORBIDDEN for c in CMDS)

s = serial.Serial(PORT, 115200, timeout=0.2)
time.sleep(0.3)
s.write(b"#"); time.sleep(1.0); s.read(65536)
out = []
for c in CMDS:
    s.write((c + "\r\n").encode())
    buf, idle = b"", 0
    while idle < 8:  # ~1.6 s of silence ends the command
        chunk = s.read(65536)
        if chunk: buf += chunk; idle = 0
        else: idle += 1
    out.append(f"===== {c}\n" + buf.decode(errors="replace"))
s.write(b"exit\r\n"); time.sleep(0.5)
s.close()
open(OUT, "w").write("\n".join(out))
print("wrote", OUT)
