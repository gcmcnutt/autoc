#!/usr/bin/env bash
# Send a knob command to the StepFPGA, then read telemetry — the A4d sweep primitive (counterpart to monitor.sh).
#   ./cmd_read.sh <mask> [seconds] [port]
#   sends '+' (REMOTE) then 0x80|mask. mask bits: 0=enA 1=enB 2=enN(noise) 3=inj-1bit 4=inj-2bit 5=weak 6=floor
#   e.g.  ./cmd_read.sh 1 5        # code A only, read 5 s
#         ./cmd_read.sh 3 5        # A+B (CDMA), read 5 s
#         ./cmd_read.sh 7 8 | python3 -c 'import sys;from beacon_telemetry import frames_from_lines;\
#             [print(f) for f in frames_from_lines(sys.stdin)]'   # A+B+noise, parse 8 s
#   (back to switches:  powershell.exe ... cmd_read.ps1 -Local)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASK="$(( ${1:-0} ))"; DUR="${2:-5}"; PORT="${3:-COM3}"      # $(( )) accepts 0x.. hex too
SB="/mnt/c/fpga-build/stepfpga"; mkdir -p "$SB"; cp -u "$HERE/cmd_read.ps1" "$SB/"
( cd "$SB" && powershell.exe -NoProfile -ExecutionPolicy Bypass -File cmd_read.ps1 -Mask "$MASK" -Seconds "$DUR" -Port "$PORT" )
