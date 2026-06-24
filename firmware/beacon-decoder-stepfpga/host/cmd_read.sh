#!/usr/bin/env bash
# Send a knob command to the StepFPGA, then read telemetry — the A4d sweep primitive (counterpart to monitor.sh).
#   ./cmd_read.sh <mask> [seconds] [port]
#   mask bits: 0=code-select(B) 1=inject-1-bit 2=inject-2-bit 3=weak-signal 4=high-floor
#   e.g.  ./cmd_read.sh 1 5        # select code B, read 5 s
#         ./cmd_read.sh 0x12 5     # code A + high-floor, read 5 s
#         ./cmd_read.sh 2 8 | python3 -c 'import sys;from beacon_telemetry import frames_from_lines;\
#             [print(f) for f in frames_from_lines(sys.stdin)]'   # 1-bit error, parse 8 s
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASK="$(( ${1:-0} ))"; DUR="${2:-5}"; PORT="${3:-COM3}"      # $(( )) accepts 0x.. hex too
SB="/mnt/c/fpga-build/stepfpga"; mkdir -p "$SB"; cp -u "$HERE/cmd_read.ps1" "$SB/"
( cd "$SB" && powershell.exe -NoProfile -ExecutionPolicy Bypass -File cmd_read.ps1 -Mask "$MASK" -Seconds "$DUR" -Port "$PORT" )
