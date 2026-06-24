#!/usr/bin/env bash
# Capture a lock transition for A4d timing (LOS / acquisition): set mask A, hold PreSec, switch to mask B,
# capture PostSec, with a "MARK" line at the edge. Pipe to a parser to time latency-to-lock-change.
#   ./transition.sh <maskA> <maskB> [preSec] [postSec] [port]
#   e.g.  ./transition.sh 1 0 2 3      # A-locked -> off (LOS)
#         ./transition.sh 0 1 1 3      # off -> A (acquisition)
#         ./transition.sh 1 4 2 3      # A -> noise-only (LOS under noise)
# mask bits: 0=enA 1=enB 2=enN 3=inj1 4=inj2 5=weak 6=floor
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MA="$(( ${1:-1} ))"; MB="$(( ${2:-0} ))"; PRE="${3:-2}"; POST="${4:-3}"; PORT="${5:-COM3}"
SB="/mnt/c/fpga-build/stepfpga"; mkdir -p "$SB"; cp -u "$HERE/transition.ps1" "$SB/"
( cd "$SB" && powershell.exe -NoProfile -ExecutionPolicy Bypass -File transition.ps1 \
    -MaskA "$MA" -MaskB "$MB" -PreSec "$PRE" -PostSec "$POST" -Port "$PORT" )
