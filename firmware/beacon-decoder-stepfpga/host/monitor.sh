#!/usr/bin/env bash
# Read StepFPGA telemetry on the WINDOWS side (COM3) via interop — the counterpart to build.sh's flash.
# Board stays on Windows the whole time: D: flashable (build.sh --flash) AND COM3 readable (here). No usbipd.
#
#   ./monitor.sh                 # read COM3 @115200 for 5 s, print each telemetry line
#   ./monitor.sh COM3 20         # 20 s
#   ./monitor.sh COM3 20 | python3 -c 'import sys;from beacon_telemetry import frames_from_lines;\
#       [print(f) for f in frames_from_lines(sys.stdin)]'   # parse BCN frames (once the gateware emits them)
#
# Why Windows-side: WSL2 can't see a Windows COM port, and usbipd-attaching the composite pulls D: off
# Windows (breaks flashing) AND the WSL CDC read hangs uninterruptibly. Reading on Windows sidesteps both.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-COM3}"; DUR="${2:-5}"

# Native-Linux branch (DGX Spark bench, 2026-08-08): no powershell interop -> read the STEPLink CDC
# directly. COM3 maps to $BCN_PORT (default /dev/ttyACM1 = DAPLink CDC). Same one-reader-at-a-time rule.
if ! command -v powershell.exe >/dev/null 2>&1; then
  DEV="$PORT"; [[ "$PORT" == COM* ]] && DEV="${BCN_PORT:-/dev/ttyACM1}"
  stty -F "$DEV" 115200 raw -echo
  timeout "$DUR" cat "$DEV" || true
  exit 0
fi
SANDBOX_WSL="/mnt/c/fpga-build/stepfpga"
mkdir -p "$SANDBOX_WSL"
cp -u "$HERE/read_com.ps1" "$SANDBOX_WSL/"
( cd "$SANDBOX_WSL" && powershell.exe -NoProfile -ExecutionPolicy Bypass -File read_com.ps1 -Port "$PORT" -Seconds "$DUR" )
