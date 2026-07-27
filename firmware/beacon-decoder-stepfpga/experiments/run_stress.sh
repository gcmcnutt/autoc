#!/usr/bin/env bash
# Build the timing/utilization stress design via Diamond (WSL→Windows interop) and print the feedback:
#   .mrp = resource utilization (LUT/register/EBR vs the 4320-LUT device)
#   .twr = timing (per-constraint slack, errors, achievable Fmax)
# No flash — this is a synthesis/map/par/trce analysis exercise.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/stress"
mkdir -p "$SB"
cp -u "$HERE"/stress.v "$HERE"/stress_pins.lpf "$HERE"/stress_build.tcl "$SB"/
( cd "$SB" && "$PNMAINC" stress_build.tcl ) | tail -5

MRP="$SB/impl1/stress_impl1.mrp"; TWR="$SB/impl1/stress_impl1.twr"
echo ""; echo "==================== UTILIZATION (.mrp) ===================="
grep -iE 'Number of (registers|SLICEs|LUT4s|LUTs|IO|EBR|PIO)|utiliz|out of|Design Summary' "$MRP" 2>/dev/null | head -25
echo ""; echo "==================== TIMING (.twr) ===================="
grep -iE 'Preference|FREQUENCY|Timing errors|Score|slack|met\.|not met|MHz|Fmax|negative' "$TWR" 2>/dev/null | head -40
