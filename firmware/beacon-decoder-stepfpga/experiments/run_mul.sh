#!/usr/bin/env bash
# Sweep the pipelined-multiplier depth and report achievable Fmax + utilization for each.
#   ./run_mul.sh            # sweep STAGES = 1 4 8 16
#   ./run_mul.sh 8 24       # sweep specific depths
# Each depth builds in its own /mnt/c sandbox (so runs don't collide). No flash — .twr/.mrp are the goal.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
STAGESET=("$@"); [ ${#STAGESET[@]} -eq 0 ] && STAGESET=(1 4 8 16)

for N in "${STAGESET[@]}"; do
  SB="/mnt/c/fpga-build/pmul_$N"
  rm -rf "$SB"; mkdir -p "$SB"
  cp "$HERE"/mul.v "$HERE"/mul_pins.lpf "$HERE"/mul_build.tcl "$SB"/
  sed -i "s/localparam STAGES = [0-9]*/localparam STAGES = $N/" "$SB/mul.v"
  ( cd "$SB" && "$PNMAINC" mul_build.tcl ) >"$SB/build.log" 2>&1 || true
  TWR="$SB/impl1/pmul_impl1.twr"; MRP="$SB/impl1/pmul_impl1.mrp"
  FMAX=$(grep -iE 'maximum frequency' "$TWR" 2>/dev/null | head -1 | sed -E 's/.*Report:[[:space:]]*//; s/.* is //' )
  MET=$(grep -iE 'All preferences were met|not met' "$TWR" 2>/dev/null | head -1)
  LUT=$(grep -iE 'Number of LUT4s' "$MRP" 2>/dev/null | head -1 | sed -E 's/^[[:space:]]*//')
  REG=$(grep -iE 'Number of registers' "$MRP" 2>/dev/null | head -1 | sed -E 's/^[[:space:]]*//')
  printf 'STAGES=%-3s | Fmax: %-28s | %-22s | %s\n' "$N" "${FMAX:-?}" "${LUT:-?}" "${REG:-?}"
done
