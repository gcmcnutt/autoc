#!/usr/bin/env bash
# Sweep carry-save multiplier configs "W:GROUP" → report Fmax + fit. No flash.
#   ./run_csa.sh 32:1 32:2 16:1     (default if none given)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
CFG=("$@"); [ ${#CFG[@]} -eq 0 ] && CFG=(32:1 32:2 16:1)

for c in "${CFG[@]}"; do
  W="${c%%:*}"; G="${c##*:}"
  SB="/mnt/c/fpga-build/csa_${W}_${G}"
  rm -rf "$SB"; mkdir -p "$SB"
  cp "$HERE"/csa.v "$HERE"/csa_pins.lpf "$HERE"/csa_build.tcl "$SB"/
  sed -i "s/localparam W = [0-9]*/localparam W = $W/;       s/localparam GROUP = [0-9]*/localparam GROUP = $G/" "$SB/csa.v"
  ( cd "$SB" && "$PNMAINC" csa_build.tcl ) >"$SB/build.log" 2>&1 || true
  TWR="$SB/impl1/csa_impl1.twr"; MRP="$SB/impl1/csa_impl1.mrp"
  FMAX=$(grep -ioE '[0-9.]+ ?MHz is the maximum frequency' "$TWR" 2>/dev/null | head -1 | grep -oE '^[0-9.]+ ?MHz')
  LUT=$(grep -iE 'Number of LUT4s' "$MRP" 2>/dev/null | grep -oE '[0-9]+ out of[ ]+[0-9]+ \([0-9]+%\)')
  REG=$(grep -iE 'Number of registers' "$MRP" 2>/dev/null | grep -oE '[0-9]+ out of[ ]+[0-9]+ \([0-9]+%\)')
  ERR=$(grep -ciE 'ERROR' "$SB/build.log" 2>/dev/null || echo 0)
  printf 'W=%-3s GROUP=%-2s | Fmax=%-12s | LUT=%-22s | REG=%-22s | logERR=%s\n' "$W" "$G" "${FMAX:-FIT?}" "${LUT:-?}" "${REG:-?}" "$ERR"
done
