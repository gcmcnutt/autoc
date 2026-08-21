#!/usr/bin/env bash
# Compile + run a correlator testbench under iverilog (WSL-native, no Diamond/Windows needed).
# +define+SIM selects the DUT's ÷100 sim-scaled clock dividers so a lock lands in ms of sim time.
#   ./run.sh                 # s6 testbench (default), PASS/FAIL summary
#   ./run.sh s7              # s7 testbench at the platform rate (288 Hz sample / 120 Hz chip)
#   ./run.sh s6 --gtk        # also open the VCD in gtkwave (if installed)
# SINGLE-RATE PLATFORM (2026-08-20): there is no rate switch any more — +define+CHIP200 was removed from
# s7.v, and tb_s7.v's CHIP_NS is the matching single constant. If you ever reintroduce a second rate set,
# the DUT and the testbench must move TOGETHER or code B silently never locks.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP="$HERE/../experiments"

DUT=s6
DEFS=(-DSIM)
for a in "$@"; do
  case "$a" in
    s6|s7)      DUT="$a" ;;
    --gtk)      ;;                      # handled below
    *) echo "usage: $0 [s6|s7] [--gtk]" >&2; exit 2 ;;
  esac
done

OUT="$HERE/${DUT}_sim.vvp"
echo "[sim] compiling $DUT (iverilog ${DEFS[*]})…"
iverilog -g2012 "${DEFS[@]}" -o "$OUT" \
  "$EXP/$DUT.v" "$EXP/spi_mcp3201.v" "$EXP/uart_bcn.v" \
  "$HERE/osch_stub.v" "$HERE/tb_$DUT.v"
echo "[sim] running (vvp)…"
( cd "$HERE" && vvp "$OUT" )
if [[ " $* " == *" --gtk "* ]] && command -v gtkwave >/dev/null 2>&1; then
  ( cd "$HERE" && gtkwave "$DUT.vcd" >/dev/null 2>&1 & )
fi
