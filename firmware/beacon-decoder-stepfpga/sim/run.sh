#!/usr/bin/env bash
# Compile + run the s6 correlator testbench under iverilog (WSL-native, no Diamond/Windows needed).
# +define+SIM selects the s6.v ÷100 sim-scaled clock dividers so a lock lands in ms of sim time.
#   ./run.sh            # run the s6 testbench, print the PASS/FAIL summary
#   ./run.sh --gtk      # also open the VCD in gtkwave (if installed)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXP="$HERE/../experiments"
OUT="$HERE/s6_sim.vvp"
echo "[sim] compiling (iverilog +define+SIM)…"
iverilog -g2012 -DSIM -o "$OUT" \
  "$EXP/s6.v" "$EXP/spi_mcp3201.v" "$EXP/uart_bcn.v" \
  "$HERE/osch_stub.v" "$HERE/tb_s6.v"
echo "[sim] running (vvp)…"
( cd "$HERE" && vvp "$OUT" )
if [[ "${1:-}" == "--gtk" ]] && command -v gtkwave >/dev/null 2>&1; then
  ( cd "$HERE" && gtkwave s6.vcd >/dev/null 2>&1 & )
fi
