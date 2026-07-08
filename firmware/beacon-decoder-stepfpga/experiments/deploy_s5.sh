#!/usr/bin/env bash
# Build + flash S5 (dual correlator; code A = synthetic RC-osc emitter via virtual MCP3201, code B = REAL wired
# MCP3201 read over SPI). s5.v + spi_mcp3201.v + uart_bcn.v.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/s5"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM pnmain.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null || true; mkdir -p "$SB"
cp "$HERE"/s5.v "$HERE"/spi_mcp3201.v "$HERE"/uart_bcn.v "$HERE"/s5_pins.lpf "$HERE"/s5_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" s5_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/s5_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -25 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/s5_impl1.mrp" "$SB/impl1/s5_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy s5_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y s5_impl1.jed D:\\" )
echo "[done] Code A = synthetic (virtual ADC): LEDl/d2/rateA = live reference lock. Code B = REAL MCP3201 over SPI"
echo "       (SPI_CLK=M4 out, SPI_DO=N4 IN, SPI_CS=P3 out). BCN telemetry 'adc' field = raw REAL ADC (watch 0..4095"
echo "       as you sweep the ADC input 0->Vref). LEDr/d2-right/rateB = code-B correlator (won't lock on a DC input)."
