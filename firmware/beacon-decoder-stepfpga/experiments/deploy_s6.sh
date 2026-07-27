#!/usr/bin/env bash
# Build + flash S6 (dual correlator; code A = synthetic emitter DIGITALLY INJECTED, code B = REAL wired MCP3201
# over SPI; sample windows = circular-buffer distributed RAM). s6.v + spi_mcp3201.v + uart_bcn.v.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/s6"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM pnmain.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null || true; mkdir -p "$SB"
cp "$HERE"/s6.v "$HERE"/spi_mcp3201.v "$HERE"/uart_bcn.v "$HERE"/s6_pins.lpf "$HERE"/s6_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" s6_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/s6_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -25 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/s6_impl1.mrp" "$SB/impl1/s6_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy s6_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y s6_impl1.jed D:\\" )
echo "[done] Code A = DIGITALLY-INJECTED synthetic emitter (own OSCH clock): LEDl/rateA = live reference lock."
echo "       Code B = REAL MCP3201 over SPI (SPI_CLK=M4 out, SPI_DO=N4 IN, SPI_CS=P3 out); LEDr/rateB = code-B lock."
echo "       7-seg d2:d1 = real ADC upper byte HEX (0x00..0xFF as input sweeps 0..Vref); BCN 'adc' = raw 12-bit."
echo "       Controls: DIP1=enA, DIP2=enB (OFF=enabled); K1/K2=inject 1/2 bit-errors into code A."
echo "       USB (COM3): '+'/'-'=remote/local; 'E'=code-A rate, 'A'=code-A amp, 'K'=code-A burst; 0x80|mask knobs."
