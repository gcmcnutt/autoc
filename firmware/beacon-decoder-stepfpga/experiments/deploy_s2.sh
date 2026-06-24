#!/usr/bin/env bash
# Build + flash S2a (virtual MCP3201 ADC + soft SPI master + emitter). Only s2.v in the sandbox.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/s2"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null; mkdir -p "$SB"
cp "$HERE"/s2.v "$HERE"/spi_mcp3201.v "$HERE"/s2_pins.lpf "$HERE"/s2_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" s2_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/s2_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -20 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/s2_impl1.mrp" "$SB/impl1/s2_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy s2_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y s2_impl1.jed D:\\" )
echo "[done] LEDs = top 8 bits of the virtual-ADC sample (flickers w/ the 200 Hz code; SW1 toggles A/B)."
echo "       scope still: I/O14(P8)=code, I/O15(N8)=epoch."
