#!/usr/bin/env bash
# Build + flash the MCP3201 SPI decode self-test (mcp3201_test.v + spi_mcp3201.v).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/mcp3201_test"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null; mkdir -p "$SB"
cp "$HERE"/mcp3201_test.v "$HERE"/spi_mcp3201.v "$HERE"/mcp3201_test_pins.lpf "$HERE"/mcp3201_test_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" mcp3201_test_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/mcp3201_test_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -20 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/mcp3201_test_impl1.mrp" "$SB/impl1/mcp3201_test_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy mcp3201_test_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y mcp3201_test_impl1.jed D:\\" )
echo "[done] LEDs: walking dot = PASS (decode OK); fast all-blink = FAIL (mismatch latched)."
echo "       scope: CS=P3, SCLK=M4 (50 kHz), DOUT=N4."
