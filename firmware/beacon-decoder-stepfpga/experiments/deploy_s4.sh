#!/usr/bin/env bash
# Build + flash S3a (matched-filter correlator + bar-graph + RGB lock; s4.v + spi_mcp3201.v).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/s4"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM pnmain.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null || true; mkdir -p "$SB"
cp "$HERE"/s4.v "$HERE"/spi_mcp3201.v "$HERE"/uart_bcn.v "$HERE"/s4_pins.lpf "$HERE"/s4_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" s4_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/s4_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -25 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/s4_impl1.mrp" "$SB/impl1/s4_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy s4_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y s4_impl1.jed D:\\" )
echo "[done] 8 LEDs = q bars (L4=codeA,R4=codeB); LEDl/LEDr = A/B lock (red/yel/grn). DIP1/2/3=mute A/B/noise, DIP4=ext/synth code-B; K1-K4=inj1/inj2/weak/floor."
echo "       P8=EXTERNAL emitter code IN (416 DIM via 5V->3.3V divider); N8=synthetic-A epoch. LEDr/d2/rateB = real-emitter lock."
