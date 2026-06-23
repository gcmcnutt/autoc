#!/usr/bin/env bash
# Build the at-speed multiplier self-test and FLASH it to the StepFPGA (board stays on Windows; copy .jed → D:).
# Only selftest.v goes to the sandbox (it is self-contained), so the top is unambiguous.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/selftest"

usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1     # ensure board (D:) is on the Windows side
rm -rf "$SB"; mkdir -p "$SB"
cp "$HERE"/selftest.v "$HERE"/fast_pll.v "$HERE"/selftest_pins.lpf "$HERE"/selftest_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" selftest_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/selftest_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -25 "$SB/build.log"; exit 1; fi
echo "[timing]"; grep -iE 'maximum frequency|All preferences were met|not met|Number of LUT4s' "$SB/impl1/selftest_impl1.twr" "$SB/impl1/selftest_impl1.mrp" 2>/dev/null | head -5
echo "[flash] copy selftest_impl1.jed → D: (STEPLink)"
( cd "$SB/impl1" && cmd.exe /c "copy /Y selftest_impl1.jed D:\\" )
echo "[done] watch the board (LEDs are ACTIVE-LOW): moving counter/ripple = running clean @108 MHz, locked,"
echo "       0 errors. All 8 blinking together = error. (No motion / dark = PLL not locked.)"
