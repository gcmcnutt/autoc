#!/usr/bin/env bash
# Build + flash S6 (dual correlator; code A = synthetic emitter DIGITALLY INJECTED, code B = REAL wired MCP3201
# over SPI; sample windows = circular-buffer distributed RAM). s7.v + spi_mcp3201.v + uart_bcn.v.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/s7"

# ---- LICENSE (msi, 2026-08-20) ----------------------------------------------------------------------
# Two failure modes cost an hour here, and BOTH present identically: pnmainc.exe exits 255 having printed
# NOTHING AT ALL (no FLEXlm error, empty build.log). If you see that, it is the license, every time.
#   1. The Windows env vars pointed at C:\Users\gcmcn\OneDrive\Documents\FPGA\license.dat, a path the
#      OneDrive Family->Business migration deleted (sync root is now "C:\OneDrive - AE", junction
#      C:\OneDrive). Now copied to C:\lscc\license.dat — OUTSIDE OneDrive, so Files-On-Demand cannot
#      dehydrate it out from under a build. setx'd to match, which also fixes the Diamond GUI.
#   2. A WSL-side `export LATTICE_LICENSE_FILE=...` DOES NOT REACH the Windows process. Windows env vars
#      flow INTO interop-launched exes, but WSL vars only cross if named in WSLENV. Hence the plumbing
#      below — without it the override silently does nothing and you debug the wrong thing.
# The license is node-locked to the Killer Wi-Fi NIC (HOSTID a002a51f3727), valid to 29-jun-2027. On a
# NIC change it must be regenerated on the Lattice portal — see /mnt/c/Incoming/instructions.md.
LIC_WIN="${LATTICE_LICENSE_FILE:-C:\\lscc\\license.dat}"
LIC_WSL="/mnt/c/${LIC_WIN#C:\\}"; LIC_WSL="${LIC_WSL//\\//}"
[ -f "$LIC_WSL" ] || { echo "[build] NO Lattice license at $LIC_WSL — regenerate or fix LATTICE_LICENSE_FILE"; exit 1; }
export LATTICE_LICENSE_FILE="$LIC_WIN" SALT_LICENSE_SERVER="$LIC_WIN"
export WSLENV="${WSLENV:+$WSLENV:}LATTICE_LICENSE_FILE:SALT_LICENSE_SERVER"
# -----------------------------------------------------------------------------------------------------

cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM pnmain.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null || true; mkdir -p "$SB"
cp "$HERE"/s7.v "$HERE"/spi_mcp3201.v "$HERE"/uart_bcn.v "$HERE"/s7_pins.lpf "$HERE"/s7_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" s7_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/s7_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -25 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|All preferences were met|not met' "$SB/impl1/s7_impl1.mrp" "$SB/impl1/s7_impl1.twr" 2>/dev/null | grep -iE 'out of|preferences' | head -3
echo "[flash] copy s7_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y s7_impl1.jed D:\\" )
echo "[done] Code A = DIGITALLY-INJECTED synthetic emitter (own OSCH clock): LEDl/rateA = live reference lock."
echo "       Code B = REAL MCP3201 over SPI (SPI_CLK=M4 out, SPI_DO=N4 IN, SPI_CS=P3 out); LEDr/rateB = code-B lock."
echo "       7-seg d2:d1 = real ADC upper byte HEX (0x00..0xFF as input sweeps 0..Vref); BCN 'adc' = raw 12-bit."
echo "       Controls: DIP1=enA, DIP2=enB (OFF=enabled); K1/K2=inject 1/2 bit-errors into code A."
echo "       USB (COM3): '+'/'-'=remote/local; 'E'=code-A rate, 'A'=code-A amp, 'K'=code-A burst; 0x80|mask knobs."
