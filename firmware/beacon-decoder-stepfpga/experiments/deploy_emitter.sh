#!/usr/bin/env bash
# Build + flash the S1 synthetic Gold-code emitter (only emitter.v in the sandbox → unambiguous top).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'
SB="/mnt/c/fpga-build/emitter"
cmd.exe /c "taskkill /F /IM par.exe /IM pnmainc.exe /IM map.exe /IM synpwrap.exe" 2>/dev/null || true
usbipd.exe detach --busid 2-1 2>/dev/null || true; sleep 1
rm -rf "$SB" 2>/dev/null; mkdir -p "$SB"
cp "$HERE"/emitter.v "$HERE"/emitter_pins.lpf "$HERE"/emitter_build.tcl "$SB"/
echo "[build] Diamond (Export → JEDEC)…"
( cd "$SB" && "$PNMAINC" emitter_build.tcl ) >"$SB/build.log" 2>&1 || true
JED="$SB/impl1/emitter_impl1.jed"
if [ ! -f "$JED" ]; then echo "[build] NO .jed — log tail:"; tail -20 "$SB/build.log"; exit 1; fi
grep -iE 'Number of LUT4s|errors' "$SB/impl1/emitter_impl1.mrp" 2>/dev/null | grep -iE 'out of|^Number' | head -2
echo "[flash] copy emitter_impl1.jed → D:"; ( cd "$SB/impl1" && cmd.exe /c "copy /Y emitter_impl1.jed D:\\" )
echo "[done] scope: I/O 14 (P8)=gold code, trigger on I/O 15 (N8)=epoch (75 ms period). SW1 (M7) toggles A/B."
