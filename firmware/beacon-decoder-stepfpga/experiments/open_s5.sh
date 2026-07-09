#!/usr/bin/env bash
# Open the s5 gateware in the Lattice Diamond GUI (pnmain.exe) for MANUAL work — floorplan, physical view,
# post-route timing analysis (TRACE), package view, netlist analyzer, etc.
#
# There is no hand-built project: the headless flow (deploy_s5.sh / s5_build.tcl) already generates a real
# Diamond project (s5.ldf + s5.lpf + impl1/ results) in the /mnt/c sandbox. Diamond rejects \\wsl$ UNC working
# dirs, so the project MUST live on C: — the repo experiments/ dir stays the git source-of-truth.
#
#   ./open_s5.sh          # sync sources -> sandbox, build if needed, open the GUI on the built project
#
# WORKFLOW / source-of-truth:
#   - Edit RTL in the repo (experiments/*.v); re-sync with ./deploy_s5.sh (or this script). The GUI shows the
#     SANDBOX copies — don't treat them as authoritative.
#   - Floorplan/timing exploration in the GUI is non-destructive.
#   - If you change constraints in the GUI (pin locks / timing preferences), copy the sandbox s5.lpf back to
#     experiments/s5_pins.lpf so git keeps them.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIA='/mnt/c/lscc/diamond/3.14/bin/nt64'
SB="/mnt/c/fpga-build/s5"
mkdir -p "$SB"
# cp -u = only-if-newer (preserves Diamond's incremental mtime detection)
cp -u "$HERE"/s5.v "$HERE"/spi_mcp3201.v "$HERE"/uart_bcn.v "$HERE"/s5_pins.lpf "$HERE"/s5_build.tcl "$SB"/
# Build headless if no results yet, so Floorplan/Timing views open with data (not an empty un-run project).
if [ ! -f "$SB/s5.ldf" ] || [ ! -f "$SB/impl1/s5_impl1.jed" ]; then
  echo "[open] no built project yet -> running a headless build first…"
  ( cd "$SB" && "$DIA/pnmainc.exe" s5_build.tcl ) >"$SB/build.log" 2>&1 || true
fi
[ -f "$SB/s5.ldf" ] || { echo "[open] ERROR: $SB/s5.ldf not generated — check $SB/build.log"; exit 1; }
echo "[open] launching Diamond GUI on C:\\fpga-build\\s5\\s5.ldf …"
# start "" <exe> <proj> : detach so this shell returns while the GUI stays open
cmd.exe /c start "" "C:\\lscc\\diamond\\3.14\\bin\\nt64\\pnmain.exe" "C:\\fpga-build\\s5\\s5.ldf"
echo "[open] GUI launched. In Diamond: Tools > Floorplan View / Physical View / Package View;"
echo "       Analysis (post-route) > Timing Analysis (TRACE). If constraint edits matter, copy sandbox"
echo "       s5.lpf back to experiments/s5_pins.lpf."
