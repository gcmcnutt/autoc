#!/usr/bin/env bash
# Build the StepFPGA gateware from WSL: sync repo source -> /mnt/c Windows sandbox, run Lattice Diamond
# via pnmainc.exe (Windows interop), copy the .jed back. Optionally flash by copying to the STEPLink D:.
#
#   firmware/beacon-decoder-stepfpga/   = source-of-truth (this repo, git, edited in WSL)
#   /mnt/c/fpga-build/stepfpga/         = transient build sandbox (gitignored; Diamond needs a native C: CWD)
#
# Usage:
#   ./build.sh            # build -> .jed back into ./build/
#   ./build.sh --flash    # build, then copy the .jed onto D:\ (STEPLink) to program + restart the board
#
# Why the sandbox: Diamond rejects \\wsl$ UNC working dirs (toolchain-plan §1). WSL cannot see the
# STEPLink D: volume, so the flash step uses a Windows-side `cmd.exe copy` over interop (§3.5).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="stepfpga"; IMPL="impl1"
SANDBOX_WSL="/mnt/c/fpga-build/$PROJ"
PNMAINC='/mnt/c/lscc/diamond/3.14/bin/nt64/pnmainc.exe'   # WSL path; binfmt interop runs the .exe

echo "[build] sync source -> $SANDBOX_WSL"
mkdir -p "$SANDBOX_WSL"
# cp -u (only-if-newer) preserves Diamond's mtime-based incremental detection — `cp -f` would bump every
# file's mtime each run and force a full rebuild. Edit a source file → its mtime advances → only then re-synced.
cp -u "$HERE"/rtl/*.v          "$SANDBOX_WSL"/ 2>/dev/null || true
cp -u "$HERE"/constraints/*.lpf "$SANDBOX_WSL"/ 2>/dev/null || true
cp -u "$HERE"/build.tcl        "$SANDBOX_WSL"/

echo "[build] Diamond via interop (CWD=sandbox so Windows sees C:\\ not UNC; incremental — only changed stages re-run)"
( cd "$SANDBOX_WSL" && "$PNMAINC" build.tcl )

# Diamond writes implementation outputs into the <impl>/ subdir.
JED="$(ls -1 "$SANDBOX_WSL/$IMPL/${PROJ}_${IMPL}.jed" 2>/dev/null || true)"
[[ -z "$JED" ]] && { echo "[build] ERROR: no .jed produced (looked in $SANDBOX_WSL/$IMPL/)"; exit 1; }
mkdir -p "$HERE/build"; cp -f "$JED" "$HERE/build/"
echo "[build] OK -> build/$(basename "$JED")"

if [[ "${1:-}" == "--flash" ]]; then
  echo "[flash] copy ${PROJ}_${IMPL}.jed -> D:\\ (STEPLink — programs + restarts the board)"
  ( cd "$SANDBOX_WSL/$IMPL" && cmd.exe /c "copy /Y ${PROJ}_${IMPL}.jed D:\\" )
fi
