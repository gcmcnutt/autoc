#!/usr/bin/env bash
# Push this checkout to beaconpi5's source tree.
#
# WHY THIS EXISTS (bench journal, 2026-08-20): the Pi's tree is an rsync'd COPY, not a git checkout, and it
# silently DRIFTED -- an older tests/golden/test_replay_parity.c plus a stray mis-pathed engine.c made
# beacon_golden_test_replay_parity fail 3/17 on the Pi while the same commit passed 10/10 on the dev box.
# Hours went into chasing a "Pi-only bug" that was really a stale file. Ad-hoc `rsync -a src/` invocations
# are what let that happen, so the sync is a script with --delete and one authoritative exclude list.
#
#   ./sync-to-pi.sh            # dry run -- ALWAYS look at this first, especially the 'deleting' lines
#   ./sync-to-pi.sh --go       # actually sync
#   PI=pi@x.y.z.w ./sync-to-pi.sh --go
#
# --checksum, not the default size+mtime: the tree is written by several machines (dev box, msi) whose
# mtimes disagree, so content is the only trustworthy comparison. It costs a few seconds on 150 MB.
#
# EXCLUDES are deliberate, not incidental. The Pi builds firmware/beacon-receiver and nothing else, so
# crrcsim/ (the sim submodule), cad/ and flight-results/ are ~490 MB of dead weight on the flight host.
# Because rsync never deletes an excluded path, anything already there is also PROTECTED -- notably the
# Pi's own build/ and /data recordings.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PI="${PI:-pi@100.97.242.96}"
DEST="${DEST:-autoc-beacon/}"

EXCLUDES=(
  --exclude '.git/'          # the Pi has no git; a .git here would be a half-repo, worse than none
  --exclude 'build/'         # Pi-local, and cross-arch objects from here would be poison
  --exclude 'build-*/'
  --exclude '__pycache__/' --exclude '*.pyc'
  --exclude '.venv/' --exclude '.pio/' --exclude 'node_modules/'
  --exclude 'crrcsim/'       # sim submodule -- never built on the flight host
  --exclude 'cad/'           # KiCad projects + datasheet PDFs
  --exclude 'flight-results/' --exclude 'eval-results/'
)

MODE=(--dry-run); TAG="DRY RUN (pass --go to apply)"
[[ "${1:-}" == "--go" ]] && { MODE=(); TAG="APPLYING"; }
echo "== sync $REPO -> $PI:$DEST  [$TAG] =="
rsync -a --checksum --delete --info=del,name1 "${MODE[@]}" "${EXCLUDES[@]}" "$REPO/" "$PI:$DEST"
echo "== done =="
[[ -z "${MODE[*]}" ]] && echo "Reminder: the Pi build is NOT rebuilt by this script -- see pi/INSTALL.md."
