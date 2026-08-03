#! /bin/bash
set -euo pipefail

# Clean optimized rebuild — autoc + crrcsim
#
# 035 FR-P04 / R3 — bit-replay regression gate basis:
#   The gate compares the per-scenario `ScenarioScore` vector
#   (computeScenarioScores output) byte-for-byte across builds, NOT whole-dmp
#   bytes. Whole-dmp equality is impossible post-035: stampEvalResultsProvenance
#   writes provenance timestamps and the .zst container adds compression framing
#   — both non-deterministic. The load-bearing invariant is fitness
#   reproducibility, which lives in the ScenarioScore numbers. (data.dat byte
#   equality was the pre-035 basis; data.dat is retired per FR-P05.)
#   The operator drives this gate; build single-threaded for FP determinism.
#
# ---------------------------------------------------------------------------
# 2026-08-02 — THREE GUARDS, each added because the failure it prevents is
# either silent or expensive. `rm -rf build` is a destructive first step, and
# this script had none of them.
# ---------------------------------------------------------------------------

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

# --- GUARD 1: a training run is using build/autoc RIGHT NOW ------------------
# The expensive one. Workers re-exec the binary, so overwriting build/autoc
# mid-run breaks a bake that may be 20+ hours in. Nothing about `rm -rf build`
# warns you first.
if pgrep -x autoc >/dev/null 2>&1; then
  echo "rebuild-perf: REFUSING — an autoc training run is active (pid $(pgrep -x autoc | tr '\n' ' '))." >&2
  echo "  This script does 'rm -rf build'. Workers re-exec build/autoc, so a rebuild" >&2
  echo "  mid-run breaks the bake. Wait for it, or set ALLOW_REBUILD_DURING_RUN=1 if" >&2
  echo "  you are certain (e.g. the run is eval-mode and already finished reading)." >&2
  [ "${ALLOW_REBUILD_DURING_RUN:-0}" = "1" ] || exit 1
  echo "rebuild-perf: ALLOW_REBUILD_DURING_RUN=1 set — proceeding anyway." >&2
fi

# --- GUARD 2: another rebuild is already in flight --------------------------
# Two concurrent `rm -rf build` produce a half-built tree with confusing
# symptoms ("getcwd() failed", "build.make: No such file or directory") that
# look like a code problem rather than a collision.
exec 9>"$REPO/.rebuild-perf.lock"
if ! flock -n 9; then
  echo "rebuild-perf: REFUSING — another rebuild-perf.sh holds the lock." >&2
  echo "  Two concurrent 'rm -rf build' runs corrupt each other's tree." >&2
  exit 1
fi

set -x
rm -rf build
mkdir build
cd build
cmake -DPERFORMANCE_BUILD=ON ..
make
set +x

# --- GUARD 3: prove the gate actually RAN, don't infer it from $? -----------
# `make` runs the test suites via the run_autoc_tests ALL target. A build that
# dies partway can still leave a zero exit visible to a caller that appends
# another command, and "no tests ran" then reads as "no tests failed". The gate
# is only meaningful if the suites executed, so count them.
cd "$REPO"
EXPECTED="$(grep -c 'add_test(NAME' CMakeLists.txt)"
echo
echo "rebuild-perf: gate self-check — expecting ${EXPECTED} suites to have run."
echo "  (This script's exit status alone is NOT sufficient evidence. If you piped"
echo "   output to a log, verify:  grep -c 'Running main() from' <log>  ==  ${EXPECTED})"
for t in autoc renderer dmp-dump; do
  [ -x "build/$t" ] || { echo "rebuild-perf: FAILED — build/$t missing after a 'successful' build." >&2; exit 1; }
done
echo "rebuild-perf: binaries present (autoc, renderer, dmp-dump). Build OK."
