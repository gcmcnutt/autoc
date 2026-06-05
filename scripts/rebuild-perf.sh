#! /bin/bash
set -ex

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
cd "$(dirname "$0")/.."
rm -rf build
mkdir build
cd build
cmake -DPERFORMANCE_BUILD=ON ..
make
