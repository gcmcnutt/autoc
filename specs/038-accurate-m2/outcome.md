# 038 Accurate M2 — Outcome / Findings

Running record of per-task verdicts, escapes, and pinned artifacts. Populated during
`/speckit.implement`; consolidated at close (T037).

## Phase 0 — Prework

### T001 / P0-A — 033 PRNG cascade determinism validation → **CLEAR** (2026-07-01)

**Verdict**: CLEAR — no bug found. The 033 PRNG-cascade rework (`acf732f`) deterministically
regenerates a scenario from its seeds; the suspected single-SHA bug did not reproduce.

**Method**: full `make -C build` (the project's regression pattern — the gtest suites run as part of
the build). All suites **PASS**, zero failures, including `scenario_prng_tests` (cascade D1–D5 contract)
and `eval_mode_replay_tests` (cross-process bitwise replay). No code change required — validation was the
deliverable per FR-P0A.

**Note**: the tasks.md T001 command originally cited `./tests/run_autoc_tests --gtest_filter=...`; this
build has no standalone test binary — `make` (or `make run_autoc_tests`) builds and runs the gtest
suites inline. Command corrected in tasks.md.

### T002 / P0-F — streak threshold revert → **DONE** (2026-07-01)

Reverted `FitStreakThreshold` 0.3→0.5 in `autoc-tracker.ini`, `autoc-eval-tracker.ini`,
`autoc-eval-tracker-visual.ini` (M1 inis already 0.5). Reward-shaping is exhausted as a depth lever
(037 t15 gain was threshold-confounded); the architecture studies start from the un-confounded cone.

### T003 / P0-E — svTau dead-path removal → **NO-OP / already done** (2026-07-01)

The svTau/servoTau code dead-path was **already removed 2026-06-12** (`craft_variation.h:150-151`:
"servo first-order tau draw removed 2026-06-12 — v2 has no lag term; draw order shifts accordingly,
which is fine cross-build"; `scenario_metadata.h:118`). No residual draw-order placeholder in the craft
PRNG cascade. The only remaining `svTau` reference is in the historical `verify_037_metrics.py`, left
untouched per the historical-scripts-immutable practice. No code change needed.

### T005 / P0-E — type-domain grep audit → **AUDITED; conversions deferred** (2026-07-01)

Grep on `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` returns ~430 unannotated
`float`/`double` hits — the **codebase-wide backfill** that Constitution VI explicitly defers to a
separate audit-pass spec ("enforced incrementally on touched code"). 038 has not yet touched any
eval/nn code, so there is no 038-introduced drift. Type conversions are determinism-affecting and must
ride a `rebuild-perf.sh` gate — not blind Phase-1 edits. The per-milestone VI audit on 038-touched code
runs at T035.
