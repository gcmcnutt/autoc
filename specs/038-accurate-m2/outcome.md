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

### T007/T008/T009 (P0-D crrcsim + protocol) → **DONE** (2026-07-01)

- **T007** (P0-D-1): `getSimulationTimeSinceReset()` rounds (`std::llround`) not truncates → exact 50 ms
  gaps; strict single-gap source-spacing restored (crrcsim + tracker_stepper). ⚠️ [OP] confirm at the
  rebuild-perf gate whether this settles the "identical-state/different-simTimeMsec" divergence noted in
  crrcsim CLAUDE.md. Commit crrcsim `e6108cc`, parent `55fd4df`.
- **T008** (P0-D-3): steady wind recorded via `eom01->getLastLocalAirmass()` (NED ft/s→m/s) → both modes.
  ⚠️ [OP] re-run `wind_study.py` against a fresh bake to confirm the getter is populated.
- **T009** (P0-D-2): `RecordedRunConfig` (fit cone / cadence / crash-penalty) appended to `EvalResults`
  (no version bump), stamped from `ConfigManager::getConfig()`. Commit `0cf29cf`.

### T009b (FR-P0H situational-awareness inputs) — **PART 1/2 DONE: layout** (2026-07-01, commit `3cd2bc9`)

Layout only (Tracker 54→60, Pathgen 33→37; weight counts 2051 pathgen / 2787 tracker; topology strings).
Population (part 2) is the next focused unit. **Recorded decisions for part 2**:
- **(B) `inward_body`** = `orientation.inverse() * normalize(-pos_horizontal)` (radial-inward toward the
  cylinder axis at the virtual-frame origin, body frame). Add a stateless `inwardBodyDirection()` helper to
  `arena.h`/`arena.cc`; populate in gather_tracker (has arena) and gather_pathgen.
- **Pathgen (B) arena plumb**: add `const FlightArena&` to `gather_pathgen_inputs`; `NNControllerBackend`
  carries one from config (desktop); **nn2cpp bakes a compile-time `FlightArena{radius,floor,ceiling}`
  literal from the run config** into the generated xiao firmware (operator 2026-07-01). Live-pathgen
  firmware change (FR-033).
- **(A) two-path state**: `time_since_seen` (tanh of blind-ticks) + `exit_dir` sin/cos (held last-seen NDC
  centroid bearing). Single-source the update in a shared helper called by BOTH `TrackerStepper` and
  `CrrcsimTrackerHelper`; **reset per scenario/engage** in both `initScenario` (FR-030 determinism).
- **dmp_dump**: add the 6/4 new columns (honest recording).
- Quaternion convention: world→body via `.inverse()` (matches `camera_projection.cc:117`,
  `aircraft_state.h:440`).
- ⚠️ layout compiles but gather funcs don't populate the new fields yet → **no bake until part 2 lands**
  (uninitialized = non-deterministic); test fixtures with hardcoded 54/33/1923/2595 fail until T011.

### T005 / P0-E — type-domain grep audit → **AUDITED; conversions deferred** (2026-07-01)

Grep on `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` returns ~430 unannotated
`float`/`double` hits — the **codebase-wide backfill** that Constitution VI explicitly defers to a
separate audit-pass spec ("enforced incrementally on touched code"). 038 has not yet touched any
eval/nn code, so there is no 038-introduced drift. Type conversions are determinism-affecting and must
ride a `rebuild-perf.sh` gate — not blind Phase-1 edits. The per-milestone VI audit on 038-touched code
runs at T035.
