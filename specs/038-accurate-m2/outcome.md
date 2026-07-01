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

#### T009b part-2 recon map (file:line — start here in a fresh /speckit.implement)

- **(B) `inward_body` helper**: add `inwardBodyDirection(const gp_vec3& pos, const gp_quat& q)` decl to
  `include/autoc/eval/arena.h` (near `distanceToBoundary`, ~line 98), body in `src/eval/arena.cc` (near
  `distanceToBoundary` at `arena.cc:48`). Math: arena centered at virtual-frame origin (0,0,0); world
  radial-inward = `normalize(-pos.x, -pos.y, 0)` (Zero if `hypot(x,y)<1e-6`); body = `q.inverse() *
  inward_world`. Convention verified: world→body = `.inverse()` (`camera_projection.cc:117`,
  `aircraft_state.h:440`).
- **`gather_tracker_inputs`** (`src/nn/evaluator.cc:438`): populate (B) `inward_body_{x,y,z}` from
  `chase.getPosition()`+`getOrientation()`; write (A) `time_since_seen`/`exit_dir_{sin,cos}` from values
  passed by the caller. Existing `dist_to_boundary_along_vel` at `evaluator.cc:488`.
  - callers: `src/eval/tracker_stepper.cc:254`, `crrcsim/.../crrcsim_tracker_helper.cpp:202`.
- **`gather_pathgen_inputs`** (`src/nn/evaluator.cc:318`): add `const autoc::eval::FlightArena&` param;
  populate (B) `dist_to_boundary` (via `distanceToBoundary`) + `inward_body`. NO (A) (M1 always visible).
  - callers: `src/nn/evaluator.cc:404` (`NNControllerBackend::evaluate` — give the backend a `FlightArena`
    member from config, `evaluator.h` ctor ~line 371); `tools/nn2cpp.cc:123,189` (codegen — **bake a
    `FlightArena{radius,floor,ceiling}` literal from config** per operator decision).
- **(A) two-path state** (`time_since_seen` = `tanh(blind_ticks·dt / ~2s)`; `exit_dir` = held NDC-centroid
  bearing of `history.left/right_{x,y}[5]`, updated when a beacon is visible = `cep < cep_gate_threshold`,
  `kCepSentinelThreshold` per `fitness_decomposition.cc:200`): single-source the per-tick update in a shared
  helper; add members + **reset in `initScenario`** to both `TrackerStepper` (`tracker_stepper.h` +
  `tracker_stepper.cc` ~108) and `CrrcsimTrackerHelper` (`crrcsim_tracker_helper.cpp` initScenario ~59-62).
- **dmp_dump columns** (`tools/dmp_dump.cc`): add the 6 tracker + 4 pathgen input columns (honest recording).
- **T011 fixtures to regen**: `tests/nn_layout_tests.cc`, `nn_evaluator_tests`, `nn_serialization_tests`,
  `tick_rescale_tests`, NN01 fixtures (hardcoded 54/33/1923/2595); xiao codegen regen via `nn2cpp`.
- **Build loop**: operator runs `cd build && make` (compiles + runs gtest) for source edits; `rebuild-perf.sh`
  for the from-scratch determinism gate. No standalone test binary — `make` IS the unit-test run.

### T009b (FR-P0H situational-awareness inputs) — **PART 2/2 DONE: population** (2026-07-01)

Gather functions now populate every new field; clean `rebuild-perf.sh` (PERFORMANCE_BUILD) is **GREEN**
(autoc + crrcsim 100% + all `run_autoc_tests` gtests PASS). **T009b is complete** (layout + population).

**What landed (population):**
- **(B) `inwardBodyDirection(pos, q)`** — stateless helper in `arena.h`/`arena.cc:91`. World radial-inward
  `normalize(-x, -y, 0)` (Zero if `hypot(x,y)<1e-6`), rotated world→body via `q.inverse()`. All-attitude.
- **`gather_pathgen_inputs`** now takes `const FlightArena&`; populates (B) `dist_to_boundary` (tanh, same
  scale as tracker slot 44) + `inward_body_{x,y,z}`. M1 gets (B) only (never blind).
- **`gather_tracker_inputs`** now takes `const SituationalAwarenessState&`; populates (B) `inward_body` +
  writes (A) via `sa.writeInputs()`.
- **(A) `SituationalAwarenessState`** (new struct in `evaluator.h`) — `blind_ticks` + held `exit_dir_sin/cos`;
  single-sourced `update(left/right x/y/cep, kCepSentinelThreshold)` + `writeInputs()` (`time_since_seen =
  tanh(blind·dt / kTimeSinceSeenScale_s=2s)`). Visibility = CEP `< kCepSentinelThreshold` (matches
  `fitness_decomposition.cc:200`). Held member on **both** `TrackerStepper` and `CrrcsimTrackerHelper`;
  **reset in initScenario** (both), advanced each real tick after `projectAndShiftHistory` (NOT during the
  history pre-fill) → deterministic per FR-030.
- **`NNControllerBackend` ctor** gained `const FlightArena&` (no default per M2 policy). Call sites updated:
  crrcsim `inputdev_autoc.cpp:522` passes `init_.flightArena`; test fixtures pass `FlightArena{}`.
- **`mode.cc`**: `kPathgenMode.gather_inputs` → new `gather_pathgen_inputs_signature_mismatch_guard` (the
  bundle pointer is vestigial; real dispatch is via `NNControllerBackend::evaluate` / nn2cpp codegen).
- **`nn2cpp`**: both codegen paths bake a `static const FlightArena arena{R,F,C}` literal (new `-a R,F,C`
  flag, default 80/5/100 = struct defaults / autoc.ini M1 config) and pass it to `gather_pathgen_inputs`.
- **`dmp_dump`**: honest recording — reads the actual per-tick stored `TrackerInputs`/`NNInputs`
  (`st.getTrackerInputs()` / `getNNInputs()`), NOT a re-derivation. Tracker +6 cols
  (`inX,inY,inZ,tSee,exS,exC`); pathgen +4 (`dBnd,inX,inY,inZ`) in both path + no-path branches.
- **Fixtures regenerated to green** (part of this pass, satisfies T011's "confirm green"):
  `nn_sensor_interface_tests` (33→37 / 54→60 counts, new anchors, pathgen meta-walk header +4 cols),
  `contract_evaluator_tests` (1923→2051), `nn_evaluator_tests` (33→37), `gather_tracker_inputs_tests`
  (float[54]→[60]), all `gather_tracker_inputs` / backend-ctor call sites. New weight counts confirmed in
  `topology.h`: pathgen **2051** ("37,32,16r,3"), tracker **2787** ("60,32,16r,3").

**⚠️ Still open (NOT part of T009b):**
- **T009c** — dedicated behavioral + determinism unit test for the sit-awareness inputs (increment/reset on
  CEP crossing, inverted-attitude inward_body, per-scenario-reset determinism). Not yet written.
- **xiao firmware regen (FR-033)** — `xiao/src/generated/nn_program_generated.cpp` still calls the OLD
  `gather_pathgen_inputs` signature and the xiao PlatformIO build must add `arena.cc` (distanceToBoundary +
  inwardBodyDirection) to its sources. Firmware is out of the desktop CMake build, so this does NOT block
  `make`; it's operator/firmware work before the next live-pathgen flash. Regen via
  `nn2cpp -i <weights> -a <R,F,C>`.
- **T013 [OP]** — the enriched re-bake + payoff measurement is still gated on this landing (now unblocked).

### T005 / P0-E — type-domain grep audit → **AUDITED; conversions deferred** (2026-07-01)

Grep on `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` returns ~430 unannotated
`float`/`double` hits — the **codebase-wide backfill** that Constitution VI explicitly defers to a
separate audit-pass spec ("enforced incrementally on touched code"). 038 has not yet touched any
eval/nn code, so there is no 038-introduced drift. Type conversions are determinism-affecting and must
ride a `rebuild-perf.sh` gate — not blind Phase-1 edits. The per-milestone VI audit on 038-touched code
runs at T035.
