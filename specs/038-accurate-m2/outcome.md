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

### Enrichment early-training tax — basic-m1 diagnostic (2026-07-02, t2)

**Question**: did the +4 FR-P0H (B) inputs (33→37, 1923→2051 weights) hurt M1 learning, or is the
stopped full-M1 rebake (t1) just the basin lottery? **Answer: an early-training tax, not fatal.**

Ran `autoc-basic-m1.ini` (pop 3000, single longSequential, 16 winds, all four variation classes) —
run `autoc-038-t2-basic-m1-diag`, master seed 1783003482. Compared against the **exact shape-match**
`autoc-037-t5-basic-m1-20hz` (same pop/cadence/paths/winds/variations; ONLY 33in/1923w vs 37in/2051w).
Comparator = `avgMaxStreak` / `pctInStreak` (servo-era progress metrics; raw fitness is a noisy cross-run
comparator per [project_late_run_fitness_interpretation]). Matched-gen:

| gen | 038 avgMaxStreak / pctInStreak | 037-t5 |
|----:|:------------------------------:|:------:|
| 10  | 2.6 / 1.5 | **6.0 / 5.2** |
| 20  | 3.1 / 2.3 | **6.0 / 5.2** |
| 50  | 1.6 / 0.4 | **6.8 / 5.3** |
| 100 | **5.7 / 2.4** | 4.2 / 2.1 |
| 173 | 5.8 / 2.5 | **8.2 / 2.3** |

**Finding**: the enriched net is **materially worse through ~gen 50** (037 reaches usable streaks ~6 by
gen 10 and holds; 038 sits at 2–3 and *dips to 1.6 @ gen 50*), crosses over ~gen 100, and by gen 400
reaches strong tracking on its own (avgMaxStreak 17.7, pctInStreak 8.6, target dist 12 m→9.6 m,
target-visible occ 3–4%→8%). So the 4 extra inputs impose an early-training tax (bigger weight space),
NOT a capability loss — consistent with the operator's "needs more than 400 gens" for the full-M1 shape
where the tax compounds over pop 5000 / 294 scenarios. Determinism unaffected: an M1 eval
(`autoc-basic-m1-eval.ini`, extract-from-live-gen) bit-matched training (second-order determinism holds).
Overlay PNGs: `specs/038-accurate-m2/autoc-038-t2-basic-m1-diag_*.png` (`--compare 037-t5-20hz`).

### Pre-t3 de-risk — new inputs are real AND used, wind populated (2026-07-02)

Before committing the ~2-day t3 bake, verified from the existing basic-m1 (t2) dmps:

- **T008 wind getter ✓** — `wN/wE/wD` non-zero across all 6690 ticks of the latest gen (wN −4.0→0.1,
  wE −3.1→4.8; wD ~0 except 156 ticks — physically correct for steady horizontal wind + occasional
  thermal). `getLastLocalAirmass()` populates. [OP] flag on T008 resolved.
- **New-input VALUES real (not stubbed)** — `dBnd` (dist_to_boundary tanh) ranges 0.17→1.0 mean 0.98
  (drops toward the wall, saturates far away); `inward_body` is a **unit vector at every tick**
  (norm 1.000 min/max), components spread [−1,1]. Gather code correct end-to-end in a live run.
- **NN ATTENDS to the new inputs (not ignored)** — first-layer per-input weight L2 norm on the gen-400
  elite (`nnextractor`→`nn2cpp`, fitness −997): legacy-33 median 7.77 (4.9–10.4); new-4 = dBnd 6.99 (27th
  pct), inX 8.62 (76th), inY 8.53 (73rd), inZ 7.99 (55th). New-input mean norm 8.03 vs legacy 7.86
  (**ratio 1.02** — parity), all ~8× above Xavier init. The inward direction cosines land upper-half;
  dist_to_boundary lower-weighted (sensible for M1 rabbit-chase). **Enrichment is integrated, not dead
  weight** — the early-training tax is the cost of the bigger space, not disuse.

### Full-M1 enriched vs 037 — t1 was killed pre-takeoff (2026-07-02)

Compared the stopped full-M1 enriched rebake **t1** (`autoc-038-t1-m1-rebake`, pop 5000, 20 Hz,
aeroStandard 6×49=294, servo on, 37in/2051w, stopped gen 175) against the exact-shape 037 pair
(only 33in/1923w): **t10** (`autoc-037-t10-m1-20hz-08swin`, the pinned best, full 800 gens) and **t6**
(`autoc-037-t6-m1-20hz`, a same-config run that stalled). Overlay PNGs:
`autoc-038-t1-m1-rebake_*.png` (`--compare 037-t10-best --compare 037-t6-stall`).

| gen | t1 (37in) | t10 (33in **winner**) | t6 (33in staller) |
|----:|:---------:|:---------------------:|:-----------------:|
| 50  | 4.0 / 3.1 | 2.2 / 1.0 | — |
| 100 | 2.3 / 0.7 | 4.5 / 1.6 | 2.1 / 1.4 |
| 175 | **4.8 / 1.8** | **8.3 / 3.2** | **3.0 / 1.0** |
| 250 | — | 14.6 / 7.3 | — |
| 300 | — | 24.1 / 12.8 | — |
| 400 | — | 58.2 / 28.0 | — |
| 800 | — | **74.9 / 38.6** | — |
(cells = avgMaxStreak / pctInStreak; higher = better tracking)

**Findings**:
1. **t1 was killed deep in the pre-takeoff zone.** Even the *winning* un-enriched t10 was only at
   avgMaxStreak 8.3 @ gen 175 — its takeoff is a **gen 250→400 event** (8.3→24→58). Judging any M1 at
   gen 175 is judging before the inflection (matches [project_no_future_curve_shape] slow-start shape).
2. **t1 is mid-pack, not doomed.** @ gen 175 t1 (4.8) sits between staller t6 (3.0) and winner t10 (8.3);
   @ gen 50 t1 (4.0) was *ahead* of t10 (2.2). Enrichment tax is real but modest at full-M1 scale.
3. **Basin lottery is independent of enrichment.** t6 vs t10 = same 33in/pop-5000 config, different seed →
   one stalled, one took off ([project_m1_basin_lottery_actual_rate]). Two separate levers: **more gens**
   buys past the enrichment tax (takeoff is late anyway); **larger pop** buys down the basin lottery (more
   basins sampled) at the cost of slower per-gen crossover + wall-clock.

**Decision (2026-07-02)**: re-run the enriched full-M1 to the **full 800 gens at pop 5000 first** (t3) —
cheapest test of "does it just need to reach the takeoff window." Bump pop only if t3 stalls *through*
gen ~300 (basin miss). Determinism already confirmed (basic-m1 eval bit-match), so t3 is a budget test,
not a correctness one.

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
