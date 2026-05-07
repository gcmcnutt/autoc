# AutoC Backlog

**Last Updated**: 2026-05-04

---

## 030 spin-offs (deferred from 030 spec at plan-research scoping)

Items extracted from the [030 tracker-mode spec](030-tracker-mode/spec.md) on 2026-05-04, before plan-research begins. These were architecturally part of the 030 epic but earned defer-status under the smoke-test-first scoping (D13 / D16). Some are 031 (sibling-feature) candidates, some are pure backlog. Tagging as such; final 030/031 split is plan-research's call.

### [031 CANDIDATE] Parallel perception-front-end — camera pixels → (x, y, CEP)

- **Trigger**: real-flight beacon hardware build / virtual-beacon flight test (staged-path row 5+).
- **Scope**: the FPGA / DSP pipeline that bridges raw camera pixels to the `(x, y, CEP)` triple 030 consumes — thresholded centroid extraction per color channel, cluster-spread (CEP) computation, threshold-fail sentinel emission. **Includes**: prop-arc occlusion modeling (rolling-shutter resonance with prop RPM, banding artifacts at certain RPMs, intermittent global-shutter occlusion at non-resonant rates); airframe self-occlusion mesh refinement beyond 030 v1's coarse body proxy; LED wavelength + emission-cone tolerance characterization.
- **Why parallel rather than 030-internal**: shares only the `(x, y, CEP)` interface contract with 030; the engineering surface (FPGA design, threshold tuning, color-channel separation, real-camera characterization) is image-domain work that runs alongside 030's controller-side training without touching it. The two features inform each other through the contract: 030's per-scenario PRNG variation experiments tell perception-front-end what camera-spec tolerances need to be met; perception-front-end tells 030 what int8 noise floor and CEP magnitude distribution to actually quantize with.
- **Source design notes**: 030 spec D7 (DMP versioning + parallel feature), D10 (camera v1 baseline + prop-occlusion deferral).
- **Files likely**: new `src/perception/` module (or top-level `perception/` peer to `crrcsim/`), new spec dir `specs/031-perception-front-end/` when this unparks.

### [031 CANDIDATE] Variable-rate / real-flight source robustness

- **Trigger**: real-flight-recorded trajectories become available (post-virtual-beacon flight test).
- **Scope**: exercise the FR-018 timing model under realistic xiao+INAV telemetry jitter and dropped samples. Confirm determinism end-to-end at non-real-time sim speeds (contract test). 030 v1 timing model is *built to handle* variable-rate sources but is *exercised* only at uniform-rate pathgen-derived sources.
- **Source design notes**: 030 D14 (timing-model exploration row).

### [031 CANDIDATE] Library curation + turn-direction mirror-pairing

- **Trigger**: when 030 smoke-test green and operator wants real-target-class generalization, OR when the controller fails an OOD turn-direction test in early eval.
- **Scope**: library composition tooling — mirror-pair every recorded trajectory into a left-turn / right-turn matched set during ingest (flip Y in NED + flip roll quat + flip rcData[0]); cross-source-run mixing (sim + real, geometry-filtered); auto-bootstrapping (winners-of-tracker-runs feed back into the library). All currently in 030 spec's Out of Scope or C1 / C5 open considerations; collecting them here.
- **Source design notes**: 030 spec C1 (turn-direction symmetry trap), C5 (pathgen vs library decision matrix).
- **Why deferred from 030**: not load-bearing for the smoke test; matters once the operator wants generalization beyond what a single source dmp delivers.

### [031 CANDIDATE] Renderer "exotic goodies" — reverse-projection + dphi-overlay + crash-strike viz

- **Trigger**: post-smoke-test, when operator finds analytics-experimentation needs them to debug specific failures.
- **Scope** (from 030 D15):
  - **Reverse-projection overlay** — given chase pose + recorded perception output `(x, y, CEP)`, compute via reverse camera model (inverting aberrations + int8 quantization, propagating CEP into a 3D uncertainty cone) where the controller "thinks" the target is. Compare to where M1 target actually is (also in M2 dmp per FR-015). Visualizes perception error in 3D — directly diagnostic for "lost fitness because of perception, or because of control?"
  - **Old-style dphi/dtheta overlay** — from chase pose + M1 target's true position, compute what pathgen-mode's old `(dphi, dtheta, dist)` projection would have shown, render alongside the new beacon view. Comparison tool for debugging whether the new representation delivers equivalent or richer information than the old.
  - **Crash-hull strike visualization** — mark hull entries (and `p_crash` fires) in 3rd-person view; aids hull-curriculum tuning.
- **What stays in 030 v1 from D15**: error bars on the camera-POV display (CEP as visible ellipse spread) — cheap, directly load-bearing for smoke-test signal-or-not assessment.
- **Why deferred**: research-grade analytics; not required for smoke-test 4th deliverable.

### [030 v1+] Live two-aircraft display in crrcsim (RobotProgrammable + mod_robots)

- **Trigger**: when operator wants to *watch training in progress* live in crrcsim's 3D viewer (vs after-the-fact in renderer playback), OR when the 031-candidate parallel perception-front-end needs a real visual rendering of the target craft to feed image-domain experiments.
- **Scope**: new `crrcsim/src/mod_robots/robot_programmable.{h,cc}` — `RobotBase` subclass consuming an in-memory pose stream pushed from autoc; `Robots::AddRobot` integration so autoc-side registers the programmable target per scenario; per-scenario teardown / reset. ~150 LOC sketch per [reference_crrcsim_mod_robots.md](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md).
- **Why deferred from 030 v1**: the smoke test (D13) computes virtual beacons strictly as math from `(chase pose, target pose in autoc memory)` — no live in-crrcsim two-aircraft display needed. Renderer 3rd-person view (FR-012) draws both aircraft from the M2 dmp's copied `targetTrajectoryList` (FR-015 self-containedness) using its own VTK actors. Adding the multi-aircraft crrcsim substrate to v1 would expand scope without serving the smoke test.
- **Source design notes**: 030 spec FR-002 (v1 deferral note); 030 plan M4 (deferred milestone description); research.md R1 (decision rationale).

### [BACKLOG] Multi-camera variant experiments

- 030 v1 ships single-camera. Multi-camera *interface* is in v1 (per FR-003b — independent per-camera parameter blocks, no assumption of matching configs). *Exercising* the interface is backlog: asymmetric wide+narrow, stereoscopic, wide+telephoto with parallax offset, forward+downward-tilted secondary. Trigger: when 030 smoke-test green and operator wants to use the sim as a camera-spec sandbox (US3-style sweeps) for hardware-design decisions.

### [BACKLOG] Minisim retire / stub / remove

- Project-level decision surfaced by 030 scoping, captured in 030 D12. Three options: ignore (default), stub, remove. Decision happens at the moment a 030 schema change first forces a touch on `tools/minisim.cc`. Default-to-ignore until then; if 030 plan-phase work hits minisim, upgrade to remove rather than spend porting cost.
- Files: `tools/minisim.cc`, `CMakeLists.txt:100-106`, audit `tests/` for hard dependencies.
- **Update 2026-05-06**: trigger fired during 030 M6a (PathgenStepper extraction) + M6c/d/e (tracker-mode dispatch wired in minisim per session 2026-05-06 routing decision). Per operator call, minisim stays alive through v1 tracker mode rather than retiring. Retirement decision now contingent on 030 smoke outcome.

### [BACKLOG] AutocConfig auto-print / extensible parameter dump

- **Surfaced 030 M6e (2026-05-06)**: `src/autoc.cc` startup logging is hand-coded `*logger.info() << "Key: " << cfg.field << endl` for every AutocConfig field. Adding the 030 tracker-mode block (~30 new fields across Source / Trail / CrashHull / Arena / Camera / Beacon) made the fragility obvious — every new knob requires both an AutocConfig field add, a parser line in `src/util/config.cc`, AND a manual print line in `autoc.cc`'s startup dump. Three-place edit per knob.
- **Why it matters**: future feature work (M7 tracker fitness, 031+ camera-config experimentation, etc.) will keep adding knobs. Drift between "config printed" and "config used" is silent — operator looks at the log, doesn't see the new field, assumes default; meanwhile the new knob is active in the run.
- **Options**:
  - **X-macro list** (`#define AUTOC_CONFIG_FIELDS(X) X(int, populationSize, 500) X(int, numberOfGenerations, 50) ...`): single source of truth, generates field decl + parse + print. Familiar pattern (already in tree for some structures?) but adds preprocessor density.
  - **Cereal-to-text adapter**: AutocConfig already could carry a `serialize` template; adding a JSON-archive output pass produces a structured dump for free. Cleaner from a typesystem perspective but requires cereal/json or hand-rolling.
  - **Reflection-via-tuple-of-members**: C++17 alternative — compile-time list of `std::tuple<std::string_view, MemberPtr>` pairs that print(field) + parse(field) iterate over. Modern but more complex to set up.
- **Trigger to act**: when the next milestone adds 5+ knobs (likely M7 tracker fitness), before the manual-print drift bites.
- Files: `include/autoc/util/config.h`, `src/util/config.cc`, `src/autoc.cc:1402-1462` (startup print block).

---

## 027 carry-forward → 028 deeper-rnn

[028 spec](028-deeper-rnn/spec.md) inherits the architectural and
incentive bets that 027 plumbed but did not validate.
[027 findings.md](027-recurrent-nn/findings.md) is the consolidated
record of what was built, what happened in rnn1/2/3, and the four
not-yet-disambiguated failure modes — required reading before
working any of the items below.

Items: either "in tree, restored later" (CADENCE7-REDUX markers —
flip to re-enable) or "investigate before retraining":

### [NEXT — 028] D-simple recurrent NN

- 1923-weight recurrent topology (16-wide layer 2 W_hh) plumbed in
  `include/autoc/nn/topology.h`, `src/nn/evaluator.cc`,
  `src/nn/population.cc`, `src/nn/serialization.cc`, `tools/nn2cpp.cc`,
  `tools/minisim.cc`, crrcsim `inputdev_autoc.{h,cpp}`.
- Currently disabled: `NN_RECURRENT[]` all-false in topology.h. Flip
  layer 2 to `true` and update `static_assert` from 1667 → 1923.
- Three rnn experiments (rnn1, rnn2, rnn3) failed to descend below
  cadence7 plateau — failure mode not yet diagnosed.

### [NEXT — 028] C2 stability lexicase axis (027 v4)

- `Σ_t (|out_pt|-1)+(|out_rl|-1)` per scenario, on `ScenarioScore`.
- Plumbed in `src/eval/fitness_decomposition.cc` and
  `src/eval/selection.cc` (commented out behind CADENCE7-REDUX).
- Restore: uncomment the `pool.push_back({s,
  &ScenarioScore::stability_score, 0.5})` in selection.cc.

### [NEXT — 028] C2 energy lexicase axis (027 v3)

- `Σ_t (out_th-1)/2` per scenario, on `ScenarioScore`. Same plumbing
  pattern as stability above; same restore path.

### [NEXT — 028] rnn1/2/3 failure-mode disambiguation

- Required prework before any 028 retraining. Four candidate
  failure modes documented in
  [`027/findings.md`](027-recurrent-nn/findings.md) §"Plausible
  failure modes" — covered by one cheap experiment.
- Cleanest first experiment: **D-alone diagnostic** (D-simple ON,
  C2 axes OFF). Outcomes drive different 028 directions per the
  table in [`028/spec.md`](028-deeper-rnn/spec.md).

### [DEFERRED — post-028] Re-enable Selection027 multi-objective tests

- 4 tests renamed `DISABLED_` in `tests/selection_tests.cc:152+`
  for the cadence7-redux build. Restore once C2 axes are
  uncommented in selection.cc.

### [DEFERRED — post-028] Xiao-side recurrent forward pass

- `nn2cpp` already emits the C code (W_hh mat-vec, hidden-state
  array, `nn_reset()`), but `xiao/src/generated/nn_program_generated.cpp`
  is not regenerated/built/flashed.
- Triggered by 028's sim gate clearing — same discipline as 027.

### [DEFERRED — post-028] Renderer scrubbing with hidden state

- 027 plan open decision #5: when user scrubs the timeline
  backwards, does the recurrent hidden state get recomputed from
  span start, or persisted forward-only? Currently no
  reconstruction; documented limitation.

---

## Legend

- `[NEXT]` - High priority, ready to start
- `[DEFERRED]` - Lower priority, will revisit
- `[DONE]` - Completed in 015 or prior

---

## NN Training Improvements (015) — active

Tracked in [specs/015-nn-training-improvements/tasks.md](015-nn-training-improvements/tasks.md).

Current milestone: robust repeatable training → flight test.

BIG-aero1 complete: 266 gens, fitness 2,955, 294/294 OK, zero crashes.

Generalization eval complete (T180–T184): 86–98% completion across novel paths,
random geometries, and 120% envelope stress. Controller has significant headroom.

Remaining 015 work:
- Phase 7: Xiao-GP sensor sync (blocker for flight test) ← NEXT
- Phase 6: Aircraft parameter variation (sim-to-real)
- Phase 8: Polish (data.stc, arena layout, legacy tearout, memory leak check)

---

## Future Features (separate from 015)

### [DEFERRED] Selection Strategy Alternatives
- NSGA-II Pareto: non-dominated sort on (tracking RMSE, energy, worst-case spread)
- Rank-based fitness shaping: CMA-ES style rank-derived weights
- sep-CMA-ES optimizer: pop 5000→50, per-weight step size adaptation
- Only pursue if/when epsilon-lexicase plateaus

### [DEFERRED] Path-Relative Smoothness
- Normalize Δu by path curvature: penalize excess control, not turns
- On hold — slew limiting already killed bang-bang, lexicase hasn't plateaued

### [ABANDONED] INAV pt3 RC Smoothing Filter in CRRCSim (023 Phase 9a experiment)
- **Attempted 2026-04-13**: replicated INAV's `rc_smoothing.c` pt3 filter (3rd-order
  cascade Butterworth LPF) in CRRCSim `inputdev_autoc.cpp`. Filter ran at 333Hz
  FDM rate, smoothing 10Hz NN command steps before FDM surface application.
- **test5 (10kHz passthrough)**: confirmed filter code path is deterministic and
  doesn't degrade convergence when effectively disabled. Matched test4 baseline.
- **test6 (40Hz cutoff)**: training stunted — best stuck at -2225 through 55 gens
  vs test4's -4410 at the same point. Avg fitness ~40% lower, pctInStreak 3% vs 12%.
  The filter changes dynamics enough that the GA can't find productive policies.
- **20Hz also tried**: even worse stunting (not logged as formal test).
- **Root cause**: the filter mechanically prevents the NN from making the quick
  corrections it needs to avoid crashes and maintain streaks. The NN must learn
  that smooth commands are better through its own fitness signal, not be
  mechanically constrained.
- **Conclusion**: pt3 filter approach abandoned for training. If INAV's
  `rc_filter_lpf_hz` is enabled for real flight, the sim-to-real gap from
  not modeling it is acceptable — the NN learns direct control and the
  physical filter just smooths the edges. A fitness-based smoothness
  incentive (lexicase or per-step penalty) is the better path.
- **Code fully backed out** — no pt3 code remains in CRRCSim or AircraftState.

### [DEFERRED] Total Energy Management + Altitude-Aware Distance
- Current distance metric is flat Euclidean — treats "5m above" same as "5m below"
- In reality above is always safer (altitude = energy reserve), below-and-inside is worst
- Observed in T184: NN flies consistently low and outside at slow rabbit speeds ("race horse"
  effect — trained at 16±4 m/s, throttle oscillates at 12 m/s)
- Proposals:
  - Total energy (altitude + airspeed) as NN input or lexicase objective
  - Altitude-aware distance: asymmetric penalty (below penalized more than above)
  - Wider rabbit speed range in training (include 8–12 m/s slow regime)
- Also enables future tactics layer: arena boundary awareness, altitude floor guard
- Post-flight-test refinement — current flat Euclidean tracking is adequate for first flight

### [DEFERRED] Simulator Sampling Time Variation
- Training uses exact 100ms steps; real hardware has jitter (~100ms ± 10ms)
- Add configurable random dither to sim tick interval during training
- Makes NN robust to real-world MSP bus contention and sensor read latency
- Sim-to-real hardening item — after initial flight test data validates baseline

### [NEXT] GPU-Native Evaluation — required for 017
- Accelerate fitness evaluation on GPU (5000 sims/sec vs ~200)
- BIG-3 training was ~352M sim evaluations (pop=3000 × 294 scenarios × 400 gens)
- 017 (vision NN) at ~3K weights needs at minimum the same scale, likely 2-5×
- Beacon projection adds ~46B projection calls at 400 gens — CPU-only is ~77 min/gen
- Current crrcsim is single-threaded C++ with OpenGL dependency — not GPU-parallelizable
- Options: (a) GPU physics sim (CUDA/Vulkan compute), (b) lightweight FDM on GPU with
  beacon projection fused, (c) hybrid: crrcsim for physics, GPU batch for projection
- DGX Spark (GB10) may be sufficient for Option (c); Options (a/b) may need larger GPU
- **Blocking dependency for 017-phase3 at training scale**
- See: [017 spec](017-visual-target-tracking/spec.md)

### [DONE] Renderer: Path reveal timing with variable rabbit speed
- Fixed: renderer now uses rabbit odometer from AircraftState to reveal path
  segments by distance traveled, not time fraction. Stops at crash point.
- Was T516 from 020, closed 2026-03-31.

---

## Infrastructure

### [NEXT — post more-rnn3 completion] Genome ablation tool — generic weight/input editor + eval harness
- **Trigger**: when more-rnn3 finishes 800-gen run. Run alongside the usual subjective robustness eval tests (the existing post-run set) to add a quantitative ablation dimension.
- **Immediate use case**: answer the question "is the R in RNN actually helping, or is more-rnn3's outperformance just from +256 W_hh weights of capacity?" Take the more-rnn3 winning genome, **zero W_hh weights**, run eval pipeline, compare fitness.
  - If zeroed-W_hh fitness collapses (e.g., back to cadence7-redux's −33000): recurrence is load-bearing.
  - If zeroed-W_hh fitness ≈ original: recurrence is decorative; FF capacity at matched weight count would have done the same. (Then the 2-day matched-FF retrain becomes the next experiment.)
- **Generalization** (the actual ask — design as a reusable diagnostic): a NNGenome editor that takes a *mask spec* and an input genome `.dmp`, produces a modified genome, runs eval, reports fitness diff. Mask spec covers ablation regions like:
  - `--zero-whh` → zero all W_hh weight blocks (drop recurrence)
  - `--zero-input GYRO_P,GYRO_Q,GYRO_R` → zero specific NN input columns at each tick (drop rate gyros — does the controller still track without body-rate sensing?)
  - `--zero-input DPHI_NOW,...` → drop temporal history features
  - `--zero-layer N` → zero entire hidden layer (drop a feedforward stage)
  - Other "drop X and re-evaluate" patterns as they arise in research questions
- **Implementation sketch**:
  - New tool: `tools/nn_ablate.cc` (or extend `nnextractor`/`minisim`).
  - Accepts an input `.dmp` (or `nn_weights.dat`) + mask spec + autoc-eval.ini (or autoc.ini).
  - Loads NNGenome, applies mask (zeros specified weights/blocks OR rewrites `nn_gather_inputs` output to mask specific input fields).
  - Runs same eval pipeline as `runNNEvaluation()` — uses identical scenario set, deterministic seeds.
  - Outputs: original fitness, ablated fitness, Δ fitness, per-axis dCtrl/`<|out|>` Δ, per-scenario fitness Δ histogram.
- **Why this design** (vs one-off ablation scripts): research questions like "drop gyros," "drop recurrence," "drop temporal history" recur. A generic mask-based editor amortizes the eval-pipeline plumbing across all of them. Names like `GYRO_P` align with the [Type-Safe NN Sensor Interface](#next-type-safe-nn-sensor-interface) work — both share the "name input columns by enum" need.
- **Out of scope for v1**: weight perturbation (Gaussian noise), targeted permutation, partial zeroing (e.g., 50% of W_hh). Add only if specific research questions need them.

### [NEXT] Type-Safe NN Sensor Interface
- Currently NN inputs/outputs are opaque `float[]` arrays indexed by magic numbers
- Topology changes (29→27) caused silent serialization corruption — now crashes, but
  still fragile for future sensor additions (gravity vector, camera, etc.)
- Need: a typed sensor struct or enum-indexed map that
  - Names each input (e.g. `GYRO_P`, `QUAT_W`, `DPHI_NOW`)
  - Carries type, units, valid range metadata
  - Auto-generates topology count from struct definition
  - Serializes with field names or tags so format is self-describing
  - Compile-time error if evaluator.cc and topology.h disagree
- Also unify the scattered constants: NN_INPUT_COUNT in topology.h, autoc.h,
  evaluator.cc, tests, data.dat format comments, sim_response.py parser
- This is load-bearing for 021+ as we iterate on sensor inputs frequently
- Files that must change when NN_INPUT_COUNT changes (021 learnings):
  - `include/autoc/nn/topology.h` — count, weight count, topology string
  - `include/autoc/autoc.h` — duplicate defines (DISTANCE_TARGET etc.)
  - `src/nn/evaluator.cc` — nn_gather_inputs(), index mapping, comments
  - `src/autoc.cc` — data.dat format string, header, field indices
  - `tests/contract_evaluator_tests.cc` — topology assertions
  - `tests/nn_evaluator_tests.cc` — input layout assertions
  - `specs/019-improved-crrcsim/sim_response.py` — data.dat parser
  - `xiao/src/msplink.cpp` — xiao-side input gathering
  - `include/autoc/eval/aircraft_state.h` — nnInputs_ array size, serialization
- Consideration: prev commands (pitchCmd/rollCmd/throttleCmd feedback) as optional
  inputs — may want to toggle these on/off during experimentation. Type-safe
  interface should support optional/conditional inputs without recompiling everything

### [NEXT] Eval Fitness Computation — Bugs
- **Bug 1: Different metric** — ✅ FIXED post-022. Both training and eval now use
  `computeScenarioScores()` + `aggregateRawFitness()` (conical surface). Verified in
  fitness_decomposition.cc and autoc.cc.
- **Bug 2: Stale fitness in S3** — Eval uploads original NN weights (with training-time
  fitness baked into NN01 format) to S3 via `evalResults.gp`. Renderer deserializes
  this and shows the ORIGINAL stored fitness, not the eval result. Even with
  radically different eval scenarios, renderer always shows the training fitness.
  - Flow: `nn_weights.dat` (carries fitness from nnextractor) → `nn_deserialize` →
    `genome.fitness=508K` → raw bytes copied to `evalResults.gp` → S3 → renderer
  - The eval-computed fitness is only printed to console/stc, never stored
  - Fix: update `genome.fitness` with eval result before serializing to evalResults,
    OR store eval fitness in a separate evalResults field the renderer can read
- **Bug 3 (NEW 2026-04-07): Eval mode missing rabbit speed config** — `runNNEvaluation()`
  in `src/autoc.cc` (around L750-787) builds `EvalData` without setting `evalData.rabbitSpeedConfig`.
  Default is `{nominal=16.0, sigma=0.0}` from `RabbitSpeedConfig::defaultConfig()` —
  i.e., **constant 16 m/s rabbit**, NOT the configured 13±2 m/s from autoc-eval.ini.
  - Training path (L939-940 and L1028-1029) correctly sets `evalData.rabbitSpeedConfig = gRabbitSpeedConfig`.
  - Symptom: eval fitness on saved gen-N weights is "slightly different" from the training
    fitness reported at gen N — same NN, same scenarios, but different rabbit trajectories
    because rabbit speed profile is wrong.
  - Fix: add `evalData.rabbitSpeedConfig = gRabbitSpeedConfig;` (and `* computeVariationScale()`
    for consistency, though both return 1.0) at line ~756 in eval path.
  - Discovered while trying to reproduce 022 betterz2 gen 400 fitness in eval mode.

### [DONE 2026-04-07] Refactor Duplicate Fitness Constants
- ✅ Resolved by 022 conical-surface refactor. The old DISTANCE_TARGET / ATTITUDE_NORM
  constants no longer exist. fitness_computer.h is the single source of truth for
  the conical scoring surface (FitDistScaleBehind, FitDistScaleAhead, FitConeAngleDeg).

### [DEFERRED] Streak Threshold Ramp (022 T024)
- Originally proposed in 022: ramp `FitStreakThreshold` from min (e.g., 0.1, ~22m
  forgiving) to max (0.5, ~7m demanding) over training via `computeVariationScale()`.
- Goal: early generations get streak credit for "getting closer," late generations
  demand tight tracking.
- **Verdict 2026-04-07**: betterz2 (V4 conical, 400 gens) converged strongly without
  this. Not needed for current curriculum. Park as a potential tool if a future
  curriculum widens or training plateaus on a harder task.
- Implementation (when needed):
  - Add `FitStreakThresholdMin` / `FitStreakThresholdMax` to autoc.ini, config.h, config.cc
  - In `computeScenarioScores()`, interpolate threshold = min + (max-min) * computeVariationScale()
  - FitnessComputer constructor takes the interpolated threshold

### [NEXT] Batch and Cache Deterministic Scenarios
- With 150+ scenarios per individual, serializing full table per eval is expensive
- Send scenario table once at generation start, cache in crrcsim
- Reduces per-eval serialization from O(scenarios × individual) to O(individual)

### [DEFERRED] Output Cleanup
- OutputDir config key, auto-created run subdirectory, clean eval prefix naming

### [DEFERRED] Training Run Archive Policy (moved from 024 T204)
- One-page `docs/TRAINING_RUN_ARCHIVE.md` documenting: naming scheme for
  training runs, retention policy (what to keep, what to discard), and a
  record of the current canonical run (`test4-data.dat`, now cadence7).
- Current ad-hoc: training runs live in `/home/gmcnutt/autoc/data.dat` +
  `logs/autoc-<feature>-<name>.log` + S3 bucket artifacts. No formal
  rotation; big `.dat` files accumulate.
- Write when the accumulated `.dat` size becomes a problem or when a
  new run needs canonical-reference status for post-flight analysis.

### [NEXT] Per-axis / per-path analysis from S3 .dmp instead of data.dat
- **Problem**: `data.dat` is overwritten at the start of every training run.
  The per-axis aggressiveness time-series + per-path roll/pitch rate analysis
  in `specs/029-no-future-arch/plot_per_axis_time_series.py` reads `data.dat`
  directly, so as soon as the next training launches, the prior run's
  analysis is no longer reproducible. Specific instance: pastonly2 finished
  2026-05-02, pastonly3 launched same day → pastonly2's data.dat lost. We
  cannot retroactively re-render pastonly2's per-path roll/pitch RATE chart
  with the new normalization (introduced 2026-05-02 in this same session).
- **Long-term solution**: extend the per-axis analysis tooling to read
  per-tick aircraft state from the S3 `.dmp` files (cereal-serialized
  `EvalResults` containing `aircraftStateList[scenario][tick]`). Each gen's
  best individual is dumped to S3 (per `src/autoc.cc:1119-1146`), so
  per-tick quaternion + outputs are recoverable for any prior run as long
  as S3 retention holds.
- **Implementation sketch**:
  - New tool: `tools/aircraft_state_extractor.cc` (or extend `nnextractor`).
  - CLI: `--source-run <S3-prefix> [--gens 0-800] --out per-tick.csv`
  - Iterates all gen .dmp files for the run, deserializes `EvalResults`,
    flattens `aircraftStateList[scenario][tick]` into a CSV with columns
    matching today's `data.dat` (Scn, Pth/Wnd:Step, qw qx qy qz, outPt
    outRl outTh, etc.).
  - Existing `plot_per_axis_time_series.py` reads either source (data.dat
    OR extracted CSV) — minor parser tweak.
- **Why this enables**: retroactive per-path metric refinement (like the rate
  normalization we just added), cross-run direct comparisons (pastonly2 vs
  pastonly3 vs more-rnn3 on the same metric, computed identically), and
  reproducibility of analysis when source data.dat has been overwritten.
- **Trigger**: next time we want to retroactively analyze a prior run with a
  metric we didn't compute at the time. Likely fires when 030 needs to
  cross-compare pastonly2 / pastonly3 / 025 controllers post-hoc, or when
  a flight-test outcome motivates re-checking some prior controller's
  per-path behavior.
- **Out of scope for v1**: extracting non-best individuals (only the gen's
  elite is dumped to S3); extracting per-tick PidInternals (not in
  EvalResults today — would need another schema add); replicating data.dat
  byte-for-byte (just the columns the per-axis analysis needs).

### [NEXT] Snapshot / resume training mid-run + adaptive gen-budget
- **Problem**: Path A config (pop 5000, gens 800, NNSigmaFloor 0.05) reliably
  hits sigma-floor around gen 500-550 across recent runs (more-rnn3, pastonly2,
  pastonly3). Once at floor, fitness drift over the remaining 250-300 gens is
  small (a few percent) — pure-exploitation refinement that could either be
  (a) skipped (terminating earlier saves ~24h compute per run) or (b) extended
  beyond 800 if the late-plateau is still moving (rare, but happens).
- **Today's blocker**: no checkpoint/resume mechanism. Each training run starts
  from gen 0; can't re-launch from gen 600 to extend, and can't kill at gen
  600 with confidence that "this is already the answer." So operator
  uniformly trains to gen 800 to confirm plateau is real even when the
  middle-third would have been definitive — wasted compute.
- **Long-term solution** (two parts):
  1. **Periodic checkpoint dump** — every N gens (say N=50), serialize the
     full population (NNGenome[]) to disk + log gen state. Same `cereal`
     stack as S3 dumps, just population-wide instead of best-only. ~1-2 GB
     per checkpoint at pop=5000 × 1923 weights — manageable rotation policy.
  2. **`--resume <checkpoint.dat>`** flag on autoc — load population, set
     gen counter, continue. Identical PRNG seeding semantics (one of the
     fields in the checkpoint).
- **Adaptive gen budget** (smaller follow-on): given checkpointing exists,
  add a runtime termination heuristic — e.g., "if last-N-gens fitness slope
  < threshold AND sigma at floor for ≥ M gens, stop." Operator-overridable.
  Saves the wasted late-plateau gens automatically.
- **Trigger**: when a 029-class run produces a clearly-converged controller
  before gen 800 AND the operator wants to run a *follow-up experiment*
  (variation in 025, derived features in 029-T061, etc.) and would benefit
  from "warm-start from the converged checkpoint" rather than re-training
  from gen 0. Concrete instance: pastonly3 may hit clean plateau by gen
  600 of 800, but operator continued because no resume mechanism exists.
- **Out of scope for v1**: distributed checkpointing across worker nodes
  (single-machine training only); checkpoint-format versioning (tracker-
  mode-aware); selective restart (e.g., load weights but reset variation
  ramp).

### [NEXT] Make pathgen.h Portable for Embedded
- Single pathgen.h that works on both desktop and embedded
- Current state: embedded_pathgen_selector.h is a manual clone of desktop pathgen.cc
  with different helpers — changes don't propagate (e.g. FortyFiveDegreeAngledLoop
  still at 0.5 rad step vs desktop 0.05 rad after fix 45df719)
- Immediate fix: update embedded FortyFiveDegreeAngledLoop to 0.05 rad step
- Long-term: refactor so both desktop and embedded use the same path generation code

---

## Embedded / Hardware

### [NEXT] USB Log Download from Xiao
- BLE log download is slow (BLE bandwidth-limited) AND unreliable in the
  field — drops/stalls observed even when no other 2.4 GHz interference is
  present. Pre-flight log retrieval blocks turnaround between flights.
- Need: USB-CDC (or USB Mass Storage if simpler) path to dump xiao flash
  log files. Xiao SAMD/nRF supports USB-CDC out of the box.
- Implementation sketch: enumerate flight log files on flash, expose a
  serial menu over USB (e.g., `LIST`, `DUMP <name>`, `ERASE`) — same
  protocol surface as BLE so the host-side download tool can share most
  code. Should not regress BLE path; both are useful (USB at the bench,
  BLE for opportunistic extracts).
- Hold the BLE-reliability investigation as a separate orthogonal item;
  USB download is the practical workaround.

### [NEXT] Export RC Commands to Xiao Log
- Log RC commands throughout entire flight for full playback visualization
- Location: xiao/src/msplink.cpp

### [ACTIVE → 021] Xiao Onboard IMU as AHRS Cross-Check
- Moved to [specs/021-xiao-ahrs-crosscheck/spec.md](021-xiao-ahrs-crosscheck/spec.md)
- P0 blocker: Mar 27 flight showed uncontrolled rotation — cannot tell if AHRS or gain mismatch
- LSM6DS3 + Madgwick on Xiao, log alongside INAV quat, compare post-flight

### [DEFERRED] GPS Dropout Handling During NN Control
- What happens when GPS drops out during NN-active control? Position freezes, NN sees stale data
- Need: xiao detects stale position (no update for N ms) and either disables autoc or flags it
- Also consider MSP communication loss detection and safe fallback

### [DEFERRED] Xiao Safety Checks Pre-Arm
- Ensure mode flip is safe: RC failsafe, RC disarm, hold/RTH should disarm co-processor

### [PRE-FLIGHT / 023] Failsafe Refinement and Bench Verification
- **Blocker for next flight test after 023 NN training lands.** Not a 023 spec
  deliverable, but must be addressed before flying the new NN policy. Source:
  `docs/failsafe-behavior-audit.md` (2026-04-08 audit, see also
  `docs/inav-signal-path-audit.md`).
- **Current state**: `xiao/inav-hb1.cfg:1419-1432` has `failsafe_procedure = DROP`.
  Acceptable for the current foamboard platform (~100g, minimal ground-impact
  risk) and has been running this way for a while. User's earlier aircraft used
  "launch, fly out, reset home, orbit home at 50m" — that is the aspirational
  target procedure for larger/future platforms.
- **Four specific items from the audit:**
  1. **Failsafe has NEVER been exercised in real flight** on this platform
     (audit confirmed no failsafe events in any 2026-04-07 flight log).
     DROP path has never actually fired during an autoc span. **Action:**
     bench test with physical SBUS disconnect during an active autoc span,
     verify the full chain works: INAV trips → DROP disarms → xiao detects
     FAILSAFE bit via `MSP2_INAV_LOCAL_STATE` → xiao calls `stopAutoc()` →
     recovery path (disarm/rearm OR stick-wiggle above `failsafe_stick_threshold = 50`).
  2. **SBUS receiver failsafe value behavior is unverified**. If the receiver
     is configured to "hold last values" on signal loss AND AUX1 was HIGH
     (armed) at the moment of loss, AUX1 stays HIGH after loss, keeping
     `BOXARM` active. The only thing that actually disarms is the main
     failsafe state machine calling `disarm(DISARM_FAILSAFE)` at
     `flight/failsafe.c:587`. **Action:** verify the receiver's failsafe
     channel config on the bench. Document the observed AUX1 behavior
     during signal loss.
  3. **Brittle recovery mechanics**. Depending on INAV settings, recovery
     from failsafe requires either (a) disarm + rearm (possibly in-flight
     if `nav_disarm_on_landing` etc. is off), or (b) sticks above
     `failsafe_stick_threshold = 50` clearing failsafe without disarm.
     Exact current behavior is undocumented for the hb1 platform. **Action:**
     document the recovery behavior, test both paths on the bench.
  4. **Aspirational upgrade path** — for larger/future aircraft or more
     aggressive flight envelopes, switch `failsafe_procedure` from DROP to
     LAND or RTH. Would require: `failsafe_fw_roll_angle` / `pitch_angle` /
     `yaw_rate` tuning for glide, `failsafe_min_distance` semantics, and
     re-auditing the MSP override + failsafe state machine interaction per
     C4 in the audit doc. Not for 023.
- **Why not in 023**: failsafe mechanism is orthogonal to NN representation +
  training work. But the NN policy being harder to pilot-debug raises the
  stakes on reliable control handoff, so failsafe refinement is pre-flight
  prerequisite work, not "nice to have."

### [NEXT / 023 follow-up] INAV Fork Patch: mspOverrideInit First-Frame Bug (C1)
- **Source**: `docs/failsafe-behavior-audit.md` §Latent Bug Assessment → C1.
- **Bug**: Even with `failsafe_recovery_delay = 0`, MSPRCOVERRIDE engage still
  pays a 200 ms floor because `mspOverrideCalculateChannels()` runs at 50 Hz
  from boot and pre-connection ticks update `validRxDataFailedAt = millis()`
  every tick. The first valid MSP frame sees a tiny
  `(validRxDataReceivedAt - validRxDataFailedAt)` difference and must wait
  the full `rxDataRecoveryPeriod` before `rxFailsafe` clears.
- **Fix**: in `src/main/rx/msp_override.c:mspOverrideInit()`, initialize
  `validRxDataReceivedAt = millis() + rxDataRecoveryPeriod`. OR: in
  `mspOverrideCalculateChannels()`, on the first
  `rxSignalReceived = true` transition, unconditionally set
  `rxFailsafe = false`. Either approach makes MSPRCOVERRIDE engage instant
  once xiao starts streaming frames.
- **Why not in 023**: INAV source change is out of scope for a feature
  focused on autoc NN representation + training. The sim already models
  the 750 ms delay via `EngageDelayMs` in Change 1b, and real flights
  work around it (pilot aligns, releases, throws the switch, aircraft
  coasts briefly on momentum). This is a correctness cleanup for a
  future release, not a 023 blocker.
- **Risk assessment**: low. The fix is local to `msp_override.c`, only
  affects the initial boot state, does not touch the main failsafe state
  machine, and does not change behavior for any scenario other than
  "first MSPRCOVERRIDE engage after boot". Bench verification is
  straightforward (measure engage delay before/after).

### [DEFERRED] Xiao-Side Independent RC Dropout Detection
- **Source**: `docs/failsafe-behavior-audit.md`.
- Currently xiao only detects failsafe via the `MSP_MODE_FAILSAFE` bit in
  `MSP2_INAV_LOCAL_STATE`. This means xiao's `stopAutoc("failsafe")` only
  fires AFTER INAV's main failsafe has already tripped. xiao has no
  independent way to detect RC dropout before INAV acknowledges it.
- Defense in depth: xiao could track MSP round-trip latency and pause
  autoc if a threshold is exceeded, OR directly monitor SBUS health via
  a separate serial channel.
- Not urgent for 023 — the current coupling works well enough for DROP.

### [DEFERRED] Speed Up Logfile Download
- BLE download may be over-bucketed from prior troubleshooting

---

## Visualization

### [NEXT] Renderer Playback Enhancements — per-tick scrub + streak/multiplier overlay
- **Per-tick scrub controls**: pause / step-forward-one-tick / step-backward-one-tick.
  Today the renderer plays a continuous timeline; for diagnostic work you want to
  freeze on a specific tick and walk one step at a time to see exactly when the NN
  does what.
- **In-streak counter overlay**: per-tick `streakCount` displayed alongside
  the rendered aircraft, so you can see "is this a streak tick? for how long?"
- **Points multiplier overlay**: per-tick `multiplier` (= 1 + (streakMultMax-1) ·
  streakCount/streakStepsToMax), the same value `FitnessComputer::applyStreak`
  uses to amplify stepPoints during fitness aggregation.
- **Data path**: per-tick streak/multiplier are NOT in `EvalResults` today
  (only scenario-aggregates `maxStreak` / `totalStreakSteps` / `maxMultiplier`
  on `ScenarioScore`). Two implementation options:
  - **Re-synthesize in renderer/script**: replay `FitnessComputer::applyStreak`
    against captured `aircraftStateList` + `pathList` + autoc.ini config knobs
    (`fitStreakThreshold`, `fitStreakRampSec`, `fitStreakMultiplierMax`).
    No schema change. Drift risk: renderer math must match training math.
  - **Capture per-tick on AircraftState**: add 2 fields (streakCount int +
    multiplier float) per tick. Ground truth, no drift, ~negligible dump size
    increase. Cereal schema bump (per project policy: no versioning shim, old
    dumps unloadable post-bump).
- Recommend the schema-bump path for permanent renderer feature; recompute
  is fine for one-off Python diagnostics in the meantime.
- **Files likely involved**: `tools/renderer.cc` (UI controls + overlay),
  `include/autoc/eval/aircraft_state.h` + cereal serialization (if capturing),
  `src/eval/fitness_computer.cc` (export the per-step multiplier alongside the
  streak update — it's already computed, just discarded).

### [DEFERRED] Blackbox Rendering Improvements
- Select path + blackbox log for comparisons, FPV mode

### [DEFERRED] CRRCSim Display Dependency
- CRRCSim requires valid DISPLAY even in headless mode

### [DEFERRED] Clean CRRCSim Shutdown
- Polling loop for keepalive; clean exit when autoc exits

---

## Code Cleanup

### [NEXT] crrcsim mod_inputdev — link autoc_common instead of cherry-picking sources
- **Current**: [crrcsim/src/mod_inputdev/CMakeLists.txt:21-23](../crrcsim/src/mod_inputdev/CMakeLists.txt#L21-L23) compiles three autoc-side files directly into `mod_inputdev.a`:
  ```
  ${CMAKE_SOURCE_DIR}/src/nn/evaluator.cc
  ${CMAKE_SOURCE_DIR}/src/nn/serialization.cc
  ${CMAKE_SOURCE_DIR}/src/eval/sensor_math.cc
  ```
- **Problem**: any new file in `src/nn/` or `src/eval/` that an above .cc references at link time silently breaks the crrcsim build at link, with `undefined reference` errors. The 028 telemetry.cc landing tripped this — `evaluator.cc` called `RecurrentTelemetry::activation_ratio()` which lived in `telemetry.cc`, and crrcsim's mod_inputdev didn't pick up telemetry.cc. Worked around by inlining the method in `evaluator.h`, but the architectural fragility remains.
- **Fix**: have `mod_inputdev` link against `autoc_common` (which is built by the parent autoc CMakeLists). crrcsim already builds via `add_subdirectory(crrcsim)` per Constitution Principle IV (Unified Build), so `autoc_common` is in scope. Remove the cherry-pick lines, add `target_link_libraries(mod_inputdev autoc_common)` (or thread it through the crrcsim link chain to wherever the final crrcsim binary links).
- **Risk**: low. autoc_common pulls in cereal/inih/Eigen/etc., all of which crrcsim already depends on transitively (mod_inputdev's evaluator.cc compile already needs them). Possible duplicate-symbol issues if any crrcsim file also defines something autoc_common does — none observed but worth checking on first build.
- **Alternate workaround**: keep the cherry-pick pattern but add a `mod_inputdev` build-time test that asserts no undefined references in the link target. Less clean but lower-risk.
- Triggered by: 028 telemetry signals (Apr 2026). Will re-trigger on the next autoc-side .cc addition that evaluator.cc transitively references at link.

### [DEFERRED] Memory Leak Investigation
- Small memory leak exists in autoc

### [DEFERRED] Cross-Platform Verification
- Train on aarch64, pull repo on x86
- Build and run renderer/nnextractor/eval against aarch64 S3 objects
- Validates cereal binary portability end-to-end

---

## Completed / Superseded

- ~~Sigma Floor~~ — done (015 Phase 1)
- ~~Curriculum Ramp~~ — done (015 Phase 2, wind scenario ramp)
- ~~Fitness Decomposition~~ — done (015 Phase 2, per-scenario scores)
- ~~Pareto Multi-Objective~~ — superseded by epsilon-lexicase (015 Phase 3)
- ~~Demetic Mode Elite~~ — superseded by lexicase selection
- ~~Wind Speed Variation~~ — done (WindScenarios with varied seeds)
- ~~Immelman Path Fix~~ — done (T121a, progressiveDistance split-S fixed)
- ~~Float Precision Non-Determinism~~ — done (integer timestamps in Path)
- ~~GP Eval Node Test Coverage~~ — superseded (GP removed, NN evaluator has tests)
- ~~Fitness Output Formatting~~ — superseded (aggregateRawFitness is canonical)
- ~~Training Record Consistency~~ — done (S3 upload in eval mode, consistent keys)
- ~~Consolidate PRNG~~ — done (rng.h covers all sites)
- ~~Upper-Level Intercept Director~~ — superseded by entry variation training
- ~~Future State Predictor NN~~ — superseded by temporal history + forecast inputs
- ~~Behavioral Cloning Bootstrap~~ — not needed, direct NN training working
