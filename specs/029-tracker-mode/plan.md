# 029 — Implementation Plan (Tracker Mode — beacon-camera target tracking via flight playback)

**Branch**: `029-tracker-mode` | **Date**: 2026-04-29 | **Spec**: [spec.md](./spec.md)
**Input**: [`spec.md`](./spec.md) (6 user stories, 18 functional requirements, 12 success criteria, 6 clarifications resolved 2026-04-29), [`research.md`](./research.md) (plan-phase architectural research)

## Summary

029 introduces a **second autoc training mode** that trains a controller to track another aircraft via a simulated wingtip-beacon camera, replacing pathgen with playback of recorded flights as targets. The control NN's output remains pitch/roll/throttle; its inputs are the analytic projection of two IR-color-distinguished wingtip beacons through a configurable camera model (v1: planar pinhole, 120° FOV, single forward-mounted) — collapsed at the perception interface to `(screen_x, screen_y, visible)` per beacon, mirroring the FPGA centroid extractor that would deploy on real hardware.

The implementation has three concentric scopes:

1. **US1 baseline experiment** — runs *first*, on the *existing 028 codebase*, with a single-file change to `NNInputs` time-sample distribution `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` (past-only, drops the +0.1/+0.5 future-lookahead samples). Validates the foundational assumption that the recurrent NN can train without lookahead before any 029 implementation work begins. Runs concurrently with the 028 flight wait.

2. **029 substrate** (US2 gateway + dependencies) — `dmp-to-playback` converter tool, `RobotProgrammable` subclass in `mod_robots`, `RobotPathProvider` integration into `inputdev_autoc`, type-safe NN sensor interface (rolled in from BACKLOG), tracker-mode autoc.ini config separate from pathgen mode, extended `EvalResults` schema for camera view data.

3. **Camera + perception modeling** (US3 + US4) — analytic beacon projection through configurable camera model (planar pinhole at 120° FOV / 30 Hz frame rate / 0 ms latency v1 baseline), color-filter / channel-response model for L/R beacon disambiguation, US3's camera-config experimentation harness, US4's controller training. Renderer dual-mode (3rd-person + 1st-person camera-POV) lands in US5.

The **US1 → US2 → US3 → US4** sequence is a critical-path chain: US1 gates the architectural assumption, US2 lights up the new training mode, US3 settles camera-design baseline before US4 commits compute to long-run training. US5 (renderer) and US6 (architectural review) are diagnostic / audit work that lands as deliverables alongside US3/US4.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, conversion tools)
**Primary Dependencies**: Eigen (vec3 / quat / matrix-vector math, projection geometry), cereal (NN serialization, EvalResults schema, new tracker-mode dump format), inih (autoc.ini parsing), GoogleTest (unit/contract tests), CRRCSim LaRCSim FDM (sim physics)
**New Dependencies**: none expected (Eigen sufficient for projection math per R3 research finding pending)
**Storage**: file-based — `data.dat` (training output, schema extends per FR-015), evolution log lines (per-gen telemetry), `data.stc`, S3 `.dmp` (cereal-serialized `EvalResults` — schema bump for tracker-mode dumps), playback library files (crrcsim binary playback format for v1, per spec assumption pending R1 research finding)
**Testing**: GoogleTest (C++ unit + contract tests), pytest-style scripts (Python analysis), determinism contract tests for playback substrate
**Target Platform**: Linux x86-64 (training & analysis), VTK-rendered renderer; xiao deferred until 029 sim+flight gate clears (a future feature)
**Project Type**: Research/training feature within an evolved-NN control system. Adds a parallel training mode alongside existing pathgen mode. No user-facing UI; deliverables are training-run telemetry, evolution PNGs, dual-mode renderer (3rd-person + camera-POV), outcome documents.
**Performance Goals**: per-generation eval time should not regress vs more-rnn3 baseline by more than ~10 % from analytic beacon projection overhead at v1's single-camera × 30 Hz × 245 scenarios setting. **R5 research confirms GPU-Native Evaluation is NOT a 029 v1 prerequisite** — projection is 8 calls per NN tick at ~30 FLOPs each, ~5–10 % over 028's per-tick cost. CPU eval is sufficient for full-population training. (Spec §Overview line 73's earlier GPU caveat assumed 017's vision-NN cost model; 029's analytic 3-float interface is fundamentally different.)
**Constraints**:
- **Determinism non-negotiable** — same Seed + same library = bitwise-identical evaluations (per project_variation_design_principles memory)
- **Joint-PRNG variation preserved** — library-entry-index becomes one PRNG axis alongside wind/entry/craft (inherited from source recording)
- **gp_fitness type throughout** — no raw double/float in fitness path
- **No cereal versioning** — schema changes are clean-cut bumps; old tracker-mode dumps unloadable post-bump (acceptable, no precious archive yet)
- **Minimal-camera-calibration** — controller architecture must work without per-unit camera calibration (FR-003f); per-scenario camera-variation training is the future mechanism (deferred)
**Scale/Scope** (research-confirmed): 6 user stories across 3 concentric scopes. 18 functional requirements. Total new C++: **~600–800 LOC** (R4 confirms ~120–180 in mod_robots, R3 confirms ~30–50 LOC projection in pure Eigen, R2 confirms ~150–250 LOC for `dmp_to_playback` converter, plus ~200 LOC `RobotPathProvider` + autoc-tracker.ini wiring). **Type-safe sensor interface refactor: ~270–330 LOC across 12 files** (R7); lands first as Phase 0 PR. Renderer dual-mode: ~500–1000 LOC including VTK camera-POV math.

## Constitution Check

*Reference: [`.specify/memory/constitution.md`](../../.specify/memory/constitution.md) v1.0.0*

| Principle | Status | Notes |
|---|---|---|
| **I. Testing-First** | PASS | Each substrate component (RobotProgrammable, RobotPathProvider, projection module, type-safe sensor interface, dmp-to-playback converter, library schema) gets contract tests before implementation. Determinism tests are load-bearing and explicit (per project invariants). The US1 baseline experiment uses existing test infrastructure unchanged. |
| **II. Build Stability** | PASS | Substrate work is additive — pathgen mode coexists in tree (FR-011). Each phase commits with green `bash scripts/rebuild.sh` + xiao build. The autoc-tracker.ini config selects mode at runtime. Tracker-mode dump schema bump triggers a clean cereal-version bump (per project policy: no shims, fresh `.dmp` files only). |
| **III. No Compatibility Shims** | PASS | Tracker-mode `.dmp` schema is a clean-cut new format; old pathgen-mode dumps continue to work with pathgen-mode tools, but tracker-mode tools won't read pre-029 dumps. No compat layer. Mode-selector in autoc.ini is a config switch, not a runtime polymorphism kludge. |
| **IV. Unified Build** | PASS | New crrcsim sources (`mod_robots/RobotProgrammable.cc`) build via existing `add_subdirectory(crrcsim)`. The new converter tool links autoc_common per the existing pattern (per R2 research finding pending). No duplicate dependency declarations. |

**Gate result**: PASS. No violations. Section §Complexity Tracking intentionally empty.

## Project Structure

### Documentation (this feature)

```text
specs/029-tracker-mode/
├── spec.md                         # 6 user stories + 18 FRs + clarifications
├── plan.md                         # This file
├── research.md                     # Phase 0 (R1-R7) — architectural decisions for plan
├── data-model.md                   # Phase 1 — entities + schema
├── contracts/                      # Phase 1 — interface contracts
│   ├── playback_file_format.md     # Per FR-001: binary playback format spec
│   ├── tracker_dmp_schema.md       # Per FR-015: extended EvalResults schema
│   ├── nn_sensor_interface.md      # Per FR-006: type-safe sensor input contract
│   └── beacon_projection_api.md    # Per FR-005: per-tick projection output
├── quickstart.md                   # Phase 1 — operator walkthrough
├── checklists/
│   └── requirements.md             # Spec quality checklist (from /speckit.specify)
├── pastonly_outcome.md             # US1 result document (created during Phase 1 execution)
└── tasks.md                        # Phase 2 output — created by /speckit.tasks (NOT here)
```

### Source Code (repository root)

```text
include/autoc/nn/
├── nn_inputs.h                     # MODIFY for US1 (time-offset comment), MODIFY for type-safe interface
└── sensor_interface.h              # NEW (US2 substrate) — typed/named sensor input enumeration

src/nn/
└── nn_input_computation.cc         # MODIFY for US1 (time offsets), MODIFY for tracker-mode inputs

include/autoc/eval/
├── beacon_projection.h             # NEW (US3) — analytic projection module API
└── camera_config.h                 # NEW (US3) — camera config struct + compile-time defaults

src/eval/
└── beacon_projection.cc            # NEW (US3) — pinhole projection + channel response + visible-flag

include/autoc/rpc/
└── protocol.h                      # MODIFY (US2) — tracker-mode EvalResults extension

crrcsim/src/mod_robots/
├── robot.h                         # MODIFY (US2 substrate) — Robots accessor for state query
├── robots.cpp                      # MODIFY — same
└── robot_programmable.cpp          # NEW (US2 substrate) — programmatic-trajectory subclass of RobotBase

crrcsim/src/mod_inputdev/inputdev_autoc/
├── inputdev_autoc.cpp              # MODIFY (US2) — RobotPathProvider; tracker-mode select; FR-002 multi-aircraft
└── robot_path_provider.h/.cc       # NEW (US2) — wraps Robot pos/quat as the rabbit input

tools/
├── dmp_to_playback.cc              # NEW (US2) — converter: source-run .dmp → 245 playback files
└── (existing tools unchanged)

src/autoc.cc                        # MODIFY — autoc-tracker.ini mode selector, tracker-mode log fields

specs/029-tracker-mode/
├── plot_tracker_evolution.py       # NEW (US4) — adapted plot for tracker-mode telemetry
└── plot_per_axis_tracker_aggressiveness.py  # NEW (US4) — adapted from per_axis_aggressiveness.py

tools/  (renderer)
├── renderer.cc                     # MODIFY (US5) — dual-mode views + per-tick scrub
└── tracker_view_modes.h/.cc        # NEW (US5) — 3rd-person + 1st-person camera-POV view modes

tests/
├── beacon_projection_tests.cc      # NEW — projection math contract + edge cases
├── nn_sensor_interface_tests.cc    # NEW — type-safe sensor interface contract
├── robot_programmable_tests.cc     # NEW — multi-aircraft + determinism
├── dmp_to_playback_tests.cc        # NEW — converter tool roundtrip
├── tracker_mode_integration_tests.cc  # NEW — end-to-end deterministic eval
└── (existing tests unchanged but the extended NNInputs may break some — migration in Phase 2)
```

**Structure Decision**: Single-project, native to existing autoc tree. Tracker mode runs as a config-selected mode of the existing autoc binary; no new top-level executable. Per [feedback memory: feature one-offs in specs/<feature>/](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_scripts_dir_scope.md), tracker-mode-specific plotting scripts live in `specs/029-tracker-mode/`. The crrcsim mod_robots integration is the largest contiguous code addition (~150 LOC per R4 research pending), with the projection module (~200 LOC est) close behind.

## Ordering principle

**Validate the foundational assumption first; then build the substrate; then build the camera + training.**

US1 runs first because it can fail cheaply and reframe everything downstream. US2 (substrate) lights up the new training mode but doesn't yet let the controller see beacons — useful as a working-but-empty checkpoint. US3 (camera config experimentation) and US4 (controller training) gate on US2; US3 lands a baseline camera config before US4 commits to a long-run training cycle. US5 (renderer) and US6 (architectural review) are diagnostic / audit; land alongside US3/US4 rather than blocking them.

## Phase 1 — US1: Past-only baseline experiment (FIRST, runs in parallel with 028 flight wait)

**Goal**: validate that the recurrent NN trains effectively without future-lookahead inputs. Single-file experimental change on the existing 028 codebase. No 029 implementation begins until this passes.

### 1.1 Modify `nn_inputs.h` time-sample comment

[`include/autoc/nn/nn_inputs.h`](../../include/autoc/nn/nn_inputs.h): update the `// Time samples: [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]` comment to `// Time samples: [-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]` (per spec US1 distribution choice).

### 1.2 Modify `nn_input_computation.cc` time offsets

Find the array of time offsets used for the 6 history-and-future samples. Replace with `{-0.5, -0.4, -0.3, -0.2, -0.1, 0.0}` (5 past + now, no future). The corresponding rabbit-position lookup goes from "advance odometer ahead by +0.1s / +0.5s" (which currently exists for the future slots) to "look up odometer at -0.5s / -0.4s / etc." (which is straightforward since the path is fully known and the rabbit's position-vs-odometer history is implicit in the path geometry).

For the past samples, the existing `target_x[6]` etc. fields already store position-direction at past odometer values; the existing code can be reused with new offset values.

### 1.3 Verify build + tests green

`bash scripts/rebuild.sh` and `ctest --output-on-failure`. The 33-input layout doesn't change (`NNInputs` struct field order preserved); only the time semantics differ. Existing topology tests pass unchanged.

### 1.4 Launch the past-only training run

```bash
nohup ./build/autoc -c autoc.ini > logs/autoc-029-pastonly.log 2>&1 &
```

Same Path A `autoc.ini` config as more-rnn3 (pop 5000 × 600 gens, all other knobs identical). Run name convention: `more-rnn4-pastonly`.

### 1.5 Monitor + analyze

Same monitoring cadence as more-rnn3 — 6-panel `plot_evolution_progress.py` every 50 gens, per-axis aggressiveness PNG every 100 gens. The plot script needs no changes; it consumes `data.stc` which has the same `#NNGen` log fields (the 028 telemetry fields land naturally — `whh_xh_ratio`, `w_*_cv` all still apply).

### 1.6 Decide: pass / fail

Pass criterion (per US1 acceptance scenarios):
- Late-plateau fitness within ±10 % of more-rnn3 (re-evaluated on fixed-difficulty eval per [project_late_run_fitness_interpretation.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md))
- Per-axis aggressiveness: same architecture-consistency pattern (roll dominates per [project_bangbang_axis_migration.md](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md))
- pctInStreak emergence on schedule

Outcome doc: `specs/029-tracker-mode/pastonly_outcome.md` with fitness + per-axis comparison + go/no-go.

**If PASS**: 029 implementation proceeds as planned. Record in outcome doc and move to Phase 2.
**If FAIL** (>10 % regression): pause 029 implementation. Investigate which input class (recent-past selectivity? long-history at -0.9s? something else?) was load-bearing. Try the documented alternative distributions (`[-0.9, -0.5, -0.3, -0.1, -0.05, now]` etc.) before declaring "no-lookahead doesn't work." A real fail here would force architectural rethinking before the rest of 029.

**Branching note**: this experiment runs ON THE 029-tracker-mode BRANCH but uses only the 028 codebase. The single-file change is committed to 029-tracker-mode as a pre-substrate-work artifact. If outcome is fail, that commit might be reverted or rebased, but the branch state holds.

**Critical sequencing**: US1 runs on **pristine 028** — the type-safe sensor interface refactor (Phase 2.1) does NOT land before US1. US1's whole point is fail-fast architectural validation; adding a behavior-preserving-but-still-meaningful refactor in the path muddies that signal. If US1 fails, you want to know it's the time-offset change, not a refactor regression. The sensor refactor is *parallel work* — it can develop and merge during the ~24-48 hour US1 training run, landing on the branch *after* US1's training completes but ready for Phase 2.2+ to consume. (The refactor's only consumer is tracker-mode-specific work, which doesn't begin until after US1 passes anyway.)

## Phase 2 — US2 substrate: type-safe sensor interface + crrcsim multi-aircraft + dmp converter

**Goal**: light up tracker mode end-to-end with placeholder beacon projection (constant zeros). The training launches against a recorded library, the second aircraft replays trajectories, but the NN sees no useful beacon data yet — that's Phase 3. This phase is the *plumbing*.

**Sub-phases**:

### 2.1 Type-safe sensor interface refactor

Per FR-006. R7 research informs cost / sequencing. Lands first because it's the foundation for the new beacon-input layout coming in Phase 3.

- New `include/autoc/nn/sensor_interface.h`: enum-indexed input names, per-input metadata (type, range, units), compile-time topology computation
- Modify `nn_inputs.h` / `nn_input_computation.cc` to use the new interface
- Migration: data.dat header, `sim_response.py` parser, `tests/contract_evaluator_tests.cc` topology assertions, `xiao/src/msplink.cpp` xiao-side gathering, `tests/nn_evaluator_tests.cc` input-layout assertions
- Tests: `nn_sensor_interface_tests.cc` for the new interface contract
- Build verification: `bash scripts/rebuild.sh` green; xiao `pio run` green

This is the single biggest pre-029 refactor. Per R7, lands as one PR before tracker-mode-specific work to avoid rebasing pain.

### 2.2 crrcsim multi-aircraft substrate (Robots accessor + reuse existing playback)

Per FR-002. **R4 research updates the architecture**: no new `RobotProgrammable` subclass needed — the existing `CRRC_AirplaneSim_Playback` class (`crrcsim/src/mod_robots/fdm_playback.{h,cpp}`) handles file-based trajectory playback exactly as 029 needs. Total cost is ~120–180 LOC (smaller than the spec's 150 estimate).

- Modify `crrcsim/src/mod_robots/robots.{h,cpp}` — add `Robots::getRobotFDM(int idx)` public accessor (~5 LOC)
- Per-scenario file selection plumbing in `inputdev_autoc.cpp` — load the right library entry into the existing CRRC_AirplaneSim_Playback per scenario (~30 LOC)
- Single airframe `hb1_streamer.xml` for both training and target craft in v1 (R4 confirms — both aircraft load the same model)
- Tests: `robot_programmable_tests.cc` (renamed from "programmable" to "playback" to match what's actually used) — multi-aircraft instantiation, state-query roundtrip, determinism under same seed and same library entry

### 2.3 dmp-to-playback converter tool

Per FR-001. R1 research provides the binary playback format spec; R2 research provides the dmp deserialization approach.

- New `tools/dmp_to_playback.cc` — reads `.dmp` files (cereal `EvalResults`), extracts per-scenario aircraft trajectories, writes one playback file per scenario in the format mod_robots' `CRRC_AirplaneSim_Playback` already reads
- CLI: `--source-run <id> --source-gen <N> --output-dir <path>`; emits 245 playback files
- Tests: `dmp_to_playback_tests.cc` — roundtrip a known dmp → playback → readback; determinism

### 2.4 RobotPathProvider + tracker-mode autoc.ini selector

Per FR-002 + FR-011.

- New `crrcsim/src/mod_inputdev/inputdev_autoc/robot_path_provider.{h,cc}` — implements the same interface as `VectorPathProvider` but pulls position from `Global::robots->getRobotFDM(targetRobotIdx)->getPos()`
- Modify `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` — autoc-tracker.ini mode selector spawns RobotProgrammable for the target craft + uses RobotPathProvider for the rabbit-substitution
- New `autoc-tracker.ini` — separate config file with `TrainingMode = tracker` selector + `LibraryDirectory = <path>` + tracker-mode-specific knobs
- Modify `src/autoc.cc` — read mode selector; delegate scenario construction to library loader when in tracker mode
- Tests: `tracker_mode_integration_tests.cc` — end-to-end deterministic eval (single scenario, fixed seed, fixed library entry → identical fitness across runs)

### 2.5 Tracker-mode `EvalResults` schema bump

Per FR-015.

- Modify `include/autoc/rpc/protocol.h` — extend `EvalResults` with optional camera view data block (per-tick projection results, camera state)
- Bump `CEREAL_CLASS_VERSION(EvalResults, ...)` per [no cereal versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md) — clean-cut, old dumps unloadable
- Tests: schema roundtrip test for tracker-mode dump

**Phase 2 exit gate**: tracker-mode autoc launches end-to-end with placeholder zero-beacon-input, runs 245 scenarios per gen with the second aircraft visibly replaying, fitness reported (will be uniformly bad since NN sees no target signal). Build green, all tests pass, xiao build green. Determinism verified (same seed = identical fitness).

## Phase 3 — US3: Camera + beacon projection + camera-config experimentation harness

**Goal**: connect the perception pipeline. NN now sees real beacon coordinates. US3's camera-config sweep capability is in place. Short training runs validate behavior before US4 commits to the long-run.

### 3.1 Camera config + projection module

Per FR-003 / FR-003a / FR-003b / FR-003c / FR-003d / FR-003e / FR-005. R3 research provides the projection-math implementation strategy.

- New `include/autoc/eval/camera_config.h` — `CameraConfig` struct with the parameter classes per spec (compile-time fixed: type, frame rate, latency, projection geometry, color filter, shutter; PRNG-varied at-nominal for v1: mount offset, orientation, FOV, projection params, aberrations)
- New `include/autoc/eval/beacon_projection.h` — projection module API: `projectBeacon(world_pos, camera_config, training_craft_pose) → (screen_x, screen_y, visible)`
- New `src/eval/beacon_projection.cc` — analytic planar pinhole projection; channel response model (binary v1: dual-pass IR filter); FOV / behind-camera tests collapsed to single `visible` flag per spec
- v1 baseline: planar pinhole, 120° FOV, single forward-mounted, 30 Hz frame rate, 0 ms latency, all aberrations stubbed identity
- Tests: `beacon_projection_tests.cc` — projection math roundtrip; edge cases (target behind camera, out of FOV, near-field); deterministic output for fixed inputs

### 3.2 Beacon model on target craft

Per FR-004.

- Define beacon body-frame positions (one per wingtip) — compile-time constant for now
- Beacon emission model: wavelength + emission cone (>180° hemisphere, brightness drops off outside)
- Wire into `RobotProgrammable` so beacon world positions emit per tick from the target's per-tick pose

### 3.3 Frame-buffer / history window infrastructure

Per FR-003e (multi-frame-per-tick → NN input mapping).

- Per-camera ring buffer holding last N camera frames (N = ceil(camera_frame_rate / NN_tick_rate)); v1 single-camera × 30 Hz × 10 Hz NN tick = 3-frame buffer minimum
- Sample selection: at each NN tick, pull frames at the dphi-pattern offsets `[-0.5, -0.4, -0.3, -0.2, -0.1, now]` (per US1 outcome — but US1 ran in pathgen mode; for tracker mode we keep the same offset distribution but apply to beacon screen coords instead of rabbit-direction unit vectors)

### 3.4 Type-safe sensor interface — beacon inputs

Per FR-006.

- Add named beacon inputs to the sensor enum: `BEACON_L_X[t]`, `BEACON_L_Y[t]`, `BEACON_L_VISIBLE[t]` for each time offset, mirrored for `BEACON_R_*`
- v1 layout: 4 history slots × 3 fields × 2 beacons × 1 camera = 24 beacon-related inputs + aircraft state ≈ 32 floats total
- The pathgen-mode rabbit-direction inputs are *replaced* (not coexisted) when in tracker mode — different sensor layout per mode

### 3.5 US3 camera-config experimentation harness

Per FR-003 / SC-009 / SC-010.

- Make `CameraConfig` instances easily editable in autoc-tracker.ini OR via compile-time selection of one of several preset configs in `camera_config.h`
- Document a v1 sweep grid: FOV {60°, 90°, 120°}, mount {nose, canopy}, frame rate {30, 60} for example
- Operator workflow: edit one field, rebuild, re-run training (10-50 gens for quick comparison), check per-axis aggressiveness for behavior comparison

**Phase 3 exit gate**: short training run (50-100 gens) at v1 baseline produces sane-looking fitness descent and the controller demonstrably tracks (visual inspection in renderer or via fitness signature). Multiple camera configs validated by US3 experimentation harness — sweep at least 3 variants (FOV change, mount change, frame rate change). Determinism preserved.

## Phase 4 — US4: Long-run controller training

**Goal**: produce a tracker-mode controller meeting the SC-003 / SC-004 / SC-005 / SC-006 success criteria.

### 4.1 Choose v1 baseline camera config from US3 results

US3 phase yields a chosen baseline config. This is the config US4 trains against.

### 4.2 Long-run training launch

```bash
nohup ./build/autoc -c autoc-tracker.ini > logs/autoc-029-tracker-base.log 2>&1 &
```

Path-A-class config (pop 5000 × 600 gens, recurrent NN, single seed). Library = source pathgen run (the more-rnn3 final or whichever recording is the agreed seed library).

### 4.3 Monitor + analyze + outcome

Same telemetry cadence as 028 (6-panel evolution + per-axis aggressiveness + per-axis time series). Outcome doc: `specs/029-tracker-mode/tracker-base_outcome.md` documenting whether SC-003 / SC-004 / SC-005 / SC-006 pass.

### 4.4 GPU-eval co-dependency — RESOLVED (NOT required)

**R5 research resolved**: GPU eval is *not* a 029 v1 prerequisite. Projection cost is 5–10 % over 028 per-tick budget — manageable on CPU. The earlier spec caveat (line 73) assumed 017's vision-NN cost model; 029's analytic 3-float interface is fundamentally different. GPU eval becomes critical only if 029 expands to multi-camera @ 60+ Hz or sub-tick NN evaluation — none of which are v1 scope.

## Phase 5 — US5 + US6: Renderer dual-mode + architectural review

### 5.1 Renderer dual-mode

Per FR-012 / FR-012a.

- Modify `tools/renderer.cc` (or wherever renderer lives — see renderer file map) to load tracker-mode `.dmp` files
- Add 3rd-person view: render both aircraft (training + target), beacons on target wingtips, camera FOV cone from training craft, per-tick error overlays
- Add 1st-person camera-POV view: project beacons through the recorded camera state, render colored points at projected positions, FOV bounds at screen edges
- Per-tick scrub controls (pause / step-forward / step-backward) — rolled in from BACKLOG renderer-playback-enhancements

### 5.2 Architectural review (US6)

Code review at end of Phase 4 to confirm the perception-to-NN interface separation is clean — same NN binary would work against sim beacons or any future perception module. Documented in outcome.

## Phase 6 — Close 029

- Update `BACKLOG.md` to mark rolled-in items complete (Type-Safe NN Sensor Interface, Renderer Playback Enhancements)
- Generate `specs/029-tracker-mode/findings.md` summarizing experimental outcomes + open questions for follow-on features
- Carry-forward to next milestone (real-target tracking — gates on path-5 random-intercept proof)

## What this plan intentionally does NOT cover

- **Real-world deployment**. Sim gate first; flight test happens in a future feature.
- **Per-scenario camera variation** (manufacturing tolerance training). Architecturally enabled, not implemented in v1.
- **Library curation / auto-bootstrapping**. v1 takes a single fixed source run.
- **Pixel-buffer rendering / vision NN perception layer**. Analytic projection only.
- **Physics-driven target aircraft**. Kinematic playback only.
- **GPU eval implementation** (if R5 says it must land first, that's a separate feature).

## Open implementation decisions (resolve in `/speckit.tasks` or during implementation)

1. **Camera latency v1 default**: 0 ms (clean fitness comparison) vs realistic 30-50 ms. Per spec deferred clarify item; pick during US3 setup.
2. **US3 camera-config sweep granularity**: how many configs to compare before US4 baseline lock-in? 3-5 seems right.
3. **GPU eval co-dependency**: per R5 research finding. Confirm in research.md outcome.
4. **Tracker-mode dump schema details**: which fields exactly land in the extended `EvalResults`. Pin in [contracts/tracker_dmp_schema.md](./contracts/tracker_dmp_schema.md).
5. **autoc-tracker.ini schema**: which knobs are per-mode vs shared with autoc.ini. Pin in [contracts/](./contracts/).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified.**

No violations. Section intentionally empty.
