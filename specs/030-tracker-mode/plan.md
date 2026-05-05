# 030 — Implementation Plan (Tracker Mode — beacon-camera target tracking via flight playback)

**Branch**: TBD (to be created when implementation begins; current scoping work happens on `029-no-future-arch`) | **Date**: 2026-05-04 (fresh rewrite per operator scoping pass) | **Spec**: [spec.md](./spec.md)
**Input**: [`spec.md`](./spec.md) (28 FRs, 16 design notes), [`research.md`](./research.md) (this plan's research output), [BACKLOG.md "030 spin-offs"](../BACKLOG.md) (031 candidates and pure-backlog defers)
**Status**: Plan-research phase — no implementation work has started.

## Summary

030 introduces a **second autoc training mode** (sibling to pathgen mode) that trains a controller to track another aircraft via a simulated wingtip-beacon camera, using a recorded prior-run S3 dmp file as the target-trajectory source. The control NN's output remains pitch / roll / throttle; its inputs are the analytic projection of two outward-facing 270° wingtip beacons through a configurable camera model (v1: planar pinhole at 120° FOV, 30 Hz, single forward-mounted on top of wing-chord), collapsed at the perception interface to **`(x, y, CEP)`** per beacon (FR-005) — mirroring the FPGA centroid + cluster-spread output the eventual real-hardware perception pipeline produces (the 031-candidate parallel feature; see [BACKLOG.md](../BACKLOG.md)).

The implementation is structured as a **smoke-test-first milestone ramp** (per spec D13 / D16). The "v1 done" floor is the four smoke-test deliverables: run M2 from an M1 file → single-path config → save results → renderer animates the M2 result. Above that floor, post-smoke-test analytics experimentation refines the tracking architecture and is the part of 030 most likely to bracket between "smoke green" and "030 declared done." Items past 030 done line move to 031 (BACKLOG roll-out already in place).

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, xiao firmware), Python 3.11 (analysis scripts, possibly per-tick dmp extractor — TBD plan-phase)
**Primary Dependencies**: Eigen (vec3 / quat / matrix math, projection geometry); cereal (NN serialization, EvalResults schema, new tracker-mode dump format); inih (autoc.ini parsing); GoogleTest (unit / contract tests); CRRCSim LaRCSim FDM (sim physics — chase craft only in v1; multi-aircraft substrate via `mod_robots` is M4-deferred); INAV MSP protocol (xiao deploy, post-v1)
**Storage**: file-based — `data.dat` (training output, schema extends per FR-015), evolution log lines (per-gen telemetry), `data.stc` (per-best-of-gen renderer feed), S3 `.dmp` (cereal-serialized `EvalResults` — schema bump for tracker-mode dumps per FR-015a + Constitution V), source-run `.dmp` (loaded directly per FR-001 — *no* separate playback library files in this revision); xiao flash logs (post-v1 deploy)
**Testing**: GoogleTest contract tests for type-safe sensor interface, beacon projection determinism, FR-018 timing-loop determinism, dmp version round-trip; existing pathgen-mode test suite must continue to pass (Constitution Principle II — Build Stability)
**Target Platform**: Linux desktop for autoc + crrcsim + renderer (training-time); xiao deploy is post-v1 and is a 031+ concern
**Project Type**: multi-component embedded-aware C++ tree (autoc evolution engine + crrcsim FDM + renderer + xiao firmware), single repo
**Performance Goals**: smoke-test single-path × small population → CPU-feasible in minutes-to-hours per run; full-scale multi-path × full-pop training is post-v1 and is gated by the [GPU-Native Evaluation backlog item](../BACKLOG.md) (re-flagged at v1→v2 boundary, NOT a v1 prereq per D13)
**Constraints**: Constitution V (versioned persistence) — first dmp schema bump lands here; Constitution III (no compatibility shims) — type-safe sensor interface is a clean cut, FR-006 names replace magic-number indexing; Constitution II (build stability) — every milestone must keep autoc+crrcsim+xiao building (the rolled-in mod_inputdev linkage fix, [BACKLOG.md "[NEXT] crrcsim mod_inputdev"](../BACKLOG.md), is M1 prerequisite work)
**Scale/Scope**: source dmps from pastonly3 / more-rnn3-class runs (~245-294 scenarios per gen, multi-tick aircraftStateList per scenario); v1 smoke test exercises 1 scenario at small population for signal-or-not.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Constitution version 1.1.0 (Last Amended 2026-05-04 to add Principle V).

| Principle | Status | Notes |
|---|---|---|
| **I. Testing-First** | PASS | Each milestone (below) names contract / integration tests it lands. Existing pathgen-mode test suite must remain green at every milestone (regression invariant). |
| **II. Build Stability** | PASS | M1 lands the mod_inputdev linkage fix as the *first* implementation work, before any new file in `src/nn/` or `src/eval/` is added — otherwise the next addition silently breaks the crrcsim build at link (the 028 telemetry incident). Every subsequent milestone passes `bash scripts/rebuild.sh` and `cd xiao && pio run -e xiaoblesense_arduinocore_mbed`. |
| **III. No Compatibility Shims** | PASS | FR-006 type-safe sensor interface is a clean replacement for magic-number `float[]` indexing — full files-to-touch (sim + xiao + tests + scripts), no backwards-compat shim. FR-015 dmp schema bump is greenfield: pre-versioning dmps are pre-Constitution-V and treated as version-0 with documented assumption (per FR-015a). No re-export / wrapper layer. |
| **IV. Unified Build** | PASS | `mod_inputdev` linkage fix routes crrcsim's `mod_inputdev` to link `autoc_common` rather than cherry-picking sources — restores Principle IV's "single source of truth" property. |
| **V. Versioned Persistence Artifacts** | PASS | First load-bearing application of the principle: tracker-mode dmp schema bump lands with FR-015a (version field embedded; readers attempt back-compat then fail loud). Establishes the pattern for future `.dmp` evolution. |

**Gate result**: PASS. No violations to track in Complexity section.

## Project Structure

### Documentation (this feature)

```text
specs/030-tracker-mode/
├── plan.md              # This file (/speckit.plan output, fresh rewrite 2026-05-04)
├── research.md          # Phase 0 output (fresh rewrite — supersedes prior 029-era research)
├── data-model.md        # Phase 1 output (entities — Source-scenario target trajectory, CameraConfig, BeaconObservation, etc.)
├── quickstart.md        # Phase 1 output (smoke-test runbook)
├── contracts/           # Phase 1 output (refreshed for (x,y,CEP) interface + FR-018 timing model)
│   ├── nn_sensor_interface.md     # FR-006 typed sensor names + dtype + dynamic ranges
│   ├── beacon_projection_api.md   # FR-005 projection module input/output contract
│   ├── tracker_dmp_schema.md      # FR-015 + FR-015a dmp schema (+ versioning)
│   └── source_dmp_loading.md      # FR-001 source-dmp load contract (replaces prior playback_file_format)
├── spec.md              # Source spec (28 FRs, 16 design notes)
└── tasks.md             # Phase 2 output (NOT created by /speckit.plan — happens at /speckit.tasks)
```

### Source Code (repository root, deltas from current tree)

```text
include/autoc/
├── nn/
│   ├── topology.h                  # FR-006: type-safe sensor enum + count auto-derivation
│   ├── nn_inputs.h                 # FR-006: typed BEACON_L_X[t] / BEACON_L_Y[t] / BEACON_L_CEP[t] etc.
│   └── ...
├── eval/
│   ├── aircraft_state.h            # FR-015: tracker-mode fields (camera view data, copied target traj)
│   ├── camera_config.h             # NEW — FR-003 + FR-003a-f camera parameter struct (v1 fixed; PRNG-varied dimension architectural)
│   ├── camera_projection.h         # NEW — FR-005 analytic projection → (x, y, CEP), FR-017 int8 quant
│   ├── beacon_config.h             # NEW — FR-004 wingtip beacon parameters (270° outward, wavelength, body-frame mount)
│   ├── trail_rabbit.h              # NEW — FR-008/8a trailing-rabbit computation
│   ├── crash_hull.h                # NEW — FR-008b hull definition + intersection test (mode-gated)
│   ├── arena.h                     # NEW or refactor — FR-016 in-flight bounds (depends on plan-phase R5 outcome)
│   └── ...
└── rpc/
    └── protocol.h                  # FR-015 EvalResults extension; FR-015a version field

src/
├── autoc.cc                        # FR-011 (autoc-tracker.ini handling), FR-018 (M1-timestamp main loop branch), tracker mode dispatch
├── eval/
│   ├── camera_projection.cc        # NEW — FR-005 implementation
│   ├── trail_rabbit.cc             # NEW — FR-008
│   ├── crash_hull.cc               # NEW — FR-008b
│   └── ...
├── nn/
│   └── (FR-006 type-safe interface impl across evaluator.cc / serialization.cc / population.cc)
└── ...

crrcsim/
└── (NO changes to crrcsim/src/mod_robots/ for v1 — M4 deferred to post-v1)
   # The post-v1 work:
   #   src/mod_robots/robot_programmable.{h,cc}  # RobotBase subclass consuming
   #     in-memory pose stream — landed when video-enabled mode requires
   #     live two-aircraft display in crrcsim's 3D viewer during training.

tools/
├── renderer.cc                     # FR-012 tracker-mode views + camera-POV mini-panel; FR-012a per-tick scrub + streak overlay
├── aircraft_state_extractor.cc     # NEW — rolled-in BACKLOG: per-tick dmp → CSV
└── ...

xiao/
└── (FR-006 type-safe interface mirror — scaffolding in v1; full perception path is post-v1, since 030 v1 is sim-only)

tests/
├── beacon_projection_tests.cc          # NEW — FR-005 contract tests (known geometry, sentinel handling, int8 quant)
├── timing_model_tests.cc                # NEW — FR-018 determinism contract test (variable-rate source, sim-clock-speed-invariance)
├── trail_rabbit_tests.cc                # NEW — FR-008/8a math + degenerate-velocity fallback
├── crash_hull_tests.cc                  # NEW — FR-008b hull intersection + p_crash distribution
├── tracker_dmp_roundtrip_tests.cc       # NEW — FR-015 + FR-015a serialize/deserialize + version handling
├── nn_sensor_interface_tests.cc         # NEW — FR-006 named-input contract (mirror of nn_evaluator_tests)
└── (existing pathgen-mode tests must stay green)
```

**Structure Decision**: Existing single-repo C++ tree extended in-place. New eval-side modules (`camera_*.{h,cc}`, `trail_rabbit.*`, `crash_hull.*`) live alongside the existing `src/eval/` family. **No changes to `crrcsim/src/mod_robots/` for v1** — M4 (live two-aircraft display in crrcsim's 3D viewer during training) is deferred to post-v1; v1 reads the target trajectory from autoc memory and computes virtual beacons in the projection module without crrcsim hosting the target. New tools (per-tick dmp extractor) live in `tools/`. No new top-level directories (perception-front-end work — when 031 unparks — likely earns one then).

## Milestone-ordered implementation phases

Each milestone has a "what works after this is green" answer (the visible checkpoint) and the contract tests that lock in that capability. Order minimizes accumulated unknowns: prerequisites first, smoke-test cone last, post-smoke analytics on top.

> **Reading guide**: M0 is plan-research (this phase). M1–M3 are precondition / scaffolding. M4–M9 build the smoke-test cone. M10 = SMOKE TEST (D13 floor). M11+ = post-smoke analytics (the area where "030 done" line gets drawn vs 031). The first ten milestones are sequenced; M11+ are largely parallel and plan-phase orderable.

### M0 — Plan-research (this phase, no code)

**What works after**: every open architectural question in the spec has a concrete decision recorded in `research.md`; gates the start of M1.

**Open questions to resolve** (full list in [research.md](./research.md)):
- **R1**: source dmp loading path — direct cereal load into autoc memory, vs. write-then-replay via crrcsim mod_robots/playback `.crrclog` files (the prior research's R1 default). The FR-001 revision favors the direct path; confirm crrcsim mod_robots can accept an in-memory pose stream from a new `RobotProgrammable` subclass.
- **R2**: arena enforcement primitive — extend existing `ENTRY_SAFE_RADIUS` / `ENTRY_SAFE_ALT_MIN/MAX` constants (`src/autoc.cc:266`, currently entry-time-only) into in-flight bounds, vs. add parallel `FLIGHT_ARENA_*` constants. Avoid silent overload (D11).
- **R3**: `p_crash` v1 default — deterministic (`p_crash = 1.0`), zero (`p_crash = 0.0`, sphere defined but inert), or curriculum-anneal (`p_crash` ramps low → high across gens). Choice depends on whether early-gen exploration is fitness-landscape blocked by deterministic crash.
- **R4**: type-safe sensor interface concrete shape — enum + tag-dispatch + `constexpr` count derivation, vs. struct-of-arrays, vs. some other pattern. Carries forward to xiao-side (FR-006 full scope), so mirroring constraints matter.
- **R5**: FR-018 main-loop refactor — how to share pathgen ↔ tracker mode worker / population / fitness-aggregation pipelines while diverging cleanly on per-tick stepping logic. Likely a strategy-pattern split inside the worker.
- **R6**: CEP encoding — what scaling produces the most-useful gradient for the NN to learn discount-on-noisy? Linear vs log-spread vs piecewise. Sentinel choice for invisibility (NaN-marker vs INT8_MIN as integer sentinel).
- **R7**: int8 quantization round-trip — exact int8 mapping for `x ∈ [-1, +1]` and CEP, sentinel reservation, fp32 conversion edge cases.
- **R8**: M1 source dmp schema — what fields actually live in `aircraftStateList[scenario][step]` today, what gaps exist for tracker-mode needs (target-craft pose-per-tick, source-scenario joint-PRNG params), what extra extraction is required vs already-present.
- **R9**: renderer — confirm existing `vtkCamera` viewport math is unrelated to FR-005 analytic projection (don't accidentally couple). Confirm renderer can load tracker-mode dmps without breaking pathgen-mode dmp loading (FR-015a back-compat path).
- **R10**: trail-rabbit degenerate fallback — exact transition policy from velocity-based to nose-based when target speed → 0 (continuous interpolation vs hard-switch threshold).

**Output**: `research.md` with R1–R10 each carrying Decision / Rationale / Alternatives / Citations.

**Tests**: none (research-only).

### M1 — Build precondition: mod_inputdev linkage fix + dmp versioning groundwork

**What works after**: `bash scripts/rebuild.sh` is clean; `cd xiao && pio run` is clean; the project is *ready* to receive new files in `src/nn/` and `src/eval/` without the cherry-picked-sources fragility detonating the crrcsim build.

**Work**:
1. crrcsim `mod_inputdev` rolled-in BACKLOG fix: replace cherry-picked `${CMAKE_SOURCE_DIR}/src/nn/evaluator.cc` etc. lines with `target_link_libraries(mod_inputdev autoc_common)`. Audit for duplicate-symbol issues. (`crrcsim/src/mod_inputdev/CMakeLists.txt:21-23`)
2. Add the FR-015a dmp version field (single integer, embedded at a stable offset) to the existing `EvalResults` cereal schema. Pre-versioning dmps in S3 are read with documented version-0 assumption. Version increments are reserved for the M8 schema bump.

**Tests**: build-only — no functional tests. Existing pathgen-mode test suite stays green.

**Visible checkpoint**: green build on a fresh checkout; one-line `git log` showing the linkage fix + version-field add.

### M2 — Type-safe NN sensor interface scaffolding (FR-006 full scope)

**What works after**: pathgen-mode runs unchanged with the new typed interface; magic-number indexing into NN inputs is gone from the entire codebase (sim + xiao + tests + analysis scripts).

**Work** (rolled-in BACKLOG entry "[NEXT] Type-Safe NN Sensor Interface" — full files-to-touch list):
1. `include/autoc/nn/topology.h` — sensor enum + count auto-derivation; `static_assert` against the historical magic count.
2. `include/autoc/nn/nn_inputs.h` — typed accessors keyed by enum.
3. `include/autoc/autoc.h` — drop duplicate defines (`DISTANCE_TARGET` etc.).
4. `src/nn/evaluator.cc` — `nn_gather_inputs()` rewritten to populate by typed name; index mapping derived from enum.
5. `src/autoc.cc` — `data.dat` format string, header, field indices generated from enum.
6. `tests/contract_evaluator_tests.cc` + `tests/nn_evaluator_tests.cc` — assertions against typed names.
7. `specs/019-improved-crrcsim/sim_response.py` — parser keys off enum-derived header names.
8. `xiao/src/msplink.cpp` — xiao-side input gathering uses the same enum mirror (compile-time-shared header).
9. `include/autoc/eval/aircraft_state.h` — `nnInputs_` array typing + serialization preserves names.

**Why this lands ahead of any beacon work**: the new `BEACON_L_X[t]` / `BEACON_L_Y[t]` / `BEACON_L_CEP[t]` names land cleanly only if the typed scaffolding is already in place. Trying to add them ad-hoc against magic-number indexing risks the silent-corruption bug class the typed interface exists to prevent.

**Tests**: `nn_sensor_interface_tests.cc` — contract test for typed name → index round-trip; `tests/contract_evaluator_tests.cc` updated; existing pathgen-mode tests stay green (the typed interface is behavior-equivalent for pathgen inputs).

**Visible checkpoint**: pathgen-mode training run completes with byte-identical `data.dat` output to the pre-M2 run (regression-tight); no magic-number indexing remains in the codebase (`grep` proof).

### M3 — Source M1 dmp loader (FR-001)

**What works after**: a CLI invocation reads a real S3 dmp from a prior pastonly3 / more-rnn3 run and prints per-scenario summary stats (scenario count, tick count per scenario, joint-PRNG variation params, target-craft pose extents). Standalone, no autoc-side integration yet.

**Work**:
1. Wire `EvalResults` cereal load into autoc startup (or initially into a standalone tool, then graduate). Pattern follows `tools/nnextractor.cc:177-192`.
2. Indexed in-memory representation: `vector<SourceScenarioTrajectory>` keyed by scenario index; each entry carries per-tick pose + the joint-PRNG variation parameters.
3. Per FR-011 source-key-string parsing: accept `autoc-storage/<run-id>/gen<N>.dmp` form, round-tripping with xiao-log convention.

**Tests**: integration test loading a known dmp file (small fixture committed to test data, or pulled from a known S3 prefix); contract test for the source-key-string parser.

**Visible checkpoint**: command-line `tools/source_dmp_inspect <path>` prints scenario count, mean-tick-count, sample target-craft pose at tick 0 and tick mid. Operator can sanity-check dmp shape before any tracker-mode run launches.

### M4 — [DEFERRED to post-v1] Two-aircraft sim infrastructure in crrcsim

**Status (2026-05-04)**: **Deferred from v1**. The smoke test (D13) strictly computes virtual beacons on a screen of a certain warp — the math; no live in-crrcsim two-aircraft visualization needed. The target trajectory lives in autoc memory only (M3); the projection module (M5) reads `(target pose at t_i)` directly from memory and computes beacon `(x, y, CEP)` from `(chase pose from crrcsim, target pose from memory)`. crrcsim during training runs the chase craft alone, exactly as today.

**Renderer 3rd-person view of both aircraft (M9)**: handled via the renderer's own VTK actors, drawing both `aircraftStateList` (chase) and `targetTrajectoryList` (target copy from source) from the M2 dmp directly per FR-015 self-containedness. Also needs no crrcsim mod_robots integration.

**When this milestone unparks (post-v1)**:
- When operator wants to *watch training in progress* in crrcsim's 3D viewer (live two-aircraft display during training, not playback after-the-fact).
- When the camera-pixel-to-(x, y) parallel feature (031 candidate) needs a real visual rendering of the target craft to feed into image-domain processing experiments.

**Original work scope** (preserved here for the post-v1 reactivation):
1. New `crrcsim/src/mod_robots/robot_programmable.{h,cc}` — `RobotBase` subclass consuming an in-memory pose stream (per R1 v1+ alternative path).
2. `Robots::AddRobot` integration so autoc-side can register a programmable robot per scenario.
3. Per-scenario teardown / reset.

Per the existing `mod_robots` reference research ([reference_crrcsim_mod_robots.md](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_crrcsim_mod_robots.md)) — ~150 LOC sketch, well-bounded. Becomes M4 work whenever v1+ priorities call it.

**v1 milestone numbering**: M5–M11 retain their original numbers despite M4 dropping out, to preserve cross-references in research.md and contracts/ during the v1 build. The "M4 slot" is intentionally empty; subsequent milestones (M5 onward) follow M3 directly with no gap in actual work.

### M5 — Beacon projection module (FR-003 + FR-004 + FR-005 + FR-007 + FR-017)

**What works after**: given (chase pose, target pose, target attitude, camera config, beacon config), the projection module emits `(x, y, CEP)` per beacon — including int8 quantization round-trip and sentinel handling. Standalone unit-tested; no NN integration yet.

**Work**:
1. `include/autoc/eval/camera_config.h` + `beacon_config.h` — parameter structs, v1 baseline values committed (single camera, planar pinhole, 120° FOV, 30 Hz, top-of-wing-chord; 270° outward beacons at wingtip body-frame positions).
2. `include/autoc/eval/camera_projection.{h,cc}` — analytic pinhole projection per the existing R3-era research math (still valid; output type changes from `(x, y, visible)` to `(x, y, CEP)`). Airframe self-occlusion via coarse body-shape proxy (D10).
3. CEP computation — small for sharp centroid, larger near edges / aberration zones, sentinel for off-screen / behind / occluded.
4. Int8 quantization (FR-017) round-trip — `[-128, +127] ↔ [-1.0, +1.0]` for x/y; CEP with `INT8_MIN` reserved as invisibility sentinel.
5. The decision from M0/R6 + R7 locks in CEP encoding semantics + sentinel.

**Tests**: `beacon_projection_tests.cc` — contract tests against known geometry (target dead-ahead, target left/right edge, target behind, target occluded by airframe proxy), int8 round-trip determinism, sentinel correctness for each invisibility cause.

**Visible checkpoint**: a contract test that loops chase/target poses through the projection and asserts known `(x, y, CEP)` triples — operator can read the test as documentation of "what does the perception pipeline produce."

### M6 — Tracker-mode autoc.ini parsing + main-loop branch (FR-011 + FR-018)

**What works after**: autoc launched with `-i autoc-tracker.ini` enters tracker mode; the FR-018 M1-timestamp-driven main loop drives one chase-craft iteration per source-data sample; pathgen mode unchanged.

**Work**:
1. New `autoc-tracker.ini` parser entries: `TrackerSourceRun`, `TrackerScenarioSubset` (single scenario for v1), `TrailDistance` (default 10ft = 3.048m), `CrashHullShape` + `CrashHullRadius` + `pCrash` (defaults from R3 decision), `ArenaRadius` + `ArenaFloorAGL`, etc.
2. Mode dispatch in autoc.cc: pathgen vs tracker by config-file content. Mutual exclusion enforced (D4).
3. FR-018 main loop refactor — the strategy-pattern split decided in M0/R5. Worker contract unchanged; per-tick stepping logic differs.
4. Source-dmp wired in via M3's loader at autoc startup.
5. Per-scenario distribution to workers.

**Tests**: `timing_model_tests.cc` — contract test for FR-018 determinism (variable-rate source samples, sim-clock-speed-invariance); functional smoke (without fitness yet) — short tracker-mode run launches, runs, terminates cleanly.

**Visible checkpoint**: log output shows tracker-mode startup, source-dmp loaded, per-scenario distribution, gen 0 evaluations executing. Operator believes the loop machinery works end-to-end.

### M7 — Tracker fitness: trail-rabbit + crash hull + arena (FR-008 + FR-008a + FR-008b + FR-016)

**What works after**: per-tick fitness for tracker mode produces sensible numbers — chase craft following the trail rabbit gets high fitness; chase craft hitting the crash hull gets a fitness penalty; chase craft leaving the arena gets penalty / scenario terminated.

**Work**:
1. `src/eval/trail_rabbit.cc` — FR-008 + FR-008a per-tick rabbit recompute from current target velocity (with degenerate-fallback per R10 decision).
2. `src/eval/crash_hull.cc` — FR-008b sphere intersection + probabilistic firing per R3 decision; mode-gated on `Mode = Tracker`.
3. Arena enforcement (FR-016) — wired to the primitive chosen in M0/R2 (extend `ENTRY_SAFE_*` vs new `FLIGHT_ARENA_*`).
4. Tracker-mode `FitnessComputer` path (reuse cone-surface fitness; substitute trail-rabbit for path point).

**Tests**: `trail_rabbit_tests.cc`, `crash_hull_tests.cc` — math + degenerate cases + p_crash distribution.

**Visible checkpoint**: tracker-mode short run produces a per-gen fitness number; operator can launch with extreme parameter values and watch fitness break in expected directions (trail distance huge → fitness terrible; arena tiny → fitness terrible).

### M8 — Tracker-mode dmp output (FR-015 + FR-015a)

**What works after**: tracker-mode best-of-gen produces a self-contained `.dmp` file (per FR-015's two embedded classes: per-tick beacon `(x, y, CEP)` projection state + camera pose; copied target-craft trajectory from the source). Pre-versioning pathgen dmps still readable (FR-015a back-compat).

**Work**:
1. `EvalResults` schema extension — camera view data, target trajectory copy.
2. Version-field bump (Constitution V).
3. Read-side adjustments: pathgen-mode renderer still loads pathgen dmps; tracker-mode renderer (M9) loads tracker dmps.
4. Round-trip serialize/deserialize tests.

**Tests**: `tracker_dmp_roundtrip_tests.cc` — serialize/deserialize identity, version field handling, pre-versioning dmp falls through to documented version-0.

**Visible checkpoint**: an actual tracker-mode `.dmp` lives in S3 from a real (small) run. `cereal` round-trips it. Operator can `cat` the metadata layer and see scenario count + version.

### M9 — Renderer tracker-mode playback (FR-012 + FR-012a)

**What works after**: renderer loads a tracker-mode `.dmp` and animates the M2 result — 3rd-person view (both aircraft, beacons on target wingtips), camera-POV mode (1st-person from chase camera), camera-POV mini-panel (current-tick beacon screen positions in the HUD). Per-tick scrub controls work (rolled-in BACKLOG renderer enhancements). Reads target trajectory from the M2 dmp directly (not from the M1 source — per FR-015 self-contained property).

**Work** (rolled-in BACKLOG entries: type-safe interface already done in M2; renderer enhancements lands here):
1. `tools/renderer.cc` — tracker-mode dmp loader path; 3rd-person + 1st-person views; camera-POV mini-panel slot in the existing HUD overlay system (per D5 — follows existing visibility logic, not always-on).
2. Per-tick scrub controls (pause / step-forward / step-backward).
3. Streak / multiplier overlay (optional plumbing — recompute path or schema-bump path per backlog entry; pick during M9).
4. **CEP error-bar visualization** — committed v1 per spec D15 trim — renders CEP as a visible spread / ellipse around each projected beacon centroid.

**Tests**: visual + a smoke renderer-loads-tracker-dmp test; existing pathgen-renderer tests stay green.

**Visible checkpoint**: **operator opens renderer on a tracker-mode dmp, sees two aircraft + beacons + camera-POV mini-panel + CEP error bars + per-tick scrub.** This is the qualitative "do we believe the loop closes" eyeball test, ahead of the full smoke test.

### M10 — SMOKE TEST (D13 floor — 030 v1 done line minimum)

**What works after**: end-to-end loop closes on a real M1 source dmp + real tracker-mode autoc run + real renderer playback. Fitness curve does *something* — descends, plateaus, or fails informatively.

**Work**:
1. Pick an M1 source dmp (pastonly3 gen-N, single scenario index).
2. Configure `autoc-tracker.ini` for that scenario, default trail / hull / quantization / arena.
3. Launch autoc, run a small population × small generations.
4. Observe results in renderer.
5. Compute per-axis aggressiveness + tracker-specific telemetry from the M2 dmp output stream.
6. Capture findings.

**Tests**: this *is* the test — operator-driven end-to-end run. No automated test gates here; the gates were at M5/M6/M7/M8/M9 individually.

**Visible checkpoint**: the four D13 deliverables, each demonstrably done:
1. ✓ Autoc loaded an M1 dmp and ran tracker mode against it.
2. ✓ Single scenario per autoc-tracker.ini.
3. ✓ Results are saved (M2 dmp + per-gen logs).
4. ✓ Renderer animated the M2 result.

### M11+ — Post-smoke-test analytics experimentation (the "030 done" line is somewhere in here)

**What works after**: operator has tools to assess what trained — per-axis aggressiveness on tracker outputs, visual-lock-fraction time series, perception-error visualization, crash-hull strike rate, etc. Plan-research's "030 done line" lands somewhere in this ramp.

**Parallel work streams** (no strict ordering; pick by what the smoke-test signal surfaces):

- **M11a — Per-tick dmp extractor** (rolled-in BACKLOG entry — the per-axis analytics tool extension): `tools/aircraft_state_extractor.cc` reads tracker-mode dmps + emits CSV with the new tracker-mode column set (beacon `(x, y, CEP)`, camera pose, target-craft pose, trail-rabbit position).
- **M11b — Eval Fitness Bug 2 fix** (rolled-in BACKLOG entry): `genome.fitness` updated with eval result before serializing. Renderer shows eval-mode tracker fitness, not training-time fitness.
- **M11c — Tracker-specific analytics**: visual-lock-fraction time series, crash-hull strike rate per-gen, per-axis aggressiveness comparison with pathgen-mode controllers.
- **M11d — Plan-phase candidates for v1 inclusion** (per D15 trim): nothing else committed; everything past D15's "stays in v1" list is 031 territory.

**Where "030 done" lands**: plan-research's call. Floor is M10 smoke green; ceiling is somewhere before the 031-CANDIDATE work starts (parallel perception-front-end, library curation + mirror-pairing, variable-rate source robustness, renderer reverse-projection / dphi-overlay / crash-strike-viz exotic goodies). Concrete ceiling proposal: **030 done = M10 + M11a + M11b + M11c**, leaving everything else to 031.

## Why this milestone order

Reasoning behind the M0 → M11 progression, in case plan-research wants to argue with it:

1. **M0 (research) before any code** — the open questions (R1–R10) are coupled enough that resolving them in isolation later means rework. Resolve up front.
2. **M1 (build precondition) before any new file in `src/nn|eval/`** — the 028 telemetry incident proved that the `mod_inputdev` cherry-pick pattern silently breaks the crrcsim build when new transitively-referenced files appear. Fixing this *first* means M2 onward never trips it.
3. **M2 (typed sensor) before M5 (beacon projection)** — the new `BEACON_L_X[t]` / `BEACON_L_CEP[t]` typed names land cleanly only against typed scaffolding. Otherwise we'd rebuild the typed interface twice (once in M5 with magic numbers, once in a follow-up).
4. **M4 deferred to post-v1** — smoke test computes virtual beacons strictly as math; no live in-crrcsim two-aircraft display needed for v1. Source target trajectory in autoc memory (M3) feeds the projection module (M5) directly. Renderer (M9) draws both aircraft from the M2 dmp via VTK without crrcsim mod_robots involvement.
5. **M3 (source dmp load) before M5 (projection)** — projection module reads target-craft pose from the in-memory `SourceScenarioTrajectory`; M3 produces that data structure. Reverse order would stub the source side.
6. **M5 (projection) before M6 (main loop)** — M6 wires perception-output into the worker contract; M5 produces that output. Reverse order would stub perception in M6 then re-wire in M5.
7. **M6 (main loop + autoc-tracker.ini) before M7 (fitness)** — M7's per-tick fitness machinery needs the M6 main loop to *fire* the per-tick callback. Could be parallelized partially; likely simpler in sequence.
8. **M7 (fitness) before M8 (dmp output)** — M8's dmp captures the fitness numbers M7 computes. Reverse order would stub fitness in M8.
9. **M8 (dmp output) before M9 (renderer)** — M9 reads dmps; M8 produces them. Hard sequential.
10. **M9 (renderer) before M10 (smoke test)** — D13's 4th deliverable is renderer animation of the M2 result. Smoke is undefined without the renderer working.
11. **M10 (smoke) before M11 (analytics ramp)** — analytics on a non-running loop is empty. Smoke green is the prerequisite; analytics tooling answers questions the smoke run raises.

## Quick-test cadence

Each milestone produces a visible / quick-testable artifact (above). Cadence target: M1 → M9 land on **roughly weekly** checkpoints if the team works straight through; M10 smoke is a multi-hour operator-driven run; M11 analytics is open-ended and bracketed by the "030 done" decision.

If a milestone overshoots its visible-checkpoint deliverable, that's a signal to split the milestone (e.g., M5 has natural sub-splits at M5a projection geometry, M5b CEP + sentinel, M5c int8 quantization). Plan-research output may pre-split these per the R1–R10 outcomes.

## Complexity Tracking

No Constitution Check violations to justify; section intentionally empty.

## Phase progression

- **Phase 0 (research.md)**: in-progress with this plan-write — see [research.md](./research.md). Resolves R1–R10. Output: each R-question has Decision / Rationale / Alternatives / Citations.
- **Phase 1 (data-model.md, contracts/, quickstart.md)**: refreshed alongside this plan to reflect current spec. Existing artifacts dating from the 029-era pre-pivot are superseded by the fresh write.
- **Phase 2 (tasks.md)**: NOT generated by `/speckit.plan`. Will be generated by `/speckit.tasks` once plan + research + design are reviewed and accepted.
