# Tasks: 040 Camera Redo — Perception Fidelity Refinement for M2

**Input**: Design documents from `/specs/040-camera-redo/`
**Prerequisites**: [plan.md](plan.md), [spec.md](spec.md), [research.md](research.md),
[data-model.md](data-model.md), [contracts/](contracts/), [input-data-checklist.md](input-data-checklist.md)

**Tests**: **MANDATORY** — Constitution Principle I (Testing-First) requires tests written before
implementation, verified failing, then made to pass. Not optional for this project.

**Organization**: Grouped by user story. ⚠️ **Unlike the usual speckit assumption, these stories are NOT
mutually independent** — see [Dependencies](#dependencies--execution-order). Ordering below is
dependency-driven; priority indicates importance, not sequence.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable (different files, no dependency on incomplete work)
- **[Story]**: US1–US7 from spec.md

## Path Conventions

Existing C++17 codebase. Perception lives in `src/eval/` + `include/autoc/eval/`; production tracker tick in
`crrcsim/src/mod_inputdev/inputdev_autoc/`; tests in `tests/` registered via `run_autoc_tests`.

---

## Phase 1: Setup (Baseline Capture)

**Purpose**: establish the baselines that later gates measure against. Nothing here changes behaviour.

- [x] T001 ✅ **DONE 2026-07-28** — `scripts/rebuild.sh` clean: **339 tests pass, 0 fail**. The two `error` strings in the log are benign (a CMake probe for an optional dependency, and a test deliberately exercising a fail-loud path)
- [x] T002 [P] ✅ **DONE 2026-07-28** — baseline from `logs/autoc-038-t9-m2-spherical.log`: 430 gens / 26.88 h = **16.00 gen/hour**, late-run **≈5600 sims/s**. Recorded in `research.md` §R10 with the note that **sims/s at a comparable generation** is the honest comparator — gen/hour is config-sensitive and the run-mean (7359) is inflated by faster early gens. FR-038 ceiling ⇒ **≥ ~5040 sims/s**
- [x] T003 [P] ✅ **CLOSED 2026-07-28 by T021** — oracle confirmed live: `NN_EVAL_SAME` reproduced `-13949.366286`. Originally recorded in `research.md` §R11 as t9 gen 430. **PARTIAL note superseded** — oracle identified and recorded in `research.md` §R11: t9 **gen 430, fitness `-13949.366286`**, already corroborated by that run's own `NN_ELITE_SAME` line. ⚠️ **Remaining step is operator-driven**: running the eval against a rebuilt binary to confirm `NN_EVAL_SAME` is the eval-vs-training bitwise gate
- [x] T003a [P] ✅ **DONE 2026-07-28** — both M1 sources verified present and pinned `retain=keep`; prefixes and the near-miss recorded in `specs/040-camera-redo/research.md` §R9. The training source (`autoc-m1`) was already pinned; the **novel-path eval source (`autoc-eval`) was `retain=expire` and ~12 days from deletion** — it is the 038 t10 source the prior baseline's generalisation was measured against, so its loss would have silently destroyed the SC-008 comparison. The retrain MUST use these same sources (see T082a)

---

## Phase 2: User Story 1 — Airframe Fidelity Verdict (Priority: P1) 🚦 GATING

**Goal**: decide whether the simulated flight model must be regenerated before any perception work begins.

**Independent Test**: a reader can see every simulated-vs-measured airframe parameter classified with its
magnitude of disagreement, ending in an explicit recorded decision. Delivers value even if nothing else ships.

**Why first**: a "regenerate" verdict forces an M1 source rebake *before* perception work, since M2 trains
off the M1 source. No code — a document and a decision.

- [x] T004 [US1] ✅ **DONE 2026-07-28** — `specs/040-camera-redo/airframe-fidelity.md`. Compare simulated vs measured airframe/propulsion parameters (mass, span, wing area, propeller diameter/pitch, CG) in a new `specs/040-camera-redo/airframe-fidelity.md`, classifying each agree/disagree/unknown with magnitude, sourced from `crrcsim/models/hb1_streamer.xml` and [input-data-checklist.md](input-data-checklist.md)
- [x] T005 [US1] ✅ **DONE 2026-07-28 — verdict: DEFER** (operator). Recorded in `airframe-fidelity.md`. Reasoning of record: the propeller value and the tuned derivative set are one fitted system, so correcting it in isolation would break a fit made against observed flight. No M1 regeneration anywhere in this feature; the T003a pins remain SC-008's basis
- [x] T006 [US1] ✅ **DONE 2026-07-28** — propeller discrepancy filed to `specs/BACKLOG.md` against the flight-model re-tune (already gated on n>1 articles), carrying the do-not-fix-in-isolation warning and the eCalc wing-area caveat

**🚦 CHECKPOINT — HARD GATE: ✅ CLEARED 2026-07-28, verdict DEFER.** Phase 3 is open. Everything downstream
may assume the existing M1 source is valid, and no M1 regeneration occurs in this feature.

---

## Phase 3: Foundational (Blocking Prerequisites)

**Purpose**: single-source the per-tick rule, fix the unusable obstruction proxy, and land the config
surface — before anything depends on them.

**⚠️ CRITICAL**: no user story after this can begin until Phase 3 completes. This is also the **last
milestone with an objective pass/fail** — it must be behaviour-preserving, so bit-identity either holds or
there is a bug. Every later phase deliberately changes outputs.

### Tests (write first, verify failing)

- [x] T007 [P] ✅ **DONE 2026-07-28** — two reset-contract tests added to `tests/tracker_stepper_init_tests.cc`, asserting **bit-exact** (not near) reproduction of first-tick perception after re-init, including after a deep prior run that fills the observation ring. Written at the observable level so they keep guarding when the acquisition state machine lands (FR-020a)
- [x] T008 [P] ✅ **DONE 2026-07-28** — four obstruction tests in `beacon_projection_tests.cc`: mount-not-inside-any-primitive, forward-ray-not-self-occluded, **leading-edge mount clears the prop disc** (the design property), and prop attenuates-but-never-gates. They caught a real bug in my own first geometry, which placed the camera exactly on the wing's top face — the same degeneracy being removed
- [x] T009 [P] ✅ **DONE 2026-07-28** — asserts all 11 airframe keys are **present in all three tracker inis**, not merely defaulted in the struct. `AutocConfig` follows local convention with in-class defaults, so this test is what stops those defaults from ever being load-bearing (Constitution VII). `contract_config_tests` field count 98 → 109

### Implementation

- [x] T010 ✅ **DONE 2026-07-28** — `include/autoc/eval/tracker_tick_rule.h` + `src/eval/tracker_tick_rule.cc`: `projectPerceptionTick()`, `advanceSituationalAwareness()`, `resetPerceptionState()`. Extracted **verbatim** — no improvement rides along, which is what makes the T021 gate meaningful (FR-031)
- [x] T011 ✅ **DONE 2026-07-28** — production tick consumes the shared rule; inline duplicate removed, stale `derived_features.h` include dropped (Constitution III)
- [x] T012 ✅ **DONE 2026-07-28** — test-only reference consumes the same rule; inline duplicate and stale include removed. Both paths now reset through one call, so the Stage E acquisition state machine lands once
- [x] T013 [P] ✅ **DONE 2026-07-28** — three primitives; `buildAirframeObstruction()` places them from **config-supplied measured dimensions** (not hardcoded). Geometry anchored to the camera mount because the thrust line's body-frame position is still unmeasured (checklist A1b)
- [x] T014 ✅ **DONE 2026-07-28** — `AirframeProxy`, `rayHitsProxy` and `kAirframeOcclusionEnabled` **deleted outright**, no shim (Principle III). `WorkerInit.airframeProxy` → `airframeObstruction`; safe because that struct is RPC-only and never enters the persisted dmp
- [x] T015 [P] ✅ **DONE 2026-07-28** — 11 airframe keys added to both the `AutocConfig` struct and the `AUTOC_CONFIG_FIELDS` X-macro (they are separate lists; only adding to the macro fails to compile)
- [x] T016 ✅ **DONE 2026-07-28** (landed with T015 in `6d3dba7`, checkbox missed; verified 2026-07-29). `airframeObstructionFromConfig(cfg)` → `init.airframeObstruction` in `src/autoc.cc`. Principle VII holds at both levels: `WorkerInit.airframeObstruction` carries no in-class initializer, and `AirframeObstruction` itself declares **none on any geometry field** — so adding a primitive is a compile error at the construction site, not a silent zero-sized box that obstructs nothing and looks like it works
- [x] T017 [P] ✅ **DONE 2026-07-29** — `frame_rate_hz`, `latency_ms` and the `Projection` enum **deleted outright** (Principle III, no shim), along with the `CameraFrameRateHz` / `CameraLatencyMs` ini keys, their `AutocConfig` fields, their X-macro entries and their contract-test assertions. Each retirement has a recorded reason in the header: cadence is `ControlIntervalMsec` (20 Hz per 037), latency emerges from the US4 acquisition machine, and `Projection` was a two-value enum with one value ever constructed. Also cleared the stale `autoc-tracker.ini` comment still advertising the `kAirframeOcclusionEnabled` compile-time constant T014 deleted. `AUTOC_CONFIG_FIELDS` 116 → 114, with the 040 arithmetic added to the count-history comment. **Behaviour-neutral by construction — a tree-wide grep found no consumer of any of the three**; landed after the T021 gate run, which it therefore cannot invalidate
- [x] T018 [P] ✅ **DONE 2026-07-28** — documented block in all three tracker inis, carrying the units convention (inches; vertical measured UP from thrust = −z per `docs/COORDINATE_CONVENTIONS.md`) and the measured values
- [x] T019 ✅ **DONE 2026-07-28** — `tracker_tick_rule.cc` + `airframe_occlusion.cc` registered in `autoc_common` (crrcsim links that target, so both paths pick them up)

**Checkpoint gates**:

- [x] T020 ✅ **DONE 2026-07-28** — clean `scripts/rebuild-perf.sh` completed (operator). Full `ctest`: **39/39 suites pass, 0 fail**, including the two new reset-contract tests
- [x] T021 ✅ **PASSED 2026-07-28** — `NN_EVAL_SAME: fitness=-13949.366286`, exact match to the oracle across all 294 scenarios, exit 0. **The Stage-B extraction is provably behaviour-preserving.** Setup required aligning the eval ini to the t9 *training* shape (source→`autoc-m1` gen9200, Seed=1783621931, 6 paths × 49 winds); the ini carries inline `RESTORE TO …` notes for T085
- [x] T022 ✅ **DONE 2026-07-28** — grep run. **Zero violations in the 040 diff**: the three `tracker_tick_rule` hits are the words float/double in explanatory prose, not code. 448 tree-wide hits are the known pre-existing backlog the constitution defers to a one-time backfill spec (enforcement is incremental on touched code). Additionally annotated one pre-existing boundary cast in `tracker_stepper.cc` while in the file (comment-only, post-gate, cannot affect T021)

---

## Phase 4: User Story 2 — Honest Camera Geometry (Priority: P1) 🎯 MVP

**Goal**: bearing quantised on a real pixel grid, reported as isotropic angles, with the corrected physical
beacon separation.

**Independent Test**: project known geometries and confirm a fixed angular separation reads the same
anywhere in frame and at any orientation; bearings resolve no finer or coarser than the pixel grid; inferred
range matches truth.

### Tests (write first, verify failing)

All six written against the not-yet-existing API and confirmed failing to compile before implementation.
They live in a `CameraGridGeometry` suite in `tests/beacon_projection_tests.cc`.

- [x] T023 [P] [US2] ✅ **DONE 2026-07-29, then STRENGTHENED by T033a.** Originally split in two, because the single assertion the task describes was not true under the planar metric: `RadialSeparationIsPositionInvariant` asserted the delivered property (15.0° within 2% at frame centre and at 50° off-axis, both pairs on exact pixel centres so quantisation contributes **zero**), and `TangentialSeparationOverReadsByThetaOverSinTheta` pinned the accepted residual. **T033a made the task's original wording achievable**, so the residual test is gone and `SeparationIsInvariantAtAnyPositionAndOrientation` replaces it — a 4 × 5 sweep of field angle (0–40°) × pair orientation (radial→tangential) at fixed true separation, with an explicit teeth assertion that the retired planar metric lands *outside* the bound at 40° tangential, so the sweep cannot go slack and pass on either metric
- [x] T024 [P] [US2] ✅ **DONE 2026-07-29** — `SeparationIsUnchangedWhenRotatedHorizontalToVertical`, equal to 1e-4 rad. This is the assertion that actually retires the bug: the ±1 encoding normalised each axis by its own half-FOV and read these **33% apart**
- [x] T025 [P] [US2] ✅ **DONE 2026-07-29** — two tests. `BearingIsQuantisedToPixelCentres` sweeps ±50° and asserts every reported bearing lands on a pixel centre; `BearingResolvesNoFinerAndNoCoarserThanOnePixel` asserts sub-pixel detail does **not** survive (two truths inside one pixel are bit-identical) and that one pixel apart resolves as exactly one pixel
- [x] T026 [P] [US2] ✅ **DONE 2026-07-29** — `RangeFromSeparationMatchesTruthWithoutSystematicBias`, 5–25 m at 0.25 m steps. The mean-bias assertion is what catches a wrong constant: 0.9 m would sit ~16.6% high uniformly. **Found a real effect while tuning it** — a genuine ~1.1% positive bias remains, which is **grid convexity** (range is separation/span and E[1/span] > 1/E[span] under symmetric quantisation error; at 25 m the pair subtends only ~4.7 px). Documented in the test and the bound set above it
- [x] T027 [P] [US2] ✅ **DONE 2026-07-29 — already enforced, no new test needed.** `TrackerInput::COUNT == 58` and `sizeof(TrackerInputs) == 58 * sizeof(float)` are `static_assert`s in `include/autoc/nn/nn_inputs.h`, mirrored in `tests/gather_tracker_inputs_tests.cc`. FR-006 is a **compile-time** guarantee here, which is stronger than a runtime test; US2 changed the units in those slots, never the count
- [x] T028 [P] [US2] ✅ **DONE 2026-07-29** — `FieldOfViewIsDerivedFromGridAndPixelPitch` (halving the pitch halves both fields; there is no setter that could contradict it) plus `FovEdgeFollowsTheDerivedField` (the derived field is the real visibility boundary, not a separate knob that could drift)

> ✅ **SC-001 RESOLVED 2026-07-29 by fixing the metric, not the wording — see T033a.** The conflict was real:
> SC-001 claims 2% invariance "wherever it appears in frame **and at any orientation**", while research R2 and
> [contracts/perception-interface.md](contracts/perception-interface.md) §6 both accepted a worst-case tangential over-read (θ/sin θ) as
> documented-not-fixed, explicitly because it "makes a separate ray-angle span computation unnecessary".
>
> **Operator decision was to reopen R2** rather than scope SC-001 radially, on the grounds that the residual
> is position-dependent and sits in the sole range channel. T033a measures the pair on the sphere; SC-001 now
> holds as literally worded. R2, contract §6, and the test comments are amended. *Also corrected in passing:
> R2's "+21% at the frame corner" was θ/sin θ at the 60° horizontal edge — the true 75° diagonal is +35%.*

### Implementation

- [x] T029 [US2] ✅ **DONE 2026-07-29** — `pixels_h` / `pixels_v` / `deg_per_px` with `fovHDeg()` / `fovVDeg()` / `halfFov*Rad()` / `radPerPx()` as **accessors, not fields**, so no setter exists that could contradict the grid. `fov_h_deg` / `fov_v_deg` deleted along with the `CameraFOVHorizontalDeg` / `CameraFOVVerticalDeg` ini keys; `CameraPixelsH` / `CameraPixelsV` / `CameraDegPerPixel` replace them in all three tracker inis. `AUTOC_CONFIG_FIELDS` 114 → 115
- [x] T030 [P] [US2] ✅ **DONE 2026-07-29** — `kBeaconSeparationM = 0.772` / `kBeaconMountY = 0.386` in `beacon_config.h`, with the mounts derived from the constant so the two can never disagree. Inis updated. Added `TrackerInisCarryTheMeasuredBeaconSeparation`, which asserts the **value** (right − left == 0.772) in all three inis, not merely the key's presence
- [x] T031 [US2] ✅ **DONE 2026-07-29** — projection emits isotropic radians and quantises on the grid. Pixel centres at `(i − (n−1)/2)·rad_per_px`, so the sensor edges land at `±n/2·rad_per_px` and the derived FOV is exactly `n × deg_per_px`. **With an even pixel count the boresight falls on a pixel BOUNDARY**, so a perfectly centred beacon reports half a pixel and never exactly zero — that is what an even-width sensor does, and it is preserved rather than papered over
- [x] T032 [US2] ✅ **DONE 2026-07-29** — int8 bearing encoding **deleted outright** (`quantize_xy` / `dequantize_xy` and their round-trip tests). `int16_t raw_px_x` / `raw_px_y` replace `raw_x_int8` / `raw_y_int8`, with `kPixelSentinel = INT16_MIN` (valid indices are never negative). CEP keeps its int8 encoding until US4 replaces it with signal-derived quality
- [x] T033 [US2] ✅ **DONE 2026-07-29** — **units changed, math did not.** `compute_pair_span` / `compute_tilt` were always pure planar geometry over whatever the caller passed, so no formula moved; what changed is what they MEAN, and every comment claiming NDC was corrected across `derived_features.h`, `nn_inputs.h`, `evaluator.h`, `evaluator.cc`. `kTiltDegenerateEpsilon` re-justified in radians (1e-4 rad ≈ 1.5% of a pixel) — and grid quantisation makes it sharper, since two beacons in one pixel now yield a distance of exactly zero. Tilt stays sin/cos per FR-005
- [x] T034 [US2] ✅ **DONE 2026-07-29** — POV panel normalises bearings against the recorded half-field. **This is why `screen_x` was renamed rather than reinterpreted**: reading a radian value as if it were ±1 NDC would have silently pinned every beacon near frame centre, and the rename made every one of the ~14 consumer sites a compile error instead
- [x] T035 [US2] ✅ **DONE 2026-07-29** — grep run on the diff. Production code is clean bar the `X(double, cameraDegPerPixel, ...)` macro token, which must match its struct field and is uniform with all 114 other entries. `cameraDegPerPixel` annotated per the `cepGateThreshold` precedent (inih returns double, cast at the WorkerInit boundary). Test scaffolding carries a block annotation explaining that raw `double` there is deliberate: the reference geometry must out-precision the code under test, or a 2%-of-a-pixel tolerance partly measures the test

- [x] T033a [US2] ✅ **DONE 2026-07-29 — added after review, on the operator's call to reopen R2 rather than reword SC-001.** Span is measured **on the sphere**: `bearing_to_unit_ray` reconstructs each ray (the exact inverse of the forward projection, so no information is added or lost) and `compute_pair_span` returns the angle between them. Exactly position- and orientation-invariant by construction, retiring the θ/sin θ tangential over-read from the sole range channel. **`2·asin(chord/2)`, not `acos(dot)`** — `gp_scalar` is float and at 25 m the dot product sits at 0.99952, where `acos` sheds half its digits. **Blast radius was one function body**: all four call sites (tick rule, `fitness_decomposition`, `dmp_dump`, and the crrcsim mirror via the shared tick rule) share the 4-arg signature, and the NN input layout is untouched — 4 bearing slots, `COUNT == 58`, FR-006 intact. **Tilt deliberately left planar** (direction residual, not magnitude; no global horizontal exists on the sphere). Range inference in the test moved to the exact `(L/2)/tan(ψ/2)`. `sepDeg` in the test now calls the production function instead of reimplementing the metric — a duplicate is how a metric change passes its own tests. ⚠️ **Breaks bit-identity; the determinism baseline needs re-establishing before the Phase 9 retrain** (free to do now, a second retrain if deferred)

**Gate**: `cmake --build build` clean, **39/39 ctest suites pass, 0 fail**, crrcsim links clean. Re-confirmed
after T033a: 39/39, and both the autoc tick-rule object and the crrcsim mirror object rebuilt against the
amended header.

**Checkpoint**: **manual renderer check** — load a playback and confirm beacons render where expected under
the new scale. Automated tests cannot catch a scale error that is self-consistent.

---

## Phase 5: User Story 3 — Prove the Mount Clears Obstructions (Priority: P2)

**Goal**: model wing, nose and propeller obstruction from measured geometry; publish the *effective* field
and the distribution of obstruction onset across the mounting-error envelope.

**Independent Test**: sweep a target through the field at nominal and across the error envelope; the
reported obstruction-onset distribution matches geometric prediction and the wing contributes nothing.

### Tests (write first, verify failing)

- [x] T036 [P] [US3] ✅ **DONE 2026-07-29** — `WingContributesNoObstructionAtBaselineMount` sweeps the whole field (4800 samples) rather than arguing it: the aperture stands 2 mm FORWARD of the LE plane, so for any forward ray the wing is strictly behind it. **Zero of 4800 field rays hit the wing slab.** The claim the LE mount's build cost rests on, so it is swept, not reasoned
- [x] T037 [P] [US3] ✅ **DONE 2026-07-29** — two tests. `PropShadowOnsetMatchesGeometricPrediction` compares the swept onset against the closed form `atan((r_cam − r_prop)/Δx)`: **42.10° measured vs 42.08° predicted**. Scoping said ≈41°. `MisalignedMountBringsTheShadowInboard` covers the envelope case — a 20° glue error does not move the airframe, it rotates the boresight, so the same shadow arrives **22.1°** from the camera's own boresight (spec predicted ≈21°)
- [x] T038 [P] [US3] ✅ **DONE 2026-07-29** — `AlternativeMountReportsDifferentObstruction` prices a wing-top mount by changing configuration only. Asserts both that the answer *differs* and the *direction* of the difference (wing-top is obstructed more), because a test that only asserts inequality would pass on a bug that made every mount look bad
- [x] T039 [P] [US3] ✅ **ALREADY SATISFIED — no new test written.** `PropDiscAttenuatesButNeverGates` has covered FR-009 since T013: crossing the disc yields `blocked == false` with `0 < attenuation < 1`. Verified against the new baseline mount rather than re-asserted
- [x] T040 [P] [US3] ✅ **DONE 2026-07-29 — and it FAILED FIRST, correctly.** I wrote it asserting `blockedFraction == 0`; the sweep reported **2.9%**. The test was wrong, not the code: the wing is eliminated but **the pod nose still shadows the inboard field** from ~48° out, exactly as [input-data-checklist.md](input-data-checklist.md) predicted ("wing occlusion is eliminated at any LE offset; the pod nose shadows the same inboard region and merges into the same patch"). Test now asserts the true structure **with attribution** — collapse the nose box and blockage must vanish entirely, so the 2.9% cannot be quietly ascribed to the wing. Also pins that the integrator integrates the SPHERE (2.989 sr) and not the flat rectangle (3.29 sr)

### Implementation

- [x] T041 [US3] ✅ **ALREADY IMPLEMENTED at T013** — `rayHitsBox` (slab method over the segment) plus the measured station stack in `hb1AirframeObstruction()`. Verified against the new mount; no change needed
- [x] T042 [US3] ✅ **ALREADY IMPLEMENTED at T013** — `rayCrossesPropDisc` + partial attenuation, no engine speed and no blade phase. Verified; no change needed
- [x] T043 [US3] ✅ **DONE 2026-07-29 — the switch this whole story exists to justify.** Mount moved to `(-0.150400, +0.203200, -0.031750)` and **obstruction turned ON**, in FOUR places that could otherwise drift: `CameraConfig`'s default, `AutocConfig`'s default, `hb1AirframeObstruction().enabled`, and all **three** tracker inis. The retired `(0, 0, -0.05)` sat on the centreline 5 cm above the axle — **inside** the 6.985 cm prop radius, i.e. squarely behind the disc — which is why obstruction had to ship disabled. Added `ConfigDefaultMountMatchesTheMeasuredLeadingEdgeMount` to pin the two code copies equal (they cannot be single-sourced without `camera_config.h` depending on the obstruction header). Two existing tests had to change with it: the "legacy mount" contrast now writes the retired value literally (reading it from config would have compared the new mount against itself), and `DisabledObstructionNeverOccludes` now disables explicitly so it tests the switch rather than the shipped default
- [x] T044 [US3] ✅ **DONE 2026-07-29** — `computeEffectiveField` in `camera_projection.cc`, reported as **solid angle, deliberately**: under an equidistant mapping equal areas in (θx, θy) do NOT subtend equal solid angle (Jacobian sin θ/θ), and an unweighted sample count would over-weight the frame corners — which is precisely where obstruction lives, so it would have flattered or damned the mount for the wrong reason. Blocked and attenuated are reported **separately, never summed**: the prop attenuates rather than blocks (FR-009), so its share is not lost field. Calls `testObstruction` rather than reimplementing it, so the report cannot drift from what the simulator does
- [x] T045 [US3] ✅ **DONE 2026-07-29** — `dmp-dump --obstruction-report -i <ini>`. **Takes no dmp argument**, because onset is a property of the configuration and not of any run; handled before the positional-argument check. The mounting-error envelope is **ASSUMED and stated in the output itself**, not buried: half-normal σ=6° clipped at 20°, swept deterministically rather than sampled so the report is reproducible. Percentiles are read from the PESSIMISTIC end, so "p95" means "95% of mounts obstruct no closer in than this"
- [x] T046 [US3] ✅ **DONE 2026-07-29** — research **R13**, consolidated out of the checklist §C that produced it so it survives independently. Records `f_bp = RPM/30` (261–679 Hz in level flight), the three commensurabilities, and the key asymmetry: **exact lock is benign** (fixed phase ⇒ uniform attenuation ⇒ absorbed by the energy-normalised AGC) while **the near-miss is the hazard** (phase walks a full cycle within one 75 ms code word ⇒ structured attenuation ramp; danger band 14 010–14 790 rpm ⇒ 12.4–13.2 m/s, which sits on autoc cruise). Notes that the checklist's independent ~19% duty estimate lands on the shipped `AirframePropAttenuation = 0.18` — but that this is **not** independent confirmation, since both rest on the same unmeasured pupil and chord. Also records that **throttle is an NN output**, so this is a closed loop rather than an external disturbance
- [x] T047 [US3] ✅ **DONE 2026-07-29** — grep clean. Three `cameraMountOffset*` config fields annotated per the `cameraDegPerPixel` precedent (inih returns double, cast at the WorkerInit boundary); the report-local statistics in `dmp_dump.cc` annotated as such — they summarise gp_scalar geometry and must out-precision it. **Cleanup**: deleted the dead `in()` inches→metres helper in `airframe_occlusion.cc`, defined at T013 and never called (the constants were written out in metres directly), so it was a second unused way to say the same thing

**Gate**: `cmake --build build` clean, **39/39 ctest suites pass, 0 fail**, Principle VI grep clean.

**Checkpoint**: **read the obstruction-onset distribution.** Per the "let it ride" clarification this is an
observation, not a pass/fail — but it is the milestone's actual output. Produced 2026-07-29 by
`./build/dmp-dump --obstruction-report -i autoc-tracker.ini`:

```
effective_field:            onset_deg_from_own_boresight:
  nominal_sr:      2.98916    nominal:          42.10   (scoping predicted ~41)
  clear_frac:      0.9708     median:           38.10
  blocked_frac:    0.0292     p95:              30.35
  attenuated_frac: 0.0220     clipped_extreme:  22.10   (spec predicted ~21)
```

**Reading it — three things, and the second is the finding.** (1) Onset lands where the geometry said it
would, at nominal *and* at the clipped extreme, so the mount decision rests on arithmetic that now
reproduces. (2) **The 2.9% that is hard-blocked is the pod NOSE, not the wing** — wing obstruction really is
eliminated at any LE offset, but the nose shadows the inboard field from ~48° out, and the nose box is
**ASSUMED geometry pending the pod measurement** (checklist A1b), so this number is the least trustworthy
one here and is deliberately modelled to under-obstruct rather than over-. (3) Even at the 20° glue-error
limit the shadow stays 22° off boresight, which is the "let it ride" answer justified: no threshold was
needed because the envelope never brings obstruction near where a tail-chased target lives.

> ⚠️ **US3 acceptance scenario 6 completes in Phase 6, not here.** "Given a brief obstruction, when it
> clears, then recovery follows the measured ride-through" (FR-013) requires the acquisition hold machinery,
> which Phase 6 builds. Tasks T056a and T064a carry it, labelled `[US4]` because they live in the state
> machine. **US3 is therefore not fully verifiable until Phase 6 completes** — the one genuine cross-story
> dependency in this feature.

---

## Phase 6: User Story 4 — Signal Quality That Means Something (Priority: P2)

**Goal**: replace the position-only CEP placeholder with a link budget and an acquisition state machine, so
confidence degrades with distance, aspect and interference, and takes real time to establish.

**Independent Test**: hold a beacon at a series of ranges and aspects and confirm confidence degrades
monotonically per the measured bench relationship; interrupt the signal and confirm re-establishment timing.

### Tests (write first, verify failing)

- [x] T048 [P] [US4] ✅ **DONE 2026-07-30** — `SignalFallsMonotonicallyWithRange` sweeps 2-120 m, plus `RangeFalloffIsInverseSquare`, which asserts doubling range costs exactly 6.02 dB. That second one is what distinguishes a real 1/r² term from a hand-tuned ramp that merely decreases. `QualityGradientSpansTheAssertedDetectionEnvelope` carries SC-004's actual content: the gradient must be USEFUL across the envelope, not saturated or floored throughout — a model can be perfectly monotonic and carry no information — orig: signal falls monotonically with range and the loss-of-tracking point lands inside the measured band (FR-015, SC-004)
- [x] T049 [P] [US4] ✅ **DONE 2026-07-30** — flat to ±45°, exactly 0.5 at ±75°, exactly 0 at ±105°. `EmissionIsFlatTopNotCosinePowerLaw` **builds the competing cos^m law fitted to the same half-power angle** and asserts we sit >0.10 above it in the flat region — asserting only "flat to 45°" would pass on a cosine law with a large enough exponent, leaving FR-019 decorative. **Also found a real modelling error while writing it**: a single outboard emission axis puts the TAIL-CHASE direction at 90° off beam, deep in the skirt, when the enclosure is a cube-minus-base with an aft face pointed straight down the chase's throat. Modelling one axis would have been wrong in the one geometry M2 spends all its time in. `FiveFaceEnclosureIlluminatesTheTailChaseDirection` pins it at >4× the single-axis answer — orig: emission follows a flat-top profile — near-constant to ±45°, half power at ±75° — and is **not** a cosine power law (FR-019)
- [x] T050 [P] [US4] ✅ **DONE 2026-07-30** — `SharedDetectorElementDegradesButNeverGates` asserts the cost is EXACTLY `cdma_penalty_db`, not merely non-zero, and that `q > 0` survives. FR-016 is emphatic this configuration is field-proven — orig: when both beacons share a detector element, detection and code identification survive with only the measured penalty (FR-016 — this configuration is field-proven, not a failure)
- [x] T051 [P] [US4] ✅ **DONE 2026-07-30** — cold 7 ticks (308 ms), warm 4 (154 ms), both ±1 per SC-005, plus `ReacquisitionBeyondTheCoastWindowIsTrueCold`. `WarmRelockIsCheaperThanCold` asserts warm < cold STRICTLY — if that ever reads equal the coast-window design has been silently defeated, and it is the single highest-value feature in the model — orig: acquisition completes within the measured window, including partial-code early lock (FR-017, SC-005)
- [x] T052 [P] [US4] ✅ **DONE 2026-07-30** — quality large on the first tentative tick, monotonically improving, never the sentinel. Plus `UnresolvedIdentityInflatesQuality` for FR-017d — orig: a tentative lock reports a bearing with large-variance quality, improving toward confirmed (FR-017a)
- [x] T053 [P] [US4] ✅ **DONE 2026-07-30** — plus the complement `InterruptionLongerThanTheHoldWindowDropsToSearching`, without which the first test would pass on a machine that never leaves HOLDING — orig: an interruption shorter than the hold window never loses tracking (FR-018, SC-005)
- [x] T054 [P] [US4] ✅ **DONE 2026-07-30** — two machines driven through a deliberately awkward 30-tick sequence (partial acquisitions, holds that expire, holds that do not, re-locks on both budgets), every field compared bit-exact each tick — orig: identical inputs produce bit-identical outputs; no PRNG in the signal path (FR-020, SC-006)
- [x] T055 [P] [US4] ✅ **DONE 2026-07-30** — two tests. Field-level reset-to-fresh, and the OBSERVABLE form mirroring T007: a recycled scenario must acquire **cold**, exactly as a virgin one does. A machine that reset its fields but leaked a warm coast passes the first and fails the second — orig: all carried state resets at scenario boundaries in both execution paths (FR-020a)
- [x] T056 [P] [US4] ✅ **DONE 2026-07-30** — a five-test `TwoEnvelopes` suite. **The crossover test found something worth keeping**: the naive continuous prediction (5 px ⇒ 23.6 m) is WRONG — measured is ~27.75 m — for two real reasons. (1) Quantisation rounds OUTWARD for a boresight-straddling pair: with an even pixel count the boresight sits on a pixel BOUNDARY, so a true 4.25 px gap reads as exactly 5. (2) **The camera is not on the centreline** — 8″ outboard, ~1¼″ up — so the pair does not straddle symmetrically. The test therefore asserts the INVARIANT (the crossover is exactly where the *quantised* gap reaches the configured limit, and one step out it is not) rather than a derived range, so it stays tied to the grid. — orig:  bearing reaches the design detection range while separation-derived range goes explicitly unavailable below the resolving limit (FR-033, SC-011)
- [x] T056a [P] [US4] ✅ **DONE 2026-07-30** — `ClearedObstructionReEstablishesViaRideThroughNotInstantly`. Guards the tempting shortcut of restoring the previous lock the instant geometry clears because "we know it is the same beacon" — orig: when a **brief obstruction** clears, recovery follows the measured ride-through rather than restoring instantaneously — distinct from T053, which covers signal loss (FR-013, US3 acceptance scenario 6)

### Implementation

- [x] T057 [P] [US4] ✅ **DONE 2026-07-30** — `signal_model.{h,cc}`. **The header carries the calibration-honesty note that matters most in the whole phase**: `ambient_floor + noise_floor` is **back-solved**, not measured, so 0 dB lands exactly at the asserted detection range. Taking the measured 031 decode floor (≤10 nA) literally would end detection at ~5 m and leave every longer range equally at the floor — monotonic and completely uninformative. FR-033a resolves it by asserting the envelope and letting the budget shape quality *within* it. `NoiseFloorIsCoherentWithTheAssertedDetectionRange` pins the coupling, because in the ini the two values look independent — orig: Create `include/autoc/eval/signal_model.h` + `src/eval/signal_model.cc` — drive × emission × 1/r² × obstruction ÷ ambient → per-chip SNR, per [data-model.md](data-model.md) §4
- [x] T058 [US4] ✅ **DONE 2026-07-30** — flat-top with a raised-cosine shoulder replaces the hard 270° cone, which is **deleted** (Principle III). The only thing that gates now is genuine darkness — no face still illuminating that direction — which is physics rather than an arbitrary cone — orig: Implement the flat-top emission profile in `src/eval/signal_model.cc`, replacing the hard 270° cone in `include/autoc/eval/beacon_config.h` (FR-019)
- [x] T059 [US4] ✅ **DONE 2026-07-30** — applied in `projectPerceptionTick`, not `projectBeacon`, because it needs BOTH beacons' pixel indices. Degrades, never gates — orig: Implement the detector-sharing penalty in `src/eval/signal_model.cc` — degrades quality, never gates detection (FR-016)
- [x] T060 [P] [US4] ✅ **DONE 2026-07-30** — `acquisition_state.{h,cc}`, mirroring the shipped gateware FSM. **One design correction landed during wiring**: quality keys off a new `blob_present` flag, NOT off `state`. HOLDING with signal RETURNED is re-integrating on a blob that is plainly there and must report a tentative bearing (FR-017c); HOLDING while still blind has nothing to report. Keying off `state` alone conflates the two — orig: Create `include/autoc/eval/acquisition_state.h` + `src/eval/acquisition_state.cc` — chip-credit integrator and four-state machine per [data-model.md](data-model.md) §5, deterministic thresholds not draws
- [x] T061 [US4] ✅ **DONE 2026-07-30** — the `0.3 × max(|x|,|y|)` edge factor is gone. **Two existing tests asserted the retired semantic** (`cep > 0.20` at the frame edge, "edge_factor near 1 ⇒ cep near 0.3") and now assert the REPLACEMENT property: frame position no longer drives quality, because position is not a signal term — orig: Derive quality from signal and lock state in `src/eval/camera_projection.cc`, replacing the linear edge-factor placeholder (FR-014)
- [x] T062 [US4] ✅ **DONE 2026-07-30** — `LockState` reaches the dmp as a diagnostic and never the NN. `COUNT == 58` static_asserts untouched — orig: Keep the state machine internal — expose only bearing and quality; record state for diagnostics only (FR-017b, [contracts/perception-interface.md](contracts/perception-interface.md))
- [x] T063 [US4] ✅ **DONE 2026-07-30** — resolving-limit gate beside the existing CEP gate. Noted while wiring: the shipped `CepGateThreshold` (1.25) is **exactly** `kCepSentinelThreshold`, so that gate is already a VISIBILITY gate and stays one under 040's quality semantics — which is what keeps it compatible with FR-017c. Lowering it below 1.0 would start suppressing tentative locks and break FR-017c; the header now says so — orig: Implement the two-envelope rule in `src/eval/camera_projection.cc`: bearing to design range, separation-derived range unavailable below the resolving limit (FR-033)
- [x] T064 [US4] ✅ **DONE 2026-07-30** — `PerceptionCarryState` added to `resetPerceptionState`, so both execution paths inherited it **without either being edited** — which is exactly what T010 built the single-source rule for — orig: Wire per-scenario reset of all acquisition state into `src/eval/tracker_tick_rule.cc` so both paths reset identically (FR-020a)
- [x] T064a [US4] ✅ **DONE 2026-07-30** — obstruction and signal loss enter the same hold machinery by construction, since `signal_present` is geometric visibility and obstruction zeroes it — orig: Route obstruction state through the acquisition hold path in `src/eval/acquisition_state.cc` so a cleared obstruction re-establishes via ride-through, not instantaneous restoration (FR-013). **Note the asymmetry**: an obstruction clears deterministically by geometry while a signal returns by SNR — both must enter the same hold machinery, but only obstruction has a knowable clear-time
- [x] T065 [US4] ✅ **DONE 2026-07-30** — grep clean. The only hits are the 19 `X(double, ...)` macro tokens, which must match their struct field type and are uniform with the other 115 entries (the `cameraDegPerPixel` precedent), plus one test-scaffolding `double`. Annotated as a block in `config.h` on the `cepGateThreshold` precedent — orig: Run the Principle VI type-domain grep on the diff; annotate or convert

### Playback controls — a Stage E DEPENDENCY, not renderer polish

> **Why these sit in Phase 6 rather than Phase 10.** The checkpoint below asks whether dropout and
> reacquisition *look physical*. It is not answerable at realtime playback: a **warm relock is 154 ms (3
> ticks)** and a **cold acquire is 308 ms (6 ticks)**, so the entire warm-vs-cold distinction — the most
> behaviour-defining thing in the model, the thing the whole coast-window design turns on — plays out in
> under a third of a second. At 20 Hz realtime both are "it blinked". The tentative→confident quality ramp
> spans the same ~3 ticks. **Single-step is what makes the checkpoint answerable at all**, which is why
> these unpark from the backlog into 040 rather than staying deferred.
>
> Backstep is cheap here: playback is *recorded* data, so a step back is an index decrement. The
> [hidden-state scrubbing](../BACKLOG.md) problem only bites a renderer that re-runs the NN, which this
> one does not.

- [x] T065a [P] ✅ **DONE 2026-07-30** — transport in `tools/renderer.h` (`CustomInteractorStyle::OnChar`) + `tools/renderer.cc` — a manual tick cursor replacing wall-clock-only playback, on the agreed bindings:

  | key | action |
  |---|---|
  | `Space` | play / pause toggle |
  | `.` / `,` | step forward / back one tick |
  | `>` / `<` | step ±10 ticks |
  | `Home` / `End` | jump to start / end |
  | `F` | rapid-finish — **moved off `Space`** |
  | `[` / `]` | jump to previous / next **event** (dropout, relock, hull strike) |

  `Space` becoming play/pause is the conventional binding and worth the small break from current
  behaviour. **Event-jump earns its place**: stepping 900 ticks to hunt a dropout is worse than not
  having the feature.

- [x] T065b [P] ✅ **DONE 2026-07-30** — tick index + `||` pause marker on the stopwatch readout. Tick length is **derived from consecutive recorded timestamps**, not assumed, so one step is always exactly one recorded sample however the run was configured — and the displayed index is countable against the model's own units (warm relock 3, cold 6, HOLDMAX 6)
- [x] T065c [P] ✅ **DONE 2026-07-30** — all three updated. *Renderer Playback Enhancements* and *Renderer scrub controls* closed against T065a; *Renderer scrubbing with hidden state* **narrowed rather than closed**, because the genuinely hard case it was about — scrubbing a view that RE-RUNS the network, where RNN hidden state cannot be rewound — is untouched by this work. Was: unpark the three superseded backlog entries (*Renderer Playback Enhancements*, *Renderer scrub controls*, *Renderer scrubbing with hidden state*) to point at T065a rather than describing the work a fourth time (Principle X)

### Perception rendering — the parallax layer (extends T088)

> **The finding this exists to make visible.** The camera sits 8″ outboard and ~1¼″ above the thrust line,
> boresight parallel to it. So the thrust axis — where the propeller actually goes, i.e. **where the
> streamer gets cut** — projects left and slightly below image centre by `atan(r_cam/d)`. Both that offset
> and the beacon separation scale as 1/d, so the parallax is a **constant 26.6% of the target's apparent
> wingspan at every range** (`r_cam/W = 0.205/0.772`):
>
> | range | beacon separation | aim offset | offset ÷ separation |
> |---|---:|---:|---:|
> | 3 m | 14.7° | 3.91° | **26.6%** |
> | 10 m | 4.42° | 1.17° | **26.6%** |
> | 25 m | 1.77° | 0.47° | **26.6%** |
> | 100 m | 0.44° | 0.12° | **26.6%** |
>
> **You cannot close your way out of it.** Anyone reading the FPV and aiming at image centre mis-aims by a
> quarter wingspan, always. This is arguably safety-relevant display scope, and it is not in the spec today.

- [x] T065d [P] ✅ **DONE 2026-07-30** — rings in `updateCameraPOVMiniPanel` — boresight cross at image centre (thin, neutral: that is the camera axis), the **thrust-axis locus** as a short track from ≈(−0.065, +0.008) NDC at 3 m to (0,0) at infinity, and **range rings centred on that locus, not on image centre**, diameter = expected beacon separation, labelled 3 / 10 / 25 / 100 m. Match the observed pair to a ring ⇒ read range; pair vs ring centre ⇒ read aiming error; the rings' migration off-centre ⇒ the parallax made visible. At 100 m the ring is ~1.2 px across, which is itself an honest statement about the resolution floor
- [x] T065e [P] ✅ **DONE 2026-07-30** — a **vertical member** through the aim point at the same angular scale as the rings. Beacon separation is a roughly *horizontal* measurement and says nothing about vertical displacement; for a tail chase closing on a streamer, up/down is the axis the display is least instrumented on
- [x] T065f [P] ✅ **DONE 2026-07-30** — quality rendering beyond radius — **Gaussian falloff, not a hard disc** (a hard edge implies a bound; CEP is a distribution, so σ ∝ CEP on an alpha-gradient sprite), **alpha = confidence** so bad data visually recedes, and **colour = lock state**: confident saturated/sharp · tentative amber/soft · **HOLD a hollow dashed ring at last-known, fading across the hold window** · searching absent. HOLD is the one worth building — you watch it hold, then either snap back solid (warm, ~3 ticks) or expire (~6 ticks) and vanish, which *is* the checkpoint question
- [x] T065g [P] ✅ **DONE 2026-07-30** — a small `q` bar (0–9, GOOD ≥ 5 marked) per blob. It is the **hardware's own metric**, so a bench capture and a sim playback can be put side by side and compared on the same number

> ✅ **Dependency discharged 2026-07-30.** T089/T090 were pulled forward as planned and `lock_state` +
> `raw_margin` now reach `CameraViewSample`, which is what let T065f draw the HOLD coast at all. The
> argument held up: without those fields the renderer could not show the one thing the checkpoint asks
> about.

**Checkpoint**: **playback review.** Does dropout and reacquisition look physical? Is the range-envelope
crossover where expected? This is where "passes its tests" and "physically plausible" most easily diverge.
**Answerable only with T065a in hand** — see the note above.

---

## Phase 7: User Story 6 — Per-Scenario Camera Variation (Priority: P3)

**Goal**: each scenario draws its own camera imperfections, so the controller cannot depend on a perfectly
known camera.

**Independent Test**: scenarios draw distinct camera parameters; the same scenario id always reproduces the
same draw; zero sigmas reproduce the baseline bit-identically.

> 🎯 **SCOPE DECISION 2026-08-02 (operator): CAMERA variation only — the emitter
> stays PERFECT.** US6 varies the camera's *mechanical and optical* imperfections
> (boresight, roll, mount translation, wing thickness) and holds the emitter /
> ambient side at nominal. `SignalAmbientKnee` stays in the model as the fidelity
> fix 031 field test #4 forced, but its **variation sigma stays at zero** for this
> pass. Two reasons, and the second is the stronger:
>
> 1. **The knee is not pinned yet.** The lens + bandpass field tests (~week of
>    2026-08-03) are expected to fix it. Varying ambient now would bake an assumed
>    compression curve into a training run and then have to be redone.
> 2. **It keeps t2 attributable.** With ambient held, `t2 − t1` is *camera
>    variation* and nothing else. Vary both and the delta confounds a mechanical
>    robustness change with a signal-model change — and 038's whole lesson was
>    that unattributable deltas cost a run.
>
> Ambient variation is therefore deferred, not dropped, and its trigger is the
> filter field data. Stated here so a later reader does not read the zero sigma
> as an oversight.

### Tests (write first, verify failing)

- [x] T066 [P] [US6] ✅ **DONE 2026-08-02** — `DifferentScenariosDrawDifferentCameras` plus `DrawsStayWithinTheTwoPointFiveSigmaEnvelope` two scenarios draw different camera parameters within configured bounds (FR-021)
- [x] T067 [P] [US6] ✅ **DONE 2026-08-02** — `SameScenarioSeedReproducesTheDrawBitExactly`, every field, bit-exact the same scenario id reproduces identical draws across evaluations (FR-022)
- [x] T068 [P] [US6] ✅ **DONE 2026-08-02** — `ZeroSigmaProducesExactlyTheNominalCamera` over 200 seeds, plus `NominalDrawLeavesTheTickConfigExactlyUntouched` one level up at the tick config. **The load-bearing pair**: together they are what lets a variation-off run be compared to t1 without an asterisk with all sigmas zero, results are bit-identical to the no-variation baseline (FR-023, SC-006)
- [x] T069 [P] [US6] ✅ **DONE 2026-08-02 — and REWRITTEN after operator review.** First cut invented `kCameraAlignmentHardClipDeg` / `kCameraMountTranslationHardClipM`; operator: *"ALL the other variations work fine — so unless you are doing exactly the same thing, this feels a little too clever."* Correct. `ClassPRNG::nextGaussian` **already** truncates at `kGaussianSigmaClamp` (2.5σ), and the convention is to set sigma so 2.5σ IS the limit — `entryConeSigma = 18.0 // 2.5sigma = 45 deg` documents it inline. Envelope now expressed as **sigma 8.0 ⇒ 20°** and **0.002 m ⇒ 5 mm**, with no second constant to disagree. `DrawsAreTruncatedNotResampled` detects truncation by the pile-up exactly on the boundary, and states the reason: resampling would change the PRNG draw COUNT and break the frozen draw-order contract draws are hard-clipped at 20°, never tail-sampled beyond it
- [x] T070 [P] [US6] ✅ **DONE 2026-08-02** — `CameraDrawIsIndependentOfTheOtherVariationClasses`. Also asserts camera stays the **last-derived** sub-seed, since inserting a class before it would silently change every prior bake's camera camera variation stays chase-specific even when environment seeds are shared with the target (FR-022)

### Implementation

- [x] T071 [US6] ✅ **DONE 2026-08-02** — the `camera` slot reserved since 033 is finally consumed. Draw-and-discard discipline matches craft: the PRNG is always advanced, so toggling `EnableCameraVariations` cannot shift any other class's draws — orig: Consume the reserved `camera` sub-seed (slot 5) in `include/autoc/util/scenario_prng.h` — the slot is already seeded and the class order frozen
- [x] T072 [US6] ✅ **DONE 2026-08-02** — appended after `craftServoPwmPhase`, raw pre-scale, no version bump (greenfield growth). Round-trip test included — orig: Add `cameraSeed` and the variation draws to `include/autoc/rpc/scenario_metadata.h`, following the craft-variation pattern (raw pre-scale draws recorded)
- [x] T073 [US6] ✅ **DONE 2026-08-02** — intrinsic yaw·pitch·roll onto `mount_orientation_body`. `RollRotatesTheCameraButBoresightOnlyOffsetsIt` pins the asymmetry the axis split rests on: roll leaves the optical axis put and rotates the image plane (the tilt bias), yaw does the reverse — orig: Apply boresight and roll error (σ 10°, hard clip 20°) to the camera pose in `src/eval/camera_projection.cc`
- [x] T074 [US6] ✅ **DONE 2026-08-02** — `ProjectionInput` gained a separate `obstruction_mount_chase_body`, so the split is structural rather than a convention someone could tidy away. `MountTranslationMovesObstructionOnlyNotBearing` guards it. ⚠️ The new field has no in-class default (Constitution VII), so an unset fixture is **silent garbage, not a compile error** — `beacon_projection_tests` caught it on the first run — orig: Apply mount translation (1 cm box) in `src/eval/airframe_occlusion.cc` — **obstruction path only**, deliberately not applied to bearing in `src/eval/camera_projection.cc`: ±5 mm is 0.03° at 10 m (negligible) but swings propeller clearance ~15% (research R6)
- [x] T075 [US6] ✅ **DONE 2026-08-02** — wing thickness grows the slab's underside downward, the direction that can intrude on a leading-edge mount's view. **Ambient variation plumbed but held at zero sigma** per the scope decision; `AmbientIsDrawnButHeldNominal` asserts BOTH that it is held AND that the plumbing works when switched on — orig: Apply wing-thickness variation to `src/eval/airframe_occlusion.cc`. **Ambient-level variation is DEFERRED per the scope decision above** — plumb the draw so it exists and is recorded, but ship its sigma at zero; the trigger to switch it on is the lens+filter field data pinning `SignalAmbientKnee`
- [x] T076 [US6] ✅ **DONE 2026-08-02** — `dmp-dump --meta-only` emits the raw pre-scale camera block per scenario — orig: Emit the camera variation draws in `tools/dmp_dump.cc --meta-only` so variation is verifiable ramp-independently
- [x] T077 [US6] ✅ **DONE 2026-08-02** — grep clean; the config-struct doubles carry a block annotation on the `cepGateThreshold` precedent — orig: Run the Principle VI type-domain grep on the diff; annotate or convert

**Checkpoint**: spot-check that variation varies and that zero-sigma is bit-identical.

---

## Phase 8: User Story 7 — Optics Record (Priority: P3) — independent, may float

**Goal**: preserve the optics analysis as a durable artefact so the eventual lens and sensor decision
inherits the reasoning.

**Independent Test**: a reader not party to the scoping conversation can state why 120° was retained, what
it costs, and which measurements would overturn it.

**Note**: pure documentation, no code dependencies. Can be done at any point after Phase 2.

- [x] T078 [P] [US7] ✅ **DONE 2026-08-02** — [optics-record.md](optics-record.md). Records that **120° was RETAINED BY DEFAULT, not selected on evidence** — no candidate lens was priced against it and no alternative was simulated. The cost is quantified: modelled receive at 100 m is **0.039 nA against a ≤10 nA measured decode floor, ≈24 dB short**, so the bench-honest range today is ≈6 m. Written from covering the wide-field signal shortfall against the existing link budget, the sensor format a real 120° build requires, and the assumptions each rests on (FR-027)
- [x] T079 [P] [US7] ✅ **DONE 2026-08-02** — five, ordered by decisiveness, with the lens+bandpass sun/shade test first. **Also records what would NOT settle it**: more short-range bench work with a bare PD, because field test #4 already showed the limiter is ambient compression and a shaded bench cannot see it. The `emitter drive` lever is struck from the table outright — measured, not argued
- [x] T080 [P] [US7] ✅ **DONE 2026-08-02** — referenced, not re-described (Principle X), with the framing that none of them helps until the compression gate is cleared. Was: link the deferred optics directions (narrower fields, dual field-of-view, second camera, raptor binocular arrangement) from `specs/BACKLOG.md` to the optics record rather than re-describing them (Principle X)

---

## Phase 9: User Story 5 — A More Robust M2 (Priority: P2) — terminal

**Goal**: a retrained M2 whose measured competence reflects what hardware can deliver.

**Independent Test**: train against the corrected perception model and evaluate on unseen paths, reporting
the aggregate delta against the prior baseline.

**Note**: depends on every preceding phase. Priority P2 but strictly last by dependency.

### Pre-run gates

- [x] T081 [US5] ✅ **PASSED 2026-07-30** — clean `scripts/rebuild-perf.sh` (optimized, single-threaded for FP determinism), log `logs/040-t1-prerun-gate.log`, exit 0. **41/41 suites, 398 tests, 0 fail**; crrcsim links clean against the new `signal_model` / `acquisition_state` objects. The one `error` string is the benign `ConfigManager not initialized` fail-loud path test T001 documented. Test count 339 (T001 baseline) → 398.

  ⚠️ **A defect in the GATE ITSELF was found and fixed while verifying it** (`ea5fb63`): `nn_telemetry_tests` was a `DEPENDS` of `run_autoc_tests` (so it compiled) and an `add_test` (so `ctest` ran it) but was **missing from the custom target's `COMMAND` chain**. Since `rebuild-perf.sh` is just `cmake && make`, the mandatory pre-run gate had been **building that suite and never executing it** — 40 of 41. Pre-existing, unrelated to 040; `add_test` names and `COMMAND` entries now match exactly at 41, so **this is the first gate run that actually ran everything**
- [x] T082 [US5] ✅ **PASSED 2026-08-01 — no regression; a GAIN.** Run mean **6,678 sims/s** over 1,176,235,200 evaluations in 176,127 s, against the T002 t9 baseline of ≈5,600 sims/s late-run. Per R10 the honest comparator is sims/s at a *comparable generation*, not the run mean, so measured that way too: mid (150-250) **+13.7%**, late (300-356) **+11.0%**. FR-038's ≤10% ceiling is not merely met, it is inverted. Credit belongs to the **analytic once-per-tick acquisition advance** — 10 chips' worth of credit in one arithmetic step rather than sub-stepping 24 camera frames — which was chosen for exactly this reason. *(The early window reads −25%, which is NOT a regression: early generations are dominated by short crash-terminated scenarios and the two runs crash at different rates there, so that window compares different workloads. R10 warns about this specifically.)*

### Run and evaluate

- [x] T082a [US5] ✅ **DONE 2026-07-30 — confirmed, not substituted.** `autoc-tracker.ini` points at `autoc-m1` · `autoc-9223370253553029228-2026-07-06T01:35:46.579Z/gen9200.dmp.zst`, which **is** the T003a-pinned `retain=keep` training source. SC-008 therefore compares like-for-like: **new M2 perception against the same old M1**, with the perception model as the only moving part. Nothing to record as confounding

**Full ini audit, 2026-07-30 (operator-confirmed).** Scenario set is the standard 6 paths × 49 winds = **294 scenarios** at pop 5000 / 800 gens — the shape the T021 gate ran. Cadence `ControlIntervalMsec=50` (20 Hz) + `ServoModelEnabled=1`, unchanged from 037. Craft sigmas unchanged from 039. `VariationRampStep = 0` — **not ramping**, full variation from gen 1.

**Two couplings surfaced and accepted by the operator rather than changed:**

1. **`VariationRampStep=0` un-ramps the CRASH PENALTIES too.** Hull and OOB both scale by `computeVariationScale()`, which returns a flat 1.0 when the ramp is off — so `HullCrashPenaltyFactor=0.75` and `OobCrashPenaltyWeight=2.0` bite at **full strength from generation 1**, which is *not* the t14 configuration where they ramped in. Not obvious from reading either key; recorded here so a surprise in the early-gen curve is attributable.
2. **`Seed = -1`** — random, so this run is not bit-replayable from the ini alone. Accepted; the run's own seed is logged.

**Cleanup done in the same pass**: `BeaconEmissionConeDeg` **deleted** — T058's flat-top emission profile left the hard 270° cutoff with no reader, so it was a live-looking knob in three tracker inis that changed nothing (Constitution III, same as T014/T017). `AUTOC_CONFIG_FIELDS` 134 → 133.

> 📊 **What the perception model actually delivers is now recorded in [research.md](research.md) §R14** — the range/SNR/q table, and the headline for the hardware work: modelled received at 100 m is **0.039 nA against a ≤10 nA measured decode floor, ≈24 dB short**. The 100 m envelope is an *assertion* (FR-033a); closing that gap is the camera-perf / emitter-power effort
- [x] T083 [US5] ✅ **DONE 2026-07-30** — launched via `scripts/train.sh`, which self-detaches (`setsid` + `nohup`, reparents to `systemd --user`) and so survived both terminal and agent-session teardown. Master seed **1785475004** (`Seed=-1`, so this value exists nowhere but the log — it is what makes the run bit-reproducible). Output prefix `autoc-m2/autoc-9223370251379769240-2026-07-31T05:16:46.567Z/`
- [x] T084 [US5] ✅ **DONE 2026-08-01 — plateau reached AND the run completed.** All 800 generations, "NN evolution complete!", best fitness −15048.21, no crashes or worker deaths. **61 elite changes, the last at gen 612 — 188 generations with no improvement**, so this is genuine convergence rather than a run cut short.

  **Result vs the 038 t9 M2 baseline (both at their final generation):**

  | metric | 040-t1 g800 | 038-t9 g430 | |
  |---|---:|---:|---|
  | avgMaxStreak | 23.60 | 21.10 | **+11.8%** |
  | pctInStreak | 8.00% | 7.50% | **+6.7%** |
  | avgInRamp | 0.096 | 0.090 | **+6.7%** |
  | avgRngMed | **15.14 m** | 16.86 m | **−10.2%** |
  | avgRngMin | **3.14 m** | 3.60 m | **−12.8%** |
  | avgVis | 0.789 | 0.795 | −0.8% |

  **This is the outcome US5 hoped for and not the one to have predicted.** Perception got materially HARDER in every direction 040 touched — acquisition costs 3-6 ticks instead of zero, quality decays with range, obstruction is ON for the first time, separation-derived range dies past ~28 m, and the beacon separation shrank 17% — and the controller came out **closer** and **stickier**. The honest reading is that the prior model's optimism was not buying the controller anything real.

  `avgVis` is the one metric that moved against the trend, and it is coherent rather than alarming: closing to a 15.1 m median puts more time in the near field where the merged-blob regime and the pod-nose shadow actually bite. **Pre-040 the perception model had no near-field cost to pay at all**, so this trade was previously invisible.

  ⚠️ **A decision for T086, to be stated rather than defaulted into**: the best *tracking* elite was **gen 585's** (pctInStreak 8.3%, avgRngMed 14.8 m); the *final* elite is **gen 612's** (8.0%, 15.1 m), which won on total fitness by trading a little tracking occupancy on another lexicase axis. Which one is the SC-008 comparator changes the reported delta.
- [ ] T085 [US5] Evaluate the resulting elite on novel paths by repointing `autoc-eval-tracker.ini` at a novel M1 eval source
- [ ] T086 [US5] Write `specs/040-camera-redo/outcome.md` reporting the **aggregate delta** against the prior baseline on the established comparators — the question is *are we in the right room, and is this more honest?*, not per-term attribution (SC-008)
- [ ] T087 [US5] Tag artifacts `retain=expire`; if the controller becomes a baseline, pin `retain=keep` and record its S3 prefix in `outcome.md` (Principle VIII)

**Checkpoint**: a competence drop attributable to more honest perception is a **valid outcome**. No floor
gates this feature.

---

## Phase 10: Polish & Cross-Cutting

- [ ] T088 [P] Update `tools/renderer.cc` to draw the effective field of view including obstructed regions and visible quality regimes (FR-030). **The parallax layer that extends this — rings, vertical member, quality rendering, `q` bar — moved to T065d–T065g in Phase 6**, because the Phase 6 checkpoint cannot be read without it
- [x] T089 [P] ✅ **DONE 2026-07-30, pulled forward as planned** — — emit the new diagnostic fields (pixel indices, correlation margin, lock state) in `tools/dmp_dump.cc` (FR-028). T065f/T065g cannot render lock state or `q` until these reach `CameraViewSample`
- [x] T090 [P] ✅ **DONE 2026-07-30** — round-trips `raw_margin` + `lock_state`; the fixture sweeps all four `LockState` values and includes a **negative** SNR, because a naive unsigned round-trip would survive positive-only data. — — extend `tests/tracker_dmp_roundtrip_tests.cc` to round-trip the new diagnostic fields (FR-029)
- [x] T091 ✅ **DONE 2026-08-02 — and the audit found real drift.** The contract was a PLAN-TIME document: **32 keys it named were never implemented under those names** and **~50 shipped perception keys were absent from it**. Regenerated against the shipped `AUTOC_CONFIG_FIELDS`, with every non-existent planned key recorded and explained so nobody hunts for it. Also documents the two invisible couplings (the back-solved noise floor; the knee sitting 100× above the floor) that break silently if edited alone. A classification record that disagrees with the code is worse than none, since T092 reads it to decide what may be overwritten
- [x] T092 ✅ **PASSED 2026-08-02** — [calibration-rehearsal.md](calibration-rehearsal.md). **15 assumed values substituted** on a *copy* of the ini; it parsed, the model moved (`blocked_frac` 0.0292 → 0.0415), and **`git status` over src/include/tools/crrcsim and the shipped ini stayed clean throughout — no structural change required. SC-012 holds.** An unplanned second property fell out: FOV stayed exactly 120°×90° across a 320→640 px substitution because 640 × 0.1875 = 120, i.e. **FR-003's derived field held with no key to keep in sync** — under the retired `CameraFOVHorizontalDeg` design the same substitution would have silently produced a config whose declared field and actual grid disagreed. ⚠️ Limits stated in the doc rather than glossed: the end-to-end path exercised is geometry/sensor; the `Signal*`/`Quality*` rows were proven to parse+print and are assigned to `WorkerInit`, but no report in this rehearsal reads them back
- [ ] T093 Run `bash scripts/rebuild.sh` for the final correctness gate (Principle II)
- [ ] T094 Final Principle VI type-domain grep across all touched paths
- [ ] T095 Walk [quickstart.md](quickstart.md) end to end and correct any drift

---

## Dependencies & Execution Order

### ⚠️ Stories are NOT independent

The usual speckit assumption does not hold. The real graph:

```
US1 (verdict) ──GATE──┐
                      ▼
                Foundational ──→ US2 ──→ US3 ──→ US4 ──→ US6 ──→ US5
                      │                                            ▲
                      └──→ US7 (independent, may float) ───────────┘
```

- **US1** gates everything — a "regenerate" verdict reorders the feature behind an M1 rebake
- **Foundational** blocks all stories; also the last objective pass/fail (bit-identity)
- **US2 → US3 → US4** is a strict chain: obstruction needs the angular representation, the signal budget
  consumes obstruction as an input term
- **US3 closes in Phase 6, not Phase 5.** Its acceptance scenario 6 (obstruction-clearing ride-through,
  FR-013) needs US4's hold machinery — carried by T056a/T064a. US3 is implemented in Phase 5 but not fully
  verifiable until Phase 6
- **US6** needs US2/US3/US4 to have something to vary
- **US5** is terminal — depends on everything, despite being P2
- **US7** is genuinely independent — documentation only, may float anywhere after Phase 2

### Parallel opportunities

Real but limited to *within* phases:

- **Phase 1**: T002, T003 in parallel
- **Phase 3**: T007–T009 (tests) in parallel; T013, T015, T017, T018 in parallel
- **Phase 4**: T023–T028 (tests) all parallel; T030 parallel with T029
- **Phase 5**: T036–T040 (tests) all parallel
- **Phase 6**: T048–T056 (tests) all parallel; T057 and T060 parallel; **T065a–T065c (playback transport) parallel with the whole signal-model chain** — they touch only `tools/renderer.{cc,h}` and depend on nothing in US4, so land them FIRST and use them while iterating on the model. T065d–T065g follow T089
- **Phase 7**: T066–T070 (tests) all parallel
- **Phase 8**: T078–T080 all parallel, and the whole phase parallel with Phases 4–7
- **Phase 10**: T088–T090 in parallel

Cross-phase parallelism is essentially limited to floating US7 alongside the chain.

---

## Parallel Example: User Story 4 tests

```bash
# Launch all US4 test-writing tasks together — distinct files, no interdependency:
Task: "signal model monotonic falloff test in tests/signal_model_tests.cc"
Task: "flat-top emission profile test in tests/signal_model_tests.cc"
Task: "detector-sharing penalty test in tests/signal_model_tests.cc"
Task: "acquisition timing test in tests/acquisition_state_tests.cc"
Task: "tentative-lock quality test in tests/acquisition_state_tests.cc"
Task: "hold ride-through test in tests/acquisition_state_tests.cc"
Task: "determinism test in tests/acquisition_state_tests.cc"
Task: "scenario-reset test in tests/acquisition_state_tests.cc"
```

---

## Implementation Strategy

### MVP scope

**Phases 1–4** (Setup + US1 verdict + Foundational + US2 geometry). That delivers the single largest
correctness fix — the ~17% beacon-separation error in the sole range channel, plus a bearing encoding that
finally matches the sensor it models — and is retrainable on its own. Everything after sharpens perception
further; nothing after is required for the geometry fix to be worth having.

### Incremental delivery

1. **Phase 2 (US1)** → verdict recorded, path cleared or reordered
2. **Phase 3** → foundation single-sourced, bit-identity proven
3. **Phase 4 (US2)** → honest geometry — **STOP AND VALIDATE**, renderer check
4. **Phase 5 (US3)** → obstruction, read the distribution
5. **Phase 6 (US4)** → signal quality — playback review
6. **Phase 7 (US6)** → variation
7. **Phase 9 (US5)** → retrain, aggregate delta
8. **Phase 10** → calibration rehearsal proves the plumbing-first claim

Phase 8 (US7) floats.

### Standing constitutional obligations

| Principle | Obligation | Where |
|---|---|---|
| **I** | Tests written first, verified failing | every story phase |
| **II** | `rebuild.sh` before commit | T093 |
| **IV** | `rebuild-perf.sh` after any `CMakeLists.txt` change — never incremental | T020, T081 |
| **VI** | Type-domain grep at every milestone close | T022, T035, T047, T065, T077, T094 |
| **VII** | No in-class defaults on `WorkerInit`-sourced members | T016 (highest attention — the cited bug lives here) |
| **VIII** | Tag `retain=expire`; pin keepers | T087 |
| **IX** | `train.sh` only; pre-run build gate | T081, T083 |
| **X** | Deferrals to `specs/BACKLOG.md` only | T006, T080 |

---

## Notes

- `[P]` = different files, no dependency on incomplete work
- **T021 is the only objective pass/fail in the feature** — every later phase deliberately changes outputs,
  so the determinism gate weakens from "identical to baseline" to "identical to itself"
- **T092 is the feature's central claim under test** — if a calibration rehearsal demands a code change, the
  plumbing-first contract is broken
- Manual checkpoints (renderer at Phase 4, distribution at Phase 5, playback at Phase 6) catch what unit
  tests structurally cannot
- Commit after each task or logical group
