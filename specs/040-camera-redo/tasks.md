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

- [ ] T001 Confirm clean baseline: run `bash scripts/rebuild.sh` and verify all tests pass before any edits
- [ ] T002 [P] Record the prior M2 run's throughput (generations/hour from its logfile) into `specs/040-camera-redo/research.md` as the FR-038 benchmark baseline
- [ ] T003 [P] Pin a prior-M2 elite and record its exact eval fitness in `specs/040-camera-redo/research.md` — this is the bit-identity oracle for the T021 foundational gate

---

## Phase 2: User Story 1 — Airframe Fidelity Verdict (Priority: P1) 🚦 GATING

**Goal**: decide whether the simulated flight model must be regenerated before any perception work begins.

**Independent Test**: a reader can see every simulated-vs-measured airframe parameter classified with its
magnitude of disagreement, ending in an explicit recorded decision. Delivers value even if nothing else ships.

**Why first**: a "regenerate" verdict forces an M1 source rebake *before* perception work, since M2 trains
off the M1 source. No code — a document and a decision.

- [ ] T004 [US1] Compare simulated vs measured airframe/propulsion parameters (mass, span, wing area, propeller diameter/pitch, CG) in a new `specs/040-camera-redo/airframe-fidelity.md`, classifying each agree/disagree/unknown with magnitude, sourced from `crrcsim/models/hb1_streamer.xml` and [input-data-checklist.md](input-data-checklist.md)
- [ ] T005 [US1] Record in `specs/040-camera-redo/airframe-fidelity.md` the explicit decision — regenerate M1 source now, or defer — with reasoning (FR-025)
- [ ] T006 [US1] File each deferred discrepancy (notably the propeller 5.0×4.5-vs-5.5×4 mismatch) into `specs/BACKLOG.md` against the feature that will address it (FR-026, Principle X)

**🚦 CHECKPOINT — HARD GATE**: if the verdict is **regenerate**, STOP. Rebake the M1 source before Phase 3.
If **defer** (expected), proceed. Everything downstream assumes the existing M1 source is valid.

---

## Phase 3: Foundational (Blocking Prerequisites)

**Purpose**: single-source the per-tick rule, fix the unusable obstruction proxy, and land the config
surface — before anything depends on them.

**⚠️ CRITICAL**: no user story after this can begin until Phase 3 completes. This is also the **last
milestone with an objective pass/fail** — it must be behaviour-preserving, so bit-identity either holds or
there is a bug. Every later phase deliberately changes outputs.

### Tests (write first, verify failing)

- [ ] T007 [P] Add per-scenario state-reset assertions to `tests/tracker_stepper_init_tests.cc` — all carried perception state must zero at scenario boundaries (FR-020a)
- [ ] T008 [P] Add a test to `tests/beacon_projection_tests.cc` asserting the camera mount never lies on an obstruction primitive boundary (guards the current degenerate default)
- [ ] T009 [P] Add a test to `tests/contract_tracker_config_tests.cc` asserting the new ini key count and that every new key is read (no silent default)

### Implementation

- [ ] T010 Extract the shared per-tick perception rule (CEP gating, separation, situational-awareness update) into new `include/autoc/eval/tracker_tick_rule.h` + `src/eval/tracker_tick_rule.cc` (FR-031)
- [ ] T011 Point the production tick `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` at the shared rule, removing its inline duplicate
- [ ] T012 Point the test-only reference `src/eval/tracker_stepper.cc` at the same shared rule, removing its inline duplicate (see research R1 — this path is test-only but is the behavioural contract)
- [ ] T013 [P] Create `include/autoc/eval/airframe_occlusion.h` + `src/eval/airframe_occlusion.cc` with three primitives — thin wing slab, pod nose box, static propeller disc — per [data-model.md](data-model.md) §3
- [ ] T014 Replace `defaultAirframeProxyHB1()` in `include/autoc/eval/camera_projection.h` with the new obstruction set and delete the degenerate single-AABB proxy (Principle III — clean cut, no shim)
- [ ] T015 [P] Add all new ini keys to the X-macro in `include/autoc/util/config.h` per [contracts/config-surface.md](contracts/config-surface.md)
- [ ] T016 Wire the new config values through `src/autoc.cc` worker init into `WorkerInit` — **no in-class default initializers on any member receiving a constructor parameter** (Principle VII; the `cepGateThreshold` bug the constitution cites lives in this exact code)
- [ ] T017 [P] Retire stale fields from `include/autoc/eval/camera_config.h`: `frame_rate_hz` (predates the 20 Hz decision), `latency_ms` (latency now emerges from acquisition timing), and the `Projection` enum
- [ ] T018 [P] Add the new ini keys with documented defaults to `autoc-tracker.ini`, `autoc-eval-tracker.ini`, and `autoc-eval-tracker-visual.ini`
- [ ] T019 Register new sources and test targets in `CMakeLists.txt`

**Checkpoint gates**:

- [ ] T020 Run `bash scripts/rebuild-perf.sh` — **required, not incremental**, because `CMakeLists.txt` changed (Principle IV; operator drives)
- [ ] T021 **BIT-IDENTITY GATE**: evaluate the T003 pinned elite and confirm `NN_EVAL_SAME` against the recorded fitness. A refactor that changes behaviour has a bug — do not proceed until this passes
- [ ] T022 Run the Principle VI type-domain grep over `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/`; annotate `// raw-ok:` or convert every hit in the diff

---

## Phase 4: User Story 2 — Honest Camera Geometry (Priority: P1) 🎯 MVP

**Goal**: bearing quantised on a real pixel grid, reported as isotropic angles, with the corrected physical
beacon separation.

**Independent Test**: project known geometries and confirm a fixed angular separation reads the same
anywhere in frame and at any orientation; bearings resolve no finer or coarser than the pixel grid; inferred
range matches truth.

### Tests (write first, verify failing)

- [ ] T023 [P] [US2] Test in `tests/beacon_projection_tests.cc`: a fixed angular separation reads within 2% of the same value at frame centre and near the edge (SC-001)
- [ ] T024 [P] [US2] Test in `tests/beacon_projection_tests.cc`: the same separation rotated horizontal→vertical is unchanged — both axes share one scale (FR-002)
- [ ] T025 [P] [US2] Test in `tests/beacon_projection_tests.cc`: bearing is quantised to the pixel grid, neither finer nor coarser (FR-001)
- [ ] T026 [P] [US2] Test in `tests/beacon_projection_tests.cc`: range inferred from separation matches truth within grid resolution, no systematic bias — guards the 0.772 m correction (SC-002)
- [ ] T027 [P] [US2] Test in `tests/gather_tracker_inputs_tests.cc`: controller input vector remains exactly 58 (FR-006)
- [ ] T028 [P] [US2] Test in `tests/beacon_projection_tests.cc`: FOV is derived from grid × deg-per-pixel and cannot be set independently (FR-003)

### Implementation

- [ ] T029 [US2] Rework `include/autoc/eval/camera_config.h` to `pixels_h` / `pixels_v` / `deg_per_px`, with `fov_h_deg` / `fov_v_deg` derived (FR-003)
- [ ] T030 [P] [US2] Correct beacon separation to 0.772 m in `include/autoc/eval/beacon_config.h` (±0.386 m mounts), replacing ±0.45 m (FR-004)
- [ ] T031 [US2] Rewrite the projection stage in `src/eval/camera_projection.cc`: quantise to the pixel grid, then emit isotropic **radians** (FR-001, FR-002)
- [ ] T032 [US2] Remove the int8 bearing encoding from `include/autoc/eval/camera_projection.h` and add `int16` pixel-index fields, `// raw-ok:` annotated (Principle III + VI)
- [ ] T033 [US2] Update derived features in `src/nn/evaluator.cc` and `include/autoc/eval/derived_features.h` for the radian scale — separation, separation rate, tilt (tilt stays sin/cos per FR-005)
- [ ] T034 [US2] Rescale the camera POV panel in `tools/renderer.cc` from the retired ±1 convention to radians, including the angular reticle
- [ ] T035 [US2] Run the Principle VI type-domain grep on the diff; annotate or convert

**Checkpoint**: **manual renderer check** — load a playback and confirm beacons render where expected under
the new scale. Automated tests cannot catch a scale error that is self-consistent.

---

## Phase 5: User Story 3 — Prove the Mount Clears Obstructions (Priority: P2)

**Goal**: model wing, nose and propeller obstruction from measured geometry; publish the *effective* field
and the distribution of obstruction onset across the mounting-error envelope.

**Independent Test**: sweep a target through the field at nominal and across the error envelope; the
reported obstruction-onset distribution matches geometric prediction and the wing contributes nothing.

### Tests (write first, verify failing)

- [ ] T036 [P] [US3] Test in `tests/beacon_projection_tests.cc`: at the baseline mount the wing contributes zero obstruction (nothing sits ahead of the leading edge)
- [ ] T037 [P] [US3] Test in `tests/beacon_projection_tests.cc`: propeller shadow onset at the baseline mount matches the geometric prediction (≈41° inboard)
- [ ] T038 [P] [US3] Test in `tests/beacon_projection_tests.cc`: an alternative configured mount (wing-top) reports different obstruction without code change (FR-011a)
- [ ] T039 [P] [US3] Test in `tests/beacon_projection_tests.cc`: propeller obstruction attenuates rather than hard-cutting (FR-009)
- [ ] T040 [P] [US3] Test in `tests/beacon_projection_tests.cc`: the reported effective field differs from the nominal rectangle by the geometrically justified amount (FR-012, SC-003)

### Implementation

- [ ] T041 [US3] Implement wing-slab and nose-box ray tests in `src/eval/airframe_occlusion.cc` using the measured station stack (prop 0″, wing LE 6″, camera 8″, wing TE 13″)
- [ ] T042 [US3] Implement the static propeller-disc angular region with representative attenuation in `src/eval/airframe_occlusion.cc` — **no engine speed, no blade phase** (FR-009)
- [ ] T043 [US3] Set the baseline mount (leading edge, 8″ outboard, ~1″ above thrust line, boresight parallel to thrust line) as the default in `autoc-tracker.ini` (FR-007a)
- [ ] T044 [US3] Compute and expose the effective field of view — nominal minus all obstructions — from `src/eval/camera_projection.cc` (FR-012)
- [ ] T045 [US3] Add an obstruction-onset distribution report (median / 95th / clipped extreme across the mounting-error envelope) to `tools/dmp_dump.cc` (FR-007b, SC-007)
- [ ] T046 [US3] Record the propeller blade-passage envelope arithmetic as a research note in `specs/040-camera-redo/research.md` — recorded, not resolved (FR-011)
- [ ] T047 [US3] Run the Principle VI type-domain grep on the diff; annotate or convert

**Checkpoint**: **read the obstruction-onset distribution.** Per the "let it ride" clarification this is an
observation, not a pass/fail — but it is the milestone's actual output.

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

- [ ] T048 [P] [US4] New `tests/signal_model_tests.cc`: signal falls monotonically with range and the loss-of-tracking point lands inside the measured band (FR-015, SC-004)
- [ ] T049 [P] [US4] In `tests/signal_model_tests.cc`: emission follows a flat-top profile — near-constant to ±45°, half power at ±75° — and is **not** a cosine power law (FR-019)
- [ ] T050 [P] [US4] In `tests/signal_model_tests.cc`: when both beacons share a detector element, detection and code identification survive with only the measured penalty (FR-016 — this configuration is field-proven, not a failure)
- [ ] T051 [P] [US4] New `tests/acquisition_state_tests.cc`: acquisition completes within the measured window, including partial-code early lock (FR-017, SC-005)
- [ ] T052 [P] [US4] In `tests/acquisition_state_tests.cc`: a tentative lock reports a bearing with large-variance quality, improving toward confirmed (FR-017a)
- [ ] T053 [P] [US4] In `tests/acquisition_state_tests.cc`: an interruption shorter than the hold window never loses tracking (FR-018, SC-005)
- [ ] T054 [P] [US4] In `tests/acquisition_state_tests.cc`: identical inputs produce bit-identical outputs; no PRNG in the signal path (FR-020, SC-006)
- [ ] T055 [P] [US4] In `tests/acquisition_state_tests.cc`: all carried state resets at scenario boundaries in both execution paths (FR-020a)
- [ ] T056 [P] [US4] In `tests/beacon_projection_tests.cc`: bearing reaches the design detection range while separation-derived range goes explicitly unavailable below the resolving limit (FR-033, SC-011)
- [ ] T056a [P] [US4] In `tests/acquisition_state_tests.cc`: when a **brief obstruction** clears, recovery follows the measured ride-through rather than restoring instantaneously — distinct from T053, which covers signal loss (FR-013, US3 acceptance scenario 6)

### Implementation

- [ ] T057 [P] [US4] Create `include/autoc/eval/signal_model.h` + `src/eval/signal_model.cc` — drive × emission × 1/r² × obstruction ÷ ambient → per-chip SNR, per [data-model.md](data-model.md) §4
- [ ] T058 [US4] Implement the flat-top emission profile in `src/eval/signal_model.cc`, replacing the hard 270° cone in `include/autoc/eval/beacon_config.h` (FR-019)
- [ ] T059 [US4] Implement the detector-sharing penalty in `src/eval/signal_model.cc` — degrades quality, never gates detection (FR-016)
- [ ] T060 [P] [US4] Create `include/autoc/eval/acquisition_state.h` + `src/eval/acquisition_state.cc` — chip-credit integrator and four-state machine per [data-model.md](data-model.md) §5, deterministic thresholds not draws
- [ ] T061 [US4] Derive quality from signal and lock state in `src/eval/camera_projection.cc`, replacing the linear edge-factor placeholder (FR-014)
- [ ] T062 [US4] Keep the state machine internal — expose only bearing and quality; record state for diagnostics only (FR-017b, [contracts/perception-interface.md](contracts/perception-interface.md))
- [ ] T063 [US4] Implement the two-envelope rule in `src/eval/camera_projection.cc`: bearing to design range, separation-derived range unavailable below the resolving limit (FR-033)
- [ ] T064 [US4] Wire per-scenario reset of all acquisition state into `src/eval/tracker_tick_rule.cc` so both paths reset identically (FR-020a)
- [ ] T064a [US4] Route obstruction state through the acquisition hold path in `src/eval/acquisition_state.cc` so a cleared obstruction re-establishes via ride-through, not instantaneous restoration (FR-013). **Note the asymmetry**: an obstruction clears deterministically by geometry while a signal returns by SNR — both must enter the same hold machinery, but only obstruction has a knowable clear-time
- [ ] T065 [US4] Run the Principle VI type-domain grep on the diff; annotate or convert

**Checkpoint**: **playback review.** Does dropout and reacquisition look physical? Is the range-envelope
crossover where expected? This is where "passes its tests" and "physically plausible" most easily diverge.

---

## Phase 7: User Story 6 — Per-Scenario Camera Variation (Priority: P3)

**Goal**: each scenario draws its own camera imperfections, so the controller cannot depend on a perfectly
known camera.

**Independent Test**: scenarios draw distinct camera parameters; the same scenario id always reproduces the
same draw; zero sigmas reproduce the baseline bit-identically.

### Tests (write first, verify failing)

- [ ] T066 [P] [US6] New `tests/camera_variation_tests.cc`: two scenarios draw different camera parameters within configured bounds (FR-021)
- [ ] T067 [P] [US6] In `tests/camera_variation_tests.cc`: the same scenario id reproduces identical draws across evaluations (FR-022)
- [ ] T068 [P] [US6] In `tests/camera_variation_tests.cc`: with all sigmas zero, results are bit-identical to the no-variation baseline (FR-023, SC-006)
- [ ] T069 [P] [US6] In `tests/camera_variation_tests.cc`: draws are hard-clipped at 20°, never tail-sampled beyond it
- [ ] T070 [P] [US6] In `tests/camera_variation_tests.cc`: camera variation stays chase-specific even when environment seeds are shared with the target (FR-022)

### Implementation

- [ ] T071 [US6] Consume the reserved `camera` sub-seed (slot 5) in `include/autoc/util/scenario_prng.h` — the slot is already seeded and the class order frozen
- [ ] T072 [US6] Add `cameraSeed` and the variation draws to `include/autoc/rpc/scenario_metadata.h`, following the craft-variation pattern (raw pre-scale draws recorded)
- [ ] T073 [US6] Apply boresight and roll error (σ 10°, hard clip 20°) to the camera pose in `src/eval/camera_projection.cc`
- [ ] T074 [US6] Apply mount translation (1 cm box) in `src/eval/airframe_occlusion.cc` — **obstruction path only**, deliberately not applied to bearing in `src/eval/camera_projection.cc`: ±5 mm is 0.03° at 10 m (negligible) but swings propeller clearance ~15% (research R6)
- [ ] T075 [US6] Apply wing-thickness and ambient-level variation to `src/eval/airframe_occlusion.cc` and `src/eval/signal_model.cc` respectively
- [ ] T076 [US6] Emit the camera variation draws in `tools/dmp_dump.cc --meta-only` so variation is verifiable ramp-independently
- [ ] T077 [US6] Run the Principle VI type-domain grep on the diff; annotate or convert

**Checkpoint**: spot-check that variation varies and that zero-sigma is bit-identical.

---

## Phase 8: User Story 7 — Optics Record (Priority: P3) — independent, may float

**Goal**: preserve the optics analysis as a durable artefact so the eventual lens and sensor decision
inherits the reasoning.

**Independent Test**: a reader not party to the scoping conversation can state why 120° was retained, what
it costs, and which measurements would overturn it.

**Note**: pure documentation, no code dependencies. Can be done at any point after Phase 2.

- [ ] T078 [P] [US7] Write `specs/040-camera-redo/optics-record.md` covering the wide-field signal shortfall against the existing link budget, the sensor format a real 120° build requires, and the assumptions each rests on (FR-027)
- [ ] T079 [P] [US7] In `specs/040-camera-redo/optics-record.md`, state the specific measurements that would confirm or overturn the analysis
- [ ] T080 [P] [US7] Link the deferred optics directions (narrower fields, dual field-of-view, second camera, raptor binocular arrangement) from `specs/BACKLOG.md` to the optics record rather than re-describing them (Principle X)

---

## Phase 9: User Story 5 — A More Robust M2 (Priority: P2) — terminal

**Goal**: a retrained M2 whose measured competence reflects what hardware can deliver.

**Independent Test**: train against the corrected perception model and evaluate on unseen paths, reporting
the aggregate delta against the prior baseline.

**Note**: depends on every preceding phase. Priority P2 but strictly last by dependency.

### Pre-run gates

- [ ] T081 [US5] Run `bash scripts/rebuild-perf.sh` and the full test suite — mandatory pre-run gate, perception changes are determinism-affecting (Principle IX)
- [ ] T082 [US5] **THROUGHPUT BENCHMARK**: measure total evaluation throughput against the T002 prior-M2 baseline; confirm ≤10% regression (FR-037/038, SC-013). A breach escalates to an explicit accept-or-optimise decision, not silent absorption

### Run and evaluate

- [ ] T083 [US5] Launch the retrain via `bash scripts/train.sh autoc-tracker.ini logs/autoc-040-t1-perception.log` — **detached only**, never a harness-tracked background task (Principle IX)
- [ ] T084 [US5] Monitor to a competence plateau; confirm the run completes without systemic failure
- [ ] T085 [US5] Evaluate the resulting elite on novel paths by repointing `autoc-eval-tracker.ini` at a novel M1 eval source
- [ ] T086 [US5] Write `specs/040-camera-redo/outcome.md` reporting the **aggregate delta** against the prior baseline on the established comparators — the question is *are we in the right room, and is this more honest?*, not per-term attribution (SC-008)
- [ ] T087 [US5] Tag artifacts `retain=expire`; if the controller becomes a baseline, pin `retain=keep` and record its S3 prefix in `outcome.md` (Principle VIII)

**Checkpoint**: a competence drop attributable to more honest perception is a **valid outcome**. No floor
gates this feature.

---

## Phase 10: Polish & Cross-Cutting

- [ ] T088 [P] Update `tools/renderer.cc` to draw the effective field of view including obstructed regions and visible quality regimes (FR-030)
- [ ] T089 [P] Emit the new diagnostic fields (pixel indices, correlation margin, lock state) in `tools/dmp_dump.cc` (FR-028)
- [ ] T090 [P] Extend `tests/tracker_dmp_roundtrip_tests.cc` to round-trip the new diagnostic fields (FR-029)
- [ ] T091 Verify every configured physical quantity is classified measured/derived/assumed in [contracts/config-surface.md](contracts/config-surface.md) and matches the shipped defaults (FR-035, SC-010)
- [ ] T092 **CALIBRATION REHEARSAL**: substitute a plausible alternative for each **assumed** value, confirm results change and **no structural change is required** (FR-036, SC-012) — this proves the feature's central claim
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
- **Phase 6**: T048–T056 (tests) all parallel; T057 and T060 parallel
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
