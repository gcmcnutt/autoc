---

description: "Task list for 032 Tracker NN Enhancements — Phase 1 (derived pose features)"
---

# Tasks: 032 Tracker NN Enhancements — Phase 1

**Input**: Design documents from `/specs/032-tracker-nn-enhancements/`
**Prerequisites**: [plan.md](./plan.md) (required), [spec.md](./spec.md) (required), [research.md](./research.md), [data-model.md](./data-model.md), [contracts/](./contracts/), [quickstart.md](./quickstart.md)

**Tests**: INCLUDED. Constitution Principle I (Testing-First) is mandatory for the autoc project; per-phase TDD applies. See [Constitution I](../../.specify/memory/constitution.md).

**Organization**: Two user stories map to the phase 1 combined bake (US1, MVP) and the phase-1b attribution-bake contingency (US2). The contingency only fires if US1's plateau-avgInRamp lands in the 0.10–0.15 partial band per spec Q7.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1 = phase 1 combined bake (MVP); US2 = phase-1b attribution bake (contingent)
- File paths are absolute or repo-root-relative

## Path Conventions (this project)

- `include/autoc/nn/`, `include/autoc/eval/` — public headers
- `src/nn/`, `src/eval/` — autoc minisim implementation
- `crrcsim/src/mod_inputdev/inputdev_autoc/` — crrcsim FDM bridge
- `tests/` — GoogleTest unit + contract tests at repo root
- `autoc-tracker.ini` — tracker-mode config at repo root
- `docs/` — operator-facing conventions (xiao migration prep)
- `specs/032-tracker-nn-enhancements/` — closeout artifacts (outcome.md)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify build + determinism state before introducing any 032 changes.

- [ ] T001 Run `bash scripts/rebuild-perf.sh` from repo root and record baseline M1 fitness numbers; gate any 032 work on this passing on a clean tree first (regression-gate reference per [quickstart.md](./quickstart.md#pre-bake--verify-build--determinism))
- [X] T002 [P] Confirm `docs/COORDINATE_CONVENTIONS.md` and `docs/sensor-pipeline.md` exist and capture their current section structure (target sections for the phase-1 doc additions). ALSO inspect [autoc.ini](autoc.ini) for the M1 `MinisimProgram` value — record whether it currently points to minisim, `./scripts/crrcsim.sh`, or both. This pre-resolves the conditional in T026 (M1-under-CRRCSim regression) so the operator knows ahead of time whether to plan the extra bake or skip it. **2026-05-16 findings**: COORDINATE_CONVENTIONS.md "## 030 Tracker-Mode NN Inputs (45 floats)" at line 132 = T023 target (retitle to 54 + add identity-stable + tilt convention); sensor-pipeline.md has §§1-10 (INAV/MSP/CRRCSim) but no tracker-mode section → T024 adds a new top-level section. **M1 already runs under crrcsim** (both autoc.ini:50 and autoc-tracker.ini:85 use `MinisimProgram = ./scripts/crrcsim.sh`) — T026 is directly executable with autoc.ini, no sibling config needed

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Schema + config plumbing that ALL phase 1 work (including US1 tests) depends on. Once these land, the contract tests in US1 compile and fail loudly; implementation can then proceed in parallel.

**⚠️ CRITICAL**: No US1 implementation can begin until this phase is complete.

- [X] T003 Extend `TrackerInput` enum in [include/autoc/nn/nn_inputs.h](include/autoc/nn/nn_inputs.h) with 9 new entries (`BEACON_PAIR_SPAN_TM5..NOW`, `SPAN_RATE`, `TARGET_TILT_SIN`, `TARGET_TILT_COS`) per [data-model.md §1.2](./data-model.md#12-full-enum--struct-shape); update `COUNT` to 54
- [X] T004 Extend `kTrackerInputMeta` array in [include/autoc/nn/nn_inputs.h](include/autoc/nn/nn_inputs.h) with 9 new `SensorInputMeta` entries (display names `spn-5..spn0 dspn tltS tltC`, header_width 7) per [data-model.md §1.3](./data-model.md#13-ktrackerinputmeta-extension)
- [X] T005 Extend `TrackerInputs` struct in [include/autoc/nn/nn_inputs.h](include/autoc/nn/nn_inputs.h) with `beacon_pair_span[6]`, `span_rate`, `target_tilt_sin`, `target_tilt_cos` fields (all `float` with `// raw-ok: NN-byte-format buffer` annotations); update size static_assert to `54 * sizeof(float)`
- [X] T006 Extend `TrackerHistoryWindow` struct in [include/autoc/eval/tracker_stepper.h](include/autoc/eval/tracker_stepper.h) with `float span[6]` field (raw-ok: NN-byte-format primitive) per [data-model.md §2](./data-model.md#2-trackerhistorywindow-extended)
- [X] T007 Add `[DerivedFeatures]` section to [autoc-tracker.ini](autoc-tracker.ini) with `CepGateThreshold = 1.25` per [contracts/ini_schema.md](./contracts/ini_schema.md). **2026-05-16 scope reduction**: `EnableDerivedFeatures` toggle dropped pre-T017 — greenfield M2 means the code IS the change; B-off attribution comes from git revert per research.md R7
- [X] T008 Extend the tracker-mode ini loader in [src/util/config.cc](src/util/config.cc) (config struct in [include/autoc/util/config.h](include/autoc/util/config.h)) to parse `CepGateThreshold` (float, range [0, 2], default 1.25, reject out-of-range at startup with clear error); plumb into the eval pipeline config struct that reaches `gather_tracker_inputs`. `EnableDerivedFeatures` was dropped 2026-05-16 (see T007 note)

**Checkpoint**: All schema and config wiring lands. Build succeeds (existing `gather_tracker_inputs` still populates 45 slots; new 9 slots are uninitialized — tests will catch this in US1).

---

## Phase 3: User Story 1 — Phase 1 combined bake (Priority: P1) 🎯 MVP

**Goal**: Ship the combined Feature A (identity-stable ordering — sim no-op per [research.md R1](./research.md#r1--feature-a-in-sim-is-already-satisfied-the-only-change-is-the-contract), so doc + contract test only) + Feature B (3 new derived features, 9 new input slots) in a single bake. Run to plateau via the 030 reference protocol (≥322 gens, last-50-gen avg). Evaluate plateau-avgInRamp against the ≥0.15 threshold (spec Q2 + Q6).

**Independent Test**: Bake completes ≥322 gens against the postdiag2 source dmp, plateau-avgInRamp readout is produced. M1 (pathgen) bakes remain bitwise-equal to pre-032 baseline per `scripts/rebuild-perf.sh`. Determinism preserved across worker counts.

### Tests for User Story 1 (TDD — write FIRST, ensure they FAIL)

- [X] T009 [P] [US1] Create [tests/derived_features_tests.cc](tests/derived_features_tests.cc) with pure-math unit tests: span scalar from two synthetic NDC pairs (3-4 cases including identity, mirror, large/small); span_rate from two consecutive spans; tilt sin/cos from synthetic NDC pairs at θ ∈ {0, π/2, π, -π/2}; degenerate-pair (pair_dist < 1e-4) → tilt (0, 1); sin² + cos² ≈ 1.0 property
- [X] T010 [P] [US1] Extend [tests/nn_sensor_interface_tests.cc](tests/nn_sensor_interface_tests.cc) with: `static_assert`-mirror runtime check `TrackerInput::COUNT == 54`; round-trip name lookup for all 9 new entries (`nameOf(TARGET_TILT_COS) == "TARGET_TILT_COS"` etc.); display_name canonicality (`spn0`, `dspn`, `tltS`, `tltC`)
- [X] T011 [P] [US1] Extend [tests/gather_tracker_inputs_tests.cc](tests/gather_tracker_inputs_tests.cc) with end-to-end cases: (a) both beacons visible at mid-range → span ~ 0.3, tilt ~ (0, 1); (b) target rolled 90° relative → tilt ~ (1, 0) or (-1, 0); (c) `history.left_cep[5] = 1.5f` → span_now=0, tilt_sin=0, tilt_cos=1; (d) two consecutive ticks → span_rate equals manual subtraction; (e) deterministic across consecutive calls with identical inputs
- [X] T012 [US1] Add identity-invariant contract test to [tests/gather_tracker_inputs_tests.cc](tests/gather_tracker_inputs_tests.cc) per [contracts/identity_invariant.md](./contracts/identity_invariant.md): synthesize target oriented at chase by heading 90° (port wing on image-plane right) → verify `left_x[5]` carries port-beacon NDC (positive x); target oriented away → verify `left_x[5]` still carries port-beacon NDC (negative x). NOT marked [P] because this edits the same file as T011 — author after T011 lands
- [X] T013 [P] [US1] Add config-loader test to [tests/contract_tracker_config_tests.cc](tests/contract_tracker_config_tests.cc): default value (`CepGateThreshold == 1.25f`); explicit override propagates; canonical default in repo-root autoc-tracker.ini. Out-of-range `CepGateThreshold = -1.0` exits with clear error (loud-fail; not unit-testable without process fork). `EnableDerivedFeatures` knob dropped 2026-05-16 (see T007)

### Implementation for User Story 1

> **Run tests T009-T013 after T003-T008 land; confirm they FAIL (count assertions fail because slots aren't populated yet, etc.) before proceeding.**

Implementation cadence (operator preference 2026-05-16): land the **minisim path** first, smoke-test it end-to-end against the unit tests AND a small bake, THEN mirror to crrcsim. Catches design bugs in the cheap path before duplicating; crrcsim mirror becomes a mechanical port once the algorithm is validated.

#### Step 1 — Minisim path implementation + smoke

- [X] T014 [US1] Implement span computation in `projectAndShiftHistory` in [src/eval/tracker_stepper.cc](src/eval/tracker_stepper.cc) (autoc minisim path only — crrcsim mirror comes later in T019): add `span` to the history shift loop; after writing NDC slot[5], compute CEP-gated `history_.span[5]` per [contracts/gather_tracker_inputs_v54.md](./contracts/gather_tracker_inputs_v54.md) "`projectAndShiftHistory` companion extension" section (gp_scalar intermediates, raw-ok cast at slot write)
- [X] T015 [US1] Extend `gather_tracker_inputs` in [src/nn/evaluator.cc](src/nn/evaluator.cc) per [contracts/gather_tracker_inputs_v54.md](./contracts/gather_tracker_inputs_v54.md) §1-§3: copy `history.span[i]` → `out.beacon_pair_span[i]`, compute `out.span_rate = history.span[5] - history.span[4]`, compute tilt sin/cos with CEP-gate + degenerate-epsilon guard (gp_scalar intermediates, raw-ok-annotated slot writes). Depends on T006, T014. `EnableDerivedFeatures` zero-fill branch was dropped 2026-05-16 (see T007)
- [X] T016 [US1] Wire `CepGateThreshold` (from T008) into the constant or config struct that `gather_tracker_inputs` and `projectAndShiftHistory` consume (single source of truth across both consumers). Replace any hardcoded `1.25f` literals with the configured value
- [X] T017 [US1] Build (incremental) + run the full GoogleTest suite — T009-T013 unit/contract tests must now pass against the minisim path (T011's gather_tracker_inputs cases are minisim-agnostic at the math layer, so they cover both paths' shared logic). Existing 030 contract tests MUST still pass. Per [feedback_incremental_build_default](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_incremental_build_default.md) use the incremental build, not a full rebuild
- [ ] T018 [US1] Run a short **minisim-FDM** smoke bake against [autoc-tracker-minisim.ini](autoc-tracker-minisim.ini) (already wired to `MinisimProgram = ./build/minisim`); operator launches `stdbuf -oL -eL ./build/autoc -i autoc-tracker-minisim.ini` and Ctrl-Cs after a few gens. Inspect resulting dmp with `./build/tracker_dmp_inspect <dmp-key> -i autoc-tracker-minisim.ini` per [quickstart.md "Pre-flight check"](./quickstart.md#pre-flight-check--do-not-run-a-no-op-bake); verify the 9 new slots (`spn-5..spn0 dspn tltS tltC`) carry non-trivial values across ticks. If slots are all-zero, the gather pipeline didn't pick up the new code — abort and debug before doing the crrcsim mirror. (Note: `tracker_dmp_inspect` may need extending to print the new slots if its current per-tick dump truncates at 45 — verify first.)

#### Step 2 — CRRCSim mirror + smoke

- [ ] T019 [US1] Mirror the same `projectAndShiftHistory` extension in [crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp](crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp): identical history-shift addition + span computation, byte-equivalent to T014. Depends on T014 (use it as the canonical reference)
- [ ] T020 [US1] Build (incremental, includes crrcsim subdirectory) + re-run the full GoogleTest suite to confirm crrcsim mirror hasn't broken anything (tests are mostly minisim-side but build-stability is the gate)
- [ ] T021 [US1] Run a short **crrcsim-FDM** smoke bake (3 gens, small pop; `MinisimProgram = ./scripts/crrcsim.sh` per default `autoc-tracker.ini`); inspect resulting dmp same as T018; verify slots populated AND values are reasonably close to the T018 minisim smoke (not bitwise — different FDMs — but qualitatively in the same ballpark for span / tilt distributions)

#### Step 3 — dmp honesty + docs

- [ ] T022 [P] [US1] Audit dmp emission path to confirm all 54 NN inputs + 3 outputs are captured per [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md) and [spec.md Q8](./spec.md#clarifications): `grep -rn "NNData\|setNNData\|TrackerInputs" src/ include/` to trace the emission; if any of the 9 new slots are not serialized to dmp, either fix the emission or write a `// dmp-honesty: <rationale>` note explaining the intentional exclusion. The schema bump is the audit boundary
- [ ] T023 [P] [US1] Update [docs/COORDINATE_CONVENTIONS.md](docs/COORDINATE_CONVENTIONS.md) per [contracts/identity_invariant.md](./contracts/identity_invariant.md) "Doc deliverables": add "Beacon Identity-Stable Ordering" section (mount convention, invariant text, pointer to contract, future-xiao failure-mode story); add tilt convention (θ = 0 = wings level relative, sin/cos encoding, atan2 of (port → starboard) NDC line)
- [ ] T024 [P] [US1] Update [docs/sensor-pipeline.md](docs/sensor-pipeline.md) per [contracts/identity_invariant.md](./contracts/identity_invariant.md): add identity-stable slot mapping section cross-referencing COORDINATE_CONVENTIONS.md; document CEP-gating + neutral-substitution rule from spec Q4 (default threshold 1.25, neutral values: span=0, span_rate=0, tilt=(0,1)); call out xiao-migration-prep purpose

### Pre-bake validation (full regression gate)

- [ ] T025 [US1] Run `bash scripts/rebuild-perf.sh` (full clean rebuild + perf-deterministic build per [reference_perf_build_reproducibility](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_perf_build_reproducibility.md)) and verify M1 (pathgen) fitness numbers remain **bitwise-equal** to the T001 baseline. M2 numbers WILL change by design (input vector grew). Fix any M1 regression as a P0 bug before continuing. **Operator-driven** per [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md)
- [ ] T026 [US1] **M1-under-CRRCSim regression** (separate from T025's minisim bitwise gate): if the M1 pathgen NN config can be run under crrcsim FDM (check `autoc.ini` for `MinisimProgram` setting; if it points to minisim, copy to a sibling `autoc-m1-crrcsim.ini` with `MinisimProgram = ./scripts/crrcsim.sh`), run a short M1 bake (few gens, small pop) under crrcsim and compare gen-0 + gen-1 fitness numbers to a pre-032 baseline (operator records baseline from a pre-032 commit). Numbers will NOT be bitwise-identical (crrcsim FDM physics differs from minisim) but the trajectory shape / gen-over-gen progression must look qualitatively unchanged. If M1 cannot run under crrcsim (no path exists today), document that finding and skip this task — phase 1's no-touch invariant for M1 is then bounded by T025 alone
- [ ] T027 [US1] Determinism sanity per [quickstart.md "Determinism sanity"](./quickstart.md#determinism-sanity-one-time-per-phase-1-build): re-run T018 or T021 smoke twice with identical seed; diff the per-gen `#GenDiag` lines — must be bitwise-identical (validates 032 hasn't introduced new non-determinism, e.g., uninitialized memory in the new derived-feature path)

### Phase 1 bake

- [ ] T028 [US1] Launch the production phase-1 combined bake per [quickstart.md "Bake — phase 1 combined run"](./quickstart.md#bake--phase-1-combined-run): `stdbuf -oL -eL ./autoc -i autoc-tracker.ini 2>&1 | tee specs/032-tracker-nn-enhancements/bake.log`. **Operator-driven** — do NOT autonomously launch this command per [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md); the task descriptor exists for tracking, the operator confirms when to fire it. Bake at least to gen 322
- [ ] T029 [US1] After bake completes, compute plateau-avgInRamp per [quickstart.md "Read result"](./quickstart.md#read-result--plateau-avginramp): average avgInRamp across the last 50 gens of `#GenDiag` lines. Record the number
- [ ] T030 [US1] Apply outcome decision rule per [quickstart.md "Outcome decision"](./quickstart.md#outcome-decision--spec-q7) and [spec.md Q7](./spec.md#clarifications): ≥0.15 → US1 SUCCESS, skip US2, route to Polish; 0.10–0.15 → US1 PARTIAL, proceed to US2; <0.10 → US1 MISS, document and route to phase 2/M3 (skip US2)
- [ ] T031 [US1] Write [specs/032-tracker-nn-enhancements/outcome.md](specs/032-tracker-nn-enhancements/outcome.md) capturing: bake commit hash, plateau-avgInRamp number, decision (SUCCESS / PARTIAL / MISS), next-milestone routing. Include 2-3 representative scenario playback observations if PARTIAL or MISS (informs phase-2 routing)

**Checkpoint**: Phase 1 SUCCESS (≥0.15) → MVP complete, jump to Polish phase. PARTIAL (0.10–0.15) → proceed to Phase 4 (US2). MISS (<0.10) → write phase-2-routing notes in outcome.md, skip US2, jump to Polish.

---

## Phase 4: User Story 2 — Phase-1b B-off attribution bake (Priority: P2, CONTINGENT)

**Goal**: When US1 lands in the 0.10–0.15 partial band, run a B-off attribution bake from a **git-reverted pre-032 commit** to isolate which sub-feature carried the lift per [research.md R7](./research.md#r7--attribution-bake-methodology-phase-1b-contingency-from-spec-q7). No runtime flag — greenfield M2 means the code IS the change; git already provides the revert mechanism. A-only attribution is NOT run in sim (it's equivalent to the existing 030 baseline; pre-032 IS A-only-without-B).

**Independent Test**: Bake completes ≥322 gens against a pre-032 commit (no derived-feature code present), plateau-avgInRamp readout produced, comparison to US1 (032) result yields a clear attribution call (derived features load-bearing / not load-bearing / mixed).

**ONLY EXECUTE if US1 result is in [0.10, 0.15) per T030 decision.**

### Implementation for User Story 2

- [ ] T032 [US2] Capture the current 032 commit so the operator can return after the attribution bake: `git rev-parse HEAD > /tmp/032-head.txt`. Then check out the last pre-032 commit (e.g., `git checkout 8bfad02` — the 030 closeout commit; verify with `git log --oneline -5` first that this is the right pre-032 SHA). The pre-032 code has no `TrackerInputs::beacon_pair_span` / `span_rate` / `target_tilt_*` and `TrackerInput::COUNT == 45`
- [ ] T033 [US2] Rebuild against pre-032: `bash scripts/rebuild-perf.sh`. Then launch the B-off bake (operator-driven, NOT autonomous): `stdbuf -oL -eL ./build/autoc -i autoc-tracker.ini 2>&1 | tee specs/032-tracker-nn-enhancements/bake_b_off.log`. Same source dmp, pop, gens, seed as US1's bake (T028) — only difference is the absence of the 032 derived-feature code path
- [ ] T034 [US2] Compute plateau-avgInRamp for the B-off bake via the same averaging protocol as T029
- [ ] T035 [US2] Update [specs/032-tracker-nn-enhancements/outcome.md](specs/032-tracker-nn-enhancements/outcome.md) with B-off plateau-avgInRamp + interpretation per [quickstart.md "Attribution bake" decision table](./quickstart.md#attribution-bake--b-off-mode-only-if-partial-band-result):
   - B-off ≈ 030 baseline (~0.07) → derived features (B) are load-bearing; refine B before phase-2 escalation
   - B-off ≈ US1 plateau → A carried most of the lift (anomalous since A is a sim no-op — investigate)
   - B-off between baseline and US1 → both contribute; document the split
- [ ] T036 [US2] Return to the 032 head and rebuild: `git checkout "$(cat /tmp/032-head.txt)" && bash scripts/rebuild-perf.sh`. Verify the working tree is back on 032 before continuing to Polish

**Checkpoint**: Attribution call recorded in outcome.md. Phase-2 routing decision is now informed by which sub-feature is load-bearing. Working tree restored to 032.

---

## Phase 4b: User Story 3 — Kamikaze hull-crash penalty experiment (Priority: P2, CONTINGENT/PARALLEL to US2)

**Goal**: Address the hull-strike escalation finding (spec.md §1.8) — controllers learn that aggressive close-tracking with occasional hull crashes is fitness-positive at scale; that's both a sim-fitness pathology AND a deal-breaker for safe real-flight deployment. Test whether a multiplicative ½-of-accumulated-score penalty on hull crash (kamikaze framing) shifts evolution toward equal-or-better tracking with fewer crashes.

**Independent Test**: Bake completes ≥322 gens with kamikaze penalty active; per-gen hull-strike count compared against US1 baseline (no penalty) at matched gen windows; plateau-avgInRamp also compared. Success criteria: hull-strike rate reduced ≥2× at gen 300+ vs US1, with plateau-avgInRamp degraded by <20%. (Sharper criteria can be set when operator looks at the US1 trajectory.)

**ONLY EXECUTE after US1 closeout** (T031 outcome.md written). Independent of US1 outcome — runs regardless of SUCCESS / PARTIAL / MISS, because the hull-penalty is also gating real-flight safety, not just plateau quality.

### Implementation for User Story 3

- [ ] T043 [US3] Add `HullCrashScoreFactor` to `[CrashHull]` section of [autoc-tracker.ini](autoc-tracker.ini) (default 1.0 = no change, US1 behavior; experiment value 0.5 = half the work doesn't count on crash). Mirror the addition in [autoc-tracker-minisim.ini](autoc-tracker-minisim.ini)
- [ ] T044 [US3] Add `hullCrashScoreFactor` field to `AutocConfig` in [include/autoc/util/config.h](include/autoc/util/config.h) (default 1.0); parse + range-check [0.0, 1.0] in [src/util/config.cc](src/util/config.cc); loud-fail on out-of-range
- [ ] T045 [US3] Thread `hullCrashScoreFactor` into the scenario aggregator. Locate the scenario-score reduction site in `computeScenarioScores` / `aggregateRawFitness` (grep `src/eval/` for the scoring path); when `crashReason == HullStrike`, multiply the accumulated scenario score by `hullCrashScoreFactor` before aggregation. Per-scenario (NOT per-tick) decision
- [ ] T046 [US3] Wire `hullCrashScoreFactor` through `WorkerInit` (crrcsim worker is separate process; no ConfigManager). Mirror the pattern from T008's `cepGateThreshold` wiring
- [ ] T047 [US3] Add unit test in [tests/fitness_decomposition_tests.cc](tests/fitness_decomposition_tests.cc) or new `tests/hull_penalty_tests.cc`: synthetic scenario ending in HullStrike with accumulated score S → factor 0.5 → aggregated = 0.5 × S; factor 1.0 → aggregated = S (no change, US1 behavior)
- [ ] T048 [US3] Add contract test in [tests/contract_tracker_config_tests.cc](tests/contract_tracker_config_tests.cc): default 1.0; explicit 0.5 parses; out-of-range (e.g., -0.1, 1.5) loud-fails at startup
- [ ] T049 [US3] Build + test suite green (`cd build && make -j8`); regression-gate via `bash scripts/rebuild-perf.sh` — M1 (pathgen, no hull) must remain bitwise-equal; M2 with `HullCrashScoreFactor = 1.0` (default) must remain bitwise-equal to US1's outcome
- [ ] T050 [US3] Set [autoc-tracker.ini](autoc-tracker.ini) `[CrashHull] HullCrashScoreFactor = 0.5`. Launch the experiment bake (operator-driven, NOT autonomous): `nohup stdbuf -oL -eL ./build/autoc -i autoc-tracker.ini > logs/autoc-032-phase1b-kamikaze-crrcsim.log 2>&1 < /dev/null & disown`. Same source dmp, pop, gens as US1's bake (T028) — only difference is the penalty factor
- [ ] T051 [US3] Bake ≥322 gens; compute plateau-avgInRamp (last 50 gens avg, same protocol as T029); also extract per-gen hull-strike count via the awk in spec.md §1.8 table
- [ ] T052 [US3] Update [specs/032-tracker-nn-enhancements/outcome.md](specs/032-tracker-nn-enhancements/outcome.md) with:
   - US3 plateau-avgInRamp vs US1 plateau-avgInRamp
   - per-gen hull-strike trajectory comparison (US3 vs US1 at gen 50/100/200/300+ windows)
   - decision: kamikaze penalty wins / loses / mixed; route to xiao deployment from US1 or US3 NN
- [ ] T053 [US3] Revert [autoc-tracker.ini](autoc-tracker.ini) to `HullCrashScoreFactor = 1.0` (don't commit experiment-state as default; if US3 wins, raise it as a separate config decision)

**Checkpoint**: Hull-penalty experiment decided. Real-flight-safe NN identified (either US1's or US3's). Either way, document the trade-off in outcome.md for the next sensor-architecture iteration to inherit.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Closeout artifacts, audits, and optional refactoring.

- [ ] T037 Run Constitution VI grep audit per [Constitution VI verification cadence](../../.specify/memory/constitution.md): `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -E '// raw-ok:'` — every remaining hit on a touched file must be annotated or converted to `gp_scalar` / `gp_fitness` / `gp_vec3` / `gp_quat`. No milestone is "done" with unannotated hits in the 032 diff
- [ ] T038 [P] Write `specs/032-tracker-nn-enhancements/SMOKE_REPORT.md` summarizing: bake duration, gen count, plateau-avgInRamp, B-off result (if run), unexpected behaviors observed in playback, dmp-honesty audit result (T022), Constitution VI grep result (T037)
- [ ] T039 [P] Generate evolution-progress / gen-diag / per-axis charts via the existing `specs/030-tracker-mode/plot_*.py` scripts, parameterized on the 032 bake name. Drop output PNGs in `specs/032-tracker-nn-enhancements/`
- [ ] T040 Update [project memory](../../.claude/projects/-home-gmcnutt-autoc/memory/) — write or update a `project_032_phase1_result.md` capturing the plateau-avgInRamp number, the decision, and any non-obvious findings (e.g., bang-bang axis migration per [project_bangbang_axis_migration](../../.claude/projects/-home-gmcnutt-autoc/memory/project_bangbang_axis_migration.md), elite divergence patterns, etc.). Add the new memory to `MEMORY.md` index
- [ ] T041 OPTIONAL refactor: consolidate the two `projectAndShiftHistory` implementations (autoc minisim + crrcsim helper) into a single shared helper per [data-model.md §8](./data-model.md#8-cross-platform-mirroring) wire-equivalent obligation. Only do this if T025 + T037 all pass cleanly AND there's operator agreement to take the cleanup hit in this milestone (otherwise file as a backlog item)
- [ ] T042 Commit closeout per [feedback_submodule_merge_order](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_submodule_merge_order.md): if any crrcsim submodule changes (T019), bump submodule pointer first, then parent commit. Commit message references spec.md + outcome.md + plateau-avgInRamp + decision

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: T001 must pass cleanly (baseline regression gate); T002 is read-only doc inspection
- **Foundational (Phase 2)**: All of T003-T008 must land. T003-T006 in `nn_inputs.h` + `tracker_stepper.h` are colocated and have natural compile-time linkages. T007-T008 are config-side, semi-independent
- **US1 (Phase 3)**: Depends on Phase 2 complete. Tests T009-T013 land first and fail. Then **minisim path** (T014-T018) lands and is validated end-to-end via smoke + unit tests. Then **crrcsim mirror** (T019-T021). Then docs + audits (T022-T024). Then full regression gate (T025-T027). Then operator-gated bake (T028-T031)
- **US2 (Phase 4)**: Depends on US1 PARTIAL outcome (T030 decision). Skipped on SUCCESS or MISS
- **Polish (Phase 5)**: Depends on US1 complete (T031); US2 if applicable (T035)

### User Story Dependencies

- **US1**: Pure dependency on Phase 2 foundational scaffolding. Self-contained MVP
- **US2**: Contingent on US1 outcome — does NOT pre-fork the bake; only kicks in for partial-band result

### Within Each Phase

- Tests (T009-T013) MUST be written and FAIL before implementation (T014+). Per Constitution I
- Schema (T003-T006) before config (T007-T008): the config knob is meaningless if the slot count is wrong
- **Minisim-first cadence (operator preference 2026-05-16)**: T014-T018 land + smoke before T019-T021. Catches design bugs in the cheap path before duplicating into crrcsim. The crrcsim mirror (T019) becomes a near-mechanical port once T014's algorithm is validated
- T015 (gather extension) depends on T006 (history window has span) and T014 (something populates span). It does NOT depend on T019 (crrcsim mirror) — the math layer is FDM-agnostic
- T017 (build + tests after minisim impl) gates the smoke (T018); T018 must demonstrate non-zero slot values in the dmp before paying the crrcsim-mirror cost
- T020 (build after crrcsim mirror) is a build-stability check; T021 is the crrcsim-FDM smoke (qualitative parity with T018's minisim smoke)
- T025 (rebuild-perf.sh M1 bitwise gate) and T026 (M1-under-CRRCSim regression) BOTH guard against M1-side contamination. T025 is the bitwise gate; T026 catches any M1↔crrcsim path breakage that minisim wouldn't catch
- Validation (T025-T027) before production bake (T028): the bake is expensive — clean rebuild + cross-FDM M1 regression + determinism check first
- Operator gates T001 (baseline rebuild), T025 (rebuild-perf), T026 (M1-on-crrcsim re-run), T028 (production bake), T033 (B-off bake) — assistant does NOT autonomously launch per project policy

### Parallel Opportunities

- T002 parallel with T001 (independent doc-read vs build)
- T003-T006 are in different sections of the same header but functionally serial (each depends on the prior to compile cleanly); could batch as one edit
- T007 and T008 can run in parallel (ini text edit vs C++ loader edit, different files)
- T009-T013 are all in different test files → all 5 can be authored in parallel
- T022, T023, T024 (dmp audit + 2 doc updates) are independent files → can be authored in parallel after the smoke tests pass
- T038 (SMOKE_REPORT.md) and T039 (chart generation) operate on separate outputs → parallel
- T014 + T019 are NOT marked [P] anymore — operator preference is sequential (validate minisim first, then mirror). Override only if the smoke contract is clear enough that the cost of debugging two FDMs at once is acceptable

---

## Parallel Example: US1 test authoring

```bash
# Launch all 5 US1 test artifacts in parallel (different files):
Task: "Create tests/derived_features_tests.cc with pure-math unit tests" (T009)
Task: "Extend tests/nn_sensor_interface_tests.cc with TrackerInput::COUNT==54 + new entries" (T010)
Task: "Extend tests/gather_tracker_inputs_tests.cc with derived-feature end-to-end cases" (T011)
Task: "Add identity-invariant contract test to tests/gather_tracker_inputs_tests.cc" (T012, same file as T011 — author sequentially or merge)
Task: "Add config-loader test for [DerivedFeatures] section" (T013)
```

(Note: T011 and T012 touch the same test file; merge into a single PR or sequence them — don't claim parallelism between them.)

---

## Implementation Strategy

### MVP First (US1 only)

1. **Phase 1: Setup** (T001-T002) — verify clean baseline regression gate
2. **Phase 2: Foundational** (T003-T008) — schema + config plumbing
3. **Phase 3: US1 — tests-first** (T009-T013) — write all 5 test artifacts; confirm they fail
4. **Phase 3: US1 — minisim path** (T014-T018) — implement in autoc minisim only; build + tests + minisim-FDM smoke; **STOP if smoke shows zero-filled slots** — debug before duplicating
5. **Phase 3: US1 — crrcsim mirror** (T019-T021) — mechanical port; build + tests + crrcsim-FDM smoke (qualitative parity with T018)
6. **Phase 3: US1 — docs + dmp audit** (T022-T024)
7. **Phase 3: US1 — full regression gate** (T025-T027) — rebuild-perf.sh M1 bitwise + M1-under-CRRCSim regression + determinism sanity
8. **Phase 3: US1 — operator-gated production bake** (T028-T031)
9. **STOP and VALIDATE**: T030 outcome decision
10. SUCCESS → skip US2, run Polish; PARTIAL → run US2; MISS → document phase-2 routing in outcome.md, skip US2

### Incremental Delivery

The "increment" here is the bake outcome, not multiple shippable user-facing features. Phase 1 ships when the combined bake's plateau-avgInRamp is captured and the decision is recorded; phase 1b (US2) is a follow-on attribution-only delivery, not a separate user-facing increment.

### Parallel Team Strategy

This is a single-operator project. Parallelism is across files within a single operator's session, not across teammates. The [P] markers indicate "safe to author in parallel via tool calls in the same message" — useful for cutting down round-trips, not for staffing.

---

## Notes

- **Tests are mandatory** per Constitution I — write T009-T013 first, confirm they fail, then implement T014-T016 + T019
- **No version bump** for the dmp schema growth (M2 policy reinforcement 2026-05-16); fail-loud cereal archive-length mismatch is the safety net per [research.md R5](./research.md#r5--dmp-schema-growth-without-version-bump-m2-policy-reinforcement) and [Constitution V](../../.specify/memory/constitution.md)
- **M1 (pathgen) must remain bitwise-equal** — T025 is the gate; any M1 regression is a P0 fix-first item
- **Operator gates the production bake** (T028) — assistant tracks the task but does not autonomously launch per [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md)
- **Incremental builds** for iteration (`scripts/rebuild.sh` or in-place CMake build); reserve `rebuild-perf.sh` for explicit gate-confirming rebuilds per [feedback_incremental_build_default](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_incremental_build_default.md)
- **Submodule merge order** (T042): if crrcsim submodule pointer needs to advance (T019 touches crrcsim/), bump submodule first then parent per [feedback_submodule_merge_order](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_submodule_merge_order.md)
- **Feature A in sim is a no-op** — T012 documents the invariant via a contract test; T023-T024 document it for the future xiao port. No sim code change is added for Feature A in phase 1
- **Commit cadence**: commit after each logical group (Phase 2 batch; tests batch; implementation batch; doc batch; smoke validation; bake closeout). Avoid one giant 032 commit
