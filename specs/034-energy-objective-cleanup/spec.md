# Feature Specification: M1/M2 Cleanup + Craft Variations → Flight Test

**Feature Branch**: `034-energy-objective-cleanup` (branch name now historical — energy split to 035)
**Created**: 2026-05-29
**Status**: Draft (scope expanded then re-split 2026-05-29 — operator routing)
**Input**: User description: "034 — M1 cleanup + energy objective revisit (continues from 033 wrap)", expanded 2026-05-29 to add minisim teardown + craft variations + cross-cutting tech-debt fold-ins, then **re-split**: the energy-objective investigation moved to [035](../035-energy-lexicase-objective/spec.md). 034 is now scoped as **tech-debt cleanup + craft variations, up to and including a flight test** — getting real mileage out of the variations + cleanup before the open multi-objective question.

## Context

033 wrapped with two outcomes: its replay-PRNG architecture shipped and validated (kept), but its M1 smoothness penalty (a scalar multiplicative factor on `stepPoints`) failed — a scalar-aggregated multi-objective collapses the controller into one Pareto corner. The smoothness plumbing is fully wired across every transport but configured as a no-op.

034 uses the fact that the team is **resetting all training and breaking the dmp + data.dat schema anyway** to clear a batch of coupled tech debt and land the long-deferred airframe-diversity work in one pass, rather than re-paying those costs later. The energy-objective investigation (the original 034 headline) is split to 035 so this feature can ship clean and fly. 034 has four jobs plus a flight-test terminus:

1. **Tear out minisim** — commit to crrcsim as the sole worker path, collapsing the "BOTH minisim AND crrcsim" tax that burdens several transport items. *(Done first — makes everything downstream single-path.)*
2. **Remove the dead smoothness apparatus** — honest codebase about what M1 optimizes.
3. **Clear the cross-cutting tech-debt fold-ins** — config auto-print, seed-width, variation-table, crash-hull PRNG determinism, mod_inputdev link fragility, S3 run-id disambiguation, EvalResults return-path trim. All cheap *now* because the schema/transport/single-path costs are already being paid.
4. **Add craft variations to the scenario mix** — CG, drag, servo responsiveness, total power, trim — the long-deferred sim-to-real airframe-diversity work (015 Phase 6).
5. **Bake + flight test** — produce a craft-variation-trained controller (M1 → M2) and fly it.

Jobs 1–3 are tech-debt cleanup and are sequenced **up front**; job 4 is the feature work; job 5 is the deliverable that proves the mileage.

## Clarifications

### Session 2026-05-29

- Q: Craft-variation scope — which bake(s) apply it? → A: Only the CRRCSim-simulated NN-controlled craft (the "chase"), in BOTH pathgen (M1) and tracker (M2) modes. Craft variations *are* CRRCSim physics changes, so they always apply to the FDM-simulated controlled craft. The target/source trajectory is not independently varied — M1 source diversity emerges because M1 sources are flown by the varied chase craft; M2 supplies own-airframe robustness directly.
- Q: Craft-variation sampling distribution & magnitude-knob units? → A: Per-parameter Gaussian, magnitude = fractional σ (multiplicative on each param's nominal, e.g. 0.05 = 5%). σ=0 → exactly nominal (clean no-op for FR-021). Matches autoc's existing σ-based variation knobs (rabbit-speed σ, entry-cone σ).
- Q: 034's definition of done & ownership of the US5 bakes/flight? → A: Implementation boundary = code-complete through US1–US4 with tests green and the fixed-eval determinism gate passing. The US5 M1/M2 bakes + flight test are operator-run acceptance milestones that gate feature *closure* but are tracked separately from implementation tasks (per `feedback_operator_runs_regression_gate`).
- Q: (D1) Craft-param scope — servo handling? → A: Defer servo *lag* (a new dynamical element, pt3-filter training-stunt risk). Instead vary **pitch & roll control effectiveness** (independent per-axis → slight asymmetry), which are settable control-authority aero coefficients (`Cm_de`/`CL_de` pitch, aileron→roll coeff) at the same per-scenario init hook as CG/drag/trim. Final craft set: CG, drag, trim, thrust-scale, pitch-effectiveness, roll-effectiveness.
- Q: (D2) Are FR-013 (crash-hull PRNG) / FR-014 (mod_inputdev link) real or vestigial? → A: Both already satisfied in-tree (verified). Crash-hull already seeds from `deriveClassSubSeeds(scenarioSeed).rabbit` (canonical deterministic route; the memory's "seed from windSeed" is stale — windSeed was removed). mod_inputdev already links `autoc_common`. **Both dropped from 034 scope.** (A separate, out-of-scope non-determinism source remains: wall-clock `multiloop`/`simTimeMsec` divergence noted in crrcsim/CLAUDE.md.)

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Tear out minisim; crrcsim is the sole worker path (Priority: P1, tech-debt, first)

The project continues with crrcsim (FDM-grade physics) and retires the minisim worker entirely. A developer should find one worker path, not two. Minisim is removed from the code, the build, the tests, and as an ini option.

**Why first:** Minisim is currently one of two live worker paths (WorkerInit priming, tracker/pathgen steppers, and the smoothness mirror all exist in both `tools/minisim.cc` and `crrcsim/src/mod_inputdev/`). Removing it first makes every downstream transport edit (smoothness removal, fold-ins, craft-variation plumbing) a *single-path* sweep — a force-multiplier on the rest of 034.

**Independent Test:** `grep -rn minisim` across `CMakeLists.txt`, `tests/`, `src/`, `tools/` returns zero live references (a historical note is acceptable). autoc + crrcsim build clean; the full suite passes with minisim-specific tests deleted (not disabled). No minisim worker option remains.

**Acceptance Scenarios:**

1. **Given** the 033 codebase with minisim + crrcsim worker paths, **When** minisim is removed, **Then** `tools/minisim.cc` is gone, `CMakeLists.txt` no longer builds it, and the minisim-coupled test files (`tests/rpc_transport_tests.cc`, `tests/tracker_stepper_init_tests.cc`) are updated to the crrcsim-only shape and pass.
2. **Given** a tracker or pathgen bake, **When** it launches after removal, **Then** it runs on crrcsim with no behavioral change to the crrcsim path.
3. **Given** the shared stepper/RPC/priming code minisim and crrcsim both consumed, **When** minisim is gone, **Then** code that existed *only* to serve minisim is removed, and shared code retained for crrcsim is left bit-behavior-identical.

---

### User Story 2 — Remove the dead M1 smoothness apparatus (Priority: P1, tech-debt)

A developer reading the M1 fitness/transport code should not encounter a fully-wired-but-inert smoothness mechanism the project has decided not to pursue. The smoothness factor, its per-tick computation, its propagation across the autoc↔sim transports, its dmp/data.dat persistence, and its ini knobs should all be gone.

**Why this priority:** Dead no-op plumbing spanning ~31 files and five transport boundaries (config → WorkerInit/EvalData → crrcsim worker → AircraftState dmp → data.dat) is active maintenance drag and a correctness trap (re-enabling `SmoothnessPenaltyFloor` would resurrect the known-bad scalar mechanism). Sequenced after US1 so the sweep is crrcsim-only.

**Independent Test:** After removal, grep for `smoothness`/`SmoothnessPenaltyFloor`/`smoothnessFactor`/`compute_smoothness_factor` finds zero live references. Build is green; all tests pass. A fresh M1 bake produces identical fitness to a pre-removal bake at `SmoothnessPenaltyFloor=1.0` (the no-op value).

**Acceptance Scenarios:**

1. **Given** the 033 codebase with smoothness wired no-op, **When** the apparatus is removed, **Then** autoc + crrcsim build clean and the full test suite passes (with smoothness-specific tests deleted, not disabled).
2. **Given** a fixed seed + config, **When** an M1 eval runs before removal (floor=1.0) and after removal, **Then** per-scenario fitness is identical — removal changed no live behavior.
3. **Given** the dmp schema after removal, **When** a fresh bake writes a gen dmp, **Then** it round-trips through the renderer/eval tools (greenfield schema change — pre-034 dmps are NOT expected to load, per project no-versioning-shim policy).

---

### User Story 3 — Cross-cutting tech-debt fold-ins (Priority: P1, tech-debt)

A batch of backlog items that become cheap because 034 is already breaking the schema, touching every transport, collapsing to a single worker path, and reworking the scenario-PRNG cascade. Clearing them now avoids re-paying those costs and removes recurring friction (notably the config-not-logged gap that bit the team repeatedly during 033 debugging).

**Why this priority:** Pure tech-debt cleanup, grouped with US1/US2 up front. Two items are elevated by the craft-variation work that follows: the **AutocConfig auto-print** (craft variations add ~5 new knobs — exactly the "next milestone adds 5+ knobs" trigger), and the **crash-hull PRNG fix** (rides the same scenario-PRNG cascade rework the craft seed needs).

**Independent Test:** Each sub-item is independently verifiable.

**Acceptance Scenarios:**

1. **Given** any bake launch, **When** the startup banner prints, **Then** every active `AutocConfig` key is shown via a single-source-of-truth mechanism (X-macro or equivalent), so a run's exact config — including the new craft-variation knobs — is recoverable from its log alone, with no hand-maintained print block to drift.
2. **Given** the seed cascade, **When** an operator pastes back a logged master seed, **Then** there is no silent truncation risk — seed widths match actual entropy and the `int seed = -1` overflow edge is guarded, without altering the PRNG sequence for in-range seeds.
3. **Given** a `SimNumPathsPerGeneration=1, WindScenarios=N` config, **When** scenarios are generated, **Then** each is a distinct (path, wind) variation — OR this sub-item is closed as "no real gap exists" with evidence.
4. ~~crash-hull PRNG seeding~~ — DROPPED (D2): verified already seeded from `deriveClassSubSeeds(scenarioSeed).rabbit`; no work needed.
5. ~~mod_inputdev → autoc_common~~ — DROPPED (D2): verified already linked; no work needed.
6. **Given** M1 (pathgen) and M2 (tracker) dmps in the same S3 bucket, **When** a tool auto-picks "latest run," **Then** run-id prefixes (`autoc-` vs `tracker-`) disambiguate them so the wrong kind is never silently selected.
7. **Given** the sim→autoc return path, **When** a non-elite eval completes, **Then** it returns a trimmed score-only result (full `EvalResults` only for elite-reeval), reducing autoc-side working set — now a single-path (crrcsim-only) change.

---

### User Story 4 — Craft variations into the scenario mix (Priority: P1, feature)

The project is approaching real-flight generalization and needs the NN-controlled craft to span an airframe-parameter envelope, not just one nominal craft. This story adds **craft variations** — CG, drag, trim, total power (thrust-scale), and pitch/roll control effectiveness — as deterministic per-scenario variations applied to the **CRRCSim-simulated NN-controlled craft (the "chase") in both pathgen and tracker modes**, each with an ini-configurable magnitude. (Servo *lag* is explicitly deferred — see Clarifications D1; control *effectiveness* substitutes for it, varying pitch and roll authority independently for slight asymmetry.) Because CRRCSim only simulates the controlled craft, craft variations *are* CRRCSim physics changes; the target/source trajectory is not independently varied (M1 source diversity emerges from sources being flown by the varied chase craft).

**Why this priority:** The long-deferred 015 "Phase 6: Aircraft parameter variation (sim-to-real)" work and the craft half of the BACKLOG "Forward loop — craft + camera variations into M1." Landing it during the reset means the airframe-diversity dimension is in place before the M1/M2 bakes and flight test that follow.

**Key design constraints (from operator):**
- **Craft variations ramp like wind/entry** (decision 2026-05-31, see [stuck-basin-bisect](stuck-basin-bisect.md) + design discussion). The draw itself is full-magnitude and deterministic per `scenarioSeed` (replay invariant); the per-eval application is multiplied by `computeVariationScale()` so early-gen NNs see near-nominal airframes and full diversity emerges by ramp end. Eval mode replays the saved `genome.variation_scale` (already plumbed via `gEvalVariationScaleOverride`), so an extracted weight reproduces its training-time airframe magnitude bit-for-bit — not 100%, not recomputed. Consistency with the existing `applyVariationScale()` pipeline; no special-case for the operator.
- **Mixed into the scenario set with determinism.** Each scenario's craft parameters are a joint-PRNG sample alongside its existing (path, wind, entry, rabbit) draw, per `project_variation_design_principles`.
- **Capture the craft seed in `ScenarioMetadata`.** At minimum the seed used to set the craft parameters must be recorded so the airframe draw is reproducible/replayable.
- **Each varied parameter gets an ini magnitude knob** (CG, drag, trim, thrust-scale, pitch-effectiveness, roll-effectiveness). Pitch and roll effectiveness vary independently so the airframe can be slightly asymmetric. Servo *lag* (control-rate time-constant) is deferred to a follow-on.
- **Camera variations are out of scope** — mostly an M2 concern, added in a later iteration. Only the craft-parameter dimension lands now; the seed cascade is built so camera-seed capture can join later without re-breaking the schema. ("We will revisit what we vary.")

**Independent Test:** With craft-variation magnitudes non-zero, two scenarios drawing different craft seeds fly measurably different airframes; the same seed reproduces the identical airframe bit-for-bit; the craft seed appears in `ScenarioMetadata` and round-trips through the dmp. With all magnitudes zero, behavior is identical to the nominal craft (no-op guard).

**Acceptance Scenarios:**

1. **Given** non-zero craft-variation magnitudes, **When** a scenario set is generated, **Then** each scenario carries a deterministic craft draw (CG, drag, servo responsiveness, total power, trim) jointly sampled with its other variation dimensions, and the craft seed is recorded in `ScenarioMetadata`.
2. **Given** a recorded craft seed, **When** the scenario is regenerated, **Then** the airframe parameters reproduce bit-for-bit (determinism gate holds).
3. **Given** craft magnitudes set to zero, **When** a bake runs, **Then** per-scenario fitness matches a nominal (no-craft-variation) bake — a true no-op at zero magnitude.
4. **Given** craft variations are active and the variation ramp is enabled, **When** the run trains, **Then** the craft DRAW per `scenarioSeed` is identical bit-for-bit across gens, and the APPLIED magnitude scales with `computeVariationScale()` (same pipeline as wind/entry). In eval mode, the saved `genome.variation_scale` is replayed exactly — not recomputed, not 100% — so an extracted weight reproduces its training-time airframe magnitude bit-for-bit.

---

### User Story 5 — Bake + flight test the craft-variation-trained controller (Priority: P1, deliverable)

034's payoff: produce a controller trained with craft variations and fly it. This is the mileage the cleanup + variations work was for — it validates that the airframe-diversity dimension produces a controller that flies, before the team takes up the open multi-objective question in 035.

**Why this priority:** The flight test is the acceptance gate for the whole feature — it proves the reset, cleanup, and craft variations cohere into a flyable controller, and it generates real-flight data to inform 035 and the camera/variation "revisit."

**Ownership:** US5 is **operator-run**. The implementation deliverable ends at code-complete through US1–US4 (tests green + fixed-eval determinism gate passing); the M1/M2 bakes and flight test are acceptance milestones the operator executes, tracked separately from implementation tasks. The FRs below are the *acceptance conditions* for those milestones, not agent-coding requirements.

**Independent Test:** An M1 (pathgen) bake with craft variations produces a source library; an M2 (tracker) bake trains a controller against it; the controller is flashed and flown; flight data is captured for analysis. A non-stuck (non-throttle-pegged) climber is obtained per the basin protocol (budget 2–3 bakes for the ~1:3 lottery).

**Acceptance Scenarios:**

1. **Given** the cleaned-up, craft-variation-enabled codebase, **When** an M1 bake runs to convergence, **Then** it produces a craft-diverse source library and a non-stuck climber (per the basin-landscape early-detection signals).
2. **Given** the M1 source library, **When** an M2 tracker bake runs, **Then** it produces a controller whose tracking quality is within the expected baseline band (per-scenario currency).
3. **Given** the trained controller, **When** pre-flight prerequisites are satisfied (failsafe bench verification + pre-flight checklist), **Then** it is flashed and flight-tested, and flight data is captured.

---

### Edge Cases

- **Minisim removal breaks shared code crrcsim still needs** — the teardown must distinguish minisim-only code (delete) from shared stepper/RPC/priming code crrcsim depends on (retain, bit-identical). Test on a crrcsim bake before/after.
- **Smoothness removal breaks a consumer not on the obvious list** (e.g., an analysis script reads the data.dat `smooth` column). Removal must sweep consumers (Python parsers in specs/*, renderer, eval tools), not just producers.
- **Craft variation perturbs the determinism gate** — the craft-seed cascade must preserve the bit-exact M1→M1 replay validated in 033 for in-range seeds; a zero-magnitude craft config must be a true no-op.
- **Craft variation ramps too aggressively** — early-gen NN may struggle to climb if magnitudes scale too fast. Watch the early-gen fitness curve; pull σ down if needed. The ramp curve itself is shared with wind/entry — change `VariationRampStep` and all three move together.
- **Wider per-scenario magnitude spread stresses constant-ε lexicase.** 034 retains the 033 constant-epsilon lexicase; if craft variations visibly degrade tracking-only lexicase selection, the MAD-relative epsilon change (owned by 035) may need to pull forward. Watch for it in the US5 bakes; flag rather than silently absorb.
- **Greenfield dmp schema break** orphans existing S3 dmps. Expected and accepted per the M2-era no-version-revision policy — old dmps are discarded with the training reset, not migrated; no version-management ceremony is added.
- **Seed-width change perturbs determinism.** Right-sizing seeds must preserve the bit-exact replay gate (or the change is reverted) — narrowing must not alter the PRNG sequence for in-range seeds.
- **WindScenarios under-sized for 5–6 dims.** Craft variations push the variation-dimension count up; the basin entry's budget table (3-dim → 36–49; 5–6-dim → 200–350 pairwise) means wind=36 may need to grow. A bake-launch decision ("revisit what we vary"), not a code requirement.
- **Basin lottery confounds the US5 flight-test bake** — a stuck (throttle-pegged) bake produces no flyable controller; budget 2–3 bakes per the basin protocol.

## Requirements *(mandatory)*

### Functional Requirements

#### Minisim teardown (P1, US1)

- **FR-001**: `tools/minisim.cc` MUST be removed, along with its `CMakeLists.txt` build target and any minisim-only ini option/default. The crrcsim worker path becomes the sole path.
- **FR-002**: Minisim-coupled tests (`tests/rpc_transport_tests.cc`, `tests/tracker_stepper_init_tests.cc`, and any others) MUST be updated to the crrcsim-only shape and pass; minisim-only tests MUST be deleted, not disabled.
- **FR-003**: Minisim removal MUST be behavior-preserving for the crrcsim path — a crrcsim eval on a fixed seed/config MUST produce identical per-scenario fitness before and after.
- **FR-004**: Code that existed solely to serve minisim (e.g., minisim-side WorkerInit/priming shim, minisim dispatch branches) MUST be removed; shared stepper/RPC/priming code retained for crrcsim MUST be left bit-behavior-identical.

#### Smoothness removal (P1, US2)

- **FR-005**: All live smoothness code MUST be removed: per-tick factor computation, the `applyStreak` smoothness parameter, the `AircraftState` smoothness field + its serialization, the `WorkerInit`/`EvalData` smoothness knobs + their serialization, the crrcsim worker per-tick mirror (both pathgen and tracker branches), the data.dat `smooth` column, and the smoothness ini keys.
- **FR-006**: Smoothness-specific tests MUST be deleted (not disabled). Tests covering shared code paths the smoothness work touched MUST be updated to the post-removal shape and pass.
- **FR-007**: Removal MUST be behavior-preserving for the shipped 033 config (floor=1.0 no-op) — an M1 eval on a fixed seed/config MUST produce identical per-scenario fitness before and after.
- **FR-008**: All smoothness *consumers* (analysis scripts, renderer, eval tooling, data.dat parsers) MUST be updated so nothing references the removed column/fields. **Carve-out**: the legacy `smoothness=` regexes in `specs/{027..033}/plot_evolution_progress.py` are harmless optional-match back-compat against historical log lines (no current emit exists — verified) and are explicitly out of scope; they need not be pruned.
- **FR-009**: The dmp (and transport) schema changes MUST be greenfield with **no version revision** during the M2 initiative (operator directive; `feedback_no_cereal_versioning`) — no version-field bump, no migration shim, no bespoke fail-loud mechanism. Old dmps are orphaned by the training reset (not migrated). If a stale dmp is loaded, normal cereal error paths apply.

#### Cross-cutting tech-debt fold-ins (P1, US3)

- **FR-010**: The startup config dump MUST print every active `AutocConfig` key via a single-source-of-truth mechanism (X-macro or equivalent) so adding the craft-variation knobs (and future knobs) requires no separate hand-maintained print line, eliminating the three-place-edit drift. Mode-gated where appropriate.
- **FR-011**: Seed widths across the cascade MUST be right-sized to actual entropy, and the `int seed = -1` paste-back truncation/overflow edge MUST be guarded — without altering the PRNG sequence for in-range seeds (determinism gate preserved).
- **FR-012**: The per-(path,wind) variation-table question MUST be resolved — either fix `gScenarioVariations` to be per-(path,wind), or close it as "no real gap" with evidence.
- ~~**FR-013**: crash-hull PRNG seeding~~ — **DROPPED (already satisfied, D2)**. Verified in-tree: `crrcsim_tracker_helper.cpp:54-56` already seeds from `deriveClassSubSeeds(scenarioSeed).rabbit` (canonical deterministic per-scenario route; `windSeed` removed). The V1.5 ELITE_DIVERGED top candidate is resolved. (Separate out-of-scope non-determinism: wall-clock `multiloop`/`simTimeMsec`, per crrcsim/CLAUDE.md.)
- ~~**FR-014**: mod_inputdev → autoc_common~~ — **DROPPED (already satisfied, D2)**. Verified in-tree: `crrcsim/src/mod_inputdev/CMakeLists.txt:26-30` already links `autoc_common`; cherry-pick removed (Constitution IV already honored).
- **FR-015**: M1 (pathgen) and M2 (tracker) runs MUST use distinguishable S3 run-id prefixes (`autoc-` vs `tracker-`) so auto-pick tooling never silently selects the wrong dmp kind.
- ~~**FR-016**: sim→autoc return-path trim~~ — **DEFERRED to `project_fitness_to_worker_backlog`** (operator decision during implement). Implementation revealed it is NOT a simple gate: `computeScenarioScores` (autoc-side) consumes `aircraftStateList` + `targetTrajectoryList` + `cameraViewList`, so trimming requires moving fitness computation to the worker (the standing fitness-to-worker refactor). Memory-opt only; the system runs fine without it (the binding OOM was the autoc→worker direction, already solved by WorkerInit priming). Routed to its proper backlog home.

#### Craft variations (P1, US4)

- **FR-017**: Craft variations MUST be added as deterministic per-scenario variations across these parameters: CG, drag, trim, total power (thrust-scale), pitch control effectiveness, and roll control effectiveness (pitch/roll independent → slight asymmetry). Servo *lag* is OUT of 034 (deferred follow-on). Each MUST have an ini-configurable magnitude knob expressed as a **fractional Gaussian σ** (multiplicative on the param's nominal; σ=0 → exactly nominal). Variations MUST apply to the CRRCSim-simulated NN-controlled craft (the chase) in BOTH pathgen and tracker modes; the target/source trajectory MUST NOT be independently varied.
- **FR-018**: Craft variations MUST be jointly sampled with the existing per-scenario variation dimensions (path, wind, entry, rabbit) from the master PRNG, preserving absolute determinism per `project_variation_design_principles`.
- **FR-019**: Craft variation magnitude MUST ramp with `computeVariationScale()` (same pipeline as wind/entry, via `applyVariationScale()`). The per-`scenarioSeed` *draw* is full-magnitude and deterministic (replay invariant); the per-eval *application* is scaled. In eval mode, `gEvalVariationScaleOverride` (loaded from `genome.variation_scale` in the saved weight file) short-circuits the gen-based ramp so eval replays the training-time scale exactly — not 100%, not recomputed.
- **FR-020**: The craft seed used to set a scenario's airframe parameters MUST be captured in `ScenarioMetadata` and round-trip through the dmp, so the airframe draw is reproducible/replayable. The cascade MUST be structured so a camera seed can be added later without another schema break.
- **FR-021**: With all craft magnitudes set to zero, a bake MUST produce per-scenario fitness identical to a nominal (no-craft-variation) bake — a true no-op guard.
- **FR-022**: Craft-variation parameters MUST be applied through the crrcsim worker path (the sole path post-US1); the autoc↔crrcsim transport MUST carry whatever per-scenario craft data the worker needs (scenario-invariant data via WorkerInit priming; per-scenario data per-eval).

#### Bake + flight test (P1, US5)

- **FR-023**: An M1 (pathgen) bake with craft variations active MUST be run to convergence, producing a craft-diverse source library and a non-stuck climber (per the basin-landscape early-detection signals: throttle-σ > 0, sigma annealing).
- **FR-024**: An M2 (tracker) bake MUST train a controller against the craft-diverse source library, with tracking quality assessed in per-scenario currency against the baseline band.
- **FR-025**: The trained controller MUST be flashed and flight-tested once pre-flight prerequisites are met, with flight data captured for analysis. (Pre-flight prerequisites — failsafe bench verification, pre-flight checklist — are dependencies, not 034 code deliverables.)

### Key Entities

- **Craft variation**: a deterministic per-scenario airframe-parameter draw (CG, drag, servo responsiveness, total power, trim), jointly sampled with the other variation dimensions, applied via the shared `applyVariationScale()` pipeline (ramps with wind/entry; eval replays saved scale), with the seed captured in `ScenarioMetadata`.
- **Smoothness apparatus**: the to-be-removed set of fields/functions/knobs/transport-carriers for the 033 per-tick smoothness factor.
- **Baseline**: the tracking-only pop=8000/wind=36 reference run (033 gen-568 closeout) that 034 bakes are compared against in per-scenario currency.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: After US1, a tree-wide search for live `minisim` references returns zero; autoc + crrcsim build clean with the full test suite green; a crrcsim bake reproduces pre-removal per-scenario fitness on a fixed seed.
- **SC-002**: After US2, a tree-wide search for live smoothness references returns zero, and a before/after M1 eval on a fixed seed produces byte-identical per-scenario fitness.
- **SC-003**: After US3, a reader can reconstruct any 034 bake's exact configuration from its log alone (no config key silently omitted) via the auto-generated dump; the per-(path,wind) variation-table question (FR-012) is resolved (fixed or proven no-gap with a test); seed paste-back round-trips without truncation; and M1/M2 dmps are S3-distinguishable by run-id prefix. *(FR-013 crash-hull determinism + FR-014 mod_inputdev link were verified already-satisfied pre-034 and are not 034 deliverables — see Clarifications D2.)*
- **SC-004**: Craft variations are deterministic and reproducible (same craft seed → bit-identical *draw*; per-eval *applied* magnitude scales with `computeVariationScale()` and eval mode replays the saved scale exactly), a true no-op at zero σ, and the craft seed round-trips through the dmp via `ScenarioMetadata`.
- **SC-005**: The bit-exact M1→M1 replay gate validated in 033 still passes after the seed-width change and the craft-seed cascade rework.
- **SC-006**: A craft-variation-trained controller is baked (M1 → M2), flashed, and flight-tested, with flight data captured — the feature's mileage is realized.

## Assumptions

- The 033 wrapped state (multi-PRNG replay architecture + pop=8000/wind=36 baseline) is the starting point and is retained unchanged.
- crrcsim is the committed sole worker path; minisim is retired with no fallback (consistent with `feedback_m2_no_fallback_patterns`).
- Greenfield cereal schema changes are acceptable (no backward-compat shims); orphaning old S3 dmps is expected.
- Craft variations land now; camera variations are deferred (mostly an M2 concern). The exact varied-parameter set and WindScenarios sizing are bake-launch decisions ("revisit what we vary"), not frozen by this spec.
- 034 retains the 033 constant-epsilon lexicase; the MAD-relative epsilon change is owned by 035 (with the energy work) unless the US5 bakes show craft variations degrading tracking-only lexicase, in which case it may pull forward.
- Flight-test prerequisites (failsafe bench verification per the BACKLOG pre-flight item; the project pre-flight checklist) are satisfied before US5 flying — they are dependencies, not 034 code deliverables.
- The basin lottery (~1:3 stuck) applies to the US5 bakes; budget 2–3 bakes to land a flyable climber.

## Out of Scope

- **Energy as a lexicase secondary objective** — split to [035](../035-energy-lexicase-objective/spec.md), along with the MAD-relative epsilon change.
- **Camera variations** — deferred to a later iteration (mostly M2); only the craft-parameter dimension lands now. The craft-seed cascade is built so camera-seed capture can be added later without re-breaking the schema.
- **WindScenarios / variation-dimension-count final sizing** — a bake-launch decision, informed by the basin entry's budget table; not frozen by this feature.
- US3/US4 from 033 (M2 inherits smoothness; kamikaze penalty) — M2 smoothness-inherit is mooted by US2's removal; kamikaze penalty remains 033-phase-2, gated on the M2 mezzanine flight pass.
- M2-parity training in the source's recorded environment — superseded by the "random variations for M2 training" conclusion (BACKLOG M2 bullet).
- Demetic / island-model GA for basin escape — separate BACKLOG research entry.
- Snapshot/resume + adaptive gen-budget, type-safe NN sensor interface, genome ablation tool — standalone features, not ride-alongs.
- Changing NN topology or the tracking fitness surface (conical scoring).
