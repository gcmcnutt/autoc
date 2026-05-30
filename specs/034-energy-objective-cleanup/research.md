# Phase 0 Research: 034 M1/M2 Cleanup + Craft Variations

**Date**: 2026-05-29
**Method**: 5 parallel producer→transport→consumer sweep agents (per operator directive to dig deeper — data.dat-style consumer misses are the recurring failure mode).

---

## R1 — Smoothness apparatus blast radius (US2)

**Decision**: Clean-cut removal of all smoothness code across 12 transport categories (~123 references). `derived_features.h` is **retained** (it also holds the 032 derived perceptual features — span/tilt); only the `SmoothnessMotionMode` enum + `compute_smoothness_factor()` are stripped from it. *(VERIFY at implement time that derived_features.h holds the 032 span features and is not smoothness-only.)*

**Rationale**: Smoothness is already a no-op AND already bypassed in the fitness math ("SMOOTHNESS BYPASS 2026-05-23", `src/eval/fitness_computer.cc:82-84` returns `stepPoints * multiplier` with the factor ignored; `src/eval/fitness_decomposition.cc:174-181` commented out). Removal is therefore **behavior-neutral by construction** — strengthens FR-007/SC-002 (the before/after fixed-seed eval is guaranteed identical, not merely expected).

**Removal map (file:line, by transport boundary):**
- **Producer (fitness)**: `include/autoc/eval/derived_features.h:26-70` (enum + `compute_smoothness_factor`); `src/eval/fitness_computer.cc:57-84` + `include/autoc/eval/fitness_computer.h:53-69` (`applyStreak` smoothness param); `src/eval/fitness_decomposition.cc:174-181` (commented block).
- **Steppers (producer call sites)**: `pathgen_stepper.{h,cc}` (ctor params, `prev_out_*` state, `initScenario` reset, `stepOnce` per-tick compute) and `tracker_stepper.{h,cc}` (identical pattern).
- **Transport**: `include/autoc/rpc/protocol.h` — `WorkerInit{smoothnessPenaltyFloor, smoothnessMotionMode}` (171-179, serialize 203) **AND** `ScenarioMetadata{...}` duplicate (371-378, serialize 399, reset 423-424). *Non-obvious second carrier — easy to miss.*
- **crrcsim mirror (both branches)**: `inputdev_autoc.{h,cpp}` pathgen mirror (state members 138-145; per-tick 1064-1093; reset 583); `crrcsim_tracker_helper.{h,cpp}` tracker mirror (members 105-115; capture 58-62; per-tick 195-226).
- **dmp**: `include/autoc/eval/aircraft_state.h` — `smoothnessFactor_` member (492-494), getter/setter (323-332), cereal `serialize()` (582-587).
- **data.dat**: `src/autoc.cc` — header label `smooth` (~790), format `% 7.4f` (~841-842), read-from-state (~770-772), dmp store (~1030). Column index **41 (pathgen) / 62 (tracker)**.
- **ini**: `autoc.ini`, `autoc-tracker.ini`, `autoc-eval.ini`, `autoc-eval-tracker.ini`, `autoc-eval-visual.ini` (`[Smoothness]` sections); `include/autoc/util/config.h:159-177` + `src/util/config.cc:170-191` (parse + range-validate).
- **Python consumers**: legacy `smoothness=` log regex in `specs/{027,028,029,030,032,033}/plot_evolution_progress.py` (back-compat log parse — leave or prune, non-blocking). **033 dir is smoothness-feature-specific** (analysis artifacts — leave as historical record).
- **tests**: DELETE `tests/derived_features_tests.cc:163-290` (8 `SmoothnessFactor.*` tests) and `tests/fitness_computer_tests.cc:304-396` (6 `FitnessComputer033Smoothness.*` tests, incl. the 4 `DISABLED_`). UPDATE `tests/tracker_stepper_init_tests.cc:104-110` (drop smoothness ctor args).

**CRITICAL consumer (the data.dat-miss class)**: `specs/029-no-future-arch/audit_shift_register.py` uses **hard column offsets** (`POS_OFF=44`) and will **silently read wrong columns** when the `smooth` column is removed. MUST refactor to header-name lookup (or `header.index("X")`) BEFORE/with the column removal. 19 other Python parsers use header-name lookup (safe). `specs/019-improved-crrcsim/sim_response.py` is already dead/broken — delete.

---

## R2 — data.dat producer→consumer map (US2 + US4, the recurring-miss focus)

**Decision**: (a) refactor the one index-fragile consumer to header lookup; (b) delete dead `sim_response.py`; (c) update the stale format-contract doc; (d) craft-variation draws are recorded in `ScenarioMetadata` (dmp), **NOT** as per-tick data.dat columns — so US4 adds **no** data.dat columns and triggers no NN_INPUT_COUNT change.

**Rationale**: Craft params are per-scenario constants, not per-tick signals; the right home is the scenario metadata in the dmp (where the craft seed already must live per FR-020). This avoids touching the scattered `NN_INPUT_COUNT` constant set (`nn_inputs.h`, `topology.h`, `autoc.cc` format strings) — that lockstep risk only applies to NN-input columns, which craft variation does not add.

**data.dat shape (writers in `src/autoc.cc`)**: pathgen ~71 cols (smooth @ 41), tracker ~82 cols (smooth @ 62). Two writers (`logEvalResults` pathgen, `logEvalResultsScenarioTracker`). No C++ readers — data.dat is write-only, Python/human-analysis only.
**Stale doc**: `specs/024-sim-real-fidelity/contracts/sim_data_dat_contract.md` is pre-032/033 — update or mark superseded.

---

## R3 — Minisim teardown (US1)

**Decision**: Surgical. DELETE `tools/minisim.cc` (327 LOC) + `CMakeLists.txt:109,117`. RENAME the misnamed config field `minisimProgram` → `workerProgram` and ini key `MinisimProgram` → `WorkerProgram` across all `.ini` (**clean rename, NO back-compat alias** — Constitution III + `feedback_m2_no_fallback_patterns`; the research agent's "maintain backward compat" suggestion is rejected). UPDATE comment-only references in `threadpool.h`, tests, README, docs.

**Rationale**: Minisim is **already inert** — both `autoc.ini:55` and `autoc-tracker.ini:87` set `MinisimProgram = ./scripts/crrcsim.sh`, so the launched worker is already crrcsim. The two test files reference minisim **in comments only** (no functional dependency). RPC protocol, `tracker_stepper`, threadpool fork logic are all shared/retained.

**OPEN — verify at implement**: Is `PathgenStepper` (`include/autoc/eval/pathgen_stepper.{h,cc}`) used ONLY by minisim? crrcsim has its own inline pathgen (`inputdev_autoc.cpp`), and `minisim.cc:255-264` is the pathgen-stepper consumer. If minisim was its sole user, removing minisim **orphans `PathgenStepper`** → delete it too (Constitution III, every file justifies its existence). `TrackerStepper` IS shared (crrcsim uses it via `crrcsim_tracker_helper`) → retain. Resolve before writing tasks.

---

## R4 — Craft-param FDM mechanism (US4, the deferred clarify item)

**Decision**: Apply craft params at the **existing per-scenario FDM reset hook** — the same path entry-pose variation uses. Insertion: extend `ScenarioMetadata` with craft deltas → carry to crrcsim → set in the per-scenario reset (`inputdev_autoc.cpp:494-520` sets the `Global::` offsets; `Global::Simulation->reset()` @ :542 → `initialize_flight_model()` (`crrc_main.cpp:186-296`) → `initAirplaneState()` (`fdm_larcsim.cpp:88-137`)). Per-scenario only, zero per-tick overhead, no new integrator state for the easy params.

**Per-parameter feasibility (the key finding — they are NOT uniform):**

| Param | FDM site | Verdict | Effort |
|---|---|---|---|
| **Drag** | `CD_prof` (`fdm_larcsim.h:164`, used `aero()` :653) | Settable scalar at init | **Trivial** |
| **Trim** | `Cm_0` (`fdm_larcsim.h:160`) | Settable scalar at init | **Trivial** |
| **CG** | `CG_arm` (`fdm_larcsim.h:193`, used :693) | Settable at init; feeds trim-moment calc | **Medium** |
| **Total power** | engine model (`power/engine_dcm.h`); thrust = throttle × max | **No thrust-scale multiplier exists** — must add one in `engine()` (`fdm_larcsim.cpp:220`) | **Medium-High** |
| **Servo responsiveness** | **DOES NOT EXIST** — CRRCSim applies NN output → control surface directly, no slew/time-constant | Requires a new first-order lag (control-path proxy in `inputdev_autoc`, or FDM state) | **High** |

**FLAGGED SCOPE DECISION (D1)** — the spec lists all 5, but research shows 3 are cheap and 2 (power, servo) need genuinely new mechanisms. Options:
- **(a) Ship CG+drag+trim now; add thrust-scale (medium); defer servo-lag to a follow-on.** ← recommended. Servo lag is a new dynamical element warranting its own validation (cf. the abandoned pt3-filter experiment, BACKLOG: a control-path filter that "mechanically prevents quick corrections" stunted training — a servo-lag model risks the same and deserves isolated study).
- (b) Build all 5 including servo lag now.
- (c) Ship CG+drag+trim only; defer both power and servo.

**Determinism + Constitution notes**: craft draws are a joint-PRNG sample via `deriveClassSubSeeds` (the existing per-class sub-seed pattern); craft seed captured in `ScenarioMetadata` (FR-020). The research agent's "old scenarios get default zeros" framing is **rejected** — greenfield, no back-compat (Constitution III). Craft-param members that flow from `ScenarioMetadata` MUST NOT carry in-class default initializers (Constitution VII). Sampling/delta code is new eval-pipeline code → `gp_scalar` (Constitution VI).
**Quat-entry precedent**: full-sphere entry needs `initAirplaneState()` to take a quaternion — NOT required for craft params (they're scalar coefficient tweaks), so that backlog item stays independent.

---

## R5 — Fold-in code sites (US3)

| # | Fold-in | Site | Scope | Status |
|---|---|---|---|---|
| 1 | Config auto-print (X-macro) | struct `config.h:12-178` (82 fields); parse `config.cc:35-196`; print `autoc.cc:1937-1990` | **Medium** | TODO — no existing X-macro to model on; build the single-source-of-truth list |
| 2 | Seed-width right-size | parse `config.cc:67` (`int seed=-1`); cascade `autoc.cc:1890-1910` (`int`→`long`→`uint64_t`→`uint32_t` Park-Miller state in `scenario_prng.h:78-88`) | **Small** | TODO — narrow `int` + `uint32_t` state is the truncation/paste-back risk |
| 3 | Crash-hull PRNG seed | `crrcsim_tracker_helper.cpp:47-56` seeds from `deriveClassSubSeeds(meta.scenarioSeed).rabbit` | **Small** | **LIKELY ALREADY DONE** — already off `scenarioSequence`. **FR-013 may be moot.** VERIFY against the actual ELITE_DIVERGED symptom before planning work. |
| 4 | mod_inputdev → autoc_common | `crrcsim/src/mod_inputdev/CMakeLists.txt:26-30` | **Small** | **ALREADY DONE** — `target_link_libraries(mod_inputdev autoc_common)` present, cherry-pick removed. **FR-014 moot.** |
| 5 | S3 run-id prefix | `autoc.cc:658` (`"autoc-"` hardcoded in `generate_iso8601_timestamp()`); `cfg.mode` available at call site ~:2143 | **Small** | TODO — prepend `tracker-` when `cfg.mode=="tracker"` at call site (don't bury mode in the timestamp fn) |
| 6 | EvalResults return-path trim | worker populate `inputdev_autoc.cpp:920-1000` (`isEliteReeval` already gates debugSamples/physicsTrace @ :949); receive/collapse `fitness_decomposition.cc:29-100` | **Large** | TODO — `aircraftStateList` is pushed unconditionally; gate non-elite to score-only |

**FLAGGED (D2)**: FR-013 (crash-hull) and FR-014 (mod_inputdev link) appear already satisfied. Recommend: verify, then **drop both from 034 scope** (mark DONE-in-prior-work in the spec) rather than plan no-op tasks. Net US3 = 4 live items (config-print, seed-width, S3-prefix, EvalResults-trim).

---

## Constitution alignment (gate inputs)

- **III No Compatibility Shims**: drives clean smoothness removal, the `minisimProgram→workerProgram` rename (no alias), and greenfield craft-param fields (no default-zero back-compat). PASS by design.
- **V Versioned Persistence**: dmp schema changes (drop `smoothnessFactor_`; drop smoothness from `WorkerInit`/`ScenarioMetadata`; add craft fields to `ScenarioMetadata`). Per project no-cereal-versioning practice + Constitution V write-side contract: **no shim**, but readers MUST fail loud on pre-034 dmps (FR-009). **RISK/OPEN**: with no version-field bump, does cereal *reliably* fail-loud vs silently misread? → contract test must assert old-dmp load fails loudly; if cereal silently misreads on a given change, a guard/magic check is needed. Record as implement-time verification.
- **VI Type-Domain Discipline**: craft sampling/delta math is new eval code → `gp_scalar`; run the audit grep on touched `src/eval/`, `src/nn/` paths at milestone close.
- **VII No Silent Fallback Defaults**: craft-param members sourced from `ScenarioMetadata`/`WorkerInit`/ini MUST NOT have in-class defaults (the cepGateThreshold bug class). Constructor initializer-list is the single assignment site.
- **I/II Testing + Build**: smoothness-test deletion + craft-variation tests + determinism gate; autoc+crrcsim rebuild + xiao build green.

## Operator decisions — RESOLVED 2026-05-29

- **D1 (craft scope)**: CG + drag + trim + thrust-scale + **pitch-effectiveness + roll-effectiveness** (independent → asymmetry). Servo *lag* deferred. Control *effectiveness* (settable control-authority coefficients `Cm_de`/`CL_de` + aileron→roll) substitutes for servo and is in the easy tier. All 6 at the per-scenario reset hook. *(Implement: confirm roll-authority coefficient name in fdm_larcsim.)*
- **D2 (fold-ins)**: FR-013 + FR-014 verified already-done → DROPPED. Crash-hull already seeds from `deriveClassSubSeeds(scenarioSeed).rabbit`; mod_inputdev already links `autoc_common`. Net US3 = 4 live items. The V1.5 memory entry is stale (update it). Separate out-of-scope non-determinism remains: wall-clock `multiloop`/`simTimeMsec` (crrcsim/CLAUDE.md).
