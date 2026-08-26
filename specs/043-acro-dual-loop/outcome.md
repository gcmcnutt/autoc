# 043 — outcome (⚠️ IN PROGRESS — feature not closed)

This file is the running record of what landed, what deferred, and why. It is completed at close-out
(T081). Until then it accumulates the deferral/decision records the tasks require (T026, etc.).

## US6 (Phase 4) housekeeping — status & deferrals (T026 · FR-076)

The FR-07x "housekeeping on the opened surfaces" items, each done or recorded deferred:

| FR | task | status |
|---|---|---|
| **FR-072** | T021 — `mod_inputdev` links `autoc_common` (no cherry-pick) | ✅ **Already satisfied** (done in 030 M1, `mod_inputdev/CMakeLists.txt:26-30`). Verified no autoc-source cherry-picking remains. |
| **FR-071** | T022 — `nnextractor -g` vs `dmp-dump --gen` footgun | ✅ **DONE**. `nnextractor -g` now takes the actual generation (via shared `extractGenNumber`), agreeing with `dmp-dump --gen`; `--help` records the meaning CHANGED. BACKLOG entry marked resolved. |
| (build) | T021a — two suites missing from the `make` ALL target | ✅ **DONE**. `shared_input_block_tests` + `nn_input_scaling_tests` added to both the `run_autoc_tests` `DEPENDS` list and the `COMMAND` chain — `make` now builds+runs them (verified locally). The 49/49 gate self-check is re-confirmed by the operator's T027 clean `rebuild-perf.sh`. |
| **FR-070** | T023 — formal **measured** input normalization (not hand-derived constants) | ⚠️ **DEFERRED** — already filed in `specs/BACKLOG.md` (*"[041 P2-8 follow-up … HIGH VALUE, LOW COST] Formal input normalization — measured statistics"*). It changes the NN input scaling → **bake-affecting** and needs its own validation (041's P2-8 rescale took multiple runs to trust); it is also cut-list item #1. Not pulled into 043's bake. |
| **FR-073** | T024 — type-safe NN sensor interface (name input columns by enum at the sites the new axes touch) | ⚠️ **RESEQUENCED to Phase 5** — the "sites the new axes touch" are the **observation-path** sites whose IMU transform is itself deferred to Phase 5 (see T015). Doing the enum-naming there keeps it with the code it hardens, rather than refactoring sites that do not yet exist. |
| **FR-074** | T025 — simulator sampling-time variation (20 Hz tick dither) | ⚠️ **DEFERRED** — new determinism-affecting variation feature (per-scenario seeded, replayable), not open-file housekeeping; cut-list item. Filed in `specs/BACKLOG.md` (043 T025 entry). Owner call on whether it rides the 043 bake. |

⭐ **Net**: the true housekeeping (build coherence + the tooling footgun) is done and verified; the three
bake-affecting / Phase-5-coupled items (T023/T024/T025) are deferred with rationale, exactly the cut order
the 043 implementation strategy names.

## Decisions of record so far

- **2026-08-25 (T003)**: as-run FDM substep is **200 Hz** (`Global::dt = 0.005 s`), not the assumed
  ~333 Hz. Phase-5 discrete constants use 5 ms. See `baseline.md` / `research.md`.
- **2026-08-25 (T006)**: the dynamic gyro notch is **modelled as absent** — effective Q = 2.5, ≥30 Hz
  floor ⇒ < 4° phase at 5 Hz. See `research.md` addendum D.
- **2026-08-25 (T015 split, operator)**: `craftCmQ → Cm_q` + Global carriers landed; the IMU
  **observation-transform** is **folded into Phase 5** (needs a sensed copy distinct from the truth
  fitness uses; also feeds `Cntrl_InavFwRate`). See `variation-inventory.md`.
- **2026-08-25 (T037, correction)**: the `<controllers>` node lives in the **global** config
  (`autoc_config.xml` + `autoc_config-eval.xml`), NOT `hb1_streamer.xml` — `crrc_fdm.cpp:38` reads
  controllers from the global cfg (model-independent). The literal task placement would have created a
  controller that never loads. `data-model.md` §3 anticipated this; 043 does no per-scenario gain
  variation, so the global path is correct.
- **2026-08-25 (T040/T041)**: no `getInputData` change — `ControllerCallback` auto-routes
  `pInputsFromUser`→controller, so the NN→rate-setpoint conversion lives in the adapter/core. ACRO is
  **always-on**, replacing MANUAL outright (Constitution III, no dual path).

## Phase-5 model gates (US2)

- **T037a ✅ (2026-08-25)**: second clean `rebuild-perf.sh` after the `mod_cntrl`/top-level CMakeLists
  changes — **50/50 suites, 0 failures**, binaries present. Constitution IV satisfied with the controller
  compiled in.
- **T043 ✅ (SC-014 part 1)**: all-attitude zero-command sweep, model standing alone
  (`tests/inav_fw_rate_tests.cc::AllAttitudeZeroCommandSweep`). Across a bank×pitch grid over the sphere,
  zero command **nulls body rate** at every attitude (|residual rate| < 2 °/s from a 20 °/s disturbance)
  and **holds attitude WITHOUT self-levelling** (mean restore fraction < 0.15). A contrast ANGLE outer
  loop on the same core+plant restores bank (> 0.8), and the ACRO residual is **not** sign-correlated with
  bank — so the FR-019a discriminator is proven, not trivially true. ⚠️ This is the model-level sweep; the
  full crrcsim-FDM attitude behaviour is additionally exercised by the operator's T044 training run.
- **T044 ⏳ trainability (SC-004)** — operator-driven: seed a short run from a known-good genome and
  confirm the GA improves rather than stalls. **Not yet run.**
- **T045 ⏳ determinism (FR-015)** — operator-driven: identical seed+config reproduce identical
  trajectories, and the eval-vs-training bitwise ScenarioScore gate holds. The `rebuild-perf.sh` clean
  build passed, but the eval-vs-training comparison itself is a separate run. **Not yet run.**
