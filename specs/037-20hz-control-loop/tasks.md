---
description: "Task list for 037 Faster Control Loop (10 Hz → projected ~20 Hz)"
---

# Tasks: Faster Control Loop (10 Hz → projected harmonic ~20 Hz)

**Input**: Design documents from `/home/gmcnutt/autoc/specs/037-20hz-control-loop/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: INCLUDED — Constitution Principle I (Testing-First) requires tests for mainline-promoted
code. Firmware bring-up (US2/US3) uses bench/flight acceptance plus host-unit tests for the pure
fusion/auto-cal math.

**Organization**: by user story = spec phase. **US1 = Phase A** (sim M1 go/no-go, the MVP). **US1b =
Phase A M2** (tracker retrain, gated on US1 signal). **US2 = Phase B** (embedded ~20 Hz). **US3 =
Phase C** (50 Hz stretch). Phase 0 research is Foundational (gates US1).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable (different files, no dependency on an incomplete task)
- **[Story]**: US1/US1b/US2/US3; Setup, Prework, Foundational, Polish carry no story label
- Exact file paths included.

## Prework index (the spec's "Prerequisite prework" items — where each lives)

| Prework | Spec section | Task | Phase | Why there |
|---|---|---|---|---|
| **P1** entry-envelope sigma tighten | "tighten the entry-variation envelope" | **T006** | Prework | config edit; must land as a clean step *before* the US1 retrain (T026) |
| **P2** slim autoc training logfile | "slim the autoc training logfile" | **T005** | Prework | cleanup; do alongside P1 (after verifying dmp reconstructability) |
| **P3** fix short-source skip (keep 294 1:1) | "fix the short-source skip" | **T004** | Prework | tracker-mode bugfix; **gates any clean M2 bake** (035 M2 rerun + US1b) |
| **P4** renderer focus-mode single-arena | "renderer focus-mode single-arena" | **T007** | Prework | **moved early (operator 2026-06-09): at 40/50 Hz the renderer steps every arena and becomes unusable for inspecting high-rate runs — needed to *use* the sim in US1** |
| **P5** Xiao packed-logging format | "Xiao recording format (packed)" | **T038** | US2 | xiao firmware; operator-deferred to Phase B, after the US1 M1 signal |

## Path Conventions

Multi-component repo: `crrcsim/` (FDM + inputdev), `src/` + `include/autoc/` (autoc eval/nn), `xiao/`
(firmware), `tests/` (GoogleTest), `specs/037-20hz-control-loop/` (feature one-off scripts +
research artifacts per `feedback_scripts_dir_scope`).

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: the cadence knob, the measurement tool, and test scaffolding everything else uses.

- [X] T001 Add a single-source-of-truth control-rate config key read from `.ini`/CLI (not the
  `AUTOC_EVAL_INTERVAL_MSEC` env var) and make `gEvalUpdateIntervalMsec`
  (`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.{h,cpp}`) and `SIM_TIME_STEP_MSEC`
  (`include/autoc/eval/aircraft_state.h:39`) derive from / assert-equal it; keep the startup
  cadence-triple assertion (`inputdev_autoc.cpp:297-316`). **Defaults to 100 like every other knob**
  (operator 2026-06-09 -- a missing key isn't fail-loud; it's auto-printed so auditable); config.cc DOES
  fail-loud on `!= SIM_TIME_STEP_MSEC` (the real coherence invariant). Per contracts/cadence-config.md +
  research.md R6.
- [X] T002 [P] New de-alias analysis script (dmp-dump-based, NOT a historical script —
  `feedback_historical_scripts_immutable`) computing per-axis roll **lag-1 autocorr, sign-flip rate,
  saturation %, dctrl** in `specs/037-20hz-control-loop/dealias_metrics.py`; reads t6 as the 10 Hz
  baseline reference.
- [X] T003 [P] Test scaffolding: create `tests/cadence_invariance_tests.cc` and
  `tests/tick_rescale_tests.cc` skeletons + register in the GoogleTest target (CMakeLists; operator-driven
  `rebuild-perf.sh` after the registration edit per Constitution IV). (Repo convention is the `*_tests.cc`
  suffix, not the `test_*` prefix; skeletons `GTEST_SKIP` so `run_autoc_tests` stays green until T014/T015.)

**Checkpoint**: cadence is a first-class configurable knob; the gate metric is measurable.

---

## Phase 2: Prework / Cleanup (do early — independent of the bake)

**Purpose**: the spec's standalone "Prerequisite prework" cleanup that should land BEFORE the main
work, not trail it. P1/P2/P3/P4 here; P5→US2 (xiao firmware, deferred by design — see prework index).
These are independent of the Phase-0 research and can run in parallel with it.

- [X] T004 [P] **P3 — fix the short-source skip**: revert the `c95887e` erase; keep `gSourceTrajectoryList`
  at **294 (1:1 with the scenario slots)** so `slot % 294 = slot` holds. **DECISION (operator 2026-06-09):
  do NOT add skip/exclude machinery — just PLAY the short corner-crash scenarios THROUGH** (they terminate
  cleanly via `CrashReason::TimeLimit`, contributing their short real fitness, garbage-in-garbage-out).
  Rationale: with the tightened entry cone (T006) such degenerate sources are rare-to-absent in new M1
  runs, so the skip footprint (touching fitness_decomposition/selection/#Gen) wasn't worth it. Touch
  `src/autoc.cc` only (keep-all + warn). **Gate: must precede any clean M2 bake** (035 M2 rerun / US1b).
  Tracker-mode only; doesn't affect M1.
- [X] T005 [P] **P2 — slim the autoc training logfile** (DONE 2026-06-10, commit 06364cd): per-scenario
  `[N] OK/CRASH` + tracker-diag lines dropped from the training loop. Reconstructability verified FIRST
  against the live t5 S3 dmp; `dmp-dump --meta-only` gained the two missing fields (streak_steps,
  max_multiplier). Per-gen `#NNGen`/`#GenCrash`/`#GenSimStats`/`#GenDiag` summaries + the eval-mode
  one-shot breakdown unchanged. (`project_dmp_driven_analytics_backlog` direction.)
- [X] T006 [P] **P1 — tighten the entry-variation envelope**: in `include/autoc/util/config.h:63,65` set
  `entryConeSigma 30→18` (⇒ 2.5σ = 45°) and `entrySpeedSigma 0.10→0.05–0.06` (⇒ 2.5σ ≈ 12.5–15%); clamp
  stays 2.5σ — do NOT add a new hard cap (existing `kEntryConeMaxRad=80°` becomes moot). Update the
  load/log in `src/autoc.cc`. Clean step coordinated with the US1 retrain (T026); t6 stays the baseline
  (Q3 caveat). **MUST precede T026.**
- [X] T007 [P] **P4 — renderer focus-mode single-arena**: in playback focus mode the renderer currently
  still steps **every** arena; change it to process **only the focused arena** (skip per-tick
  update/step for non-focused arenas, not just skip draw). **Moved early** because at 40/50 Hz the
  all-arena step makes the renderer too slow to *use* for inspecting high-rate runs — this is what keeps
  the sim usable during the US1 retrain/inspection, not just a Phase-C compute win. Scope: the
  visualization/playback renderer path (confirm whether eval-visual sampling also benefits).

**Checkpoint**: the tree is clean for a reproducible bake; the M2-gating bug is fixed; the M1 difficulty
envelope is set; the renderer stays usable at the higher sim rates.

---

## Phase 3: Foundational — Phase 0 research (Blocking Prerequisites for US1)

**Purpose**: produce the smoothing theory + projected cadence + derived-bundle decisions that GATE the
bake. **⚠️ No US1 retrain begins until T008 + T011 + T012 complete.**

- [X] T008 **Smoothing theory (RT — GATES the bake)**: extract roll-subsidence τ_roll from the FDM
  step-response trace (`crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` trace-capture path) + `Cl_p=-0.47`;
  inspect **raw pre-tanh roll drive** in the t6 traces; produce the per-axis (A) smooths / (B) relay
  prediction + command-vs-motion target. Write to research.md RT. A (B) verdict = cheap no-go, STOP
  before any bake.
- [ ] T009 [P] R2 camera fps grid + AGC/exposure budget for VD55G1 (primary) / OV9281 (backup): reachable
  fps-vs-window curve (PLL/HTS/VTS), fixed-exposure/gain beacon operating point; co-resolve with 031
  shortlist (`docs/aircraft_tracker_handoff.md` §4). → research.md R2.
- [ ] T010 [P] R3 transport ceiling: measure MSP read+write at 115200 and at a bumped baud
  (921600/1M) on the real harness; document the per-tick budget + baud-vs-SPI recommendation. →
  research.md R3, contracts/transport-link.md. Update `project_sim_latency`.
- [X] T011 R1 **projected NN loop rate**: DECIDED 2026-06-10 — **20 Hz** (operator "get going with
  20 Hz now"); R2/R3 stay open for the Phase-B flown rate, not the sim gate. → research.md R1.
- [X] T012 R5 **history time-basis decision**: DECIDED 2026-06-10 — ms-based log-spaced lags
  `{1600,800,400,200,100,0}`; `NN_INPUT_COUNT` stays 33 (M1) / 54 (M2); fail-loud via
  `kNNHistoryLayoutVersion=2` serialized marker. → research.md R5.
- [X] T013 [P] Tick-rescale audit sign-off: confirm the research.md audit table against current code
  (`src/eval/fitness_decomposition.cc:179-180,206-244`, `src/nn/evaluator.cc:310-334`) — list every term
  to change and its rescale rule before touching code.

**Checkpoint**: rate projected, bundle defined, theory predicts smooth-or-not. US1 can begin.

---

## Phase 4: User Story 1 — Phase A sim M1 retrain & de-alias gate (Priority: P1) 🎯 MVP

**Goal**: retrain M1 at the projected rate with the derived bundle and confirm the smoothing theory
against the de-alias gate — the whole-feature go/no-go, sim-only, no firmware.

**Independent Test**: at the projected rate, roll lag-1 autocorr negative→≥0 AND sign-flip −≥20 pts
(t6 56%→≤36%) at tracking within noise of historical t6 10 Hz (Clarifications Q1/Q3), matching the
theory's (A)/(B) prediction.

### Tests for User Story 1 (write first, ensure they FAIL) ⚠️

- [X] T014 [P] [US1] Cadence-invariance test in `tests/cadence_invariance_tests.cc`: stability + energy
  totals for a fixed synthetic trajectory are ~equal at 10 Hz vs the projected rate (within FP).
- [X] T015 [P] [US1] Tick-rescale test in `tests/tick_rescale_tests.cc`: `closing_rate` for a known dDist
  is correct at two intervals; engage-delay ticks = `ceil(EngageDelayMs/controlIntervalMsec)` at 10/20/50.
- [X] T016 [P] [US1] NN-layout round-trip + fail-loud test in `tests/nn_layout_tests.cc`: write/read a dmp
  at the new layout byte-identical; an old-layout dmp triggers a clear fail-loud error (Principle V).

### Implementation for User Story 1

- [X] T017 [US1] Set the projected cadence: `SIM_TIME_STEP_MSEC` 100→50 + `ControlIntervalMsec=50` in
  all 6 inis; in-struct config default now tracks the master. **No `autoc_config.xml` change needed at
  20 Hz**: dt=0.005/fps=20 gives cycleLength 50 ms, framesPerEval 1, oversample exactly 10× (the 2 kHz
  FDM bump is the US3 50 Hz arm's requirement). Per contracts/cadence-config.md.
- [X] T018 [P] [US1] Tick-rescale: accumulators ×`kCadenceTickScale` (= interval/100 — anchored to the
  historical 100 ms-tick scale, NOT ×dt, so the 10 Hz gate stays bitwise and lexicase ε=0.5 keeps its
  meaning). **Scope grew at T013: the main per-tick path score also rescales** (was missing from the
  audit table; crash-penalty weight would have silently halved).
- [X] T019 [P] [US1] Tick-rescale: closing-rate divisor = the actual NOW↔TM1 lag gap
  (`kNNHistoryRecentGapSec` = 100 ms at every cadence); closure/thrash already SIM_TIME_STEP-derived ✓.
- [~] T020 [US1] Rate-independent engage delay: shared `engageDelayTicks()` ceil helper
  (aircraft_state.h) + crrcsim pathgen branch routed through it (bitwise no-op). **Tracker-stepper
  engage WINDOW deferred to US1b** (M2-fidelity only, gated; needs ctor plumbing through 6 call
  sites — do with T029 prep).
- [X] T021 [US1] History time-basis (M1): `kNNHistoryLagsMsec` table + `kNNHistoryLayoutVersion` in
  nn_inputs.h; `HIST_PAST[]` derived (50 ms → {32,16,8,4,2,0}); `HISTORY_SIZE` derived (33);
  static_assert lag integrality. `NN_INPUT_COUNT` stays 33 ⇒ NO topology/weight-count change (R5
  chose slot-count-preserving lags); fp32 kept.
- [X] T022 [US1] Tracker (M2) mirror: shared `TrackerObservationRing` (evaluator.h) materializes the
  6-slot `TrackerHistoryWindow` view at the lag offsets in BOTH mirrors (tracker_stepper.cc +
  crrcsim_tracker_helper.cpp); `span_rate` now a true rate over the 100 ms gap; **fail-loud
  source-tick-spacing check** (old 10 Hz source libraries refuse to play at 2× speed). M2 retrain
  gated US1b.
- [X] T023 [US1] Fail-loud: `kNNHistoryLayoutVersion` marker serialized in the per-state NN block +
  checked on read (counts didn't change, so the existing count check couldn't catch stale dmps); v=1
  gen9200 baseline path untouched. Honest recording unchanged (all inputs+outputs already recorded).
- [~] T024 [US1] Latency/jitter model — RESOLVED 2026-06-10 as: `COMPUTE_LATENCY` stays 30 ms (bench
  truth, fits the 50 ms tick; re-measure at T010); cadence-jitter DEFERRED (needs a per-tick-PRNG
  determinism design pass); PID filter cutoffs are INAV-side → Phase B T034 (sweep found no live LPF
  constants in the sim path). See research.md "T024 latency/jitter bundle status".
- [X] T025 [US1] Build + verify tests pass (DONE 2026-06-10): clean `rebuild-perf.sh` (covers autoc +
  crrcsim) post-CMakeLists-change, then incremental `(cd build; make)` for the T005 follow-up — **all
  34 suites green** under the perf build. The stage-1-commit (`9592dea`) 10 Hz bit-replay gate was
  **WAIVED (operator 2026-06-10: "we are beyond 10 Hz now")** — 20 Hz is the baseline cadence going
  forward; `9592dea` remains the checkout point if 10 Hz archaeology is ever needed.
- [X] T026 [US1] Retrain M1 at 20 Hz — DONE (two arms, see outcome.md):
  `scripts/train.sh autoc.ini logs/autoc-037-t6-m1-20hz.log` (pop 5000 / 800 gens / 6×49 scenarios /
  lexicase+MAD / seed 13337; dumps auto-tagged `retain=expire`). NOTE: "037-t6" is this 20 Hz bake;
  the *de-alias baseline* "t6" in dealias_metrics.py is the 035 run `autoc-035-t6-m1-energy` — don't
  conflate. Preceded by smoke runs t4 (FAIL → crrcsim f81fd31 command-starvation fix) and t5 (PASS,
  climbs earlier/farther than 10 Hz — see finding.md).
- [X] T027 [US1] Measure the gate — **MEASURED, verdict NO-GO (RT case B)**: t7 (20 Hz, no servo)
  converged-tracking roll ac −0.27 / flips 59% / dCtrl 0.94 ≈ the 10 Hz baseline (−0.24 / 56% /
  ~1.0). Both gate legs FAIL at tracking depth; bang-bang is objective-optimal, not a sampling
  artifact. t6 (20 Hz + servo) was smooth but tracking-capped (plant-imposed). Full three-arm
  analysis + decision in outcome.md. **US1b / US2 / US3 are gated OFF.**
- [ ] T028 [US1] **Milestone close**: type-domain grep audit on US1-touched `src/eval/ src/nn/` (Principle
  VI per-milestone — annotate `// raw-ok:` or convert). The 10 Hz bit-replay leg is **WAIVED**
  (operator 2026-06-10 — committed to 20 Hz; the smoke runs t4/t5 are the refactor's behavioral
  evidence, and `9592dea` is the archaeology checkout if ever needed).

**Checkpoint (MVP)**: a defended smooth-vs-relocate verdict at a hardware-projected cadence, clean per
the constitution. **Clears → US1b (M2) and/or US2; persists → US3; not a clear win → STOP** before
firmware (cheap no-go).

---

## Phase 5: User Story 1b — Phase A M2 tracker cadence retrain (Priority: P1, GATED on US1 signal)

**Goal**: carry the cadence win into tracker mode — retrain M2 at the projected cadence on the
already-implemented M2 layout/rescale. **In scope ONLY IF US1 (T027) shows a sensible higher-rate
signal** (operator, 2026-06-09); skipped otherwise.

**Independent Test**: M2 at the projected cadence holds tracking (crash % / on-track parity with the M2
baseline) with the per-axis de-alias improvement the theory predicted.

- [ ] T029 [US1b] Retrain M2 (tracker) at the projected cadence via `scripts/train.sh` — requires the P3
  short-source fix (T004) for a clean M2 bake and the M2 layout/rescale (T022). Name
  `autoc-037-t<N>-m2-<rate>...`; tag `retain=expire` (Principle VIII).
- [ ] T030 [US1b] Measure M2 de-alias + tracking vs the M2 baseline with `dealias_metrics.py`; confirm vs
  the theory; record the go/no-go for carrying M2 to flight. Per-milestone type-audit on any
  M2-specific touched code (Principle VI).

**Checkpoint**: M2 validated (or declined) at the projected cadence. Gate decides whether the flown
controller (US2) is M1, M2, or both.

---

## Phase 6: User Story 2 — Phase B embedded ~20 Hz (Priority: P2)

**Goal**: run a fresh ~20 Hz control tick on real hardware (local-IMU fast loop + INAV slow sync),
validated against INAV, producing the flown controller. **GATED on US1 clearing at ~20 Hz.**

**Independent Test**: local-IMU fusion agrees with INAV (attitude/heading/convention) within tolerance in
a logged cross-check flight; then promoted, the 20 Hz loop holds (no `ctl loop` overruns) and sim↔real
de-alias matches.

### Tests for User Story 2 (host-unit, the pure math) ⚠️

- [ ] T031 [P] [US2] Host-unit tests for the fusion/auto-cal math in `tests/test_imu_fusion.cc`:
  quaternion gyro propagation, complementary blend, **latency forward-propagation** to "now", and the
  auto-cal rotation solve — pure functions, testable off-target (Constitution I before promotion).

### Implementation for User Story 2

- [ ] T032 [US2] Execute the **021 convention cross-check** (`autoc::imu::inavQuatToAerospaceEB` boundary)
  — the Phase-B prerequisite gate.
- [ ] T033 [US2] Raise the link baud on both ends (`xiao/src/msplink.cpp:342 Serial1.begin`, INAV
  `inav-hb1.cfg` MSP port) per T010; re-measure read+write fits the 20 Hz budget
  (contracts/transport-link.md).
- [ ] T034 [US2] INAV-side: command override + state serve at 20 Hz; keep CH6=manual semantics.
- [ ] T035 [US2] New `xiao/src/imu_local.{cpp,h}`: LSM6DS3 TWIM-DMA read + completion ISR (tail-safe,
  double-buffered) — there is currently NO onboard-IMU code.
- [ ] T036 [US2] Complementary/loosely-coupled fusion: local gyro propagation + accel leveling +
  gyro-bias estimate; INAV slow-sync blend with **latency compensation** (forward-propagate stale INAV
  attitude to "now" via MSP `timestamp_us` before blend). Per spec Fusion scheme. (math covered by T031)
- [ ] T037 [US2] Three rate tiers (data-model §5): fast local IMU @ control rate; intermediate INAV
  position poll (dead-reckon bridge); slow INAV attitude resync. Auto-calibrate local-IMU→body rotation
  from INAV (not manual board-alignment).
- [ ] T038 [US2] **P5 — packed dual-stream log** (contracts/packed-log-format.md): variable frame
  type/rate, delta+varint, pre-erased async-DMA ring fixing the blocking `flash_logger.cpp:666-713`
  tail; shared writer/reader + `join_flight_analysis.py` decode. Also firmware history layout in lockstep
  with the sim (T021/T022).
- [ ] T039 [US2] Set `xiao/include/main.h` `MSP_NN_EVAL_DIVISOR=1` (20 Hz NN) once T033/T036 fit the tick.
- [ ] T040 [US2] Validation: `activate → capture → confirm` (local-IMU logged beside INAV) → promote →
  flight test → sim↔real de-alias confirm. Pin the flown controller S3 prefix in the outcome doc
  (Principle VIII).

**Checkpoint**: a flown ~20 Hz controller with confirmed sim↔real smoothing.

---

## Phase 7: User Story 3 — Phase C 50 Hz stretch (Priority: P3)

**Goal**: reach 50 Hz (20 ms tick) with a bounded eval + tail-safe real-time loop. **GATED on US1
showing 20 Hz insufficient.**

**Independent Test**: 50 Hz loop sustains within budget on-target (eval ≤ defended budget; no tick
overrun under worst-case jitter), and the 50 Hz sim arm (FDM ≥500 Hz) clears the de-alias gate.

### Implementation for User Story 3

- [ ] T041 [P] [US3] R4 cycle-count harness (contracts/eval-cycle-harness.md): a `xiao/` bench target
  running the codegen'd unrolled M1/M2 forward, counting `DWT->CYCCNT` for {macs, +tanh (expf vs
  poly/LUT), +gather}; off-target op-counter cross-check. Emits the defended eval budget. (Cheap;
  runnable early/parallel.)
- [ ] T042 [US3] Unroll + fast-tanh the NN forward (`xiao/src/generated/nn_program_generated.cpp` →
  codegen'd straight-line FMA, fp32) guided by T041; M1 33→32→16→3, M2 54→32→16→3.
- [ ] T043 [US3] Real-time slot scheduling (collect→process→output→log) with control-critical slots
  preempt-protected; dt-aware processing via `inavSampleTimeMsec`; degrade logging under overrun.
- [ ] T044 [US3] Tail-bounding: async QSPI flash (no in-loop erase), NVIC priorities so the control tick
  isn't preempted, DMA completion ISRs; instrument sub-phase timestamps to attribute the 7.4 ms tail.
  (Renderer single-arena cleanup is prework T007.)
- [ ] T045 [US3] Validate the 50 Hz sim arm at FDM ≥500 Hz (≥10× oversample) clears the gate; on-target
  budget check.

**Checkpoint**: 50 Hz viable and justified, or documented as unnecessary (20 Hz sufficed).

---

## Phase 8: Polish & Cross-Cutting Concerns

- [ ] T046 [P] Final type-domain grep audit (Principle VI) across all touched `src/eval/ src/nn/
  include/autoc/eval/ include/autoc/nn/` (closes any per-milestone gaps from T028/T030).
- [ ] T047 [P] 031 write-back: fold the chosen control rate into `docs/aircraft_tracker_handoff.md` as the
  acquisition-budget input; carry the per-beacon independent-correlator requirement (R7).
- [X] T048 Outcome doc + memory: `outcome.md` written 2026-06-11 (case-B verdict, three-arm table,
  pinned S3 prefixes, what-037-bought, route forward); project memory updated (cadence lineage,
  servo-era metrics, bang-bang physics-migration, basin-robustness observation).

---

## Post-outcome track (2026-06-11): return to 10 Hz + servo model v2

The cadence verdict (outcome.md) routes the next experiments to actuator-model fidelity at 10 Hz.
Implemented after the d480241 checkpoint:

- [X] P-O1 **10 Hz flip-back**: `SIM_TIME_STEP_MSEC` 50→100 (+outcome comment), `ControlIntervalMsec=100`
  in all 6 inis. Lag set integral at 100 ms ({16,8,4,2,1,0}); cadence triple → framesPerEval=2.
- [X] P-O2 **Servo model v2** (datasheet-shaped per finding.md DSM-44 check + operator spec):
  - 50 Hz **PWM command latch** in fdm_larcsim (replaces the v1 #if-0 lag block): per-scenario phase
    = uniform [0, 20) ms craft-class draw (`craftServoPwmPhase`, appended draw + ScenarioMetadata
    field, drawn unconditionally so toggling never shifts PRNG order) — the real command dead-time.
    Target FROZEN between latches (delays, doesn't smooth); timer/latch state reset per scenario.
  - **Pure slew** toward the latched target: center re-derived from the 0.055 s/60° transit on a 90°
    span ⇒ ≈12.1 full-throw/s (was 6.0, shaded slow), clamp 8–16 (was 3–9). NO first-order lag in
    v2 — tau is still drawn (order stability) but unused by the FDM.
  - **`ServoModelEnabled` ini knob** (int 0/1, default 0) → WorkerInit → `Global::servoModelEnabled`,
    so the t8 (off) / t9 (on) A/B differs by ini only, no rebuild. autoc.ini carries the key =0.
  - Variation table gains the `pwmPh` column; contract_config_tests field count 93→94.
- [X] P-O3 Build green (incremental, perf tree), **all 34 suites pass**.
- [ ] P-O4 **basic-m1 smoke ×2** (standing rule after loop plumbing — caught both prior cadence bugs):
  servo OFF (validates the 10 Hz flip) and servo ON (exercises the new latch path; needs
  `ServoModelEnabled=1` in autoc-basic-m1.ini for the second arm). Check: gen-1 population spread on
  the historical scale, no identical-fitness signature, startup `servoModelEnabled=` line.
  **STATUS 2026-06-12: never run — t8 was launched directly. Still owed before t9 (and the slew-fix
  rebuild re-triggers the standing rule anyway).**
- [ ] P-O5 **10 Hz confirm bake** (servo OFF, autoc.ini as-is): expect ≈035-t6
  signal ("roughly same, maybe more robust"); ALSO the crash-floor attribution run (if residual
  crashes ≈ t7's ~13–23/294, the floor is config — craft-full/history — not cadence).
  **STATUS 2026-06-12: SKIPPED — the run named t8 took the servo-ON arm instead. Crash-floor signal
  partially recovered from t8 anyway (21/294 = 7.1% ≈ t7's floor, with servo on → floor is config,
  not cadence). Arm remains unrun; demoted behind the t9 slew-fix rerun (t7 already vouches for the
  non-servo bundle).**
- [X] P-O6 **t8 — servo v2 bake** (`ServoModelEnabled=1`, gens 1–625, stopped 2026-06-12): "does a
  little honest filtering coexist with tracking?" **RAN BUT DID NOT ANSWER THE QUESTION** — audit
  found a factor-2 slew unit mismatch (craft_variation.h derives span/s; the FDM consumes
  half-span/s → effective transit 165 ms vs the 82.5 ms datasheet servo), so t8 tested a servo 2×
  slower than honest. Result on the bugged params: deadened flat (pctInStreak ≤2.9% all run vs
  18–25% no-servo arms; throttle pinned 100%). Full analysis + fix in finding.md §t8-final.
- [ ] P-O7 **slew unit fix + t9 — corrected servo v2 bake**: slew now expressed in autoc [-1,1]
  command units/s (operator convention — INAV/xiao speak the same units; platform code translates,
  crrcsim's ×0.5 = the surface conversion). `kCraftServoSlewCenter` ≈24.2 units/s, clamp 16–32,
  ini sigma 2.0→4.0 (doubled with the units). basic-m1 smoke ×2 (P-O4), then rerun the servo-ON
  arm as t9, same config as t8 otherwise. THIS is the honest P-O6 experiment: full reversal now
  completes within a 100 ms tick (82.5 ms + ≤20 ms latch) — relay authority preserved, honest delay.

---

## Dependencies & Execution Order

### Phase dependencies

- **Setup (P1)**: no deps — start immediately.
- **Prework (P2)**: independent of Foundational; run in parallel. **T004 (P3) gates any clean M2 bake
  (US1b T029)**; **T006 (P1) MUST precede the US1 retrain T026**; **T007 (P4 renderer) before inspecting
  any ≥40 Hz sim run**.
- **Foundational (P3)**: after Setup; **T008+T011+T012 BLOCK US1**. T009/T010 feed T011.
- **US1 / Phase A M1 (P4)**: after Foundational (+ T006 prework; T007 for high-rate inspection). The MVP.
  Its gate (T027) BLOCKS US1b/US2/US3. Closes with T028 (per-milestone audit + regression gate).
- **US1b / Phase A M2 (P5)**: GATED on US1 showing a sensible signal (T027). Needs T004 + T022.
- **US2 / Phase B (P6)**: after US1 clears at ~20 Hz.
- **US3 / Phase C (P7)**: after US1 shows 20 Hz insufficient (T041 may start early/parallel).
- **Polish (P8)**: after their phase.

### Critical gate chain

`T008 (theory) → T011 (projected rate) → T017–T024 (bundle) → T026 (retrain) → T027 (gate) →
{US1b M2 | US2 20 Hz | US3 50 Hz}`

### Within US1

Tests (T014–T016) written first and FAIL → cadence/FDM (T017) → rescale (T018–T020) → history layout
(T021–T023) → latency model (T024) → build (T025) → retrain (T026, needs T004/T006) → measure (T027) →
milestone close (T028).

### Parallel opportunities

- Setup: T002, T003 [P].
- Prework: T004, T005, T006, T007 all [P] — and parallel with Foundational research.
- Foundational: T009, T010 [P] (then T011); T013 [P].
- US1 tests: T014, T015, T016 [P]. US1 impl: T018, T019 [P] (different files).
- US2: T031 [P]. US3: T041 [P]. Polish: T046, T047 [P].

---

## Parallel Example: Prework + Foundational together

```bash
# Prework cleanup (independent) runs alongside Phase-0 research:
Task: "P3 fix short-source skip in src/autoc.cc + inputdev_autoc.cpp:700"
Task: "P2 slim autoc training logfile (after dmp-reconstructability check)"
Task: "P1 tighten entry sigmas in include/autoc/util/config.h"
Task: "P4 renderer focus-mode single-arena in the crrcsim viewer path"
Task: "RT smoothing theory → research.md"
Task: "R2 camera fps grid + AGC → research.md"
Task: "R3 transport ceiling → research.md"
```

---

## Implementation Strategy

### MVP first (US1 / Phase A M1 only)

1. Phase 1 Setup → 2. Phase 2 Prework/Cleanup → 3. Phase 3 Foundational (RT theory may no-go here
cheaply) → 4. Phase 4 US1 → 5. **STOP & VALIDATE**: the de-alias gate vs the theory prediction →
6. decide US1b (M2) / US2 / US3 / stop.

### Incremental delivery

US1 (sim M1 go/no-go + the rate decision) → US1b (M2 retrain, if signal) → US2 (flown 20 Hz) → US3
(50 Hz only if needed). Each is independently testable; US1b/US2/US3 are gated on US1's result.

---

## Notes

- [P] = different files, no incomplete-task dependency.
- **Prework is its own early phase now** (P1/P2/P3/P4 — P4 moved up because the all-arena renderer is
  unusable at ≥40 Hz); only P5 (xiao packed log) stays deferred to US2 (see index).
- **US1b (M2) is gated-in-scope**: retrained only if US1 (M1) shows a sensible higher-rate signal.
- Theory (T008) is the true gate — it can kill the feature cheaply before any bake.
- The cadence change is a *bundle* (cadence + latency/jitter + history buckets + FDM), not one knob.
- Principle V: the history-layout change is greenfield (no cereal bump) but fail-loud on read.
- Principle VI audit runs **per milestone** (T028 US1, T030 US1b, T046 final), not only at the end.
- Operator drives the regression gate and clean rebuilds; never rebuild during a live training run.

---

## Session implementation status + design decisions (2026-06-09)

**Prework / Setup / theory done (uncommitted, awaiting a clean build):** T001, T002, T003, T004, T006,
T007, T008. T005 deferred to just-before-training (operator). Notes:

- **T004** final form is **play-through, no skip machinery** (operator 2026-06-09). The impact analysis
  first led to a `skip`-flag exclude (ScenarioScore.skipped honored across fitness/lexicase/minimax/#Gen),
  but that was REMOVED as over-engineered: short sources terminate cleanly via `CrashReason::TimeLimit`
  (tracker_stepper.cc:218) and just contribute their short real fitness. The fix is now `src/autoc.cc`
  only -- replace the `c95887e` erase with keep-all-294 + a warning; `computeScenarioScores` stays
  single-arg; no `SourceScenarioTrajectory.skip`, no `ScenarioScore.skipped`. With the tightened entry
  cone these degenerate sources should be rare-to-absent anyway.
- **T008 gate = GO** (roll = case A, aliasing-dither) but with a real-flight caveat: the corpus shows
  roll is NOT authority-capped (peaks 540-620 deg/s), so the verdict now rests on the Nyquist/tau_roll
  under-sampling + the -0.24 anti-persistent autocorr, NOT "saturation is averaged." **The FDM
  held-command roll step-response trace remains the one outstanding analytic validation.**

**Sim-to-real fidelity work added this session (beyond the original task list; feeds the T024 bundle):**

- **Energy curve** bumped `throttle^2 -> throttle^2.5` in `throttleEnergyStep` (closer to real prop
  power; operator-chosen). `energy_metric_tests.cc` fixtures updated. NOTE: this shifts the 035 energy
  objective landscape vs the t6 baseline (intended).
- **Actuator dynamics (servo lag+slew + thrust lag) -- IMPLEMENTED INSIDE THE FDM.** Decision (operator
  2026-06-09): the lag lives in `fdm_larcsim` at the **substep dt**, NOT the command path. Rationale:
  tau_servo ~= 20 ms is <= the once-per-outer-frame command interval (20-100 ms), so a command-path
  filter would settle ~instantly and barely model lag; the FDM substep loop (200 Hz -> 2 kHz at higher
  cadences) resolves it; and it keeps actuator dynamics in the same layer the other craft variations
  already modify (`Global::craft*`). Files: servo slew->lag on aileron/elevator + thrust first-order lag
  in `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` (state in `.h`, reset to neutral every
  `initAirplaneState` for determinism); `Global::servoTau/servoSlew/thrustTau` (`crrcsim/src/global.*`)
  set per-scenario in `inputdev_autoc.cpp` (mirrors the `craftCGDelta` path).
- **The 3 dynamics params are craft variations** (evaluated: a ~4-craft fleet differs in servos/motors).
  `CraftServoTauSigma`/`CraftServoSlewSigma`/`CraftThrustTauSigma` wired end-to-end exactly like
  `craftThrustSigma` (config.h struct+macro, training inis only, `craft_variation.h` clamped draw
  appended at the bottom -- draw order frozen for determinism, `scenario_metadata.h`, the ramp in
  `scenario_meta_apply.h`, and the per-scenario apply). Centers tau_servo 0.020 s / slew 6.0 (full-
  throw/s) / tau_thrust 0.150 s, each Gaussian-drawn then CLAMPED to a positive physical range. Drawn
  values appear as new columns in the startup per-scenario variation table; sigmas auto-print via the
  config macro.
- **Servo-composition fix (2026-06-09, found via the basic-m1 smoke test).** The first basic-m1 run
  stalled (best plateaued, all-crash, avg flat) -- diagnosed to the servo filter COMPOUNDING slew x lag
  (it slew-clamped the error THEN multiplied by the lag blend), so the effective servo rate was ~4x too
  slow (~1.5 vs the intended 6 full-throw/s). Fixed in `fdm_larcsim.cpp`: exact dt-invariant lag
  `1 - exp(-dt/tau)` with the slew cap applied to the lag STEP (independent, no compounding); tau sets the
  response, slew only clips fast/large commands. Re-run is bit-deterministic (209/209 elite SAME, 0
  diverged) and the population learns again. The smoke test also surfaced that the **thrust lag is
  near-inert** -- it lags the `craftThrustScale` (~=1.0 multiplier), NOT the throttle->thrust path, so
  spool-up fidelity is NOT yet modeled; left as a separate rework.

**Determinism**: no new PRNG draws beyond the 3 appended craft draws (order unchanged); the FDM filter is
pure dt arithmetic with per-scenario state reset. A no-variation run should differ from pre-037 only by
the new nominal lag model + the energy-curve change. **Operator's bitwise regression gate must confirm
this after a clean build** (cross-component: autoc + crrcsim).

---

## Session implementation status (2026-06-10) — the 20 Hz flip

**Done (T011–T019, T021–T023 + tests; T020/T024 partial — see task annotations):** committed in two
stages so the 10 Hz bit-replay gate has a clean point:

1. **Stage-1 commit (`9592dea`)** — rescale refactor, bitwise no-op at the 10 Hz config:
   `kCadenceTickScale` on score/stability/energy accumulators (×1.0 exact at 100 ms), shared
   `engageDelayTicks()`, T014/T015 tests, research decisions (R1=20 Hz, R5 lag set, T013 sign-off).
   **Run the operator bit-replay gate against THIS commit** — the flip commit after it is
   intentionally not bitwise (new lag semantics).
2. **Stage-2 commit** — the flip: `SIM_TIME_STEP_MSEC=50`, inis=50, config default tracks the
   master, ms-based history lags (M1 `HIST_PAST` {32,16,8,4,2,0}, ring depth 33), shared
   `TrackerObservationRing` in both M2 mirrors, span_rate→rate, source-spacing fail-loud,
   `kNNHistoryLayoutVersion=2` dmp marker, nn_layout_tests. All 34 suites green.

**Notes for the operator:**
- crrcsim needs its own rebuild (`cd crrcsim/build && make -j8` or the usual script) — agent built
  autoc only per crrcsim/CLAUDE.md.
- Two pre-existing `craft_variation_tests` failures (stale vs 88a52ea env-only-ramp) were updated to
  the new contract — they were red BEFORE this session's changes (verified by stash).
- `SIM_TOTAL_TIME_MSEC` stays 100 s ⇒ 2000 ticks/scenario at 20 Hz ⇒ ~2× eval compute per scenario.
  Shortening scenarios is an open operator choice, NOT made here.
- Old t6/10 Hz dmps now fail loud on read (layout marker) — rate comparison uses recorded metrics
  per Q3, no replay. Old M2 source libraries fail loud on spacing — US1b needs a fresh 20 Hz
  source bake (already implied by the gate chain).
- xiao untouched (Phase B): `MSP_NN_EVAL_DIVISOR` stays 2 until T039.

**Update (later 2026-06-10): smoke runs + T005 + bake launch.**
- **t4 smoke FAIL** → second flip bug found+fixed (crrcsim `f81fd31`): pending-command apply ran
  after eval staging, so at framesPerEval=1 commands never reached the surfaces (3000 identical
  fitnesses). Apply moved to frame top; behavior-identical at 10 Hz.
- **t5 smoke PASS** (pop 3000/16 winds): historical-scale gen-1 spread, climb starts earlier and goes
  farther than the 10 Hz t2 baseline (gen 160 best -403 vs -246; t2 finished gen 280 at -277).
  Killed at gen 173. Details in finding.md.
- **T005 landed** (06364cd) after live dmp-dump reconstructability verification on the t5 dmp.
- **T026 bake LAUNCHED 11:38**: `autoc-037-t6-m1-20hz` (pop 5000 / 800 gens / 294 scenarios).
  Wall-clock impact of 2× ticks is modest (~20%, operator estimate): the FDM substep work — the
  dominant cost — is duration-based and unchanged; only NN evals/RPC/per-tick recording double.
  Gen 1-2 healthy: spread
  -2280/-1251/-1004 on the historical scale, elite bit-SAME, ~75 s/gen, slim log confirmed.
  **10 Hz bit-replay gate WAIVED** (operator: beyond 10 Hz now). Next: T027 `dealias_metrics.py`
  vs the 035-t6 recorded metrics.
