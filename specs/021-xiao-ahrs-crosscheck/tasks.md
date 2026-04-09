# Tasks: 021 — AHRS Cross-Check + NN Input Redesign

**Input**: Design documents from `/specs/021-xiao-ahrs-crosscheck/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Organization**: Sim-first approach. All sim-side changes (CRRCSim rate PID,
NN input redesign, training validation) are completed and proven BEFORE
touching flight hardware. INAV/xiao changes are plumbing to replicate
what the sim already provides.

## Status (2026-04-07, post-022)

| Phase | Status | Notes |
|-------|--------|-------|
| 1: Bench Verification | ✅ DONE | Polarity verified, INAV PID config documented |
| 2: CRRCSim Rate PID | 📁 ARCH-DEFER | Infrastructure built; PID intentionally disabled — NN learns own rate control via gyro inputs |
| 3: NN Input Redesign 29→27 | ✅ DONE | NN_INPUT_COUNT=27, gyro p/q/r at indices 24-26 |
| 4: Training | ✅ DONE | Superseded by 022 betterz2 (400 gens, V4 conical, best -34771) |
| 5: INAV MSP Extension | ✅ DONE | gyro[3] in MSP2_AUTOC_STATE, both targets built |
| 6: Xiao Consumer + ACRO | ⚠️ PARTIAL | Gyro consumer + sign correction done; ACRO mode override DEFERRED (NN uses MANUAL with gyro inputs) |
| 7: Characterization Flight | ⚠️ PARTIAL | 4 flights captured; clean rate-response data needed next flight |
| 8: Safety Overrides | 📁 DEFERRED | Pilot stick + INAV failsafe primary; range cutoff is backup |
| 9: Production Flight | ⚠️ PARTIAL | Hardware ready; needs betterz2 weights deployed and flown |
| 10: Polish | ⚠️ PARTIAL | COORDINATE_CONVENTIONS.md updated; final commit pending |

**Legend**: ✅ DONE | ⚠️ PARTIAL | 📁 DEFERRED (intentional) | ❌ NOT DONE

**Critical path to next flight**: redeploy xiao with 022 betterz2 weights, ground check, fly.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US*]**: User story mapping:
  - US1: CRRCSim ACRO rate PID (sim-side inner loop)
  - US2: NN input redesign (new sensor vector)
  - US3: Training + sim validation
  - US4: INAV MSP extension (gyro in MSP2_AUTOC_STATE)
  - US5: Xiao consumer + ACRO mode override
  - US6: Safety overrides
  - US7: Characterization flight + analysis (parallel, when props arrive)

---

## Phase 1: Bench Verification

**Purpose**: Confirm gyro/accel polarity and ACRO config on bench hardware before any code changes.

- [X] T001 Ground polarity check on bench (verified 2026-03-30, commit 02325db: pitch/yaw INVERTED from aerospace RHR; documented in COORDINATE_CONVENTIONS.md)
- [X] T002 [P] INAV ACRO PID config documented in xiao/inav-bench.cfg and inputdev_autoc.h

**Checkpoint**: Gyro polarity confirmed. ACRO gains documented. Safe to implement sim-side rate PID.

---

## Phase 2: CRRCSim ACRO Rate PID (US1) — MVP

**Goal**: Implement rate PID in CRRCSim that converts NN rate commands to surface deflections, matching INAV's ACRO behavior. This is the sim-side inner loop.

**Independent Test**: Train a short run (50 gens) with rate PID active and existing 29 inputs. NN should converge — completion rate >80% on aeroStandard paths. Compare control smoothness to BIG3 baseline.

**ARCHITECTURAL DECISION (2026-04-07)**: Rate PID is **intentionally disabled**. The NN
receives gyro rates as inputs (p, q, r at indices 24-26 of the 27-input vector) and
learns its own rate-control strategy. NN outputs map directly to surface deflections
(MANUAL mode in INAV). The rate PID infrastructure exists in code but is not active.

- [X] T010 [US1] Gyro rate fields in AircraftState (aircraft_state.h L256-263)
- [X] T011 [US1] Body rates wired through minisim RPC (verified)
- [📁] T012 [US1] Rate PID implementation — DEFERRED (NN learns own rate control via gyro inputs)
- [X] T013 [US1] Autoc rebuild + smoke test (training runs validate architecture)
- [📁] T014 [US1] CRRCSim rate PID — DEFERRED. Body rate extraction code present (v_R_omega_total → AircraftState), but PID computation not active. inputdev_autoc.cpp L1005-1010 explicitly disables.
- [X] T015 [US1] CRRCSim builds successfully with current direct-control config

**Checkpoint**: CRRCSim has rate PID. NN outputs rate commands. Sim behavior plausible.

---

## Phase 3: NN Input Redesign (US2)

**Goal**: Change NN input vector from 29 to 27 inputs. Remove alpha/beta and previous commands, add rate gyros from FDM.

**Independent Test**: Train 50 gens with new inputs + ACRO rate PID. Compare convergence to Phase 2 baseline (same PID, old inputs). New inputs should converge at least as well.

- [X] T020 [US2] Body rates wired through RPC (AircraftState gyro fields populated by CRRCSim FDM)
- [X] T021 [US2] nn_gather_inputs() updated — alpha/beta and prev commands removed, gyro rates p/q/r added at indices 24-26 (evaluator.cc L235-244). Note: stored as rad/s unscaled, NN learns natural scale.
- [X] T022 [US2] NN_INPUT_COUNT=27 in topology.h, topology string "27,16,8,3" verified
- [X] T023 [P] [US2] Tests pass with 27-input topology
- [X] T024 [US2] Autoc rebuilds, eval pipeline validated

**Checkpoint**: 27-input NN compiles and trains. Gyro rates wired in sim. Alpha/beta and previous commands removed.

---

## Phase 4: Training + Sim Validation (US3)

**Goal**: Validate the new architecture converges. Short runs first, then production.

**Independent Test**: Eval suite passes (>95% completion on aeroStandard). Smoothness metrics comparable or better than BIG3.

- [X] T030 [US3] Short training runs done (autoc-021-test1..5.log, autoc-021-crrcsim1..11.log)
- [X] T031 [US3] Response analysis infrastructure available
- [X] T032 [US3] BIG production run completed — superseded by 022 betterz2 (400 gens, V4 conical surface, best -34771)
- [X] T033 [US3] Eval suite infrastructure exists (scripts/eval_suite.sh)
- [📁] T034 [US3] Response analysis vs flight data — pending next characterization flight

**Checkpoint**: Trained controller with new architecture. Eval passes. Sim validated.

---

## Phase 5: INAV MSP Extension (US4)

**Goal**: Add filtered gyro[3] to MSP2_AUTOC_STATE response. Only proceed after sim validation (Phase 4) confirms the architecture works.

**Independent Test**: Bench test: xiao receives and logs gyro rates from INAV. Compare to blackbox gyroADC[0-2] — should match within rounding.

- [X] T040 [US4] MSP2_AUTOC_STATE gyro[3] extension shipped (~/inav/src/main/fc/fc_msp.c L705-711)
- [X] T041 [US4] INAV builds for bench (MAMBAF722_2022A) and flight (MATEKF722MINI)
- [X] T042 [US4] INAV flight binary built and ready for deployment

**Checkpoint**: INAV sends gyro rates via MSP. Both targets built.

---

## Phase 6: Xiao Consumer + ACRO Mode (US5)

**Goal**: Xiao parses extended MSP, populates gyro rates for NN, overrides to ACRO mode.

**Independent Test**: Bench test with autoc enabled: xiao log shows gyro rate values, flight mode shows ACRO|MSPRCOVERRIDE, NN outputs interpreted as rate commands.

- [X] T050 [US5] msp_autoc_state_t has int16_t gyro[3] field (xiao/include/MSP.h L260-273)
- [X] T051 [US5] MSP parser extracts gyro[3] (xiao/src/msplink.cpp L519, L672-674) with sign correction (pitch/yaw negated)
- [X] T052 [US5] Xiao nn_gather_inputs populates gyro rate inputs at indices 24-26 (rad/s, sign-corrected)
- [📁] T053 [US5] ACRO mode override — DEFERRED. Xiao keeps CH6=1000 (MANUAL). NN uses gyro rates as inputs to learn its own rate control instead of using INAV ACRO PID. Architectural choice (see Phase 2).
- [X] T054 [US5] rc_expo=0 in bench config (xiao/inav-bench.cfg)
- [X] T055 [US5] Weight extraction → nn2cpp → xiao build pipeline working
- [X] T056 [US5] Bench verification done (commit b4f2f19: "feat(021): xiao MSP gyro consumer + nn2cpp fix + bench verified")

**Checkpoint**: Full pipeline on bench: INAV → MSP (with gyro) → xiao → NN (27 inputs) → rate commands → INAV ACRO PID → servos.

---

## Phase 7: Characterization Flight + Analysis (US7)

**Goal**: Fly the choreography from `flight-choreography.md`. Collect response data for sim tuning.

**Note**: Runs in parallel with Phases 2-6 whenever props arrive. No code changes needed — uses current firmware with updated blackbox config. Data feeds back into T034 (PID tuning).

- [X] T070 [P] [US7] Blackbox config applied to flight hardware (1/32 rate, GYRO_RAW + ACC + QUAT enabled per flight-plan.md)
- [⚠️] T071 [P] [US7] Partial: 4 flights captured (2026-03-20, 03-22, 03-27, 04-03). 03-27 was the tumble. Better characterization data expected next flight.
- [⚠️] T072 [US7] Partial: blackbox CSVs decoded for available flights
- [📁] T073 [US7] Response analysis vs sim — pending next clean characterization flight
- [📁] T074 [US7] Gyro filter (25Hz LPF) analysis — pending

**Checkpoint**: Response data collected. Sim PID tuning validated or updated.

---

## Phase 8: Safety Overrides (US6)

**Goal**: Distance-from-origin sphere check on xiao. Disables autoc if aircraft drifts too far. Pilot override remains primary safety; this is a backup.

- [📁] T060 [US6] Distance-from-origin sphere check — DEFERRED. Origin capture exists (test_origin_offset, msplink.cpp L21-22, L408) but no threshold check. Rely on pilot stick override + INAV failsafe + RC disarm as primary safety.
- [📁] T061 [US6] Safety state log — DEFERRED with T060
- [📁] T062 [US6] Bench test of safety override — DEFERRED with T060

**Checkpoint**: Safety override active.

---

## Phase 9: Flight Test

**Goal**: Fly with new NN (ACRO mode, 27 inputs, rate commands). Expect sustained tracking.

- [X] T080 INAV with MSP gyro extension flashed to flight hardware
- [⚠️] T081 Xiao deployed with production weights — current is older training, **needs update to 022 betterz2 weights with V4 conical surface**. Safety/ACRO deferred per Phase 6/8 architectural decisions.
- [⚠️] T082 Ground check passed for current weights (commit b4f2f19). Re-verify with betterz2 weights before next flight.
- [⚠️] T083 4 flight tests captured (2026-03-20 → 04-03). Next flight needed with betterz2 + V4 conical NN.
- [⚠️] T084 Post-flight analysis: data exists for prior flights. Pending next-flight comparison vs sim sustained-lock predictions (median 4.9m, 45% locked, max 14s lock).

**Checkpoint**: Flight validated. Architecture proven.

---

## Phase 10: Polish

- [X] T090 [P] COORDINATE_CONVENTIONS.md updated with gyro rate wiring + sign conventions
- [📁] T091 [P] BACKLOG.md update — pending end of feature
- [📁] T092 Commit final configs — pending

---

## Dependencies & Execution Order

### Critical Path (sim-first)

```
T001-T002 (bench polarity/config check)
    ↓
T010-T013 (minisim rate PID — MVP, validates architecture)
    ↓
T014-T015 (CRRCSim rate PID — full physics)
    ↓
T020-T024 (NN input redesign, 29→27)
    ↓
T030-T034 (training + sim validation)
    ↓  ← T074 feeds back PID tuning here
T040-T042 (INAV MSP extension)
    ↓
T050-T056 (xiao consumer + ACRO)
    ↓
T060-T062 (safety overrides)
    ↓
T080-T084 (flight test)
    ↓
T090-T092 (polish)
```

### Parallel Track (characterization flight)

```
T070-T074 (whenever props arrive, no code dependency)
    ↓
feeds back into T034 (CRRCSim PID tuning)
```

### Parallel Opportunities Within Phases

- **Phase 1**: T001, T002 can run in parallel
- **Phase 3**: T023 (GoogleTest) parallel with T020-T021
- **Phase 5**: T040, T041 can overlap (design MSP while building)
- **Phase 8**: entirely parallel with Phases 2-6

---

## Implementation Strategy

### MVP: Minisim Rate PID (Phase 2, T010-T013)

1. Implement rate PID with minisim (simplified physics, fast iteration)
2. Train with existing 29 inputs but ACRO mode
3. **STOP and VALIDATE**: does ACRO help convergence?
4. If yes: port to CRRCSim (T014-T015), proceed to Phase 3
5. If no: investigate PID tuning before continuing

### Incremental

1. Phase 1 → bench polarity confirmed
2. Phase 2 → ACRO in minisim then CRRCSim (MVP — validates architecture)
3. Phase 3 → new 27 inputs (validates sensor changes)
4. Phase 4 → production training (validates at scale)
5. Phase 5+6 → INAV/xiao plumbing (replicates sim on hardware)
6. Phase 7 → characterization data (parallel, feeds back to PID)
7. Phase 8 → safety overrides
8. Phase 9 → flight test (endgame)
