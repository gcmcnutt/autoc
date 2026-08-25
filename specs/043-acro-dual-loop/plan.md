# Implementation Plan: 043 — ACRO dual-loop

**Branch**: `043-acro-dual-loop` | **Date**: 2026-08-24 | **Spec**: [spec.md](spec.md)
**Input**: [spec.md](spec.md) — governs. Definition of record: spec.md **§ What ACRO is**.

## Summary

Reframe the NN's action space from **surface deflection** to **rotation-rate setpoint**, so the 2–5 Hz
oscillation measured on 041-t7 is handled by INAV's 2 kHz ACRO loop instead of by a 20 Hz outer loop that
structurally cannot damp it. crrcsim gains a model of INAV's fixed-wing rate controller; the xiao commands
ACRO instead of MANUAL; a few craft-variation axes are added for IMU imperfection and pitch damping; one M1 bake carries
all of it to a flight. ⭐ **No xiao log-format change** — US1 and FR-005 were both cut.

⭐ **Why this works at all** (operator 2026-08-24): *a stable planform does not need a PID the way an
unstable quad does.* The airframe is open-loop stable, so INAV's fixed-wing loop is **feed-forward with a
small PID for trim and disturbance rejection** — not a stabilizer. That is why `kFF` ≫ `kP`, why P and D
attenuate away as commanded rate rises, and why an *untuned* PID already damped this plant to 24 °/s.
⚠️ It also means the modelled loop's fidelity rests on the **airframe's** open-loop stability being right —
see research.md Finding 1a and its three consequences.

**Technical approach, from Phase 0**: add a new `Cntrl_InavFwRate` controller to crrcsim's **existing**
`mod_cntrl` framework — reusing the `Controller` base class, the XML gain-loading registry, and the
`fdm_larcsim` per-substep hook (which already fires *before* the 037 servo model, so servo lag lands
correctly **inside** the rate loop) — while writing the loop math to match what INAV actually does:
**feed-forward-dominant** (`kFF` 1.61 roll / 2.26 pitch vs `kP` 0.484), with **Gaussian setpoint
attenuation** on P and D, an I-term lock, and D on the gyro delta. The servo/mixer path downstream is
unchanged: MANUAL and ACRO both feed ±500 into the same mixer, so only what *fills* that range changes.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim), C (INAV fork), C++ (xiao / PlatformIO arduino-mbed), Python 3.11 (analysis)
**Primary Dependencies**: Eigen (vec3/quat), cereal (NN + `EvalResults` + dmp), inih (ini), GoogleTest, CRRCSim LaRCSim FDM, INAV MSP
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen), S3 `autoc-m1`; xiao QSPI flash log
**Testing**: GoogleTest (`run_autoc_tests`); `scripts/rebuild-perf.sh` bitwise eval-vs-training gate; bench + flight
**Target Platform**: Linux desktop (train), Seeed XIAO BLE Sense (deploy), STM32F722 (INAV — bench `MAMBAF722_2022A`, flight `MATEKF722MINI`)
**Project Type**: three build surfaces — autoc/crrcsim, xiao, INAV
**Performance Goals**: inner loop modelled at the FDM substep (~333 Hz, `Global::dt` ms-quantized — confirm as-run); outer loop 20 Hz unchanged; one 800-gen M1 bake
**Constraints**: determinism non-negotiable; `fw_*` gains fixed; INAV config as-is except FR-012a phase-delay params; 294 scenarios, no population increase
**Scale/Scope**: ~40 FRs across 5 live user stories; one production bake; one flight

## Constitution Check

*GATE: checked before Phase 0, re-checked after Phase 1.* Constitution v1.8.0.

| Principle | Status | Note |
|---|---|---|
| **I. Testing-First** | ✅ | SC-012 (zero command ⇒ zero rate, **and no self-levelling**) and SC-004 (trainability) are written as tests before the model exists. Contract tests in `contracts/`. |
| **II. Build Stability** | ✅ | Three surfaces each gated: `rebuild.sh`, `pio run`, INAV both targets. |
| **III. No Compatibility Shims** | ✅ | Rate command replaces surface command outright; no dual path. ⚠️ Watch: FR-014a rejects "adapt `Cntrl_Omega` in place" *because* it would be a shim. |
| **IV. Unified Build** | ⚠️ | New file in `crrcsim/src/mod_cntrl/` + FR-072 (`mod_inputdev` → link `autoc_common`) both touch CMakeLists ⇒ **clean `rebuild-perf.sh` required**, not an incremental reconfigure. Operator drives it. |
| **V. Versioned Persistence** | ✅ | `ScenarioMetadata` grows (new craft axes). Greenfield break, no cereal version bump, readers fail loud. ⛔ **FR-057 extraction must precede it.** |
| **VI. Type-Domain Discipline** | ✅ | New craft axes are `gp_scalar`; crrcsim-side is `SCALAR` (FDM-native, `// raw-ok`). Closing report runs the grep. |
| **VII. No Silent Fallback Defaults** | ✅ | New controller reads gains from XML with no in-class defaults on constructor-supplied members. |
| **VIII. Artifact Lifecycle** | ✅ | Bake tagged `retain=expire`; pinned + manifested only on promotion (FR-061). |
| **IX. Detached Training Launch** | ✅ | `scripts/train.sh`; pre-run build gate before the bake. |
| **X. Single Ordered Backlog** | ✅ | Deferrals (US1's log work, FR-07x not done) append to `specs/BACKLOG.md`. |

**Gate result: PASS.** One flagged item (IV) is a procedure requirement, not a violation — recorded so
`/speckit.tasks` places a clean rebuild after the CMakeLists edits.

## Project Structure

### Documentation (this feature)

```text
specs/043-acro-dual-loop/
├── README.md                  # derivation-of-record (seed)
├── spec.md                    # GOVERNS
├── plan.md                    # this file
├── research.md                # Phase 0 — R1 + R9 answered from source
├── data-model.md              # Phase 1
├── quickstart.md              # Phase 1
├── contracts/
│   ├── inav-fw-rate-loop.md   # what the model must reproduce
│   ├── action-space.md        # command → rate → surface, incl. polarity
│   └── craft-imu-axes.md      # the new variation axes
├── checklists/requirements.md
└── tasks.md                   # /speckit.tasks — NOT created here
```

### Source Code (repository root)

```text
crrcsim/src/mod_cntrl/
├── controller.cpp                      # + one else-if in LoadList
└── cntrl_inavfwrate/                   # NEW — the inner-loop model
    ├── cntrl_inavfwrate.h
    └── cntrl_inavfwrate.cpp
crrcsim/src/mod_fdm/fdm_larcsim/
└── fdm_larcsim.cpp                     # hook already present at :246 — no change expected
crrcsim/src/mod_inputdev/inputdev_autoc/
├── inputdev_autoc.cpp                  # command → rate setpoint; per-scenario gain/IMU plumbing
└── CMakeLists.txt                      # FR-072: link autoc_common
crrcsim/models/hb1_streamer.xml         # NEW <controllers> node — gains, rates, filters (FR-014)

include/autoc/eval/craft_variation.h    # + IMU axes, appended (FR-054)
include/autoc/rpc/scenario_metadata.h   # + the same fields, appended last
include/autoc/nn/nn_inputs.h            # unchanged unless R2 says otherwise
src/autoc.cc                            # draw + log the new axes
autoc.ini                               # new sigmas; ⛔ fix the stale craft-ramp comment (FR-051)

xiao/src/msplink.cpp                    # ACRO not MANUAL; unforce on disengage; FR-005 seq
~/inav/src/main/rx/msp_override.c       # FR-042 first-frame floor

tests/                                  # contract + unit tests per Constitution I
```

**Structure Decision**: no new top-level structure. The inner loop lands as a peer of the existing
`cntrl_*` modules — the framework is already wired into the FDM autoc uses, and that wiring (hook fires
before the servo model) is the single most valuable thing being reused.

## Phases

⛔ **Order comes from spec.md § Execution order, NOT from story priority.** One bake means every story is
upstream of US4.

| Ph | work | story | gate to leave |
|---|---|---|---|
| **0** | R1 + R9 from source | — | ✅ done — [research.md](research.md) |
| **1** | ⛔ **FR-057** — extract from the pinned 041-t7 dmps | US5 | extraction verified readable *before* any metadata change. **Permanent if skipped.** |
| **2** | Variation inventory + ramp-doc fix (FR-050/FR-051); **IMU axes** (FR-052) + **`craftCmQ`** (FR-052b); FR-052a bound | US5 | SC-009 (σ=0 bit-identical; σ>0 replayable; existing draws unchanged) |
| **3** | Housekeeping on the opened surfaces (FR-070…FR-076) | US6 | each item done or recorded deferred |
| **4** | `Cntrl_InavFwRate` + XML `<controllers>`; action space (FR-016 incl. **polarity check**); FR-019/019a/019b | US2 | **SC-012** incl. the no-self-levelling converse; SC-014 part 1 |
| **5** | Bench servo step-response; 037-constant review (⭐ **incl. static margin**); **FR-012a** phase-delay evaluation; ⚠️ **read the notch centre frequency from the 041-t7 blackbox** | US2 | SC-010; every constant marked changed-or-checked |
| **6** | INAV fork (FR-042), ACRO engage, bench verification, telemetry-rate measurement (FR-044) | US3 | bench shows ACRO in, mode released out, no yaw surface response |
| **7** | Arm's-length answer (R2 / FR-030/FR-031) against a 45-input baseline | US2 | SC-011 — including "nothing" |
| **8** | Production bake (FR-060/FR-061) | US4 | Constitution IX pre-run gate; **SC-004 trainability** before committing |
| **9** | Flight + outcome (FR-062/FR-063) | US4 | SC-001, **SC-001a (the magnitude)**, SC-003, SC-005, SC-006, SC-013, SC-014 part 2 |

### Requirements added or changed after this plan's first draft — now covered above

| | landed | phase |
|---|---|---|
| **FR-012a** INAV phase-delay params IN scope (gains stay fixed) | 2026-08-24 | 5 |
| **FR-013** ⛔ corrected — only `gyro_main_lpf_hz` is inside the ACRO loop; `acc_lpf_hz` is observation-path | 2026-08-25 | 4 |
| **FR-019a/b** no self-levelling; setpoint accel limit off | 2026-08-23 | 4 |
| **FR-052a** gross misconfiguration out of scope | 2026-08-23 | 2 |
| **FR-052b** `craftCmQ` pitch-damping axis | 2026-08-25 | 2 |
| **SC-001a** the result carries a **magnitude** | 2026-08-25 | 9 |
| ⛔ **FR-001…FR-005 removed** (US1 dropped, FR-005 cut) ⇒ **no xiao log-format change in 043** | 08-24 / 08-25 | — |
| ⛔ **No inner-loop gain variation** — `craftGyroScale` carries it | 2026-08-25 | 2 |

⚠️ **Phases 1–3 are the format-break window** — one break, sequenced so the pinned baseline is mined first.

## Complexity Tracking

| Item | Why needed | Simpler alternative rejected because |
|---|---|---|
| New `Cntrl_InavFwRate` rather than extending `Cntrl_Omega` (✅ confirmed 2026-08-24) | R1 shows every term differs (FF-dominant, Gaussian attenuation, I-lock, INAV limits) | Extending in place replaces all the math while keeping the name — a shim (Constitution III), and it breaks `Cntrl_Omega` for any other consumer |
| Three build surfaces | FR-042's engage-latency floor is an INAV fork change; ACRO selection is xiao | Neither is reachable from the sim side |
| One bake carrying ACRO + variations + housekeeping | Operator decision (spec assumption 12) | Two bakes cost ~27 h each; attribution handled post-hoc against 041-t7 |
