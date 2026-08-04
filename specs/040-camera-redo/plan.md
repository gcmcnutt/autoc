# Implementation Plan: 040 Camera Redo — Perception Fidelity Refinement for M2

**Branch**: `040-camera-redo` | **Date**: 2026-07-28 | **Spec**: [spec.md](spec.md)
**Input**: [spec.md](spec.md) + [input-data-checklist.md](input-data-checklist.md) (input of record — measured
hardware values, sources, open items; consume rather than re-derive)

## Summary

Replace the sim's placeholder beacon-camera model with one grounded in buildable hardware, then retrain M2
against it. Four substantive changes to the chase-side perception chain — **pixel-grid geometry in isotropic
radians**, **obstruction from the measured airframe**, **signal-quality CEP driven by a link budget and an
acquisition state machine**, and **per-scenario camera variation** — plus a gating airframe-fidelity check
and an aggregate-delta M2 retrain.

**Governing constraint is architectural, not numeric**: *plumbing first*. Every physical quantity becomes a
configured value classified measured/derived/assumed, so a later real-to-simulation calibration lands as
config edits rather than redesign (FR-034/035/036). Numeric accuracy is explicitly provisional.

**Blast radius is deliberately small**: perception runs chase-side at M2 train time, so **no M1 source rebake**
and **no change to the 58-input controller vector**. Tracker mode is `#ifndef ARDUINO`, so **xiao is
untouched**. The feature runs parallel to M1 flight-test work.

## Technical Context

**Language/Version**: C++17 (autoc + crrcsim); Python 3.11 (analysis/plots only)
**Primary Dependencies**: Eigen (vec3/quat), cereal (dmp + EvalData wire), inih (ini), GoogleTest, CRRCSim
LaRCSim FDM
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen), S3 `autoc-m2` / `autoc-eval`
per-mode buckets (`<run-id>/gen<N>.dmp`)
**Testing**: GoogleTest under `tests/`, registered via `run_autoc_tests`; determinism via the
`rebuild-perf.sh` FP-deterministic build + eval-vs-training bitwise gate
**Target Platform**: Linux desktop (training). **xiao explicitly out of scope** — `gather_tracker_inputs` is
`#ifndef ARDUINO`; tracker mode has never shipped to firmware
**Project Type**: evolution/simulation engine — three components (autoc evolution, crrcsim FDM, xiao embedded)
**Performance Goals**: total evaluation throughput regression **≤10%** benchmarked against the prior M2 run
(FR-037/038, SC-013). Perception sits in the innermost loop: ~294 scenarios × ~450–900 ticks × population ×
2 beacons per generation
**Constraints**: bit-determinism non-negotiable (FR-020, SC-006); no PRNG in the signal path; NN input vector
stays 58 (FR-006); no M1 source regeneration (FR-006, SC-009); dmp changes limited to diagnostics (FR-029)
**Scale/Scope**: ~6 headers + ~5 implementation files + 5 existing test files extended; 1 gating research
memo; 1 M2 training run + novel-path eval

### Ground truth established during planning

Three findings from reading the tree that shape the work — recorded fully in [research.md](research.md):

1. **`TrackerStepper` is test-only.** The constitution states crrcsim FDM is the *sole worker since 034;
   minisim retired*, and `TrackerStepper` now appears only in `tests/` plus comments. The production tracker
   tick is `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp`. **This corrects FR-031's
   stated rationale** — it is not two live production paths but one production path plus a test-only
   reference. The duplication still matters (the tests are the contract and would silently encode stale
   behaviour), but it is a *correctness-of-tests* concern, not a production-divergence one.
2. **Camera config is already wired.** `src/autoc.cc:955-960` reads `CameraFOVHorizontalDeg`,
   `CameraFOVVerticalDeg`, `CameraMountOffset{X,Y,Z}` from the ini into `CameraConfig`. FR-032 is therefore
   partially satisfied; the remaining work is the new signal-model keys plus retiring the stale
   `frame_rate_hz` / `latency_ms`.
3. **The dmp append convention is established in-code.** `protocol.h` documents append-at-end-of-v≥2-block
   with no `CEREAL_CLASS_VERSION` bump, old dmps orphaned. The diagnostic fields follow that convention.

## Constitution Check

*GATE: evaluated before Phase 0; re-evaluated after Phase 1 design.*

| Principle | Assessment | Status |
|---|---|---|
| **I. Testing-First** | Five existing test files carry the contract and will be extended before implementation: `beacon_projection_tests.cc` (geometry, quantisation, obstruction), `gather_tracker_inputs_tests.cc` (input assembly, CEP gating), `contract_tracker_config_tests.cc` (ini field-count + new signal keys), `tracker_dmp_roundtrip_tests.cc` (diagnostic fields), `tracker_stepper_init_tests.cc` (per-scenario state reset, FR-020a). New: signal-model and variation tests. | ✅ PASS |
| **II. Build Stability** | `scripts/rebuild.sh` compile-and-test gate before any commit; xiao unaffected so its build gate is not in play. | ✅ PASS |
| **III. No Compatibility Shims** | Clean cut throughout: int8 bearing encoding is **removed**, not deprecated alongside a replacement; the linear CEP placeholder is **replaced**, not switched. Old M2 elites are invalidated — accepted, retraining is the planned outcome (Assumption 10). | ✅ PASS |
| **IV. Unified Build** | No new third-party dependencies. New test targets touch `CMakeLists.txt` ⇒ that change MUST go through a clean `scripts/rebuild-perf.sh`, not an incremental reconfigure. Operator drives the clean rebuild. | ✅ PASS *(with the rebuild-perf obligation recorded)* |
| **V. Versioned Persistence** | `BeaconObservation` gains pixel-coordinate + diagnostic fields; `CameraViewSample` carries them into the dmp. Follows the in-code convention documented at `protocol.h:428-447` — append at end of the v≥2 block, no `CEREAL_CLASS_VERSION` bump, **old dmps orphaned** (they fail to parse rather than mis-parse, which is the fail-loud outcome the principle requires). The M1 source format is untouched, so M1 dmps remain readable. | ✅ PASS |
| **VI. Type-Domain Discipline** | New perception code uses `gp_scalar` / `gp_vec3`. The pixel-coordinate and diagnostic fields are cereal byte-format ⇒ `// raw-ok:` annotated at declaration. **Milestone-closing obligation**: run the principle's grep over `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` and annotate-or-convert every hit in the diff. | ✅ PASS *(audit obligation carried into every milestone)* |
| **VII. No Silent Fallback Defaults** | **Directly load-bearing here.** The constitution cites the `cepGateThreshold` bug — which lives in *this exact code* — as the motivating failure. Every new signal-model value flowing from `WorkerInit` MUST have no in-class default; the constructor initializer list is the single assignment site. | ✅ PASS *(highest-attention principle for this feature)* |
| **VIII. Training-Artifact Lifecycle** | The M2 retrain uploads tagged `retain=expire`; if the resulting controller becomes a baseline it is pinned `retain=keep` and its S3 prefix recorded in the outcome doc. | ✅ PASS |
| **IX. Detached Training Launch** | Retrain launched via `scripts/train.sh <ini> <logfile>` — never a harness-tracked background task. Pre-run build gate applies: `rebuild-perf.sh` before the run, since perception changes are determinism-affecting. | ✅ PASS |
| **X. Single Ordered Backlog** | All deferrals already filed in `specs/BACKLOG.md` (camera-hardware phase, raptor binocular arrangement, IMU misalignment, engine-speed propeller work). No per-item files created. | ✅ PASS |

**Gate result**: PASS, no violations to justify. Two standing obligations carried into implementation —
the Principle VI grep at every milestone close, and Principle VII discipline on all new `WorkerInit`-sourced
values.

## Project Structure

### Documentation (this feature)

```text
specs/040-camera-redo/
├── spec.md                     # feature specification (44 FR, 13 SC, 7 stories)
├── input-data-checklist.md     # INPUT OF RECORD — measured values, sources, open items
├── plan.md                     # this file
├── research.md                 # Phase 0 — ground truth + design decisions
├── data-model.md               # Phase 1 — entities, fields, state transitions
├── quickstart.md               # Phase 1 — how to run, verify, and calibrate
├── contracts/                  # Phase 1 — perception interface + config + dmp contracts
├── checklists/requirements.md  # spec quality checklist (passing)
├── camera_considerations.md    # 040 reference — sensor/link-budget (rate-stale banner)
└── camera-hardware-phase/      # PARKED — recorder chain; not in this feature
```

### Source code (repository root)

```text
include/autoc/eval/
├── camera_projection.h        # BeaconObservation: pixel coords replace int8; + diagnostics
├── camera_config.h            # sensor grid + deg/px; retire frame_rate_hz / latency_ms
├── beacon_config.h            # separation 0.9 → 0.772 m; flat-top emission params
├── signal_model.h             # NEW — link budget → per-chip SNR → quality
├── acquisition_state.h        # NEW — chip-credit integrator + lock/hold state machine
└── airframe_occlusion.h       # NEW — wing slab + nose + prop disc (replaces the AABB proxy)

src/eval/
├── camera_projection.cc       # pixel quantisation, isotropic radians, effective FOV
├── signal_model.cc            # NEW
├── acquisition_state.cc       # NEW
└── airframe_occlusion.cc      # NEW

crrcsim/src/mod_inputdev/inputdev_autoc/
└── crrcsim_tracker_helper.cpp # PRODUCTION tick — consumes the shared per-tick rule

src/eval/tracker_stepper.cc    # test-only reference — must consume the SAME shared rule
include/autoc/util/config.h    # new signal-model + variation ini keys (X-macro)
include/autoc/util/scenario_prng.h        # camera sub-seed (slot 5, reserved, unused)
include/autoc/rpc/scenario_metadata.h     # cameraSeed + variation draws (craft pattern)
include/autoc/rpc/protocol.h              # CameraViewSample diagnostic fields
src/autoc.cc                   # worker init: new config → WorkerInit (Principle VII)
tools/renderer.cc              # POV panel: radians scale, effective FOV, obstruction
tools/dmp_dump.cc              # emit new diagnostics

tests/
├── beacon_projection_tests.cc      # EXTEND — quantisation, isotropy, obstruction
├── gather_tracker_inputs_tests.cc  # EXTEND — CEP semantics, tentative lock
├── contract_tracker_config_tests.cc# EXTEND — ini field count + new keys
├── tracker_dmp_roundtrip_tests.cc  # EXTEND — diagnostic round-trip
├── tracker_stepper_init_tests.cc   # EXTEND — per-scenario state reset (FR-020a)
├── signal_model_tests.cc           # NEW — link budget, CDMA, monotonicity
├── acquisition_state_tests.cc      # NEW — timing, hold, determinism
└── camera_variation_tests.cc       # NEW — reproducibility, zero-sigma identity
```

**Structure Decision**: extend the existing `src/eval/` perception module rather than introducing a new
top-level component. Perception is already an eval-pipeline concern (`camera_projection` lives there, the
tracker stepper consumes it, crrcsim links `autoc_common`), and Principle IV's single-CMakeLists rule makes
a peer directory pure overhead. The three new headers/implementations are siblings of
`camera_projection`, not a new subsystem.

## Implementation Sequencing

Ordering is driven by two hard constraints rather than convenience.

> **Naming note**: stages are lettered **A–H**, *not* numbered M0–M7. In this project **M1 / M2 / M3 denote
> controller modes** (pathgen / tracker / optical+ToF) and appear throughout this document in that sense —
> "no M1 source rebake", "the M2 retrain". Numbered milestones would collide with that vocabulary exactly
> where a misread is most expensive.

| Stage | Work | tasks.md | Why here |
|---|---|---|---|
| **A** | **Airframe-fidelity verdict** (US1, FR-024/025/026) | Phase 2 | **Gating.** A verdict of "regenerate" would force an M1-source rebake *before* any perception work, since the M2 controller trains off the M1 source. Expected outcome is "defer", but it must be established, not assumed. Cheap: a document, no code. |
| **B** | **Shared per-tick rule** (FR-031) + obstruction-proxy fix + config surface (FR-032) | Phase 3 | Prerequisite. The acquisition state machine must land in exactly one place; doing this later would mean writing it twice. Also fixes the degenerate default proxy (camera on `box_min_z`) before anything depends on obstruction. |
| **C** | **Pixel-grid geometry** (US2, FR-001–006) | Phase 4 | Foundation — every later quantity is expressed in the new representation. Includes the 0.772 m separation correction. |
| **D** | **Obstruction** (US3, FR-007–013) | Phase 5 | Depends on Stage C's angular representation and Stage B's proxy fix. Validates the leading-edge mount across the variation envelope. |
| **E** | **Signal-quality CEP** (US4, FR-014–020a) | Phase 6 | The feature's core. Depends on Stage C (geometry) and Stage D (obstruction feeds the budget). |
| **F** | **Camera variation** (US6, FR-021–023) | Phase 7 | Layers on top; needs Stages C–E in place to vary anything meaningful. |
| **G** | **Instrumentation + optics record** (US7, FR-027–030) | Phases 8, 10 | Diagnostics and the durable optics artefact. |
| **H** | **Retrain + evaluate** (US5, FR-037/038) | Phase 9 | Terminal. Throughput benchmark against the prior M2 run, then novel-path eval, aggregate delta reported. |

Stages A and B are genuinely blocking. C→D→E is a strict dependency chain. F and G could run in either
order; G is documentation-heavy and may float.

### Verification type per stage — where operator time is needed

| Stage | Automated verification | Operator / manual |
|---|---|---|
| **A** | — | **The entire stage.** Comparison memo + a recorded decision. Gate. |
| **B** | **Bit-identity** — pinned elite evals `NN_EVAL_SAME`. A behaviour-preserving refactor that changes behaviour has a bug | Operator drives `rebuild-perf.sh` (Principle IV) |
| **C** | Isotropy, pixel quantisation, separation invariance, range-vs-truth | **Renderer POV panel** — the ±1 convention is retired and the scale changes; confirm beacons land where expected |
| **D** | Visibility-map geometry assertions | **Read the obstruction-onset distribution.** Per the "let it ride" clarification this stage's output is an observation, not a pass/fail |
| **E** | Monotonic falloff, acquisition timing, hold ride-through, determinism | **Playback.** Does dropout/reacquire look physical? Is the ~25 m range-envelope crossover where expected? Unit tests cannot answer this |
| **F** | Reproducibility from scenario id; zero-sigma bit-identity | Spot-check only |
| **G** | dmp round-trip | **Renderer visual** (effective FOV, obstructed regions, quality regimes) + **read the optics record** |
| **H** | Throughput benchmark vs the prior M2 run | **Effectively the entire stage** — pre-run gate, launch, monitor, judge the plateau, write the aggregate delta |

**Three hard operator gates**: A (a decision), B (clean rebuild), H (pre-run gate then judgment).

**Four visual checks**, all renderer-mediated: C, D, E, G. **Stage E is the one not to skip** — the signal
model is where "physically plausible" and "passes its unit tests" diverge most easily, and playback is the
only instrument that catches it.

**Stage H is the long pole.** A–G are code and documents; H is a training run plus a novel-path eval — days
of wall clock, and the only stage whose verdict is a judgment rather than a check.

**Not required anywhere in 040**: bench hardware (031 supplies recorded measurements; 040 consumes them)
and flight testing (that is the parallel M1-controller flight-test track). Nothing here blocks on weather,
hardware availability, or a field trip.

**The asymmetry worth respecting**: Stage A costs almost nothing and gates everything. A "regenerate"
verdict reorders the whole feature behind an M1-source rebake — the cheapest possible guard against the
most expensive possible surprise.

## Phase 0: Research

See [research.md](research.md). Topics resolved:

1. Production vs test tracker path — and what FR-031 should actually do about it
2. Pixel quantisation and the isotropic-radian representation; residual anisotropy
3. Obstruction primitive selection (slab vs box) and the degenerate-proxy fix
4. Signal-budget shape and its calibration against 031 bench data
5. Acquisition state machine — states, timing, determinism strategy, per-scenario reset
6. Camera-variation plumbing modelled on the existing craft-variation pattern
7. dmp diagnostic-field convention
8. Throughput measurement method against the prior M2 run

## Phase 1: Design & Contracts

- [data-model.md](data-model.md) — entities, fields, units, validation, state transitions
- [contracts/perception-interface.md](contracts/perception-interface.md) — the `(bearing, quality)` contract
  the controller consumes, and what is deliberately *not* exposed
- [contracts/config-surface.md](contracts/config-surface.md) — every new ini key with default, unit, and
  measured/derived/assumed classification (FR-035)
- [contracts/dmp-diagnostics.md](contracts/dmp-diagnostics.md) — the diagnostic fields and their convention
- [quickstart.md](quickstart.md) — build, test, verify determinism, run the retrain, calibrate later

## Complexity Tracking

No constitution violations. Two deviations from the spec's stated assumptions, both simplifications,
recorded here rather than as violations:

| Item | Spec said | Reality | Effect |
|---|---|---|---|
| FR-031 rationale | "duplicated across two execution paths" | One production path + one test-only reference | Work still required (tests are the contract), but it is a test-correctness fix, not a production-divergence fix. Lower risk than the spec implies. |
| FR-032 scope | "configuration values MUST be externally settable" | FOV and mount offsets already wired at `autoc.cc:955-960` | Less work than stated; remaining scope is the new signal-model keys plus retiring stale fields. |
