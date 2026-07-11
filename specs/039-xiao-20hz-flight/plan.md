# Implementation Plan: 039 Xiao 20 Hz Flight — embedded control-loop catch-up

**Branch**: `039-xiao-20hz-flight` | **Date**: 2026-07-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/039-xiao-20hz-flight/spec.md`

## Summary

Fly 038's M1 elite (37-in/2051-w t5 rebake) at 20 Hz on the xiao and demonstrate sim-grade control
smoothing in the real flight logs (SC-005: per-axis dCtrl/amplitude within ±25% of sim). Technical
approach from the plan-phase research ([research.md](research.md)): regenerate the firmware to the
038 contract with a **mutable engage-centered arena** (D5) and **unrolled-recurrent NN codegen**
(D6); replace the per-tick text logging with a **versioned int16-quantized packed binary flight log**
carrying all 37 inputs + 3 outputs + aux state (D3), with console reduced to events + heartbeat
(D4); run the NN every tick at 20 Hz on the existing transport (fits at 115200 with 82% margin, D7);
re-bench the pipeline latency on the new firmware and put the amend-model/retrain decision to the
operator with numbers (D1); fly, download over BLE, and produce the per-axis sim-vs-real comparison.

## Technical Context

**Language/Version**: C++17 (autoc tools/desktop reader), C++ (xiao, PlatformIO arduino-mbed,
nRF52840), C (INAV fork), Python 3.11 (ground decode + sim-vs-real analytics)
**Primary Dependencies**: nn2cpp codegen (`tools/nn2cpp.cc`), xiao QSPI flash logger + BLE stack,
custom MSP transport (`MSP2_AUTOC_STATE` / `MSP_SET_RAW_RC`), crrcsim (latency/servo model + any
FR-005 retrain), Eigen (arena math)
**Storage**: xiao QSPI flash (2 MB log region, pre-erased ring, ground `ERASE:ALL`); new versioned
binary flight-log format (contracts/flight-log-format.md); S3 pinned t5 elite as weight source
**Testing**: GoogleTest desktop-side (nn2cpp unrolled≡table-driven equivalence, arena re-center
math, log encode/decode round-trip); PlatformIO build gate (Constitution II); stationary-bench
observational span (FR-002); DWT cycle-count harness on target (037 eval-cycle-harness contract)
**Target Platform**: Seeed XIAO BLE Sense (nRF52840, Cortex-M4F @64 MHz) + INAV FC (custom fork at
~/inav) + Linux desktop (tools/analytics)
**Project Type**: embedded firmware + codegen tool + ground tooling (multi-component repo:
autoc / crrcsim / xiao)
**Performance Goals**: 20 Hz control tick (50 ms) with zero overruns; NN eval ~0.1 ms class
(unrolled) vs 2.6 ms today; per-tick log ≤ ~100 B (int16-packed ~93 B) at ≤ ~2 KB/s write
**Constraints**: 2 MB flash must hold 2 × 3–4 min flights' engaged spans between clears; pipeline
worst case (43 ms measured at 10 Hz era) must be characterized against the 50 ms tick; BLE-only
field download; no in-loop data erase (existing); console = events + ~2 Hz heartbeat only
**Scale/Scope**: one flight candidate (t5 elite; FR-005 retrain path if the operator's latency
review says amend); ~4 firmware surfaces (generated NN, gather/arena, logger, msplink), 1 codegen
tool change, 1 ground decoder + comparison report

## Constitution Check

*GATE: evaluated pre-Phase-0 and re-checked post-design — PASS (no violations to justify).*

- **I Testing-First**: new logic gets failing-first tests — nn2cpp unrolled-recurrent equivalence
  (desktop, bit-comparable same-order accumulation), engage-centered arena vertical rule (unit test
  on the NED formula incl. the −25 m clamp), log format encode→decode round-trip (field-for-field
  with quantization tolerance), decoder rejection of wrong version byte. Firmware-only observational
  steps (bench span) are validation, not unit-testable — recorded per FR-002 as log review.
- **II Build Stability**: xiao `pio run -e xiaoblesense_arduinocore_mbed` must compile at every
  commit that touches xiao; desktop `rebuild.sh` green for tool/reader changes.
- **III No Compatibility Shims**: the text per-tick log path is REPLACED, not kept alongside;
  old flights stay readable via the existing historical scripts (immutable per project practice),
  new decoder is a new tool. `MSP_NN_EVAL_DIVISOR=1` is set, not defaulted around.
- **IV Unified Build**: nn2cpp/test changes ride the existing targets; any CMakeLists change ⇒
  operator-driven clean `rebuild-perf.sh` (also required before any FR-005 retrain — determinism).
- **V Versioned Persistence Artifacts**: the new binary flight log **bridges xiao → ground tooling
  and feeds the SC-005 acceptance analysis**, so it gets an explicit format-version byte in the
  file header despite the "ephemeral log" exemption for the old text form; reader fails loud on
  unknown version. (The version is in the file header once, not per record.)
- **VI Type-Domain Discipline**: xiao-side packed structs are wire-format (`// raw-ok: hardware
  byte layout`); desktop reader converts to `gp_scalar` domain at the boundary; grep audit on
  touched paths at implement close.
- **VII No Silent Fallback Defaults**: the engage-scoped arena state has no in-class default — it
  is constructed at span activation from `z_engage` + template geometry or the build fails; the
  decoder takes scale tables from the file header, not compiled-in defaults.
- **VIII Artifact Lifecycle**: weight source = already-pinned t5 elite (`retain=keep`); if FR-005
  retrains, the new elite is pinned and recorded in outcome.md before flash.
- **IX Detached Training Launch**: any FR-005 M1 retrain launches via `scripts/train.sh` (operator),
  after the pre-run build gate.
- **X Single Ordered Backlog**: deferred items discovered here (jitter dither already there;
  delta+varint at 50 Hz; BLE reliability) live in `specs/BACKLOG.md`.

## Project Structure

### Documentation (this feature)

```text
specs/039-xiao-20hz-flight/
├── plan.md              # This file
├── research.md          # Phase 0 — latency ground truth, log/compression audit, regen/transport
├── data-model.md        # Phase 1 — flight-log records, engage header, latency memo, comparison report
├── quickstart.md        # Phase 1 — regen → bench → fly → decode → compare walkthrough
├── contracts/
│   ├── flight-log-format.md   # versioned int16-packed binary log (writer/reader pair)
│   ├── latency-memo.md        # FR-004/005/006 decision-memo contract
│   └── bench-validation.md    # FR-002 observational bench span + FR-011 cadence soak
└── tasks.md             # Phase 2 (/speckit.tasks — NOT created by /speckit.plan)
```

### Source Code (repository root)

```text
tools/
└── nn2cpp.cc                  # unrolled-recurrent emission; arena template (not placement) baking

xiao/
├── src/generated/nn_program_generated.cpp   # REGENERATED: 37-in/2051-w unrolled
├── src/msplink.cpp            # 20 Hz divisor=1; engage-time arena re-center; binary log calls;
│                              # console split (events + 2 Hz heartbeat); pipeline timing capture
├── src/flash_logger.cpp       # binary record write path (encoder), engage header
├── src/bluetooth.cpp          # unchanged surface (DL/LIST/ERASE); downloads new format
└── include/main.h             # MSP_NN_EVAL_DIVISOR 2 → 1; (optional) baud constant

src/nn/evaluator.cc, include/autoc/eval/arena.h   # shared gather/arena (already 038-ready;
                                                  # touched only if the mutable-arena seam needs it)
crrcsim/src/mod_inputdev/inputdev_autoc/*          # only if the operator's latency review amends
                                                   # COMPUTE_LATENCY (FR-005)

tests/
├── nn2cpp_unroll_tests.cc     # unrolled ≡ table-driven on same weights (recurrent covered)
├── arena_recenter_tests.cc    # engage-centered vertical rule + −25 m clamp (NED)
└── flightlog_roundtrip_tests.cc  # encode→decode field-for-field; version-byte loud-fail

src/analytics/ (new script)    # flight-log decoder + per-axis sim-vs-real comparison report
```

**Structure Decision**: existing multi-component layout (autoc / crrcsim / xiao); no new top-level
components. The log encoder lives xiao-side with a desktop decoder + tests sharing the contract doc
as the single format definition (single authoritative writer/reader pair per FR-009 — the shared
artifact is the *contract*, since the xiao build cannot link desktop code).

## Phase 0 — research

Complete: [research.md](research.md). All Technical Context unknowns resolved; decisions D1–D7.
Headlines: sim 30 ms ≈ real ~29 ms mean today (retrain question re-opens with the 20 Hz bench, D1);
local IMU stays deferred (D2); log v1 = versioned int16-packed binary, text blocks 20 Hz (D3);
transport fits at 115200 (D7); nn2cpp needs unrolled-recurrent (D6); firmware needs mutable
engage-centered arena, K = 47.5 m (D5).

## Phase 1 — design & contracts

Complete: [data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md).

- **data-model.md**: flight-log file layout (file header w/ version + scale tables, engage header,
  per-tick record — all 37 inputs named per the 038 layout), latency-memo fields, comparison-report
  fields, flight-candidate identity.
- **contracts/flight-log-format.md**: byte-exact v1 format + quantization scales + loud-fail rules.
- **contracts/latency-memo.md**: what FR-004/005/006 must deliver for the operator review.
- **contracts/bench-validation.md**: FR-002 observational span checklist + FR-011 cadence soak
  (several consecutive 3–4 min spans), DWT eval-cost measurement.

## Phase 2 — NOT here

`/speckit.tasks` derives the dependency-ordered task list. Expected story order (from spec
priorities + research): US1 regen+unroll (with desktop tests first) → US3 log format (round-trip
tests first) + FR-014 console split → US4 20 Hz enable + bench soak → US2 latency bench + memo +
operator review (gates candidate) → [FR-005 retrain if amended] → US5 flight + SC-005 report.
