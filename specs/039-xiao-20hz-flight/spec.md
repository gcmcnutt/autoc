# Feature Specification: 039 Xiao 20 Hz Flight — embedded control-loop catch-up

**Feature Branch**: TBD (`039-xiao-20hz-flight` when unparked).
**Created**: 2026-06-19.
**Status**: DRAFT — re-homed from 037 (operator 2026-06-19: "push off the Xiao catch-up to 039").
**Input**: 037 closed the M1 *controller* question (t10 GO at 20 Hz + 0.8 s window + honest servo). The
remaining 037 work was always the **flight-enablement / firmware** track — get the xiao to actually run
the 20 Hz control loop in real flight. That is engineering on a different surface (embedded firmware,
INAV link, real-time scheduling) than the controller-cadence feature, so it earns its own spec.

## Overview

Make the xiao run the evolved NN control loop at **20 Hz on real hardware** for an M1 flight, then push
toward **50 Hz**. This is the bridge between "the sim says 20 Hz is GO" (037) and "we flew it." It runs
**parallel to 038** (M2 robustness) and **031** (camera hardware) — an M1 flight does not need the camera.

## Scope — carried verbatim from 037 Phases 6 & 7 (US2 + US3)

The dependency-ordered task breakdown already exists as 037 Phase 6 (US2) / Phase 7 (US3); it should be
lifted into `specs/039-xiao-20hz-flight/tasks.md` at `/speckit.tasks` time. Headline content:

### US2 — Phase B: embedded ~20 Hz (Priority: P1 for 039)
Run a fresh ~20 Hz control tick on real hardware (local-IMU fast loop + INAV slow sync), validated
against INAV, producing the flown controller. Was 037 T031–T040:
- **Local IMU** — `xiao/src/imu_local.{cpp,h}`: LSM6DS3 TWIM-DMA read + completion ISR; complementary /
  loosely-coupled fusion (local gyro propagation + accel leveling + INAV-aided heading); resolves the
  "do we trust the onboard IMU at control rate" question (037's local-IMU thread).
- **021 convention cross-check** at the `autoc::imu::inavQuatToAerospaceEB` boundary (host-unit + logged).
- **Link baud raise** on both ends (`xiao/src/msplink.cpp` Serial1.begin, INAV side) for the 20 Hz budget.
- **INAV-side** command override + state serve at 20 Hz; keep CH6 = manual semantics.
- **Three rate tiers** (fast local IMU @ control rate / intermediate INAV sync / slow telemetry).
- **`MSP_NN_EVAL_DIVISOR=1`** (20 Hz NN) once the tick budget fits.
- **Packed dual-stream log** (P5; `contracts/packed-log-format.md`) — local-IMU logged beside INAV.
- **Validation**: `activate → capture → confirm` (local-IMU logged beside INAV) → promote → flight.

### US3 — 50 Hz (Priority: P2 for 039, gated on US2 at 20 Hz)
Reach a 20 ms tick with a bounded, tail-safe real-time loop. Was 037 T041–T045:
- **R4 cycle-count harness** (`xiao/` bench target) to measure the eval budget.
- **Fully unroll + fast-tanh** the NN forward pass (`xiao/src/generated/nn_program_generated.cpp`).
- **Real-time slot scheduling** (collect→process→output→log) with control-critical slots protected.
- **Tail-bounding**: async QSPI flash (no in-loop erase), NVIC priorities so the control tick never
  starves.
- **Validate** the 50 Hz sim arm at FDM ≥500 Hz (≥10× oversample) clears the gate; then on-target.

## Dependencies & Sequencing
- **Upstream**: 037 t10 (the 20 Hz controller to flash) — done.
- **Parallel, not blocking**: 038 (M2 robustness), 031 (camera). An M1 flight needs neither.
- **Gating within 039**: US3 (50 Hz) gated on US2 (20 Hz) holding on-target.

## Out of Scope
- The M2 / tracker controller work (038).
- Camera/beacon hardware + optics (031).
- Any controller-cadence / fitness changes (037 closed that; 038 owns M2 fitness).

> Detailed acceptance, contracts, and the T0xx task list live in 037 Phases 6–7 today; migrate them here
> at plan/tasks time. This stub exists so the xiao track has a stable home per the 2026-06-19 descope.
