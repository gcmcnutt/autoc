# Contract: xiao firmware NN sync points

**Feature**: 038 | **Status**: design | **Consumers**: `tools/nn2cpp.cc`, `xiao/src/generated/`,
`xiao/include/nn_program.h`.

038's NN-architecture changes must keep the embedded target (xiao) coherent with the desktop NN, even though
the **tracker is not yet deployed to xiao** (pathgen-only firmware today; tracker port is deferred BACKLOG).
This contract records the sync points so the codegen stays correct and the firmware contract is updated per
FR-033.

## Codegen is topology-agnostic, but compile-time constants are not

- `nn2cpp.cc` reads `topology`/`recurrent`/weights from NN01 and emits correct array sizes + loops — so US2
  (recurrence) and US3 (output count) propagate **automatically** on regen.
- **Manual sync required** when:
  - **Cadence / history change (US1)**: `HIST_PAST[]` is derived from `SIM_TIME_STEP_MSEC` at compile time;
    the firmware has no runtime history table → regenerate firmware on any lag/cadence change.
  - **Input/output count change (US1 slot-count, US3 aux head)**: the generated `NNInputs`/output buffers
    resize from the NN01 topology, but the firmware build + `nn_program.h` signature must be rebuilt and
    re-flashed; the first-3-outputs-for-control convention MUST hold (aux outputs sunk/ignored on device).

## Sync checklist on any 038 NN-contract change

1. Regenerate `xiao/src/generated/nn_program_generated.cpp` via `nn2cpp` from a 038-topology genome.
2. Rebuild xiao (`cd xiao && pio run -e xiaoblesense_arduinocore_mbed`) — MUST compile (Constitution II).
3. Confirm `generatedNNProgram()` signature in `nn_program.h` still matches the caller (control returns
   pitch/roll/throttle; aux outputs not actuated).
4. Record the contract update in the outcome doc (FR-033: format-breaking NN-input change → xiao contract
   updated, no cereal version bump, genomes retrain from scratch).

## Note

The M2 tracker firmware port itself (gather_tracker_inputs under `#ifndef ARDUINO`, mode select, beacon
source) remains deferred (BACKLOG "Xiao tracker-mode prototype"). 038 keeps the desktop/codegen contract
coherent so that port, when it lands, inherits the 038 architecture cleanly.
