# Contract: NN01 + dmp format (the one P0-D break)

**Feature**: 038 | **Status**: design | **Consumers**: autoc, crrcsim, renderer, dmp_dump, nnextractor,
nn2cpp→xiao, analytics, eval-mode replay gate.

038's architecture changes are format-breaking. This contract defines the **single** dmp/NN01 break (P0-D)
and the read-side guarantees. Principle: greenfield, **no cereal version bump**, **fail-loud on read**
(Constitution V practice); old dmps/genomes are orphaned and runs retrain from scratch.

## NN01 genome format

- Wire layout unchanged in *structure* (magic `NN01`, format version, num_layers, topology[], recurrent[],
  num_weights, weights[], fitness, generation, sigma, variation_scale, source) — it is topology-agnostic.
- **Breaking via content**: a genome whose `topology`/`recurrent`/weight-count does not match the compiled
  `TRACKER_NN_TOPOLOGY`/`NN_RECURRENT` (US1 slot-count, US2 slow channel, US3 output 3→7) MUST fail to load
  with a clear topology-mismatch error (`aircraft_state.h:582` style + `serialization.cc` weight-count
  validation). No silent default-init.
- **Output convention (US3)**: `outputs[0..2]` = pitch/roll/throttle (actuated); `outputs[3..N]` = predicted
  optical state (scored, not actuated). Codegen + xiao sink the aux outputs.

## dmp `EvalResults` / `AircraftState` (cereal)

- **No `CEREAL_CLASS_VERSION` bump.** A reader hitting a pre-038 dmp (missing the self-describing config
  block, or wrong NN topology) MUST **fail loud** (identify what mismatched), never silently truncate or
  default — this is the V safety net that substitutes for a version bump.
- **New embedded config block** in `EvalResults` (P0-D-2): fitness cone params, cadence
  (`SIM_TIME_STEP_MSEC`/`kCadenceTickScale`), crash-penalty knobs, US4 visibility knobs. Makes a dmp
  standalone-replayable.
- **`AircraftState.simTimeMsec`** (P0-D-1): exact 50 ms gaps (step-count derived). The M2 source-spacing
  check is **strict single-gap** post-fix (no more average-gap tolerance).
- **`AircraftState.wind_velocity`** (P0-D-3): populated from crrcsim FDM at record time (was zero).
- **New `ScenarioScore` axes** (US3 `prediction_score`, US4 `visibility_score`) if persisted for analytics:
  part of this break; honest-recording (capture the inputs+outputs the selection actually used).

## Read-side contract (P0-B)

- `renderer.cc` + `dmp_dump.cc` MUST prefer the dmp-recorded config block over
  `ConfigManager::getConfig()` for fitness/cadence params, so a drifted live `.ini` cannot misrender a
  pinned run. Only remaining `.ini` dependence: S3 bucket/profile (bootstrap).

## Determinism gate

- The eval-vs-training **bitwise `ScenarioScore` gate** (`rebuild-perf.sh`, single-threaded FP-deterministic)
  MUST pass with all 038 changes. Whole-dmp byte equality is NOT the gate (provenance timestamps + `.zst`
  framing are non-deterministic) — the per-scenario `ScenarioScore` vector is.
