# 043 — Phase 1 baseline (T001–T003)

Recorded 2026-08-25. Clean baselines and the two cheap facts that shape later work.

## T001 — clean debug build + test count (autoc + crrcsim)

- Command: `bash scripts/rebuild.sh` from repo root (clean `rm -rf build`, `cmake -DCMAKE_BUILD_TYPE=Debug`, `make`).
- Result: **build exit 0**.
- Tests: `ctest` in `build/` → **49 test suites, 100% passed, 0 failed** (14.94 s wall).
  - This matches the pre-implement measurement noted in `tasks.md` T001: the 2 suites T021a
    wires into the ALL target (`shared_input_block_tests`, `nn_input_scaling_tests`) are *registered*
    via `add_test`, so `ctest` runs all 49; the T021a gap is only that `make` (the ALL target) does
    not build/run those two. That is a Phase-4 fix (T021a), out of the P1–P3 window.
  - Individual gtest case total across suites ≈ 508 (per the tasks.md T001 note); the suite-level
    gate (49/49) is what was re-measured here.
- One suite skipped by design: `source_dmp_s3_integration_tests` (needs `AUTOC_S3_TESTS=1`).

## T002 — xiao host compile

- Command: `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed` from `xiao/`.
- Result: **exit 0** (host compile passes). No xiao source changed; this is the clean-tree baseline.

## T003 — as-run FDM substep (`Global::dt`)

⭐ **Finding: the as-run FDM substep is `Global::dt = 0.005 s = 200 Hz`, NOT ~333 Hz.**

Derivation (deterministic from config; `Global::dt` has exactly one source):
- `crrcsim/src/crrc_main.cpp:492` — `Global::dt = cfgfile->getDouble("simulation.flightModel.dt", 0.002777);`
- `crrcsim/src/crrc_main.cpp:499` — `Global::dt = (int)(Global::dt*1000 + 0.5)/1000.;` (round to integer ms).
- The autoc configs override the default: `crrcsim/autoc_config.xml:165` and
  `crrcsim/autoc_config-eval.xml:166` both set `<flightModel dt="0.005" />`.
- `0.005 s` is already integral in ms, so the rounding is a no-op ⇒ **`Global::dt = 0.005 s`**.
- `write_globals_into_config()` (`:510`) writes the same rounded value back — no further change.
- Logged empirically at scenario init by the existing cadence line
  (`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp:360`, `"[AUTOC] cadence: ... dt=..."`);
  the value is fully config-determined, so no new logging was added.

### 333 Hz-vs-2 kHz phase justification (research.md addendum)

- The real INAV ACRO rate loop runs at **2 kHz** (0.5 ms). The sim models it at the FDM substep,
  which is **200 Hz** (5 ms) as measured above — a 10× cadence gap, not the ~6× the "333 Hz" assumption
  implied.
- ⚠️ **Consequence for Phase 5**: `Cntrl_InavFwRate` fires once per FDM substep (200 Hz). The plan's
  Technical Context ("inner loop modelled at the FDM substep (~333 Hz)") is superseded here — the correct
  figure is **200 Hz**. The gyro `PT1` corner (25 Hz) and the loop's discrete-time constants must be
  evaluated against **5 ms**, not 3 ms. Filter phase at the frequency the loop controls (a few Hz) is
  nearly cadence-independent in this range, so the substep gap does not change the *character* of the
  modelled loop — but every discrete constant (integrator dt, PT1 alpha) uses **dt = 0.005 s**.
- This is recorded as the research.md substep addendum (T003 deliverable). See research.md.
