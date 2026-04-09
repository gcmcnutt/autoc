# Quickstart — 023 Minisim Smoke Test

**Feature**: 023 | **Phase**: 2 (Minisim Smoke Test) per Implementation Order

This document is a runnable recipe for the Phase 2 minisim smoke test — the
gate that proves "the new code path can train *anything*" before promoting
to CRRCSim for real training. Not a tracking quality gate; a plumbing gate.

## Prerequisites

Before running the smoke test, ALL of the following must be complete:

1. **Phase 0a (type-safe NN refactor) committed and merged** on the 023 branch
   - `NNInputs` struct exists at 33 fields in `include/autoc/nn/nn_inputs.h`
   - `static_assert(sizeof(NNInputs) == 33 * sizeof(float))` fires if broken
   - NN topology is `{33, 32, 16, 3}`, 1667 weights
   - All existing unit tests pass after the refactor

2. **Phase 0b (train/eval dedup) committed and merged** on the 023 branch
   - `buildEvalData()` helper exists and both training + eval paths route through it
   - `tests/eval_determinism_tests.cc` passes
   - Running eval on a known weight file reproduces training fitness within FP rounding

3. **Change 1 (history reset) implemented**
   - `AircraftState::resetHistory()` exists and is called at engage in CRRCSim, minisim, xiao
   - `tests/engage_reset_tests.cc` passes

4. **Change 1b (engage delay window) implemented**
   - `EngageDelayMs` config knob parses correctly
   - CRRCSim `inputdev_autoc.cpp` gates stick application per the contract
   - Minisim applies the same delay window logic
   - `tests/engage_delay_tests.cc` passes

5. **Change 6 (direction cosines) implemented**
   - `computeTargetDir()` helper exists and is used by all producers
   - `target_x/y/z` fields populated correctly in sim + xiao
   - Unit vector invariant verified by `tests/nn_inputs_tests.cc`

6. **Change 8 (authority limit) config knob exists**
   - `NNAuthorityLimit` parses from config
   - Default `1.0` (no limit) — smoke test uses default
   - Applied identically in sim bridge and xiao (though xiao isn't in the
     smoke test loop, the code path exists)

7. **Logging coverage verified**
   - `data.dat` emits all 33 NNInputs columns + 3 output columns + meta columns
   - Column headers derived from struct field names
   - `sim_response.py` parses the new column layout without error

## The smoke test

### Step 1 — Build

```bash
cd /home/gmcnutt/autoc
bash scripts/rebuild.sh
```

Expected: clean build, all unit tests pass. If any fail, stop and fix.

### Step 2 — Prepare smoke test config

Create `autoc-smoke.ini` as a copy of `autoc.ini` with variations disabled
and a single path:

```ini
# autoc-smoke.ini — 023 minisim smoke test configuration

# Use minisim, NOT crrcsim
MinisimProgram                  = ./build/minisim
EvalThreads                     = 4         # small, diagnostic

# Population / generations kept small for fast turnaround
PopulationSize                  = 500
NumberOfGenerations             = 50

# Single path, no variations — we're testing plumbing, not tracking quality
SimNumPathsPerGeneration        = 1
PathGeneratorMethod             = aeroStandard  # use 022's deterministic paths
WindScenarios                   = 1

# All variation master switches OFF
EnableEntryVariations           = 0
EnableWindVariations            = 0
EnableRabbitSpeedVariations     = 0
VariationRampStep               = 0

# Rabbit at constant speed
RabbitSpeedNominal              = 10.0
RabbitSpeedSigma                = 0.0

# New 023 fields
EngageDelayMs                   = 750
NNAuthorityLimit                = 1.0

# Fitness unchanged from 022 (conical surface V4)
FitDistScaleBehind              = 7.0
FitDistScaleAhead               = 2.0
FitConeAngleDeg                 = 45.0
FitStreakThreshold              = 0.5
FitStreakRampSec                = 5.0
FitStreakMultiplierMax          = 5.0

# Output: smoke-specific directory
# (or whatever the current output-dir convention is)
```

### Step 3 — Run minisim training

```bash
cd /home/gmcnutt/autoc
nohup stdbuf -oL -eL build/autoc --config autoc-smoke.ini \
    >logs/autoc-023-smoke.log 2>&1 &
```

Expected runtime: a few minutes for 50 generations at population 500 on minisim.
Check that training is actually running:

```bash
tail -f logs/autoc-023-smoke.log
```

Look for per-generation output. Fitness should be decreasing on average (not
necessarily monotonically — evolution is noisy).

### Step 4 — Pass criteria (all must be true)

| # | Criterion | How to verify |
|---|---|---|
| 1 | Build compiled, all unit tests pass | `echo $?` after `rebuild.sh` returns 0 |
| 2 | Minisim runs 50+ gens without NaN | `grep -i nan logs/autoc-023-smoke.log` returns nothing |
| 3 | Minisim runs 50+ gens without hang | Process completes in reasonable time (< 10 min) |
| 4 | Minisim runs 50+ gens without segfault | Exit status 0, no "core dumped" |
| 5 | `data.dat` emits all 33+3+meta columns | `head -1 eval-results/*/data.dat \| awk -F' ' '{print NF}'` matches expected count |
| 6 | Column headers match NNInputs field names | `head -1 eval-results/*/data.dat` contains `target_x_0 target_x_1 ... gyro_r` |
| 7 | Fitness decreases on average across gens | `python specs/019-improved-crrcsim/sim_response.py --plot-fitness logs/autoc-023-smoke.log` shows downward trend |
| 8 | Eval determinism — eval on gen-N weights reproduces training fitness | `build/autoc --eval eval-results/*/gen*.dmp` fitness matches training log within FP rounding |
| 9 | No engage transient visible in data.dat | First few `closing_rate` values are 0 (flat history), not a spike |
| 10 | Engage delay window suppresses outputs | First ~7 ticks of `data.dat` show `nn_output_*` nonzero but `stick_*` zero (if logged) |
| 11 | Unit vector invariant holds | `python -c "import pandas; df = pandas.read_csv('...', sep=' '); assert ((df.target_x_3**2 + df.target_y_3**2 + df.target_z_3**2 - 1).abs() < 0.01).all()"` |
| 12 | `sim_response.py` parses without error | `python specs/019-improved-crrcsim/sim_response.py --parse eval-results/*/data.dat` exits 0 |

### Step 5 — If it passes

Promote to CRRCSim (Phase 3 of Implementation Order). Same config file,
swap `MinisimProgram = ./build/minisim` for the CRRCSim launcher, run the
CRRCSim Milestone A zero-variation baseline.

If any criterion fails → STOP. Fix the failure before promoting. Promoting
broken plumbing to CRRCSim just burns training walltime on a known-bad
baseline.

## What this smoke test does NOT validate

- **Tracking quality** — we don't care if the NN actually tracks well, just
  that it's learning *something* and not producing garbage
- **Real dynamics behavior** — minisim has no wind, no aerodynamics model, no
  craft parameter variations; that's CRRCSim's job
- **Variation handling** — all variation enables are OFF; Milestone B in
  CRRCSim layers them in one at a time
- **INAV filter interaction** — not modeled in minisim; will be exercised
  once on CRRCSim
- **Xiao inference correctness** — xiao-side `nn2cpp` codegen should be
  tested independently via `tests/contract_evaluator_tests.cc` bitwise check
  against a saved weight file

## Troubleshooting

### Build fails at `static_assert`

The `NNInputs` struct has padding, wrong count, or wrong alignment. Check
`include/autoc/nn/nn_inputs.h` field declarations. `sizeof(NNInputs)` must
equal exactly `33 * sizeof(float)` = 132 bytes.

### Build fails at "undefined reference to computeTargetDir"

`src/nn/nn_input_computation.cc` wasn't added to `CMakeLists.txt`. Add it
to the autoc_common target sources.

### Minisim exits immediately

Most likely: config file parse error. Check `logs/autoc-023-smoke.log` for
`FATAL ERROR: Cannot parse configuration file`. Verify `autoc-smoke.ini`
syntax.

### Eval determinism check fails

`buildEvalData()` is not producing identical `EvalData` between training
and eval paths. Check `tests/eval_determinism_tests.cc` for the specific
field mismatch. Probable cause: a global (like `gRabbitSpeedConfig`) that
one caller uses and the other doesn't.

### Data.dat column count wrong

Probable cause: `emitNNInputsColumns()` helper hand-lists columns instead
of deriving from the struct. Fix: derive column headers from struct field
reflection (or a manually-maintained list that is asserted against
`sizeof(NNInputs) / sizeof(float)` at compile time).

### Unit vector invariant fails

`computeTargetDir()` is not normalizing correctly. Check the fallback case
(`dist < DIR_NUMERICAL_FLOOR`): `rabbit_vel_dir_body` must itself already
be a unit vector — if the caller passes a non-normalized path tangent, the
fallback output will not be unit-norm.

### NaN in fitness

Probable cause: divide-by-zero in fitness computation at singular geometry.
The Phase 0a.3 deliberate-break test should have caught struct layout
bugs; this is a different class. Add an assertion in
`computeScenarioScores()` that flags non-finite values with the scenario
index and step number, rerun, inspect.

## Expected walltime

- Build (rebuild.sh): ~60 seconds
- Smoke test (50 gens × 500 pop × 1 scenario on minisim): 1-3 minutes
- Total: under 5 minutes

If the smoke test is taking >10 minutes, something is wrong. Stop and
diagnose.
