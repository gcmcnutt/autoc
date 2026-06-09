# Quickstart: 033 phase-1 M1 bake protocol

**Audience**: operator running phase-1 (M1) bakes for 033 — multiplicative smoothness + master-seed PRNG architecture.
**Branch**: `033-m1-smooth-plus-variations` (autoc) + matching `033-m1-smooth-plus-variations` (crrcsim submodule).
**Mode**: M1 pathgen (minisim path → fast iter, then crrcsim path → bake target).

## Phase-1 success criteria (from Clarifications Q1 + spec §2.B "Phase-1 success criteria")

Closeout gates (all required):

1. **Continued learning toward higher fitness**: gen-vs-fitness curve has positive slope through bake end (no plateau-collapse).
2. **Streak% ≥ 2/3 of 029 pastonly3 baseline**: avgMaxStreak ≥ 30 in the variation-ramp plateau (baseline avgMaxStreak ≈ 42-45 per research R5; 30 = floor).
3. **Best-fitness in same range as 029 pastonly3**: per-gen best-fitness not collapsed to << 029-pastonly3 final-plateau values.
4. **Materially-better per-axis aggressiveness**: dCtrl/⟨|out|⟩ ratios per axis show clear reduction vs 029 pastonly3 final-elite (the smoothness penalty's intended effect).
5. **Mezzanine real-flight test**: phase-1 elite flown on hardware → operator visual + telemetry assessment. THIS IS THE QUALIFIER to phase 2 (M2 inheritance + kamikaze).

## Pre-bake setup

### 1. Verify branch + submodule pointer

```bash
cd ~/autoc
git status                       # on 033-m1-smooth-plus-variations
git submodule status crrcsim     # crrcsim pointer matches 033-m1-smooth-plus-variations tip
```

### 2. Build (full clean baseline)

```bash
cd ~/autoc
./rebuild-perf.sh                # FP-deterministic perf build (per reference_perf_build_reproducibility)
```

`rebuild-perf.sh` is the bake-grade build; do NOT use `rebuild.sh` (debug) for the bake.

### 3. Confirm new tests pass

```bash
cd ~/autoc/build
ctest --output-on-failure -R 'derived_features|fitness_computer|scenario_prng|tracker_dmp_roundtrip|contract_config'
```

All listed tests must pass before bake. New 033 tests:
- `derived_features_tests` — `compute_smoothness_factor` pure math
- `fitness_computer_tests` — `applyStreak(stepPoints, smoothness_factor)` integration
- `scenario_prng_tests` — master → scenario → class sub-PRNG chain determinism
- `tracker_dmp_roundtrip_tests` — `scenarioSeedList[K]` roundtrip
- `contract_config_tests` — `[Smoothness]` section parsing + range checks

### 4. Verify ini values match phase-1 plan

```bash
cd ~/autoc
grep -A3 '\[Smoothness\]' autoc.ini
```

Expected:
```ini
[Smoothness]
SmoothnessPenaltyFloor=0.5
SmoothnessMotionMode=pythagorean
```

If you want a back-compat reference bake (PRNG architecture without smoothness penalty), set `SmoothnessPenaltyFloor=1.0` instead and run as a parallel-baseline reference.

## Bake — M1 minisim (fast iter, ~30 min for first signal)

```bash
cd ~/autoc
nohup stdbuf -oL -eL ./build/autoc -i autoc.ini > logs/autoc-033-phase1-minisim.log 2>&1 &
disown
```

Use the `nohup ... & disown` pattern so the process reparents to systemd --user and survives VSCode reload (per prior bake-killed-by-reload incident).

Monitor:

```bash
tail -f ~/autoc/logs/autoc-033-phase1-minisim.log | grep -E '^#GenDiag|^Effective master seed'
```

The first line in the log MUST be `Effective master seed: <uint64>`. Copy this value for the eval-mode bit-identical regression check at closeout.

### Phase-1 minisim early-signal checks (gen 50-100)

- `#GenDiag` lines emit per-gen telemetry; capture best-fitness and avgMaxStreak fields.
- If avgMaxStreak < 20 by gen 100 → smoothness penalty is too strong (suggest re-checking floor; or NN topology insufficient for the new constraint).
- If best-fitness collapses to ~0 → PRNG init bug (scenario seeds all converging, or wind seed feeding back into nn-evolution stream). Stop bake; debug.

## Bake — M1 crrcsim (primary closeout target, ~16 hr)

```bash
cd ~/autoc
nohup stdbuf -oL -eL ./build/autoc -i autoc.ini > logs/autoc-033-phase1-crrcsim.log 2>&1 &
disown
```

Phase-1 closeout dataset = the crrcsim bake's gen-trajectory through end-of-bake (typical: gen 540 or wherever the operator chooses to stop based on closeout-criteria converged).

### Bake monitoring + intermediate analysis

Periodically (every ~50 gens) run intercept-analysis on the most recent dmp:

```bash
cd ~/autoc
python3 specs/032-tracker-nn-enhancements/intercept_analysis.py \
  --dmp /path/to/recent.dmp \
  --out specs/033-m1-smooth-plus-variations/intercept_gen<N>.png
```

(`intercept_analysis.py` from 032 is reusable; will be branched to a 033-specific variant if 033 reveals new failure modes during phase-1 mid-bake review.)

### Bake closeout (operator-triggered)

When the operator decides to close out the bake (closeout-criteria met OR plateau-with-streak-floor-met):

1. Capture the final dmp tag (`gen<N>.dmp`).
2. Run `specs/033-m1-smooth-plus-variations/intercept_analysis.py` on the final dmp.
3. Run per-axis aggressiveness analysis (extension of 032's per-axis tools) — confirm dCtrl/⟨|out|⟩ reduction vs 029 pastonly3.
4. Cross-check `avgMaxStreak` against 2/3 baseline floor (30).
5. Write closeout snapshot to `specs/033-m1-smooth-plus-variations/outcome.md` (mirror 032's outcome.md layout).

## Eval-mode bit-identical regression check

After closeout, the D4 determinism contract must be verified:

```bash
# 1. Pick a representative scenario K from the closeout dmp.
# 2. Run autoc in eval mode with:
#    - master seed = value printed at training start (logs/autoc-033-phase1-crrcsim.log first line)
#    - the closeout elite NN
#    - the closeout scenarioSeedList[K] specifically
# 3. Capture per-tick aircraftState for scenario K.
# 4. Assert bitwise equality with the training trace for scenario K.

./build/autoc -i autoc-eval.ini  # appropriate eval-mode ini configured per above
```

If this fails → PRNG decoupling has a leak; debug before closing out.

## Mezzanine real-flight test (phase-1 → phase-2 gate)

After bake closeout + bit-identical regression check pass:

1. Flash phase-1 elite NN to xiao via the existing pipeline.
2. Fly with operator (mezzanine — short controlled flight, not the M2 tracker pattern).
3. Operator assesses: control feel, smoothness vs prior tags, no obvious regressions.
4. If GO → phase 2 (M2 inheritance + kamikaze penalty experiment per spec §2.C + §2.D).
5. If NO-GO → revise phase-1 (knob tuning: floor 0.3 / 0.7 sweep, mode sum/max comparison, etc.); rebake.

## Failure modes + debug hooks

- **Bake silent + log empty**: check `autoc.ini` was found (run with absolute path if relative resolution fails); check crrcsim worker process group spawned.
- **"Effective master seed" line missing**: 033 init chain not invoked; check `src/autoc.cc` startup sequence wasn't reverted.
- **Cereal length mismatch loud-fail on old dmp**: expected (per `dmp_schema_extension.md` failure-modes section); re-bake or use pre-033 binary for that specific dmp.
- **fitness drifts down vs 029 pastonly3 baseline plateau**: smoothness penalty + new PRNG ordering may have created a regression; check the back-compat baseline (`SmoothnessPenaltyFloor=1.0` bake) to isolate which knob caused the drift.
- **avgMaxStreak collapse**: smoothness penalty is making the controller too sluggish to compound streaks; consider floor=0.7 (less aggressive penalty) before claiming failure.

## Branch + PR workflow

Per `feedback_submodule_merge_order`: crrcsim submodule PR merged FIRST, autoc parent PR merged SECOND. Both PRs share the branch name `033-m1-smooth-plus-variations`.

## Citations

- [spec.md](./spec.md) §3 (success criteria) + Clarifications Q1 (M1 metric definition)
- [plan.md](./plan.md) (Constitution Check + Build Stability intentional-break note)
- [research.md](./research.md) R5 (avgMaxStreak baseline from 029 pastonly3)
- [contracts/scenario_prng_chain.md](./contracts/scenario_prng_chain.md) (D4 eval-mode regression contract)
- [contracts/ini_schema.md](./contracts/ini_schema.md) (`[Smoothness]` section spec)
- [reference_perf_build_reproducibility](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_perf_build_reproducibility.md)
- [reference_autoc_launch_command](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_autoc_launch_command.md)
- [feedback_operator_runs_regression_gate](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_operator_runs_regression_gate.md)
- [feedback_submodule_merge_order](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_submodule_merge_order.md)
