# 033 Phase-3 stall report

**Status:** 3 consecutive bakes (phase-1 floor=0.5, phase-2 floor=1.0, phase-3 floor=1.0 post-ConfigManager-bypass fix) all converge to the **same throttle-pegged spiral attractor**. Phase-3 stopped manually at gen 243 once the pattern was unambiguous.

## Symptom

Per-axis aggressiveness at gen 209 of phase-3 (294 scenarios sampled):

| axis     | `<|Δ|>` mean | `<|out|>` mean |
|----------|--------------|----------------|
| pitch    | 0.50         | 0.76           |
| roll     | 0.62         | 0.45           |
| **throttle** | **0.000 (σ=0.000)** | **1.000 (σ=0.000)** |

Throttle is **exactly +1.0 every tick across all 294 scenarios** — a dead-neuron saturation. Cross-checked via `.stc` energy_score column (`(out[2]-1.0)/2` per tick, summed):

| run        | gen 1 energy | latest energy |
|------------|--------------|---------------|
| phase-3    | -9,992       | **-0.85** at gen 243 (essentially 0 → throttle pegged) |
| 029-pastonly3 | -5,176    | -22,317 at gen 800 (throttle ≈ +0.75 mean, modulating) |

Throttle lock-in trajectory in phase-3:

| gen | mean throttle | within-scenario σ |
|-----|---------------|---------------------|
| 1   | -0.33 | 0.31 |
| 6   | +0.64 | 0.45 ← transition |
| 13  | +0.98 | 0.05 ← essentially pegged |
| 105 | +1.00 | 0.001 |
| 243 | +1.00 | 0.000 |

Pastonly3 also briefly touched throttle=1.0 around gen 33 but **broke out by gen 38** and ended with active throttle modulation. Phase-3 (and phase-1/phase-2) does not escape.

## What we ruled out

| candidate | verification | verdict |
|-----------|--------------|---------|
| Sensor pipeline corruption (032 added derived features) | unit-norm of `tgX0²+tgY0²+tgZ0² = 1.00000 ± 0.001` across 1000 ticks; quat norm = 1.0; history shift-register 0 errors across 2986 consecutive-tick pairs | clean |
| Pathgen layout / direction cosines regressed since 029 | `gather_pathgen_inputs` byte-identical to 029; topology 33,32,16r,3 unchanged | clean |
| SmoothnessPenaltyFloor=1.0 silently re-enabled | 49,999/49,999 sampled ticks have `smooth = 1.000000` exactly; ini → WorkerInit → worker wire path verified | true no-op |
| Config parser regression on Fit*/Streak* | every ini key parsed; diff to 029 in `config.cc` shows only additive changes | clean |
| `SimNumPathsPerGeneration = 5 vs 6` | both 029-pastonly3 and 033-phase3 use 6 (verified from log) | ruled out |
| Lexicase pool changed | `selection.cc` zero-diff since 029; same single-objective tracking pool | clean |
| NN output indexing off-by-one | `setPitch(out[0])`, `setRoll(out[1])`, `setThrottle(out[2])`; all worker/data.dat write sites match | clean |
| 64-bit seed overflow | full pipeline traced; entropy is 31 bits (Park-Miller modulus), collision rate ≈ 2×10⁻⁵ at M=294 — not biting | minor sharp edge, not the cause (T058 cleanup) |

## What's left as the suspect

**033's PRNG architecture rework.** The cascade structurally differs from 029:

| stream | 029 | 033 |
|--------|-----|-----|
| NN-evo `rng::seed` | `cfg.seed` directly | `MasterPRNG.next()` (1 PM step + fold) |
| Per-scenario wind/rabbit/entry seeds | `rng::randLong()` per scenario from NN-evo stream | `ClassPRNG(subseeds.{wind,rabbit,entry})` from `ScenarioRootPRNG(scenarioSeed[K])` |
| Entry-pose Gaussian | legacy LCG + Box-Muller | ClassPRNG (Park-Miller) + Box-Muller |
| CRRC_Random sim seed | `rng::randLong()` | `ClassPRNG(subseeds.wind).next()` |
| Rabbit profile | legacy LCG (unchanged); seeded by `rng::randLong()` | same algo; seeded by `subseeds.rabbit` |

Same statistical distributions, **different specific draws everywhere**. Operator hypothesis: pastonly3 may have been lucky-seeded into the modulating-throttle basin; 033's PRNG put 3 consecutive runs into the throttle-pegged basin. Confirmation requires running 029-era code with the same wall-clock seeds (or specifically `Seed = 1777749601` = pastonly3's recorded seed) to see whether 029's code reproduces the modulating outcome from the same input.

## Cleanup-deferred items captured during this debug

- **T056** ([tasks.md](tasks.md)) — `gScenarioVariations` is per-wind, not per-(path, wind); paths=1 × winds=N would run the same scenario N times
- **T057** — startup banner echoes only a subset of config keys; Fit*/Streak*/Smoothness not log-verifiable today
- **T058** — 64-bit seed widths carry only 31 bits of entropy after `fold_to_pm_state`; narrow to uint32 to match (Option A) so log-displayed seeds round-trip into `Seed = N` correctly

## Next experiment

Check out **commit `0b12070b763c89b7a444ee6fe03a03f5ec17218e`** (`Merge pull request #3 from gcmcnutt/029-no-future-arch`) with its matching crrcsim submodule pointer. Run with `Seed = 1777749601` (pastonly3's effective master seed). If the resulting run reproduces pastonly3's energy/streak trajectory → confirms 033 PRNG architecture is the regression. If the resulting run also stalls → pastonly3 was a lucky bake and the throttle-pegged attractor is genuinely the system's gradient-descent fixed point under the current fitness shape, in which case the real fix is multi-objective lexicase (uncomment energy_score + stability_score in `selection.cc:68-69`).

## Artifacts

- `033-phase3_evolution_progress.png` — fitness/streak curves vs 029-pastonly3 overlay
- `033-phase3_per_axis_aggressiveness.png` — degenerate throttle histograms confirming +1.0 lock
- `033-phase3_per_axis_time_series.png` — throttle locks at +1.0 by gen ~20 and never recovers
- `data.stc` (current) — 243 gens of phase-3 with `SmoothnessPenaltyFloor=1.0`, master seed 1779485775
- `data.dat` (current, 6.8 GB) — per-tick NN inputs/outputs for the elite per gen
