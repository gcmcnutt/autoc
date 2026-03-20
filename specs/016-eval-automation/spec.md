# Feature 016: Eval Automation & Config Overlays

## Problem

After a training run, characterizing the learned controller requires:
1. Manually editing `autoc-eval.ini` for each test scenario
2. Running `build/autoc -i autoc-eval.ini` one scenario at a time
3. `data.dat` and `data.stc` are overwritten each run — previous results lost
4. No structured way to compare results across runs or track regression
5. Config duplication: eval INI must repeat all base settings, only differing in a few keys
6. No automated pass/fail criteria or summary reporting

## Proposed Solution

Three layers, each independently useful:

### Layer 1: Config overlay (`-i` stacking)

Allow multiple `-i` flags where later files override earlier values:

```bash
./build/autoc -i autoc.ini -i eval-base.ini -i eval-random-12x12.ini
```

**Implementation**: In `ConfigManager::initialize()`, accept a vector of filenames.
Parse each INI in order, merging into a single `AutocConfig`. Later values win.
inih's INIReader already works key-by-key — just call it N times on the same struct.

**Benefit**: Eval scenarios become tiny overlay files:
```ini
# eval-random-12x12.ini (10 lines instead of 80)
EvaluateMode = 1
PathGeneratorMethod = random
SimNumPathsPerGeneration = 12
WindScenarios = 12
RandomPathSeedB = 99999
```

### Layer 2: Data archival (no more volatile data.dat)

**Problem**: `data.dat` and `data.stc` are opened with truncate mode. Every run
destroys the previous.

**Options** (pick one):
- **a) Timestamped filenames**: `data-2026-03-20T08:39:46Z.dat` — simple, no code churn
- **b) Output directory flag**: `-o eval-results/run1/` — all outputs go there
- **c) S3 upload of data files**: extend existing S3 upload to include data.dat/data.stc

Recommend **(b)**: `-o <dir>` flag. Console log, data.dat, data.stc, config dump all
go into the specified directory. Default behavior unchanged (current dir).

**Implementation**: ~20 lines in main() — prepend output dir to file open paths.

### Layer 3: Eval suite script (done — `scripts/eval_suite.sh`)

Shell script that orchestrates the tiered eval schedule:
- Tier 1: Quick validation (294 flights, same geometry as training)
- Tier 2: Generalization (242 flights, novel paths)
- Tier 3: Stress (145 flights, 120% envelope + quiet throttle)

Generates per-test and overall summary with pass/fail criteria.
Archives all artifacts (configs, logs, data files) under timestamped directory.

Currently works by generating standalone INI files per scenario.
With Layer 1 (config overlay), would simplify to tiny overlay fragments.
With Layer 2 (-o flag), would not need to manually mv data files.

## Migration Path

1. **Now**: `scripts/eval_suite.sh` works with current single-`-i` by generating
   complete INI files. Functional but verbose.

2. **Layer 1** (config overlay): Implement multi-`-i` in ConfigManager. Update
   eval_suite.sh to use overlays. ~2 hours work.

3. **Layer 2** (output dir): Add `-o <dir>` flag. Update eval_suite.sh to use it.
   ~1 hour work.

4. **Layer 3** already done. Update to use Layers 1+2 when available.

## Non-goals (this feature)

- Web dashboard or visualization (renderer already handles S3 data)
- CI/CD integration (manual trigger is fine for now)
- Automated training restart after crash (separate concern)
- Per-scenario data.stc logging (T125 in Phase 8 polish)

## Dependencies

- Working `build/autoc` with eval mode
- crrcsim available via `scripts/crrcsim.sh`
- inih library (already in tree)
