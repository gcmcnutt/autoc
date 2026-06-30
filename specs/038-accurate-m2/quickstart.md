# Quickstart: 038 Accurate M2 — operator runbook

**Plan**: [plan.md](plan.md) · **Research**: [research.md](research.md) · **Data model**:
[data-model.md](data-model.md)

How to land Phase 0, run the parallel architecture ablations, and judge them. The operator drives all
launches and the regression gate (Constitution IX; `feedback_operator_runs_regression_gate`).

## 0. Order of operations

```
Phase 0 (P0-A..G)  →  one P0-D dmp break + fresh M1 source re-bake  →  M2 baseline re-bake
   →  initial-wave ablations: US1 (history, M1-first) + US3 (predictor, M2-direct)  in parallel
   →  combine winners (≤2-way)  →  US4 visibility reward
   [US2 two-timescale recurrence DEFERRED to specs/BACKLOG.md — unpark only if US1+US3 don't move a ceiling]
```

## 1. Phase 0

- **P0-A (PRNG validation)**: `cd build && make run_autoc_tests && ./tests/run_autoc_tests
  --gtest_filter='*scenario_prng*:*eval_mode_replay*'` → write the verdict (clear / bug-found) into the
  outcome doc.
- **P0-F (streak revert)**: edit `FitStreakThreshold 0.3→0.5` in `autoc-tracker.ini:258`,
  `autoc-eval-tracker.ini:215`, `autoc-eval-tracker-visual.ini:219` (M1 inis already 0.5).
- **P0-E (housekeeping)**: remove `svTau` dead path; time-denominate rate-dependent reports; run the
  type-domain grep audit:
  ```bash
  grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -E '// raw-ok:'
  ```
  Each hit → annotate `// raw-ok: <reason>` or convert to `gp_scalar`/`gp_fitness`.
- **P0-D (one dmp break)** — crrcsim submodule first (pointer-bump-first):
  - simTimeMsec step-count stamp (`SimStateHandler.cpp:392`); revert source-spacing to strict gap.
  - wire `setWindVelocity()` from FDM (`inputdev_autoc.cpp` ~906).
  - self-describing `EvalResults` config block (`protocol.h`); flip renderer/dmp_dump to read it (P0-B).
  - **No cereal version bump**; readers fail loud on old dmps.
- **Build gate** (Constitution IX, before any larger run): clean build + tests.
  `bash scripts/rebuild-perf.sh` for the determinism-affecting P0-D; `cd build && make` for ordinary edits.
  **Operator runs the eval-vs-training bitwise `ScenarioScore` gate** before committing compute.

## 2. Re-bake the source (P0-G)

P0-D + any M1-arch change drives a fresh M1 source re-bake, then the M2 library re-bakes from it. Launch
detached (NEVER `run_in_background`):

```bash
scripts/train.sh autoc.ini logs/autoc-038-t1-m1-rebake.log          # fresh M1 source
# pin the winning M1 source in autoc-m1, record the S3 prefix in the outcome doc (Constitution VIII)
scripts/train.sh autoc-tracker.ini logs/autoc-038-t2-m2-baseline.log  # M2 baseline off the new source
```

## 3. Parallel architecture ablations

Bake each variant off the SAME post-P0-D baseline (same seed/source), named lexicographically
`autoc-038-tN-<lever>`:

- **US1 history (M1-FIRST GATE)** — bake the deeper layout on M1 first; only inherit to M2 if it moves an M1
  ceiling:
  ```bash
  scripts/train.sh autoc.ini logs/autoc-038-t3-m1-hist1p6s.log
  ```
- **US2 two-timescale recurrence (M2-direct allowed)**:
  ```bash
  scripts/train.sh autoc-tracker.ini logs/autoc-038-t4-m2-slowchan.log
  ```
- **US3 predictor head (M2-direct allowed)**:
  ```bash
  scripts/train.sh autoc-tracker.ini logs/autoc-038-t5-m2-predhead.log
  ```

Each NN-contract change → rebuild-perf + bitwise gate + xiao codegen sync (see
[contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md)) before launch.

## 4. Judge the ablations

Generate the full report set from the logfile (P0-C wrapper):

```bash
scripts/generate_pngs.sh m2 logs/autoc-038-t4-m2-slowchan.log \
  --compare baseline:logs/autoc-038-t2-m2-baseline.log
```

**Success gate (SC-001)**: an ablation moves at least one reward-invariant 037 ceiling —
- close-tracking fraction up from ~11–13 % (`mode_progress` track occupancy),
- median error down from ~17 m (`mode_progress` range, `score_by_path`),
- in-FOV up from ~70 % (`mode_progress` perception),
- reacquire (`maxLost`) shorter than 8–10 s (`mode_progress` reacquire).

Use the **fixed-eval comparator**, not raw late-run training fitness (variation-ramp pressure confounds it —
`project_late_run_fitness_interpretation`). `rnn_capacity` watches eff-rank for US2.

## 5. Combine winners + US4

Combine the winning structural levers into one architecture; re-bake. Then add the US4 visibility-maintenance
reward (reward-only, not format-breaking — can iterate without a re-bake) behind `EnableVisibilityReward`,
and confirm it lifts in-FOV / shortens reacquire without destabilizing selection or regressing the carried
crash penalty (SC-003, SC-004).

## Done-when

SC-000 (Phase 0) + SC-001 (a ceiling moved) + SC-005 (bitwise gate passes) — pin the milestone baseline,
record its S3 prefix, write the outcome doc. US5 (camera variations) stays a follow-on.
