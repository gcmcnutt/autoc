# 028 — Quickstart (operator walkthrough)

End-to-end recipe for running the 028 experiment envelope. Audience: someone executing
the plan after `/speckit.tasks` has produced `tasks.md`.

## Prerequisites

- Branch `028-deeper-rnn` checked out, clean working tree.
- `bash scripts/rebuild.sh` green on the cadence7-redux baseline (i.e., 027 close state).
- Logs directory exists: `mkdir -p logs/` if missing.
- `autoc.ini` present at repo root with `nMembers=3500`, `nGenerations=400`, current cadence.
- Read [`spec.md`](./spec.md), [`plan.md`](./plan.md), and the 027 findings.md before starting.

## Phase 1 — Land telemetry on the cadence7-redux binary (telemetry-only smoke)

Goal: prove signals 1 + 2 compute correctly with `NN_RECURRENT[2]=false`.

```bash
# 1.1 Implement signal 1 hooks (src/nn/evaluator.cc) per data-model.md §2
# 1.2 Implement signal 2 hooks (src/autoc.cc population stats) per data-model.md §3
# 1.3 Add tests/nn_telemetry_tests.cc per evolution_log_columns.md test surface
# 1.4 Add specs/028-deeper-rnn/plot_evolution_progress.py per plot_panel_api.md

bash scripts/rebuild.sh                                    # green
./build/tests/nn_telemetry_tests                           # all pass

# Smoke run on cadence7-redux (recurrent OFF, no marker flips yet)
nohup ./build/autoc -c autoc.ini > logs/autoc-028-smoke.log 2>&1 &
# Wait ~30 generations
tail -f logs/autoc-028-smoke.log | grep '^#NNGen'

# Verify each #NNGen line has the four new fields:
#   whh_xh_ratio=0.0000
#   w_xh0_cv=<positive> w_xh1_cv=<positive> w_hh_cv=<sentinel>

# Render 6-panel plot
python3 specs/028-deeper-rnn/plot_evolution_progress.py logs/autoc-028-smoke.log \
    --out specs/028-deeper-rnn/smoke_evolution.png

# Visual check: panels 1–5 look like cadence7-redux baseline.
# Panel 6 top half: empty / "no recurrent layer" annotation.
# Panel 6 bottom half: w_xh0_cv + w_xh1_cv traces only (red trace omitted).
```

**Gate**: smoke run completes 30+ gens, telemetry fields present in every `#NNGen` line,
6th panel renders without errors. Per-gen wall-clock within ~5 % of cadence7-redux.

## Phase 2 — Flip CADENCE7-REDUX markers to re-enable D-simple

Per [plan §Phase 2](./plan.md#phase-2--re-enable-d-simple-cadence7-redux-marker-flips). Markers
3 + 5 (selection.cc + selection_tests) **stay commented** for D-alone.

```bash
# Edit the 3 markers that flip for D-alone (NOT 3 + 5):
#   include/autoc/nn/topology.h:52-57   → NN_RECURRENT[2] = true
#   include/autoc/nn/topology.h:69      → static_assert == 1923
#   tests/contract_evaluator_tests.cc:14 → expect 1923 / NN_RECURRENT[2]=true

bash scripts/rebuild.sh                                    # green
./build/tests/contract_evaluator_tests                     # passes with new expectation
./build/tests/nn_evaluator_tests                           # recurrent forward pass tests pass
./build/tests/nn_telemetry_tests                           # signals still pass
```

## Phase 3 — D-alone diagnostic run

```bash
nohup ./build/autoc -c autoc.ini > logs/autoc-028-dalone.log 2>&1 &

# Monitoring loop — every 50 gens regenerate the 6-panel plot
# (kick off in another terminal):
while true; do
  python3 specs/028-deeper-rnn/plot_evolution_progress.py logs/autoc-028-dalone.log \
      --out specs/028-deeper-rnn/dalone_evolution.png
  sleep 600    # ~10-min cadence; tune to per-gen time
done

# Also regenerate control aggressiveness PNG every 50 gens
# (existing tool from 027 — exact invocation TBD in tasks):
python3 scripts/control_aggressiveness.py logs/autoc-028-dalone.log \
    --out specs/028-deeper-rnn/dalone_aggressiveness.png
```

**Early-stop conditions** (per plan §Phase 3): kill the run if either triggers and the
condition is sustained for ≥ 30 generations.

- Best fitness > pid1's −27045 floor by gen 100 with no descent.
- `whh_xh_ratio` flat near zero across gens 50–150 (recurrent block never engages).

## Phase 4 — Branch on D-alone outcome

Read the final-generation values from `logs/autoc-028-dalone.log`:

```bash
# Last #NNGen line
tail -20 logs/autoc-028-dalone.log | grep '^#NNGen' | tail -1

# Control aggressiveness late-plateau (last 50 gens)
python3 scripts/control_aggressiveness.py logs/autoc-028-dalone.log --last-n-gens 50
```

Compare against [validation gate](./spec.md#validation-gate-carries-from-027-plan):

| If… | Then proceed to… |
|---|---|
| Fitness ≤ −30000 AND dCtrl ≤ 0.80 AND ⟨\|out\|⟩ ≤ 2.00 | **Phase 6 win path** (sim gate + flight test) |
| Fitness ≤ −30000 BUT dCtrl > 0.80 or ⟨\|out\|⟩ > 2.00 | **Phase 5a** (stability-only lexicase) |
| Fitness > −30000 AND `whh_xh_ratio` flat near zero | **Phase 5b** (orthogonal W_hh init) |
| Fitness > −30000 AND `w_hh_cv` low vs `w_xh*_cv` healthy | **Phase 5c** (larger budget) |
| Fitness > −30000 AND no clear telemetry signature | **Phase 5b** by default (cheapest non-budget lever) |

## Phase 5a — Stability-only single-axis lexicase (if triggered)

```bash
# Edit selection.cc marker 3: uncomment ONLY the stability pool.push_back call.
# Leave energy commented.
# Un-DISABLED only the Selection027v4* tests in tests/selection_tests.cc.
bash scripts/rebuild.sh
./build/tests/selection_tests                              # Selection027v4* passes

nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5a.log 2>&1 &
# Same monitoring cadence as Phase 3.
```

Decision after run: same gate-comparison as Phase 4 against the new log.

## Phase 5b — Orthogonal W_hh init (if triggered)

```bash
# Edit src/nn/population.cc: replace Xavier init for W_hh with orthogonal init.
# Reference: research_rlayer_placement.md §4 (Saxe et al. 2013).
# Verify spectral radius of init lands in [0.7, 1.05] via assert.
bash scripts/rebuild.sh
./build/tests/nn_telemetry_tests                           # green

nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5b.log 2>&1 &
```

## Phase 5c — Larger budget (if triggered)

```bash
# Edit autoc.ini: nMembers 3500 → 5000 (or nGenerations 400 → 600).
# No code changes.
nohup ./build/autoc -c autoc.ini > logs/autoc-028-pattern2-5c.log 2>&1 &
```

## Phase 6 — Win path (sim gate + flight)

```bash
# 1. Confirm robustness metric pop_spread ≥ threshold per data-model.md §4
python3 scripts/compute_pop_spread.py logs/autoc-028-<winning-attempt>.log
# If pop_spread < 0.05, run a second-seed sanity run before flight.

# 2. Extract NN weights from winning generation
./build/tools/nnextractor <S3-or-local-dmp-path> nn_weights.dat

# 3. Generate xiao firmware (separate xiao port plan triggered here)
# Out of 028 plan scope; see triggered xiao plan for details.

# 4. Flight test on the winning architecture.
# Successful flight → 028 closes, ships winner.
```

## Phase 6 — Bounded no-go (if 3 attempts exhausted without a win)

```bash
# Produce specs/028-deeper-rnn/findings.md summarizing:
#   - Telemetry signal evolution across all attempts (which hypothesis the data supports).
#   - Final fitness/dCtrl/amplitude per attempt.
#   - Carry-forward to 029 (which CADENCE7-REDUX markers stay tripped, what the data says
#     about scenario-physics need from spec §Q4 adjacent observation).

# 028 closes; spec 029 picks up.
```

## Troubleshooting

- **Build fails after marker flip**: weight count mismatch in test or static_assert. Walk
  the markers in [plan §Phase 2](./plan.md#phase-2--re-enable-d-simple-cadence7-redux-marker-flips)
  table — all 3 (D-alone) or 5 (pattern 2) must be in sync.
- **`whh_xh_ratio` always 0.0 with recurrent ON**: telemetry capture flag isn't being set
  for best-of-gen eval. Check `src/autoc.cc` near line 1206 for the flag-set call before
  `logGenerationStats(gen)`.
- **`w_hh_cv` nan when recurrent ON**: weight block isn't being included in the population
  stats loop. Verify per-block iteration covers `W_hh` when `NN_RECURRENT[i]==true`.
- **Plot panel 6 empty with full 028 log**: parser missed the new fields. Confirm
  `whh_xh_ratio` and `w_*_cv` keys present in `#NNGen` lines via `grep '^#NNGen' logs/...`.

## Hand-off if you stop mid-attempt

The `CADENCE7-REDUX` markers are durable carry-forward — leave them in whatever state the
last attempt requires; the next operator picks up from there. Per attempt, snapshot to
`specs/028-deeper-rnn/<attempt>_evolution.png` and `<attempt>_aggressiveness.png` for the
findings.md handoff at close.
