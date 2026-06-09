# Quickstart: 034 M1/M2 Cleanup + Craft Variations

Operator runbook for verifying the implementation and running the US5 acceptance bakes.

## Build & test (per milestone)

```bash
bash scripts/rebuild.sh                              # autoc + crrcsim, runs GoogleTest
cd xiao && pio run -e xiaoblesense_arduinocore_mbed  # xiao still compiles (no 034 code there)
```

## Cleanup verification gates

```bash
# US1 — minisim gone
grep -rn minisim CMakeLists.txt tests/ src/ tools/ include/ | grep -v '// historical'   # → empty
ls tools/minisim.cc                                                                       # → not found

# US2 — smoothness gone (live refs only; historical comments/033-dir analysis OK).
# crrcsim's two "display smoothness" comments (crrc_main.cpp, CTime.cpp) are the
# render-loop frame pacing, unrelated to the NN motion-smoothness penalty — excluded.
grep -rniE 'smoothness|SmoothnessPenaltyFloor|smoothnessFactor|compute_smoothness_factor' \
  src/ include/ tools/ crrcsim/src | grep -v 'historical\|specs/033\|display smoothness'    # → empty

# Constitution VI — type-domain audit on touched paths
grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v '// raw-ok:'
```

## Behavior-neutrality (SC-002) — smoothness removal changed nothing

Smoothness was already bypassed, so a fixed-seed M1 eval must match pre-removal byte-for-byte:

```bash
# before removal (tag/stash), and after — compare per-scenario fitness
./build/autoc -i autoc-eval.ini   # grep the eval-mode #NNEval line from the run .log; diff before vs after
                                  # (per-gen stats now live in the .log, not eval-data.stc — retired T052)
```

## Craft-variation checks (SC-004 / SC-005)

```bash
# no-op: all Craft*Sigma = 0 must reproduce nominal-craft fitness exactly
./build/autoc -i autoc-eval.ini                      # [Craft] sigmas = 0 → identical to a no-craft baseline

# determinism: same Seed → identical craft draws (check craftSeed in dmp ScenarioMetadata round-trips)
# replay gate: the 033 bit-exact M1→M1 replay must still pass after the craft-seed cascade + seed-width change
```

## US5 — acceptance bakes (operator-run)

> Per `feedback_operator_runs_regression_gate`, the operator drives these. The basin lottery (~1:3 stuck) applies — budget 2–3 bakes for a flyable climber. Watch the throttle-σ + sigma-anneal early-detection signals (not the gen-150 heuristic).

```bash
# M1 (pathgen) bake with craft variations — produces craft-diverse source library
stdbuf -oL -eL ./build/autoc -i autoc.ini            # [Craft] sigmas set non-zero; pop=8000 wind=36

# M2 (tracker) bake against the M1 source library
stdbuf -oL -eL ./build/autoc -i autoc-tracker.ini

# config recoverability (SC-006): the startup banner now prints EVERY key (X-macro auto-print)
grep -A200 'AutocConfig' logs/autoc-*.log | head -120  # confirm [Craft] + all keys present
```

Then: extract winner → flash xiao → satisfy pre-flight prerequisites (failsafe bench verification per BACKLOG pre-flight item + `project_preflight_checklist`) → flight test → capture flight data (SC-006).

## Decisions to confirm before `/speckit.tasks`

- **D1**: craft-param scope — default = CG+drag+trim + thrust-scale, defer servo-lag.
- **D2**: drop FR-013 (crash-hull PRNG) + FR-014 (mod_inputdev link) as already-done? (verify first).
