# Quickstart — 035 verification gate + energy bake

The Phase A → **GATE** → Phase B runbook. The gate is the user's load-bearing ask: prove the
factory still works (basic train, basic eval equivalent, basic M2, dmp readable) **before** any
fitness change.

> Build discipline: interactive iteration = incremental builds
> ([feedback_incremental_build_default]); the bit-replay gate uses a clean `rebuild-perf.sh`
> ([reference_perf_build_reproducibility]). Never rebuild while a training run is live
> ([feedback_no_rebuild_during_training]). The operator drives the regression gate
> ([feedback_operator_runs_regression_gate]).

---

## Prerequisite (admin, before bucket cutover — R6)
- [ ] Create buckets `autoc-m1`, `autoc-m2`, `autoc-eval`.
- [ ] Grant `s3:PutObjectTagging` + `s3:GetObjectTagging` to IAM user `autoc-generator`.
- [ ] Apply `contracts/lifecycle-policy.json` to each bucket (+ legacy `autoc-storage`).
- [ ] Re-tag milestone runs `retain=keep` (LETTER §1 table) before lifecycle goes live.

(Code/gate can run on the *old* buckets first; flip `S3Bucket` per `contracts/ini-config.md` once
the above is done.)

---

## Phase A — build
```bash
bash scripts/rebuild-perf.sh        # clean FP-deterministic build; links libzstd
```
Expect: libzstd found at top-level CMake; `dmp-dump` target built; crrcsim mod_inputdev links
`autoc_common`.

---

## GATE — must all pass before Phase B

### 1. basic M1 train works (no data.dat, zstd, tagged)
```bash
stdbuf -oL -eL ./build/bin/autoc -i autoc-basic-m1.ini   # pathgen, pop 3000, ~few gens
```
- [ ] **zero** `data.dat` written anywhere (FR-P05).
- [ ] dmps land under `s3://autoc-m1/autoc-<id>/gen<N>.dmp.zst` (uniform naming, FR-P07b).
- [ ] each object carries tag `retain=expire` (`aws s3api get-object-tagging …`) (FR-P10).

### 2. basic eval is equivalent (bit-replay gate, FR-P04)
```bash
# eval the just-trained elite; compare per-scenario scores to the training-time scores
./build/bin/autoc -i autoc-eval.ini          # EvaluateMode path
```
- [ ] `NN_EVAL_SAME` (or per-scenario `ScenarioScore` vector byte-identical) — **not** whole-dmp
      bytes (dmp carries non-deterministic provenance timestamps).
- [ ] confirms Bug 3 (rabbit-speed) holds: eval uses the configured rabbit profile (FR-P14).

### 3. basic M2 works (tracker, source resolves)
```bash
stdbuf -oL -eL ./build/bin/autoc -i autoc-tracker.ini    # smoke; source dmp from autoc-m2
```
- [ ] `TrackerSourceRun` resolves (no dangling-pointer fail) — FR-P13.
- [ ] dmps land under `s3://autoc-m2/autoc-<id>/gen<N>.dmp.zst` (same naming as M1; bucket is the
      only difference).

### 4. we can read the dmp files (FR-P02/P03)
```bash
dmp-dump s3://autoc-m1/autoc-<id>/gen<N>.dmp.zst --meta-only      # YAML run/scenario meta
dmp-dump s3://autoc-m1/autoc-<id>/gen<N>.dmp.zst --csv-only \
  | python3 specs/029-no-future-arch/plot_per_axis_time_series.py # existing plot consumes it
```
- [ ] YAML block parses; CSV columns match the documented set; a plot script renders without
      data.dat.
- [ ] `.zst` auto-inflates; a legacy plain `.dmp` also reads (back-compat read).

**GATE GREEN ⇒ proceed to Phase B.**

---

## Phase B — energy bake

### 5. unit-level (pre-bake)
- [ ] `energy_metric_tests` — convex integral `Σ((out_th+1)/2)²` correct, ≥0 (data-model).
- [ ] 4 Selection027 tests re-enabled and green (FR-004): `EnergyBreaksTrackingTie`,
      `TradeoffBothSurvive`, `StabilityBreaksTrackingTie`, `ThreeWayTradeoffAllSurvive`.
- [ ] MAD-epsilon test; `LexicaseEpsilonMode=constant` reproduces a prior run bit-for-bit (SC-003).

### 6. M1 energy bake (FR-005)
```bash
# autoc.ini: Mode=pathgen, pop=8000, wind=36, LexicaseEpsilonMode=mad, energy axis on
stdbuf -oL -eL ./build/bin/autoc -i autoc.ini
```
- Budget **2–3 bakes** (basin lottery). Watch the throttle-σ=0.000 stuck-basin tell
  ([project_m1_basin_lottery_actual_rate]).
- Compare vs M1 tracking-only baseline (pop=8000/wind=36 + craft variations). **Primary comparator
  = per-axis aggressiveness distributions** (`dmp-dump … | plot_per_axis_time_series.py` →
  `per_axis_aggressiveness` PNGs): variation/config-stable, so it isolates the energy effect from
  the simultaneous MAD-ε change. Expect movement toward goal on **all 3 controls
  (pitch/roll/throttle)**; tracking quality (per-scenario score + avgMaxStreak) must not materially
  regress; `energy_score` must materially improve.

### 7. M2 energy bake (FR-005b)
```bash
# autoc-tracker.ini: same energy axis + MAD epsilon; objective identical (on-point + min energy)
stdbuf -oL -eL ./build/bin/autoc -i autoc-tracker.ini
```
- Budget 2–3 bakes. Capture `#GenCrash hullStrike=N` escalation (seeds the future hull-crash
  feature). Compare vs the **pinned M2 tracking-only baseline = 032-phase1**
  (`autoc-9223370257807536859-2026-05-17…`) or a fresher confirmed non-stuck climber — M2 is
  lottery-prone (stalls / dead neurons), so the baseline MUST be a confirmed climber. Same per-axis
  comparator as M1.

### 8. verdict (SC-001/002, FR-007)
- [ ] Outcome doc classifies **each mode**: energy-works / tracking-collapses / energy-unmoved,
      with per-scenario evidence.
- [ ] total-energy (PE/KE) go/no-go recorded (FR-007); ESC-current-monitor calibration follow-on
      noted if energy works but the modeled `f()` is the limiter.
- [ ] pin any flown/milestone run `retain=keep` and record its S3 prefix in the outcome doc
      (Principle VIII).
