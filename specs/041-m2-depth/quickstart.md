# 041 Quickstart

How to run each phase, in order, with the gates that apply. Commands are illustrative of the intended
workflow; exact flags settle at `/speckit.tasks` and implementation.

**Read first**: [hypothesis.md](hypothesis.md) for why, [plan.md](plan.md) for the ordering constraints.

---

## Standing rules that apply throughout

| rule | source |
|---|---|
| Training launches only via `scripts/train.sh <ini> <logfile>` — detached, line-buffered, core-enabled. **Never** an agent background task | Constitution IX |
| **Never rebuild while a bake is live** — workers re-exec `build/autoc` | [[feedback_no_rebuild_during_training]] |
| Pre-run build gate before every bake — `cd build && make` for ordinary edits; `rebuild.sh` / `rebuild-perf.sh` when FP-determinism could move | Constitution IX |
| A `CMakeLists.txt` touch requires a clean `rebuild-perf.sh`, **operator-driven** | Constitution IV, [[feedback_operator_runs_regression_gate]] |
| Submodule pointer bump **first**, parent merge second | [[feedback_submodule_merge_order]] |
| Never `git push` — the operator drives pushes | [[feedback_operator_drives_push]] |
| ⚠️ **Disconnect the GPS module before flashing an INAV controller** | `specs/020-pre-flight-pipeline/plan.md` |
| INAV builds **two targets** — bench `MAMBAF722_2022A`, flight `MATEKF722MINI`. **Bench first**; bench-only validation is not flight validation | 021 T041 precedent |
| Reports come from `scripts/generate_pngs.sh <m1\|m2> <log>` — hand-calling the plotters loses the per-gen S3 cache | [[feedback_generate_pngs_wrapper]] |

---

## Phase A0 — research scan (no code changes)

Produce the inventory. Grep-driven, then read the hits:

```bash
# collection pairs indexed by a shared loop variable across a producer/consumer boundary
grep -rn "List\.at(\|List\[" src/eval/ src/nn/ tools/ crrcsim/src/mod_inputdev/ | less

# structs serving two lifetimes (RPC-only vs persisted)
grep -rn "serialize" include/autoc/rpc/ include/autoc/eval/ | less

# values duplicated across two definitions
grep -rn "default\|Default" include/autoc/eval/camera_projection.h include/autoc/util/config.h
```

**Output**: an inventory document listing every index-coupled pair, every two-lifetime struct, every
duplicated value — each marked *fixed*, *structurally eliminated*, or *covered by a zero-answer test*.
Plus the grouped-record migration list.

**Exit gate**: no known instance remains asserted only by a comment (SC-001).

---

## Phase A2/A3 — instruments (build these before spending compute)

### The ablation tool

```bash
cmake --build build --target nn_ablate
# validate the instrument BEFORE trusting it — empty mask must reproduce exactly
build/bin/nn_ablate -i autoc-eval.ini --genome <pinned-M1-elite>
# expect: ablated fitness == baseline fitness, bit-exact  (SC-004)
```

### The offline studies

```bash
# physics columns from the already-recorded trace (no recording change)
build/bin/dmp-dump --physics <run-id>/gen<N>.dmp.zst > ticks.csv

# Study A — regime + load
python3 src/analytics/regime_load_study.py --ticks ticks.csv --out study-a/

# Study B — predictor feasibility (target (b) only AFTER the FR-004 pairing fix)
python3 src/analytics/regime_load_study.py --predictor --ticks ticks-m2.csv --out study-b/
```

**Study A is REPORT-ONLY** (clarified 2026-08-07). 041 builds no load axis. Its findings feed the
follow-on aggressiveness feature, sequenced after the M2 predictor go/no-go. It therefore does not gate
A1 — run it whenever convenient, but before the outcome is written.

---

## Phase A1 — the one contract-break commit

Everything model- or schema-incompatible lands together (FR-005). Nothing else lands until Phase B
completes.

Checklist:

- [ ] Grouped per-tick record; initial state as a named field; `stepIndex - 1` clamp **deleted**
- [ ] Prediction-score pairing fix (FR-004)
- [ ] New input slots, both modes, with metadata rows — see
      [contracts/nn-input-layout.md](contracts/nn-input-layout.md)
- [ ] `wind_velocity` set at record time
- [ ] Config block recorded per gen; readers prefer it over the ini
- [ ] Exact tick stamping — **crrcsim submodule, pointer bump first**
- [ ] Version field bumped; fail-loud read naming both versions
- [ ] `INI_MAX_LINE` raised + `ParseError()` line number surfaced
- [ ] `contract_tracker_config_tests.cc` decoupled from mutable production values
- [ ] 4 `DISABLED_` tests in `selection_tests.cc:152+` re-enabled if the selection plumbing is reused
- [ ] `nn2cpp` regenerated; xiao builds

Gate:

```bash
bash scripts/rebuild.sh                                    # compile + full test suite
cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed
# CMakeLists.txt touched (nn_ablate target) → operator runs:
bash scripts/rebuild-perf.sh
# Constitution VI audit on touched paths:
grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'
```

---

## Phase B — M1 bake

### B1 — smoke tests (plumbing and ballpark only, NOT comparison arms)

The input change is generic, so prove it runs in **both** modes:

```bash
scripts/train.sh autoc-basic-m1.ini logs/autoc-041-smoke-m1.log     # small pop, fast
scripts/train.sh autoc-tracker.ini  logs/autoc-041-smoke-m2.log     # short, kill once ticking
```

Check: builds, runs, climbs at all, new inputs carry sane values. **No delta is being measured.**

### B2 — production bake

```bash
scripts/train.sh autoc.ini logs/autoc-041-t1-m1-envelope.log        # pop 8000 / 49 winds
tail -f logs/autoc-041-t1-m1-envelope.log
```

Artifact naming follows `autoc-<feature>-t<N>-<details>`
([[feedback_artifact_naming_convention]]).

**Abort early on the stuck-basin signature** (research.md R8): throttle amplitude exactly 1.000 with
**σ = 0.000** and `dCtrl` 0.000, plus `avgMaxStreak` frozen and best-sigma not annealing past ~0.14 by
gen 200. Note that throttle *saturation* (mean ≈0.85, σ>0) is **not** the tell — climbers pass through it.
Budget ~3 attempts.

Judge climb on `pctInStreak` / `avgMaxStreak`, **not** completions
([[project_servo_era_progress_metrics]]).

### B — post-bake reads

```bash
scripts/generate_pngs.sh m1 logs/autoc-041-t1-m1-envelope.log

# H1a — does the policy USE the envelope state?  (the rigorous read)
build/bin/nn_ablate -i autoc-eval.ini --genome <new-elite> --zero-input IN_ENVELOPE,ENVELOPE_SECS
build/bin/nn_ablate -i autoc-eval.ini --genome <new-elite> --zero-input ACCEL_X,ACCEL_Y,ACCEL_Z

# per-regime aggressiveness + load
python3 src/analytics/regime_load_study.py --ticks <new-elite-ticks.csv> --out study-a-new/
```

**Then, immediately** — pin and archive, because this becomes the M2 source:

```bash
# tag all objects retain=keep (Constitution VIII.2) and record the prefix in the outcome doc
# archive nn_weights*.dat beside the dmp (FR-010) — the dmp preserves numbers, only NN01 preserves a
# controller you can re-fly
cp data.dat specs/041-m2-depth/artifacts/m1-t1-data.dat     # FR-022 — next launch overwrites it
```

---

## Phase C — predictor decision (no bake)

```bash
# E1 — is the dead head taxing the search?
# EnablePredictorHead = 0 vs 1, short tracker runs (NOT an M1 run — the head is tracker-only)
scripts/train.sh autoc-tracker.ini logs/autoc-041-t2-m2-nohead.log
```

Combine with Study B's verdict → **C3 decision**: re-targeted head, value-head fallback, or **retire**
(output 7→3). Retirement is an accepted outcome.

---

## Phase D — M2 bake

```bash
# repoint autoc-tracker.ini at the new pinned M1 source, then:
scripts/train.sh autoc-tracker.ini logs/autoc-041-t3-m2-predictor.log
scripts/generate_pngs.sh m2 logs/autoc-041-t3-m2-predictor.log
```

⚠️ **Eval-ini hazard**: `autoc-eval-tracker.ini` is one file serving two incompatible jobs
(training-repro vs novel-geometry). Reproducing a run once needed **four** hand-aligned fields, including
the scenario *shape*. The 1:1 seed-table guard catches count mismatches only — a config differing in sigmas
or enables produces a plausible wrong number. Check shape explicitly when repointing.

Novel-geometry read uses the pinned 038-t10 source. The second novel source was deliberately left to
expire, so a difference cannot be cross-checked against an independent sample — state that limitation in
the outcome.

---

## Wrap

- Record every hypothesis as supported or refuted **with its evidence**. A refuted hypothesis is a
  successful outcome (SC-012).
- Pinned prefixes recorded in the outcome doc (Constitution VIII.3 — provenance lives in the repo).
- Deferred items returned to `specs/BACKLOG.md` in order (Constitution X), including accelerometer
  noise/bias modelling and anything Study A removed from scope.
