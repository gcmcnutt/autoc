# 044 — quickstart (operator runbook)

All commands from the repo root on branch `044-m1-craft-history` (autoc and crrcsim). ⛔ No rebuild while a run is training.

## 0. Preserve (before any layout change)

```bash
git tag 044-last-45in                     # at the last 45-input commit (after plant + fixes, before the layout break)
# pin the 043 eval dumps the new build cannot read (keys recorded in the 044 MANIFEST)
aws s3api put-object-tagging --bucket autoc-eval --key <run-id>/<file> --tagging 'TagSet=[{Key=retain,Value=keep}]'
```

## 1. Profile (FR-015)

```bash
perf stat -e task-clock -- ./build/autoc -i <tier1-eval.ini>           # per-tick cost, with / without the controller node
perf record -g -p $(pgrep -x autoc) -- sleep 600                       # parent profile across one smoke generation
```
Record in `specs/044-m1-craft-history/profile.md`; keep only bitwise-neutral speed-ups (tier-0 replay identical).

## 2. Plant on the 45-input build + honesty ladder (FR-006…008)

```bash
bash scripts/rebuild-perf.sh && (cd build && ctest)
# step test at class centre / clamps (crrcsim StepTest model) → roll corner ≈ 3 / 5 / 1.5 Hz, same steady roll rate per aileron
# g752 on t6 vs new plant, same never-trained seed, full strength (EvalVariationScale = 1.0 once FR-011 lands):
./scripts/eval_suite.sh specs/043-acro-dual-loop/artifacts-t6/nn_weights-t6-g752.dat 0 20261018
python3 specs/043-acro-dual-loop/candidate_compare.py --draws <draws> --sim g752-t6:<csv>:<meta> g752-new:<csv>:<meta>
python3 specs/043-acro-dual-loop/airframe_id.py <bb.csv> --V 15      # flight reference for the closed-loop roll FRF
```
Pass: roll 3–5 Hz share rises toward the flight's 50 %.

## 3. Baseline fixes (FR-009…011) → new baselines

Unit tests for stream independence; scenario table shows `clP windMps turbX gustLx thermX`; `EvalVariationScale` honoured.
Record the new tier-0 baselines (eval vs training bitwise).

## 4. Smoke A (plant only, 45 inputs) — launch detached

```bash
bash scripts/train.sh specs/044-m1-craft-history/smoke.ini logs/autoc-044-smokeA-plant.log
```

## 5. Format break (FR-001…004), then Smoke B (plant + history)

```bash
bash scripts/rebuild-perf.sh && (cd build && ctest)
~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed -d xiao
bash scripts/train.sh specs/044-m1-craft-history/smoke.ini logs/autoc-044-smokeB-history.log
```
Compare per generation (SC-006): `scripts/generate_pngs.sh m1 <log> --compare smokeA:<logA>`.

## 6. Bake (FR-013)

```bash
bash scripts/train.sh autoc.ini logs/autoc-044-t1-m1-history.log      # MANIFEST written first; pin retain=keep at the end
bash scripts/generate_pngs.sh m1 logs/autoc-044-t1-m1-history.log --compare 043-t6-plant:logs/autoc-043-t6-m1-plant.log
```

## 7. Pick, fly, judge (FR-014, SC-001…005)

Equal-environment eval of finalists (fixed seed, `EvalVariationScale = 1.0`) → `candidate_compare.py`; `nnextractor` → `nn2cpp` → xiao
build and bench; fly with the frozen FC config (blackbox 1/8, SERVOS + ATTI, streamer per span, level boot); analyse with
`candidate_compare.py` (flight mode) and `airframe_id.py` against the 10-18 reference. A miss on SC-001 or SC-002 → another bake /
flight cycle; 045 waits.
