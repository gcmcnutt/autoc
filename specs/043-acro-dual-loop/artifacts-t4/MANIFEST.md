# 043-t4 — M1 bake MANIFEST (wind envelope + second random course + excess-rotation axis)

**Purpose**: reproduce the genome this bake produces, and fix at LAUNCH what it may and may not be
compared against. Written 2026-10-06 at launch, **before** any `nnextractor` call (it overwrites
`nn_weights.dat`, which currently holds the t3 genome `a5d097da18bf33ed`).

Fresh-context handoff for the run: [`../NEXT-T4-RUN.md`](../NEXT-T4-RUN.md). Derivation of the objective
change: [`../tempered-control-first-principles.md`](../tempered-control-first-principles.md).

---

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370245510873367-2026-10-07T03:31:42.440Z/` |
| Retention | ⏳ uploads as `retain=expire` — **pin `retain=keep` on all 800 objects at the end and verify by sampling** (t3 procedure: 48/48 sampled) |
| Master seed | **1791343902** (`Seed = -1` in the ini; logged as "Effective master seed" — **this is the only record of it**) |
| Regiment | pop **5000** × **800** generations × **294** scenarios (6 paths × 49 winds; `ExpectedScenarioCount = 294` guards it) |
| Launched | 2026-10-06 20:31 PDT, `bash scripts/train.sh autoc.ini logs/autoc-043-t4-m1-windrot.log`, pid 90646, detached under `systemd --user` |
| Variation ramp | `VariationRampStep = 40` ⇒ 20 steps over 800 gens; env classes (incl. the new wind envelope) at scale **0 through gen 40**, 1.0 from gen 761 |
| Completed | ⏳ |

## 2. Commits

| | |
|---|---|
| autoc | `ad674d1` — *feat(043 t4): excess-rotation lexicase axis* (contains T087/T087a/T087b/T088/T100/T102). Tree clean at launch. |
| crrcsim | `c4b3e49` — *feat(043 t4): per-scenario wind envelope*. Pointer unmoved. |
| `autoc.ini` | as-run snapshot [`autoc.ini.as-run`](autoc.ini.as-run), sha256 `2551f61cee10bea0…` — identical to the working copy at launch |
| build | clean `scripts/rebuild-perf.sh` (operator, 2026-10-06 20:26), `PERFORMANCE_BUILD=ON`; ctest 53/53; t3 as-run ini reproduces `NN_EVAL_SAME −88026.619367` on this binary |

## 3. What changed vs 043-t3 (the ONLY ini deltas: 1 value, 12 new keys, nothing removed)

| | t3 | **t4** |
|---|---|---|
| **objective** | tracking + throttle energy; OOB crash price 10.0 | **+ `EnableExcessRotationAxis = 1`** — craft rotation minus the path's demand, per scenario, co-equal lexicase case |
| **wind speed** | 3.66 m/s every tick | **uniform 0–8 m/s** per scenario (`WindSpeedMinMps/MaxMps`), ramped |
| **direction** | σ 45° about 330° | **σ 60°** |
| **turbulence** | Dryden, TI 15.7%, L fixed | **TI ×0.5–2.0**, **L ×0.25–1.0** per scenario |
| **thermals** | 0–5 cells, 2.0 ± 0.5 m/s | **count_max 8**, **strength ×0.5–1.5** |
| **courses** | 45° loop in slot 3 (7.7 s) | **`AeroStandardPath3 = randomA`**, `RandomPathSeedA = 12345` — 2/6 random-entry |
| entry geometry, crash price, streak multiplier, servo/craft variations, pop, gens | unchanged | unchanged |
| D1 (pitch short period in sim) | absent | **still absent** — see § 6 |

All new keys default OFF in code; no wire-format change; t2/t3 dmps remain readable.

## 4. ⛔ What is and is not comparable

- **Raw fitness is NOT comparable to t3's −88,026.62** — different regiment (wind, a second random course) and a different selection pool. Do not quote it against t3.
- Compare on: **crash rate** (t3 endgame: 0/294); **`pctInStreak` binned by realized wind** (`wN/wE` in the dmp; t3's 54.6% was at a fixed 3.66 m/s); **`rotRatio`** (`#NNGen`), baseline **5.10×** = the t3 genome under *this* regiment (Straight 6.09× / Spiral 4.57 / Fig-8 4.03 / RandomA 5.10 / HighPerch 6.92 / RandomB 4.87); **cold-start completion** on tier2-random (t3: 84.7%); per-axis `dCtrl`/⟨|out|⟩ (variation-stable comparator).
- ⚠️ `#NNGen` `excessRot=`/`rotRatio=` describe the **elite**; the axis selects per scenario.
- **Compute parity**: gen 1 t3 112.50 s vs t4 113.31 s; gen 2 107.68 vs 106.71; identical 1,470,294 sims/gen ⇒ no measurable per-cycle cost from the changes (re-check at matched gens 50/100/200/400).

## 5. "Tempered, not dull" — acceptance stated now (tasks.md T106)

Sim: `pctInStreak` on calm-wind bins within noise of t3; crash rate ≤ t3; **`rotRatio` median < 5.10×** with the straight-and-level path no longer the worst; `|n_z|` p99 < 4.30 g and ticks > 3 g < 12.6% as consequences; pitch 2–3 Hz share stays ~10%. Flight: pitch-rate power in 2–3 Hz ≪ 56–81%; `|accSmooth[2]|` p99 < 5.2 g; flown `pctInStreak` ≥ 54.6%.

## 6. ⚠️ What this run does NOT fix — recorded at launch

| | |
|---|---|
| **D1 pitch short period** | the sim settles by ~175 ms where the real aircraft rings at 3.0–3.8 Hz (`actuator-pin.md` § 7). **Not in t4.** ⇒ t4 cannot remove the flight 2–3 Hz oscillation by itself; the excess-rotation axis removes the *commanded* floor (the sim's 5× wandering), D1 is what lets the sim price the resonance. **Do not judge t4's flight on pitch smoothness alone.** |
| **D2 second airframe** | still n = 1, known-asymmetric wing. |
| **D4 sensor-path delay** | ~21 ms accel / ~6.4 ms gyro, un-modelled (`sensor_lpf.h` records the correct approaches). |
| **Entry dither** | deferred (T085/T086): 5 of 6 paths still start latched; cold-start content is the two random courses. |
| **AIRSPEED = groundspeed** on both sides | in a 0–8 m/s envelope the policy has no airspeed signal; pitot is backlog. |

## 6b. ⛔ Mid-run finding (gen 396, 2026-10-07) — the Independent rotation case diluted selection

Operator decision: **run to 800 for the record**; the fix is staged as t5 (tasks T109/T110, [NEXT-T5-RUN.md](../NEXT-T5-RUN.md)).

| | t3 | t4 |
|---|---|---|
| pool per scenario | score, energy | score, energy, **excess_rotation** (tracking 1/2 → 1/3 of starts) |
| pop-avg fitness gen 40 (**identical wind**, ramp 0) | −4.2k | **−1.4k** |
| pop-avg fitness gen 396 | −26.6k | −3.9k (flat since gen 200) |
| elite replaced after gen 281 | every ≤ 26 gens | **never** — `best=` moves only at ramp steps 281/321/361 |
| `pctInStreak` gen 396 | 48.7% | 30.5% (flat since 250); per path behind on every course |
| shared-5 `rotRatio` gen 396 | 6.11× | 7.25× (same genome since 281) |
| determinism | `NN_ELITE_SAME` 800/800 | `NN_ELITE_SAME` **405/405**, 0 diverged — not the cause |

Mechanism: `path_rotation` is the same for every genome on a scenario, so a standalone rotation case is won by whoever rotates least — a passive flyer wins it everywhere. **What t4 still delivered**: the wind envelope works and reproduces the flight brittleness — at gen 405 (realized 1.7–5.9 m/s) score/step 0.48 → 0.41 → 0.29 and `pctInStreak` 41 → 36 → 24% across the 0–2 / 2–4 / 4–6 m/s bins, every path worse in 4–6 (`tracking_by_wind.png`). T108: per-gen wall clock ~2× faster than t3 at matched gens — a weaker population dying early, not a compute saving.

## 7. Post-run (fill in at the end, BEFORE extraction)

| Field | Value |
|---|---|
| **STOPPED by operator 2026-10-07 21:3x PDT at gen 413** (of 800; 25 h 00 m) | after the § 6b diagnosis — "stop current run and close it out"; t5 carries the fix |
| gen-413 fitness / `pctInStreak` / `avgMaxStreak` / crashes / `rotRatio` | **−52569.586322** / **30.5%** / 69.9 / **0/294** / **7.25×** (same elite since gen 281; `best=` moved only at ramp steps 281/321/361/401) |
| per path at 413 (score/step, pctInStreak, rotRatio) | Straight 0.476 / 43.8% / 8.77× · Spiral 0.462 / 44.9% / 7.42× · Fig-8 0.422 / 40.4% / 5.94× · randomA 0.215 / 21.1% / 7.25× · HighPerch 0.386 / 40.3% / 9.53× · RandomB 0.177 / 16.8% / 6.55× — all 0 crashes |
| `retain=keep` pinned + sampled | ✅ all **413** objects tagged (0 failures), verified by sampling 22 incl. `gen9999` (gen 1) and `gen9587` (gen 413) |
| extracted genome | `artifacts-t4/nn_weights-t4-g413.dat` sha256 **`7bb858ae439a2fbd`** (2307 weights, fitness −52569.586322, σ 0.115645). ⛔ **Not flown, not copied to `nn_weights.dat`** — which still holds t3 `a5d097da18bf33ed`, now also backed up as `artifacts/nn_weights-t3-a5d097da.dat`. No `weight_id`/`firmware_id`: nothing was generated for the aircraft. |
| eval suite (`eval_suite.sh artifacts-t4/nn_weights-t4-g413.dat all 1791343902`, `eval-results/2026-10-08T04:35:13Z`) | see table below — **more robust, less precise** than t3 |
| determinism | `NN_ELITE_SAME` **413/413**, 0 diverged |

**Eval suite, t4 gen-413 genome vs t3 gen-800** (`eval-results/2026-10-08T04:35:13Z`, tier 3 re-run in `…T04:38:48Z` after the quiet-overlay fix below):

| tier | t4 completion | t4 score / strk / mult | t3 completion | t3 score / strk / mult |
|---|---|---|---|---|
| tier0-repro | 294/294, **bitwise** −52569.586322 | 178.8 / 69.9 / 3.80 | 294/294, bitwise | 299.4 / 85.7 / 4.43 |
| tier1-aeroStandard (novel seed) | 294/294 | 180.9 / 70.6 / 3.83 | 294/294 | 297.9 / 82.3 / 4.29 |
| tier2-progressive | 49/49 | 620.5 / 97.6 / 4.90 | 49/49 | 1270.0 / 100.0 / 5.00 |
| tier2-long | 48/49 (1 crash) | 110.5 / 59.6 / 3.39 | 49/49 | 295.8 / 97.6 / 4.90 |
| tier2-random (cold start, novel courses) | **144/144 (100%)** | 152.4 / 42.6 / 2.70 | 122/144 (84.7%) | 43.2 / 58.2 / 3.33 |
| tier3-stress (120% env sigmas) | **144/144 (100%)** | 147.9 / 39.7 / 2.59 | 122/144 (84.7%) | 44.3 / 59.4 / 3.38 |
| tier3-quiet | 1/1 | 96.3 / 33.0 / 2.30 | 1/1 | 255.3 / 100.0 / 5.00 |

Read: the t4 genome **generalises better and tracks worse** — the operator's feared trade, literally. Cold-start/novel-course completion went 84.7% → **100%** on both 144-flight tiers (second random course + wind envelope doing their job), while on-course precision fell by ~40% everywhere (tier1 score 181 vs 298, quiet 96 vs 255) and the trained-regiment `pctInStreak` sat at 30.5% vs 54.6%. Not flown: the precision loss is the Independent-case dilution (§ 6b), which t5 removes while keeping the envelope and the second random course.

⚠️ **Suite fix found here**: `tier3-quiet` aborted (`std::runtime_error`: "Wind envelope keys are set but EnableWindVariations=0") because `autoc-eval.ini` now carries the envelope keys and the quiet overlay only cleared the master enable — the fail-loud worked as designed. `scripts/eval_suite.sh` quiet overlay now also sets the nine envelope keys to their OFF values.
