# 043-t3 — M1 bake MANIFEST (crash-cost run)

**Purpose**: reproduce the exact genome produced by the t3 bake. ⛔ A genome without its **input scale
constants** loads clean and flies wrong — that is why they are transcribed below rather than referenced.

Written 2026-09-11, at run completion and **before** any `nnextractor` call (per NEXT-M1-RUN.md § 6:
`nnextractor` overwrites `nn_weights.dat`, after which which-run-and-generation-flew is unrecoverable).
Facts verified against S3, the run log, and git at that date.

---

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370248021823368-2026-09-08T02:02:32.439Z/` |
| Retention | ✅ `retain=keep` — re-tagged 2026-09-11, all 800 objects (0 failures), **verified by sampling 48**: `gen9200` (gen 800), `gen9999` (gen 1), `gen9900`, `gen9777`, `gen9666`, `gen9500`, `gen9350`, `gen9201` plus 40 at random — 48/48 `keep`. Objects upload as `retain=expire`; the 30-day lifecycle is real and would otherwise have reached this run ~2026-10-08. |
| Master seed | **1788832952** (logged as "Effective master seed" at run start; `autoc.ini` has `Seed = -1`, so the seed is resolved at launch and **this value is the only record of it**) |
| Regiment | pop **5000** × **800** generations × **294** scenarios (6 `SimNumPathsPerGeneration` × 49 `WindScenarios` ⇒ `ExpectedScenarioCount = 294`) |
| Launched | 2026-09-07 19:02 PDT, detached via `scripts/train.sh autoc.ini logs/autoc-043-t3-m1-crashcost.log` |
| Completed | gen 800 uploaded 2026-09-11 17:08 (`gen9200.dmp.zst`, the lowest-numbered object under the prefix) |
| Wall clock | 3 d 22 h (338,753 s), 1,176,235,200 sims @ 3472 sims/s |

⚠️ **A first launch attempt at 19:00 the same evening died ~1 s in** and wrote
`logs/autoc-043-t3-m1-crashcost.log.failed-noxvfb`. Cause: the `Xvfb :2` display the crrcsim workers need
had died, so every worker failed `SDL_SetVideoMode` ("Failed to create minimal OpenGL context for headless
mode", `crrcsim/src/crrc_main.cpp:822`) and the parent surfaced it as `TcpSocket::read recv: Connection
reset by peer`. That aborted attempt drew master seed **1788832839** — ⛔ **not** this run's seed, and it
produced no S3 prefix. Recorded here only so the two seeds in the logs are never confused.

## 2. The genome

| Field | Value |
|---|---|
| Object | `gen9200.dmp.zst` (42,153,467 bytes) |
| Generation | **800** — ⚠️ two labels for one net: `nnextractor` prints `Generation: 800` (filename-derived, `10000 − 9200`), `autoc` eval mode prints **`799`** (`genome.generation`, the trainer's 0-indexed counter). **Match on fitness, not on the generation label.** |
| Fitness | **−88026.619367** |
| Mutation sigma | 0.058121 |
| Topology | **45 → 32 → 16r → 3**, 2307 weights |
| Scenarios completed | **294 / 294** (`rabbitComplete=294`, `#GenCrash gen=800` all-zero) |
| `nn_weights.dat` | 9371 bytes, sha256 `a5d097da18bf33ed…` (extracted 2026-09-11 19:09) |
| ⭐ `weight_id` | **`a5d097da18bf33ed`** — sha256[0..15] of `nn_weights.dat`, emitted as `generatedNNWeightId` and written into every flight-log header. **This is the field that proves flown firmware carries THIS genome.** (t2 was `3af8e3ab787b75a5`.) |

Extract with:

```bash
./build/nnextractor -k autoc-9223370248021823368-2026-09-08T02:02:32.439Z -g 800 -o nn_weights.dat
./build/nn2cpp -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp
```

⚠️ `nnextractor -g` takes the **actual** generation (800), matching `dmp-dump --gen` — this CHANGED in 043
(T022); old `-g 9200`-style invocations are wrong now.
⚠️ `nn2cpp`: `-w` is the genome, `-i` is the config.
⛔ `xiao/src/generated/` is **git-ignored** — it does not survive a checkout and must be regenerated.

## 3. Commits

| Commit | What it pins |
|---|---|
| `be86390` | autoc at launch (`docs(043): NEXT-M1-RUN — fresh-context handoff for the t3 bake`, 2026-09-07). Tree was clean at launch. |
| `4a68a72` | **crrcsim submodule pointer** (`fix(043): back out the sensor LPF wiring — it was above Nyquist at the gather rate`). Pointer unmoved since. |

`autoc.ini` is **unchanged since `be86390`** (verified `git diff be86390 -- autoc.ini` empty), so the
working-copy ini *is* the as-run ini. Snapshotted alongside as
[`autoc.ini.as-run`](autoc.ini.as-run), sha256 `0020d171d39fb751…`.

## 4. ⭐ What this run changed vs 043-t2

The **only** `autoc.ini` delta between t2's launch (`c8d00ab`) and t3's is the crash-penalty block:

| key | t2 | t3 |
|---|---|---|
| `EnableHullCrashPenalty` | 0 | 0 (unchanged — hull is a TRACKER concept; enabling it for M1 is a category error) |
| `HullCrashPenaltyFactor` | 0.5 | 0.75 (inert for M1, gated off) |
| ⭐ `OobCrashPenaltyWeight` | **0.0** | **10.0** |

The other three t3 changes are outside `autoc.ini`: `COMPUTE_LATENCY_MSEC_DEFAULT` 30→10 ms (C++,
flight-measured 9.9 ms), `rc_expo` 20→0 and gyro Kalman OFF (aircraft/INAV), and the engage-prefill frame
fix + flight log v5 (firmware — affects *flight*, not this bake).

## 5. ⛔ Input scale constants (`include/autoc/nn/nn_inputs.h`)

Without these the genome loads clean and flies wrong. **Unchanged from t2** — verified against the header
at manifest time.

| Constant | Value |
|---|---|
| `kCruiseSpeed_mps` | 13.0 |
| `kDistToBoundaryScale_m` | 20.0 |
| `kTargetDistScale_m` | 26.0 |
| `kClosingRateScale_mps` | 16.0 |
| `kGyroScale_radps` | 6.0 |
| `kAccelScale_g` | 8.0 |
| `kEnergyScale_m` | 145.0 |
| `kScoreGradScale` | 0.78 |
| `kTimeSinceSeenScale_s` | 2.0 |
| `kNNHistoryLagsMsec[6]` | 800, 400, 200, 100, 50, 0 |
| `kNNHistoryLayoutVersion` | **3** |

Arena and cone are baked into the generated evaluator from `autoc.ini`:
arena `70.0, 25.0, 105.0` (`FlightArenaRadius` / `FloorAGL` / `CeilingAGL`);
cone `7.000, 2.000, 45.000, 0.500, 5.000, 5.000` (`FitDistScaleBehind`, `FitDistScaleAhead`,
`FitConeAngleDeg`, `FitStreakThreshold`, `FitStreakRampSec`, `FitStreakMultiplierMax`).

## 6. Result — the question this run answered

⭐ **Did pricing arena egress at 29/30 per crash move the endgame crash rate to 0–2 of 294 without costing
tracking?** Yes, on both halves.

| | 041-t7 (baseline of record) | 043-t2 (no crash price) | **043-t3** |
|---|---|---|---|
| gen-800 fitness | −81412.63 | −88013.84 | **−88026.62** |
| gen-800 `pctInStreak` | — | 54.6% | **54.6%** |
| gen-800 crashes | — | — | **0 / 294** |
| mean OOB/gen, gens 555–739 | 2.03 | 14.14 | **0.00** |
| mean OOB/gen, gens 751–800 | 4.32 | 13.14 | **0.00** |
| longest crash-free run | — | — | **466 gens** (335→800; final crash band was gens 321–334, 1–2 scenarios each) |

⭐ Per NEXT-M1-RUN.md § 2 the raw fitness is normally **not** comparable across the objective change — but
a crash-free genome scores ×1.000, so t3's −88,026.62 *is* fairly comparable to t2's −88,013.84. t3 is
marginally better on fitness, identical on streak, and carries **zero** crashes against t2's 12–21.

Both silent-failure modes from § 5 of the handoff are ruled out empirically:
- **(a) penalty never activates** — refuted: crash rate collapsed post-ramp and stayed at zero for
  hundreds of generations, which cannot happen if the term is a no-op.
- **(b) over-corrects into timidity** — refuted: `pctInStreak` matched the crashiest run (54.6% = t2) and
  beat 041-t7 by ~5 points. The policy goes to the boundary and does not cross it.

## 7. ⚠️ What this run does NOT fix — recorded at launch, not post-hoc

| | |
|---|---|
| **D1 pitch peak timing** | sim peaks at **85 ms**, real at **133–166 ms**. Measured, unfixed, and it is *the* axis 043 exists to fix. Candidates: `Cmq`, `Cm_alpha`, pitch inertia. **If the pitch result disappoints in flight, D1 is the first suspect** — it is named here at launch so it cannot read as an excuse invented later. |
| **D2 second airframe** | everything rests on **n = 1**, with a known-asymmetric wing and nose-heavy CG. The stall cycle in `actuator-pin.md` §8/§9 is the kind of thing that is an *article* property, so this bake may be tuned to one aircraft's defects. Parts procurement is slow — the operator accepted this. |
| **D4 NN sensor-path delay** | ~21 ms on accel, ~6.4 ms on gyro, un-modelled. An implementation was **backed out** (filters at the 20 Hz gather sit above Nyquist — that back-out is crrcsim `4a68a72`, pinned above); `include/autoc/eval/sensor_lpf.h` records the two correct approaches. |

## 8. Post-extraction (fill in after `nnextractor`)

| Field | Value |
|---|---|
| `nn_weights.dat` size / sha256 | 9371 bytes / `a5d097da18bf33edd4e16d6f08af3b3281a82737ad34ae3c23e5dbb58a4f6a1e` |
| ⭐ `weight_id` | `a5d097da18bf33ed` — embedded as `generatedNNWeightId`, verified to round-trip from `nn_weights.dat` |
| `firmware_id` | `1e8342d6a38d55f6` — sha256[0..7] of the generated NN source text (`nn2cpp`), embedded as `generatedNNFirmwareId` |
| Eval suite result | ✅ run 2026-09-11, `eval-results/2026-09-12T02:12:54Z/` — see § 9 |

Command: `./scripts/eval_suite.sh nn_weights.dat all 1788832952` (the seed arg is what makes tier 0 a real
determinism gate rather than a novel scenario table).

## 9. Eval suite — 2026-09-11

| tier | result | completion | score | strk | mult |
|---|---|---|---|---|---|
| tier0-repro | ✅ PASS | 294/294 | 299.4 | 85.7 | 4.43 |
| tier1-aeroStandard | ✅ PASS | 294/294 | 297.9 | 82.3 | 4.29 |
| tier2-progressive | ✅ PASS | 49/49 | 1270.0 | 100.0 | 5.00 |
| tier2-long | ✅ PASS | 49/49 | 295.8 | 97.6 | 4.90 |
| tier2-random | ❌ FAIL | 122/144 (84.7%) | 43.2 | 58.2 | 3.33 |
| tier3-stress | ❌ FAIL | 122/144 (84.7%) | 44.3 | 59.4 | 3.38 |
| tier3-quiet | ✅ PASS | 1/1 | 255.3 | 100.0 | 5.00 |

⭐ **tier0 matched BITWISE**: eval fitness `−88026.619367` == stored. Determinism holds and the master
seed above is confirmed correct.

⭐ **tier1 reproduces the crash-free result on a NOVEL seed** — 294/294, zero crashes. The zero is a
property of the policy, not of the trained scenario table.

⚠️ **The two FAILs are NOT a t3 regression — they are a mis-calibrated threshold.** Both are the
`random` 12×12 patrol/intercept geometry, which M1 never trains on (`aeroStandard` is the training path
type). Baseline comparison, same tier, same regiment, t2's flown genome
(`nn_weights-t2-3af8e3ab.dat`, run `eval-results/2026-09-12T02:15:14Z/`):

| tier | 043-t2 | **043-t3** |
|---|---|---|
| tier2-progressive | 48/49 (97.9%) | **49/49 (100%)** |
| tier2-long | 47/49 (95.9%) | **49/49 (100%)** |
| tier2-random | **49/144 (34.0%)** | **122/144 (84.7%)** |

t3 beats t2 on every generalization tier and is **2.5× better** on `random`. The 95% pass criterion in
`run_eval()` is inherited from the trained geometry and does not describe achievable performance on
untrained paths for an M1 genome — treat tier2-random / tier3-stress as **informational for M1** until
the threshold is re-derived. Crashes there are all `reason=Eval` (OOB) with `maxStrk=0`: the aircraft
departs the cylinder before ever tracking, clustered by path (path 0 loses 10 of 12 winds), i.e.
geometry-driven, not envelope-driven — tier3's 120% sigmas hit a *different* scenario set for the same
total.

⭐ **Root cause of the two FAILs (dug 2026-09-11) — entry RANGE, not the policy and not the script.**
`GenerateRandom` is the only generator that does not anchor `path[0]` at the canonical origin: every
control point, first included, is `localRandomPointInCylinder(...)`. `aeroStandard` (`entryPoint(0,0,0)`),
`progressiveDistance` (`raceStart(0,0,0)`) and `longSequential` (`origin(0,0,0)`) all anchor — and all
three score 100%. The aircraft is always placed at the origin (entry position sigmas are **0.0**;
run log confirms `posR=0.000000m posAlt=0.000000m`), so:

| | initial dist to rabbit at t=0 |
|---|---|
| tier1-aeroStandard (trained) | median **0.3 m** (min 0.1) |
| tier2-random | median **38.0 m** (min 19.3) |

`random` therefore poses a **cold acquisition from ~38 m**, while M1 is trained and scored on a tail-chase
already latched at t=0. Different task, same rubric. Failure signature matches: **100.0%** of crashed
ticks sit at idle throttle (`out_th` < −0.9) vs 6.9% in-distribution — the aircraft glides, `dist` grows
38 → 75 m monotonically, `maxStrk=0`, and it descends into the floor having never entered the cone.
⭐ Note the t2 OOD **pitch** rail (33.5% of floor-dying ticks) is **0.0%** in t3 — the crash price killed
that mechanism and the saturation migrated to throttle. Full write-up and routing consequences in
`specs/BACKLOG.md` § "does not generalize to the patrol/intercept start" → UPDATE 2026-09-11.

⇒ These two tiers measure an **untrained capability**, not a regression. t3's 84.7% is the acquisition
baseline to beat; the 95% bar in `run_eval()` has never been met by any M1 genome.

⛔ **`scripts/eval_suite.sh` was repaired to run this suite at all**: tiers 2/3 never restated
`ExpectedScenarioCount`, so the FR-058 guard (added in 043 `82e45f6`) aborted every regiment-changing
tier with `FATAL ERROR: scenario regiment = 49 … but ExpectedScenarioCount = 294`. Tiers 0/1 survived
only because they happen to run the training regiment. Five values added (49/49/144/144/1).

## 10. Flight prep (order per tasks.md: T068 → T059 → flash → T069 → T070)

⭐ **No INAV work.** t3's two aircraft-side changes — `rc_expo` 20 → 0 and `setpoint_kalman_enabled = OFF`
— were made and captured in `025ea0d` at **2026-09-07 13:44**, i.e. **before** the bake launched at 19:02.
t3 was baked against the aircraft as it now stands, so the flight article needs no retune and no reflash
(T052: no custom INAV firmware in 043, CLI config only). `control_profile 1` is active and carries
`rc_expo = 0`.

✅ **Rate-parity gate (T051b hard stop) holds**: model `maxRate` **360 / 240**
(`crrcsim/models/hb1_streamer.xml`) == FC `rates` **36 / 24** (`xiao/inav-hb1.cfg`, control_profile 1).
Neither side moved since t2. ⚠️ Re-assert after flashing the xiao — flashing is what could silently move it.

| step | state |
|---|---|
| T068 MANIFEST + `retain=keep` | ✅ done (this file; 800 objects pinned, verified 48/48) |
| T059 `nn2cpp` regenerate | ✅ done 2026-09-12 — header source `…2026-09-08T02:02:32.439Z/gen9200.dmp.zst`, topology `45 -> 32 -> 16r -> 3`, fitness −88026.619367, zero residual t2 references |
| xiao build gate | ✅ **SUCCESS** — RAM 53.5% (127204 / 237568), Flash 45.5% (369388 / 811008). Built, **not yet flashed**. |
| flash xiao | ⏳ operator |
| T069 bench-verify | ⏳ operator — ⭐ flash identity must read `weight_id=a5d097da18bf33ed` (t2 was `3af8e3ab787b75a5`); re-assert rate parity; ACRO purity; arm-C `axisF[1]` saturation |
| T070 fly | ⏳ operator — capture xiao log **and** blackbox; the clock-join is required (US1 dropped) |

⚠️ **Bench watch-item**: T069 recorded t2 railing **nose-down at 93% for the whole bench engagement** —
OOD saturation provoked by static airspeed (≈0.2 vs 13 m/s trained). t3 took the pitch rail from 33.5% to
**0.0%** on the *cold-start* OOD (§ 9), so it may behave better here too — but that is a **different** OOD
axis (zero airspeed, not range). ⛔ Do not treat it as a prediction: if t3 still rails on the bench, the
crash-price fix is range-specific and the static-airspeed hole is a separate, still-open gap.