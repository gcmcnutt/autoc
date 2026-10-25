# 044-t1 — `autoc-044-t1-m1-history` — MANIFEST (written before launch 2026-10-19; launched 2026-10-20)

Pattern of record: [`../../043-acro-dual-loop/artifacts-t6/MANIFEST.md`](../../043-acro-dual-loop/artifacts-t6/MANIFEST.md). Why t1
exists: the 2026-10-18 t6 flight halved the 2 Hz pitch cycle but left 3–5 Hz dither in pitch and **doubled** it in roll, because the
sim's roll mode responded ~4.5× too slowly and the policy had no history of its own sensors. t1 is **t6's regiment, objective and
pool** with three changes — a partly honest roll plant, streamer-spanning pitch damping, and craft-sensor history — plus the baseline
fixes. Spec: [`../spec.md`](../spec.md); evidence: [`../honesty.md`](../honesty.md), [`../profile.md`](../profile.md),
[`../MANIFEST.md`](../MANIFEST.md) (gates, baselines, smokes).

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370244339188361-2026-10-20T16:59:47.446Z/` |
| Master seed | **1792515587** (`Seed = -1`; logged as "Effective master seed" — the only record of it) |
| Regiment / pool / ini | **identical to t6** except the keys in § 2: pop 5000 × 800 gens × 294 scenarios (6 × 49, `ExpectedScenarioCount 294`), wind envelope 0–8 m/s, `randomA` slot 3, `ExcessRotationAxisMode = conditional`, ramp 40 |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-044-t1-m1-history.log` |
| Launched | **2026-10-20 09:59 PDT**, pid 3279118, detached via `scripts/train.sh`; startup log: topology `53,32,16r,3` (2563 weights), 294 scenarios (first `0x2842893059ea1d1e`), `AeroStandardPath3: randomA`, `ExcessRotationAxisMode: conditional`, scenario table with `cmQ clP windMps turbX gustLx thermX`, 20 workers, no errors |

## 2. What changed vs t6

| | t6 | **044-t1** |
|---|---|---|
| NN inputs | 45 (layout v3) | **53 (layout v4)**: gyro p/q/r + accel z at 100 / 50 ms beside each current value; weights 2307 → **2563**; layout signature `a505f1be760da67d` |
| roll damping | fixed `Cl_p` −0.135 (~1.1 Hz corner) | **`craftClP` class**: centre −0.36 (~3 Hz), clamp [−0.60, −0.18], σ 0.10; `Cl_da` scaled with it (steady roll rate per aileron held) |
| pitch damping class | centre −1.0, clamp [−2.0, −0.2], σ 0.6 | **centre −2.25, clamp [−3.5, −1.0], σ 0.5** (bare → streamer-on) |
| FC sensor filters | none (raw FDM rates / specific force) | **gyro PT1 25 Hz + accel biquad 15 Hz at the 5 ms sub-step**, read at the 20 Hz tick (~6 / ~15 ms) |
| variation streams | class streams = shifted copies of one Park–Miller sequence; wind direction shared the speed draw | **fmix32 class sub-seeds; wind direction from its own slots** |
| logging / eval | — | scenario-table wind columns; `EvalVariationScale`; `#GenTiming` (+ population ticks); async elite-dmp upload (bitwise-neutral) |
| formats | dmp v4, NN01 v1, flight log v5 | **dmp v6, NN01 v2 (layout signature), flight log v6 (slot names in the header)** |
| objective, pool, courses, wind envelope, seed policy | | **unchanged** |

Commits: autoc **`a2dadea`** (launch; + the record commit that follows), crrcsim **`54d2485`**. `autoc.ini` as-run snapshot [`autoc.ini.as-run`](autoc.ini.as-run), sha256 `f71f686f5e0f4677…`.

## 3. Gates before launch

| gate | result |
|---|---|
| roll honesty ladder | g752 on the new plant: 3–5 Hz roll RMS **112 / 96 (calm) °/s vs flight 97**; closed-loop roll \|H\| 0.82–0.92 vs flight 0.8–1.0; class corners by the flight's fit 3.24 / 5.81 Hz (`honesty.md`) |
| 45-input baselines (T026) | t6 g800 `-69975.152740`, g752 `-67058.096781`; eval-vs-training bitwise on a 3-gen run |
| index-free gate | clean (168 files); layout change touched only the layout header, topology, recording sites, schema bump and one flight-log scale range |
| 53-input baseline (T046) | eval-vs-training bitwise: `NN_EVAL_SAME -5.443911` on a 3-gen smoke-regiment run |
| ctest / xiao | 55/55; `pio run` SUCCESS with a 53-input program |
| attribution (SC-006) | **not met** — history arm no smoother in 200 gens (late medians 3–5 Hz roll 132 vs 94 °/s, pitch 86 vs 78); early pitch advantage gens 51–101; bake keeps history (clarification 1) — `../MANIFEST.md` § SC-006 |
| **pre-run gate at launch** (Constitution IX) | ✅ clean `scripts/rebuild-perf.sh` 2026-10-20 09:54 on `a2dadea` (no run training), ctest **55/55**, input-layout gate clean (169 files) |

## 3a. First generations

| gen | best | pop avg | pctInStreak | `#GenSel` bite | gen time (serial tail) | mean ticks / sim |
|---|---|---|---|---|---|---|
| 1 | −3358 | −1093 | 0.4 % | 95.8 % | 107.8 s (2.3 s) | 85.6 |
| 2 | −4286 | −1094 | 0.3 % | 95.8 % | 103.1 s (2.6 s) | 81.6 |

`NN_ELITE_SAME` gens 1–2; `uploadWaitSec` ≈ 0 (the async upload keeps up); per-tick cost 16.8 µs.

## 4. What is and is not comparable

- Fitness is not comparable to any earlier run (plant, inputs and scenario draws all changed).
- Comparable to t6 at matched gens: per-path score/step, `pctInStreak`, crashes, `rotRatio`, population average — t6 is the dashed
  reference (`generate_pngs.sh m1 logs/autoc-044-t1-m1-history.log --compare 043-t6-plant:logs/autoc-043-t6-m1-plant.log`).
- The roll rows of `pitch_cycle_progress` (roll-rate RMS, 3–5 Hz share) are the new primary read beside pitch.
- `#GenTiming meanTicksPerSim` lets the per-generation cost be read against population flight length (profile.md (c)).

## 5. Early calls and the exit

t6 for reference: gen 5 best −4458 / avg −1142 / pctInStreak 0.9 %; gen 40 −17409 / −2943 / 4.8 %; gen 100 −38337 / −11203 / 19.3 %;
generation time 102 s (gen 5), 237 s (40), 465 s (100), 550–575 s (400–800) before the async upload.

| when | healthy | stop-and-rethink |
|---|---|---|
| gen 2–5 | `#GenSel` bite ≫ 0; `#GenTiming` serial tail < 2 s | inert tie-break; `uploadWaitSec` > 0 every gen |
| gen 5–40 | elite replaced every few gens; pop-avg moving (a slower start than t6 is plausible: harder roll plant, more inputs) | elite frozen 40 gens or pop-avg flat |
| gen ≥ 100 | 3–5 Hz roll and pitch rate RMS **falling** with gens while `pctInStreak` rises | roll 3–5 Hz RMS stuck near g752's 112 °/s on the new plant |
| exit (T054) | the flight: SC-001 roll 3–5 Hz < 82 °/s, SC-002 pitch 3–5 Hz < 48 and 2–3 Hz ≤ 51 °/s, SC-003–005 | a miss on SC-001/002 → another bake / flight cycle; 045 waits |

## 6. Close-out  *(T050)*

`retain=keep` on every object (verify by sampling incl. gen 1 / gen 800), final-elite extraction, eval suite with the run's seed
(tier 0 bitwise), then the equal-environment candidate pick (T051, `eval_suite.sh … 0 <never-trained seed> 1.0`).

## 7. Finished — 2026-10-25

800 generations, `NN_ELITE_DIVERGED` 0 over the whole run. Final elite **−103751.666151**, pctInStreak **49.5 %** (t6 at gen 800:
−93924.534457, 46.7 %). Smoothness on the training curves (each elite at its own variation strength) flat from ~gen 550.

### Pre-flight gate estimate (T051 prep)

Equal environment: tier 0, seed **20261018**, `EvalVariationScale 1.0`, the 044 plant — the same environment t6 g752 (flown
2026-10-18) was evaluated in during the honesty ladder. `../gate_estimate.py`; departed scenarios excluded; medians over scenarios.
Flight prediction = candidate sim × (g752 flight ÷ g752 sim) per metric.

| genome (sim, equal env) | roll 3–5 Hz | pitch 3–5 Hz | pitch 2–3 Hz | load 1.5–5 Hz | in-streak |
|---|---|---|---|---|---|
| t6 g752 (flown reference) | 112.7 °/s | 27.4 | 20.1 | 0.84 g | 36.1 % |
| **t1 g800 (final)** | **31.0** | **17.9** | 21.5 | **0.58** | **52.0 %** |
| t1 g396 | 26.8 | 31.0 | 26.4 | 0.75 | 40.6 % |
| t1 g606 | 31.2 | 29.5 | 24.7 | 0.73 | 50.7 % |
| t1 g541 | 34.7 | 29.2 | 30.5 | 0.94 | 46.4 % |

sim→flight ratios from g752: roll 0.86, pitch 3–5 Hz 2.05, pitch 2–3 Hz 2.53, load 2.24.

| predicted flight | roll 3–5 Hz (SC-001 < 82) | pitch 3–5 Hz (SC-002 < 48) | pitch 2–3 Hz (SC-002 ≤ 51) | load (SC-005 < 1.88) |
|---|---|---|---|---|
| g752 (measured) | 97 | 56 | 51 | 1.88 |
| **g800** | **27 ✅** | **37 ✅** | **54 — borderline** | **1.30 ✅** |
| g396 / g606 / g541 | 23–30 ✅ | 60–63 ✗ | 63–77 ✗ | 1.6–2.1 |

Reading:
- **Roll — very likely passes.** −72 % against the flown genome on the same plant and environment; the roll calibration (0.86) is close to
  1 because the honesty ladder fitted it.
- **Pitch 3–5 Hz — likely passes** (−35 % in sim). **Pitch 2–3 Hz — borderline**: g800 is no better than g752 in that band in sim
  (21.5 vs 20.1), predicted 54 vs the ≤ 51 gate. The pitch ratios (2.0–2.5) say the sim still shows about half the pitch activity the
  aircraft does, so the pitch predictions are the least certain.
- **Load passes; tracking better** (52 vs 36 % in-streak on the same plant).
- **The mid-run "smooth" elites are not smoother in an equal environment** — their training-curve smoothness came from easier
  scenarios (lower variation strength). g800 is the best candidate on every metric but pitch 2–3 Hz.
- **Attribution**: this compares t1 (trained on the honest roll plant, with history) against t6 (trained on the old plant). It cannot
  separate the plant from the history inputs; the smokes (same plant, ±history) found no smoothness advantage from history, so the
  roll gain is most plausibly the plant.
