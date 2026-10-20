# 044-t1 — `autoc-044-t1-m1-history` — MANIFEST (written BEFORE launch; draft 2026-10-19, T048)

Pattern of record: [`../../043-acro-dual-loop/artifacts-t6/MANIFEST.md`](../../043-acro-dual-loop/artifacts-t6/MANIFEST.md). Why t1
exists: the 2026-10-18 t6 flight halved the 2 Hz pitch cycle but left 3–5 Hz dither in pitch and **doubled** it in roll, because the
sim's roll mode responded ~4.5× too slowly and the policy had no history of its own sensors. t1 is **t6's regiment, objective and
pool** with three changes — a partly honest roll plant, streamer-spanning pitch damping, and craft-sensor history — plus the baseline
fixes. Spec: [`../spec.md`](../spec.md); evidence: [`../honesty.md`](../honesty.md), [`../profile.md`](../profile.md),
[`../MANIFEST.md`](../MANIFEST.md) (gates, baselines, smokes).

## 1. The run  *(fill at launch — T049)*

| Field | Value |
|---|---|
| S3 prefix | *(from the log)* |
| Master seed | *(`Seed = -1`; the "Effective master seed" log line is the only record of it)* |
| Regiment / pool / ini | **identical to t6** except the keys in § 2: pop 5000 × 800 gens × 294 scenarios (6 × 49, `ExpectedScenarioCount 294`), wind envelope 0–8 m/s, `randomA` slot 3, `ExcessRotationAxisMode = conditional`, ramp 40 |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-044-t1-m1-history.log` |
| Launched | *(date, pid; startup log: 53,32,16r,3 / 2563 weights, scenario table with `clP windMps turbX gustLx thermX`, 20 workers, no errors)* |

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

Commits *(fill at launch)*: autoc `…`, crrcsim `…`; `autoc.ini` as-run snapshot `autoc.ini.as-run` + sha256.

## 3. Gates before launch

| gate | result |
|---|---|
| roll honesty ladder | g752 on the new plant: 3–5 Hz roll RMS **112 / 96 (calm) °/s vs flight 97**; closed-loop roll \|H\| 0.82–0.92 vs flight 0.8–1.0; class corners by the flight's fit 3.24 / 5.81 Hz (`honesty.md`) |
| 45-input baselines (T026) | t6 g800 `-69975.152740`, g752 `-67058.096781`; eval-vs-training bitwise on a 3-gen run |
| index-free gate | clean (168 files); layout change touched only the layout header, topology, recording sites, schema bump and one flight-log scale range |
| 53-input baseline (T046) | eval-vs-training bitwise: `NN_EVAL_SAME -5.443911` on a 3-gen smoke-regiment run |
| ctest / xiao | 55/55; `pio run` SUCCESS with a 53-input program |
| attribution (SC-006) | *(smokes A / B — `../MANIFEST.md` § SC-006; the bake keeps history either way)* |
| ⛔ **pre-run gate at launch** (Constitution IX) | clean `scripts/rebuild-perf.sh` + `ctest` on the launch commit — **after both smokes have finished** (no rebuild while a run trains) |

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
