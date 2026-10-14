# 043-t6 — `autoc-043-t6-m1-plant` — MANIFEST (written BEFORE launch, 2026-10-13)

Pattern of record: [artifacts-t5/MANIFEST.md](../artifacts-t5/MANIFEST.md). Why t6 exists: five ACRO flights showed a 2 Hz, ±200 °/s,
±7 g pitch cycle that no sim genome exhibited, because the sim's pitch short period sat at 6.9 Hz / ζ 0.81 (model `I_yy 0.0013`) where the
aircraft's is ~3 Hz / ζ 0.2–0.3. t6 is **t5's regiment, objective and pool on an honest plant**. Evidence and derivation:
[sim-pitch-fidelity-proposal.md](../sim-pitch-fidelity-proposal.md) (§ 1, § 2b "applied"), [actuator-pin.md](../actuator-pin.md)
(§ Inertia measurement, § The t6 plant), [tempered-control-first-principles.md § 11](../tempered-control-first-principles.md).

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370244903245464-2026-10-14T04:18:50.343Z/` |
| Master seed | **1791951530** (`Seed = -1`; logged as "Effective master seed" — the only record of it) |
| Regiment / pool / ini | **identical to t5**: pop 5000 × 800 gens × 294 scenarios, wind envelope 0–8 m/s, `randomA` slot 3, `ExcessRotationAxisMode = conditional`, ramp 40. The only ini change is `CraftCmQSigma 0.32 → 0.6` (class re-centred in code). |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-043-t6-m1-plant.log` (Xvfb `:2` up) |
| Launched | **2026-10-13 21:18 PDT**, pid 2146100, detached via `scripts/train.sh`; startup log confirms `ExcessRotationAxisMode: conditional`, `AeroStandardPath3: randomA`, 20 workers, no errors; `autoc.ini` sha `344330b996958ab3` = as-run |

## 2. What changed vs t5 — the plant, and only the plant

| | t5 | **t6** |
|---|---|---|
| `Mass` / `I_xx` / `I_yy` / `I_zz` | 0.515 / 0.024 / 0.0013 / 0.026 | **0.510 / 0.0069 / 0.0094 / 0.016** (measured bare, bifilar; `I_zz` flat-body estimate) |
| pitch `Cm_a` / `Cm_q` / `Cm_de` | −0.55 / −4.2 / −0.36 | −0.55 / **−1.0** / −0.36 — `Cm_a`, `Cm_de` unchanged; the FRF of gen 488 rejected −1.0/−0.65 |
| pitch short period (analytic, 14 m/s) | 6.9 Hz / ζ 0.81 | **2.6 Hz / ζ 0.4** (aircraft ~3.0 / 0.2–0.3) |
| roll `Cl_*` / yaw `Cn_*` | — | × 0.2875 / × 0.615 — roll and yaw dynamics **held constant** across the inertia change (roll FRF of gen 488 verified) |
| `kCraftCmQCenter` / clamp / `CraftCmQSigma` | −4.2 / [−5.0, −3.6] / 0.32 | **−1.0 / [−2.0, −0.2] / 0.6** (pitch damping stays a craft variation) |
| NN→surface latency | configured 10 ms, **honoured as one 50 ms frame** (frame-top apply) | **honoured at the 5 ms sub-step** (`applyPendingSubstep`) |
| `Global::craftCmQ` default | hard-coded −4.2, overrode the xml in non-autoc runs | NaN (unset) + xml fallback |
| objective, pool, courses, wind, seeds policy | | **unchanged** |

Commits: autoc **`290b098`** (+ the record commit that follows it), crrcsim **`7f345c5`**. `autoc.ini` as-run snapshot [`autoc.ini.as-run`](autoc.ini.as-run), sha256 `344330b996958ab3…`.
Build: clean `scripts/rebuild-perf.sh` 2026-10-13 21:06 (`PERFORMANCE_BUILD=ON`), ctest **53/53**.

## 3. Gates run before launch (actuator-pin § "The t6 plant")

| gate | result |
|---|---|
| step test (plant only, same binary A/B) | pitch peak **135–155 ms** (old plant 50 ms; aircraft 133–166), overshoot 1.3–1.7× (aircraft ~2.5×), no FDM overflow after the harness trim retune |
| **gen 488 reproduces its own flights** | RMS **105–123 °/s** (flight 124–153), peak **1.9–2.3 Hz** (2.0–2.3), 2–3 Hz share 24–47 % (31–63 %), closed-loop \|H\| 1.34 at 2.5 Hz (1.5–1.6), tracking 48 → 40 %, 3 crashes — the sim now exhibits the cycle |
| roll preserved | gen 488 roll FRF t5 vs t6 plant within ~15 % at 0.5–2.5 Hz |
| new eval baselines (plant changed; expected) | t3 genome **−54922.067792** (was −88026.619367); gen 488 **−65928.363124** (was −95067.305736) |
| cross-build determinism | ✅ incremental build vs clean `rebuild-perf.sh`: t3 genome **−54922.067792**, gen 488 **−65928.363124** — identical to the last digit |
| **T092 trainability** | ✅ **passes, ~2× slower start** — basic-m1 smoke (pop 3000, 16 winds, full variations from gen 1, Seed 4242, same binary): old plant takes off at gen 13 (−835 at 25); new plant flat to gen 26, then −621 (40) → −799 (50) → −1029 (64–80), 16/16 complete from gen 42, `pctInStreak` 10.4 % at 80 |

## 4. ⛔ What is and is not comparable

- Fitness is not comparable to any earlier run (plant changed).
- **Comparable to t5 at matched gens**: per-path score/step, `pctInStreak`, crashes, `rotRatio`, population average (`#NNGen avg=`) — t5 is the
  dashed reference now (`generate_pngs.sh … --compare 043-t5-condrot:logs/autoc-043-t5-m1-condrot.log` builds its per-path cache once).
- `sim_pitch_spectrum.py` on the elite's dmp is the new primary read: on this plant a genome that dithers pays in pitch rate.

## 5. Early calls (same discipline as t5) and the exit

| when | healthy | stop-and-rethink |
|---|---|---|
| gen 2–5 | `#GenSel` bite ≫ 0 (same pool as t5) | inert tie-break |
| gen 5–40 | elite replaced every few gens; pop-avg moving (t5: −1.1k @5, −4.3k @40; a slower start is expected on the harder plant) | elite frozen 40 gens (the t4 signature) **or** pop-avg flat as in the smoke — then T092's options |
| gen ≥ 100 | `sim_pitch_spectrum` of the elite: pitch RMS and 2–3 Hz share **falling** with gens while `pctInStreak` rises | RMS stuck at g488's 105–123 °/s = the objective still cannot see it |
| exit (T118) | the t6 genome **in flight**: pitch-rate RMS well under 124–153 °/s, 2–3 Hz share well under 31–63 %, setpoint off the cap, tracking within noise of g488 | |

### 5a. ✅ Gen 2–5 and gen 40 calls (2026-10-13/14)

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t6 elite complete | note |
|---|---|---|---|---|---|---|
| 5 | −1145 | −1142 | 0.6 | 0.9 | 109/294 | `#GenSel` bite 96–97 %, `tbMeanIn` 12–122 — the tie-break has leverage on this plant too |
| 10 | −1187 | −1155 | 0.4 | 0.9 | 109/294 | |
| 20 | −1603 | **−1164** | 1.7 | 0.9 | 109/294 | the slow start the smoke predicted: elite unchanged, population flat |
| 30 | −2937 | −1394 | 3.2 | 0.8 | 258/294 | takeoff between 25 and 30 (smoke: gen 27) |
| 40 | −4292 | **−2943** | 5.6 | 4.8 | **294/294** | slope 30→40 now steeper than t5's (+1549 vs +1355); ~8 gens behind t5, far above t4's −1391 |

Reading: healthy — the honest plant costs ~8 generations at the start, exactly the smoke's pattern, then climbs at least as fast; the
elite is crash-free at 40 as t5's was; 11 elite replacements in 40 gens; `NN_ELITE_SAME` 40/40; `rotRatio` 9.4 vs t5 9.1 (crash-dominated
regime, no read yet). Pace 237 s/gen at gen 40 vs t5's 313 at the same gen (identical 1,470,294 sims/gen) — t6 is *faster* so far, the usual
weaker-population-flies-shorter effect; no compute concern from the sub-step hook.

### 5b. Gen 92 read (2026-10-14 morning): tracking has caught t5 — and the early elite is a full relay

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete |
|---|---|---|---|---|---|---|---|
| 50 | −5228 | −4590 | 8.3 | 8.6 | 9.42 | 7.81 | 293 |
| 70 | −6718 | −7860 | 13.4 | 12.4 | 7.72 | 7.08 | 286 |
| 90 | −8422 | **−10198** | 16.6 | 16.8 | 8.88 | 6.55 | 293 |
| 92 | −8663 | **−10310** | 17.2 | 16.5 | 7.63 | 7.08 | 291 |

Population average passed t5's at ~gen 60 and is 19 % ahead at 92; `pctInStreak` level; 29 elite replacements; `NN_ELITE_SAME` 92/92;
tie-break bite 89 %, `tbMeanIn` 6. Crash-carrying elites are frequent (45 of gens 41–92 carry 1–8 crashes) — the lightly damped plant
departs more easily; watch whether that settles as it did in t5. Pace 470 s/gen at 92 (t5 ~330) — the population now flies long.

**First `sim_pitch_spectrum` read of the elite (gen 92)** — the number that was blind on the old plant:

| | **t6 elite gen 92** | flown g488 on this plant | g488 in flight |
|---|---|---|---|
| pitch-rate RMS | **118–152 °/s** | 105–123 | 124–153 |
| mean \|pitch cmd\| | **179–191 °/s** | 115–121 | setpoint median 114–138 |
| ticks at the cap | **44–56 %** | 0–1.8 % | p5/p95 at the cap |
| command flips/s | **5.1–6.0** | ~3 | — |
| pitch energy | 3–3.6 Hz peak, 3–5 Hz share 35–55 % | 1.9–2.3 Hz | 2.0–2.3 Hz |
| closed-loop \|H\| 2.5 Hz / phase | 1.34 / −102° | 1.34 / −83° | 1.5–1.6 / −60° |

**Matched-generation check (same script on t5's gen-92 dmp)** — this is the early-training character on *either* plant, not a
t6 artefact: t5's gen-92 elite was at the pitch cap **43.9 %** of ticks (|out| 0.72, 3.7 flips/s) and ended at **1.4 %** (0.42, 2.9) by
the flown gen 488. t6 at 92 is harsher by degree — pitch cap 50.8 %, 5.5 flips/s, and **roll** at the cap 45 % (t5: 7 %), throttle 55 %
(43 %). So the verdict is the **slope**: t5 shed the relay from 44 % → 1.4 % over ~400 gens on a plant where only the tie-break could
see it; on this plant the relay also costs tracking, so t6 should shed it at least as fast. `pitch_cycle_progress.py` (cap %, flips/s,
pitch RMS per generation, t5 dashed) is the panel for it.

Reading, stated plainly: the population that evolves *on* the ringing plant goes straight to **relay control at the cap** — the pitch
setpoint is pinned at ±240 °/s half the time and flips 5–6×/s, which is how one tracks tightly on a lightly damped airframe early, and it
tracks as well as t5 did. The plant now *shows* this (RMS at flight level, a 3–3.6 Hz cycle); whether the objective *prices* it is the
question of the next 100–300 generations: **cap %, flips/s and pitch RMS must fall while `pctInStreak` rises**. On this plant the relay also
costs tracking (g488 lost 8 pts to it), so plain tracking selection — not only the tie-break — pushes against it once smoother equal
trackers exist in the population. If at gen ~300 the elite is still at the cap 50 % of the time, the measure (or the action space) is next,
not the plant.

## 6. Post-run

⏳
