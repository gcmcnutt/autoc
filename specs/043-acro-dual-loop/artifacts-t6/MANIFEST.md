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

### 5c. ✅ Gen 100 call (2026-10-14): the relay is coming off, on t5's schedule

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete |
|---|---|---|---|---|---|---|---|
| 60 | −6340 | −6541 | 13.4 | 9.6 | 7.72 | 8.64 | 294 |
| 80 | −8124 | −9280 | 14.6 | 13.3 | 7.90 | 7.13 | 291 |
| 100 | −9391 | **−11203** | 22.0 | 19.3 | 6.46 | 6.67 | 293 |

Pop-avg 19 % ahead of t5, `pctInStreak` ~3 pts behind (within the gen-to-gen jitter of a single elite), 32 error-ish log lines are all
startup/benign (0 DIVERGED), 465 s/gen.

**The relay panel** ([`autoc-043-t6-m1-plant_pitch_cycle.png`](../autoc-043-t6-m1-plant_pitch_cycle.png), new this call — elite
pitch-rate RMS, cap % per axis, flips/s, band shares per sampled gen, t5 dashed from a one-off 161-gen cache):

| elite | pitch cap % | roll cap % | throttle cap % | pitch flips/s | pitch RMS (its own plant) |
|---|---|---|---|---|---|
| t5 gen 96 → 101 → 106 | 46 → 19 → **7** | 8 → 1 → 0 | 49 → 46 → 37 | 4.2 → 4.6 → 3.6 | 153 → 126 → 101 |
| **t6 gen 94 → 96 → 100** | 52 → 27 → **31** | 41 → 43 → **19** | 48 → 50 → 41 | 5.8 → 5.5 → 4.1 | 132 → 139 → 129 |
| t5 gen 486 (flown g488 ±) | 0.4 | 0.2 | 31 | 2.9 | 81 |

Per-path at gen 100 (`sim_pitch_spectrum`): cap 26–39 % (was 44–56 at 92), flips 3.9–4.6 (5.1–6.0), RMS 113–144 °/s (118–152), peak
1.4–2.0 Hz with the 3–5 Hz share down to 29–37 % — the elite is holding the setpoint shorter and the cycle is sliding down toward the
aircraft's 2 Hz, which is what the honest plant should produce.

Two readings. (1) **t5 came off the cap at exactly this point too** — 46 % at 96, 7 % at 106, and never above 38 % again — so the relay
phase is the *takeoff* strategy on either plant and t6 is on the same clock, two or three gens behind. (2) ⚠️ **Level, not shape, is the
new information**: t5's dashed RMS is measured on the stiff plant that hid the cycle — its gen-486 elite reads 81 °/s there and flew at
124–153. t6's solid line is honest. So the exit test for this run is not "cap % → 1 %" (t5 did that and still rang in flight) but
**pitch RMS well below the 124–153 band while tracking holds**, and that is a gen-200+ read. Watch for the t5 pattern of brief cap
relapses (gen 121–136, 260, 310) — each was a new elite re-finding tracking the crude way before smoothing again.

### 5d. Gen 180 read (2026-10-14 evening): tracking well ahead of t5, relay half off, roll still dithering

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete | t6 s/gen | t5 s/gen |
|---|---|---|---|---|---|---|---|---|---|
| 120 | −9885 | −12519 | 27.3 | 25.2 | 5.05 | 6.62 | 290 | 457 | 422 |
| 140 | −10257 | −14528 | 29.7 | 28.4 | 6.16 | 7.03 | 292 | 497 | 417 |
| 160 | −10626 | −17877 | 31.9 | 31.8 | 6.21 | 5.67 | 294 | 506 | 422 |
| 180 | −12496 | **−18142** | 32.3 | **33.6** | 6.43 | 6.10 | 292 | 505 | 451 |

Population average **45 % ahead of t5** at 180; `pctInStreak` has crossed t5's (33.6 vs 32.3); per-path score/step 0.423 vs t5's
0.42 at the same gen; 16 elite replacements in 100–180; `NN_ELITE_SAME` 181/181; 0 DIVERGED. Crash-carrying elites remain frequent
(50 of gens 101–180 carry 1–4 crashes; 2/294 at 180) — the lightly damped plant departs more readily and the price is small.

**Relay panel, gen 100 → 180 (elite medians over 294 scenarios; t5 at the same gens in brackets)**

| | gen 100 | gen 140 | gen 160 | **gen 180** | t5 @181 |
|---|---|---|---|---|---|
| pitch-rate RMS °/s | 129 | 133 | 109 | **108** | 128 (stiff plant, understated) |
| pitch ticks at cap % | 31 | 63 | 14 | **13.5** | 6 |
| roll ticks at cap % | 19 | 53 | 18 | **18** | 5 |
| throttle ticks at cap % | 41 | 43 | 47 | **45** | 40 |
| pitch flips/s | 4.1 | 6.0 | 6.0 | **4.9** | 4.5 |
| roll flips/s | 7.4 | 8.5 | 8.9 | **7.9** | 6.1 |
| 2–3 Hz / 3–5 Hz share % | 18 / 32 | 16 / 46 | 24 / 21 | **16 / 25** | 9 / 38 |

Reading: the pitch relay is half off (cap 52 → 13.5 %, RMS 132 → 108 °/s, 3–5 Hz share 46 → 25 %) with the t5-style relapses
(gen 106, 126–131, 141) each undone within ~5 gens. Pitch RMS **108 is already below the flight band (124–153) and below g488's
105–123 on this plant only marginally** — the exit asks for "well below", so this is the number to keep watching. **Roll is the
laggard**: 18 % at the cap and ~8 flips/s where t5 was at 5 % / 6 flips at the same gen — the lighter `I_xx` (0.0069 vs 0.024) makes
roll cheap to dither; the tie-break sees it (excess rotation counts all axes), tracking does not much. Throttle at the cap 45 % is
the familiar un-lagged-actuator bang-bang (037 pattern), unchanged from t5.

**Why generations are slow (asked 2026-10-14; log-only analysis, all five bakes have identical 1,470,294 sims/gen on this host)**

| run | plateau s/gen | sims/s at plateau | elite ticks/flight | gen-1 s |
|---|---|---|---|---|
| 041-t7 | 333 | 4408 | (dmp unreadable, old schema) | 102 |
| 043-t2 | 310 | 4726 | 438–445 | 107 |
| 043-t3 | 465 | 3163 | 445 | 112 |
| 043-t5 | 577 | 2548 | 570 | 110 |
| **043-t6** | 505 @180, rising | 2900 | 570 | 108 |

- **Within a run, gen time tracks the population average**: corr(dur, pop-avg) = −0.87 (t5), −0.91 (t6), −0.89 (041-t7); the mechanism
  is flight length — a flight ends at a crash/egress or at path completion, so as more of the 5000 fly the whole course the tick count
  per generation rises until everyone completes. Gen-1 costs are identical across all five runs (102–112 s): per-sim overhead has not
  changed, it is all per-tick.
- **t3 → t5 (+24 %) is the path swap**: FortyFive → randomA took the elite's flight from 445 to 570 ticks (+28 %). Fully explained.
- **t2 → t3 (+50 %) is NOT flight length** — same courses, same 445 ticks/flight, same arena (70/25/105 in both inis) — it is a
  **+47 % per-tick cost** (8.1 → 11.9 µs core-time per 20 Hz tick). The change set between the two bakes that runs per tick: the INAV
  rate loop moved from once per 50 ms frame (`crrc_fdm.cpp`) into the 5 ms physics loop (`fdm_larcsim.cpp`, 10× the calls, two `exp()`
  each — physically right, cost unmeasured) and per-tick `stepScore`/`envelopeSecs` added to every sim's `EvalResults`. Unresolved;
  a step-test A/B with and without the `<controllers>` node settles it in minutes once the box is free.
- **t5 → t6**: same elite ticks (570); t6 is 12 % slower than t5 *at the same gen* because its population flies longer sooner
  (pop-avg +45 %). Expect the plateau at or just above t5's 577 s.
- **Fixed overheads measured live (1 s CPU samples over gen 182)**: the parent runs at **161 % of a core for the whole eval phase**
  (deserialising ~1.7 G `EvalTick`s/gen and scoring parent-side), the 20 workers get **84 % of a core each** on a box that is 96 % busy
  (my analytics took another ~0.8 core), and there is a **25 s single-threaded parent phase between generations** (selection ~1 s;
  the rest is dmp serialise/zstd/S3 + data.dat). So ~15 % of the generation is contention and ~5 % is serial — recoverable by moving
  scoring to the workers (the standing backlog item) and overlapping the dmp write with the next dispatch; neither touches results.
  Analytics passes cost ~2 % (gens 94–97 during a 25-min cache build: 473–488 s vs 464–475 around them).

### 5e. Gen 275 read (2026-10-15 morning): off the rail, but the cycle has plateaued at g488's level — the first warning

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete | t6 s/gen | t5 s/gen |
|---|---|---|---|---|---|---|---|---|---|
| 200 | −14457 | −20000 | 32.8 | 33.4 | 5.95 | 5.54 | 294 | 512 | 471 |
| 220 | −17216 | −21420 | 38.6 | 35.5 | 6.14 | 5.78 | 294 | 519 | 500 |
| 240 | −17747 | −22023 | 40.4 | 39.5 | 5.10 | 5.17 | 294 | 521 | 480 |
| 260 | −19759 | −23343 | 40.8 | 38.3 | 5.16 | 5.59 | 294 | 530 | 498 |
| 275 | −20863 | **−23931** | **43.1** | 40.7 | 5.37 | 5.13 | 294 | 539 | 492 |

Tracking: pop-avg still ahead of t5 (+15 %, the gap closing as t5's own curve steepened here), `pctInStreak` 2.4 pts **behind** t5 at the same
gen, score/step 0.536 vs t5 ~0.57, crash-free elite (0/294, 18 crash-carrying elites in 181–275 vs 50 in the previous 80), 13 elite
replacements, bite 92 %, 0 DIVERGED. The flat-path / random-course split is the same as t5's (StraightAndLevel 0.78, SeededRandomB 0.21).

**Relay panel, gen 180 → 275 (t5 at 271 in brackets):**

| | 180 | 201 | 231 | 251 | **275** | t5 @271 |
|---|---|---|---|---|---|---|
| pitch-rate RMS °/s | 108 | 106 | 108 | 110 | **101** | 115 (stiff plant) |
| pitch / roll / throttle at cap % | 13.5 / 18 / 45 | 8 / 12 / 43 | 4.5 / 2.5 / 38 | 7 / 7.5 / 37 | **9 / 7 / 49** | 3.5 / 0.3 / 44 |
| pitch / roll flips/s | 4.9 / 7.9 | 5.7 / 8.1 | 5.9 / 5.1 | 5.3 / 6.4 | **7.3 / 8.1** | 4.8 / 5.5 |
| mean \|pitch cmd\| (×240 °/s) | 0.54 | 0.54 | 0.55 | 0.54 | **0.56** | 0.50 |
| 2–3 Hz / 3–5 Hz share % | 16 / 25 | 24 / 19 | 27 / 22 | 22 / 23 | **26 / 17** | 4 / 27 |

Per path at 275 (`sim_pitch_spectrum`): RMS 78 (StraightAndLevel) → 111 (SeededRandomB), cap 5–11 %, flips 6.4–8.0/s, lag 2 ticks,
peak 2.0–2.2 Hz on the two flat courses.

**Reading, plainly.** The *amplitude* relay is gone — pitch and roll at the cap are single digits, where t5 was at the same gen. But the
*cycle* is not: elite pitch-rate RMS has sat at **100–110 °/s for 75 generations** (gens 200–275), which is g488's own level on this
plant (105–123), the mean pitch command has not moved (0.54–0.56 of the cap, t5 0.50), the sign-flip rate is **rising** (4.9 → 7.3/s;
t5 4.8), and the 2–3 Hz share is at 26 % against the flight band's 31–63 %. What evolution found is a smaller, faster dither rather than
a slower hand — it took the setpoint off the rail (which the tie-break and the crash price both see) and kept the reversal rate (which
nothing prices directly; excess rotation is a magnitude integral, and a ±130 °/s chatter at 7 Hz integrates to little). This is the § 5
gen ≥ 100 "stop-and-rethink" row half-triggered: RMS stuck at g488's level. Half, because t5's own reference curve kept falling from 115
to 85 over gens 300–800 on its plant, so there may be slope to come, and the easy courses are already below g488 (78 vs ~105). **Call at
gen ~350**: if the elite's pitch RMS is still ≥ 100 and flips/s have not turned down, the measure is next — a reversal-rate or
command-slew case (T103/T104, parked in BACKLOG "only after the plant is honest" — the plant is now honest) — not more generations.

Generation time 539 s at 275 (t5 492 at the same gen, plateau 577); the per-gen record is in § 5d.

**Why the chatter survives (same day, T120 → [policy-temporal-view.md](../policy-temporal-view.md))**: the input vector has no history
of the craft's own rotation — only the target bearing at 800/400/200/100/50/0 ms — and the 1.5–4 Hz pitch command is computed from
that bearing and its 50/100 ms differences, in flight exactly as in sim (gyro contributes ΔR² 0.003). The policy is a P+short-lead
attitude loop through a 100–150 ms rate lag; it limit-cycles where its gain crosses 1 and has only amplitude to give back. The
gen-275 elite's recurrent state has become a damper (zeroing it doubles the band power; in g488 it *carried* the cycle). The next
bake's lever is T121 (own-rotation history slots), not the measure.

### 5f. Gen 370 read (2026-10-15 evening): the cycle is coming down, and t6 now beats the flown genome on equal footing

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete | t6 s/gen | t5 s/gen |
|---|---|---|---|---|---|---|---|---|---|
| 300 | −23185 | −26089 | 45.1 | 41.0 | 5.82 | 5.75 | 294 | 544 | 508 |
| 325 | −25643 | −24625 | 43.2 | 39.9 | 5.52 | 5.00 | 294 | 529 | 527 |
| 350 | −26911 | −26545 | 47.6 | 40.5 | 4.95 | 5.92 | 294 | 534 | 527 |
| 370 | −27270 | **−28405** | 47.6 | **43.1** | 4.96 | **4.12** | 294 | 549 | 522 |

16 elite replacements in 276–370, 11 crash-carrying elites (0/294 at 370), bite 92 %, 0 DIVERGED. Score/step 0.565 vs t5's ~0.645 at
the same gen; `rotRatio` now **below** t5's (4.1 vs 5.0).

**Relay panel (elite medians over 294 scenarios; t5 @371 for reference, on its stiff plant):**

| | 275 | 311 | 346 | **370** | t5 @371 |
|---|---|---|---|---|---|
| pitch-rate RMS °/s | 101 | 87 | 104 | **80** | 88 |
| pitch / roll / throttle at cap % | 9 / 7 / 49 | 2.4 / 3.0 / 30 | 1.8 / 3.8 / 30 | **5 / 1 / 26** | 2.0 / 2.2 / 26 |
| pitch / roll flips/s | 7.3 / 8.1 | 5.0 / 5.4 | 6.9 / 9.0 | **7.3 / 7.1** | 3.1 / 5.5 |
| 2–3 Hz / 3–5 Hz share % | 26 / 17 | 13 / 16 | 20 / 21 | **17 / 8** | 13 / 28 |

Per path at 370 (`sim_pitch_spectrum`): RMS 54–89 °/s, mean \|cmd\| 87–101 °/s, cap 4–6 %, 2–3 Hz share 15–22 %, 3–5 Hz 7–11 %,
spectral peak at manoeuvre frequencies (0.2–0.6 Hz) — the cycle is no longer the dominant pitch motion.

**Equal footing with the flown genome** (both flying the t6 plant, which reproduces g488's flights): g488 = RMS 105–123 °/s,
`pctInStreak` 40 %; **t6 gen 370 = RMS 54–89 °/s (≈ −30 %), `pctInStreak` 43.1 %**. The first 043 genome that out-tracks *and*
out-smooths the flown baseline on the plant that matches the air. What remains: command sign flips 5–9/s (t5 3/s) — small-amplitude
dither around zero that the airframe filters (3–5 Hz share 8 %) — and a tracking gap to t5's stiff-plant numbers (−4.5 pts).

**The gen-350 call (§ 5e).** At 350 the elite read 104 °/s with 6.9 flips/s — by the letter, the stop-and-rethink row. At 366 a new
elite dropped to 80 °/s with tracking up. Decision: **the run continues unchanged to 800**; the RMS is still falling on t5's schedule,
and the next lever (T121 craft-sensor history) belongs to the next bake, not to a mid-run change. The t6 elite from here on is a
**T118 flight candidate** against g488 whenever the operator chooses to fly.

**Operator decision 2026-10-15 (gen 379): keep t6 running; no stop for T121.** A flight test is targeted ~2.5 days out, so it flies a
**t6 genome on the current 45-input firmware**, not the T121 slots. At 551 s/gen the run reaches gen 800 in ~2.7 days; leaving
~8 h for extraction, eval suite, nn2cpp, xiao build and bench check puts the flight snapshot at **≈ gen 708–728** (or the
final elite if the flight slips). T121 (craft-sensor history) is built after the flight, on its own branch, with the filter-delay model first.

### 5g. Gen 435 read (2026-10-16 morning): pitch control has plateaued; selection is trading smoothness for tracking

| gen | t5 avg | **t6 avg** | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 elite complete | t6 s/gen |
|---|---|---|---|---|---|---|---|---|
| 370 | −27270 | −28405 | 47.6 | 43.1 | 4.96 | 4.12 | 294 | 549 |
| 390 | −28801 | −29919 | 48.0 | 43.7 | 4.55 | 4.87 | 294 | 541 |
| 410 | −31427 | −30508 | 47.5 | 43.4 | 4.74 | 4.76 | 293 | 554 |
| 435 | −33671 | **−31631** | 48.3 | **43.4** | 5.29 | 4.96 | 294 | 549 |

**Relay panel, elite medians:** after the gen 366–375 elite (80 °/s, the smoothest of the run) every elite has sat at **87–96 °/s**;
pitch at the cap 0.7–2.3 %, throttle at the cap back to 33–40 %, pitch flips 6.4–7.6/s (unchanged), 2–3 Hz share 14–29 %, 3–5 Hz 11–18 %.
t5 over the same gens: 84–102 °/s, flips 3.1–3.8/s.

| elite (t6 plant) | pitch RMS by path | 3–5 Hz share | flips/s | score/step | `pctInStreak` | fitness | genome sha |
|---|---|---|---|---|---|---|---|
| g488 (flown) | 105–123 | — | ~3 | — | 40 % | −65928 (eval) | `1731d937e3af9c1c` |
| **t6 gen 370** | **54–89** | **7–11 %** | 5.2–8.9 | 0.565 | 43.1 % | −81227 | `6108e4f667efc11c` |
| t6 gen 435 | 62–107 | 15–22 % | 5.8–8.5 | **0.597** | 43.4 % | **−86210** | `4190c5302dc75552` |

Reading: tracking per step is still creeping up (0.565 → 0.597, crash-free) while `pctInStreak` is flat and the pitch RMS has given back
~10 °/s with `rotRatio` 4.1 → 5.0 — the newer elites buy tracking with a little more motion, which is what the lexicase cases reward; the
tie-break only ranks equal trackers. Still ~20 % smoother than g488 on the same plant (median ~86 vs ~112 °/s), but **no longer improving**.
⚠️ **Flight-candidate consequence:** the best fitness is not the smoothest genome. Both genomes are archived (git-ignored `.dat` in this
directory). Before the T118 flight, run the eval suite on the gen-370 elite, the flight-window elite (≈ gen 710–730) and the final one, and
choose on pitch smoothness at matched tracking, not on fitness.

### 5h. Gen 523 read (2026-10-16 evening): through ramp step 13 (variation scale 12/19 → 13/19 = 0.632 → 0.684, effective gen 521) — a calmer elite takes over

| gen | t6 best | t6 avg | t6 pct | t6 rot | | t5 best | t5 pct |
|---|---|---|---|---|---|---|---|
| 480 | −88909 | −33593 | 45.7 | 5.08 | | −98011 | 49.1 |
| 500–520 (one elite) | −90692 | −34411 … −35442 | **46.0** | 4.69 | | −97542 … −99998 | 48.7 … 49.5 |
| 521 (same elite, harder air) | −87056 | −34418 | 45.0 | 4.78 | | −98970 | 49.2 |
| 522–523 (**new elite**) | −87488 | −34447 | 43.4 | 4.62 | | −98970 | 49.2 |

Before the step `pctInStreak` had climbed 43.4 → 46.0 (gap to t5 3.5 pts). The step cost the standing elite 4 % fitness and 1 pt of streak
(t5's step: 1 % and 0.3 pt) and changed the winner on the next generation. All elites 294/294, 0 DIVERGED, ~550 s/gen.

| elite (t6 plant) | pitch RMS by path | median: flips/s | 2–3 / 3–5 Hz % | mean \|cmd\| °/s | roll flips/s | streak | genome sha |
|---|---|---|---|---|---|---|---|
| g488 (flown) | 105–123 | ~3 | — | 115–121 | — | 40 % | `1731d937e3af9c1c` |
| gen 370 | 54–89 | 7.3 | 17 / 8 | 87–101 | 7.1 | 43.1 % | `6108e4f667efc11c` |
| gen 435 | 62–107 | 6.9 | 20 / 17 | 98–115 | 7.6 | 43.4 % | `4190c5302dc75552` |
| gen 500 (to 521) | 60–100 | 6.0 | 16 / 8.5 | 80–103 | 8.5 | **46.0 %** | `e537584e26b18b9a` |
| **gen 522–523** | **56–98** | **4.8** | **11 / 10** | **73–92** | 7.2 | 43.4 % | `4b330b61827dd09b` |

Reading: the step re-ranked the population toward a **calmer** pitch controller — the lowest command flip rate since gen ~100, the lowest
mean command and 2–3 Hz share of the run — at the same pitch-rate RMS and 2.6 pts less streak. Harsher air makes chatter cost more on
this plant, so the ramp itself is pushing toward smoothness. The flight shortlist is now **gen 370, gen 500, gen 523 and the flight-window
elite**; all four genomes except the last are archived here (git-ignored `.dat`). Choose on pitch smoothness at matched tracking.

### 5i. The windiest arena on each path (2026-10-16, gen 525)

Wind speed / turbulence / gust-length / thermal are per-scenario draws made **in the worker** from the scenario's wind stream and are not
logged; the log's scenario table carries the **direction** draw (`WindDir°` = full-strength offset from the 330° base — the dmp's
`wind_dir_offset_deg` is that × the current ramp) but its `Speed%` column is the **entry airspeed** variation (±15 %, r = −0.07 with the
wind draw). Exact draws reproduced with [`wind_draws.cc`](../wind_draws.cc) (it matches the dmp's wind sub-seeds) and ranked with
[`windiest_scenarios.py --draws`](../windiest_scenarios.py); the realized-wind ranking from the elite's own ticks picks the same arena on
every path. Ramp: s = 13/19 = 0.684 now (wind flown = 3.66 + s·(draw − 3.66) m/s), steps at gens 561, 601, 641, 681, 721, **full strength
from gen 761**.

| path | arena | wind # | speed draw (full) | flown now | from, full strength | turbulence × | gust length × | elite streak (path median) | pitch RMS (path median) |
|---|---|---|---|---|---|---|---|---|---|
| StraightAndLevel | **46** | 46 | 7.98 m/s | 6.61 | 356° | 0.94 | 1.00 | 42 % (65 %) | 91 (56) °/s |
| SpiralClimb | **64** | 15 | 7.78 | 6.48 | 100° | 1.77 | 0.29 | 58 % (68 %) | 74 (67) |
| HorizFigure8 | **120** | 22 | 7.98 | 6.62 | 22° | 1.65 | 0.99 | 44 % (48 %) | 99 (82) |
| randomA | **150** | 3 | 8.00 | 6.63 | 51° | 0.52 | 0.44 | 38 % (36 %) | 98 (93) |
| HighPerchSplitS | **229** | 33 | 7.98 | 6.61 | 22° | 0.89 | 0.92 | 48 % (53 %) | 110 (86) |
| SeededRandomB | **281** | 36 | 7.79 | 6.48 | 120° | 1.16 | 0.90 | 14 % (27 %) | 98 (98) |

The gen-525 elite completes all six; the windiest arena costs it 0–23 pts of streak against the path median (worst on StraightAndLevel
and SeededRandomB) and adds 0–35 °/s of pitch-rate RMS (worst on StraightAndLevel and HighPerchSplitS). Smaller gust length = gusts
that change faster within the run (SpiralClimb arena 64 has the fastest and the strongest turbulence of the six).

### 5j. Gen 621 read (2026-10-17 midday): tracking keeps climbing, pitch smoothness holds its plateau

| gen | t5 avg | t6 avg | t5 pct | **t6 pct** | t5 rot | t6 rot | t6 best | t6 elite complete |
|---|---|---|---|---|---|---|---|---|
| 525 | −37481 | −34855 | 49.2 | 43.4 | 4.97 | 4.62 | −87488 | 294 |
| 560 | −39996 | −36771 | 49.6 | 45.8 | 4.32 | 4.87 | −92546 | 294 |
| 561 (step 14) | −39377 | −35210 | 49.0 | 45.0 | 4.37 | 4.92 | −89965 | 294 |
| 600 | −41653 | −39171 | 50.6 | **47.8** | 4.85 | 4.90 | −93831 | 294 |
| 601 (step 15) | −40153 | −37244 | 46.9 | 46.4 | 5.06 | 4.97 | −88712 | 294 |
| 621 | −40834 | −38578 | 49.6 | **46.6** | 4.51 | 5.02 | −92414 | 294 |

Streak gap to t5 down to ~3 pts; **no crash-carrying elite since gen 525**; each ramp step costs ~1 pt for a few gens. 560 s/gen: gen 715
≈ 2026-10-18 01:45, gen 800 ≈ 2026-10-18 15:00.

**Relay panel, gens 521–621 (elite medians):** pitch-rate RMS 84–98 °/s, pitch flips 4.8–6.4/s, 2–3 Hz share 11–21 %, 3–5 Hz 8–20 %,
mean \|cmd\| 0.36–0.48 of the cap, throttle at the cap 31–45 %. The calmest elite of the stretch is still gen 522–526 (4.8 flips/s, 11 %);
the current one (since gen 613, `pctInStreak` 46.6 %, genome `5525f3a75ad06c62`) is back at 6.1 flips/s, 21 % at 2–3 Hz, RMS 75–103 by path.
t5 over the same gens calmed to ~2 flips/s on its plant; t6 has not shown that late calming yet. Against g488 on the same plant it
remains ~20 % smoother (median ~87 vs ~112 °/s by path) with 6–7 pts more streak. Flight shortlist: gen 370 / 500 / 522 / 621 + the
snapshot-window elite — still to be chosen on smoothness at matched tracking (T122).

### 5k. T122 — the flight candidates in ONE environment (2026-10-18): gen 752 wins on both axes; the predicted flight

**Method.** Eval tier 0 with one never-trained seed (20261018) for every candidate, on weights copies whose stored `variation_scale` is
forced to **1.0** ([`set_eval_scale.py`](../set_eval_scale.py)). Eval otherwise flies each genome at its own save-time ramp value —
g370 0.47, g500 0.63, g523 0.68, g621 0.79, g752 0.95, and the flown **g488 0.63** — so the cross-generation numbers in §§ 5f–5j and the
gen-488 honesty check were *not* on identical air (the operator's point: fitness and training dmps do not compare generations). Metrics
by [`candidate_compare.py`](../candidate_compare.py); the same code run on the 10-11 xiao logs calibrates sim → flight through g488.

| candidate (full strength, 294 scenarios) | pitch RMS °/s | 2–3 Hz % | 3–5 Hz % | flips/s | cap % | cmd p5 / p95 °/s | \|a_z\| p99 g | streak % | crashes |
|---|---|---|---|---|---|---|---|---|---|
| t5 g488 (flown) | 117 | 29 | 23 | 2.8 | 1.5 | −133 / 197 | 6.25 | 33.4 | 1 |
| t6 g370 | 87 | 19 | 10 | 6.3 | 6.3 | −176 / 208 | 5.78 | 36.0 | 1 |
| t6 g500 | 93 | 17 | 9 | 5.6 | 0.6 | −151 / 159 | 5.93 | 40.3 | 1 |
| t6 g523 | 89 | 11 | 11 | 4.7 | 0.9 | −137 / 149 | 5.94 | 39.8 | **6** |
| t6 g621 | 95 | 20 | 12 | 6.1 | 0.8 | −146 / 162 | 6.29 | 42.5 | 1 |
| **t6 g752** | **78** | **13** | **5** | **4.6** | **0.4** | **−99 / 142** | **5.57** | **46.2** | 1 |

Calm subset (exact wind draw ≤ 3 m/s, n = 106, like the 10-11 days): g752 RMS 70 / 2–3 Hz 10 % / \|a_z\| p99 4.92 g / streak 56.1 % vs
g488 112 / 30 % / 5.40 g / 39.3 %. Windy (≥ 6 m/s, n = 78): g752 87 / 15 % / 6.18 g / 35.1 % vs g488 125 / 27 % / 7.16 g / 25.0 %.

**The generalization question.** The latest elite is better *in equivalent air* on every pitch measure and on tracking — not
over-specialized, not peaked early. The elites that looked smooth earlier were selected in milder air: gen 523 (the calmest at its own
0.68) leaves the arena 6× at full strength, and gen 370 rides the cap 6.3 % with the widest command spread. Every single crash here is
an arena exit (OOB), in a different scenario per genome.

**Predicted flight for g752** (flight(g488) × sim(g752) / sim(g488), calm subset as the field-day proxy; g488 flew 146 °/s, 49 %,
2.8 flips/s, 6.6 g): pitch-rate RMS **≈ 91 °/s**, 2–3 Hz share **≈ 17 %**, flips ≈ 4.8/s, \|a_z\| p99 ≈ 6.0 g; command p5 ≈ −165 °/s
(g488 flew −227 … −239, on the cap) and p95 ≈ 140 (198). Against T118: pitch RMS ✓ well under 124–153; 2–3 Hz ✓ (within 1.5× of MANUAL's
3–16 %); off the cap ✓; tracking ✓ (calm streak +17 pts in sim); **\|a_z\| p99 < 5 g ✗ likely** (≈ 6.0, g488 6.6). Residual risk: the
higher command flip rate (small-amplitude dither the airframe filters). Caveats: one-genome calibration with a proportional mapping;
flight wind, streamer state and the unmodelled FC sensor-filter delays differ.

**Pick (T122): the gen-752 elite** — created at gen 750, genome `7faeea2d022cbb95`, archived as `nn_weights-t6-g752.dat` (operator
2026-10-18: *"we'll grab this generation and try it"*). Fly the archived original, not the eval copy.

## 6. Post-run (closed 2026-10-18)

| field | value |
|---|---|
| finished | gen 800, 2026-10-18 15:26 PDT, 410,854 s (4.76 days), 1.176 G sims |
| gen 800 | best −93924.534457, pop-avg −37593, `pctInStreak` 46.7 %, `rotRatio` 4.78, 294/294 complete |
| final elite | `nn_weights-t6-g800.dat`, sha `809fe847aca4af5a` |
| flown elite | gen 752 (created 750), `7faeea2d022cbb95` — the 043 exit flight, 2026-10-18 ([analysis](../flight-analysis-20261018.md)) |
| `retain=keep` | ✅ all 800 objects tagged, 0 failures; verified by sampling 9 incl. gen 1, gen 800 and the flown gen 752 (`gen9248`) |
| eval suite (seed 1791951530) on gen 800 | tier0 **PASS bitwise** (−93924.534457 = training); tier1 294/294; tier2-progressive 48/49; tier2-long 49/49; tier2-random 144/144; tier3-stress 140/144; tier3-quiet 1/1 — all PASS |

Verdict on the run: on the honest pitch plant t6 out-tracked and out-smoothed the flown t5 genome, and its gen-752 elite halved the 2 Hz
pitch cycle in the air (T118 partly met: pitch ✓, load ✗, roll regressed — the sim roll mode is ~4.5× too slow, flight-analysis § 3).
