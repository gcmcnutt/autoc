# 043-t5 — `autoc-043-t5-m1-condrot` — MANIFEST (written BEFORE launch, 2026-10-08)

Pattern of record: t3's `artifacts-t3/MANIFEST.md`, t4's `artifacts-t4/MANIFEST.md` (read its § 6b first — t5 exists
because of it). Handoff: [`NEXT-T5-RUN.md`](../NEXT-T5-RUN.md). Fields marked ⏳ are filled at launch / at the end.

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370245346926812-2026-10-09T01:04:08.995Z/` |
| Retention | uploads as `retain=expire` — **pin `retain=keep` on all objects at the end and verify by sampling** (t4: 413/413, 22 sampled) |
| Master seed | **1791507848** (`Seed = -1`; logged as "Effective master seed" — the only record of it) |
| Regiment | pop **5000** × **800** generations × **294** scenarios (6 paths × 49 winds; `ExpectedScenarioCount = 294`) — **identical to t4** |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-043-t5-m1-condrot.log` (Xvfb `:2` must be up — it does not autostart) |
| Launched | **2026-10-08 18:04 PDT**, pid 838074, detached via `scripts/train.sh`; startup log confirms `ExcessRotationAxisMode: conditional`, `AeroStandardPath3: randomA`, 20 workers, no errors |
| Variation ramp | `VariationRampStep = 40` ⇒ env classes (incl. the wind envelope) at scale 0 through gen 40, 1.0 from gen 761 — identical to t4 |
| Completed / stopped | ⏳ |

## 2. Commits

| | |
|---|---|
| autoc | `cd6fc00` — *feat(043 t5): conditional excess-rotation case + selection telemetry; t4 stopped and closed out at gen 413*. Tree clean. |
| crrcsim | `c4b3e49` — unchanged since t4 (pointer unmoved, submodule clean). |
| `autoc.ini` | as-run snapshot [`autoc.ini.as-run`](autoc.ini.as-run), sha256 `74576e1c0c69d5fa…` |
| build | clean `scripts/rebuild-perf.sh` (operator, 2026-10-08 17:45), `PERFORMANCE_BUILD=ON`; ctest 53/53 binaries (`selection_tests` 21/21, `contract_config_tests` 12/12); **t3 gate `NN_EVAL_SAME −88026.619367`** on this binary with the axis off (the `filter()` refactor is PRNG-free) |

## 3. What changed vs 043-t4 — ONE key, nothing else

| | t4 | **t5** |
|---|---|---|
| **lexicase pool per scenario** | `score`, `energy`, `excess_rotation` (three co-equal; tracking 1/3 of starts) | **`score ⊕ excess_rotation`, `energy`** — rotation is a tie-break INSIDE each score case (within-MAD on score, then on rotation among those); tracking back to 1/2 of starts |
| ini | `EnableExcessRotationAxis = 1` | + **`ExcessRotationAxisMode = conditional`** (required when the axis is on; forbidden when off) |
| telemetry | — | **`#GenSel`** per gen: `selections casesPerSel tbApplied tbPerSel tbBitePct tbMeanIn tbMeanOut` |
| wind envelope, courses (`randomA` slot 3), crash price, ramp, pop, gens, the measure itself | | **unchanged** |

Why (t4 MANIFEST § 6b, first-principles § 10): `path_rotation` is identical for every genome on a scenario, so a
standalone rotation case is won by whoever rotates least — a passive flyer — and t4's population never converged on
tracking (pop-avg a third of t3's at gen 40 under identical wind; elite frozen from gen 281). Conditional lets
smoothness order only genomes that already track equally well. No weight, no threshold, no epsilon changed.

## 4. ⛔ What is and is not comparable

- **Scalar fitness is not comparable to t3 or t4** (regiment and objective both differ from t3; the pool differs from t4).
- **Comparable to t3 at matched gens**: per-path `score/step`, `pctInStreak`, crashes, and `rotRatio` — t3's full
  post-hoc trajectory is cached from its pinned dmps and drawn dashed on `tracking_by_path` (`--ref`).
- **Comparable to t4 at matched gens**: the same per-path panel (t4's cache `…_bypath_v2.csv`), and the
  population average in `#NNGen avg=` — the number that exposed t4 by gen 40.
- **Not available from any pinned run**: `#GenSel`. It is the mechanism check, not a comparison.
- Wind-binned views (`tracking_by_wind`) are degenerate until the ramp is well past gen 80 (realized spread < 1 m/s).

## 5. Acceptance (tasks T106, re-based 2026-10-08) and the early calls

| when | check | healthy | stop-and-rethink |
|---|---|---|---|
| gen 2–5 | `#GenSel` | tie-breaks on most selections, `tbMeanIn` ≫ 1, `tbBitePct` high | `tbBitePct ≈ 0` or `tbMeanIn ≈ 1` — the tie-break is inert |
| gen 40 (ramp 0, wind = t3's) | `#NNGen avg=`, `pctInStreak` | near t3's −4.2k / 5% | at t4's −1.4k / 1.6% |
| gen 100 / 183 | avg, pctInStreak | t3: −9.2k / 20%; −14.6k / 33.7% | t4: −3.2k / 10%; −3.6k / 25.5% |
| ≥ gen 150 | `rotRatio` vs t3's dashed curve | **below** it at matched gens (t3: 8.0 @150, 6.3 @250, 5.85 @396, **4.55 @800**) | merely on it = pressure real but not enough → demand side next, not a knob |
| any 40-gen ramp window | elite replaced? | yes (t3: every ≤ 26 gens) | frozen a full window = the t4 stall signature |
| end | T106 | straight line no longer the worst path; calm-bin `pctInStreak` and per-path lines within noise of t3; crashes ≤ t3; `rotRatio` clearly under 4.55× | |

### 5a. ✅ Gen 2–5 call (2026-10-08 18:15 PDT): the tie-break has leverage

| gen | selections | casesPerSel | tbPerSel | **tbMeanIn** | tbMeanOut | **tbBitePct** |
|---|---|---|---|---|---|---|
| 1 | 8929 | 2.44 | 0.66 | 8.2 | 1.54 | 87.4 |
| 2 | 8876 | 2.19 | 0.50 | 35.4 | 1.31 | 95.5 |
| 3 | 8889 | 2.18 | 0.49 | 75.1 | 1.33 | 96.3 |
| 4 | 8900 | 2.14 | 0.53 | 63.4 | 1.21 | 98.7 |
| 5 | 8951 | 2.16 | 0.47 | 59.0 | 1.26 | 95.9 |

Reading: a selection event resolves in ~2 cases; **half of them run a tie-break, which chooses among 35–75
genomes that track the scenario equally well and removes all but ~1.3 of them 96–99 % of the time.** That is the
opposite of 035's "tie-break effectively absent" (constant epsilon, ~1 survivor): under MAD epsilon the
equal-tracker set is large and the rotation pressure inside it is near-total. Whether it is *enough* is the slope
question (§ 5, ≥ gen 150). Population average gens 1–5: t5 −1085 → −1145, t4 −1092 → −1129 (same regiment,
same shape so far — gen 1 is random init, so t5-vs-t4 is the fair pair; t3's −1372 → −1462 is a different
regiment at gen 1). **Next call: gen 40** (t3 −4.2k / 5 %; t4 −1.4k / 1.6 %). Determinism: `NN_ELITE_SAME` 5/5.

### 5b. ✅ Gen 40 call (2026-10-08 19:40 PDT): the population converges on tracking again

| gen | t3 avg | t4 avg | **t5 avg** | t3 pct | t4 pct | **t5 pct** |
|---|---|---|---|---|---|---|
| 20 | −1858 | −1115 | **−1603** | 1.1 | 0.7 | **1.7** |
| 30 | −2828 | −1168 | **−2937** | 2.9 | 0.9 | **3.2** |
| 40 | −4163 | −1391 | **−4292** | 5.0 | 1.6 | **5.6** |
| 49 | −5374 | −1634 | **−5034** | 8.0 | 2.9 | **8.3** |

t5's population average is on t3's trajectory (−4292 vs −4163 at gen 40) and **3× t4's** on the identical regiment —
the dilution is gone; tracking is being selected for. Elite crash-free by gen 40 (294/294; t3 needed ~gen 60, t4
~gen 60). `rotRatio` 9.4× at gen 49 ≈ t3's 9.3× (gen 46) and t4's 9.6× — no separation yet, as expected while
the elites are still crash-dominated; the slope question opens at ~gen 150. Tie-break still live: `tbBitePct`
88–95 %, `tbMeanIn` 17–41 (the equal-tracker set is narrowing as the population spreads — healthy).

### 5c. Gen 134 read (2026-10-09 06:30 PDT): first separation on `rotRatio`, tracking level with t3

| gen | t3 avg | t5 avg | t3 pct | t5 pct | t3 rot (post-hoc) | t4 rot | **t5 rot** |
|---|---|---|---|---|---|---|---|
| 75 | −7882 | −7702 | 10.5 | 13.2 | 8.60 | 9.51 | **7.78** |
| 100 | −9198 | −9391 | 17.1 | 22.0 | 8.88 | 9.73 | **6.46** |
| 134 | −11676 | −9863 | 26.0 | 27.2 | 7.85 | 9.15 | **6.14** |

Per path at 134: Straight 0.51 / 48 % / 7.4×, Spiral 0.43 / 43 % / 6.2×, HighPerch 0.37 / 34 % / 7.4×, Fig-8 0.31 / 27 % / 4.9×,
randomA 0.20 / 20 % / 6.1×, RandomB 0.16 / 14 % / 5.0×; 0/294 crashes. Crash-gens 60–134: t5 **25** vs t3 32 / t4 29; longest
elite stall 16 gens; `NN_ELITE_SAME` 134/134; `tbBitePct` 85–92 %, `tbMeanIn` 8–17. Per-axis aggressiveness (elite, 294 scenarios):
pitch ⟨|Δ|⟩ 0.326 / ⟨|out|⟩ 0.653, roll 0.310 / 0.333, throttle 0.256 / 0.800 — baseline for the later gens.

Read: the regiment `rotRatio` is **below t3's dashed curve by ~1.7× at gen 134** (6.14 vs 7.85) with `pctInStreak` level and crashes
cleaner — the first sign the tie-break buys smoothness without costing tracking. ⚠️ Elite-level numbers jump with each new elite
(5.05 at gens 110–120, 6.14 from 121); the slope read proper is from gen 150 on, and the straight line is still the worst path.

### 5d. Gen 215 read (2026-10-09 ~16:00 PDT): tracking ahead of t3, `rotRatio` just under t3's curve — margin narrowed

| gen | t3 avg | **t5 avg** | t3 pct | t4 pct | **t5 pct** | t3 rot | t4 rot | **t5 rot** |
|---|---|---|---|---|---|---|---|---|
| 150 | −12607 | −10937 | 29.7 | 17.9 | 31.9 | 7.63 | 9.24 | **6.21** |
| 175 | −14653 | −12111 | 33.7 | 24.6 | 31.8 | 6.94 | 8.29 | **6.76** |
| 200 | −14232 | −14457 | 36.1 | 26.8 | 32.8 | 6.15 | 8.02 | **5.95** |
| 215 | −14917 | **−15979** | 35.7 | 28.1 | **37.2** | 6.15 | 7.56 | **5.89** |

Per path at 215 (score/step, pct, rot): Straight 0.80 / 62 % / **6.1×**, Spiral 0.66 / 60 % / 6.3×, HighPerch 0.47 / 46 % / **7.4× (now the worst)**,
Fig-8 0.49 / 47 % / 5.0×, randomA 0.25 / 26 % / 6.0×, RandomB 0.19 / 19 % / 5.5×; 0/294 crashes. Crash-gens 135–215: t5 25 = t3 25 (t4 11).
Longest stall 16; `NN_ELITE_SAME` 215/215; `tbBitePct` 86–93 %, `tbMeanIn` 6–10. Wall clock 480 s/gen vs t3 415 s at the same gen
(+16 % — longer flights plus the tie-break pass; not a concern). Wind (ramp 5/18, realized 2.7–4.8 m/s): 2–4 → 4–6 bin costs
0.52 → 0.37 score/step and 41.5 → 29.7 % `pctInStreak`; `rotRatio` flat across bins (5.9 / 5.9).

Honest read: **tracking is level-to-ahead of t3** on every measure, and the **straight line is no longer the worst path** (T106
item: 6.1× vs HighPerch 7.4×). But the `rotRatio` **margin over t3's curve narrowed** from ~1.7× at gen 134 to 0.2–0.4× here —
t3's own curve dropped fast through 150–200 (7.63 → 6.15) and t5 got there earlier rather than lower. Still below at every
checkpoint; the bar at 800 is 4.55×. If t5 merely tracks t3's curve from here, the tie-break is real (bite ~90 %) but its
*marginal* smoothing is small — the demand-side question (§ 9 item 4 of the first-principles doc), not a knob.

### 5e. Gen 330 read (2026-10-10 ~09:00 PDT): tracking ahead of t3; `rotRatio` ON t3's curve, not under it

| gen | t3 avg | **t5 avg** | t3 best | **t5 best** | t3 pct | t4 pct | **t5 pct** | t3 rot | t4 rot | **t5 rot** |
|---|---|---|---|---|---|---|---|---|---|---|
| 250 | −17214 | −18034 | −57778 | −75217 | 42.0 | 31.9 | 41.2 | 6.50 | 7.33 | **5.40** |
| 275 | −20418 | −20863 | −65798 | −79694 | 43.6 | 33.3 | 43.1 | 5.97 | 7.17 | **5.37** |
| 300 | −21012 | −23185 | −66563 | −81916 | 44.4 | 32.6 | 45.1 | 5.94 | 7.22 | **5.82** |
| 330 | −22570 | **−25458** | −68460 | **−86511** | 46.5 | 32.0 | 46.0 | 5.06 | 7.23 | **5.22** |

(t5's best −86.5k is a number t3 first reached at gen 621 — but the regiments differ, so this is shape, not a comparison.)
Per path at 330 (score/step, pct, rot): Straight 0.94 / 68 % / 5.8×, Spiral 0.81 / 65 % / 5.1×, Fig-8 0.68 / 59 % / **4.4×**, HighPerch 0.68 / 57 % / **6.7× (worst)**,
randomA 0.35 / 37 % / 5.3×, RandomB 0.23 / 24 % / 4.9×. Wind (ramp 8/18, realized 2.1–5.5 m/s): 2–4 → 4–6 bin 0.69 → 0.50 score/step,
52 → 38 % `pctInStreak`; `rotRatio` flat across bins (5.2 / 5.2). `NN_ELITE_SAME` 330/330; `tbBitePct` 89–95 %, `tbMeanIn` 5–7; longest stall still 16;
wall clock 521 s/gen vs t3 433 (+20 %).

**Crashes — the one t5-specific signal so far.** Crash-gens 216–330: **t5 36 vs t4 17 (same regiment) vs t3 11**. Detail: 28 gens with 1 crash, 8 with 2;
30 of the 36 are an *unchanged* elite — i.e. ~5 crash-carrying elites persisting 3–20 gens each (a 1-in-294 OOB is penalised mildly by design, so such an elite
stays until something beats it). Types: `egRadius` 28 + `egFloor` 16 (t4's window was all `egFloor`). The current elite's one crash: **RandomB**, `Eval`/OOB-radius,
621/610 steps, 4–6 m/s bin — the long cold-start course overrun at its end, blown out of the arena. Hypothesis to test later, not now: the tie-break can
prefer, among equal trackers, a genome that is smoother *because it commits less*, and that genome is the one that fails to recover at the arena edge in wind.
Watch whether crash-carrying elites become more frequent as the ramp opens (steps at 361/401/441).

**Smoothness, honestly**: t5 was under t3's curve by 1.1× at 250 and 0.6× at 275, level at 300, and **0.16× above at 325–330** (5.22 vs 5.06). Over
gens 240–330 t5's elite `rotRatio` wandered 5.10 → 6.69 → 5.22 while t3's curve fell 6.5 → 5.1. **Through gen 330 the tie-break has not separated t5
from t3's natural decline** — tracking is as good or better, crashes slightly worse, rotation equal. The mechanism is unambiguously active (bite ~90 %);
its marginal effect on the *elite's* rotation is not visible above t3's own trend. Remaining room: t3 ended at 4.55×; t5 must get clearly under that.
If the gap is still ~0 at gen 500, the conclusion is that selection on tracking already extracts most of the available smoothness under this measure,
and the next lever is the demand side (first-principles § 9 item 4) or a load-based companion measure — not a weight.

### 5f. Gen 411 read (2026-10-10 evening): `rotRatio` back under t3's curve; early-snapshot candidate extracted

| gen | t3 avg | **t5 avg** | t3 pct | **t5 pct** | t3 rot | **t5 rot** |
|---|---|---|---|---|---|---|
| 350 | −24984 | −26911 | 48.2 | 47.6 | 5.66 | **4.95** |
| 375 | −25653 | −27317 | 47.9 | 47.6 | 6.09 | **4.96** |
| 400 | −27708 | −30504 | 49.9 | 48.0 | 5.40 | **4.55** |
| 411 | −26858 | −31919 | 49.7 | 47.5 | 5.40 | **4.74** |

After the gen-330 convergence (§ 5e) the two curves separated again: t5 **4.55× at gens 385–400 — t3's gen-800 endpoint, reached at
half the generations** — and 4.74× at 411 vs t3's 5.40×. `pctInStreak` 47.5–48.0 vs t3 49.7–49.9 (within seed noise); pop-avg ahead.
Crash-gens 331–411: t5 **14** vs t3 4 (t4 0 to 413) — all from the gen-401+ elite carrying one OOB (293/294) after the ramp step at 401.
`NN_ELITE_SAME` 411/411.

**Early-snapshot candidate — operator 2026-10-10: "bake overnight, then decode on 400 or beyond."** Extracted now so it is in hand:
`artifacts-t5/nn_weights-t5-g400.dat` sha256 **`116ce3245c594de7`**, fitness **−95573.667378**, σ 0.0955 — the elite of gens **385–400,
294/294 crash-free for all 16 generations it held**, `rotRatio` 4.55×, `pctInStreak` 48.0 %, trained under ramp 0.47 (wind 1.9–5.7 m/s,
dir σ 60°, turb/gust/thermals at ~half envelope). ⛔ Not the gen-401+ elite (one OOB on RandomB). If a later crash-free elite with
≥ this `pctInStreak` and ≤ this `rotRatio` appears overnight, prefer it; otherwise g400. `nn_weights.dat` still holds t3 (`a5d097da18bf33ed`).

**Flight-prep chain for the snapshot** (t3 record, MANIFEST-t3 § 10): copy → `nn_weights.dat`; eval suite (tier0 must reproduce
−95573.667378 bitwise; ⚠️ spawns a second autoc + 20 workers — tiers 0–1 only while the bake runs, or accept the contention);
`nn2cpp -w nn_weights.dat -i autoc.ini` → `xiao/src/generated/nn_program_generated.cpp`; `pio run -e xiaoblesense_arduinocore_mbed`
(⭐ this flash also makes **slot 3 a random course** — `EMBEDDED_PATH3_RANDOM_A 1`, seed 24680, never flown); flash + identity check
(`weight_id=116ce3245c594de7`); **INAV: no firmware, no rates** (parity 36/24 ↔ 360/240 unchanged) but **restore the autoc-flight
blackbox fields** from the Sortie-A vibration set (T097 Sortie B: `QUAT`+`ATTI`+`NAV_ACC` on, keep `VIBE`+`PEAKS`, `rate_denom 4`,
`save`, power-cycle, header read-back, `flash_info` + erase); T069 bench verify (identity, rate parity after flash, ACRO purity,
static-airspeed rail watch-item).

### 5g. Gen 485 read (2026-10-11 morning): under t3's curve throughout 450–485; snapshot candidates compared per path

| gen | t3 avg | **t5 avg** | t3 pct | **t5 pct** | t3 rot | **t5 rot** |
|---|---|---|---|---|---|---|
| 425 | −27887 | −32206 | 49.7 | 48.3 | 5.71 | **5.29** |
| 450 | −28949 | −32566 | 50.3 | 47.5 | 5.30 | **4.64** |
| 475 | −31541 | −36108 | 50.3 | 48.4 | 5.39 | **4.68** |
| 485 | −32678 | −36329 | 51.6 | 48.3 | 5.37 | **4.78** |

Crash-gens 412–485: t5 **8** (all the gen-401–419 elite) vs t3 0; **every elite since gen 420 is 294/294 clean**. `pctInStreak` 47.5–49.9 vs t3 49.7–51.6
(t5 ~2 pts under; within seed noise but consistently so since ~400). `NN_ELITE_SAME` 485/485. Ramp step 481 ⇒ scale 0.63, wind 1.35–6.4 m/s.

**Snapshot candidates — all crash-free, all extracted to `artifacts-t5/` (`.dat`, git-ignored):**

| | **g400** `116ce3245c594de7` | **g422** `02acc4d39074c520` | **g480** `dfaf23b9b4558e2c` |
|---|---|---|---|
| fitness / tenure | −95573.67 / 16 gens | −95987.19 / 1 gen (replaced by a higher scalar, not a crash) | −98011.58 / 3 gens (ramp step at 481 ended it) |
| regiment: score/step · pct · rot | 0.671 · 48.0 · 4.55 | 0.671 · **49.9** · **4.34** | **0.688** · 49.1 · 4.77 |
| Straight | 1.02 · 71.6 · 4.34 | 0.98 · 71.9 · **3.76** | 1.02 · 73.1 · 4.92 |
| Spiral | 0.94 · 69.9 · 3.83 | 0.91 · 69.5 · **3.71** | 0.95 · 70.6 · 4.55 |
| Fig-8 | 0.72 · 60.2 · 3.93 | 0.78 · 62.5 · 3.98 | 0.82 · 63.8 · **3.81** |
| randomA (cold start) | 0.36 · 35.1 · 4.93 | 0.37 · **38.7** · **4.60** | 0.38 · 36.7 · 5.00 |
| HighPerch | 0.74 · 61.0 · 5.28 | 0.75 · **63.8** · **4.91** | 0.72 · 60.3 · 5.78 |
| RandomB (cold start) | 0.24 · 26.8 · 4.57 | 0.25 · **28.1** · **4.45** | 0.24 · 27.3 · 4.70 |
| wind seen (ramp scale) | 0.47 → 1.9–5.7 m/s | 0.53 → 1.7–5.9 m/s | 0.58 → 1.5–6.2 m/s |

**Recommendation: g422.** Smoothest on 5 of 6 paths and tracking equal-or-better on 5 of 6; the straight line at 3.76× is the lowest any
043 elite has shown on the path that defined the problem. Its 1-gen tenure is a fitness artefact (a higher-scalar genome arrived next gen;
elite re-eval is bitwise, so there is no robustness story in tenure). g480 is the alternative if on-course score on the tail-chase trio
is preferred over smoothness; g400 is dominated by g422 on every row.

## 6. ⚠️ What this run does NOT fix (unchanged from t4 § 6)

D1 pitch short period absent from the sim; D2 n = 1 airframe; D4 sensor-path delay un-modelled; entry dither
deferred (cold-start content = the two random courses); AIRSPEED = groundspeed on both sides (pitot is backlog).

## 7. Post-run (fill in at the end, BEFORE extraction)

| Field | Value |
|---|---|
| final gen fitness / `pctInStreak` / `avgMaxStreak` / crashes / `rotRatio` | ⏳ |
| per path (score/step, pctInStreak, rotRatio) | ⏳ |
| `#GenSel` summary (bite rate trajectory) | ⏳ |
| `retain=keep` pinned + sampled | ⏳ |
| extracted genome sha256 (⛔ never overwrite `nn_weights.dat` = t3 `a5d097da18bf33ed` without backing it up) | ⏳ |
| eval suite (`eval_suite.sh <t5.dat> all <master seed>`) | ⏳ |
| determinism (`NN_ELITE_SAME` count) | ⏳ |

### 5h. Flight 2026-10-11 — gen 512 flown (not a § 5g candidate)

The operator cut **gen 512** from the live run after the gen-485 read and flew it the same morning:
`weight_id=1731d937e3af9c1c`, `firmware_id=c3eb28e6b938dd1a`, program `…2026-10-09T01:04:08.995Z/gen9512.dmp.zst`.
⚠️ No `.dat` for it is in `artifacts-t5/` and `nn_weights.dat` in the repo hashes `3af8e3ab787b75a5` — the
flight article was built off-repo; extract and hash it here, then eval-suite it for the record. Result:
best-tracking ACRO flight to date (median error 10–15 m, 4 spans, 0 gaps, no Z steps), pitch cycle unchanged.
Full read: [`../flight-analysis-20261011.md`](../flight-analysis-20261011.md).

**Second flight, same article, 2026-10-11 afternoon** (log 2 in the same flash; NAV_POS restored, rate 1/8): 3 spans,
Straight / Fig-8 / RandomB, all path-complete, 1585 ticks, 0 gaps. Straight 8.1 m median, Fig-8 0.344 / 25.5 % in-streak
(best ACRO span to date), RandomB 0.163 (sim g422 0.25). `navEPV` max 541 cm — estimator seen clean directly. Pitch cycle
unchanged. Full read: [`../flight-analysis-20261011b.md`](../flight-analysis-20261011b.md). The `.dat` / hash / eval-suite
step above is still open.
