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
