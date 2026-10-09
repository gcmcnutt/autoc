# 043-t5 — `autoc-043-t5-m1-condrot` — MANIFEST (written BEFORE launch, 2026-10-08)

Pattern of record: t3's `artifacts-t3/MANIFEST.md`, t4's `artifacts-t4/MANIFEST.md` (read its § 6b first — t5 exists
because of it). Handoff: [`NEXT-T5-RUN.md`](../NEXT-T5-RUN.md). Fields marked ⏳ are filled at launch / at the end.

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | ⏳ `s3://autoc-m1/autoc-<id>-<launch time>/` (from the log's "Run ID") |
| Retention | uploads as `retain=expire` — **pin `retain=keep` on all objects at the end and verify by sampling** (t4: 413/413, 22 sampled) |
| Master seed | ⏳ (`Seed = -1`; logged as "Effective master seed" — the only record of it) |
| Regiment | pop **5000** × **800** generations × **294** scenarios (6 paths × 49 winds; `ExpectedScenarioCount = 294`) — **identical to t4** |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-043-t5-m1-condrot.log` (Xvfb `:2` must be up — it does not autostart) |
| Launched | ⏳ |
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
