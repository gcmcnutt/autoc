# NEXT: t5 — the conditional excess-rotation case (handoff, written 2026-10-07 at t4 gen ~405)

Read first: [artifacts-t4/MANIFEST.md § 6b](artifacts-t4/MANIFEST.md) (why t4 stalled),
[tempered-control-first-principles.md § 10](tempered-control-first-principles.md) (what changes and why
it is not tuning), tasks.md **T109 / T110**.

## 0. State at handoff

- **t4 was STOPPED at gen 413** (operator, 2026-10-07 evening; seed 1791343902, prefix
  `autoc-9223370245510873367-2026-10-07T03:31:42.440Z`, 413 objects `retain=keep`, elite
  `7bb858ae439a2fbd` in `artifacts-t4/`, MANIFEST § 7 filled). Step 1 below is **done** except pasting
  the eval-suite result. ⛔ The only remaining lock: the t4 eval suite uses `build/autoc` — rebuild after
  it finishes.
- The t5 code is **written and syntax-checked but NOT built**: `include/autoc/eval/selection.h`,
  `src/eval/selection.cc`, `include/autoc/util/config.h` (+`ExcessRotationAxisMode`), `src/autoc.cc`
  (startup validation), `tests/selection_tests.cc` (+4), `tests/contract_config_tests.cc` (165→166).
- Reports already carry what t5 needs: `plot_evolution_progress.py` rotRatio panel,
  `tracking_by_path_progress.py` (per-path lines over gens), `tracking_by_wind.py` (realized-wind bins).

## 1. When t4 ends (in this order)

1. **t4 close-out** (MANIFEST § 7): pin `retain=keep` on the prefix; record gen-800 fitness /
   `pctInStreak` / crashes / `rotRatio`; back up t3's `nn_weights.dat` before any extraction;
   `eval_suite.sh <t4.dat> all 1791343902` for the record (expect poor cold-start and poor 4–6 m/s bins).
2. ✅ **Operator**: clean `scripts/rebuild-perf.sh` done 2026-10-08 17:45 (`PERFORMANCE_BUILD=ON`).
3. ✅ `ctest` 53/53 binaries (the new cases are inside `selection_tests` 21/21 and `contract_config_tests` 12/12).
4. ✅ **Bitwise gate**: `autoc-eval.ini` with the t3→t4 deltas reverted (axis off) ⇒
   `NN_EVAL_SAME: fitness=-88026.619367` on the new binary — the `filter()` refactor consumes no PRNG, so
   Off/Independent select identically. Also ✅ both fail-loud paths verified, ✅ 2-gen smoke with
   `#GenSel` live (tasks T110 for the numbers).
5. **ini delta for t5** — ✅ **APPLIED 2026-10-07** to both `autoc.ini` and `autoc-eval.ini` (own lines;
   inih leaves `#` in string values). Shown for the record:
   ```ini
   EnableExcessRotationAxis        = 1
   ExcessRotationAxisMode          = conditional
   ```
   Everything else stays at the t4 values (wind envelope 0–8 m/s, turb 0.5–2.0, gust 0.25–1.0, thermal
   0.5–1.5 ×8, `AeroStandardPath3 = randomA`, `WindDirectionSigma 60`). ⚠️ t4's as-run ini now
   **aborts at startup by design** (mode required when the axis is on) — restate `independent` to replay it.
6. **t5 MANIFEST** (`artifacts-t5/`): copy the t4 one; § 3 delta = this one key; baselines: t3 genome
   under the t4 regiment `rotRatio` 5.10× (unchanged — same measure), t3 gen-396 per-path table from T109.
7. Launch: `bash scripts/train.sh autoc.ini logs/autoc-043-t5-m1-condrot.log` (Xvfb `:2` first — it does
   not autostart). Monitor with
   `bash scripts/generate_pngs.sh m1 logs/autoc-043-t5-m1-condrot.log --compare 043-t3-crashcost:logs/autoc-043-t3-m1-crashcost.log`.

## 2. What to watch, and when to call it

**Gen 2–5, first**: the new `#GenSel` line (selection telemetry, drained after every breed).
`tbPerSel` is how many tie-breaks a selection event ran, `tbMeanIn` how many candidates each got to
choose among, `tbBitePct` how often it removed at least one. **`tbBitePct` ≈ 0 or `tbMeanIn` ≈ 1 means
the tie-break has no leverage** (035's "effectively absent") — stop and rethink the same day; a healthy
start is tie-breaks on most selections, `tbMeanIn` well above 1, bite rate high. This number cannot be
read off any pinned run, so it has no reference — it is the mechanism check, not a comparison.

The t4 failure was visible by **gen 40–50** in the population average, not in the elite. Checkpoints:

| gen | healthy (t3 reference) | t4 (the failure) | call |
|---|---|---|---|
| 40 (ramp 0, wind identical to t3) | pop-avg ≈ −4.2k, `pctInStreak` ≈ 5% | −1.4k, 1.6% | if t5 is at t4's numbers, the tie-break is not enough — stop early, no need for 800 |
| 100 | avg −9.2k, pct ≈ 20% | −3.2k, ≈ 10% | |
| 183 | avg −14.6k, pct 33.7% | −3.6k, 25.5% | |
| 396 | avg −26.6k, pct 48.7%, elite replaced every ≤ 26 gens | −3.9k, 30.5%, elite frozen since 281 | an elite frozen for a full ramp window (40 gens) is the stall signature |

Then T106, **re-based 2026-10-08 on t3's own post-hoc trajectory** (now cached from its pinned dmps and
overlaid dashed on `tracking_by_path`): with NO rotation axis t3's regiment `rotRatio` fell **11.8× (gen 30)
→ 8.0 (150) → 6.3 (250) → 5.85 (396) → 4.55 (800)** — the natural decline reaches most of the way to the
old 5.10× line by itself, so "< 5.10×" is not evidence of the axis working. t4 (Independent) rode that
curve to gen 250 and froze at 7.25×. **t5 passes on smoothness only if its `rotRatio` sits BELOW t3's
dashed line at matched gens from ~gen 150 on and ends clearly under 4.55× at 800**, with the straight line
no longer the worst path; and on tracking only if calm-bin `pctInStreak` (`tracking_by_wind`) and the
per-path lines are within noise of t3's, crashes ≤ t3. The `#GenSel` bite rate says whether the pressure
exists; the slope against the dashed line says whether it is enough.

## 3. Open alongside (unchanged from t4)

D1 (T093–T096), the prop-balance/instrumented flight (T097/T098), `sample_elite.sh` (T107), T101 six
random courses, Xvfb user unit. The wind brittleness t4 reproduced in sim (41 → 24% `pctInStreak` from calm
to 4–6 m/s) is the flight problem; t5's job is to get a population that converges on tracking *under*
that envelope — the envelope itself is right.
