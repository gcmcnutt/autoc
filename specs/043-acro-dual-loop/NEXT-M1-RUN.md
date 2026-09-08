# 043-t3 — how to run the next M1 bake

⭐ **Written for a FRESH CONTEXT.** Everything needed is here or linked; nothing relies on remembering the
session that produced it. Read this top to bottom before touching anything.

**Status at handoff (2026-09-07)**: autoc `d87102f`, crrcsim `4a68a72`, both trees clean.
⛔ **Constitution IX pre-run gate: the operator ran `rebuild-perf.sh` manually and it PASSED.** Do not
re-run it, and ⛔ **do not rebuild anything** — see § "While it runs".

---

## 1. What this run is

The first M1 bake since **043-t2** (`autoc-9223370248704297747-2026-08-31T04:27:58.060Z`, gen 800,
fitness −88,013.84), which was flown three times (09-05, 09-06, 09-07) and analysed in
[flight-analysis.md](flight-analysis.md), [flight-analysis-20260906.md](flight-analysis-20260906.md) and
[actuator-pin.md](actuator-pin.md).

It carries **four** changes. Full derivation in [m1-run-delta.md](m1-run-delta.md); the short version:

| | change | why |
|---|---|---|
| ⭐ **objective** | `OobCrashPenaltyWeight` **0 → 10.0**, `EnableHullCrashPenalty` stays **0** | t2 crashed **4–7%** of scenarios vs 041-t7's 0.7%, **100% arena egress**, because M1 charged *nothing* for it while breaking a streak cost the whole 5 s climb from 1× to 5×. Busting was cheaper than backing off. |
| **plant** | `COMPUTE_LATENCY_MSEC_DEFAULT` **30 → 10 ms** | flight-measured 9.9 ms (fetch 2.9 + eval 1.6 + send 5.4). The 30 predated current firmware. |
| **aircraft** | `rc_expo` **20 → 0**, gyro Kalman **OFF** | ACRO silently applied 20% expo that the sim never modelled; Kalman also cost ~10 ms of loop latency |
| **firmware** | engage-prefill frame fix, flight log **v5** | affects *flight*, not the bake — the sim never had the prefill bug |

⛔ **The single question this run answers**: does pricing arena egress at **29/30 per crash** move the
endgame crash rate to **0–2 scenarios** without costing tracking?

## 2. What to compare, and what NOT to

⛔ **Raw fitness is NOT comparable to t2's −88,013.84.** The objective changed. ⭐ **Exception**: a
crash-free genome scores ×1.000, so a zero-crash result *is* fairly comparable.

Judge on, in order:

1. ⭐ **crash rate** — target **0–2 of 294** at the end. t2 ran 12–21.
2. **`pctInStreak`** — t2 hit **54.6%** (041-t7: 51.3%). ⚠️ If this collapses, the penalty over-corrected
   into timidity; see § 5.
3. **per-axis `dCtrl` / ⟨|out|⟩** — the variation-stable comparator
   ([project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md))
4. avg target distance / `avgMaxStreak` — early-progress signals; ⚠️ *completions are a red herring* in
   the servo era ([project_servo_era_progress_metrics](../../.claude/projects/-home-gmcnutt-autoc/memory/project_servo_era_progress_metrics.md))

⚠️ **Do not judge it dead at a plateau around gen 240.** The past-only architecture produces a
slow-start / accelerating curve — 037-t9 sat at ≤2.9% `pctInStreak` until gen 243 and then broke out to
11.4% by 631 ([project_no_future_curve_shape](../../.claude/projects/-home-gmcnutt-autoc/memory/project_no_future_curve_shape.md)).

## 3. How to launch

```bash
cd /home/gmcnutt/autoc
bash scripts/train.sh autoc.ini logs/autoc-043-t3-m1-crashcost.log
```

⛔ **DETACHED, and never through a harness background task.** Agent-launched background tasks get reaped
mid-run and look exactly like a silent crash
([project_autoc_worker_crash](../../.claude/projects/-home-gmcnutt-autoc/memory/project_autoc_worker_crash.md)).
Launch it in a terminal that outlives the session (`nohup`/`tmux`/`screen`), or hand it to the operator.

**Name**: `autoc-043-t3-...` — lexicographic `autoc-<feature>-t<N>-<detail>`, no timestamps
([feedback_artifact_naming_convention](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_artifact_naming_convention.md)).

**At startup, confirm before walking away:**

- `Effective master seed` — ⭐ **write it down immediately**; `Seed = -1`, so this is the only record
- regiment reads **294** scenarios (6 paths × 49 winds; `ExpectedScenarioCount` guards it)
- pop **5000**, **800** generations
- ⭐ **the crash penalty is actually live** — see § 5, this is the highest-risk silent failure

Expect **~43 h** wall clock (t2 was 43.5 h for the same regiment).

## 4. While it runs

⛔ **DO NOT REBUILD ANYTHING.** Overwriting `build/bin/autoc` breaks worker re-execs mid-run
([feedback_no_rebuild_during_training](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_rebuild_during_training.md)).
That includes "just a quick fix" to an analytics script that shares the build.

Monitor with the wrapper, never by hand-calling the analytics:

```bash
bash scripts/generate_pngs.sh m1 logs/autoc-043-t3-m1-crashcost.log
```

Hand-calling `src/analytics/*.py` loses the incremental per-gen S3 cache
([feedback_generate_pngs_wrapper](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_generate_pngs_wrapper.md)).

## 5. ⛔ The two ways this run can fail quietly

**(a) The penalty never activates.** It was a silent no-op until 2026-09-07: `applyCrashPenalty()` used to
return early on `!enableHullCrashPenalty`, and **that gate covered the OOB branch too**. The gates are now
independent (`src/autoc.cc`), but verify empirically rather than trusting it — ⭐ **the run would complete
all 800 generations with the objective unchanged and nothing in the log would say so.**

Check: `EnableHullCrashPenalty = 0` **and** `OobCrashPenaltyWeight = 10.0` in `autoc.ini`, and by ~gen 150
confirm crash rate is trending *down* relative to t2's curve at the same generation. The penalty is
curriculum-ramped and is **0.00% per crash through gen 40** by design, so absence of effect before then is
expected, not evidence of the bug.

**(b) It over-corrects into timidity.** The penalty multiplies **every case** of a genome uniformly, so
under epsilon-lexicase its bite depends on clearing the per-case MAD epsilon (~0.3% on a converged case,
~100% on a hard one). Because 3.34% *exceeds* 0.3%, it acts on converged cases as a **threshold, not a
gradient** — 1 crash and 3 crashes both fall outside epsilon.

⚠️ Symptom: crash rate → 0 **but `pctInStreak` collapses** — the policy has stopped going near the
boundary at all, and tracking needs the boundary.
⇒ Fix if seen: penalise **only the crashed scenario's case** rather than every case uniformly. That is the
lexicase-native form and a real code change; do not make it blind, and do not simply lower `w` — that
re-opens the original problem.

## 6. At the end

1. ⭐ **Pin `retain=keep` on the S3 prefix** and verify by sampling objects — the 30-day `retain=expire`
   lifecycle is real and a lost run is unrecoverable.
2. **Write the MANIFEST** — copy the structure of
   [artifacts/MANIFEST.md](artifacts/MANIFEST.md) (t2's). It must carry the S3 prefix, master seed,
   regiment, gen-800 fitness, the commits, `autoc.ini.as-run`, and ⛔ **the input scale constants** —
   without them the genome loads clean and flies wrong.
   ⚠️ **Do this BEFORE extracting weights**: `nnextractor` overwrites `nn_weights.dat`, and which run and
   generation flew becomes unrecoverable.
3. Extract and regenerate for flight:
   ```bash
   ./build/nnextractor -k <run-prefix> -g 800 -o nn_weights.dat
   ./build/nn2cpp -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp
   ```
   ⚠️ `-w` is the genome, `-i` is the config. ⚠️ `nnextractor -g` takes the **actual** generation (800),
   matching `dmp-dump --gen`. ⚠️ `nnextractor` prints `Generation: 800` while `autoc` eval mode prints
   `799` — same net; **match on fitness**, not the label.
   ⚠️ `xiao/src/generated/` is **git-ignored**, so it never survives a checkout — always regenerate.

## 7. ⚠️ What this run does NOT fix — say so in the outcome

| | |
|---|---|
| **D1 pitch peak timing** | sim peaks at **85 ms**, real at **133–166 ms**. Measured, unfixed, and it is *the* axis 043 exists to fix. Candidates: `Cmq`, `Cm_alpha`, pitch inertia. |
| **D2 second airframe** | everything rests on **n = 1** with a known-asymmetric wing and nose-heavy CG. The stall cycle in `actuator-pin.md` §8/§9 is exactly the kind of thing that is an *article* property, so this bake may be tuned to one aircraft's defects. Parts procurement is slow — the operator accepted this. |
| **D4 NN sensor-path delay** | ~21 ms on accel, ~6.4 ms on gyro, un-modelled. An implementation was **backed out** (filters at the 20 Hz gather sit above Nyquist); `include/autoc/eval/sensor_lpf.h` records the two correct approaches. |

⭐ **Record these in the run's MANIFEST at launch**, not at analysis time. If the pitch result disappoints,
D1 is the first suspect and it must not look like a post-hoc excuse.
