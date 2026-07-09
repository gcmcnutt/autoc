# 038 Accurate M2 — Outcome / Findings

Running record of per-task verdicts, escapes, and pinned artifacts. Populated during
`/speckit.implement`; consolidated at close (T037).

## Phase 0 — Prework

### T001 / P0-A — 033 PRNG cascade determinism validation → **CLEAR** (2026-07-01)

**Verdict**: CLEAR — no bug found. The 033 PRNG-cascade rework (`acf732f`) deterministically
regenerates a scenario from its seeds; the suspected single-SHA bug did not reproduce.

**Method**: full `make -C build` (the project's regression pattern — the gtest suites run as part of
the build). All suites **PASS**, zero failures, including `scenario_prng_tests` (cascade D1–D5 contract)
and `eval_mode_replay_tests` (cross-process bitwise replay). No code change required — validation was the
deliverable per FR-P0A.

**Note**: the tasks.md T001 command originally cited `./tests/run_autoc_tests --gtest_filter=...`; this
build has no standalone test binary — `make` (or `make run_autoc_tests`) builds and runs the gtest
suites inline. Command corrected in tasks.md.

### T002 / P0-F — streak threshold revert → **DONE** (2026-07-01)

Reverted `FitStreakThreshold` 0.3→0.5 in `autoc-tracker.ini`, `autoc-eval-tracker.ini`,
`autoc-eval-tracker-visual.ini` (M1 inis already 0.5). Reward-shaping is exhausted as a depth lever
(037 t15 gain was threshold-confounded); the architecture studies start from the un-confounded cone.

### T003 / P0-E — svTau dead-path removal → **NO-OP / already done** (2026-07-01)

The svTau/servoTau code dead-path was **already removed 2026-06-12** (`craft_variation.h:150-151`:
"servo first-order tau draw removed 2026-06-12 — v2 has no lag term; draw order shifts accordingly,
which is fine cross-build"; `scenario_metadata.h:118`). No residual draw-order placeholder in the craft
PRNG cascade. The only remaining `svTau` reference is in the historical `verify_037_metrics.py`, left
untouched per the historical-scripts-immutable practice. No code change needed.

### T007/T008/T009 (P0-D crrcsim + protocol) → **DONE** (2026-07-01)

- **T007** (P0-D-1): `getSimulationTimeSinceReset()` rounds (`std::llround`) not truncates → exact 50 ms
  gaps; strict single-gap source-spacing restored (crrcsim + tracker_stepper). ⚠️ [OP] confirm at the
  rebuild-perf gate whether this settles the "identical-state/different-simTimeMsec" divergence noted in
  crrcsim CLAUDE.md. Commit crrcsim `e6108cc`, parent `55fd4df`.
- **T008** (P0-D-3): steady wind recorded via `eom01->getLastLocalAirmass()` (NED ft/s→m/s) → both modes.
  ⚠️ [OP] re-run `wind_study.py` against a fresh bake to confirm the getter is populated.
- **T009** (P0-D-2): `RecordedRunConfig` (fit cone / cadence / crash-penalty) appended to `EvalResults`
  (no version bump), stamped from `ConfigManager::getConfig()`. Commit `0cf29cf`.

### T009b (FR-P0H situational-awareness inputs) — **PART 1/2 DONE: layout** (2026-07-01, commit `3cd2bc9`)

Layout only (Tracker 54→60, Pathgen 33→37; weight counts 2051 pathgen / 2787 tracker; topology strings).
Population (part 2) is the next focused unit. **Recorded decisions for part 2**:
- **(B) `inward_body`** = `orientation.inverse() * normalize(-pos_horizontal)` (radial-inward toward the
  cylinder axis at the virtual-frame origin, body frame). Add a stateless `inwardBodyDirection()` helper to
  `arena.h`/`arena.cc`; populate in gather_tracker (has arena) and gather_pathgen.
- **Pathgen (B) arena plumb**: add `const FlightArena&` to `gather_pathgen_inputs`; `NNControllerBackend`
  carries one from config (desktop); **nn2cpp bakes a compile-time `FlightArena{radius,floor,ceiling}`
  literal from the run config** into the generated xiao firmware (operator 2026-07-01). Live-pathgen
  firmware change (FR-033).
- **(A) two-path state**: `time_since_seen` (tanh of blind-ticks) + `exit_dir` sin/cos (held last-seen NDC
  centroid bearing). Single-source the update in a shared helper called by BOTH `TrackerStepper` and
  `CrrcsimTrackerHelper`; **reset per scenario/engage** in both `initScenario` (FR-030 determinism).
- **dmp_dump**: add the 6/4 new columns (honest recording).
- Quaternion convention: world→body via `.inverse()` (matches `camera_projection.cc:117`,
  `aircraft_state.h:440`).
- ⚠️ layout compiles but gather funcs don't populate the new fields yet → **no bake until part 2 lands**
  (uninitialized = non-deterministic); test fixtures with hardcoded 54/33/1923/2595 fail until T011.

#### T009b part-2 recon map (file:line — start here in a fresh /speckit.implement)

- **(B) `inward_body` helper**: add `inwardBodyDirection(const gp_vec3& pos, const gp_quat& q)` decl to
  `include/autoc/eval/arena.h` (near `distanceToBoundary`, ~line 98), body in `src/eval/arena.cc` (near
  `distanceToBoundary` at `arena.cc:48`). Math: arena centered at virtual-frame origin (0,0,0); world
  radial-inward = `normalize(-pos.x, -pos.y, 0)` (Zero if `hypot(x,y)<1e-6`); body = `q.inverse() *
  inward_world`. Convention verified: world→body = `.inverse()` (`camera_projection.cc:117`,
  `aircraft_state.h:440`).
- **`gather_tracker_inputs`** (`src/nn/evaluator.cc:438`): populate (B) `inward_body_{x,y,z}` from
  `chase.getPosition()`+`getOrientation()`; write (A) `time_since_seen`/`exit_dir_{sin,cos}` from values
  passed by the caller. Existing `dist_to_boundary_along_vel` at `evaluator.cc:488`.
  - callers: `src/eval/tracker_stepper.cc:254`, `crrcsim/.../crrcsim_tracker_helper.cpp:202`.
- **`gather_pathgen_inputs`** (`src/nn/evaluator.cc:318`): add `const autoc::eval::FlightArena&` param;
  populate (B) `dist_to_boundary` (via `distanceToBoundary`) + `inward_body`. NO (A) (M1 always visible).
  - callers: `src/nn/evaluator.cc:404` (`NNControllerBackend::evaluate` — give the backend a `FlightArena`
    member from config, `evaluator.h` ctor ~line 371); `tools/nn2cpp.cc:123,189` (codegen — **bake a
    `FlightArena{radius,floor,ceiling}` literal from config** per operator decision).
- **(A) two-path state** (`time_since_seen` = `tanh(blind_ticks·dt / ~2s)`; `exit_dir` = held NDC-centroid
  bearing of `history.left/right_{x,y}[5]`, updated when a beacon is visible = `cep < cep_gate_threshold`,
  `kCepSentinelThreshold` per `fitness_decomposition.cc:200`): single-source the per-tick update in a shared
  helper; add members + **reset in `initScenario`** to both `TrackerStepper` (`tracker_stepper.h` +
  `tracker_stepper.cc` ~108) and `CrrcsimTrackerHelper` (`crrcsim_tracker_helper.cpp` initScenario ~59-62).
- **dmp_dump columns** (`tools/dmp_dump.cc`): add the 6 tracker + 4 pathgen input columns (honest recording).
- **T011 fixtures to regen**: `tests/nn_layout_tests.cc`, `nn_evaluator_tests`, `nn_serialization_tests`,
  `tick_rescale_tests`, NN01 fixtures (hardcoded 54/33/1923/2595); xiao codegen regen via `nn2cpp`.
- **Build loop**: operator runs `cd build && make` (compiles + runs gtest) for source edits; `rebuild-perf.sh`
  for the from-scratch determinism gate. No standalone test binary — `make` IS the unit-test run.

### Enrichment early-training tax — basic-m1 diagnostic (2026-07-02, t2)

**Question**: did the +4 FR-P0H (B) inputs (33→37, 1923→2051 weights) hurt M1 learning, or is the
stopped full-M1 rebake (t1) just the basin lottery? **Answer: an early-training tax, not fatal.**

Ran `autoc-basic-m1.ini` (pop 3000, single longSequential, 16 winds, all four variation classes) —
run `autoc-038-t2-basic-m1-diag`, master seed 1783003482. Compared against the **exact shape-match**
`autoc-037-t5-basic-m1-20hz` (same pop/cadence/paths/winds/variations; ONLY 33in/1923w vs 37in/2051w).
Comparator = `avgMaxStreak` / `pctInStreak` (servo-era progress metrics; raw fitness is a noisy cross-run
comparator per [project_late_run_fitness_interpretation]). Matched-gen:

| gen | 038 avgMaxStreak / pctInStreak | 037-t5 |
|----:|:------------------------------:|:------:|
| 10  | 2.6 / 1.5 | **6.0 / 5.2** |
| 20  | 3.1 / 2.3 | **6.0 / 5.2** |
| 50  | 1.6 / 0.4 | **6.8 / 5.3** |
| 100 | **5.7 / 2.4** | 4.2 / 2.1 |
| 173 | 5.8 / 2.5 | **8.2 / 2.3** |

**Finding**: the enriched net is **materially worse through ~gen 50** (037 reaches usable streaks ~6 by
gen 10 and holds; 038 sits at 2–3 and *dips to 1.6 @ gen 50*), crosses over ~gen 100, and by gen 400
reaches strong tracking on its own (avgMaxStreak 17.7, pctInStreak 8.6, target dist 12 m→9.6 m,
target-visible occ 3–4%→8%). So the 4 extra inputs impose an early-training tax (bigger weight space),
NOT a capability loss — consistent with the operator's "needs more than 400 gens" for the full-M1 shape
where the tax compounds over pop 5000 / 294 scenarios. Determinism unaffected: an M1 eval
(`autoc-basic-m1-eval.ini`, extract-from-live-gen) bit-matched training (second-order determinism holds).
Overlay PNGs: `specs/038-accurate-m2/autoc-038-t2-basic-m1-diag_*.png` (`--compare 037-t5-20hz`).

### T013 (M2 half) — enriched M2 baseline: sanity-check PASSED, reproduces the perception-capped ceiling (2026-07-05)

**t4 = `autoc-038-t4-m2-baseline`** (autoc-tracker.ini, 60-input enriched tracker, pop 5000, 20 Hz,
source = t3 gen800 repointed, **stopped at gen 307/800** — plateaued ~150 gens, US3 is the priority).
**S3**: `autoc-m2/autoc-9223370253664873750-2026-07-04T18:31:42.057Z/`.

**Launch gotcha (resolved)**: first attempt fail-loud-rejected t10's 33-input source dmp (topology
mismatch — greenfield break, *why* T013 re-bakes M1 first). Repointed `TrackerSourceRun` → t3 gen9200. ✅

**Sanity check PASSED** (first-ever live run of the enriched tracker path):
- Tracker SA wiring (reset/update/writeInputs) runs clean through eval, 20 workers, zero crashes.
- Source loads (37-input t3), tracker analytics populate, new M2 sensors sane + attended.
- Clean + stable: **0 hull, 0 over**, avgVis ~73%, median range ~17 m, spiral ~0.30 — normal M2 behavior.

**Result — plateaued at the perception-capped ceiling.** gen 307: best −14,033, avgMaxStreak 23.1
(**1.16 s**), pctInStreak 8.4%. Flat since ~gen 150. vs 037 t11 (0.5-threshold baseline): t4's avgMaxStreak
≈ t11's *final* (23.0) but pctInStreak trails (8.4 vs 12.2%) — long single streaks, thinner coverage.

**Key finding — capacity is NOT the M2 limiter, information/anticipation is.** `rnn_capacity` over the run:
eff-rank held ~11/16 (recurrence broadly engaged), **spectral radius inflated 2.5 → 5.7** (memory timescale
growing) — while **tracking depth moved nothing** for ~150 gens. The net has memory, uses it, keeps growing
it, and it doesn't convert to depth → the bottleneck is upstream (perception / what-to-anticipate). Direct
motivation for **US3 (predictor head)** over more recurrence.

**Overrun (FR-P0H payoff check)** — no isolable anti-*target*-overrun effect, as predicted. The visible
difference (t4 **hull=0** throughout vs t11 **hull 4→23** growing; t4 median range 16.8 m vs t11's tighter
13.6 m) is the **hull PENALTY** (t4 `EnableHullCrashPenalty=1`, t11 OFF) standing the chase off farther for
safety — NOT the arena sensors (which sense the arena boundary, not the target, so can't reduce
target-overshoot). `over` (overshoot loss) ~0 in both. Target-overrun remains a US3/anticipation problem.

**Sensor note**: `exit_dir_sin/cos` assessed **low-value** (redundant with the beacon history inside the
0.8 s window; unique info only for blind periods *longer* than the window, where a stale bearing is
dubious in dynamic flight). To be **tossed** riding the US3 format break (free — US3 breaks the format
anyway for the 3→7 output growth). `time_since_seen`, `span_rate`, `tilt` are keepers; `inward_body` too.

PNGs: `specs/038-accurate-m2/autoc-038-t4-m2-baseline_*.png` (`--compare 037-t11 / 037-t15`).

### US3 predictor — design rationale: forward-model the direct observable, not the target's state (2026-07-07)

Core "why" behind the span/closure-predictor head, from the t6 predictor-analysis discussion:

- **Predict the direct image-space observable, not the latent target state.** The aux head predicts
  `beacon_pair_span` (on-screen wingtip separation) at +50/+100/+150 ms — i.e. it forward-models *the sensor
  signal the chase will actually see*. It deliberately does **not** estimate the target's world-frame state
  (position / velocity / attitude). That estimation is the hard **inverse** problem — monocular depth from a
  beacon pair, ego-motion compensation, aspect disambiguation, filtering — and it forces a latent world
  representation the controller doesn't need. Predicting the observable is *more direct and far more
  achievable* (the achievability leg of [feedback_clear_objectives_not_tuning] /
  `.claude/.../feedback_clear_objectives_not_tuning.md`): don't ask the net to solve for state it can't
  cleanly observe; ask it to predict what it will directly perceive.
- **What `span` means / is contaminated by.** Span = raw NDC distance between the two wingtip beacons
  (`compute_pair_span`), dominated by **range** (closure proxy: closer ⇒ wider), modulated by **target
  aspect** (foreshortening), and — because the sim projects rectilinearly (`screen = tan θ / tan(fov/2)`) —
  by an **off-axis tan-stretch** (a fixed angular wingspan reads wider toward the frame edge). That last term
  is an ego-pointing contamination; span is roll-invariant. The design-comment claim that span is
  "ego-invariant" holds only near frame center.
- **The forward-model is closed-loop.** Control (out[0..2]) and prediction (out[3..5]) share the recurrent
  trunk, so "span@+150 ms" is "given the target dynamics in my hidden state **and the control I'm about to
  apply**, how wide will the beacons be" — a forward-model of the *consequence* of relative motion, not an
  external world state.
- **If ever enriched, stay in the observable domain.** Span is a scalar that collapses range×aspect and
  drops **bearing**. A richer "predict toward intercept" target would add future **centroid/bearing** and
  **tilt** — but those are *also direct observables* ("where the blob will be on screen"), never world-state.
  So the only real axis is *how much of the image-space observable to forward-model*; state estimation is
  off the table by design. Operator: "span is nice, keep it."
- **Follow-on (backlog, not now)**: switch the sim to a **spherical/equidistant** projection so span becomes
  a clean angular quantity (removes the ego-contamination from the very thing we predict) — captured in
  `specs/BACKLOG.md` → [040 — camera redo / research] Question 3. Model the *actual* chosen lens's projection
  in sim once hardware is picked.

Status: t6 (M2 predictor run, `EnablePredictorHead=1`, tracking t5 gen800) is the first test. An early read
(gen ~120–150) looked encouraging — span-calibration diagonal emerging + near-horizon error bending down
after a gen-~85 peak — but it **did not hold**; see the interim result below. Panel:
`autoc-038-t6-m2-predictor_predictor_analysis.png`.

### US3 predictor (t6) — result @ gen 344 (STOPPED 2026-07-08): predictor forms weakly at best, ceiling holds

**t6 stopped at gen 344/800** (operator: freed the box for the t7 shared-env A/B — the env-seed question is
now the priority; the variation-ramp back-half wildcard did not kick 318→344, best only −13670→−13382).
It ran *past* t4's endpoint (307), so it's a clean predictor-vs-no-predictor read. Verdict:
**anticipation via a passive span-predictor does not buy tracking depth.** Gen-344 final panel confirms the
gen-318 read — in-streak error stayed *reversed* (0.50 vs out 0.45), closure-rate error *rose* to ~0.75,
span still uncalibrated (vertical spray).

- **Competence: t6 == t4.** Fully converged onto t4's perception-capped plateau on all four mode metrics —
  perception ~0.70, track occupancy ~0.095, median range ~17 m, worst-blind ~8 s — despite the predictor +
  enriched inputs + running 11 gens longer. Raw best fitness marginally better (−13670 vs t4 −14033) but the
  honest competence comparators say *equal*. **No ceiling break.**
- **Predictor: the encouraging early signals did not hold.** Aggregate per-horizon error stayed a noisy
  plateau ~0.5–0.7 NDC and never approached the realized-span σ floor (~0.06). The gen-210 calibration
  diagonal turned out **elite-dependent/transient** (gen-315 elite is back to a vertical ±1 spray). Most
  tellingly, the **in-streak accuracy advantage reversed** — in<out (0.47 vs 0.56) at gen 210, the "loop
  forming where tracking is good" signal, flipped to **in>out (0.62 vs 0.52) at gen 315**. The closure-rate
  head is **saturating the ±1 tanh rails** (structural ceiling vs realized ±7–14 NDC/s).
- **Read (architectural, not tuning):** a passive, **non-actuated** aux predictor head (out[3..6] never feed
  control) under **weak tie-break lexicase selection** neither learns a robust forward-model nor lifts
  depth. The shared-recurrent-trunk coupling hypothesis — that predicting span shapes a more anticipatory
  hidden state the controller also reads — is **not bearing out**; t6 sits exactly on the predictor-less
  baseline.
- **Caveat:** gen ~318/800, pctInStreak ~8% — the **back-half variation-ramp** is the one remaining wildcard
  (has kicked late takeoffs before: [project_no_future_curve_shape], [project_late_run_fitness_interpretation]).
  Operator: let it run.
- **Next-idea seeds** (operator thinking): (1) **spherical/equidistant projection** so `span` (|gap|) is
  position-invariant — remove ego-contamination from the predicted signal (pulled forward from 040, see
  BACKLOG top). (2) The deeper question this run raises: does prediction have to be **actuated / consumed by
  control** (a two-loop or lead-computed input), not just a passive scored aux output, to change behavior at
  all — i.e. is a *passive* forward-model objective structurally too weak to close the perception→control
  loop.

### t7 shared-source-env A/B — result @ gen 193 (STOPPED 2026-07-08): env fidelity ruled out as the ceiling cause

**t7 = `autoc-038-t7-m2-shared-env`** (`autoc-m2/autoc-9223370253324834099-2026-07-08T16:59:01.708Z/`,
master seed `1783529939`, `TrackerChaseUseSourceScenarioSeed=1`, same t5 gen800 source + predictor config as
t6 — the ONLY difference vs t6 is shared vs independent env seeds). Stopped at gen 193, past t6's plateau
onset (~gen 130), so the competence A/B is conclusive for this config. PNGs:
`autoc-038-t7-m2-shared-env_*.png` (`--compare t6-indep-env`).

- **Competence: t7 == t6 == t4.** All four mode metrics converged onto the same plateau — perception ~0.70,
  track occupancy ~0.095, median range ~17 m, worst-blind ~7–8 s. Best fitness −13474 @ 193 ≈ t6's −13382 @
  344 (and fitness is confounded anyway — different realized scenarios). **Sharing the source's airspace does
  not lift the tracking ceiling.**
- **Predictor: t7 ≈ t6.** Per-horizon error the same noisy ~0.55–0.75 plateau, span calibration still a
  vertical spray, closure-rate still pinned at the ±1 tanh rails (realized ±7). A consistent air mass does
  not make the passive span head learn either.
- **What shared env DID buy (early)**: faster perception climb (~0.65 by gen 65 vs t6 ~0.58) and a much lower
  early crash rate (14.6% @ gen 80 vs t6 ~59% @ gen 47) — the chase no longer dies to wind the target never
  flew. Real, but transient: it changes the on-ramp, not the ceiling. Worth keeping ON for realism/cleanliness.
- **Verdict**: the "good tracking but in beyond-physics envs" confound is **ruled out as the bottleneck** for
  both depth and prediction. The M2 cap remains perception/architecture-limited — pointing back at
  spherical-projection (clean observable) + actuated-prediction (close the loop).
- **Known gap in t7's "sameness" (drives t8)**: t7 shared the *seed* but the chase still applied the M2
  **variation-ramp scale** (≈0 early → 1 late), while the source flew at scale ≈1.0 — so the chase's realized
  env only truly matched the source's late in the run. **t8 = t7 + `VariationRampStep=0`**
  (`computeVariationScale`: ramp=0 ⇒ numSteps=0 ⇒ scale **pinned 1.0 from gen 1**) — the actually-identical
  shared env, full magnitude from the start, at the cost of losing the easy-early curriculum (expect a harder
  start / higher early crash; the ramp is a search-time nudge, not the objective, per
  [feedback_clear_objectives_not_tuning]).

### t8 full-ramp shared-env — result @ gen 240 (STOPPED 2026-07-09): curriculum ramp ruled out too; same shelf

**t8 = `autoc-038-t8-m2-shared-env-fullramp`** (`autoc-m2/autoc-9223370253286427619-2026-07-09T03:39:08.188Z/`,
master seed `1783568345`) = t7 + `VariationRampStep=0` (scale pinned 1.0 from gen 1 — the chase flies the
source's *actually identical* full-magnitude air mass the whole run; t7 shared the seed but its ramp scaled
the applied env down early). Stopped at gen 240 (best −13066.93, avgMaxStreak 21.8, pctInStreak 7.4% — still
slowly improving, but spherical projection was judged the better use of the cycles). PNGs:
`autoc-038-t8-m2-shared-env-fullramp_*.png` (`--compare t7-ramp40 + t6-indep-env`).

- **Same shelf, again**: perception ~0.76 (top edge of the t4/t6/t7 band, reached ~80 gens sooner), median
  range ~18–20 m, worst-blind ~7.5 s, occupancy ~0.08 (the one lag — holding streaks in full gusts is
  harder). Best fitness under FULL difficulty ≈ t7's under ~21% difficulty — the curriculum ramp is not
  earning its keep in this config (gen-1 crash was only 24% at full env).
- **W_hh finding** (matched-age vs t7, single-variable): full-magnitude turbulence from gen 1 selected a
  ~1.5–2× **uniformly amplified** recurrent spectrum (ρ 2.48→3.24) with IDENTICAL eff-rank (11.1 vs 11.2/16)
  and unchanged whh/xh balance (~0.49) — gain up, structure unchanged. Capacity is not the binding
  constraint; the same-sized brain just learned to fly harder air (and smoother: stability −16.5k vs t7's
  −36.5k, energy 41.7k vs 53.9k).
- **Predictor persistence-baseline finding (2026-07-09, panel upgrade)**: the "no-change" bar sits at
  ≈0.01 NDC for all horizons — the head is ~50× worse than persistence AND the ≤150 ms span-prediction task
  itself has ~no information content (persistence is below the σ-floor); the CEP gate also excludes blind
  gaps — the one regime where prediction would pay. See BACKLOG "US3 predictor VERDICT + re-target design".
- **AMENDMENT (operator, 2026-07-09) — first predictor signal, in CLOSURE RATE**: the gen-240 closure-rate
  calibration is no longer a vertical blob — the cloud **leans along y=x** within the representable ±1 band,
  and late-run per-horizon errors bent down (+150 ms ~0.45–0.5 around gens 200–240, closure-rate error
  dipping to ~0.45–0.55 from its ~0.7 plateau). Still far from the persistence bar, but it is the first
  structure any predictor head has shown. **With the t9 spherical (angular-span) target + a long run this
  might get interesting** — a keep-watching reason for leaving the head AS-IS in t9.
- **AMENDMENT (operator, 2026-07-09) — gen_runtime watch item**: t8's per-gen wall-clock rose to ~235 s/gen
  by gen ~85 then went **flat through 240**, while t6/t7 kept climbing (~310–330 s/gen at matched gens,
  t6 reaching ~400 by gen 345). Rising runtime = the population broadly surviving longer; t8's early flat
  plateau at a LOWER level reads as **either rapid progress saturating early (elite fine, average
  individuals dying fast under full-magnitude env) or a brittle low-diversity population** — the flat line
  says population-wide survival stopped improving after ~gen 90 even as elite fitness kept creeping.
  Watch this on t9 (same full-ramp config): if t9's curve also flattens low, consider it a
  full-difficulty-from-gen-1 population-health cost to weigh against the faster perception on-ramp.
- **038 arc closed on the env axis**: t7 (seeds) + t8 (seeds+magnitude) rule out env fidelity AND the
  curriculum ramp as ceiling causes. Eliminated across 038: reward shaping (037 t11–t15), env fidelity,
  situational sensors, passive predictor, recurrent capacity. **t9 = spherical/equidistant projection**
  (predictor head kept AS-IS per operator — watch for any signal against the persistence bar), likely the
  038 wrap; remaining levers (streak/envelope input, predictor re-target, US2) route to BACKLOG for the
  next feature.

### t5 M1 source rebake — PINNED `retain=keep` (2026-07-08): the M2 source library for t6/t7/t8

**t5 = `autoc-038-t5-m1-source-rebake`** (enriched 37-input M1, pop 5000, 20 Hz, 6×49=294, 800 gens).
**S3 pin (Constitution VIII)**: `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` (800 dmps
tagged `retain=keep` 2026-07-08), master seed `1783301746`. Gen 800 elite: best **−41083.52**,
avgMaxStreak 61.5, pctInStreak 30.9%, 0 hull, 294/294 complete. `gen9200.dmp.zst` (= gen 800) is the
**TrackerSourceRun for the entire US3/env-fidelity M2 series (t6, t7, t8)** — with
`TrackerChaseUseSourceScenarioSeed=1` (t7/t8) its per-scenario `scenarioSeed`s are also the *chase's*
env/craft/entry seeds, so this run is load-bearing beyond the trajectory itself. Pinned so the lifecycle
never expires it while the M2 series (and any future re-eval) references it.

### T013 (M1 half) — enriched M1 baseline complete, ≈ parity with best-ever (2026-07-04)

**t3 = `autoc-038-t3-m1-baseline`** (enriched full-M1, pop 5000, 20 Hz, aeroStandard 6×49=294, servo on,
37in/2051w, 800 gens, ~44.7 h). **S3 pin (Constitution VIII)**: `autoc-m1/autoc-9223370253844606963-2026-07-02T16:36:08.844Z/`,
master seed `1783010168`.

**Result — ≈ parity with the best-ever un-enriched M1 (037 t10):**

| final @800 (20 Hz) | best | streak | pctInStreak |
|---|---|---|---|
| t3 (37in, enriched) | −50,895 | 3.50 s (70.0) | 38.7% |
| t10 (33in, best-ever) | −52,568 | 3.75 s (74.9) | 38.6% |

~93% streak depth, **matched pctInStreak**, ~97% fitness. The +4 arena inputs cost ~100 gens of takeoff
delay (inflection ~gen 300 vs t10's ~gen 200) but **no meaningful ceiling loss** — minimal-regression thesis
CONFIRMED on M1. Second-order determinism holds (basic-m1 eval bit-match, T007 settled).

**Control quality (per_axis_time_series, t3 vs t10) — clean baseline, two mild watch-flags:**
Both trend |dCtrl| DOWN and converge near the 0.27 per-axis budget by end — **neither leaves an axis in hard
bang-bang**. Deltas vs t10: (1) **dominant bang-bang axis migrated pitch→roll** (t10 final dCtrl pitch ~0.30
busiest; t3 roll ~0.32 busiest) — both near budget, a mild lean not a runaway; (2) **t3's throttle is busier**
(dCtrl ~0.25 vs t10's ~0.16 — t10 kept throttle nearly flat-smooth), plausibly the `dist_to_boundary` input
driving more throttle response. Amplitude profiles near-identical (throttle sat ~0.68, roll drops ~0.45,
pitch ~0.67). **Verdict: clean enough to be the regression yardstick**; the two deltas are the things
ablations must not amplify.

**Process note**: the enriched `dmp-dump` **cannot read t10's 33-input dmps** (topology mismatch — greenfield
format break, working as designed). So t10's per-axis is the *committed 037 PNG* (visual reference only); the
ablation regression baseline is **t3** (same 037→038 format as everything downstream).

PNGs: `specs/038-accurate-m2/autoc-038-t3-m1-baseline_*.png` (`--compare 037-t10-best`).

### Pre-t3 de-risk — new inputs are real AND used, wind populated (2026-07-02)

Before committing the ~2-day t3 bake, verified from the existing basic-m1 (t2) dmps:

- **T008 wind getter ✓** — `wN/wE/wD` non-zero across all 6690 ticks of the latest gen (wN −4.0→0.1,
  wE −3.1→4.8; wD ~0 except 156 ticks — physically correct for steady horizontal wind + occasional
  thermal). `getLastLocalAirmass()` populates. [OP] flag on T008 resolved.
- **New-input VALUES real (not stubbed)** — `dBnd` (dist_to_boundary tanh) ranges 0.17→1.0 mean 0.98
  (drops toward the wall, saturates far away); `inward_body` is a **unit vector at every tick**
  (norm 1.000 min/max), components spread [−1,1]. Gather code correct end-to-end in a live run.
- **NN ATTENDS to the new inputs (not ignored)** — first-layer per-input weight L2 norm on the gen-400
  elite (`nnextractor`→`nn2cpp`, fitness −997): legacy-33 median 7.77 (4.9–10.4); new-4 = dBnd 6.99 (27th
  pct), inX 8.62 (76th), inY 8.53 (73rd), inZ 7.99 (55th). New-input mean norm 8.03 vs legacy 7.86
  (**ratio 1.02** — parity), all ~8× above Xavier init. The inward direction cosines land upper-half;
  dist_to_boundary lower-weighted (sensible for M1 rabbit-chase). **Enrichment is integrated, not dead
  weight** — the early-training tax is the cost of the bigger space, not disuse.

### Full-M1 enriched vs 037 — t1 was killed pre-takeoff (2026-07-02)

Compared the stopped full-M1 enriched rebake **t1** (`autoc-038-t1-m1-rebake`, pop 5000, 20 Hz,
aeroStandard 6×49=294, servo on, 37in/2051w, stopped gen 175) against the exact-shape 037 pair
(only 33in/1923w): **t10** (`autoc-037-t10-m1-20hz-08swin`, the pinned best, full 800 gens) and **t6**
(`autoc-037-t6-m1-20hz`, a same-config run that stalled). Overlay PNGs:
`autoc-038-t1-m1-rebake_*.png` (`--compare 037-t10-best --compare 037-t6-stall`).

| gen | t1 (37in) | t10 (33in **winner**) | t6 (33in staller) |
|----:|:---------:|:---------------------:|:-----------------:|
| 50  | 4.0 / 3.1 | 2.2 / 1.0 | — |
| 100 | 2.3 / 0.7 | 4.5 / 1.6 | 2.1 / 1.4 |
| 175 | **4.8 / 1.8** | **8.3 / 3.2** | **3.0 / 1.0** |
| 250 | — | 14.6 / 7.3 | — |
| 300 | — | 24.1 / 12.8 | — |
| 400 | — | 58.2 / 28.0 | — |
| 800 | — | **74.9 / 38.6** | — |
(cells = avgMaxStreak / pctInStreak; higher = better tracking)

**Findings**:
1. **t1 was killed deep in the pre-takeoff zone.** Even the *winning* un-enriched t10 was only at
   avgMaxStreak 8.3 @ gen 175 — its takeoff is a **gen 250→400 event** (8.3→24→58). Judging any M1 at
   gen 175 is judging before the inflection (matches [project_no_future_curve_shape] slow-start shape).
2. **t1 is mid-pack, not doomed.** @ gen 175 t1 (4.8) sits between staller t6 (3.0) and winner t10 (8.3);
   @ gen 50 t1 (4.0) was *ahead* of t10 (2.2). Enrichment tax is real but modest at full-M1 scale.
3. **Basin lottery is independent of enrichment.** t6 vs t10 = same 33in/pop-5000 config, different seed →
   one stalled, one took off ([project_m1_basin_lottery_actual_rate]). Two separate levers: **more gens**
   buys past the enrichment tax (takeoff is late anyway); **larger pop** buys down the basin lottery (more
   basins sampled) at the cost of slower per-gen crossover + wall-clock.

**Decision (2026-07-02)**: re-run the enriched full-M1 to the **full 800 gens at pop 5000 first** (t3) —
cheapest test of "does it just need to reach the takeoff window." Bump pop only if t3 stalls *through*
gen ~300 (basin miss). Determinism already confirmed (basic-m1 eval bit-match), so t3 is a budget test,
not a correctness one.

### T009b (FR-P0H situational-awareness inputs) — **PART 2/2 DONE: population** (2026-07-01)

Gather functions now populate every new field; clean `rebuild-perf.sh` (PERFORMANCE_BUILD) is **GREEN**
(autoc + crrcsim 100% + all `run_autoc_tests` gtests PASS). **T009b is complete** (layout + population).

**What landed (population):**
- **(B) `inwardBodyDirection(pos, q)`** — stateless helper in `arena.h`/`arena.cc:91`. World radial-inward
  `normalize(-x, -y, 0)` (Zero if `hypot(x,y)<1e-6`), rotated world→body via `q.inverse()`. All-attitude.
- **`gather_pathgen_inputs`** now takes `const FlightArena&`; populates (B) `dist_to_boundary` (tanh, same
  scale as tracker slot 44) + `inward_body_{x,y,z}`. M1 gets (B) only (never blind).
- **`gather_tracker_inputs`** now takes `const SituationalAwarenessState&`; populates (B) `inward_body` +
  writes (A) via `sa.writeInputs()`.
- **(A) `SituationalAwarenessState`** (new struct in `evaluator.h`) — `blind_ticks` + held `exit_dir_sin/cos`;
  single-sourced `update(left/right x/y/cep, kCepSentinelThreshold)` + `writeInputs()` (`time_since_seen =
  tanh(blind·dt / kTimeSinceSeenScale_s=2s)`). Visibility = CEP `< kCepSentinelThreshold` (matches
  `fitness_decomposition.cc:200`). Held member on **both** `TrackerStepper` and `CrrcsimTrackerHelper`;
  **reset in initScenario** (both), advanced each real tick after `projectAndShiftHistory` (NOT during the
  history pre-fill) → deterministic per FR-030.
- **`NNControllerBackend` ctor** gained `const FlightArena&` (no default per M2 policy). Call sites updated:
  crrcsim `inputdev_autoc.cpp:522` passes `init_.flightArena`; test fixtures pass `FlightArena{}`.
- **`mode.cc`**: `kPathgenMode.gather_inputs` → new `gather_pathgen_inputs_signature_mismatch_guard` (the
  bundle pointer is vestigial; real dispatch is via `NNControllerBackend::evaluate` / nn2cpp codegen).
- **`nn2cpp`**: both codegen paths bake a `static const FlightArena arena{R,F,C}` literal (new `-a R,F,C`
  flag, default 80/5/100 = struct defaults / autoc.ini M1 config) and pass it to `gather_pathgen_inputs`.
- **`dmp_dump`**: honest recording — reads the actual per-tick stored `TrackerInputs`/`NNInputs`
  (`st.getTrackerInputs()` / `getNNInputs()`), NOT a re-derivation. Tracker +6 cols
  (`inX,inY,inZ,tSee,exS,exC`); pathgen +4 (`dBnd,inX,inY,inZ`) in both path + no-path branches.
- **Fixtures regenerated to green** (part of this pass, satisfies T011's "confirm green"):
  `nn_sensor_interface_tests` (33→37 / 54→60 counts, new anchors, pathgen meta-walk header +4 cols),
  `contract_evaluator_tests` (1923→2051), `nn_evaluator_tests` (33→37), `gather_tracker_inputs_tests`
  (float[54]→[60]), all `gather_tracker_inputs` / backend-ctor call sites. New weight counts confirmed in
  `topology.h`: pathgen **2051** ("37,32,16r,3"), tracker **2787** ("60,32,16r,3").

**⚠️ Still open (NOT part of T009b):**
- **T009c** — dedicated behavioral + determinism unit test for the sit-awareness inputs (increment/reset on
  CEP crossing, inverted-attitude inward_body, per-scenario-reset determinism). Not yet written.
- **xiao firmware regen (FR-033)** — `xiao/src/generated/nn_program_generated.cpp` still calls the OLD
  `gather_pathgen_inputs` signature and the xiao PlatformIO build must add `arena.cc` (distanceToBoundary +
  inwardBodyDirection) to its sources. Firmware is out of the desktop CMake build, so this does NOT block
  `make`; it's operator/firmware work before the next live-pathgen flash. Regen via
  `nn2cpp -i <weights> -a <R,F,C>`.
- **T013 [OP]** — the enriched re-bake + payoff measurement is still gated on this landing (now unblocked).

### T005 / P0-E — type-domain grep audit → **AUDITED; conversions deferred** (2026-07-01)

Grep on `src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/` returns ~430 unannotated
`float`/`double` hits — the **codebase-wide backfill** that Constitution VI explicitly defers to a
separate audit-pass spec ("enforced incrementally on touched code"). 038 has not yet touched any
eval/nn code, so there is no 038-introduced drift. Type conversions are determinism-affecting and must
ride a `rebuild-perf.sh` gate — not blind Phase-1 edits. The per-milestone VI audit on 038-touched code
runs at T035.
