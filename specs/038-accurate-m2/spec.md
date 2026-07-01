# Feature Specification: 038 Accurate M2 — Architecture for Tracking Depth

**Feature Branch**: `038-accurate-m2`
**Created**: 2026-06-16 · **Rewritten**: 2026-06-22 (clean slate — re-centered on architecture per the
037 wrap handoff; the prior hull-penalty/camera-variation-headline draft and `t12-retro.md` are in git
history).
**Status**: CLARIFIED (2026-06-27) — big-picture deliverables settled (scope, parallel-ablation
sequencing, mixed M1-first policy, success gate; see Clarifications). Ready for `/speckit.plan`. The
remaining `[NEEDS RESEARCH]` markers (history layout, recurrence structure, predictor head/objective) are
intentional plan/research-phase outputs.
**Input**:
- Operator direction 2026-06-22: 038's headline is **RNN architecture, not sensors**. The 037 t11–t15 M2
  arc proved tracking depth is **architecture-capped, not reward-limited** — so the crash penalty (done)
  and reward-shaping (exhausted) are *carried mechanisms*, and the feature spine is **temporal memory +
  a target predictor**. Shape: **prework / tech-debt → RNN architecture studies → M2 prototyping.**
- Handoff document: [037 wrap.md](../037-20hz-control-loop/wrap.md) (§3 the finding, §5 the
  GENERIC-vs-FOV-SPECIFIC split, §6 the 038 bridge). Companion: `project_m2_tracking_ceiling`.
- Backlog source (`specs/BACKLOG.md`): M2-architecture / temporal-memory + predictor; "Standardize
  training reporting"; "Self-describing dmp"; 033 PRNG validation; camera/CEP realism; lexicase MAD-ε.

## The headline finding (037 t11–t15) — why this feature is about architecture

The 037 branch grew an M2 de-risking arc (t11 baseline → t12 hull penalty → t13 OOB-clamp collapse →
t14 curriculum-ramped penalty → t15 widened streak cone). **Every reward knob only relocated
trade-offs; the reward-invariant ground truth never moved:**

- **Close tracking ~11–13 % of ticks** (within 5 m), **median error ~17 m** — flat while elite fitness
  climbed −15k → −17k.
- **Perception plateaus ~70 % in-FOV** (≈30 % blind); **reacquire fails for 8–10 s** (`maxLost`
  160–200 ticks). Reward-invariant across runs (`mode_progress_t11_t14_t15.png`).
- **`rnn_capacity`: not width-saturated** (W_hh eff-rank ~10/16) → the limiter is **structure, not
  size**. 32r stays down-weighted.
- **The dominant failure (overrun) is ~87 % prediction-driven** (vis-loss + target-turn), only ~12 %
  pure speed.
- **Once close, track holds ~96 %** → the *track controller works*; the bottleneck is **acquire +
  reacquire-through-blindness**.

→ **Reward shaping is exhausted as a depth lever.** The next lever is architecture / perception. That is
the thesis of 038.

## Program context — three parallel tracks

| Track | Owner | Goal | Relationship to 038 |
|---|---|---|---|
| **xiao M1-flight** | **039** | xiao runs the 20 Hz loop in real flight (local-IMU, full NN unroll, defined latency). | Re-homed to 039 (was the 037-closeout parallel track). **Not 038.** |
| **M2 tracking depth** | **038 (this spec)** | break the architecture ceiling: temporal memory + target predictor (generic), then FOV-specific perception. | This feature. |
| **Camera design** | **031** | real beacon/camera/recording hardware → field-calibrated noise/CEP model. New **sim CEP rules expected to arrive from 031** (different host) and replace the off-axis placeholder. | 038 FOV-specific camera-variation work consumes 031's σ/CEP rules when they land; runs on parameterized σ until then. Not gated on 031. |

## The architecture split (GENERIC vs FOV-SPECIFIC) — the spine

From wrap §5. The crucial sequencing insight (operator 2026-06-21): **some of the architecture work is
generic and can be proven on M1 first with no FOV confound; some is strictly for the tracker's limited
camera FOV** (M1's rabbit is always visible). **Prove the generic temporal/predictor architecture
cleanly (M1 or both), then layer the FOV-specific perception/visibility work onto M2.**

**GENERIC — helps M1 *and* M2** (M1 also tracks a target → benefits from better temporal prediction):
- **Deeper / non-uniform (log / geometric) history buffer** — M1 and M2 currently share
  `kNNHistoryLagsMsec`; no reason they must.
- **Two-timescale recurrence** — a structural leaky-slow channel for trajectory memory (`rnn_capacity`
  says *structure, not width*).
- **Auxiliary target-predictor head** — trained via a **lexicase prediction-accuracy objective**
  (evolution-native, no backprop) or a gradient-pretrain-then-evolve hybrid. A short-horizon (~50 ms)
  next-tick predictor for anti-overrun; a *possible* substrate for later planning, not a multi-second oracle.

**M2-FOV-SPECIFIC — strictly the tracker's limited FOV** (does not apply to M1):
- **Visibility-maintenance reward (US4 — IN SCOPE)** — fly to keep the target framed; M1 never goes blind.
  The one FOV-specific deliverable in 038.
- **Prediction-through-blindness / reacquire (US5 — DEFERRED)** — the ~30 % blind, 8–10 s gaps; only a
  `time-since-seen` scalar survives optical flight (a stale bearing drifts with ego-rotation — folds into
  deeper gyro/attitude history, not a stored world position).
- **Camera variations (US5 — DEFERRED to a follow-on)** — per-scenario camera intrinsics/extrinsics/CEP
  draw for build-tolerance generalization; consumes 031's incoming CEP rules. Deferred: depends on the
  predictor ablation winning + 031's rules arriving.

**Carried mechanisms (NOT 038 headlines — already settled in 037):**
- **Hull-crash penalty** — **DONE.** t14's curriculum-ramped `0.75^K_hull × exp(−w·scale·K_oob/N)` is the
  validated mechanism (member-level multiplier; OOB smooth-cost; never-clamp; `applyCrashPenalty`
  all-paths). 038 carries it forward (weights tunable); it is not a feature deliverable here.
- **Reward-gradient shaping** (signed-ahead / streak-cone reshaping) — **exhausted as a depth lever**
  (t15). Survives only as an optional near-term complement to the architecture work, not the spine.

## Overview

M2 (the optical-only chase/tracker) tracks a recorded M1 source through the analytic beacon-projection
front-end, with craft variations (034) and the t14 crash penalty. It tracks safely but **shallow**: it
acquires late, loses the target ~30 % of the time, and cannot reacquire through blind gaps — and 037
proved **no reward knob moves these ceilings**. 038 attacks the ceiling at the **architecture** level, in
three phases:

1. **Phase 0 — prework / tech-debt.** PRNG validation, renderer config-hygiene, standardized reporting,
   and the one agreed clean-slate dmp break (since the architecture changes re-bake everything anyway).
2. **RNN architecture studies (generic).** Deeper/non-uniform history, two-timescale recurrence, and the
   auxiliary target-predictor head — run as **parallel independent ablations off the same baseline**
   (compare, then combine winners), with a **mixed M1-first policy** (history US1 gated on an M1 ablation
   first; predictor US3 / recurrence US2 may go straight to M2).
3. **M2 prototyping (FOV-specific) — scoped to US4 only.** The visibility-maintenance reward lands in 038.
   **US5 (camera variations + prediction-through-blindness / reacquire) is deferred to a follow-on** — it
   depends on US3's predictor and on 031's not-yet-arrived CEP rules.

This document is a **reframed skeleton**: the *specific* architecture choices (which history layout,
predictor-head shape and training objective, recurrence structure) are deliberately left to the plan/
research phase — see the `[NEEDS RESEARCH]` markers. The big-picture deliverable decisions (scope,
sequencing, M1-first policy, success gate) are settled in **Clarifications** below.

## Clarifications

### Session 2026-06-27

- Q: 038's deliverable boundary (how much M2 FOV-specific work lands here)? → A: **Generic (US1-US3) +
  US4 visibility reward.** US5 (camera variations + reacquire) is deferred to a follow-on (depends on
  US3's predictor and on 031's incoming CEP rules).
- Q: Which generic architecture study leads? → A: **Parallel independent ablations** off the same
  baseline (US1/US2/US3), then combine the winners.
- Q: Must each generic study be validated on M1 first before layering onto M2? → A: **Mixed** — M1-first
  is a required gate for history (US1); predictor (US3) / recurrence (US2) may go straight to M2.
- Q: What defines an 038 architecture win (primary acceptance metric)? → A: **Any one reward-invariant
  037 ceiling moves** (close-track %, median error, in-FOV %, or reacquire) — per SC-001.

### Session 2026-06-30

- Q: Run all three generic ablations (US1/US2/US3) in parallel, or pick a subset to start? → A: **Pick
  US1 + US3 to start.** A read of 037 leans "not enough *state*" — and M2 has long loss-of-signal windows
  (several seconds → effectively a re-acquire) — so the levers that add state win: **US1 deeper history**
  (longer trajectory context) + **US3 predictor head** (explicit motion model). **US2 (two-timescale
  recurrence) is deferred to `specs/BACKLOG.md` as a follow-on lever** — it overlaps US1 (both add temporal
  memory) and is the least-orthogonal of the three; pick it up only if US1+US3 don't move the ceiling or if
  more state is still the bottleneck. This refines (does not erase) the 2026-06-27 "parallel US1/US2/US3"
  decision — the parallel-then-combine *protocol* stands, the initial wave is now two arms not three.

### Session 2026-07-01

- Q: Does the (B) arena-inward situational-awareness input land on M1/pathgen too, or M2-tracker only? → A:
  **Both M1 and M2.** M1 is the clean/fast learner, so the arena sensor's value (and any efficiency gain in
  boundary behavior) shows up *early on M1* — the same M1-first signal logic as US1. M1/pathgen's NN input
  format therefore also breaks (bundled into the one P0-D break; M1 re-bakes and relearns with the cue).
  **(A) target-lost stays tracker-only** (M1's rabbit is always visible → `time_since_seen`/exit-side are
  meaningless for M1).
- Q: If the enriched baseline shows no measurable overrun/boundary improvement, keep the inputs or revert? →
  A: **Unconditional — keep regardless.** The inputs are cheap/safe/real-grounded and folded into the one
  P0-D break every ablation bakes on; reverting would force a second format break + full re-bake. A null
  result is recorded in `outcome.md` but triggers **no revert and no escape gate** (unlike the ablation
  levers). "Good to have in general" — baseline, not a lever.

## Sensor inventory & sim→real grounding (carried reference, operator review 2026-06-16)

All 54 M2 NN inputs are sim→real-grounded — ~83 % is unitless optical, every physical input is genuinely
available on the aircraft, and CEP is the one 031-fed model. **Architecture changes (history depth,
predictor head) alter how these inputs are *consumed over time*, not the input set** — except a
history-layout change is a format-breaking NN-input change (retrain from scratch, xiao contract update). The
one deliberate *input-set* growth in 038 is the **situational-awareness enrichment** below (a small set of
target-lost + arena-inward scalars, folded into the P0-D break) — baseline, not an architecture lever.

| Inputs (count) | Derivation | Units | Real-flight source |
|---|---|---|---|
| beacon L/R x,y (24) | world→NDC beacon centroid | NDC [−1,1] | camera (031) — position quality |
| beacon L/R CEP (12) | classic CEP — DSP position-uncertainty (decode/reacquire confidence + intermittency + crossing-rate over the ~150 ms code-acquisition window) | [0,1] + sentinel | 031 decode pipeline (sim = off-axis placeholder → **incoming 031 decode-time/intermittency/crossing-rate rules**) |
| beacon_pair_span (6) | NDC `sqrt(dx²+dy²)` | NDC | derived from beacon positions |
| span_rate (1) | NDC/s closure | ~unitless (NDC/s) | derived; finer at 20 Hz |
| tilt sin/cos (2) | `atan2` over NDC | [−1,1] | derived |
| quat w/x/y/z (4) | chase attitude | [−1,1] | AHRS (GPS-aided) |
| airspeed (1) | relVel / 13 m/s cruise | ≈[0,2] | AHRS/GPS velocity |
| gyro p/q/r (3) | body angular rates | rad/s (physical) | onboard IMU |
| dist_to_boundary (1) | `tanh(d/20m)` to arena cylinder, along velocity | [0,1) | deliberate crutch (anti-flyaway / patrol-inducing) |

Design constraints that survive the reframe: **M2 stays optical-only** (no physical target-range / hull
sensor — the predictor must infer range/closure from the two beacon points + span/span-rate, unitless);
**avoid direct physical-unit conversions**; the predictor-head and history work must keep every NN input
sim→real-grounded. Note the optical-only invariant constrains **target perception** — not the aircraft's own
ego/arena state (attitude, airspeed, arena geometry), which is legitimately GPS/AHRS-available.

## Situational-awareness input enrichment (baseline, folded into P0-D) — operator 2026-06-30

Two "which way to turn" needs that are **not** temporal-architecture problems (so not US1/US3 levers) but
input-representation ones — added to the **shared baseline** via the P0-D format break, so every ablation
inherits them. **No new reward tuning**: the existing penalty stack (tracking cone, t14 OOB smooth-cost +
hull penalty) already presses *stay on track / don't exit the arena / really don't hit the craft*; 038 just
gives the network the **sensory inputs to act on that pressure** and lets the (heavy-handed) evolutionary
engine arbitrate — no hand-tuned mode logic.

- **(A) Target-lost turn cue** — after the target drops out, *which way is a decent turn?* A `time_since_seen`
  scalar (non-decaying — the one cue that survives optical flight) + the **last-seen exit-side / image-velocity
  direction** (survives only briefly; the net learns to bank toward it and decay confidence via
  `time_since_seen`). A **decaying directional heuristic, NOT a stored world-position** — a stored bearing
  drifts with ego-rotation (which is *why* full 4–5 s prediction is off the table).
- **(B) Arena / boundary awareness** — phase-independent (patrol, intercept, track all need it), and applied
  to **both M1 and M2** (Clarifications 2026-07-01 — M1 is the clean/fast channel to read whether the sensor
  helps + improves boundary efficiency; M1's NN format breaks too, bundled into P0-D). Today the only arena
  input is `dist_to_boundary` (`tanh(d/20m)` along velocity): it says *how far to the wall ahead*, not *which
  way to bank to get back inside*. Add a **heading-to-inward `sin/cos`** pair (angle from velocity heading to
  arena center) — symmetric with the target bearing+span shape, so the net can learn one unified policy:
  "turn toward target **unless** the wall is close, then bank inward" — the emergent call-off / re-engage.
  Optionally **time-to-boundary** (`dist_to_boundary / airspeed`) for closure-awareness.

**Grounding & discipline**: (A) is optical-derived, (B) is GPS/AHRS ego-state (arena center is a fixed
geofence; position + heading are available) — nothing here is a magic target sensor, so **optical-only-for-
the-target** holds. Kept **rotation/translation-relative** (heading-to-inward *angle*, not raw `(x,y)`) to
preserve the minimal-crutch, anti-overfit philosophy. Current-tick / held-value scalars — **not** per-slot
historied — so the input-count growth is small. **Baseline enrichment, NOT an ablation lever**: measured as
enriched-baseline vs the old M2 baseline (does overrun drop / boundary behavior improve?), then US1/US3
ablate temporal depth on top of it. **Unconditional** — kept regardless of the measurement (a null result is
recorded, not reverted; no escape gate).

## Phase 0 — Prework / tech-debt (prerequisite, ahead of the architecture work)

Front-matter the operator wants landed first; each maps to an existing backlog thread; none is novel
feature scope, but they de-risk the bakes that follow. (Carried from the prior draft — these are settled.)

- **P0-A — 033 PRNG validation.** Confirm the 033 PRNG-cascade rework (`acf732f`) deterministically
  regenerates a scenario from its seeds. Clear it or find the suspected single-SHA bug. Ref:
  `project_v15_determinism_candidates`.
- **P0-B — Renderer reads less from `.ini`.** Source fitness/cadence params for replay from the run/dmp,
  not the live `.ini`, so a drifted ini can't misrender an old run. Scope to what the renderer needs now.
- **P0-C — Standardized report script.** One `scripts/` wrapper taking a logfile name → emits the full
  PNG report set (derives run-id/bucket/gen/mode from the log head). Folds the dmp-fed plotters into a
  maintained `src/analytics/` home with `pyproject`/`requirements`. (Largely landed in 037 — the
  `scripts/generate_pngs.sh m2 <log>` → 11-report toolkit; this finishes/formalizes it.)
- **P0-D — Clean-slate contract changes (ONE dmp break).** The architecture studies re-bake M1/M2 anyway,
  so this is the agreed moment to land the deferred rule-changing items together — one dmp/determinism
  break, retrain-from-scratch, no version bump (readers fail-loud). Absorbs:
  - **simTimeMsec stamping** — round / derive from step-count so 20 Hz records exact 50 ms gaps (cleans
    the history-lag + `span_rate`-denominator jitter; lets the M2 source-spacing check revert to strict).
  - **Self-describing dmp** — serialize the fitness/cadence config block into `EvalResults` (the FULL
    version beyond P0-B's renderer slice), so a dmp is standalone-replayable.
  - **`wind_velocity` recording** — populate from crrcsim at record time (today zero), so realized gusty
    wind is auditable; re-run `wind_study.py` against real wind.
  - **Situational-awareness inputs (operator 2026-06-30)** — the (A) target-lost + (B) arena-inward baseline
    scalars (see "Situational-awareness input enrichment" above). A format-breaking NN-input *add* — folded
    into this one break, real-grounded, no reward tuning, current-tick/held-value (not historied).

  (NOT the V1.5 crash-hull PRNG determinism — already fixed in the 033 cleanup; don't re-open.)

  **Expected consequence (operator 2026-06-22)**: any prework that changes **M1 architecture** (P0-D's
  format-breaking inputs, or a generic history-layout study landed as prework) will **drive a fresh M1
  source re-bake** — that is **accepted**, not a blocker. The M2 source library re-bakes from the new M1
  baseline; budget an M1 run whenever prework touches M1 arch.

- **P0-E — 037 close-out housekeeping (carried forward 2026-06-21, BACKLOG "037 close-out").** Three
  small non-blocking tech-debt items filed at the 037 wrap; cheap to clear at the front of 038:
  - **svTau cleanup (P-O8)** — `svTau` is kept only to preserve draw order and doesn't help; remove the
    dead path.
  - **Time-denominate the rate-dependent reports (P-O11)** — raw tick-denominated streak metrics read
    2× at 20 Hz; partly handled by `--tick-sec`, but surface streak / avgMaxStreak in seconds (or
    pctInStreak) consistently. Reporting hygiene — pairs with P0-C.
  - **Type-domain grep audit (T028/T046, Principle VI)** — grep `src/eval/ src/nn/` for float/double
    drift on the 037-touched paths; confirm `gp_scalar`/`gp_fitness` convention. Also covers the
    `EnableHullCrashPenalty`-flag-vs-parameter config-convention nit (BACKLOG L146).

- **P0-F — Revert the t15 streak threshold (0.3 → 0.5).** t15 widened the streak cone
  (`FitStreakThreshold` 0.5 → 0.3) probing reward-shaping as a depth lever; 037 concluded that lever is
  exhausted (the gain was threshold-confounded — error distance unchanged). Restore the 0.5 baseline in
  the three tracker inis (`autoc-tracker.ini`, `autoc-eval-tracker.ini`, `autoc-eval-tracker-visual.ini`;
  M1 inis are already 0.5) so the architecture studies start from a clean, un-confounded cone. (Reward
  shaping stays available behind the ini switch per FR-021, but 0.5 is the default going in.)

**Carried backlog (support, not core prework)**: `EvalVariationScaleOverride` ini knob (eval at a forced
variation scale for robustness-vs-repeatability — wrap §6 "open backlog carried") lands if/when the
reporting+eval work touches it; it doesn't gate the architecture studies. (The per-gen SQLite analytics
store stays in `specs/BACKLOG.md` as a P0-C follow-on — deliberately not 038 prework.)

**Phase-0 acceptance**: P0-A produces a written verdict (clear / bug-found-and-fixed) + determinism
check; P0-B/P0-D let the renderer replay a pinned run without the live `.ini`; P0-C reproduces today's M2
PNG set from one `scripts/<wrapper> <logfile>` call; P0-D bakes a fresh source with exact 50 ms gaps,
recorded wind, and a self-describing dmp — the strict source-spacing check passes.

## User Scenarios & Testing

> **Scope and sequencing settled (Clarifications 2026-06-27, refined 2026-06-30):** 038 = generic studies
> run as parallel independent ablations off the same baseline (combine winners) **+ US4** visibility reward.
> **Initial wave = US1 (deeper history) + US3 (predictor head)** — the "add state" levers picked for M2's
> long loss-of-signal windows; **US2 (two-timescale recurrence) is deferred to `specs/BACKLOG.md`** as a
> follow-on lever (overlaps US1). **M1-first is mixed**: required for US1 (history), optional for US3.
> **US5 is deferred to a follow-on.** Success gate = any one 037 ceiling moves (SC-001).

### US1 — Deeper / non-uniform temporal history buffer (GENERIC, M1-first gate) (Priority: parallel ablation)

As the operator, I want the NN history buffer to be **deeper and non-uniformly spaced** (log / geometric
lags, possibly toward the 1.6 s window, possibly more than 6 steps) so the controller has richer
**derivative/closure context** — lagged copies of the inputs let the RNN compute its own rates (span-rate,
turn-rate, closure) and behave like an emergent PD/PID loop. **This is a short-horizon anti-overrun lever,
NOT a 4–5 s target oracle**: in high-rate all-attitude maneuvering you *cannot* predict a target's position
seconds out, and the buffer is deliberately **not** sized to bridge the multi-second blind gap (that would be
prediction-through-blindness, deferred US5; the turn-at-loss cue is the situational-awareness
`time_since_seen`/exit-side scalars, not history depth). 1.6 s is a compromise giving more derivative context,
not a claim of gap-spanning sufficiency. Provable on **M1 first** (no FOV confound), then inherited by M2.

**Sequencing**: the cheapest structural lever and a clean generic test, run as a parallel ablation —
**but with a required M1-first gate** (validate on M1, no FOV confound, before layering onto M2). M1 and
M2 currently **share** `kNNHistoryLagsMsec` ([−0.8, −0.4, −0.2, −0.1, −0.05, now], 6 slots, t10 0.8 s);
there is no reason they must, and `rnn_capacity` says the limiter is *structure*, not width.

**Independent Test**: a bake with an alt history layout (deeper window / more steps / log-or-geometric
spacing) vs the t10-matched layout at the same seed; the alt shows **equal-or-better tracking**
(pctInStreak / median error) and/or cleaner regime transitions on M1, determinism + bitwise replay
preserved. [NEEDS RESEARCH: depth, step count, spacing scheme — judged by tracking + regime-transition
quality.]

**Acceptance Scenarios**:
1. **Given** an alt history layout distinct from t10's, **When** a bake runs, **Then** it is honored
   deterministically and replays bitwise.
2. **Given** a layout change, **When** it lands, **Then** genomes retrain from scratch (format-breaking
   NN-input change; no cereal version bump; xiao firmware contract updated) per P0-D's one break.
3. **Given** the M1 result, **When** the layout helps M1, **Then** M2 inherits it as the new shared
   (or M2-specific) baseline.

---

### US2 — Two-timescale recurrence (structural slow channel) (GENERIC) — DEFERRED-TO-START (follow-on lever)

> **Deferred out of the initial ablation wave (Clarifications 2026-06-30).** Recorded in `specs/BACKLOG.md`
> as a follow-on lever, not a starting deliverable: it overlaps US1 (both add temporal memory) and is the
> least-orthogonal of the three generic studies. Unparks if US1+US3 don't move a ceiling, or if "not enough
> state" persists as the bottleneck after they land. The starting config + format surface stay recorded
> below and in research.md §2 so the arm can launch off the same post-P0-D baseline when picked up.

As the operator, I want a **structural leaky-slow recurrent channel** for trajectory memory alongside the
existing fast hidden state, so the NN can hold a longer-horizon motion model without widening W_hh
(`rnn_capacity` says structure, not width — eff-rank ~10/16, 32r down-weighted).

**Sequencing**: directly addresses the "structure not size" finding; runs as a parallel ablation and
**may go straight to M2** (M1-first not required here — per the mixed policy).

**Independent Test**: a bake with the two-timescale recurrence vs the single-timescale RNN at the same
seed; the two-timescale variant shows better tracking and/or `rnn_capacity` eff-rank usage at equal or
fewer total recurrent parameters, determinism + bitwise replay preserved. [NEEDS RESEARCH: channel
structure (fixed-leak vs evolved time-constant), how it composes with the predictor head (US3).]

---

### US3 — Auxiliary target-predictor head (GENERIC, the pivot, M2-direct allowed) (Priority: parallel ablation)

As the operator, I want an **auxiliary head that predicts the target's *next-tick* optical state** (the two
beacon points / span at +1 tick ≈ 50 ms), trained by a **lexicase prediction-accuracy objective**
(evolution-native, no backprop) or a gradient-pretrain-then-evolve hybrid, so the controller carries a
**short-horizon closure estimate it can act on to stop overrunning** ("am I about to overtake?"). **Scoped as
a ~50 ms lookahead anti-overrun aid, not a multi-second motion model** — 4–5 s prediction is physically off
the table (see US1); any reacquire-through-blindness benefit is a *possible* downstream bonus, not the claim
here (that is deferred US5).

**Sequencing**: the highest-leverage architecture change — it sharpens "track" into "anticipate-closure";
generic (a next-tick predictor helps M1 too) but **may go straight to M2** (M1-first optional per the mixed
policy). Highest research risk. Runs as a parallel ablation.

**Independent Test**: a bake with the predictor head + prediction objective vs the control-only baseline
at the same seed; the predicted optical state matches the realized state within a target error AND
tracking depth improves (or the predictor measurably aids reacquire on M2), determinism + replay
preserved. [NEEDS RESEARCH: head shape, prediction target + horizon, lexicase prediction-accuracy
objective design (must compose with the existing tracking lexicase axes without re-creating the 033
scalar-collapse), pretrain-then-evolve vs pure-evolve.]

---

### US4 — Visibility-maintenance reward (M2-FOV-SPECIFIC, IN SCOPE) (Priority: P2)

As the operator, I want a fitness term that **rewards keeping the target framed** (in-FOV / low CEP), so
the chase learns to fly geometries that *maintain perception* rather than blundering into blind gaps —
strictly an M2/FOV concern (M1 never goes blind). The one FOV-specific deliverable in 038 (US5 deferred).

**Why this priority**: directly attacks the ~30 % blind / 8–10 s reacquire ceiling; it is a reward-term
change that can proceed **independently of** the parallel architecture ablations (does not gate on the
predictor head landing first). **Must stay optically-/geometrically-derived and unitless** and be
validated to not destabilize lexicase selection. [NEEDS RESEARCH: reward shape; whether the predictor
ablation (US3), if it wins, reduces the need for an explicit visibility reward or complements it.]

---

### US5 — Camera variations + prediction-through-blindness (M2-FOV-SPECIFIC) — DEFERRED to a follow-on

> **DEFERRED out of 038 (Clarifications 2026-06-27).** Not a 038 deliverable: it depends on US3's
> predictor (only available once that ablation wins) **and** on 031's not-yet-arrived CEP rules. Kept here
> as the scoped follow-on so the dependency and the reserved insertion point are recorded; it unparks when
> the predictor lands and 031 publishes its CEP model.

As the operator, I want (a) **per-scenario camera variation** (intrinsics/extrinsics/CEP within σ) so M2
generalizes across the real build-tolerance envelope, and (b) **prediction-through-blindness / reacquire**
behavior so the chase recovers a lost target via the predictor + a `time-since-seen` scalar rather than
flailing — the FOV-specific perception layer that consumes 031's incoming CEP rules.

**Why deferred**: generalization + the reacquire failure mode, but it depends on US3's predictor and on
031's CEP model maturing. The `cameraSeed` insertion point is reserved at
[scenario_metadata.h:98](../../include/autoc/rpc/scenario_metadata.h#L98) for the follow-on.

**Independent Test**: (a) `EnableCameraVariations=0` ⇒ bit-identical to pre-038; on ⇒ per-scenario camera
params deterministic from `cameraSeed`, bitwise on replay, and a varied-camera controller tracks a
held-out novel camera draw at least as well as a fixed-camera controller tracks nominal. (b) On a
target-loss event, the predictor-driven controller reacquires faster / with shorter `maxLost` than the
reactive baseline. [NEEDS CLARIFICATION: σ list + defaults — seed from 031 `camera_considerations.md` /
the incoming CEP rules; position-quality vs signal-quality (CEP = dropout/confidence, not position
jitter) as distinct axes; spike on source-side-vs-M2-side camera variation (M2-side expected — the M1
source is blind to the chase camera).]

---

### Edge Cases

- **History/predictor as format breaks**: US1 (and any input the predictor head adds/reshapes) is a
  format-breaking NN-input change — fold into P0-D's single dmp break; retrain from scratch; update the
  xiao firmware contract; no cereal version bump.
- **Predictor objective vs lexicase**: a prediction-accuracy objective must compose with the existing
  tracking lexicase axes **without** re-creating the 033 scalar-objective Pareto collapse
  (`project_scalar_multiobjective_collapse`) — default to a separate lexicase axis or staged objective,
  not scalar compositing.
- **Optical-only invariant**: the predictor must infer range/closure from the two beacon points + span /
  span-rate (unitless); **no** physical target-range / hull-proximity sensor (FR carried from the prior
  draft). A from-behind overshoot stays unperceivable; the predictor + visibility reward select against
  the geometry, they don't add a sensor.
- **Camera variation + visibility**: a draw must not degenerate to all-beacons-blind purely from camera
  variation; clamp/validate.
- **Determinism**: every new PRNG draw (`cameraPRNG`) and every new NN state preserves absolute
  per-scenario determinism + eval-vs-training bitwise parity (the regression gate).
- **Cross-run comparability**: architecture changes move the fitness scale; ablation/baseline comparisons
  use a fixed-eval comparator, not raw late-run training fitness (`project_late_run_fitness_interpretation`).

## Requirements

> Lighter than a post-clarify spec on purpose. FR-numbers are placeholders to be firmed up after clarify.

### Phase 0 — Prework (prerequisite)
- **FR-P0A**: 033 PRNG cascade MUST be validated to deterministically regenerate a scenario from its
  seeds; outcome (clear / fixed) recorded with a determinism check.
- **FR-P0B**: The renderer/replay path MUST source fitness/cadence params from the run/dmp rather than the
  live `.ini` for replaying a pinned run (scope: renderer's current needs).
- **FR-P0C**: A single `scripts/` wrapper taking a logfile MUST regenerate the full PNG report set;
  plotters live in a maintained `src/analytics/` home with `pyproject`/`requirements`.
- **FR-P0D**: The one clean-slate dmp break MUST land simTimeMsec stamping (exact 50 ms gaps),
  self-describing dmp (fitness/cadence config in `EvalResults`), and `wind_velocity` recording — together,
  no cereal version bump, readers fail-loud, retrain-from-scratch.
- **FR-P0E**: The 037 carry-forward housekeeping (svTau dead-path removal, time-denominated rate-dependent
  reports, `src/eval/ src/nn/` type-domain grep audit + config flag/parameter convention) SHOULD be
  cleared at the front of 038; non-blocking but cheap.
- **FR-P0F**: The t15 streak threshold MUST be reverted to the 0.5 baseline in the tracker inis (reward
  shaping stays switchable per FR-021, but 0.5 is the default going into the architecture studies).
- **FR-P0G**: Prework that changes M1 architecture MUST be expected to trigger a fresh M1 source re-bake
  (and the M2 library re-bake from it); this is accepted scope, budgeted as an M1 run.
- **FR-P0H**: The P0-D break MUST also land the **situational-awareness input enrichment** (baseline, NOT an
  ablation lever): **(A)** a `time_since_seen` scalar + a last-seen exit-side / image-velocity cue
  (optical-derived; a decaying heuristic, no stored world position) — **tracker-only** (meaningless for M1's
  always-visible rabbit); and **(B)** a heading-to-inward `sin/cos` pair (± `time_to_boundary`) for
  arena/boundary turn guidance (GPS/AHRS ego-state, rotation/translation-relative — NOT raw `(x,y)`) — on
  **both M1/pathgen and M2/tracker** (Clarifications 2026-07-01; both NN input formats break, bundled into the
  one P0-D break). It MUST stay determinism/bitwise-replay clean, add **NO** new reward tuning (the existing
  penalty stack arbitrates), keep the **optical-only-for-the-target** invariant, be honestly recorded in the
  dmp (surfaced by `dmp_dump`), and be measured as enriched-baseline vs the old baseline (M1 and M2 overrun /
  boundary behavior — early M1 read on the arena cue). It is **unconditional** (Clarifications 2026-07-01):
  kept regardless of the measurement; a null result is recorded but triggers no revert and no escape gate.

### RNN architecture studies (generic — US1/US2/US3)
- **FR-001**: The M2 (and M1) history layout — window depth, step count, lag spacing — MAY differ from
  today's shared `kNNHistoryLagsMsec`; a layout change is format-breaking (retrain, xiao contract update,
  no version bump) and MUST replay bitwise. [NEEDS RESEARCH: depth / count / spacing.]
- **FR-002** *(deferred-to-start — follow-on lever per Clarifications 2026-06-30)*: A two-timescale
  recurrence structure MAY be added without widening W_hh; it MUST preserve determinism + bitwise replay.
  Not in the initial ablation wave (overlaps FR-001); recorded for unpark. [NEEDS RESEARCH: structure.]
- **FR-003**: An auxiliary target-predictor head MAY be added, trained via a lexicase prediction-accuracy
  objective (or pretrain-then-evolve hybrid) that composes with the existing tracking lexicase axes
  WITHOUT scalar-objective compositing (sidestep the 033 collapse). [NEEDS RESEARCH: head + objective.]
- **FR-004**: The generic studies MUST be run as **parallel independent ablations** off the same baseline,
  then combined by winner. **The initial wave is FR-001 (history) + FR-003 (predictor)** — the "add state"
  levers (Clarifications 2026-06-30); **FR-002 (two-timescale recurrence) is deferred to `specs/BACKLOG.md`**
  as a follow-on lever and is NOT launched in the initial wave. **M1-first is mixed**: FR-001 (history) MUST
  be validated on an M1 ablation (no FOV confound) before layering onto M2; FR-003 (predictor) MAY go
  straight to M2. FR-002's starting config stays recorded (research.md §2) so it can launch off the same
  baseline if unparked.

### M2 prototyping (FOV-specific — US4 in scope)
- **FR-010**: A visibility-maintenance reward MAY be added (the one FOV-specific 038 deliverable); MUST
  stay optically-/geometrically-derived and unitless and MUST NOT destabilize lexicase selection. It MAY
  proceed independently of the architecture ablations (does not gate on the predictor landing).
  [NEEDS RESEARCH: shape.]

### M2 prototyping (FOV-specific — US5 DEFERRED to a follow-on)
> Recorded for the follow-on, not 038 deliverables (depend on FR-003's predictor + 031's incoming CEP).
- **FR-011** *(deferred)*: Add `cameraSeed` to `ScenarioMetadata` at the reserved insertion point + a
  `cameraPRNG` cascade slot (craft pattern); per-scenario camera draws across **position quality**
  (centroid/intrinsics/extrinsics) and **signal quality** (CEP) MUST be deterministic, ramped via
  `applyVariationScale`, gated by `EnableCameraVariations` (off ⇒ bit-identical), bitwise-replayable,
  clamped against all-blind, with σ defaults config-resident, consuming 031's incoming CEP rules.
- **FR-012** *(deferred)*: Prediction-through-blindness / reacquire MAY use the predictor head + a
  `time-since-seen` scalar (no stored world-frame target position). MUST stay optical-only.

### Carried mechanisms (settled in 037 — not feature deliverables)
- **FR-020**: The t14 hull/OOB crash penalty (`0.75^K_hull × exp(−w·scale·K_oob/N)`, member-level,
  never-clamp, `applyCrashPenalty` all-paths) is CARRIED FORWARD behind its enable knob; weights are
  tunable but the mechanism is DONE — not re-derived here.
- **FR-021**: Reward-gradient shaping (signed-ahead / streak-cone) is OPTIONAL near-term complement only;
  it is NOT the depth lever (037 proved it exhausted). Any use stays behind an ini switch defaulting to the
  historical cone.

### Cross-cutting
- **FR-030**: All new state (predictor, history layout, `cameraPRNG`, crash-cost) MUST preserve absolute
  per-scenario determinism and eval-vs-training bitwise parity (the regression gate).
- **FR-031**: Schema changes follow no-cereal-versioning / fail-loud practice (greenfield, no version bump).
- **FR-032**: Lexicase MAD-relative epsilon stays available behind an ini switch (constant-0.5 default for
  reproducibility); promote only if the added objective dimensionality degrades selection signal
  (`project_lexicase_mad_epsilon`).
- **FR-033**: Any format-breaking NN-input/output/topology/cadence change MUST keep the xiao firmware
  contract coherent (regen `nn2cpp` codegen, rebuild `pio`, preserve the first-3-outputs-for-control
  convention) and the update MUST be recorded in the outcome doc — even though the tracker port itself is
  deferred (BACKLOG). The desktop/codegen contract stays correct so the deferred port inherits 038's
  architecture cleanly. See [contracts/xiao-nn-sync.md](contracts/xiao-nn-sync.md).

### Key Entities
- **History layout (revised)** — depth/count/spacing constant(s), possibly M2-specific; format-breaking.
- **Two-timescale recurrent state (new)** — slow channel alongside the fast hidden state.
- **Target-predictor head (new)** — auxiliary NN head + its prediction-accuracy objective.
- **`cameraSeed` / `cameraPRNG` (new)** — per-scenario camera-variation PRNG root + cascade slot.
- **Crash-penalty machinery (carried)** — t14 member-level multiplier; OOB smooth-cost.
- **Camera (existing)** — `camera_projection.cc` analytic projection + the incoming-031 CEP model.

## Success Criteria

> Anchored on the architecture thesis: the win is moving the **reward-invariant ceilings** 037 measured.

- **SC-000**: Phase 0 done — 033 PRNG verdict + determinism check; renderer replays a pinned run without
  the live `.ini`'s fitness params; the full M2 PNG set regenerates from one `scripts/<wrapper> <logfile>`
  call; a fresh source bakes with exact 50 ms gaps, recorded wind, self-describing dmp; **and the FR-P0H
  situational-awareness inputs are landed (M1+M2 baked on the enriched format, unit test green) with the
  enriched-vs-old overrun/boundary read recorded** (unconditional — landed regardless of the read).
- **SC-001 (PRIMARY GATE)**: At least one of the parallel architecture ablations (history / two-timescale /
  predictor) moves **at least one reward-invariant 037 ceiling** — close-tracking fraction up from
  ~11–13 %, median error down from ~17 m, in-FOV up from ~70 %, or reacquire (`maxLost`) shorter than
  8–10 s. The history study (US1) result MUST be demonstrated on an M1 ablation first; recurrence/predictor
  may show the lift directly on M2.
- **SC-002 (supporting)**: The auxiliary predictor's predicted optical state matches the realized
  next-tick state within a target error — the threshold is set at the US3 gate, but the comparator is
  concrete: aux NDC error must beat the naive **persistence baseline** (predict "next = current") on the
  same scenarios, i.e. the head demonstrably models motion rather than echoing the present. AND its
  presence improves tracking depth and/or reacquire (vs the control-only baseline). Supporting evidence for
  SC-001, not the primary gate.
- **SC-003**: The US4 visibility-maintenance reward, when enabled, improves in-FOV fraction / shortens
  reacquire without destabilizing lexicase selection or regressing the carried crash penalty.
- **SC-003b** *(deferred with US5)*: `EnableCameraVariations=0` ⇒ bit-identical to pre-038; on ⇒
  deterministic from `cameraSeed`, bitwise on replay; varied-camera M2 generalizes to a held-out camera.
- **SC-004**: The carried t14 crash penalty still holds hull-strikes ~0 with no population collapse under
  the new architecture (regression, not a new deliverable).
- **SC-005**: The eval-vs-training bitwise regression gate passes with all 038 schema/PRNG/NN changes.

## Out of Scope (explicit)
- **A wider W_hh / "32r" recurrent layer** — down-weighted by `rnn_capacity` (eff-rank ~10/16, not
  saturated); the lever is structure, not width. Revisit only if eff-rank climbs toward 16.
- **037 xiao M1-flight track** — re-homed to **039** (local IMU, full loop unroll, 20 Hz on-target,
  defined latency). Not 038.
- **M2 environment parity** (chase re-simulated in the source's own air mass) — dropped (operator: not
  needed at this accuracy); remains a BACKLOG item. (Wind parity was a cheap falsification in 037; the
  architecture thesis supersedes it as the depth lever.)
- **Eval-playback / determinism-witness harness** (`EvaluateMode=playback`) — deferred.
- **031 hardware/firmware/optics and the camera-noise *calibration*** — 038 consumes the incoming CEP
  rules + calibration; it does not produce them.
- **US5 — camera variations + prediction-through-blindness / reacquire** — **deferred to a follow-on**
  (Clarifications 2026-06-27): depends on US3's predictor winning its ablation AND on 031's incoming CEP
  rules. The `cameraSeed` insertion point + FR-011/FR-012 are recorded for that follow-on.
- **Full MoE expert split** — waits for *mode-interference* evidence (one mode caps while another climbs);
  the predictor/control split (US3) is the first MoE-lite seam, instrumented via `mode_progress`.
- **Final form of the predictor objective / history layout / recurrence structure** — research outputs of
  the study phase, not pre-committed here.

## Dependencies & Sequencing

1. **Phase 0** (P0-A PRNG, P0-B/P0-D renderer + self-describing dmp + simTime + wind, P0-C reporting) —
   front-matter; P0-D's one dmp break is the moment to land the format-breaking architecture inputs.
2. **RNN architecture studies (generic)** — **initial wave US1 history + US3 predictor head** run as
   **parallel independent ablations** off the same baseline (then combine winners). **US2 two-timescale
   recurrence is deferred to `specs/BACKLOG.md`** as a follow-on lever (overlaps US1; unpark if US1+US3
   don't move a ceiling). **Mixed M1-first**: US1 gated on an M1 ablation; US3 may go straight to M2.
3. **M2 prototyping (FOV-specific) — US4 only** — visibility-maintenance reward; may proceed independently
   of the ablations (does not gate on the predictor). **US5 deferred to a follow-on** (needs US3's
   predictor + 031's incoming CEP rules).
4. **Carried, not re-derived**: the t14 crash penalty (regression-guarded); reward-shaping optional.
5. **Parallel, not 038**: 039 xiao M1-flight; 031 camera design (source of the incoming CEP rules) +
   the deferred US5 follow-on once both predictor and CEP rules land.
6. Upstream baseline: 037 t10 source + t11/t14 M2 retrain. Downstream: a deeper, flyable M2.
