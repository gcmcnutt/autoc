# Phase 0 Research: Faster Control Loop (037)

**Date**: 2026-06-09 | **Status**: research-to-run (deliverables defined; several decisions deferred to
bench/datasheet/sim execution). Each item below is either **DECIDED** (settled in spec/grounding) or
**OPEN** (a research task with a defined output artifact). Phase A may not start until R1–R3, R5, R6 are
DECIDED; R4 runs in parallel; R7 (031 co-resolve) runs alongside R1–R3.

Format per item: **Decision / Rationale / Alternatives considered / Output**.

---

## R0. Grounding facts (DECIDED — from the 2026-06-09 code scan)

- **Decision**: The sim control cadence is governed by two constants that must move together —
  `gEvalUpdateIntervalMsec` (crrcsim `inputdev_autoc.h:53`, default 100 ms, env-overridable
  `AUTOC_EVAL_INTERVAL_MSEC`) and `SIM_TIME_STEP_MSEC` (autoc `aircraft_state.h:39`, 100 ms). FDM dt is
  `autoc_config.xml:157` (`dt=0.005`=200 Hz); outer fps `:3` (20 Hz). A startup assertion already enforces
  `cycleLengthMs | evalIntervalMsec`.
- **Rationale**: Establishes the exact knobs Phase A turns and the invariant they must satisfy.
- **Alternatives**: n/a (factual).
- **Output**: the touchpoint table in [plan.md](./plan.md#resolved-cadence-touchpoints-from-phase-0-grounding-scan-2026-06-09).

## RT. Smoothing theory — smooth vs relocate bang-bang (OPEN — gates the whole bake)

- **Decision (to produce)**: a defended **per-axis prediction** of whether a faster cadence smooths the
  command or just relocates bang-bang to the new Nyquist — *before* any retrain. See spec "Smoothing
  theory" section.
- **Rationale**: Faster ≠ smoother. A relay/sliding-mode-optimal controller will keep saturating-and-
  flipping at the new rate (the airframe low-passes a faster dither *better*). We need a first-principles
  reason to expect (A) aliasing-dither (smooths) over (B) relay-optimal (won't), so the bake is a
  confirmation, not a fishing trip.
- **Discriminators (from data we already have)**: (1) airframe roll time constant τ_roll from the FDM
  step-response trace + `Cl_p=-0.47`, compared to candidate tick T; (2) **raw pre-tanh roll drive** in t6
  traces — near the saturation knee ⇒ (A), deep ⇒ (B); (3) servo slew (low-passes command→motion) →
  define whether we optimize **command** or **motion** smoothness.
- **Alternatives considered**: empirical "try 20 Hz and look" (rejected — wastes bakes, and a null result
  wouldn't distinguish A from B); analytic-only without sim confirm (rejected — pair theory with the
  retrain that tests it).
- **Output (produced 2026-06-09, T008)** — **GATE VERDICT: GO. Roll = (A) aliasing/dither → a faster
  cadence is predicted to smooth the roll command.** Per-axis prediction, the numbers behind it, and the
  de-alias gate it implies are below.

  ### Leg 1 — airframe roll time constant τ_roll (analytic, from FDM + model)

  The larcsim roll moment (`fdm_larcsim.cpp:750,767`) is `L = Cl·QS·B_ref` with the rate-damping term
  `Cl_p·Phat` and `Phat = p·B_ref/(2V)` (`:608`). Linearizing the roll EOM `Ixx·ṗ = L`:

  `ṗ = [Cl_p·QS·B²/(2V·Ixx)]·p + (aileron term)`, so the **roll-subsidence time constant** is

  `τ_roll = 2·V·Ixx / (|Cl_p|·QS·B²) = 4·Ixx / (|Cl_p|·ρ·V·S·B²)`.

  With `hb1_streamer.xml` params (`Ixx=0.024 kg·m²`, `Cl_p=−0.47`, `S=0.136 m²`, `B=0.762 m`,
  `V=11 m/s` trim, `ρ=1.225 kg/m³`): numerator `4·0.024 = 0.096`; denominator
  `0.47·1.225·11.0·0.136·0.762² = 0.500`; **τ_roll ≈ 0.192 s ≈ 192 ms** (subsidence pole −5.2/s).
  Cross-check: the same linearization gives steady full-aileron roll rate
  `p_ss = 2·Cl_da·V/(|Cl_p|·B) = 774 deg/s` (Cl_da=0.22) — consistent with the model's own embedded note
  (`hb1_streamer.xml:96`, no-factor-of-2 form ≈ 387 deg/s) up to the Phat b/2 convention; the τ is
  internally consistent (`p_ss = ṗ₀·τ`). τ_roll is a physical time, so the slug/ft unit conversion is
  irrelevant to it. (Analytic; **not yet** cross-checked against the `MAX_TRACE_STEPS` step-response
  capture — see "unvalidated" note.)

  | cadence | T | T/τ_roll | airframe response in one held tick |
  |---|---|---|---|
  | 10 Hz | 100 ms | **0.52** | `1−e^(−0.52)` = **40%** of steady — appreciable, *not* T≪τ |
  | 20 Hz | 50 ms | **0.26** | 23% of steady — held intermediate command tracks finer |
  | 50 Hz | 20 ms | 0.10 | 10% of steady |

  **Reading.** This is the *nuanced* case the spec flagged: at 10 Hz `T` is the **same order** as τ_roll
  (0.52), so the airframe is neither a hard low-pass (which would force (B)) nor instantaneous. The
  control-loop Nyquist is **5 Hz**, and the observed roll oscillation (~0.56 flips/tick ≈ 2.8 Hz
  fundamental, energy out to ~5 Hz) sits **right at that Nyquist** — the textbook under-sampled signature.
  Raising to 20 Hz lifts Nyquist to 10 Hz (resolves the oscillation) **and** drops per-tick airframe
  response to 23%, so a *held intermediate* roll command becomes both commandable and physically smoother.

  ### Leg 2 — saturation depth / pre-tanh drive in t6 (validated against recorded t6 metrics)

  `out_rl` in the dmp CSV is the **post-tanh** NN output (`aircraft_state.h:478` "Raw tanh outputs";
  `topology.h:26` "roll [−1,1] via tanh"), applied directly as the command
  (`evaluator.cc:414` `setRollCommand(outputs[1])`, clamp ±1). So **pre-tanh drive = atanh(out_rl)** and
  saturation depth is directly readable: `|out|>0.9 ⇒ |pre-tanh|>1.47`; `0.95→1.83`; `0.99→2.65`.

  Validated t6 per-axis numbers (gen 494, energy-lexicase M1 = run
  `autoc-9223370256079660488-2026-06-06T19:45:15.319Z`; from
  `specs/035-energy-lexicase-objective/tuning-backlog.md:97`):

  | axis | dctrl | % sat `>0.9` | sign-flips/tick | lag-1 autocorr | pre-tanh implied |
  |---|---|---|---|---|---|
  | roll | 1.07 | **61%** | **0.56** | **−0.24** (anti-persistent) | `>1.47` on a majority of ticks |
  | pitch | 0.37 | **0%** | 0.26 | +0.44 (smooth) | well inside linear band |
  | throttle | 0.18 | 97% | 0.08 | +0.80 (pinned-high) | one-sided rail, not flipping |

  The saturation depth alone (61% deep, flipping 56%) *looks* relay-like (an argument **for (B)**) and is
  the honest tension in this gate. The discriminator that breaks the tie is **command-vs-motion**: a true
  min-time/sliding-mode relay (B) drives the airframe to its **rate limit** — but the recorded physical
  roll rate is **~155 deg/s, only ~20% of the τ-implied steady 774 deg/s**, and is *airframe-limited to
  ~150–200 deg/s across every 032→035 objective regime*. The saturated ±1 command is therefore **not
  delivering max physical authority** — it is being **averaged by the airframe** into an intermediate
  rate. That is the definition of (A) sub-resolution dither (PWM-like synthesis of an intermediate mean),
  not (B) bang-bang-optimal. The **−0.24 anti-persistent** lag-1 autocorr is the direct dither signature
  (deliberate alternation to synthesize a sub-tick average); a (B) relay tracking a slow switching surface
  would show *persistent* (≥0) autocorr, not anti-persistent.

  ### Leg 3 — command vs motion: what the gate measures

  The actuation chain (2 kHz FC / 400 Hz motor / ~50 Hz servo; measured cmd→gyro ≈ 12 ms, spec
  "Empirical grounding") is 5–200× faster than the 10 Hz NN loop, so the **servo barely slew-limits at
  10 Hz** and the airframe (τ_roll≈192 ms) is the dominant low-pass. Motion smoothness is therefore
  *already* airframe-bounded and is a **weak, near-saturated discriminator** — it would move little even
  under (B). **Decision: the Phase-A de-alias gate optimizes COMMAND smoothness** (servo wear, manned-feel,
  and the directly-observable de-alias signature), measured on the recorded `out_rl` series. This matches
  the gate the plan already states (lag-1 autocorr, sign-flip rate on the command), and it is the metric
  that actually distinguishes (A) from (B): under (A) the command de-aliases (autocorr→≥0, flips↓); under
  (B) it stays ugly (autocorr stays <0 / flips persist) even though motion is unchanged either way.

  ### Per-axis verdict

  - **ROLL → (A) aliasing/dither. GO.** Confidence **moderate-high**. τ_roll≈192 ms, T/τ=0.52 at 10 Hz
    (oscillation at the 5 Hz Nyquist), command saturated/flipping but airframe at only ~20% of steady rate
    (saturation is synthetic, not authority-limited), autocorr −0.24 (anti-persistent = dither). 20 Hz
    lifts Nyquist to 10 Hz and halves per-tick airframe response — both necessary conditions for the
    intermediate command to become commandable and smoother. **Prediction:** a 20 Hz retrain lands roll in
    the linear tanh band → saturation ↓, sign-flips ↓, autocorr → ≥0.
  - **PITCH → already smooth (A-side, not the limiter).** 0% saturated, +0.44 autocorr, dctrl 0.37. Pitch
    is already in the linear band; faster cadence will not hurt it and gives slightly finer trajectory.
    Not a gate axis.
  - **THROTTLE → one-sided rail, neither (A) nor (B) bang-bang.** 97% sat but only 0.08 flips/tick and
    +0.80 autocorr — pinned **high**, a coherent DC operating point (climb-energy), not a dither. Faster
    cadence leaves it essentially unchanged. Not a gate axis.

  ### Implied de-alias gate (what Phase A must show)

  On the **command** (`out_rl`) series of the 20 Hz retrain vs historical t6 10 Hz, fixed-eval
  (per `project_late_run_fitness_interpretation`): roll **lag-1 autocorr negative (−0.24) → ≥ 0** AND roll
  **sign-flip rate −≥20 pts (56% → ≤36%)**, with tracking (crash %, on-track %) within noise of t6.
  Matches plan.md Performance Goals exactly. Pitch/throttle are pass-through (must not regress).

  ### No-go clause

  If the 20 Hz retrain **fails** this gate (autocorr stays <0 / flips persist at a higher frequency),
  that empirically falsifies (A) in favor of (B) — relay/sliding-mode-optimal — and is a **cheap no-go**:
  the lever would then be objective/airframe (X-wing, prediction-aiding arch — out of 037 scope), and we
  stop before the expensive bundle bake. The *theory* verdict here is GO; the gate is the confirmation.

  ### Real-flight cross-check (added 2026-06-09, T008 — "a little bit of real back to sim")

  The gate above was analytic + t6-**sim**-validated only. The load-bearing assumption — "the saturated
  ±1 roll command is *not* delivering max physical authority; the airframe averages it to ~155 deg/s,
  only ~20% of the τ-implied 774 deg/s steady, *airframe-limited to ~150–200 across every regime*" — was
  inherited from `035 tuning-backlog.md:106` and `spec.md:23`. **That "~155 deg/s, 150–200 cap" number is
  a SIM figure**, not a real measurement: it traces to `docs/sim-to-real-analysis.md` (2026-04-15), where
  155 deg/s is explicitly the **sim** roll-rate std and the **real** (manual-pilot) std was 86 deg/s. The
  RT verdict treated a sim median as "the recorded physical roll rate." So we cross-checked it against
  the **actual** roll *rate* (`gyro_p` = INAV `gyroADC[0]` = deg/s, no scaling, per
  `join_flight_analysis.py:108`) on the most recent confirmed flight: **2026-05-17, pastonly3 converged
  gen 800**, both engage spans (the same flight `reference_sim_to_real_analysis` cites for the roll
  56%≈57% sign-flip match). Measured on the engage-span join CSVs
  (`flight-results/flight-20260517/join_analysis_*_span{1,2}_*.csv`,
  script `specs/037-20hz-control-loop/rollrate_crosscheck.py`):

  | quantity (deg/s) | span1 path0 (18.4 s) | span2 path2 (23.8 s) |
  |---|---|---|
  | \|roll rate\| median (p50) | **134** | **155** |
  | std | 231 | 237 |
  | p90 / p95 / p99 | 434 / 464 / 514 | 398 / 463 / 532 |
  | max | 540 | 620 |
  | frac \|rate\|>200 | **36%** | **40%** |
  | frac \|rate\|>300 | 22% | 26% |
  | \|bank angle\| p50 / max | 36 / 180° | 61 / 180° |

  **CONFIRMS (weakly):** the **median** real roll rate is 134–155 deg/s — i.e. the "~155" figure is right
  *as a typical/central value*, and the deg/s unit is sanity-checked (p50 bank 36–61° with 180° peaks is a
  physically consistent spiral). The real command character also independently corroborates the gate's
  *command-side* premise: the same flight's roll sign-flip rate is **57%** (span1) / **49%** (span2)
  (`FLIGHT_REPORT.md:145`), matching the t6 sim **56%** that RT used. So the *command is bang-bang in real
  flight too* — the phenomenon the gate is built on is real, not a sim artifact.

  **CONTRADICTS:** the **"airframe-limited to ~150–200 deg/s across every regime"** claim, which is the
  actual load-bearing half. Real roll rate is **not** capped near 150–200: **36–40% of engage-span samples
  exceed 200 deg/s, ~22–26% exceed 300, p95 ≈ 463–464 (≈60% of the τ-implied 774), p99 ≈ 514–532, peaks
  540–620 deg/s**. The airframe demonstrably reaches a *large fraction of full roll authority* on the
  reversals — the saturated ±1 command **does** deliver substantial physical rate, it is not purely
  sub-resolution dither low-passed to a benign mean. The "only ~20% of steady" framing holds only for the
  median; the distribution has a heavy authority-reaching tail the gate did not account for. This **does
  not flip the verdict to (B)** — the −0.24 anti-persistent command autocorr (dither signature) and the
  τ_roll T/τ=0.52-at-10 Hz Nyquist argument stand on their own — but it **weakens the specific "motion is
  already saturated/benign, so only the command can change" claim** that justified gating on *command*
  smoothness alone (Leg 3). If the airframe is being driven to 60% authority on flips, a faster cadence
  may move *motion* (not just command), which is a stronger and more flyable win than the gate currently
  credits — but it also means the (A)-vs-(B) tie is closer than "20% of steady" suggested.

  **CANNOT resolve from this flight:** (1) whether 20 Hz actually de-aliases the command — that is exactly
  what the Phase-A retrain gate tests; real flight is 10 Hz only, so it cannot pre-confirm the lever.
  (2) The clean (A)-vs-(B) discrimination at the *motion* level — real flight has no held-command
  step-response segment, so it cannot substitute for the FDM trace. (3) These are 2 spans on the two
  *calmest* paths (0, 2) under stronger 350° wind; the heavy rate tail may be partly gust-driven, which
  would *over*-state the authority-reaching contradiction (a calm-air NN-only span could sit lower).

  **Net effect on GO:** still **GO**, but the confidence basis shifts. The verdict no longer rests on
  "saturation is purely synthetic / airframe is capped" (real flight contradicts the cap). It now rests on
  the two surviving legs: the **Nyquist/τ_roll under-sampling argument** (Leg 1, unchanged) and the
  **anti-persistent command autocorr** (Leg 2 dither signature, corroborated by the 57%/49% real flip
  rate). The real-flight tail is, if anything, *additional* upside (motion may improve too). **The FDM
  held-command step-response trace remains the outstanding analytic validation** — it is now *more*
  important, because it is the only thing that can settle whether the 540–620 deg/s peaks are
  rate-limit-pinned (→ (B) risk) or just the linear airframe responding to a deep but finite-duration
  command (→ (A) survives at the motion level too). Do not close T008's "unvalidated" note until that
  trace is captured.

  ### Craft-variation adequacy at 20 Hz (added 2026-06-09, T008)

  **Question:** does the faster cadence expose dynamics the current craft-variation set
  (`034 US4`, `config.h:106–112`) does **not** cover, such that a sim-to-real-faithful 20 Hz retrain
  needs *additional* simulated variations?

  **The current set is entirely STATIC, quasi-steady aero.** `enableCraftVariations` draws six
  per-scenario airframe parameters — `craftCGSigma` (±7% MAC), `craftDragSigma` (±5% CD), `craftTrimSigma`
  (±1.15° Cm_0), `craftThrustSigma` (±5%), `craftPitchEffSigma`/`craftRollEffSigma` (±5% Cm_de / Cl_da).
  All six are **constant per scenario** and shift *steady-state* coefficients — control *effectiveness*,
  *drag*, *trim*, *CG*. Per `project_craft_variations_not_diversity_fix` they were never meant to address
  control character; they perturb the airframe operating point, not its temporal/loop dynamics. **None of
  them models a time constant, a bandwidth, or a delay.** At 10 Hz this is fine: RT Leg 3 establishes the
  servo (~50 Hz) and compute delay are 5–200× faster than the NN tick, so actuator dynamics are
  negligible against the τ_roll≈192 ms airframe pole and need not be drawn.

  **At 20 Hz that separation-of-timescales argument shrinks by half** (T=50 ms vs T=100 ms), and the
  de-alias gate explicitly cares about behavior *near the new Nyquist* (10 Hz) — exactly where the
  previously-ignorable actuator dynamics live. Two dynamics the current set does NOT cover become
  first-order at the faster cadence:

  1. **Servo slew-rate + actuator bandwidth.** There is **no servo lag or slew limit anywhere in the FDM**
     (`COMPUTE_LATENCY` is a pure sensor→output *dead-time*, `inputdev_autoc.h:56`; grep finds no
     `tau_servo` / rate-limit). A real ~50 Hz servo has a finite slew rate; at 50 ms ticks a full ±1
     reversal every other tick (the 57% real flip rate) is **the regime where servo slew actually clips
     the commanded step** — the one place the (A) de-alias could be *masked* (servo can't follow the fine
     dither) or, conversely, where modeling it would reveal motion *does* smooth. A sim with an
     infinitely-fast servo will over-state how cleanly 20 Hz de-aliases. This is the single most important
     missing axis, and it is *coupled to* the gate's verdict, not just to fidelity.
  2. **Control/compute LATENCY jitter.** T024 already plans to retarget `COMPUTE_LATENCY` to the projected
     loop's measured value **and add cadence-jitter** so training is dt-tolerant (per 020). That covers the
     *timing* jitter axis. It does **not** cover servo/actuator *bandwidth* as a physical first-order lag —
     T024 re-derives the INAV PID filter cutoffs (`rc_filter_lpf`/`gyro_lpf`/`dterm_lpf`) but those are the
     *flight-controller* filters, not an airframe servo transfer function.

  **Recommendation: add ONE new variation axis — a servo first-order lag + slew-rate, with a per-scenario
  sigma (`craftServoTauSigma` / `craftServoSlewSigma`)** — and fold it into the T024 bundle rather than the
  static `[Craft]` block, because it is a *dynamics* parameter that only bites at the new cadence, not a
  steady-airframe draw. Center it on the bench-measured cmd→gyro lag (≈12 ms, RT "Empirical grounding")
  with a sigma wide enough to span plausible servo grades. Rationale: it is the only candidate that (a) is
  silent at 10 Hz but active at 20 Hz, (b) sits exactly at the new Nyquist the gate measures, and (c) can
  *change the gate outcome* (a slew-limited servo is the mechanism by which command-smoothness and
  motion-smoothness could diverge — the very distinction the real-flight cross-check above showed the gate
  underweights). Without it, a "passing" 20 Hz de-alias gate risks being an artifact of an idealized
  infinite-bandwidth actuator.

  The other static craft sigmas need **no change for 20 Hz** — CG/drag/trim/thrust/eff are cadence-
  independent and already adequate for what they cover. This is an *addition* (one dynamics axis), not a
  rework of the existing set.

  **IMPLEMENTED 2026-06-09 (in-FDM; scope grew to 3 axes).** Built as actuator dynamics INSIDE
  `fdm_larcsim` at the **substep dt**, NOT the command path -- a once-per-outer-frame command-path filter
  would barely resolve a ~20 ms servo (tau <= the 20-100 ms frame interval) and would not benefit from the
  raised FDM rate at 20/50 Hz; the FDM is also the layer the other craft variations already modify
  (`Global::craft*`). Per the operator's 4-craft-fleet evaluation the single recommended axis became
  THREE craft variations: `CraftServoTauSigma` (center 20 ms), `CraftServoSlewSigma` (center 6 full-throw
  per s), `CraftThrustTauSigma` (center 150 ms -- thrust lag was also absent; prop shaft `J=0`). Each is
  Gaussian-drawn then clamped to a positive physical range, wired exactly like `craftThrustSigma`, with
  per-scenario filter-state reset every `initAirplaneState` for determinism, and drawn values shown in the
  startup variation table. Also landed: the energy curve `thr^2 -> thr^2.5` (closer to real prop power).
  Awaits the operator's clean build + bitwise regression gate. See tasks.md "Session implementation status
  (2026-06-09)".

  ### Real-flight roll / throttle / servo characterization (added 2026-06-09, T024 — full-corpus models)

  The two prior subsections recommended a servo-lag axis but **guessed** its center/sigma (≈12 ms bench +
  "wide enough"). This refines that from the *whole* flight corpus, INCLUDING manual (pilot) segments, to
  replace the guess with measured airframe models for the FDM. Sources: all 14 raw INAV blackbox CSVs in
  `flight-results/flight-2026*/` (scripts `specs/037-20hz-control-loop/flight_corpus_characterize.py`,
  `servo_throttle_dynamics.py`, `throttle_step_response.py`). The 2026-05-17 autoc-span numbers reproduce
  `rollrate_crosscheck.py` exactly (p95=464, max=620, frac>200=39%), validating the pipeline.

  **Corpus inventory (14 logs, 10 flight days).** Every log interleaves MANUAL (pilot) and AUTOC
  (`mspOverrideFlags` bit1 = MSP override) segments — typically 4–7 manual segments of 15–35 s each per
  flight, plus autoc engage spans of 4–32 s. One log is **100 % manual** (2026-04-03 `.02`, 225 s) — the
  best throttle/roll-authority reference. **Key correction to docs:** the blackbox main frame is
  **~60 Hz (16.7 ms)**, NOT the 200 Hz `docs/INAV_BLACKBOX.md:155` claims — so sub-sample lag resolution is
  ~17 ms and fast servo dynamics (<17 ms) are below the log's Nyquist; everything below is lower-bounded by
  that grid. Total usable: ~30 min of flight, roughly half manual.

  **1. Roll authority — the airframe is NOT rate-capped; the 150–200 "cap" is pilot behavior.**
  Manual vs autoc `|gyro_p|` (deg/s), per-flight, pooled over >=2 s segments:

  | regime | typical p95 | typical p99 | max seen | frac>200 |
  |---|---|---|---|---|
  | MANUAL (pilot) | 86–157 | 144–534 | **486–646** (4 flights >480) | 0–4 % |
  | AUTOC (NN) | 289–464 | 358–525 | 413–659 | 13–39 % |

  The *pilot* rarely commands hard rolls (p95 86–157, sits where the "150–200" figure came from) — but the
  **manual peaks reach 486 / 594 / 646 deg/s** (2026-03-20, 2026-04-03 `.01`/`.02`), and the manual
  cmd->rate gain is **~410–520 deg/s per full deflection**. So the airframe demonstrably *delivers* near-
  full authority when asked; the median-low manual rates are a *pilot habit*, not an airframe limit. This
  **confirms and generalizes the cross-check's CONTRADICTS finding across the whole corpus**: the
  load-bearing "airframe-limited to ~150–200 across every regime" premise is false on manual data too. The
  τ-implied 774 deg/s steady is approached (peaks 60–85 % of it) on both pilot snap-rolls and NN flips.

  **2. Servo -> craft-rate lag — ~90 ms lumped, and it is PHYSICAL, not transport.**
  Cross-correlating `rcCommand[roll]` against `gyro_p` over **manual** segments (clean pilot doublets, not
  the autoc bang-bang whose flat spectrum smears xcorr), the corr-vs-lag curve is unimodal and consistent
  across all 13 flights: corr ≈ 0.65 already at 0 ms (rate starts responding within one sample), **peaks at
  83–101 ms (corr 0.77–0.91)**, then decays. The **command->roll-rate lag-to-peak = ~90 ms (median 101 ms,
  range 83–117)**. Decomposition:
  - `rcCommand[roll] -> servo[0]` lag is **0–17 ms** (one sample) with corr 0.6–0.8 — the FC mixer/servo
    *command* is effectively instantaneous in the log. So the 90 ms is **not** compute/transport dead-time;
    it is the **physical** servo actuation + airframe roll-rate build-up, lumped.
  - This ~90 ms cmd->rate peak is within ~2× of the FDM's own modeled **τ_roll ≈ 192 ms** subsidence pole
    (Leg 1) — the cross-correlation peak of a doublet-vs-rate sits earlier than the full settling time, so
    a 90 ms peak is *consistent with* (not contradicting) a ~150–190 ms first-order roll pole. **The FDM
    airframe roll dynamics are in the right ballpark; what it lacks is the servo lag IN FRONT of that pole.**
  - Servo-role note: `servo[0]` and `servo[1]` BOTH correlate **−0.59 / −0.58** with `gyro_p` (and their
    difference ~0.00) — they are the two elevon channels deflecting *together* for roll on this flying wing,
    not a differential pair. (Corrects the `servo[0]-servo[1]` roll proxy used in the first script pass; the
    `rcCommand`-based lag above is unaffected.)

  **3. Throttle -> power — slow (~0.5 s vehicle response), continuous, and the FDM is instantaneous.**
  - Motor command on the 100 %-manual flight is **smooth**: `|d(motor)|>100 us/sample` fraction = **0.000**,
    max single-sample step 159 us, P5..P95 = 1098..2000 us. The autoc-era "binary throttle" is a
    *controller* artifact (NN saturates), not an airframe/pilot property — pilots modulate throttle
    continuously over the full 1000–2000 us range.
  - One clean isolated step-up (2026-04-03 `.02`, t=212 s, +500 us hold): ground-speed `|navVel|` rose
    **12.4 -> 16.5 m/s with a 63 % rise time of ~477 ms**. Pooled motor-PWM -> `|navVel|` cross-correlation
    across flights gives an effective lag of **~0.5–2 s** (corr only 0.2–0.4 — heavily wind/attitude
    contaminated, no airspeed sensor; GPS ground-speed only). This ~0.5 s is a **vehicle speed** time
    constant (thrust build-up + mass/drag accel), so the *pure thrust* time constant is **shorter** —
    bounded below by the 17 ms log grid and not separately resolvable from this corpus.
  - **FDM gap:** `hb1_streamer.xml` power block has shaft inertia **`J="0"`** and `propeller J="0"`
    (`crrcsim/models/hb1_streamer.xml:150-151`), so the CRRCSim `automagic` battery/shaft/prop model
    delivers thrust **instantaneously** with throttle — no spin-up lag at all. Steady-state thrust is already
    tuned (`F="7.5"`, cruise ~60 % throttle matches real per the 019/023 comment trail); only the *transient*
    is missing.

  **Recommended FDM model additions + parameters (replaces the guessed `craftServoSlewSigma`):**

  - **(a) Throttle -> power: first-order thrust lag.** Add a single thrust-command low-pass
    `T_cmd_filt += (dt/tau_thr)*(T_cmd - T_cmd_filt)` (or set the CRRCSim shaft inertia `J` non-zero so the
    existing power-ODE provides the lag). **Center `tau_thr ≈ 150 ms`** (a conservative under-estimate of the
    ~0.5 s vehicle response, attributing most of the 477 ms to airframe mass not thrust), **per-scenario
    sigma `tau_thr_sigma ≈ 100 ms`** (bracket 50–300 ms — covers a snappy ESC up to a sluggish prop/battery).
    At 10 Hz this is near-invisible (T/tau = 0.67) but at 20 Hz (T=50 ms) it materially shapes how fast the
    NN can pump energy, which matters for the throttle-smoothness axis.
  - **(b) Servo -> craft-rate: first-order servo lag + slew limit, IN FRONT of the existing roll pole.**
    A real ~50 Hz analog servo on a foam elevon has both a bandwidth (lag) and a slew-rate ceiling. Model
    `u_act += clamp((u_cmd - u_act)/tau_srv * dt, -slew*dt, +slew*dt)` on each control channel before the
    aero/mixer. Measured anchors: cmd->servo ~1 sample (≤17 ms) and lumped cmd->rate ~90 ms of which ~150 ms
    is the modeled airframe pole. That leaves the **servo lag itself small but non-zero**:
    - **`tau_srv ≈ 20 ms` center** (one log-sample's worth; consistent with the ≈12 ms bench cmd->gyro and a
      ~50 Hz servo's ~3 ms mechanical + filtering), **per-scenario sigma `tau_srv_sigma ≈ 15 ms`** (bracket
      5–50 ms: from a fast digital servo to a tired analog one). This is the **single most important new
      axis** — silent at 10 Hz (T/tau = 5), first-order at 20 Hz (T/tau = 2.5), and it sits exactly at the
      new Nyquist where the de-alias gate measures. It is the mechanism by which 20 Hz command-smoothness and
      motion-smoothness can diverge.
    - **`slew_srv ≈ 6.0 (full-throw/s)` center** (a ~50 Hz hobby servo does ~0.1 s/60° ≈ a full ±1 sweep in
      ~150–300 ms under load → 3–7 /s), **sigma `slew_srv_sigma ≈ 2.0`** (bracket 3–9 /s). At 20 Hz a full
      ±1 reversal every other tick (the 57 % real flip rate) demands ~40 /s instantaneous — far above any
      real servo — so the slew limit **will** clip the finest dither, which is precisely the test of whether
      20 Hz de-aliases *motion* or only *command*. This is the parameter that can change the gate verdict.

  **Bundle placement (ties to T024 + craft set):** fold `tau_srv` / `slew_srv` / `tau_thr` (+ their sigmas)
  into the **T024 latency/dynamics bundle** alongside the `COMPUTE_LATENCY` retarget and cadence-jitter —
  NOT into the static `[Craft]` block (`config.h:106–112`), because they are *temporal* parameters that only
  bite at the faster cadence. Add three ini keys mirroring the craft-sigma convention
  (`CraftServoTauSigma` / `CraftServoSlewSigma` / `CraftThrustTauSigma`) with the centers above as defaults
  and `enable` gated like `enableCraftVariations`. The existing six static craft sigmas (CG/drag/trim/
  thrust/pitchEff/rollEff) stay unchanged — they are cadence-independent.

  **What could NOT be measured (missing data):** (1) **Pure servo bandwidth** isolated from airframe — the
  60 Hz log floor (17 ms) is coarser than a 50 Hz servo's own time constant, so `tau_srv` is an *upper*
  estimate; a bench servo-step test (scope on a function generator) would pin it. (2) **Pure thrust time
  constant** — no airspeed sensor and no RPM/`escRPM` populated (column present but unlogged), so only the
  ~0.5 s *vehicle* response is observable, not the prop spin-up; `escRPM` logging or a thrust-stand would
  resolve `tau_thr`. (3) **Slew-rate ceiling directly** — the pilot never commands fast enough to hit it and
  the NN's bang-bang is logged at 60 Hz only; `slew_srv` is inferred from servo-spec datasheets, not
  measured in flight. (4) **20 Hz behavior itself** — the corpus is 10 Hz outer-loop only, so it cannot
  pre-confirm whether the modeled servo lag actually masks the de-alias; that is exactly the Phase-A retrain
  gate (T026/T027). These three new axes make that gate *honest* (an infinite-bandwidth actuator would
  over-state how cleanly 20 Hz de-aliases) but do not substitute for it.

## R1. Projected cadence from candidate hardware (DECIDED 2026-06-10 — 20 Hz for the Phase-A sim arm)

> **DECISION (operator 2026-06-10): get going with 20 Hz now.** The Phase-A sim retrain runs at
> **20 Hz (50 ms)**. Rationale: (a) RT verdict is GO with roll = case A (aliasing-dither), and the
> Nyquist/τ_roll argument already justifies 2× over 10 Hz (τ_roll ≈ 192 ms vs the 100 ms tick);
> (b) 20 Hz is the rate the existing hardware loop already *sends* at (`MSP_LOOP_INTERVAL_MSEC=50`,
> `MSP_NN_EVAL_DIVISOR=2`), so the embedded path (US2) needs no new transport to reach it;
> (c) the cadence triple holds with the **unchanged** sim config — `video.fps=20`, `Global::dt=0.005`
> ⇒ cycleLength 50 ms, framesPerEval 1, and FDM oversample = (1/0.005)/(20 Hz) = 10× (meets the ≥10×
> contract floor exactly; the 2 kHz FDM bump is deferred to the US3 50 Hz arm which requires it).
> R2 (camera grid) and R3 (transport ceiling) stay OPEN — they gate the Phase-B *flown* rate and the
> 50 Hz ceiling, not the sim go/no-go, and 20 Hz is reachable on today's link as measured.

- **Decision (to produce)**: a **projected NN loop rate** (the cadence we'd actually fly) + a per-part
  clock table. Not pre-picked to 20/40/48 — **projected from what candidate/similar hardware can sustain**
  (camera grid R2, transport ceiling R3, servo/IMU responsiveness), avoiding equal/near-equal unsynced
  collisions, and guided by RT's prediction of where (A) starts to win. Output is a small **selectable-rate
  shortlist** + the chosen projected rate, not a single guessed number.
- **Bundle note**: the projected rate is not trained alone — it derives a config **bundle** (cadence +
  latency/jitter model + history-bucket sizing + FDM rate) per spec "Smoothing theory §projected-cadence
  flow". R1 feeds the cadence; R5 the buckets; the latency model from R3+020; FDM from the cadence-config
  contract.
- **Rationale**: Achievable rates are set by real part tolerances, not nominal harmonics; the harmonic
  family only *bounds* beats, and equal-rate unsynced clocks are the worst case (spec correction
  2026-06-09). The real robustness is dt-aware control + per-source self-syncing correlators.
- **Alternatives considered**: (a) pick 20 Hz outright — rejected: doesn't use the camera-grid/transport
  facts and risks an unsynced collision; (b) full 10/20/50 sweep first — rejected: prove the lever on one
  researched rate first (spec).
- **Output**: table `{part → clock source (RC/crystal) → tolerance ±% → programmable-rate grid →
  implication}` for camera, Xiao nRF52840 time-base (HFXTAL vs RC), Lattice FPGA candidates; INAV I/O
  noted non-binding for rate (binds the MSP budget only).

## R2. Camera frame-rate grid + AGC/exposure budget (OPEN — co-resolved with 031)

- **Decision (to produce)**: the camera's chosen reachable frame rate (+ register recipe) and a fixed
  exposure/gain operating point for beacon detection. Candidate parts: **ST VD55G1** (primary), **OV9281**
  (backup) — both 300+ fps at ~320×240.
- **Rationale**: Frame rate is **near-continuous** via `pixel_clock/(HTS×VTS)` + PLL — 320/410/450/460/
  465/470/480 are all reachable, not a coarse menu. Bounds: max pixel-clock/PLL (fps ceiling per window),
  min HTS/VTS, line-time granularity. **AGC coupling**: `t_exp ≤ frame_period − readout`; higher fps →
  less exposure → more gain → less SNR. AEC/AGC must be **fixed/disabled** (auto-hunting aliases the
  pulsed beacon); pick fixed ~1 ms exposure (matches 1 kHz chip) + fixed gain at max range.
- **Alternatives considered**: free-running 480 assumption (rejected — quantized + tolerance-bound);
  auto-exposure (rejected — aliases beacon).
- **Output**: fps-vs-window curve for the chosen part; register recipe (PLL/HTS/VTS) for the selected
  rate; exposure/gain/SNR at candidate rates with beacon link-margin check.

## R3. Serial transport ceiling — read AND write (OPEN — gates the rate ceiling)

- **Decision (to produce)**: the transport decision (baud bump vs SPI) and the resulting read/write
  budget at the target rate.
- **Rationale**: 115200 is a **soft config limit** (both `inav-hb1.cfg` serial ports and
  `xiao msplink.cpp:342 Serial1.begin(115200)`), not hardware. Read (12.6 ms) + write (9.2 ms) share the
  pipe; raising baud to 921600/1M (≈8×) cuts these to ~1.5/~1.2 ms — likely removing the link as the
  binding limiter before any local-IMU work. SPI (nRF52840 SPIM ~32 MHz / STM32F7 ~25–50 MHz) is the
  ceiling option. **We already send at ~20 Hz with duplicate contents** (`MSP_NN_EVAL_DIVISOR=2`), so the
  send rate is partly proven; the open part is fresh read+write together at a higher rate.
- **Alternatives considered**: keep 115200 (rejected — caps rate well below 50 Hz); jump straight to SPI
  (deferred — try baud bump first, cheapest).
- **Output**: minimum command-frame cost at 115k and at the bumped baud; max sustainable read+write rate;
  baud-vs-SPI recommendation with the post-change budget. Cross-check/update `project_sim_latency`.

## R4. NN-eval cycle-count harness (OPEN — parallel; informs Phase C scope)

- **Decision (to produce)**: measured cycles/µs per (shape × tanh-impl × gather) on the real nRF52840,
  and the defended eval budget.
- **Rationale**: Don't trust the ~0.1 ms theoretical floor; measured eval is 2.7 ms avg / 7.4 ms max
  today (library `nn_forward_recurrent`, looped, `nn_program_generated.cpp`; timed with `micros()` not
  DWT). The harness decides whether unroll+fast-tanh (Phase C) is even needed at the chosen rate.
- **Alternatives considered**: FLOP-count estimate only (rejected — misses cache/flash-wait/preemption);
  on-target only (kept, plus an off-target op-counter for fast iteration).
- **Output**: on-target `DWT->CYCCNT` numbers for (a) MACs, (b) +tanh (`expf` vs poly/LUT), (c) +gather,
  for M1 (1923 w) and M2 (2595 w); off-target op-counter cross-check; fp32 kept for parity.

## R5. History time-basis (DECIDED 2026-06-10 — ms-based log-spaced lags, slot count unchanged)

> **DECISION (2026-06-10, T012): history lags are defined in MILLISECONDS, log-spaced (octaves),
> with the slot count kept at 6:**
>
> `kNNHistoryLagsMsec[6] = {1600, 800, 400, 200, 100, 0}`  (slot order TM5…NOW, oldest first)
>
> Tick offsets derive at compile time: `lagMsec / SIM_TIME_STEP_MSEC`, static_assert-ed integral.
> At 50 ms ticks → `{32,16,8,4,2,0}`; at the legacy 100 ms tick the same set is `{16,8,4,2,1,0}` —
> the lag set is genuinely **time-based and rate-invariant** across the 10/20 Hz family (50 Hz:
> `{80,40,20,10,5,0}`).
>
> - **Window grows 0.5 s → 1.6 s** (the R5 complaint was the window being far too short vs
>   lost-sight runs); recent motion keeps 100 ms granularity (lags 0/100/200 ms) for
>   derivative quality, the far tail (400/800/1600 ms) carries trend.
> - **`NN_INPUT_COUNT` stays 33 (M1) and 54 (M2)** — no topology/weight-count change, no nn2cpp or
>   xiao layout churn; T021/T022 reduce to lag-index + ring-depth changes. Chosen over the
>   Fibonacci 7-slot example (window only 0.65 s, count churn for no window gain).
> - **Derivatives use the actual lag gap**: closing_rate and span_rate are computed over the
>   NOW↔TM1 gap = 100 ms at every rate (so closing_rate keeps its historical 10 Hz semantics
>   exactly; span_rate changes from a raw one-tick diff to a true per-second rate — M2 retrains
>   from scratch anyway).
> - **Fail-loud (Principle V)**: counts don't change, so the count check can't catch the semantic
>   change. A `kNNHistoryLayoutVersion = 2` marker is serialized in the per-state NN block and
>   checked on read (v1 = uniform tick lags {5..0}; old dmps throw a layout error — same
>   no-backward-compat pattern as 030 M9.preA).
> - Error-ring depth derives from the max lag: `HISTORY_SIZE = 1600/SIM_TIME_STEP_MSEC + 1`
>   (= 33 at 50 ms). Tracker mirrors keep the 6-slot `TrackerHistoryWindow` as the gather view,
>   materialized from a deeper per-stepper ring at the same lag offsets.

- **Decision (lean, to confirm in retrain)**: move history to **time-based / log-spaced lags**, choosing
  N on a *time* basis — not "just add slots." Current: M1 6 slots (`nn_inputs.h:36-58`), M2 `span[6]` +
  beacon `[6]` (`evaluator.h:112-127`), uniform 100 ms past-only (`HIST_PAST[]`), error ring
  `HISTORY_SIZE=10` hardcoded "1 s @ 10 Hz".
- **Rationale**: At a faster cadence, fixed-N-ticks shrinks the trend window (6 ticks: 600 ms→300 ms at
  20 Hz), and it's already 15× too short vs lost-sight (`project_032_phase1_setup`). Log/Fibonacci-spaced
  lags hold a much longer window with few slots and are ~rate-invariant in time.
- **Alternatives considered**: (a) more uniform slots — rejected: grows NN input linearly, still
  tick-bound; (c) full time-resampling — viable, heavier; log-spaced is the lean middle.
- **Output**: chosen lag set (e.g. t−1,−2,−3,−5,−8,−13…), the new `NN_INPUT_COUNT` for M1/M2, and the
  Principle-V layout note (see contracts/nn-input-layout.md). **This changes the dmp/NN layout.**

## R6. Cadence config representation (OPEN — small, do with Phase A)

- **Decision (to confirm)**: unify the control-rate cadence into a single source of truth read from
  `.ini`/CLI (not the `AUTOC_EVAL_INTERVAL_MSEC` env var), keeping `gEvalUpdateIntervalMsec` and
  `SIM_TIME_STEP_MSEC` coherent.
- **Rationale**: `feedback_cli_over_env_vars` (tooling configures via CLI/ini, not env); two constants for
  one cadence is a drift hazard. Per Principle VII, the new key carries no in-class default.
- **Alternatives considered**: keep env var (rejected — violates the convention, and the rate is now a
  first-class experiment knob); fully merge the two constants into one (preferred if low-risk; confirm the
  crrcsim/autoc boundary allows it).
- **Output**: the cadence config contract (contracts/cadence-config.md) + the chosen representation.

## R7. 031 co-resolve (OPEN — runs with R1–R3)

- **Decision (to produce)**: a jointly-feasible {control rate, camera part, decoder FPGA} point.
- **Rationale**: Two-way loop — 037→031 the rate sets 031's acquisition/frame/chip budget; 031→037 the
  candidate parts' clocks/fps/PLL bound the rate (`docs/aircraft_tracker_handoff.md` §4). Neither is
  upstream; co-resolve from the options at both ends.
- **Alternatives considered**: treat 031 as purely downstream (rejected — its part clocks bound R1/R2).
- **Output**: write the chosen rate back into `aircraft_tracker_handoff.md` as the acquisition-budget
  input; pull 031's part shortlist forward as R1/R2's parts-to-characterize; carry the per-beacon
  independent-correlator requirement.

---

## Tick-rescale audit (DECIDED scope — feeds Phase-A tasks)

The cadence change is **not just the interval constant**. Confirmed per-tick/cadence-coupled terms:

| Term | Site | Action |
|---|---|---|
| streak ramp | `fitness_decomposition.cc:33-34` | none — already `fitStreakRampSec/(SIM_TIME_STEP_MSEC/1000)` ✅ |
| stability accum | `fitness_decomposition.cc:179` | normalize per second (Σ → Σ·dt) or rescale — **was unnormalized** |
| energy accum | `fitness_decomposition.cc:180` | same — normalize/rescale |
| closing_rate | `evaluator.cc:333` `/0.1f` | replace with actual dt |
| closure_rate | `fitness_decomposition.cc:212` | uses `SIM_TIME_STEP_MSEC/1000` ✅ but verify magnitude semantics |
| thrash_rate | `fitness_decomposition.cc:233-244,301-305` | per-second already; raw count granularity changes — document |
| error ring | `aircraft_state.h:330` `HISTORY_SIZE=10` | resize on time basis (R5) |
| engage delay | `023 contract` (not in code) | implement `ceil(ms/step)` rate-independent |
| SIM_TOTAL_TIME_MSEC | `aircraft_state.h:40` | unchanged (time-based) — tick count doubles, fine |

**Comparison discipline**: per `project_late_run_fitness_interpretation`, compare rates with fixed-eval
and the variation-stable per-axis dctrl/sign-flip comparators, not raw training fitness.

### T013 sign-off — full-tree buried-time-constant sweep (2026-06-10)

Audit table above confirmed against code, with these ADDITIONS found by the sweep:

| Term | Site | Verdict |
|---|---|---|
| **main per-tick path score** | `fitness_decomposition.cc:170` `accumulatedScore += multipliedScore` | **MISSING from the original table — must rescale.** stepPoints is a per-tick sample of instantaneous geometry; at 20 Hz the total ~doubles while `SIM_CRASH_PENALTY=300` stays fixed, silently halving the crash penalty's relative weight and breaking t6 comparability (Q1/Q3 gate needs "tracking within noise of t6"). Fix: `× kCadenceTickScale` alongside stability/energy. |
| rescale anchor | (design) | Rescale anchor is **`kCadenceTickScale = SIM_TIME_STEP_MSEC/100`** (100 ms-tick-equivalent units), NOT `×dt` seconds: ×1.0 at 10 Hz is bitwise-exact (regression gate holds), totals stay on the historical scale, and the constant lexicase epsilon 0.5 (`project_lexicase_mad_epsilon`) keeps its meaning. |
| M2 source-tick spacing | `tracker_stepper.cc:238-242` (and crrcsim mirror) | **NEW FINDING**: stepper advances chase one `SIM_TIME_STEP_MSEC` per **source tick**; source dmps are 10 Hz-recorded, so at 20 Hz an old source library plays the target at 2× speed *silently*. Fix: fail-loud spacing check (`SourceTickSample.simTimeMsec` deltas vs `SIM_TIME_STEP_MSEC`). M2 (US1b) needs a fresh 20 Hz source bake — already implied by the gate chain. |
| rabbit speed-profile resolution | `variation_generator.h:221` `cycleDuration / 0.1` | **NO CHANGE** — waypoint resolution of a cosine-eased profile that is *time-interpolated* on read (`getSpeedAtTime`); cadence-independent geometry, and changing it would shift PRNG-independent profile shape. |
| `MAX_OFFSET_STEPS = 10` | `aircraft_state.h:104` | **Dead constant** (no usages anywhere). Derive from time anyway (±1 s) so it can't rot if revived. |
| `SIM_MAX_INTERVAL_MSEC` | `aircraft_state.h:42` | Already `SIM_TIME_STEP_MSEC × 5` — scales automatically. |
| thrust lag blend | `fdm_larcsim.cpp:605` `min(1, dt/thrustTau)` | Linear blend vs servo's exact `1−exp(−dt/τ)`: at substep dt=5 ms / τ=150 ms the difference is 1.6% and the FDM substep doesn't change at 20 Hz — **not a cadence hazard**. Real gap (already in finding.md): thrust lag lags `craftThrustScale` (a ≈1.0 multiplier), not the throttle→thrust path, so spool-up is effectively unmodeled. Separate rework, not 037-blocking. |
| engage delay | `inputdev_autoc.cpp:628` | Already `ceil(ms/interval)` rate-independent in crrcsim **pathgen** branch; tracker branch (both mirrors) has NO engage window — T020 adds it. |
| servo/IMU sensor filters | (sweep) | No LPF/alpha constants in the sim command path; PidInternals is captured-but-inert (no live ACRO PID in sim). The `rc/gyro/dterm_lpf` cutoffs in T024 are INAV-side → Phase B. |
| `SIM_TOTAL_TIME_MSEC = 100 s` | `aircraft_state.h:40` | Time-based, stays. Note: 2000 ticks/scenario at 20 Hz ⇒ ~2× eval compute per scenario (training wall-clock impact; operator may choose to shorten scenarios — NOT changed here). |
| xiao firmware | `xiao/include/main.h:24-25` | `MSP_LOOP_INTERVAL_MSEC=50` already; `MSP_NN_EVAL_DIVISOR` 2→1 is **T039 (Phase B, gated)** — untouched now. |

### Servo-constant provenance resolution (operator question 2026-06-10)

The "90 ms vs 0.15 s" servo discrepancy **dissolves on inspection — three different constants**:

- **τ_srv = 0.020 s** is the servo first-order lag (center; per-scenario draw 5–50 ms). Introduced
  9c88216; anchored on the ≤17 ms blackbox sample floor + ~12 ms bench cmd→gyro + 50 Hz-servo
  mechanics. *Upper estimate; bench step-test still TODO (finding.md).*
- **~90 ms** is the **measured lumped cmd→roll-rate lag-to-peak** from the 13-flight xcorr
  (median 101 ms, range 83–117): ≈ transport (≤17 ms) + servo (~20 ms) + roll build-up (~60-70 ms
  of the τ_roll ≈ 192 ms airframe pole). It is a *composite measurement*, not a model constant —
  and it **validates** the τ_srv-in-front-of-τ_roll decomposition.
- **0.150 s is `tau_thr`** — the thrust/spool-up lag center (flight evidence: 477 ms 63% ground-speed
  rise on the 2026-04-03 step, conservatively attributed mostly to airframe mass), NOT a servo
  number. Pre-037 the FDM had **no** servo or thrust lag at all, so 0.15 was never an FDM-native
  servo constant either.
