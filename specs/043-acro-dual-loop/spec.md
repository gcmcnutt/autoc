# Feature Specification: 043 — ACRO dual-loop

**Feature Branch**: `043-acro-dual-loop`
**Created**: 2026-08-23
**Status**: Draft
**Input**: Operator scoping 2026-08-23, after the 041-t7 flight. Derivation-of-record is
[README.md](README.md) — read it for *why*; this document is *what*.

---

## READ-FIRST — document order

| document | role |
|---|---|
| [README.md](README.md) | **derivation-of-record.** The phase budget, the 039 prior art, the transport decision, the arm's-length research question. Superseded where it conflicts with this spec's § Clarifications. |
| **spec.md** (this) | **governs.** Scope, requirements, acceptance. ⭐ Start at **§ What ACRO is** — the rest of the document is downstream of that definition. |
| [`../041-m2-depth/outcome.md`](../041-m2-depth/outcome.md) | the baseline 043 must not regress, and the flight that motivated it. |
| [`../BACKLOG.md`](../BACKLOG.md) | six items are swept into 043 because it opens their files. Listed in FR-070…FR-076. |

---

## Why this feature exists

041-t7 is the best M1 the project has produced — fitness +47% over the all-time record, throttle
de-pegged, airframe loads down, **flown**. One thing did not transfer: **pitch and roll oscillate at
2–5 Hz in the air and not in the sim.** Amplitude matches (roll rate mean |p| 88.4 sim vs 87.3 real);
the *frequency distribution* does not — 3–5 Hz carries 30.1% of real roll-rate power against sim's
12.6%, and roll acceleration is 1.66× sim at matching rate amplitude.

⭐ **This is architecture, not tuning.** The measured phase budget totals **81.6 ms = 147° at 5 Hz**,
and the two dominant terms — the 20 Hz control loop's own ZOH (25 ms) and the actuator (30 ms) — are
**67% of it**. Every filter, baud and loop-rate lever *combined* buys 32 ms and moves the predicted
crossover from 6.1 Hz to 10 Hz, still inside the observed band. ⛔ **You cannot damp a 2–5 Hz limit
cycle from a 20 Hz loop; muting the oscillation and dulling the controller are the same knob at this
bandwidth.**

The fix already flies on this airframe. INAV runs a fixed-wing rate controller at `looptime = 500`
= **2 kHz**, 100× the xiao loop, and ACRO flew every non-autoc segment of the 041-t7 flight rock
solid. 039 measured the case two flights earlier: pitch-rate RMS **24 °/s in INAV acro** versus
**141 °/s NN-direct** — the inner loop is **5.9× better than what autoc does today, on this
airframe, measured** — and concluded the oscillation was *"closed-loop plant sensitivity, not command
roughness"*, which is exactly where 041's independent phase-budget analysis landed.

043 therefore **reframes the NN's job from "what surface deflection right now" — a stabilization task
it is structurally unequipped for at 20 Hz — to "what rate do I want", which 20 Hz handles fine.** The
ZOH and actuator lag move INSIDE a loop with 100× the bandwidth to fight them.

⛔ **This runs BEFORE any M2 training.** M2 inherits whatever control architecture M1 lands on;
training a tracker on top of a control loop we already know oscillates would bake the problem in one
milestone deeper. 044 is sequenced after.

---

## What ACRO **is** — the definition everything here is built on

⭐ **043 targets ACRO exactly as INAV defines it: the command is a rotation RATE.** Everything else in this
section *follows from that definition* — none of it is a separate design choice, and none of it is
negotiable without leaving ACRO.

**The structure, verified in the fork** (`src/main/flight/pid.c`):

1. INAV **always** derives a rate setpoint from the command —
   `rateTarget = pidRcCommandToRate(rcCommand, rates)`, full deflection ⇒ `rate × 10 °/s` ("Step 2").
2. **Only** under `ANGLE` / `HORIZON` / `ANGLEHOLD` does `pidLevel()` then **overwrite** that setpoint with
   `angleError × PID_LEVEL.P`, which the source itself annotates as *"self-leveling strength"* ("Step 3").
3. ⛔ **In ACRO, Step 3 does not run. `attitude.raw[]` is never read.**

⭐ **So ACRO is not "ANGLE without the levelling" — it is a different control problem.** ANGLE is a cascade
whose outer loop regulates *attitude*; ACRO is the inner loop alone, regulating *rate*. ⚠️ ANGLE is also
unusable here for a reason of its own: it only makes sense when zero input means straight and level, and a
chase policy must fly the whole envelope, where "level" is not the goal.

**What follows, by definition:**

| | consequence | where it lands |
|---|---|---|
| **zero command** | *stop rotating* — ⛔ **not** *hold this attitude* | FR-019, SC-012 |
| **attitude** | not a controlled variable. Retention is **emergent** in normal regimes — the operator's *"mostly maintains attitude except at unusual attitudes"* is the accurate claim | FR-019 |
| **self-levelling** | ⛔ **none.** There is no attitude reference, so an excursion that has happened is never corrected back | FR-019a |
| **beyond the envelope** | ⛔ **no recovery behaviour at all.** The loop keeps nulling rate and does nothing else — it holds inverted as readily as level, and descends. **Recovery stays the policy's job, unchanged from MANUAL** | SC-014, Edge Cases |
| **what "roll" and "pitch" mean** | the **IMU's** axes. INAV assumes the IMU is more-or-less aligned with the airframe; it never is exactly | FR-052 |
| **steady balance error** (trim, CG) | absorbed — the I-term winds on to null the *rate* the imbalance produces; the residual attitude offset persists uncorrected | FR-019, FR-059 |

⛔ **The modelling hazard this section exists to prevent.** Implementing "zero command holds attitude" as a
*goal* builds INAV's **ANGLE** mode. It would look correct in every casual check, and it would train a
policy against a self-levelling safety net the aircraft does not have — discovered in the air. **ACRO is
implicitly NOT ANGLE, and the model must be implicitly not ANGLE too.** See FR-019a; ⭐ reusing
`Cntrl_Omega` (FR-014a) is safe on this point by construction, because it is a pure rate controller.

---

## IN scope

1. **Model INAV's fixed-wing ACRO inner loop in crrcsim**, gains **and cascaded rates**, with the
   sensor filters proven active from the flight's own blackbox header.
2. **Pin the actuator model** against a bench servo step-response, and do a **targeted** pass over the
   037-era constants asking which ones the 041-t7 flight actually contradicts.
3. **Answer the arm's-length question** — does the outer loop need new inputs about the inner loop —
   **in the sim, after ACRO is modelled**, not by adding inputs first.
4. **Switch the flight stack from MANUAL to ACRO** under autoc engage, and un-force it on disengage.
5. **Variations**: a written inventory of every variation class and whether it ramps; a realism pass on
   the craft axes; and **a few more craft axes covering IMU imperfection** — mount alignment, gyro/accel
   scale, accel bias — folded into the existing 294 scenarios. ⭐ Incremental realism, not a new
   subsystem: *"craft variations are many and we add a few more."*
6. **A new M1 bake, and a flight**, judged on materially smoother control at non-regressed tracking.
7. **Housekeeping** on the surfaces 043 opens anyway (FR-070…FR-076).
8. **A small, disciplined evaluation of INAV's phase-delay parameters** (FR-012a) — gains stay fixed;
   filter and update-rate delays are re-judged against the *inner* loop's phase margin, which is a
   different budget from the one the README computed.

⛔ **Dropped from scope 2026-08-24**: xiao logging outside autoc spans (was pre-work item 1). See the
first row of § OUT of scope.

## OUT of scope

| what | why | where |
|---|---|---|
| ⛔ **Xiao logging outside autoc spans** (the `FlightStateRecord` enrichment — controls, gyro, accel on non-engaged ticks) | **Dropped 2026-08-24.** Operator: *"isn't worth dragging all that in through msplink. No need for now."* The RC-command half in particular needs a new MSP fetch outside spans. ⚠️ **Accepted cost**: the pilot-flown-vs-NN-flown comparison still needs a **blackbox clock-join**, as 041's did. That join is proven (−970 ppm fit, cross-validated to 0.5% against `ARM\|MANUAL\|MSPRCOVERRIDE`), so this is a known-working path, not a gap. **Re-open trigger**: a flight where the join fails or is unavailable. | BACKLOG (*"Export RC Commands to Xiao Log"*, already filed) |
| **Transport / baud raise (115200 → 230400/460800)** | ⭐ **Latency that is faithfully modelled is a property, not a defect** — the 13 ms MSP fetch is part of the 81.6 ms budget 043 exists to model. Removing 5 ms while the sim still ignores it would *widen* the sim-real gap. The physical layer changes with the 2nd article anyway, and ACRO may make the question moot. | BACKLOG (039 T021), post-ACRO-signal, on the new article, **instrumented first** |
| Camera model / M2 perception | 042 produces the parameters; M2 consumes them | 042 → 044 |
| M2 tracking bake | inherits 043's control architecture, so it waits | 044 |
| Tuning the `fw_*` **gains** (P/I/D/FF) | ⭐ operator: **use `xiao/inav-hb1.cfg` as-is to start** — it is proven on this airframe. ⚠️ **This covers the GAINS only** — phase-delay parameters are now IN scope, see FR-012a | follow-on |
| `servo_pwm_rate` 50 → 200+ Hz | a real ~7 ms lever and now inside the loop being modelled, but it is a **plant change** landing in the same run as an architecture change; ⛔ check the servos are digital first | ⚠️ **moved under FR-012a** as a gated candidate rather than a flat exclusion |
| Renderer playback / streak overlay / X-portability | 043 is a control-loop feature; the renderer is not genuinely opened | BACKLOG |

---

## Clarifications

### Session 2026-08-23 — operator scoping (encoded from README.md)

- Q: Is 043 committed to the architecture change, or is one of the three cheaper 039 levers (forward CG
  ballast, sim static-margin match, static-margin variation axis) enough first? → A: **Committed to
  ACRO.** The 039 backlog entry named this as one of four levers and gated it on `n>1` flight articles;
  **the gate has now fired** and the phase budget independently says the cheap levers cannot reach a
  2–5 Hz limit cycle from a 20 Hz loop. ⚠️ The other three stay on the table and are cheaper — a
  static-margin variation axis in particular is a natural fit for the FR-05x variations work — but they
  are *complements*, not a substitute, and none of them is a gate on starting.
- Q: Which INAV configuration is the model built against? → A: **[`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg)
  as-is**, `platform_type = AIRPLANE`, i.e. `fw_p_pitch 15 / i 5 / d 5`, `fw_p_roll 15 / i 3 / d 7`.
  ⭐ **GAIN tuning is deferred**: the baseline must be the configuration that already flies.
  ⚠️ **Amended 2026-08-24 — phase-delay parameters are a different question and ARE in scope.** Operator:
  *"there might be a param or two to tune in INAV, especially for phase delay impacts."* See FR-012a for
  the reasoning and the discipline.
- Q: What is the key modelling risk? → A: ⭐ **The RATES, not the gains.** Operator: *"modelling the config
  **rates** is key."* A model that implements `fw_p_roll = 15` and stops there will be wrong, because the
  loop's behaviour is set as much by *when* things happen as by *how hard*. These are **cascaded** rates —
  a 2 kHz PID inside a 50 Hz servo inside a 20 Hz outer loop, with sensor filters at 15–25 Hz feeding the
  innermost — and **getting the ratios right matters more than getting any single constant exactly right.**
  A model with all the right gains and one wrong rate will oscillate somewhere the real aircraft does not.
- Q: Do we add inputs so the outer loop can see the inner loop? → A: ⛔ **Not first.** Model ACRO, train
  with the CURRENT 45 inputs, and only then ablate/add. 041's lesson was that inputs added on reasoning
  rather than measurement got de-weighted (the eight added there are *still* unproven). ⭐ **"Nothing" is a
  genuine and cheap outcome** — if the inner loop tracks setpoints well it is a near-ideal actuator, which
  is the definition of "at arm's length" and what 039's 24 °/s *"on a rail"* hints at. See FR-030.
- Q: Where do the constants live? → A: **the existing crrcsim XML**, so the ones we might change stay
  changeable without a rebuild.
- Q: Does the variation work need a bigger population? → A: **No.** Operator: *"the training regiment
  doesn't really need bigger population."* IMU/mount variations fold into the existing **294 scenarios**
  (6 paths × 49 winds) as static-per-scenario draws, exactly like craft — not a new axis count.
- Q: How much mount misalignment is realistic? → A: **±5° in all directions**, operator estimate; the real
  IMU is foam-taped to the underside of the upper surface, and GPS is similar. ⚠️ **Motivation is measured,
  not assumed**: the 041-t7 sensor audit found `accel_y` **1.70σ** off sim at matched distance — sim sits at
  exactly 0.000 (perfectly symmetric, no mount error, because it is computed from FDM truth in
  [`include/autoc/eval/specific_force.h`](../../include/autoc/eval/specific_force.h)) while the real airframe
  carries a constant lateral bias.
- Q: Is the 30 ms actuator term trustworthy? → A: ⚠️ **No — it is from the 037 model, not measured on this
  airframe**, and it is the largest single term in the budget. `servo_pwm_rate = 50` makes at least 10 ms of
  it real; the rest is modelled. ⭐ Servo response is **already** a variation class (`craftServoSlew`,
  `craftServoPwmPhase`, `craftThrustTau`), so the sim can already express a spread — what is missing is
  knowing **where the real airframe sits inside that spread**. Bench step-response is the pre-work
  (FR-020); the **2nd flight article** gives the first real read on spread and is not a blocker.
- Q: How much of the 037 sim model should be re-derived? → A: ⛔ **Not all of it.** The 037-era actuator
  and latency constants were measured on this same airframe and much of it is unmodified. This is a
  **fine-tune**, not a re-derivation: find the ones the 041-t7 flight data actually contradicts and leave
  the rest alone.
---

### Session 2026-08-23 (later) — architecture, after reading the INAV source

⭐ These were settled by **reading `src/main/flight/pid.c` and the flying config**, not by reasoning from
the README. The definitional half is hoisted to **§ What ACRO is**; what remains here is what that
definition *costs and changes*.

- Q: Do the NN's input and output vectors change? → A: **No — and this is the feature's central
  simplification.** Same 45 inputs, same 3 outputs, **same magnitude and same polarity** on both. What
  changes is only how the outputs are *interpreted*: they become rate commands instead of surface
  deflections. ⭐ **The whole architecture change lives on the far side of the wire, in a loop we do not
  write.**
- Q: ACRO or ANGLE, and what does a zero command mean? → A: **ACRO — and the two are different control
  problems, not two settings of one. Zero command means *stop rotating*, never *hold this attitude*.**
  ⭐ Hoisted out of this list into **§ What ACRO is**, which carries the INAV structure, everything that
  follows by definition, and the modelling hazard. ⚠️ The change from MANUAL is real either way:
  centre-stick used to mean *neutral surfaces*, with the aircraft doing whatever trim, CG and asymmetry
  made it do.
- Q: What does that do to the craft-balance variations? → A: ⛔ **It buries them, and this is THE key
  attribute to hold on to while modifying crrcsim** (operator 2026-08-23). The inner loop's integral term
  absorbs exactly the class of steady error the balance axes inject — `craftTrimDelta` (Cm_0), and much of
  `craftCGDelta` and the control-effectiveness deltas. The I-term winds on to null the steady *rate* the
  imbalance produces; the residual attitude offset simply persists, uncorrected, because attitude is not a
  controlled variable. Under MANUAL the outer loop had to learn to compensate these; under ACRO it never
  sees them. ⭐ **They resurface only at PHYSICAL LIMITS** — when the inner loop runs out of authority,
  saturates, or is asked for a rate the airframe cannot deliver.
  Three consequences, all recorded as requirements:
  1. The sim's inner-loop model **must include the I-term** (FR-019). A P- or PD-only model would leave
     trim and CG visible to the policy and would therefore mis-model the *entire variation regime*, not
     merely the trim axis — a wrong answer that looks like a working one.
  2. Every craft axis needs an **observability re-check** (FR-059). An axis the inner loop absorbs is no
     longer training robustness; it is spending scenario diversity and PRNG draws to teach nothing.
  3. The training regiment must actually **reach the limits** where the buried axes reappear, or the
     variation is genuinely inert. ⚠️ This connects to R8: on the config of record full command is
     ±360 °/s roll against a *measured* mean |p| of 88 °/s, so a large part of the upper command range is
     unreachable — which is precisely the saturated region where craft variation becomes visible again.
     The useful small-signal region and the observable region are **not the same region**.
- Q: Is INAV the right place for the fast loop, long-term? → A: ⚠️ **Not established, and 043 does not
  need it to be.** Operator 2026-08-25: *"isn't clear we really want the added complexity of INAV in here —
  it's there for safety and, sure, a FF system — so for now we keep it, and we are building it into the
  toolchain. Sure, xiao or some other AHRS can replace INAV to be dedicated inside our fast loop."*
  ⭐ **So the experiment is architectural, not vendor-specific**: *does offloading basic stability to a
  high-speed loop help, and how much?* INAV is the fast loop that already exists, is already flying, and
  already carries the safety case — not a conclusion that it is the right host.
  Two consequences:
  1. ⭐ **The result must carry a MAGNITUDE, not just a verdict** — *how much* it helps is what decides
     whether a dedicated fast loop is worth building later. See SC-001a.
  2. **Keep the inner-loop model's seam clean.** It is one instantiation of "a fast rate loop"; ⛔ do not
     entangle INAV specifics into the action space or the policy contract, which are already
     vendor-neutral (a rate setpoint in, rates and accels back). A xiao-hosted alternative is filed to
     BACKLOG.
- Q: How ambitious is 043 allowed to get? → A: ⛔ **Not.** Operator 2026-08-25: *"at this point we're just
  looking for a bit more smooth controls."* ⚠️ Read this against the fidelity bar below and the FR-059
  observability work: both are bounded by it. If a strand starts growing into a modelling programme, it has
  left 043.
- Q: How firm are the craft-variation magnitudes? → A: ⚠️ **Provisional, and expected to NARROW.** Operator
  2026-08-25: *"we will probably be a bit more deliberate on getting the craft builds to be a bit more
  repeatable so the variations are reasonable, and then the sim static and dynamic behaviour can be
  sensibly modelled."*
  ⭐ **So do not over-invest in modelling a spread we are about to reduce.** Pick defensible σ now
  (FR-052), record them as assumptions, and treat build repeatability as the thing that eventually makes
  them measurements. ⛔ This caps FR-056's realism pass: characterise, do not chase.
- Q: How faithful does the INAV emulation have to be? → A: ⭐ **A decent stab, then measure, then go from
  there** (operator 2026-08-23). ⛔ **This is NOT a precision-matching exercise, and the spec must not be
  read as demanding one.**
  - **Why this was avoided until now, stated plainly**: *"there are a lot of knobs and loop rate and
    phasing errors to consider."* The deferral was rational — this is the one change with enough
    interacting constants that a bad model is easy to build and hard to detect. It is being done now
    because the phase budget says nothing cheaper reaches the problem, not because it got easier.
  - **The craft operates at its aerodynamic limit**, which is exactly where any FDM is least trustworthy.
    ⚠️ So sim-real divergence *at the limits* is an expected property of the exercise, not a defect to be
    tuned away — and chasing it would be the 023-Phase-9a mistake in a new costume.
  - ⭐ **Therefore the acceptance is BEHAVIOURAL, not numeric**: the bar is *"well, the craft can recover
    to patrol"* — does it fly sensibly, does it come back from an upset, does it hold what it should hold
    — rather than a percentage match against the flight. See SC-002 and SC-014.
  - **What IS required is structural**: the right loop topology, the right *ratios* between the cascaded
    rates, and the right ordering (controller inside the servo, servo inside the outer loop). Consistent
    with the rates answer above — get the structure right and the constants approximately right; ⛔ do not
    spend the feature tuning constants in a regime the plant model cannot resolve anyway.
- Q: Does IMU alignment matter under ACRO, and how much do we model? → A: ⭐ **Yes — materially, and on
  two channels rather than one.** ⚠️ It is tempting to dismiss: alignment does not break attitude
  retention, so it looks like a pure observation defect. That reading is wrong, because **INAV's roll and
  pitch ARE the IMU's axes** (§ What ACRO is) — at ±5° the cross-axis coupling is `sin 5° ≈ 9%` on *every*
  command, which is first-order. Three mechanisms, and they are not the same:

  | | effect of a misaligned IMU | verdict |
  |---|---|---|
  | **zero-rate nulling** | none — the misalignment rotation is invertible, so driving measured rate to zero drives true rate to zero | ✅ the earlier point stands: alignment does **not** break rate nulling in normal regimes |
  | **commanded-rate axes** | ⚠️ **INAV's roll and pitch ARE the IMU's axes.** A commanded pure roll is commanded about the *IMU's* roll axis and lands as roll **+ ~9% pitch** in the airframe | ⛔ material, first-order |
  | **the policy's view of its own goal** | the same FC IMU feeds the gyro, accel and attitude estimate the policy reads, so its whole self-view — and hence the geometry it computes to the target — is consistently rotated | ⛔ material |

  ⭐ **Operator's framing, and the training value**: *"INAV assumes a more-or-less aligned IMU so the axes
  correspond to the craft. Of course no align is perfect, so we should model enough variation that the
  controller controls based on its **error view of its goal**."* That is the point — the policy must fly
  the task from a slightly wrong picture of itself, which is the situation every real airframe presents.
- Q: How much imperfection, and where is the line? → A: **Small and realistic — roughly ±5° of alignment
  and ~5% of scale — and nothing gross.** ⛔ **We do NOT model an upside-down AHRS**, a swapped axis, or a
  90°/180° board-alignment error. Those are *configuration* faults, caught on the bench in minutes, not
  build tolerances; the backlog's 170°-vs-180° board-alignment item is that class and stays out of scope.
  ⭐ **And the framing is deliberately modest**: *"craft variations are many and we add a few more, to be
  closer to reality."* This is **not a new subsystem** — it is a handful of additional axes of exactly the
  kind the craft class already carries (CG, drag, trim, thrust, control effectiveness, servo slew, thrust
  tau, PWM phase) — so they **append to that class**, per FR-054.
- Q: Does adding filtering to crrcsim risk trainability? → A: ⚠️ **Yes, and there is direct in-tree
  evidence.** The abandoned 023 Phase 9a pt3 RC-smoothing experiment stunted training badly — test6 at a
  40 Hz cutoff stuck at −2225 through 55 gens vs −4410 baseline, `pctInStreak` 3% vs 12%; conclusion
  recorded as *"the filter changes dynamics enough that the GA can't find productive policies."*
  ⭐ **That is not a reason to skip this** — ACRO is a rate CONTROLLER, not a command smoother, so it should
  make the plant *easier* where the pt3 filter made it harder. But the acceptance test must include
  **"does a known-good genome still train"**, not only "does it fly". See SC-004.

---

---

### Session 2026-08-25 — decisions taken after the plan landed

⭐ Resolved against [plan.md](plan.md) and [research.md](research.md) rather than in the abstract, so each
answer is informed by what Phase 0 found.

- Q: `craftGyroScale` and a possible inner-loop **authority scale** perturb the same thing from the
  policy's seat — commanded rate ≠ achieved rate. Which carries it? → A: ⭐ **Gyro scale only.**
  ⛔ **No inner-loop gain variation in 043.** The gains are read exactly from the flying config, so varying
  them for *uncertainty* would be theatre, and shipping both mechanisms would double-count the one
  uncertainty that matters most. ⚠️ **This DECLINES research.md's addendum recommendation** — recorded
  because that document still argues for it; spec.md governs. Robustness to a future INAV retune is real
  but is a follow-on concern, not 043's.
- Q: Does a **static-margin / pitch-stability** craft axis ship in 043? → A: ✅ **Yes — as a variation
  axis** (FR-052b). Research Finding 1a is the reason: with a feed-forward-dominant loop the **airframe's**
  open-loop stability sets the response, so this moves from background plant detail to first-order. Varying
  it trains across the spread instead of tuning the sim to one article — the right move at `n = 2`.
  ⭐ **And it splits cleanly in two, which avoids the double-count trap the gyro question just closed:**

  | side | knob | status |
  |---|---|---|
  | **static** (mass) — CG vs neutral point | `craftCGDelta` | ⛔ **already exists** (σ 0.02 MAC ≈ ±7% at `CG_arm 0.28`). **Do not duplicate it.** |
  | **dynamic** (aero) — pitch damping | **`craftCmQ`** ← NEW | the genuine gap |

  ⭐ **The range is already derived in-model.** `hb1_streamer.xml` carries the note *"For variations: Cm_q
  range −3.6 (no streamer) to −5.0 (25 ft streamer)"* — a physically-motivated span, centre −4.2, from the
  streamer's own effect. ⚠️ `Cm_alpha` is not an explicit XML knob (it falls out of geometry); CG carries
  the static side, so `Cm_q` is the tractable dynamic one.
- Q: Keep FR-005, the outbound command sequence number? → A: ⛔ **Cut, with US1.** Same reasoning: not worth
  the firmware churn now, the transport work is deferred, and 041-t7 logged **1,682 of 1,682** valid ticks
  with zero fetch failures. ⭐ **043 therefore makes NO xiao log-format change at all** — a clean
  simplification. The xiao→INAV direction stays uninstrumented; that is recorded as an accepted risk and
  stays with the deferred transport item in BACKLOG.

---

## User Scenarios & Testing *(mandatory)*

### ~~User Story 1 — The flight log stands alone~~ — **DROPPED 2026-08-24**

Cut at the operator's request; see § OUT of scope. The remaining stories keep their numbers so every
`(US2)`…`(US6)` reference below stays valid.

---

### User Story 2 — The simulator flies a rate-commanded aircraft (Priority: P1)

Training runs against a chase aircraft whose commands are **rate setpoints** interpreted by a model of
INAV's fixed-wing rate loop, at the cadences that loop actually runs at, rather than surface deflections
applied directly.

**Why this priority**: it is the feature. Everything downstream — the bake, the flight, the arm's-length
answer — is built on this model being right. It is also fully testable with no hardware.

**Independent Test**: replay the 041-t7 recorded 20 Hz command stream through the new sim and compare the
resulting rate response and its frequency content against the flight's own ACRO segments; separately,
train a short run from a known-good genome and confirm the GA still finds productive policies.

**Acceptance Scenarios**:

1. **Given** a constant rate setpoint, **When** the sim runs to steady state, **Then** the achieved body
   rate converges on the commanded rate with a rise time consistent with the modelled `fw_*` gains.
2. **Given** the recorded flight command stream replayed in sim, **When** roll- and pitch-rate power
   spectra are compared against the flight's ACRO-flown segments, **Then** they agree within the tolerance
   set in SC-002.
3. **Given** all rate constants sourced from the crrcsim XML, **When** a constant is changed, **Then** the
   behaviour changes with **no rebuild**.
4. **Given** the ACRO model enabled, **When** a known-good genome is used to seed a short training run,
   **Then** the GA improves rather than stalling — the 023-Phase-9a trainability guard.

---

### User Story 3 — The aircraft commands ACRO, and the bench proves it (Priority: P2)

Autoc engage puts INAV into ACRO instead of MANUAL, disengage returns the mode to the pilot, and the bench
shows both transitions before anything flies.

**Why this priority**: without it the sim model is unflown. It is separated from US2 because it is testable
entirely at the bench — mode annunciation, servo response to a commanded rate, engage latency — and needs
no trained genome.

**Independent Test**: on the bench, engage and disengage autoc while watching the INAV mode flags and the
servo response, with the GPS disconnected and the bench target flashed first.

**Acceptance Scenarios**:

1. **Given** the xiao streaming RC frames, **When** autoc engages, **Then** INAV reports ACRO — not MANUAL,
   not ANGLE — and rate commands drive the surfaces through the rate PID.
2. **Given** autoc engaged, **When** it disengages, **Then** the flight mode returns to whatever the
   pilot's switch selects, with no residual forcing from the xiao.
3. **Given** `failsafe_recovery_delay = 0`, **When** the xiao's first valid MSP frame arrives, **Then**
   MSPRCOVERRIDE engages without the 200 ms first-frame floor.
4. **Given** the bench link, **When** INAV telemetry is collected as fast as it will go, **Then** the
   achievable rate is measured and recorded — the answer decides whether 60 Hz is enough, rather than being
   assumed either way.

---

### User Story 4 — A new M1, baked and flown, materially smoother (Priority: P2)

A production M1 is trained against the rate-commanded plant and flown on the airframe, and the flight is
materially smoother than 041-t7 without giving up tracking.

**Why this priority**: it is the deliverable outcome, and it is what closes the feature — but it depends on
US2, US3 and US5, and cannot start before them.

**Independent Test**: fly the baked genome and evaluate against SC-001/SC-005/SC-006, from the xiao tick
log plus the blackbox clock-join (the join is required again now that US1 is dropped).

**Acceptance Scenarios**:

1. **Given** the baked genome flown on the airframe, **When** engaged-segment roll- and pitch-rate power
   spectra are computed, **Then** the 3–5 Hz and 5–10 Hz power fractions fall materially toward the sim's,
   per SC-001.
2. **Given** the same flight, **When** tracking occupancy is compared with the 041-t7 baseline, **Then** it
   is within noise — smoothness is not bought by giving up the task.
3. **Given** the same flight, **When** sim and flight are compared distance-standardized on the channels
   041 already validated (throttle, `specific_energy`, `dist_to_boundary`, rate amplitude), **Then** none of
   them regresses.

---

### User Story 5 — Variations that mean something (Priority: P2)

The variation regime is documented class by class — what varies, at what magnitude, and **whether it
ramps** — the craft axes are reviewed for realism, and mount/IMU alignment enters as a new class.

**Why this priority**: operator-scoped **pre-work**, not an afterthought — and the IMU class is motivated
by a *measured* 1.70σ `accel_y` gap, not by reasoning. It is independently valuable (the inventory alone
retires a live documentation contradiction). It sits at P2 rather than P1 because the oscillation fix does
not depend on it, and at P2 rather than P3 because it carries a **format change**, so it must be in the
tree before the single bake starts. ⚠️ It is also the right moment: 043 opens the sensor input path anyway,
and doing IMU first makes the camera case a **second consumer of a proven slot** rather than the
pathfinder.

**Independent Test**: read the inventory against the code and confirm every class is listed with the
correct ramp status; separately, run with the new class at σ=0 and confirm bit-identical results, then at
σ>0 and confirm the draws reach the FDM and are replayable from the recorded seed.

**Acceptance Scenarios**:

1. **Given** the inventory document, **When** it is checked against
   [`scenario_meta_apply.h`](../../include/autoc/eval/scenario_meta_apply.h) and `autoc.ini`, **Then** every
   variation class appears with its ramp status and **the two sources agree** — ⛔ today they do not
   (`autoc.ini` says craft *"RAMPS with wind/entry"*; the code says craft is explicitly **not** ramped).
2. **Given** the new IMU craft axes at σ=0 on every axis, **When** a run executes, **Then** results are
   bit-identical to the class not existing.
3. **Given** σ>0, **When** the same scenario seed is replayed, **Then** the same misalignment is drawn —
   determinism is non-negotiable.
4. **Given** the new class is appended to the sub-seed cascade, **When** existing classes are checked,
   **Then** their sub-seeds are unchanged.

---

### User Story 6 — Housekeeping on the surfaces 043 opens (Priority: P3)

Backlog items whose only cost is that someone has the file open get done while 043 has it open.

**Why this priority**: real value, near-zero marginal cost, zero coupling to the oscillation fix — and each
item is independently droppable if 043's critical path tightens.

**Independent Test**: each item has its own acceptance in FR-070…FR-076 and is verifiable alone.

**Acceptance Scenarios**:

1. **Given** any swept-in item, **When** it is deferred instead of done, **Then** it is recorded as
   deferred in the outcome document rather than silently dropped.

---

### Edge Cases

- **The rate command saturates the inner loop.** The NN commands a rate the airframe cannot achieve (roll
  is configured for ±360 °/s against a measured mean |p| of 88 °/s). The sim must express the shortfall the
  way the real loop does — the aircraft simply does not follow — rather than silently clipping to something
  the real loop would not do. This is also the one condition FR-030's rate-tracking-error candidate exists
  to reveal, ⭐ **and the regime where the craft-balance variations FR-059 says are buried come back**.
- **The useful command range and the observable command range are not the same range.** Small-signal
  commands are where the policy actually operates and where the inner loop hides every balance
  imperfection; the saturated upper range is where those imperfections reappear but the command is no
  longer meaningfully controllable. ⚠️ A variation regime that only bites in the region the policy avoids
  is not training robustness. FR-059a is the check.
- **~~Yaw is commanded to zero, continuously.~~** ⭐ **RESOLVED 2026-08-23 — the airframe has no rudder,**
  so there is no behaviour change here. `msp_override_channels = 47` does override channel 3 and the xiao
  pins it to `MSP_DEFAULT_CHANNEL_VALUE = 1500`, but the servo mixer routes only ROLL and PITCH to its two
  elevon outputs; the yaw axis reaches no surface in either mode. Retained as a *checked-and-dismissed*
  entry rather than deleted, because "ACRO holds yaw rate with rudder" is the obvious wrong inference and
  the next reader will make it too. See FR-018.
- ⛔ **Beyond the commandable envelope there is NO recovery behaviour — none** (§ What ACRO is). At unusual
  attitudes or past aero/control limits the inner loop keeps nulling rate and does nothing else: it holds
  inverted or knife-edge as readily as level, and descends. Upset recovery is the policy's responsibility,
  unchanged from MANUAL — ⚠️ a deliberate property, not a gap to be modelled away.
- **Engage lands mid-manoeuvre with a non-zero body rate.** ACRO engage from a rolling attitude hands the
  inner loop an instantaneous setpoint error; the I-term state at engage is not ours to see.
- **Inner-loop I-term wind-up across a long engage.** Nothing in the current input set would reveal it.
- **A rate command arrives late or is lost.** ⛔ INAV drops a bad `MSP_SET_RAW_RC` frame to `MSP_IDLE` with
  **no counter and holds the previous RC values** — under ACRO, a stale frame is a stale *rate setpoint*
  the 2 kHz loop will faithfully chase. The xiao→INAV direction is uninstrumented on both ends.
- **The pinned 041-t7 comparator becomes unreadable.** ⛔ Any change to `ScenarioMetadata`'s wire format
  orphans every dmp ever written, including the `retain=keep`-pinned 041-t7 baseline. This is the exact
  wall 040 hit. Extraction of anything needed from those dmps must precede the format change — see FR-057.
- **The mount misalignment is large enough to be unflyable.** The class is *diversity*, not *difficulty*;
  a 2.5σ draw must still produce a flyable aircraft.

---

## Execution order — and why it is NOT the priority order

⚠️ **P1/P2/P3 above are PRIORITY, not phase.** In speckit a priority answers *"what do we keep if scope
tightens"*; it does not answer *"what do we do first"*. In 043 the two genuinely disagree, and the reason is
assumption 12: **one bake carries everything.** That makes the bake strictly last and every other story
upstream of it — so US5 (P2) and US6 (P3) both land *before* US4 (P2).

| # | work | story | why it sits here |
|---|---|---|---|
| 1 | ⛔ **FR-057 — extract what is needed from the pinned 041-t7 dmps** | US5 | **permanent if missed.** Must precede any per-scenario metadata format change |
| 2 | variation inventory, craft-realism pass, IMU craft axes | US5 | operator pre-work item 4; carries the format change, so it is upstream of the bake |
| 3 | housekeeping on the opened surfaces | US6 | FR-070 / FR-073 touch the same input path US5 edits — cheaper together than twice |
| 4 | INAV ACRO analysis (R1) + the crrcsim inner-loop model | US2 | operator order-of-work item 2, plus pre-work items 2 and 5 |
| 5 | bench servo step-response; targeted 037-constant review | US2 | FR-020 / FR-021 — pins the largest modelled term before it is trained against |
| 6 | ACRO engage on the stack, bench verification, INAV rate measurement | US3 | operator order-of-work item 3 |
| 7 | the arm's-length answer (R2 / FR-030) | US2 | needs the model *and* a short 45-input baseline; must precede the bake because a "yes" changes the input vector |
| 8 | production bake | US4 | one run, everything in |
| 9 | flight | US4 | |

⭐ **The one-bake decision moves the drop gate to the FRONT.** Priority still says what gets cut if 043's
critical path tightens — US6 first, then US5's optional axes. But because everything must be in the tree
before a single multi-hour run starts, **that cut has to be decided before the bake, not discovered during
it.** ⛔ A P3 item deferred *after* the bake commits is not a deferral; it is a second bake.

⚠️ **Steps 1–3 are the format-break window.** FR-057's extraction, the `ScenarioMetadata` growth and any
NN input-vector change from step 7 should be treated as **one break**, sequenced so the pinned baseline is
mined before anything becomes unreadable.

---

## Requirements *(mandatory)*

### B. The ACRO inner-loop model (US2)

- **FR-010**: The simulated chase aircraft MUST be driven by **rate setpoints** interpreted by a model of
  INAV's fixed-wing rate controller, not by direct surface deflection.
- **FR-011**: The model MUST reproduce the **cascaded rate structure** of the flying configuration, and the
  ratios between the stages MUST be right even where individual constants are approximate:

  | stage | value on the flying config | source |
  |---|---|---|
  | inner PID cadence | **500 µs = 2 kHz** | `looptime` |
  | servo command frame | **50 Hz** (20 ms ⇒ ~10 ms ZOH) | `servo_pwm_rate` |
  | gyro filter | **PT1 25 Hz** (~6.4 ms) | `gyro_main_lpf_hz` |
  | accel filter | **BIQUAD 15 Hz** (~15 ms) | `acc_lpf_hz`, `acc_notch_hz = 0` — ⛔ **OBSERVATION path only**, not the rate loop (see FR-013) |
  | dynamic gyro notch | **30 Hz min, Q 250, 2D** | `dynamic_gyro_notch_*` |
  | D-term filter | **10 Hz PT2** | `dterm_lpf_hz` |
  | MSP service task | **100 Hz, LOW priority** | INAV `TASK_SERIAL` |
  | outer control loop | **20 Hz** (25 ms ZOH) | xiao |

- **FR-012**: The model MUST use the gains of the config of record — `fw_p_pitch 15 / i 5 / d 5 / ff 70`,
  `fw_p_roll 15 / i 3 / d 7 / ff 50` — and MUST NOT retune them as part of this feature.
- **FR-012a**: ⭐ **A small number of INAV PHASE-DELAY parameters MAY be tuned, and the candidates MUST be
  evaluated** (operator 2026-08-24). ⛔ This is **not** a relaxation of FR-012 — the `fw_*` gains stay
  fixed. It is a separate question, and the reframing is what makes it live:

  > **A sensor delay that was negligible in the outer loop's budget is not negligible in the inner
  > loop's.** The gyro PT1 at 25 Hz costs 6.4 ms — **11° at 5 Hz**, which is why the README correctly
  > dismissed it as 8% of the 81.6 ms outer-loop budget. But the same 6.4 ms is **115° at 50 Hz**, and the
  > 2 kHz inner loop is trying to have authority up there. ⚠️ **The filters move from a rounding error to a
  > phase-margin term the moment the loop closes at a higher frequency.**

  Candidates, each to be judged on measured phase contribution and none pre-approved:

  | parameter | now | delay | note |
  |---|---|---:|---|
  | `gyro_main_lpf_hz` | 25 (PT1) | 6.4 ms | inside the inner loop; measured roll-off is steeper than PT1 alone (dynamic notch @30 Hz, Q 250) |
  | `dterm_lpf_hz` / type | 10 / PT2 | — | shapes the inner loop's own phase margin directly |
  | ~~`acc_lpf_hz`~~ | 15 (BIQUAD) | 15 ms | ⛔ **removed 2026-08-25 — not in the ACRO loop** (FR-013). Observation-path only; revisit under a different argument if at all |
  | `dynamic_gyro_notch_*` | 30 Hz / **Q 2.5** (`q/100`) | — | ⚠️ broad, not razor — but it *tracks the prop peak*, so it may sit far above the control band. ⛔ **Read the logged centre frequency from the 041-t7 blackbox before deciding** |
  | `servo_pwm_rate` | 50 Hz | ~10 ms | ⛔ a plant change and gated on the servos being digital — evaluate, do not assume |

  ⛔ **Four disciplines, all mandatory**:
  1. **Motivated by arithmetic, not instinct** — a computed phase contribution at the frequency the inner
     loop is trying to control, before the value is touched.
  2. **Mirrored in the sim.** Any parameter changed on the aircraft MUST be changed identically in the
     model. ⚠️ Changing one side only *widens* the sim-real gap — the exact error the transport deferral
     was argued on.
  3. **Bench-verified**, and folded into the config of record before the bake, since the policy trains
     against whatever is set.
  4. **A small number.** ⭐ Operator: *"a param or two."* Every change costs attribution in a feature that
     already ships one bake with everything.
- **FR-013**: The sensor filters MUST be modelled **in the path that actually reads them**, because that is
  where their group delay costs phase margin. Values MUST come from the flight's own blackbox header, the
  verified source, not the `.cfg` alone.
  ⛔ **Corrected 2026-08-25 — only ONE sensor filter is inside the ACRO loop.**
  `pidApplyFixedWingRateController` reads `pidState->gyroRate` and nothing else; the accelerometer reaches
  the controller only via `attitude.raw[]`, which **only ANGLE/HORIZON read**.

  | filter | path | contributes phase to |
  |---|---|---|
  | `gyro_main_lpf_hz` 25 PT1 (6.4 ms) | ⭐ **inside the rate loop** | the inner loop's own margin |
  | `acc_lpf_hz` 15 BIQUAD (15 ms) | **observation only** — the policy's `ACCEL_*` and `quat` | ⛔ **nothing** in ACRO |
  | dynamic gyro notch, 30 Hz **Q = 2.5** (`q/100`) | inside the rate loop *if* it sits near its floor | ⚠️ open — see research.md addendum D |

  ⚠️ Consequence for FR-012a: `acc_lpf_hz` drops off the phase-delay candidate list **on inner-loop
  grounds**. It may still be revisited for observation fidelity, which is a different argument.
- **FR-014**: Every rate, gain and filter constant introduced by FR-010…FR-013 MUST be settable from the
  existing crrcsim model XML so it can be changed **without a rebuild**.
- **FR-014a**: ✅ **SATISFIED IN PHASE 0 — no task.** Evaluated in [research.md](research.md) R9 and decided
  by the operator 2026-08-25: **reuse the framework, write the math** (`Cntrl_InavFwRate`). Recorded here so
  coverage checks stop reading it as a gap.
  ⭐ **CRRCSim ALREADY HAS a rate controller, and the plan MUST evaluate reusing it before
  writing a new one.** Verified 2026-08-23 (operator's *"there may be a rate controller in CRRCSim
  already — worth the research"*, confirmed):

  | what exists | where |
  |---|---|
  | `Cntrl_Omega` — documented as *"This PID-controller makes stick inputs control rotation rates"* | `crrcsim/src/mod_cntrl/cntrl_omega/` |
  | per-axis `scale` / `kp` / `ki` / `kd`, PT1-filtered D term (`tau_d`), integrator with anti-windup | same |
  | **gains loaded from XML** — matching the operator's "constants in the existing XML" scoping | `Controller::LoadList`, `crrcsim/src/mod_cntrl/controller.cpp` |
  | the FDM autoc uses **already calls the controller hook**, per substep, and — ⭐ critically — **before** the 037 servo model and `aero()` | `fdm_larcsim.cpp:246` vs the servo block at ~282 |

  ⭐ **The call-order is already physically right**: user input → controller → servo latch/slew → aero, so
  the servo lag correctly sits **inside** the rate loop, which is exactly what makes the modelled inner
  loop's phase margin honest. ⚠️ Nothing is active today — `hb1_streamer.xml` has no `<controllers>` node.
  ⛔ **Four gaps MUST be closed or consciously accepted before reuse**, and each is a fidelity question the
  spec does not pre-decide:
  1. **Feed-forward is missing.** INAV's fixed-wing gains are `fw_ff_pitch 70` / `fw_ff_roll 50` against
     `fw_p 15` — **FF dominates P**. `Cntrl_Omega` is P/I/D only. A PID-only model would mis-model the
     loop badly, and in the direction that makes it look worse than it is.
  2. **Loop rate.** The controller runs at the FDM substep — `simulation.flightModel.dt` defaults to
     0.002777 s and is ms-quantized, i.e. **~333 Hz** — against INAV's **2 kHz**. The ZOH phase difference
     at 5 Hz is small (~2.3°) but the integrator and D-term discretization differ more. State the as-run
     value and justify the ratio; ⛔ do not assume it.
  3. **Anti-windup differs.** `Cntrl_Omega` uses conditional integration gated on `|pd| > 1`; INAV has its
     own I-term limiting. Since FR-019 makes the I-term load-bearing for the entire variation regime, the
     windup scheme is not an implementation detail here.
  4. **Expo.** `Cntrl_Omega` applies a *"massive expo for loops and flips"* on the stick→setpoint map;
     INAV's stabilized rate map is linear (`rate × 10 °/s`). Neutralize it or match INAV, and check whether
     `rc_expo` applies upstream on this config.

  ⚠️ Also a scoping decision, not a defect: controllers currently load from the **global** config
  (`crrc_fdm.cpp:38`), not per-model. If inner-loop gains are ever to vary per scenario (FR-056), per-model
  loading — which `fdm_mcopter01` already does — is the pattern to follow.
- **FR-015**: The model MUST be **deterministic** — same seed and configuration reproduce the same run, and
  the eval-vs-training bitwise match remains the regression gate.
- **FR-016**: The commanded-rate action space MUST be documented with its scaling **and its polarity**,
  in one place shared by sim and firmware so the two cannot disagree. ⚠️ On the config of record INAV maps
  full command deflection to `rate × 10 °/s`, giving **±360 °/s roll, ±120 °/s pitch, ±40 °/s yaw** — a 3:1
  roll/pitch asymmetry against a *measured* roll-rate mean |p| of 88 °/s. 043 does not change these (they
  are part of "use the config as-is").
  ⛔ **Polarity MUST be verified end to end, not assumed.** NN output sign → PWM about 1500 → INAV
  `rcCommand` sign → commanded rate sign → achieved body rate sign, checked on the bench against the sim.
  The output vector is unchanged in magnitude *and* polarity (§ Clarifications), so a sign error here would
  be invisible in every artifact that survived the change and would only appear in the air. This is the
  same class of defect as the six stale labels 041 found, in the one place it cannot be tolerated —
  and `msplink` already flips y/z at the FRD↔FLU boundary for the quat, gyro and accel, so a frame
  convention is genuinely in play.
- **FR-017**: Throttle MUST remain a direct command, not a rate — it has no inner-loop equivalent.
- **FR-018**: Yaw MUST be modelled as **having no actuator**. ⭐ **Verified 2026-08-23 — this airframe has
  no rudder.** The servo mixer defines exactly two outputs, each combining stabilized ROLL and PITCH
  (`smix 0/1` → servo 1, `smix 2/3` → servo 2) — elevons; yaw (input 2) appears in **no** mix rule, and the
  FDM's servo model slews aileron and elevator only. So the xiao's centred yaw channel is a commanded zero
  yaw rate that INAV's rate PID computes and then sends nowhere: **inert, on both sides.** The requirement
  is therefore to *record and verify* this rather than to decide it — the bench check (FR-045) confirms no
  surface responds to the yaw axis, and the sim model MUST NOT introduce a yaw actuator that the aircraft
  does not have. ⚠️ The consequence that is NOT inert: with no rudder, sideslip and adverse yaw are
  uncontrolled in both regimes, so the inner loop's authority over the oscillation is **roll and pitch
  only** — which is also exactly the pair that oscillates.

- **FR-019**: ⭐ **The inner-loop model MUST include the integral term**, with accumulation and limiting
  behaviour representative of INAV's fixed-wing PID (`fw_i_pitch 5`, `fw_i_roll 3`). This is not a
  completeness nicety — the I-term is the mechanism by which the inner loop absorbs steady-state balance
  error, and it is therefore the mechanism that makes FR-059's observability question have the answer it
  has. ⛔ A P- or PD-only model would leave trim and CG variation visible to the policy and would
  mis-model the whole variation regime while otherwise looking correct.
  **Qualitative acceptance, and the model's single most important check**: with a **zero rate command**, a
  craft carrying a **non-zero trim / CG draw** MUST settle to **zero measured body rate** and stay there —
  the same behaviour as a nominally-balanced craft, reached by the I-term winding on to null the steady
  trim moment. If a mis-trimmed craft keeps rotating under zero command, the I-term is not modelled.
  ⚠️ **The controlled variable is RATE, not attitude** (§ What ACRO is).
  ⚠️ **And only within physical limits** (operator 2026-08-23): beyond the airframe's aerodynamic or
  control authority the loop cannot null rate, and the model must fail the same way the aircraft does.
  ⛔ **The check is a sweep across ALL attitudes, and it runs BEFORE autoc is connected** — in sim across
  the attitude sphere, and at the bench — so a failure is attributable to the inner-loop model rather than
  discovered later as a policy that behaves oddly in one corner. A gate on the model, not a training
  observation.

- **FR-019a**: ⛔ **The model MUST NOT implement self-levelling, attitude hold, or any attitude feedback.**
  The inner loop MUST read **body rates only**; an attitude term anywhere inside it is a defect, not a
  refinement — it builds ANGLE, not ACRO (§ What ACRO is, "the modelling hazard"). ⭐ `Cntrl_Omega`
  (FR-014a) is safe here by construction: it is a pure rate controller.
- **FR-019b**: The **setpoint acceleration limit** MUST be modelled as configured. INAV applies
  `pidApplySetpointRateLimiting` to the rate target; on the config of record
  `rate_accel_limit_roll_pitch = 0` (**off**) and `rate_accel_limit_yaw = 10000`. ⭐ Roll/pitch being
  *unlimited* is itself a modelling fact worth carrying — the rate setpoint can step, and the loop's
  response to a step is what the 20 Hz outer loop will actually deliver every tick.

### C. Pinning the plant (US2 pre-work)

- **FR-020**: A **bench servo step-response** MUST be measured on the flight article and used to place the
  real actuator inside the existing `craftServoSlew` / `craftServoPwmPhase` spread. The 30 ms actuator term
  is the largest modelled contributor to the phase budget and is currently **unmeasured on this airframe**.
- **FR-021**: A **targeted** review of the 037-era latency and actuator constants MUST identify which ones
  the 041-t7 flight data actually contradicts. ⛔ Constants the data does not contradict MUST be left alone
  — this is a fine-tune of a model measured on this same, largely unmodified airframe, not a re-derivation.
- **FR-022**: The review's conclusions MUST be recorded per constant with the evidence, so a later reader
  can tell "checked and unchanged" from "not checked".

### D. The arm's-length question (US2, research-gated)

- **FR-030**: 043 MUST answer, **in the sim and after ACRO is modelled**, whether the outer loop needs new
  visibility into the inner loop. The method is a baseline trained on the **current 45 inputs**, against
  which candidates are then ablated or added:

  | candidate | what it would reveal | note |
  |---|---|---|
  | **nothing** | ⭐ the genuine and cheapest outcome | if the inner loop tracks setpoints well it is a near-ideal actuator — the definition of "at arm's length" |
  | **efference copy** (its own previous command) | the net sees the *result* of its last setpoint but not the setpoint itself | ⚠️ the RNN *may* already encode this: `W_hh` effective rank is 11.2–11.4 of 16, so capacity is unused — but "unused" is not "unable" |
  | **rate-tracking error** (commanded − achieved) | inner-loop saturation or rate limiting; what a pilot feels as *"it isn't following"* | the only channel that would show it |
  | **inner-loop health** (I-term, output saturation) | wind-up and authority limits | ⛔ needs a new MSP field; do not add speculatively |

- **FR-031**: ⛔ New inputs MUST NOT be added ahead of that measurement. A candidate earns its place against
  the 45-input baseline or it does not ship.

### E. The flight stack (US3)

- **FR-040**: Autoc engage MUST place INAV in **ACRO** — neither MANUAL nor ANGLE. ⭐ On the config of
  record this is reachable without an INAV change: the aux-2 channel band that currently selects MANUAL at
  1000 selects ACRO in its mid-band.
- **FR-041**: On disengage the xiao MUST stop forcing the mode, returning flight-mode selection to the
  pilot's switch. Today the mode channel is forced on **every** frame the xiao sends.
- **FR-042**: MSPRCOVERRIDE engage MUST NOT pay the **200 ms first-frame floor** at
  `failsafe_recovery_delay = 0`. ⚠️ Engage latency matters more in a dual-loop design than it did before.
  This is an INAV fork change (`mspOverrideInit` / `mspOverrideCalculateChannels`) and therefore a **third
  build surface** for this feature.
- **FR-043**: INAV MUST be built and deployed for **both targets** — bench `MAMBAF722_2022A` first, then
  flight `MATEKF722MINI`. ⚠️ Disconnect the GPS before flashing.
- **FR-044**: The **achievable INAV telemetry rate** MUST be measured on the bench and recorded.
  `blackbox_rate_denom = 32` gives 60 Hz today; ⭐ the operator's expectation is *"maybe with acro we don't
  need to"* go faster — **the point of measuring is to find out**, and either answer is a result.
- **FR-045**: Bench verification MUST cover mode entry, mode exit, rate response at the surfaces, and the
  yaw behaviour of FR-018, **before** any flight.

### F. Variations (US5)

- **FR-050**: 043 MUST produce a **written inventory of every variation class**: what it varies, its
  magnitude, its enable knob, whether it is drawn per scenario, and **whether the difficulty ramp applies
  to it**. The classes as they stand are wind, rabbit, entry, craft and camera.
- **FR-051**: The inventory MUST reconcile the code with the configuration comments, and the disagreement
  MUST be fixed rather than documented. ⛔ `autoc.ini` currently states craft *"RAMPS with wind/entry (same
  VariationRampStep)"*; `scenario_meta_apply.h` states the opposite in detail and is what actually runs —
  **only the environmental classes ramp**, on the rationale that the ramp exists so a fresh population is
  not killed by unflyable *difficulty*, and a varied airframe is *diversity*, not difficulty.
- **FR-052**: **A few more craft axes MUST be added covering IMU imperfection** — static per scenario like
  every other craft axis, and framed as incremental realism rather than a new subsystem (§ Clarifications).
  ⛔ **There is no perfect IMU, accelerometer or gyro**; the axes are, at minimum:

  | axis | what it represents | magnitude anchor (2.5σ = the hard limit) |
  |---|---|---|
  | mount **attitude** error, 3-axis | the IMU is foam-taped to the underside of the upper surface | **±5°** (operator estimate; GPS similar) ⇒ σ = 2.0° |
  | **gyro scale** error, per axis | sensor and calibration tolerance | **~5%** ⇒ σ = 2% |
  | **accel scale** error, per axis | same | **~5%** ⇒ σ = 2% |
  | **accel bias** | ⭐ measured, not assumed — the 041-t7 audit found `accel_y` **1.70σ** off sim, where sim sits at exactly 0.000 | set from that measurement |

  ⚠️ Sigmas MUST be set so that **2.5σ equals the intended limit**, using the pipeline-wide truncation —
  ⛔ no bespoke clip constants (the convention `camera_variation.h` documents).
- **FR-052b**: A **pitch-damping craft axis (`craftCmQ`) MUST be added**. ⭐ Rationale is research Finding
  1a: under a feed-forward-dominant loop the airframe's own open-loop stability sets the response, so
  pitch stability is first-order for the modelled inner loop, not background plant detail.
  ⭐ **Range is already derived in-model** — `hb1_streamer.xml`: *"For variations: Cm_q range −3.6 (no
  streamer) to −5.0 (25 ft streamer)"*. Centre **−4.2**, σ set so 2.5σ spans that range, clamped to it.
  ⛔ **The static side is NOT duplicated.** `craftCGDelta` already varies CG against the neutral point,
  which *is* static margin on the mass side. This axis carries the **dynamic** side only. ⚠️ The plan MUST
  check the two do not double-count — the same discipline that resolved the gyro-scale question.
  ⚠️ `Cm_alpha` is not an explicit XML knob (it falls out of geometry) and is therefore not an axis; if R4
  finds the flight contradicts the modelled static margin, that is a **constant correction**, not a draw.
- **FR-052a**: ⛔ **Gross misconfiguration is OUT of scope** — no upside-down AHRS, no swapped axis, no
  90°/180° board-alignment error. Those are configuration faults caught at the bench, not build
  tolerances, and modelling them would spend training capacity on a failure the pre-flight catches. The
  backlog's 170°-vs-180° board-alignment item is that class and stays out.
- **FR-053**: The new axes MUST be a **true no-op at σ=0** — bit-identical to them not existing.
- **FR-054**: ⭐ **Append to the existing craft class**, not a new PRNG class. Craft is already the home for
  per-airframe build tolerance (servo slew, control-horn slop, trim); IMU mount error is the same category;
  and `craft_variation.h` documents the recipe — *"append the draw at the bottom and append the field to
  `ScenarioMetadata` + `CraftDeltas` last"* — which keeps every existing draw's value unchanged.
  ⚠️ A **separate sixth class** is justified only if the axes must be toggleable independently of
  `EnableCraftVariations`. If the plan wants that, its sub-seed MUST be appended after `camera` so the five
  existing sub-seeds are unchanged. ⚠️ The backlog's *"5th PRNG class slot"* framing is stale either way —
  camera took the fifth slot in 040.
  Whichever route, **draw-and-discard** MUST hold so toggling cannot shift another class's draws.
- **FR-055**: Like the rest of craft, the new axes MUST **not** be ramped: diversity, not difficulty.
- **FR-056**: The **craft** axes MUST be reviewed for realism now that `n = 2` flight articles exist —
  in particular AHRS alignment, control-surface trim/bias, and control response gains and rates, which is
  where the ACRO change concentrates its sensitivity. ⭐ **Resolved 2026-08-25**: the pitch-stability lever from the 039 list ships, split as **FR-052b**
  (`craftCmQ`, dynamic) with the static side already carried by the existing `craftCGDelta`. Axes whose spread is still an assumption MUST be
  labelled as such.
- **FR-057**: ⛔ Before any change to the persisted per-scenario metadata format, everything still needed
  from the **pinned 041-t7 baseline dmps** MUST be extracted to a format that survives the break. That
  baseline is `retain=keep` in `s3://autoc-m1/autoc-9223370249590214474-2026-08-20T22:22:41.333Z/` and the
  format change makes it unreadable. This is the 041 T011a hazard repeating, and it is permanent if missed.
- **FR-058**: Variations MUST fold into the **existing 294 scenarios**; population size is not increased.
- **FR-059**: ⭐ **Every variation axis MUST be re-checked for OBSERVABILITY under the rate-commanded
  architecture**, and the result recorded per axis. The inner loop absorbs steady balance error (§
  Clarifications), so axes that trained real robustness under MANUAL may now teach nothing:

  | axis | expected under ACRO | must be measured, not assumed |
  |---|---|---|
  | `craftTrimDelta` (Cm_0) | ⛔ **absorbed** — this is precisely what an I-term eats | does any residual reach the policy at all? |
  | `craftCGDelta` | ⚠️ **largely absorbed**; changes the plant the PID sees, not the attitude it holds | shows up as inner-loop response shape, and at limits |
  | `craftPitchEffDelta` / `craftRollEffDelta` | ⚠️ **compensated up to saturation** — the PID trades deflection for the same rate | becomes visible exactly when authority runs out |
  | `craftServoSlew` / `craftServoPwmPhase` | ⭐ **more important, not less** — now INSIDE the loop being modelled, so it sets the inner loop's own phase margin | see FR-020 |
  | `craftThrustScale` / `craftThrustTau` | ✅ **unaffected** — throttle is not rate-commanded (FR-017) | — |
  | `craftDragDelta` | ✅ **visible** — energy state, not attitude | — |
  | mount / IMU (FR-052) | ⭐ **visible, on BOTH channels.** It does *not* break zero-rate nulling — the misalignment rotation is invertible, so *"alignment shouldn't impact attitude hold in normal commandable regimens"* holds. But **INAV's roll and pitch ARE the IMU's axes**, so ±5° puts **~9% cross-axis coupling on every command** (first-order), *and* biases the attitude estimate and accel channels the policy reads — where the measured 1.70σ `accel_y` gap lives | the strongest-justified of the new axes: it **is** the *"error view of its goal"* the policy must fly from |

  ⚠️ **Read this audit's verdicts as SIM verdicts.** FR-059a says the buried axes resurface at the aero /
  power limits, and § Clarifications says the limits are exactly where the plant model is least
  trustworthy. Those are the same regime. The audit is still worth doing — it is the only way to find dead
  sigma before spending a bake on it — but its conclusions are **provisional on the flight**, and an axis
  should not be permanently retired on a sim-only reading taken in the regime the sim resolves worst.
  An axis measured as inert MUST be either **retired**, **re-scoped to where it is observable**, or
  **explicitly kept with its inertness recorded** — ⛔ silently keeping dead sigma spends scenario
  diversity and PRNG draws to teach nothing, and reads in every report as robustness that was trained.
- **FR-059a**: ⭐ **The limits are reached easily, and the dominant mechanism is ENERGY, not rate
  saturation.** Operator 2026-08-23: *"this is an underpowered craft — we will hit aero or control limits
  easily. I mean hold attitude and drop power. Often the craft will maintain attitude and descend."*
  ⛔ **This corrects the framing above.** A rate controller holds **rotation**, not **trajectory**: at
  reduced power the inner loop will faithfully hold a nose-up attitude while the aircraft mushes and
  descends. So the buried craft-balance variation does not principally resurface through commanded-rate
  saturation (the rare case) — it resurfaces through the **energy channel** (the common one), which is:
  1. ⭐ **already observable to the policy** — `SPECIFIC_ENERGY` and `AIRSPEED` are existing inputs, so
     the information the inner loop removed from the *attitude* channel reappears on a channel the network
     already has. The variation is **transformed, not deleted**.
  2. **plausibly how trim reappears too, and this is a research item not a claim**: the I-term holds
     attitude by parking a *deflected* surface, and a deflected surface costs drag —
     `hb1_streamer.xml` carries `CD_ELsq = 0.05`, annotated in-model as adding *"significant drag at full
     elevator"*. If that mechanism is real, `craftTrimDelta` becomes an efficiency penalty rather than an
     attitude bias. ⚠️ **Measure it; do not assume it** (FR-059's audit is where the answer lands).
  The regiment MUST therefore be checked for whether it actually reaches these limits — and the check is
  about **power-limited and aerodynamically-limited flight**, not only about high commanded rates.

### G. Bake and flight (US4)

- **FR-060**: A production M1 MUST be trained against the rate-commanded plant, launched per Constitution IX
  (detached, via `scripts/train.sh`) and gated per Constitution IX's pre-run build gate.
- **FR-061**: The run MUST be pinned and manifested per Constitution VIII, including the input scale
  constants — ⛔ without them a genome loads clean and flies wrong.
- **FR-062**: The baked genome MUST be flown, and the flight evaluated against SC-001…SC-006. ⚠️ With US1
  dropped, engaged-segment data comes from the xiao tick log and everything else from a **blackbox
  clock-join** — the proven 041 path. The join MUST be cross-validated as 041's was.
- **FR-063**: The flight MUST be preceded by a bench verification of the deployed firmware and INAV build.

### H. Housekeeping swept in (US6)

- **FR-070**: **Formal input normalization from measured statistics** rather than hand-derived constants.
  ⭐ HIGH VALUE / LOW COST — 041's whole result came from discovering a 200:1 input-scale spread that made
  quiet slots unreachable, and FR-052 touches the input path anyway.
- **FR-071**: `nnextractor -g` takes the FILE number while `dmp-dump --gen` takes the GENERATION — a live
  footgun that cost real time during 041 analysis. Make the two agree or make each say which it means.
- **FR-072**: crrcsim's `mod_inputdev` MUST link `autoc_common` rather than cherry-picking individual
  source files, which silently breaks the crrcsim link whenever a referenced file moves. 043 opens
  `inputdev_autoc.cpp` for FR-010.
- **FR-073**: **Type-safe NN sensor interface** — FR-052 edits the sensor interface, and all six stale-label
  bugs found in 041 were "a caller was not updated".
- **FR-074**: **Simulator sampling-time variation** — the 20 Hz ZOH is 25 ms of the 81.6 ms budget, so
  varying it is directly on-topic for this feature.
- **FR-075**: ✅ **BY DESIGN, no task.** The **abandoned pt3 filter** experiment MUST be carried as a **warning, not a task** — it is
  the evidence behind SC-004's trainability guard.
- **FR-076**: Any FR-07x item not done MUST be recorded as deferred in the outcome document.

---

## Key Entities

- **Rate setpoint** — the new action-space element: a commanded body rotation rate for roll and pitch (and
  a decision on yaw per FR-018), replacing commanded surface deflection. Shares one scaling definition
  between sim and firmware.
- **Inner-loop model** — the simulated stand-in for INAV's fixed-wing rate controller: gains, cadence,
  sensor filters, D-term filtering, and the servo command frame beneath it.
- **Phase budget** — the accounting of every delay between "the NN decides" and "the airframe responds",
  and the quantity this feature exists to restructure.
- **Variation class** — a named, per-scenario source of diversity with its own PRNG sub-seed, enable knob,
  sigma set, and ramp status. Currently wind, rabbit, entry, craft, camera; 043 adds mount/IMU.
- **Baseline of record** — 041-t7 (`gen 633` flown 2026-08-23), the pinned run every 043 comparison is
  made against and the one FR-057 protects.

---

## Success Criteria *(mandatory)*

### Measurable outcomes

- **SC-001** — **Materially smoother in the air.** On the 043 flight's engaged segments, roll- and
  pitch-rate power redistributes back **down** in frequency relative to 041-t7, measured against that
  flight's own spectra as the reference:

  | roll-rate power | 041-t7 real | 041-t7 sim | 043 target |
  |---|---:|---:|---|
  | 0–1 Hz | 30.6% | 55.9% | up |
  | 3–5 Hz | **30.1%** | 12.6% | down |
  | 5–10 Hz | **7.4%** | 2.3% | down |

  ⚠️ **SUBJECTIVE INITIALLY, and deliberately so** (operator 2026-08-23, the same call 041 made for its
  AC-1). No numeric threshold yet: quantifying the bar before the strategy has shown it can work is how a
  gate gets loosened later to fit the result. Refine **after** ACRO demonstrates signal, not before. The
  supporting time-domain reference is 039's measured pitch-rate RMS by regime — INAV acro **24 °/s**
  (*"on a rail"*), pilot MANUAL 50, NN-direct **141** — reported alongside the spectra.
- **SC-001a** — **The result carries a MAGNITUDE.** The outcome states *how much* offloading stabilization
  to the fast loop helped — quantified against 041-t7 on the same measures — not merely whether it did.
  ⭐ This is what makes the finding portable to the question behind the experiment: whether a dedicated
  fast loop (xiao-hosted or otherwise) is worth building. ⚠️ A null or small result is a valid and useful
  answer; ⛔ an unquantified "it seems better" is not.
- **SC-002** — **The emulation is structurally faithful, and its divergence is MEASURED.** Replaying the
  recorded command stream through the ACRO model produces a rate response of the right character against
  the flight's ACRO-flown segments, and wherever it diverges the divergence is **quantified and recorded**
  rather than tuned away. ⛔ **Not a numeric match gate** — the operator's bar is a *decent stab, measured*
  (§ Clarifications). What must hold is structural: loop topology, the ratios between the cascaded rates,
  and the ordering (controller inside servo inside outer loop). ⚠️ Divergence **at the aerodynamic limit is
  expected**, because that is where the plant model itself is weakest; it is recorded as a known limit of
  the exercise, not treated as a defect.
- **SC-003** — **Tracking does not regress.** Tracking occupancy on the new M1 is within noise of the
  041-t7 baseline, measured with the same definition on both sides. ⚠️ Smoothness bought by giving up the
  task is the failure mode this criterion exists to catch, and it is a *compound* gate with SC-001 for the
  same reason 041's AC-1 was: either half alone can certify a worse controller.
- **SC-004** — **Trainability survives the plant change.** A known-good genome still improves under the
  ACRO model, and the production run's fitness trajectory does not stall in the way 023 Phase 9a's filtered
  runs did (best stuck at −2225 through 55 gens; `pctInStreak` 3% vs 12%).
- **SC-005** — **Nothing 041 proved regresses.** Distance-standardized sim-vs-flight agreement holds on
  throttle, `specific_energy`, `dist_to_boundary` and rate amplitude — the channels 041-t7 validated at
  0.04σ–0.11σ.
- **SC-006** — **Loop health holds.** The flight records zero fetch failures, zero overruns, zero resyncs
  and zero tick gaps, as 041-t7 did across 1,682 ticks. ⚠️ The xiao→INAV direction stays uninstrumented (FR-005 cut 2026-08-25); zero *losses*
  is not zero *errors*, and that gap rides with the deferred transport item.
- **SC-008** — **The variation regime is legible.** A reader can determine, for every class, whether it
  ramps, from a single document that the code agrees with.
- **SC-009** — **The new IMU axes are real and free when off.** σ=0 is bit-identical to them not existing;
  σ>0 is replayable from the recorded seed; every pre-existing variation draw is unchanged in value. ⭐ And
  the policy trains successfully *from an imperfect self-view* — the axes are there so the controller
  learns to pursue its goal through its own measurement error, not so a report can say they exist.
- **SC-010** — **The actuator term is measured, not modelled.** The bench step-response places the real
  airframe inside the servo variation spread, and every 037-era constant is marked either "contradicted by
  041-t7 and changed" or "checked and unchanged".
- **SC-011** — **The arm's-length question is answered with evidence.** The outcome states whether the
  outer loop needs inner-loop visibility, supported by a measurement against the 45-input baseline —
  including if the answer is "nothing".
- **SC-012** — **Zero command drives RATE to zero, mis-trimmed or not.** In sim, a craft carrying a
  non-zero trim/CG draw settles to zero measured body rate under a zero rate command and stays there,
  indistinguishable from a nominally-balanced craft. ⭐ This proves the I-term is modelled and is the
  qualitative gate on the inner-loop model (FR-019). ⛔ **The converse is also a pass condition**: the
  model must show **no self-levelling**. ⚠️ **Stated carefully** (operator 2026-08-25) — with no attitude
  reference the bank is *"sort of held but not a solid force; it will eventually drift, on the order of
  seconds."* So **drift is expected and correct**, and "holds the bank" is not the test.
  ⭐ **The discriminator is sign correlation, not stability**: from +30° and −30° bank, ANGLE drives *both*
  toward zero — deviation correlated with bank sign — while ACRO's drift is uncorrelated with it. A
  sign-correlated restoring trend means ANGLE has been built by accident (FR-019a).
  ⚠️ Note the asymmetry with the rate test above: **rate is controlled and should null tightly; attitude is
  not controlled and should not.** A model where attitude holds *better* than rate is suspect.
- **SC-013** — **No dead sigma.** Every variation axis is recorded as observable, absorbed, or
  observable-only-at-limits under the rate-commanded architecture, with the measurement behind it — and any
  axis found inert is retired, re-scoped, or kept with its inertness stated (FR-059).
- **SC-014** — **The craft flies sensibly, and RECOVERY IS THE POLICY'S JOB.** ⭐ The operator's bar is
  *"the craft can recover to patrol"* — but ⛔ under ACRO that is **not a property of the inner loop**
  (§ What ACRO is). Two parts:
  1. **Inner loop**: across the FR-019 attitude sweep the aircraft is *controllable* — a commanded rate
     produces the corresponding rotation, zero command stops rotation — within physical limits, failing
     the way the aircraft fails beyond them.
  2. **Whole system**: the trained policy returns the aircraft to patrol from upsets and unusual
     attitudes. ⚠️ That burden is **unchanged from MANUAL** — ACRO removes the *stabilization* task from
     the policy, not the *recovery* task. Judged behaviourally, as the fidelity bar allows.


---

## Assumptions

1. **The INAV configuration of record is used as-is.** `xiao/inav-hb1.cfg`, `platform_type = AIRPLANE`. Gain
   tuning, rate-profile changes and `servo_pwm_rate` are deliberately excluded so that a single variable —
   the control architecture — moves.
2. **Throttle stays a direct command** (FR-017); only the attitude axes become rate setpoints.
3. **There is no yaw actuator.** Verified in the servo mixer (two elevon outputs, ROLL + PITCH only) and in
   the FDM's servo model (aileron and elevator only). Yaw stays overridden and centred; the commanded zero
   yaw rate is inert. See FR-018.
4. **INAV's command→rate mapping is `rate × 10 °/s` at full deflection**, giving ±360/±120/±40 °/s on this
   config. Verified in the fork's rate conversion; if the plan finds a modifier (expo, TPA, gyro-saturation
   clamp) that changes it, FR-016's single shared definition is where that lands.
5. **The 041-t7 measurements are the baseline** for every comparison, and the pinned S3 run remains
   retrievable until FR-057's extraction is done.
6. **294 scenarios (6 paths × 49 winds) remain the training regiment.** No population increase.
7. **Three build surfaces**: autoc/crrcsim, xiao, and INAV (two targets, bench first, GPS disconnected).
8. **IMU imperfection is modelled small and realistic** — ~±5° alignment, ~5% scale, plus the measured
   accel bias — and **gross misconfiguration is excluded** (FR-052a). The purpose is a controller that
   pursues its goal through its own measurement error, not robustness to a mis-flashed board.
9. **The 2nd flight article is not a blocker.** It gives the first real read on servo-response spread
   across airframes; until it exists the variation range is an assumption and is labelled as one.
10. **Determinism and the eval-vs-training bitwise gate remain non-negotiable** through every change here.
11. **The NN's input and output vectors are UNCHANGED** — 45 inputs, 3 outputs, same magnitude and same
    polarity. Only the outputs' *interpretation* changes. The architecture change lives on the far side of
    the wire, in INAV's rate loop. ⚠️ Two consequences carried as requirements: polarity must be verified
    end to end because a sign error would be invisible in every surviving artifact (FR-016), and the
    craft-balance variations are absorbed by the inner loop's I-term (FR-019, FR-059).
12. **ONE production bake carries all of 043** (operator 2026-08-23) — ACRO, the pinned actuator, the
    mount/IMU craft axes and sampling-time variation land together, matching how 041 shipped its
    format break. ⚠️ **Accepted cost**: if the flight regresses, attribution is a post-hoc ablation
    against the 041-t7 baseline rather than a clean A/B. Mitigated by SC-004's trainability gate firing
    *before* the flight, and by the per-axis reports being comparable across runs.

---

## Dependencies & Risks

| | |
|---|---|
| **Depends on** | 041-t7 as the pinned baseline and its flight data; the INAV fork at `~/inav` (branch `autoc`); the bench and flight INAV targets; a bench servo rig for FR-020. |
| **Blocks** | 044 (M2 tracking) — M2 inherits the control architecture, so it must not start first. |
| ⛔ **Risk — trainability** | Changing the plant can silently destroy trainability (023 Phase 9a). Mitigated by SC-004 as an explicit gate, not an afterthought. |
| ⛔ **Risk — the pinned baseline** | A metadata format change orphans the 041-t7 dmps permanently. Mitigated by FR-057, which must precede the change. |
| ⚠️ **Risk — the actuator term** | 30 ms of the 81.6 ms budget is modelled, not measured; if it is wrong the crossover estimate moves. Mitigated by FR-020. |
| ⚠️ **Risk — silent transport errors** | A corrupt MSP frame is re-read with no counter and no log; it surfaces only as fetch-time creep, and `state_valid` proves zero *losses*, not zero *errors*. ⛔ 043 does not fix this — transport is out of scope and FR-005 was cut 2026-08-25, so the outbound direction remains uninstrumented. Accepted, and it rides with the deferred transport item. |
| ⚠️ **Risk — the sim-model iteration loop, not the bake, is the schedule risk** | *"Decent stab, measure, go from there"* is a loop with an unknown number of turns, and it must **converge before** the single bake is committed (assumption 12). ⚠️ The failure shape is not a bad bake; it is an open-ended modelling phase. Mitigation is SC-002's bar being explicitly structural-and-measured rather than a numeric match — ⛔ if the team finds itself tuning constants to close a gap at the aerodynamic limit, that is the signal to stop and record the divergence instead. |
| ⚠️ **Risk — one run, many changes** | **Accepted** (operator 2026-08-23, assumption 12): ACRO, log growth, the new IMU craft axes, sampling-time variation and housekeeping all land before **one** bake. If something regresses, attribution is post-hoc ablation against 041-t7, not a clean A/B. The cheap partial mitigations — SC-004's trainability gate before the run, and a short ACRO-only smoke before committing 27 h — are plan-phase concerns. |

---

## Research questions for `/speckit.plan`

⭐ Per the operator's scoping, most research belongs to the plan phase. These are the open questions, in
the order they gate work:

- **R1 — What does INAV's fixed-wing rate loop actually do to a setpoint?** ⭐ **Structurally answered
  already** — see **§ What ACRO is**. What remains open: gain semantics, feed-forward,
  I-term handling and limits, TPA, the gyro-saturation clamp, and how the D-term filter shapes the inner
  loop's own phase margin. This is the model's specification.
- **R2 — Are the two loops at arm's length?** FR-030. ⭐ Answered in the sim, after the model exists.
- **R3 — Where does the real airframe sit inside the servo-response spread?** FR-020.
- **R4 — Which 037-era constants does the 041-t7 flight actually contradict?** FR-021.
- **R5 — What is the complete variation inventory, and is the current ramp policy right?** FR-050/FR-051.
- **R6 — Which craft-variation axes are worth having, at what spread, for a rate-commanded plant?**
  FR-052 / FR-056 — including the append-to-craft-vs-new-class call (FR-054), and whether the IMU axes
  want independent toggling. ⭐ Framing is *"craft variations are many and we add a few more"*; resist
  re-inflating it. Also —
  AHRS alignment, control biases, response gains and rates, static margin.
- **R7 — How fast can INAV telemetry actually be collected?** FR-044.
- **R8 — Does the ±360 °/s roll / ±120 °/s pitch asymmetry need anything from the action space** beyond
  documenting it? FR-016. ⚠️ Note the asymmetry is between the two axes that carry **all** of this
  airframe's control authority (FR-018 — no rudder), so it is not softened by a third axis anywhere.
  ⭐ It also interacts with R10: the unreachable upper range *is* the region where buried variation
  reappears.
- **R9 — Reuse `Cntrl_Omega` or write the inner loop fresh?** FR-014a. ⭐ The machinery, the XML config
  path and the correctly-ordered FDM hook all already exist; the open question is whether the four gaps
  (feed-forward, loop rate, anti-windup, expo) are cheaper to close inside it than to build alongside it.
  ⛔ Answer this **early** — it is the largest single swing in the sim work's size. ⭐ The *"decent stab,
  then measure"* bar argues toward **reuse-and-extend**: a from-scratch high-fidelity INAV port buys
  precision the plant model cannot resolve at the limits anyway, while `Cntrl_Omega` already supplies the
  structure — topology, XML config path, and the correctly-ordered FDM hook — that SC-002 actually gates on.
- **R11 — Which INAV phase-delay parameters, if any, are worth changing?** FR-012a. ⭐ The reframing is
  that a filter delay negligible against the 81.6 ms *outer*-loop budget can dominate the *inner* loop's
  phase margin (6.4 ms = 11° at 5 Hz, 115° at 50 Hz). ⚠️ Answer with the model in hand, so the phase
  contribution can be computed rather than guessed — and remember every change must land on both sides.
- **R10 — Which variation axes survive the architecture change?** FR-059 / FR-059a. ⭐ Operator's key
  attribute: the inner loop buries craft balance, so the observability of every axis must be **measured
  under the new architecture**, and the regiment checked for whether it reaches the limits where the
  buried ones reappear. This is the variations work's real question — larger than "is the inventory
  correct". ⚠️ Note the asymmetry is between the two axes that carry **all** of this
  airframe's control authority (FR-018 — no rudder), so it is not softened by a third axis anywhere.
- ~~**R12 — what to do about yaw**~~ — **CLOSED 2026-08-23 before planning**: no rudder, verified in the
  `smix` mixer and the FDM servo model. See FR-018.
