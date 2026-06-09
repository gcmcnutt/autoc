# Feature Specification: Faster Control Loop (10 Hz → 20 Hz)

**Feature Branch**: `037-20hz-control-loop` (to be created when picked up; seeded on the
`035` branch on 2026-06-07 since 035 is unmerged — split to its own branch off `main` when
035 lands)
**Created**: 2026-06-07
**Status**: Draft (seeded from the 035 per-axis bang-bang analysis)
**Input**: Operator direction 2026-06-07 — promote the long-discussed 20 Hz control-loop idea
to a feature, motivated by the 035 finding that roll bang-bang is largely a 10 Hz artifact.

## Context

The 035 per-axis analysis (see `specs/035-energy-lexicase-objective/tuning-backlog.md`) showed
that the controller's roll **bang-bang is not an objective problem** — it's substantially a
consequence of the **10 Hz control loop** (`evalIntervalMsec=100`):

- Per-tick traces (t6 gen 494): only **roll** bang-bangs — 61% saturated, sign-flips on 56% of
  ticks, lag-1 autocorrelation −0.24 (anti-persistent). Pitch (0% saturated, +0.44 autocorr)
  and throttle (pinned-high, +0.80 autocorr) are smooth/coherent.
- At 10 Hz with a saturating roll command and airframe roll inertia/damping (`Cl_p=-0.47`), the
  ±1 dither is partly the controller **synthesizing intermediate average roll authority**
  (PWM-like). The airframe low-passes it to a bounded ~155 deg/s physical rate — the *command*
  looks violent, the *motion* doesn't.
- deg/sec rotation rates are **airframe-limited** (~150–200 roll across every objective regime
  032→035); the objective only trims them ~15–20%. The lever to smooth roll is therefore a
  **faster control loop, not the objective**.

Tracking is already met at 10 Hz (035 t6: 0% crash, 294/294 on challenging courses). **This is a
smoothness / control-headroom upgrade and a manned-feel enabler, not a fix for a failure.**

## Empirical grounding (2026-05-17 flight)

Analyzed `flight-results/flight-20260517/` (model from the xiao log:
`autoc-...2026-05-02T19:20:04.115Z/gen9200.dmp` = pastonly3 / 029-no-future-arch, converged
gen 800). Findings that anchor this feature:

- **Real-hardware actuation is fast; the loop is the bottleneck.** Measured RC command → body-rate
  response cross-correlates at **+0.81/+0.83 @ ~10–12 ms** (roll/pitch); aero/servo +0.74–0.79 @
  10–14 ms. That's only ~10–14% of the 100 ms tick — the airframe reacts in ~12 ms while the
  controller can only correct every 100 ms. So the dominant "delay" is the **10 Hz control rate**,
  not actuation lag. The output path has ample phase margin for a faster loop; the limiter is the
  IMU **input** path (MSP), which is exactly the architecture target below.
- **Flight ran at ~10 Hz** (engage spans 170 evals/18.4 s, 219 evals/23.8 s ≈ 9.2 Hz; ~100 ms
  between log ticks). Confirms current real rate = 10 Hz. (The flight report says "~20 Hz" in
  places, but its own engage table and raw timestamps are 10 Hz — treat as 10 Hz.)
- **Measured MSP pipeline (xiao log, May-17):** `fetch = 6.9/12.6/25.6 ms`,
  `eval = 2.3/2.7/7.4 ms`, `send = 7.0/9.2/12.2 ms`, `total = 16.6/24.4/39.1 ms` (min/avg/max per
  100 ms tick). **The attitude FETCH from INAV over MSP serial (12.6 ms avg / 25.6 ms max) is the
  input bottleneck** — there is currently *no* onboard-IMU read (no LSM6DS3/I2C/TWIM code in the
  tree; all state comes from `MSP2_AUTOC_STATE`).
- **Budget math:** 10 Hz (100 ms) — total 24/39 ms, slack. 20 Hz (50 ms) — feasible, tight at
  worst (~78% of tick). **50 Hz (20 ms) — impossible over MSP:** fetch 12.6 + eval 2.7 + send 9.2
  = 24.4 ms avg > 20 ms. So onboard IMU (12.6 → ~0.3 ms via TWIM EasyDMA + LSM6DS3 FIFO/data-ready
  INT) is **required** for 50 Hz, after which the **9.2 ms MSP command *send*** becomes the next
  limiter.
- **Sim-to-real is confirmed on the bang-bang.** Per-axis sign-flip rate matches across a sim
  controller (035 t6, energy-lexicase) and this real flight (pastonly3, 029) — *different
  objective, arch, generation, and sim-vs-real*, yet: roll **56% (sim) ≈ 57% (real, path 0)**,
  pitch 26% ≈ 26%, throttle pinned both. Pilot qualitative: **violent and mostly on-track, rough
  on the airframe — "pretty much like sim."** Two implications: (1) roll bang-bang is a robust,
  objective-independent property (spiral + 10 Hz + airframe), so this feature's premise holds
  across controllers; (2) the sim faithfully reproduces the real bang-bang, so **20 Hz can be
  validated in sim before flying.**

## Goal

Run the NN control loop at **20 Hz** (50 ms tick) on the real aircraft, and confirm it reduces
roll bang-bang / improves tracking fidelity without regressing the tracking goal.

## Rate selection — a harmonic family, not 10/20/50 (operator, 2026-06-08)

We are **not bound to 10/20/50 Hz** — those are round-number placeholders. The control rate is a free
choice; pick it to **relate by integer ratios to the other system clocks** — camera (480 fps), servo
PWM, motor PWM, IMU ODR, TX. Rationale: we will **not synchronize clocks** across TX / video / IMU /
FC (independent oscillators), so the protection against **low-frequency resonance/beating** is to make
the *nominal* rates harmonically related — then any beat stays high-frequency/benign even under clock
drift; incommensurate rates produce slow beats that alias into control and perception.

Anchored on the **480 fps** camera:
- **Control rate ∈ exact divisors of 480**: 20 / 40 / 48 / 60 / 120 Hz divide cleanly; **50 Hz does
  NOT (480/50 = 9.6 → beat)** — prefer e.g. **48 or 60 Hz over 50**.
- **Misfits to resolve**: analog **servo 50 Hz** (480/50 = 9.6), **motor PWM 400 Hz** (480/400 = 1.2),
  **LSM6DS3 ODR** (104/208/416/833…) don't divide 480 — pick configurable servo/IMU rates that align
  (e.g. servo 60/120; IMU via decimation to a divisor), or characterize/accept the beat where a rate
  is fixed.
- **Phase A sweep should use harmonic rates** (e.g. **20/40/60/120**), not 10/20/50, so the chosen
  control rate already lines up with the camera + actuator + IMU family.
- **Clocks are ±5% (internal RC), unsynced — so harmonic is *nominal-only*; design for tolerance.**
  Every clock (gold-code chip, camera, IMU ODR, servo, TX) runs off its own built-in RC oscillator at
  ~±5%, and we don't sync them — so even nominally-harmonic rates drift relative to each other. The
  harmonic choice **bounds** the beat; the real robustness is **tolerating** the slip: the **gold-code
  correlator self-syncs to the *actual* chip rate** (±5%) instead of assuming nominal — **480 fps
  oversampling gives the phase-lock margin** to do that — and the **control loop is dt-aware**
  (timestamp-driven, the cadence-jitter item) instead of assuming exact dt. Harmonic family +
  self-syncing correlator + dt-aware control = **beat-safe without clock sync**.
- **The two beacons are mutually independent ±5% — decode them independently, not as a harmonic pair
  (operator, 2026-06-08).** The harmonic-family discipline protects against beating *within one
  receiver's clock domain* (control vs camera vs IMU on the same airframe). It does **NOT** make the
  **two target beacons** harmonically related to each other: each beacon has its **own** RC oscillator
  at **±5%**, so beacon-A and beacon-B chip rates can differ by up to ~10% and drift independently. The
  **FPGA decoder must run a separate, independently-self-syncing correlator per beacon** — each locking
  to *its own* actual chip rate/phase — and must **not** assume the two share a clock, a phase, or an
  integer ratio. (This is the real harmonics issue: harmonics buy intra-airframe beat-safety; the
  inter-beacon relationship is unconstrained and must be handled by per-beacon independent decode.) This
  is **input to 031** — see the 031 cross-link below.

So "20/50 Hz" throughout this doc is shorthand — read it as "a harmonic divisor of the camera/clock
family near that rate."

## Required research — clock/frequency-tolerance survey + the 115 kbaud send limiter (operator, 2026-06-08)

The harmonic-rate plan above assumes nominal rates; the achievable rates are set by the **actual
frequency tolerance of each real part** and by the **serial budget for sending servo updates**. Survey
the candidates and reason from measured/datasheet tolerances — **we do not need high absolute accuracy**
(we'll train with jitter and run a dt-aware loop), but we **do** need to know what frame/chirp/servo/
control rates are physically sustainable.

**Frequency-tolerance survey — every clock in the loop (datasheet + bench):**
- **Camera (proposed 480 fps part):** actual frame-rate tolerance/jitter and whether the frame clock is
  free-running RC or crystal-locked. Sets the oversampling margin the gold-code correlator has to play with.
- **Xiao (nRF52840):** the control-tick time-base — is the loop timed off the 32.768 kHz crystal / HFXTAL
  (crystal, tight) or an internal RC (±%)? Determines how tightly the control rate itself holds.
- **Lattice FPGA candidates (the beacon decoder):** each candidate's clock source tolerance (internal
  oscillator ±% vs external crystal) — drives how much per-beacon self-sync the correlator must absorb,
  given the independent ±5% beacons above.
- **INAV I/O:** crude but **not a constraint** — its sampling rate is large relative to our control rate,
  so its timing tolerance is a non-issue for rate selection (still matters for the MSP send budget below).

**The binding limiter is the servo-update *send* rate over 115 kbaud (operator, 2026-06-08).** The real
question isn't compute or IMU read — it's **how often we can actually push a servo/command update down
the link**. Today the command *send* path is **9.2 ms avg / 12.2 ms max** (May-17 MSP), consistent with a
115 200-baud serial link (~11.5 kB/s; an MSP override frame is tens of bytes + protocol/turnaround). That
**~9–12 ms send caps the sustainable command rate well before the 20 ms (50 Hz) tick** — the send alone is
~half a 50 Hz tick. So:
- Quantify the **minimum command frame size** (which MSP override carries the servo command) and the true
  per-frame cost at 115 kbaud → the **max servo-update rate the link sustains**. This, not compute, likely
  sets the practical control-rate ceiling on the current MSP-over-serial architecture.
- Cross-check against the actuation-matched ceiling (~50 Hz servo, harmonic section): if the link can't
  sustain 50 Hz sends, the link — not the servo — is the limit, and a **faster serial rate or a leaner
  command frame** becomes prework for any >20 Hz target.
- Some of this was researched earlier (MSP consolidation, `project_sim_latency` 50→30 ms) — **update those
  numbers against the 115 kbaud send budget and the new rate target**, don't re-derive from scratch.

Output: a table of {part → clock source → tolerance → implication for rate selection}, and a defended
**max sustainable control/send rate** that bounds the Phase-A sweep's upper arm.

## Phasing (A/B/C) — execution order; everything below sorts under a phase

037 spans ~5 subsystems and is too big as one push. Phase it so the **cheap decision comes first**
and the big firmware only follows if justified. Prework, open questions, and carried-in deps each
sort under their phase:

**Phase A — sim rate sweep (cheap, NO firmware/hardware; the go/no-go + rate gate). Runnable right
after M2.**
- Sweep: retrain M1 at **10/20/50 Hz**, measure bang-bang de-alias, **pick the rate** (→ Aliasing
  analysis; Testing §sweep).
- **First concrete step (operator, 2026-06-08): one M1 retrain at an initial rate strictly between 10
  and 50 Hz** — a harmonic divisor of the camera/clock family (e.g. **20, 40, or 48 Hz**, see
  Rate-selection), bounded above by the link-sustainable send rate from the clock/115 kbaud survey.
  Don't open with the full 10/20/50 sweep — prove the lever on one intermediate rate first.
- **Ripple-if-solid:** if that retrain is a **solid improvement** (roll dctrl/sign-flip de-aliases vs
  10 Hz at equal tracking), it cascades two ways — (1) into **M1 flight prep** (the flown M1 in Phase
  B moves to that cadence: COMPUTE_LATENCY model, PID cutoffs, engage-delay, Xiao loop all retarget to
  it), and (2) into the **031 hardware design** (the chosen control rate fixes the camera/chirp/chip
  acquisition budget). If it's *not* a clear win, that's the cheap no-go — stop before any firmware.
- Prework: **entry-envelope sigma tighten (45°/10–15%)** — folds into the retrains.
- Carried-in deps: **COMPUTE_LATENCY model + jitter** (020); **re-derive sim PID filter cutoffs** (026).
- Decide (not yet firmware): **history time-basis** (log-spaced vs more slots) — it's a training-input
  change, settle it in the retrain.
- Sim-fidelity: bump FDM ≥500 Hz for the 50 Hz arm.

**Phase B — embedded 20 Hz (037-proper). Gated on A picking 20 Hz; produces the flown M1.**
- Local-IMU TWIM-DMA read + AHRS fusion + three-tier rates + alignment auto-cal from INAV.
- **Prerequisite: 021 convention cross-check** (execute).
- Prework: **Xiao packed-logging format** (dual-stream high-rate flight log; tail-safe async-DMA).
- INAV: command override + state serve at 20 Hz.
- History-rep firmware implementation (if A changed it) — sim/firmware layout in lockstep.
- Testing: activate→capture→confirm (local-IMU logged beside INAV) → promote → flight test →
  sim↔real confirm = done.

**Phase C — 50 Hz stretch (gated: only if A shows 20 Hz insufficient).**
- NN optimization: **unroll + fast-tanh** (M1 33→32→16→3, M2 54→32→16→3) — target the eval 7.4 ms max.
- **Real-time slot scheduling** (cadence-jitter manage/tolerate) + **interrupt/DMA tail-bounding**
  (BLE quiesce, QSPI async, NVIC priorities).
- Prework: **renderer focus-mode single-arena** (compute reduction matters most here).

The detailed sections below are the reference; this index is the execution order. (Phase A is a
candidate to split into its own small near-term feature.)

## Primary challenge

**IMU attitude latency from INAV over the slow MSP serial link.** Pulling attitude from INAV via
MSP each control tick is the bottleneck for a faster loop. (Real-hardware cousin of the resolved
sim-side `project_sim_latency` 50→30 ms MSP consolidation — here it's the physical serial rate.)

## Candidate architecture (to validate)

**Slave the Xiao to INAV; run the high-rate loop on the Xiao's own IMU.** The Xiao carries an
onboard IMU (LSM6DS3). Extend it from AHRS cross-check (`project_xiao_imu_crosscheck`) to the
**primary high-rate attitude source**:

- Xiao runs the **20 Hz attitude/control tick off its onboard IMU** — lowest-latency signal, no
  MSP serial round-trip per tick.
- Xiao runs a **slower sync loop from INAV** to correct/align the onboard estimate (drift
  correction, absolute datum) at whatever rate the MSP serial sustains.
- Net: high-rate local IMU drives the fast control tick; low-rate INAV sync supplies the absolute
  reference — **decouples control rate from MSP serial rate.**

### Hardware capability — what the nRF52840 + LSM6DS3 can actually do (to be bench-confirmed)

**Latency to raw attitude (vs the 12.6 ms MSP fetch it replaces):**
- I2C burst read (accel+gyro = 12 B) via nRF52840 **TWIM EasyDMA** @ 400 kHz ≈ **0.3–0.5 ms**,
  CPU-free (DMA straight to a RAM buffer). LSM6DS3 also has FIFO + data-ready INT for
  interrupt-driven pulls.
- LSM6DS3 internal digital-filter **group delay** ≈ 0.5–2 ms depending on ODR/bandwidth (the real
  unknown — bench-measure at the chosen ODR).
- → raw attitude available in **~1–2 ms vs 12.6 ms over MSP (~10×).**

**Compute (M4F @ 64 MHz, hardware FPU):**
- AHRS fusion (Madgwick/Mahony complementary): a few hundred FLOPs/update ≈ **10–50 µs** — negligible.
- NN forward pass already measured at `eval = 2.7 ms avg / 7.4 ms max` — comfortably inside a 20 ms
  (50 Hz) tick. Compute is **not** the limiter.

**The real constraint is architectural, not compute or IMU-read latency:**
- The local IMU gives **attitude + angular rate + linear accel only** — NOT position/velocity
  relative to the rabbit. The NN's target geometry (tX/tY/tZ, distance) depends on aircraft
  **position**, which comes from INAV's nav fusion (GPS+baro+IMU) over MSP at the slow rate.
- No magnetometer (6-axis IMU) → local **yaw drifts** → needs the INAV sync loop for heading.
  Roll/pitch from accel+gyro are solid (and roll is the bang-bang axis we care about).
- ⇒ A 50 Hz local loop has **fresh attitude/rates at 50 Hz but stale position/target between INAV
  updates.** Resolution is a **two-rate split**: fast attitude inner loop (local IMU) + slow
  target/position outer loop (INAV), with position dead-reckoned from the local IMU between syncs.
  Ties to the two-loop perception/control architecture (`project_perception_control_two_loop`).
  The bang-bang axis (roll attitude) is exactly the fast-attitude-driven part, so it benefits most.

**Latency analysis to run (the gating work):** bench-measure (a) I2C+filter latency at the target
ODR, (b) fusion update time, (c) the command-*send* path (9.2 ms today — the next limiter once
fetch is local), and (d) the acceptable staleness of position/target at the outer-loop rate.

### Fusion scheme — local IMU + INAV sync (candidates)

Keep the heavy EKF **in INAV**; the xiao runs a **lightweight complementary / loosely-coupled
aiding** filter that treats INAV as truth and uses the local IMU only to fill the fast gaps. This
is classic loosely-coupled INS aiding (INAV = aided solution, slow & authoritative; local IMU =
high-rate inertial propagation), not a fresh full EKF on the xiao.

- **Attitude (the 50 Hz path):** propagate the quaternion at 50 Hz by integrating the **local
  gyro**; level roll/pitch with the **local accel** (Mahony/Madgwick — its integral term doubles
  as a **gyro-bias estimator**). At each slow INAV sync, blend toward INAV's fused quaternion
  (small-gain complementary correction). Roll/pitch are locally observable from gravity, so they
  stay tight between syncs; **yaw is NOT locally observable on a 6-axis IMU**, so absolute heading
  must come from INAV's mag/GPS-aided estimate at the sync — local gyro only carries yaw between
  syncs (drift bounded by sync rate).
- **Latency compensation (the easy-to-get-wrong part):** INAV's synced attitude is *stale* by the
  time it arrives (~12.6 ms fetch + send). Don't blend toward the stale value directly — first
  **forward-propagate INAV's attitude to "now" with the local gyro** (time-align via the MSP
  `timestamp_us` the system already keys on), then correct. Skipping this re-injects the very lag
  we're removing.
- **Position/velocity:** INAV position is the authority; between syncs hold/dead-reckon from INAV
  velocity + local accel (accel position dead-reckon is only good for ~tens of ms — fine at a
  10–20 Hz outer-loop sync, useless longer).
- **Convention coherence is mandatory:** local-IMU and INAV frames must agree
  (`autoc::imu::inavQuatToAerospaceEB` boundary + `project_board_alignment`) — a frame/sign
  mismatch between the two IMUs would be silent and corrupt the blend.

Minimal-viable variant: since **roll attitude is the bang-bang axis and is locally observable**, a
first cut can run just local accel+gyro complementary for roll/pitch + gyro-propagated yaw, INAV
for everything else — capturing most of the 50 Hz benefit on the axis that needs it before
building the full two-rate stack.

## Open questions / dependencies

- **Retrain at 20 Hz cadence.** The current controllers were evolved at the 10 Hz sim cadence;
  the sim eval loop must move to 50 ms so sim and real match (the bang-bang structure itself is
  cadence-dependent). Sim/real cadence parity is a prerequisite for a fair real-flight test.
- **Hypothesis to confirm:** 20 Hz actually reduces roll bang-bang (smoother command for the same
  tracking) — measure dctrl/saturation/autocorr at 20 Hz vs 10 Hz, in sim and in flight.
- **Servo reality:** ±1 roll reversals at 50 ms — confirm servo slew-rate / wear is acceptable
  (real servos have rate limits the sim airframe-inertia hides).
- **Board-alignment coherence** between the Xiao IMU and INAV IMU (`project_board_alignment`).
- **Shared NN log format** at the higher tick rate (`project_log_format_shared_parser`).
- **Phase-delay budget:** *measured* (see Empirical grounding) — actuation responds in ~12 ms,
  slack at 20 Hz (≈4× faster than the 50 ms tick). Grounds the 20 Hz target; becomes binding at
  50 Hz (see stretch).

## Aliasing analysis — should we jump straight to 50 Hz? (sim-sweep first)

Cadences in this build (`autoc_config.xml` + the worker cadence log):
- **FDM physics: dt = 0.005 s = 200 Hz** (Nyquist 100 Hz). High-fidelity; the power/motor model
  sub-integrates finer. (Not 250 — config overrides crrcsim's ~360 Hz default down to 200.)
- Outer cycle 50 ms (20 Hz); **NN control eval 100 ms = 10 Hz → 20 FDM steps per NN command (ZOH)**.

**Where aliasing is — and isn't:**
- **The FDM is NOT the aliasing risk.** At 200 Hz it oversamples the 10 Hz command 20×; the held
  command integrates cleanly and the FDM faithfully represents airframe dynamics to ~100 Hz.
- **The aliasing risk is the CONTROL loop.** The NN samples state at 10 Hz → **Nyquist 5 Hz**. The
  roll bang-bang (sign-flips ~56% of ticks ≈ 2.8–5 Hz) sits **at/near that 5 Hz control Nyquist** —
  the classic signature of an under-sampled *saturating* loop **dithering** because it wants finer
  authority than 100 ms resolution allows. So the bang-bang is plausibly partly a **control-rate
  aliasing/dither artifact**, not purely airframe-optimal.

**Decision: don't jump to 50 Hz — sim-sweep first.** The sim is the right oracle precisely because
its 200 Hz FDM is high-fidelity (it shows what each control rate would actually experience).
- **Experiment (M1 sim, cheap, no hardware): retrain at 10 / 20 / 50 Hz control** — the bang-bang
  is cadence-dependent, so *retrain* at each rate (don't replay the 10 Hz NN), same FDM, and measure
  roll dctrl / sign-flip. Find the knee where the dither de-aliases.
  - Clears at **20 Hz** (Nyquist 10 Hz resolves the ~5 Hz oscillation) → 20 Hz suffices, and we
    avoid the 50 Hz local-IMU/latency cost.
  - Persists at 20 Hz → 50 Hz justified.
- **Sim-fidelity caveat for the 50 Hz arm:** at 50 Hz control the 200 Hz FDM is only **4×
  oversampled** (4 steps/command) — too coarse to trust the result. Raise the FDM rate
  (dt ≤ 0.002 = ≥500 Hz) for the 50 Hz arm to keep ≥10× oversample and avoid control↔FDM
  interaction artifacts. 20 Hz is clean at the current 200 Hz (10× oversample).

This front-loads the 50 Hz question into the M1 sim work we're doing first, and makes it empirical
rather than a guess.

**Note — actuation cadence: the hardware is already fast; 10 Hz NN is the slow element.** From the
2026-05-17 blackbox header: INAV FC inner loop **2 kHz** (`looptime:500`µs), **motor PWM 400 Hz**
(`motor_pwm_rate:400`), servos ~50 Hz (INAV analog default; `servo_pwm_rate` not in this header —
**verify**, may be digital/higher). Our autoc NN override sits on top at just **10 Hz**.
Implications:
- The actuation chain (2 kHz FC / 400 Hz motor / ~50 Hz servo) is **5–200× faster than the NN
  loop**, so the bang-bang is unambiguously a **control-rate (NN-sampling) artifact, not
  actuation-limited** — the hardware can faithfully execute a much faster NN loop. The craft is
  "quite responsive" (measured cmd→gyro ~12 ms is consistent with a fast servo + responsive
  airframe), so actuation is *not* a barrier to 20/50 Hz.
- **~50 Hz servo = the natural actuation-matched ceiling for NN control.** For an analog 50 Hz
  servo there's no point commanding faster than its frame, so the 10/20/50 sweep spans
  current → servo-matched, and **50 Hz is the principled top end** (revisit if servos are digital
  at e.g. 330 Hz). Confirm the actual `servo_pwm_rate` — it sets the true ceiling.

## Stretch: 50 Hz (local-IMU only)

A **50 Hz** loop (20 ms tick) may be feasible **if the loop runs entirely off the Xiao's onboard
IMU** — no MSP round-trip per tick. Flagged for the future, with a hard prerequisite: **a latency
analysis is mandatory before committing.** At 50 Hz the ~12 ms actuation response is ~60% of the
20 ms tick (vs ~24% at 20 Hz), so the combined input+output latency budget goes from slack to
**binding** — latency *will* matter at 50 Hz in a way it doesn't at 20 Hz. The local-IMU
architecture is the enabler for both rates; 20 Hz is the committed target, 50 Hz the stretch
contingent on that analysis.

## Design note — evolve the integrator, don't hand-add it

A manual integral ("I") term was tried earlier and dropped in favor of **first principles: let the
RNN build its own integrator** from hidden state (035 t6 shows live recurrence, `whh_xh_ratio`
~0.48–0.61). 037 follows the same philosophy — the lever is the **loop rate**, not hand-added
control/smoothing terms. Don't reintroduce manual control terms to paper over bang-bang; give the
evolved controller a faster clock and let it use it.

## Required research — signal rate tiers

Operator framing (2026-06-07): signals split into **three rate tiers**, not two. The naive
assumption (attitude fast, position slow) is backwards on the drift physics — single-integrated
attitude (gyro) holds far better than double-integrated position (accel), so attitude needs only a
*slow* absolute resync while position needs *intermediate* bridging.

| tier | signals | source / rate | rationale |
|---|---|---|---|
| **Fast** | linear **accel** + angular **rate** (gyro) | local IMU @ control rate (20–50 Hz) | low-latency primary; "great" off the local IMU; drives the fast control tick |
| **Intermediate (bridge)** | **position**, velocity, airspeed, other nav-derived | INAV poll at an intermediate rate, **dead-reckoned/bridged** between polls | accel→position double-integration drifts in ~tens of ms, so position can't ride the slow tier — needs intermediate INAV updates to stay usable |
| **Slow (resync)** | **attitude** absolute correction (+ heading/yaw) | INAV **slow poll** | local gyro+accel complementary propagates attitude well between corrections; only a slow absolute drift/yaw resync is needed |

**Research questions to answer (bench + sim):**
1. **Attitude resync interval:** how slow can the INAV attitude poll be before the local
   complementary filter's drift (gyro bias × interval, yaw especially) exceeds tracking tolerance?
   Sets the slow-tier rate.
2. **Position bridge rate:** what INAV position-poll rate keeps the dead-reckon bridge accurate
   enough for the rabbit-relative geometry (tX/tY/tZ, distance)? Sets the intermediate-tier rate.
3. **Signal placement:** classify every NN input (airspeed, gyro, quat, pos, vel, target cosines,
   distances) into fast / intermediate / slow — which are local, which bridged, which resynced.
4. **NN consumption model:** the NN currently takes all inputs in one forward pass per tick — does
   it tolerate mixed-freshness inputs (fast attitude + bridged position) directly, or does this
   force the two-loop split (`project_perception_control_two_loop`)?
5. Per-tier MSP poll cost vs the measured 12.6 ms fetch / 9.2 ms send budget (which MSP requests
   carry which tier, and at what rate the serial link sustains them).

**Prior art — feature 021 (`specs/021-xiao-ahrs-crosscheck/`).** 021 *intended* to capture the
IMU-transform → autoc-convention mapping; that transform was since **found independently and lives
in current code** (`autoc::imu::inavQuatToAerospaceEB` + `docs/COORDINATE_CONVENTIONS.md`) — so
**current code is the source of truth for the transform**, not 021's planned work. What 021 left
undone (deferred to 022, never executed) is the *execution*: standing up the Xiao local LSM6DS3 +
onboard AHRS and cross-checking it against INAV. 037 promotes that from cross-check to primary
high-rate source. Related: `project_xiao_imu_crosscheck`, `project_board_alignment`.

### Alignment: auto-calibrate from INAV (primary) — not manual settings

The Xiao's physical **mounting orientation is arbitrary** and must not depend on hand-entered
board-alignment settings. **Primary approach: auto-calibrate the local-IMU → aircraft-body rotation
from INAV** — during a brief calibration window, solve the fixed rotation that best maps the local
LSM6DS3 attitude/gravity onto INAV's authoritative attitude, *regardless of how the Xiao is
mounted*. Manual alignment settings are a fallback/sanity consideration only, not the path. This
supersedes the manual-settings framing of `project_board_alignment` for this feature: the Xiao
should learn its own orientation from INAV, continuously sane-checked by the same slow attitude
resync that anchors the fusion. (Convention coherence — the `inavQuatToAerospaceEB` boundary — is
still mandatory; auto-cal solves the *mounting* rotation on top of the *convention* transform.)

### Future direction — Xiao standalone, past M2

Design the rate-tier sources to be **swappable**, anticipating a post-M2 world where the **Xiao is
the only processor in the loop** (no INAV). There, the tiers re-source: attitude + rates stay
local (IMU + onboard AHRS), and **position comes from a directly-hooked GPS** (position-only — we
don't need INAV's full nav, just the GPS fix to fill the intermediate position-bridge tier). So the
intermediate tier's source is INAV-now / direct-GPS-later, and the slow attitude-resync (INAV
today) would fall away when INAV does — replaced by the local AHRS standing alone with GPS-aided
position. Keep the architecture from hard-wiring INAV as the only possible source for any tier.

## Testing & validation methodology

Same **activate → capture → confirm** loop as the flight-results bracket (and 021's cross-check
intent): bring the local-IMU path up **first as a logged cross-check running alongside the live
INAV path** (don't hand it control yet), fly, capture flight results, and confirm the local fusion
agrees with INAV (attitude/heading/convention) within tolerance — *then* promote it to driving the
fast loop. The 021 convention cross-check is the gate; this is how it gets exercised on real data.
That means logging **both** streams simultaneously (local IMU + INAV) at high rate for comparison —
which drives the prework below.

## Prerequisite prework — Xiao recording format (blackbox-style packed)

**Likely a prerequisite prework task before 037 proper.** Today's xiao log is a verbose
human-readable **text dump** (`#seq millis inav_ms i Nav State: pos=… quat=… gyro=…`, ~328-char
lines, one monolithic line per tick). At 20–50 Hz, with multi-tier signals and **dual-stream
cross-check logging** (local + INAV side-by-side), data volume explodes past what the text dump and
QSPI-flash write budget can sustain. Move to a **blackbox-style differential, packed binary
format** (à la INAV blackbox):

- **Variable frame *type*** — distinct record types per rate tier (fast IMU frame, intermediate
  position/nav frame, slow attitude-resync frame; plus a cross-check frame pairing local vs INAV),
  rather than one fat line carrying everything every tick.
- **Variable frame *rate*** — each frame type emitted at its own tier cadence (fast IMU at the
  control rate, position intermediate, attitude slow), not fixed per-tick.
- **Compact / differential encoding** — delta against the previous frame of that type +
  varint/bitpack, to fit flash and sustain the high write rate; periodic keyframes for resync.
- **Shared decode discipline** — keep one authoritative writer/reader pair so xiao ↔
  renderer/analysis don't drift (`project_log_format_shared_parser`); the join-analysis tooling
  (`join_flight_analysis.py`) must be able to decode the packed format.

Rationale: the whole point of 037's testing form is collecting *a lot more* data (higher rate, more
signals, two IMU streams) — the text dump can't carry it, so the format change gates the validation
flights. Track as prework T-prereq.

## Prerequisite prework — tighten the entry-variation envelope (operator, 2026-06-08)

Before the 037 retraining sweep (10/20/50 Hz), **tighten the entry-variation envelope to max ±45°
entry angle and ±10–15% entry speed** (from today's ±75° / ±25%). Today's caps come from
`EntryConeSigma=30°` and `EntrySpeedSigma=0.10` × the global `kGaussianSigmaClamp=2.5` (truncated
normal) → ±75° cone / ±25% speed. That tail admits **near-unrecoverable 2.5σ corner entries** —
e.g. t6 M1 source scenarios 93 (~75° cone, −52° pitch) and 271 (−25% speed cap, −52° pitch) augered
in within 15–16 ticks, wasting training/tracking budget and producing degenerate source trajectories.

**Mechanism: lower the SIGMAS** (the clamp stays 2.5σ; do NOT add a separate hard cap or change the
global clamp):
- `EntryConeSigma` 30° → **18°** ⇒ 2.5σ = **45°** max entry angle.
- `EntrySpeedSigma` 0.10 → **0.05–0.06** ⇒ 2.5σ = **~12.5–15%** max entry speed (operator target
  10–15%).

Caveat: this changes the M1 difficulty distribution, so M1 results before/after aren't directly
comparable — do it as a clean step, not mid-bake. Why 037: the loop-rate sweep already requires M1
retrains, so fold the envelope tightening into that retraining pass rather than spending a separate
bake on it.

### Rework consideration — slim the autoc training logfile (operator, 2026-06-08)

Alongside the sigma change, **trim verbose per-scenario detail from the autoc training `.log`** — e.g.
the per-arena/per-scenario **best-of-gen `[N] OK/CRASH …` lines** (294 per gen) and per-scenario
decomposition — **ASSUMING that data is all in the dmp** (verify first: confirm the elite per-scenario
results/scores are reconstructable from the gen's `.dmp` via `dmp-dump` before deleting the log
lines). Keeps the per-gen `#NNGen`/`#GenCrash`/`#GenSimStats` summary; drops the bulky per-scenario
dump. This is the same direction as `project_dmp_driven_analytics_backlog` ("move analytics off the
logfile onto the dmp, slim per-gen log output") — fold it in here. (Sim-side autoc log, distinct from
the Xiao packed-log prework below.)

## Prerequisite prework — renderer focus-mode single-arena processing (operator, 2026-06-08)

In **playback focus mode** (the renderer displays only one arena), it currently still **processes
every arena**, even the invisible ones. Change it to **process only the focused arena** and skip
all others entirely (no per-tick update/step compute for non-focused arenas, not just no draw).

Why 037: at 20/50 Hz the per-sample compute budget tightens hard, and wasting cycles stepping
invisible arenas eats into it. Cutting the renderer to the single focused arena reduces compute so
higher-rate playback/sampling stays real-time. (Scope: the visualization/playback renderer path —
confirm whether the saved cycles also help any eval-visual sampling path, or it's purely the
viewer.)

## Prerequisite prework — fix the short-source skip (keep 294 1:1, neutralize in place)

**Bug (found 2026-06-08 in t7):** the short-source skip *erases* the 2 short (corner-crash) source
trajectories from `gSourceTrajectoryList` (autoc.cc, committed `c95887e`). But the eval iterates
**294 scenario slots** (`generationScenarios` = paths×winds, built independently of the source list),
and the worker maps slot→source via **`srcIdx = pathSelector % sourceList.size()`**
([`inputdev_autoc.cpp:700`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L700)).
With a full 294 list that modulo is the identity; trimming to 292 turns it into a **reshuffle** —
slots ≥93 shift onto the wrong source and slots 292/293 wrap onto sources 0/1. So the skip log says
"292 remain" but the eval still runs 294 slots with ~200 misaligned (variation, source) pairings + 2
duplicated sources. **t7 ran on this** — accepted as off-nominal-but-valid for the qualitative M2
verdict, but **NOT clean/reproducible.**

**Fix:** **keep the source list at 294 (1:1 with the scenario slots)** and **neutralize the short
scenarios in place** — score them as the corner-crash they are / exclude their fitness contribution —
instead of erasing. Preserves `slot % 294 = slot`. (Revert the `c95887e` erase approach.)

**Gate:** must land **before any clean / milestone M2 bake** (the 035 M2 rerun and any 037 M2 retrain).
Tracker-mode fix; doesn't affect M1.

## Related rate/latency dependencies (carried-in backlog, 2026-06-08)

Pre-existing items across the repo that the loop-rate change touches — consolidated so 037 is the
single home for the 20/50 Hz thread:
- **COMPUTE_LATENCY calibration + jitter at the new rate** (`020-pre-flight-pipeline/spec.md:12,56`,
  `project_sim_latency`, `project_preflight_checklist`). Sim trains with `COMPUTE_LATENCY` (40→30 ms);
  real is ~10–12 ms (measured). 037 retrains MUST set the latency model to the new loop's measured
  latency and decide jitter (020: "if well under 50 ms on 20 Hz cadence, jitter is unnecessary").
- **Sim PID filter cutoffs are sim-cadence-derived** (`026-nn-temporal-state/plan.md:58-61`):
  `rc_filter_lpf`, `gyro_lpf` (40 Hz = 2× outer frame), `dterm_lpf` (20 Hz = outer frame) come from the
  sim cadence, NOT INAV. Changing the outer/control rate ⇒ re-derive these or the sim inner loop is
  mistuned.
- **Engage-delay at 20 Hz** (`023-ood-and-engage-fixes/contracts/engage_delay.md:80-83`) — flagged
  "10→20 Hz, out of scope for 023"; 037 inherits it.
- **Two-loop different update rates** (perception 30–60 Hz / control 10 Hz,
  `project_perception_control_two_loop`; hardware budget `project_perception_hw_budget`; 30 Hz camera,
  030 plan) — 037's local-IMU-fast + slow-INAV-sync is the same two-rate family.
- **Downstream → the 031 perception chain must correspond** (`docs/aircraft_tracker_handoff.md`). That
  design budgets beacon acquisition at **≤ half the autopilot sample interval**, so a faster control
  rate tightens it directly: 10→20→50 Hz = 100→50→20 ms interval ⇒ **≤50→25→10 ms acquisition**. That
  cascades to **video frame rate** (operator now considering **480 fps as the standard** — high frame
  oversampling per chip is what robust **gold-code detection + sync** needs), **chip rate** + **code
  length** (≤7 chips for high-dynamics), all co-designed against the acquisition budget. Same Nyquist
  discipline as the control-loop aliasing, one layer down — the camera must oversample the chirp to
  decode/sync it. **If 037 picks 20/50 Hz, the 480 fps video rate + chirp/chip rate must correspond**
  — fold the chosen control rate back into `aircraft_tracker_handoff.md` as the acquisition-budget
  input. (031 is post-037 in the queue, but the rate is decided here.)
  - **031 decoder requirement (from 037, 2026-06-08): the two beacons are mutually independent ±5%
    oscillators — decode each with its own self-syncing correlator.** The FPGA must NOT assume the two
    beacons share a clock, phase, or integer ratio; beacon-A and beacon-B chip rates can differ by ~10%
    and drift independently, so each needs an independent acquisition/tracking loop that locks to its
    *own* actual chip rate. The harmonic-family discipline protects the intra-airframe clocks (control/
    camera/IMU), not the inter-beacon relationship. The **480 fps oversampling** is what gives each
    per-beacon correlator the phase-lock margin to self-sync at ±5%. Carry this into
    `aircraft_tracker_handoff.md` alongside the rate-correspondence note (see the harmonic-rate section
    for the full rationale).
  - **The achievable chirp/frame/chip rates are gated by the clock/frequency-tolerance survey + the
    115 kbaud servo-send limiter** (see "Required research — clock/frequency-tolerance survey" above) —
    031's chip-rate/code-length co-design must use 037's surveyed part tolerances and the link-bounded
    control rate as its inputs, not nominal numbers.
- Rate lineage: current = **10 Hz** (oldest tracked, 014/015); operator notes the original was **5 Hz**
  (predates the repo docs). 5 → 10 → 20/50.

## Open design questions — NN history, compute budget, latency (2026-06-08)

The loop-rate change reopens the NN representation and embedded-compute envelope:

1. **History buffers are tick-based — rethink the time basis.** The past-only lookback (029, 6 slots)
   and the beacon span history (032, `beacon_pair_span[6]`) are fixed *N ticks*. At 10 Hz, 6 ticks =
   600 ms; at 50 Hz the same 6 ticks = **120 ms** — the trend window shrinks 5×, and it's *already*
   too short (`project_032_phase1_setup`: 600 ms vs ~9 s lost-sight = 15× short). Options:
   - **(a) more slots** — 50 Hz needs ~30 to hold 600 ms; grows the NN input linearly → compute cost.
   - **(b) log / Fibonacci-spaced lags** (t−1,−2,−3,−5,−8,−13,−21… or powers of two) — recent-dense /
     old-sparse: captures a *much longer* window with *few* slots and is ~rate-invariant in time;
     fixes both the rate-shrink and the 15×-too-short problem (clockwork-RNN / dilated-memory style).
   - **(c) time-based resampling** — fixed-ms lags, decoupled from tick rate entirely.
   - Lean: move history to **time-based / log-spaced** rather than fixed ticks as part of the rate
     change; don't just add slots.
2. **NN compute budget — use the MEASURED Xiao eval, not a theoretical floor.** The May-17 flight log
   measured `eval = 2.3 / **2.7** / **7.4** ms` (min/avg/max) on the Xiao at 10 Hz, where `eval` wraps
   **input-gather (33 inputs) + NN forward + output map** (`xiao/src/msplink.cpp:276–308`). Per-tick
   budget shrinks 100 ms → 50 ms (20 Hz) → 20 ms (50 Hz), so at **50 Hz the measured eval is 13.5% avg
   / 37% max** of the 20 ms tick — NOT negligible, and the **7.4 ms max is a jitter risk** that, with
   AHRS fusion + MSP send, can overrun a 20 ms tick. So the eval IS budget-relevant at 50 Hz (becomes
   the #2 cost once the local IMU removes the 12.6 ms MSP fetch — then eval 2.7 + send 9.2 ≈ 12 ms of
   20 ms, and trimming eval buys real margin). Define a target eval budget on the Xiao.
3. **Optimized "brute-force minimum-latency" NN forward (trade code for latency) — M1 & M2 shapes.**
   Shapes: **M1 = 33→32→16→3 (1923 weights)**, **M2 = 54→32→16→3 (2595 weights)**, recurrent at the
   16-wide layer. A per-topology, fully-unrolled forward (straight-line FMA, compile-time dims,
   **codegen'd C from the fixed shape**, weights contiguous in flash, fp32 to keep sim↔real parity —
   no fixed-point, no alloc/dispatch) trades binary size (fine on the nRF52840's ~1 MB flash) for zero
   loop/branch overhead.
   Theoretical M4F @ 64 MHz floor (single-cycle VFMA, ~2 cyc/MAC): MACs M1 ≈ 60 µs / M2 ≈ 81 µs; tanh
   (48 units) **naive `expf` ≈ +130 µs — the dominant *compute* term, but ~0.13 ms, NOT the ms-scale
   7.4 ms tail (that's flash/preemption, item 4)** vs poly/LUT ≈ +20 µs → optimized forward ≈ **0.1 ms**.
   **The gap is the lever:** measured eval **2.7 ms avg / 7.4 ms max** vs ~0.1 ms forward floor = ~10–25×,
   spread across input-gather (sensor math: dir-cosines/dist/quat), the looped forward, and `expf`
   tanh. So at 50 Hz the optimization is **worthwhile** (reverses an earlier wrong call that assumed
   the 0.1 ms floor was the live number): unroll + **fast-tanh** + a tightened input-gather target the
   2.7 ms→~0.3 ms avg. (The **7.4 ms max tail is preemption/flash — item 4's job, not compute.**) Note
   `eval` = gather + forward, so optimizing spans BOTH — unroll/fast-tanh for the forward, and a look
   at the sensor-math gather. Keep fp32 for parity. (Bench-measure the split first to target effort.)
4. **Interrupt / DMA impact on long-tail latency (the 7.4 ms max is almost certainly preemption, not
   compute).** Avg eval 2.7 ms vs max 7.4 ms = a ~5 ms jitter tail — on the nRF52840 that's
   preemption/stalls, and at 50 Hz the worst case matters more than the average (a 20 ms tick can't
   absorb a 7.4 ms+ stall plus fusion + send). Concrete suspects found in the current firmware:
   - **QSPI flash logging — the PRIME in-flight tail suspect.** The 7.4 ms max was measured *armed*
     (BLE off — see below), so it isn't BLE; flash is the ms-scale culprit. The logger is **blocking
     with erase** in the path (`qspiEraseBlocking`, `qspiWriteBlocking`, `saveMetadataBlocking`,
     `META_FLUSH_ISSUE/WAIT_ERASE`, `irq_priority=6` in `flash_logger.cpp`); flash erase is
     ms–100s-of-ms, so any in-loop erase/blocking write IS a multi-ms tail. **Folds into the
     packed-logging prework: fully async-DMA, a pre-erased ring (no in-loop erase), double-buffered** —
     worse at higher rate × dual-stream volume if not fixed.
   - **BLE / SoftDevice — ruled out in flight (corrects an earlier call).** `ArduinoBLE` is active on
     the bench, but it's **disabled on arm** (`bluetooth.cpp:95-101`: `BLE.stopAdvertise()` + central
     disconnect, "BLE advertising disabled"), so it does NOT preempt the armed critical path. Only a
     pre-arm/bench concern.
   - **`noInterrupts()` critical sections** (`msplink.cpp:628-634`, command-cache update) — keep
     minimal.
   - **New TWIM/SPIM DMA for the local IMU (037)** — design tail-safe from the start: DMA + completion
     ISR (not blocking poll), double-buffered, sane NVIC priority.
   Action: **instrument sub-phase timestamps** to *attribute* the tail (forward vs gather vs
   flash-flush; BLE is out when armed) — `eval` is one number today; bound the worst case, then set NVIC
   priorities so the control tick isn't preempted by non-critical ISRs. DMA helps by overlapping I/O
   with compute, but completion ISRs + bus contention still add tail — design for it.
5. **Cadence jitter: manage vs tolerate — real-time slot scheduling.** As the tick shrinks (20→50 Hz)
   the loop wants a deterministic structure: time-budgeted **real-time slots** per phase —
   **collect** (IMU read + sync/fetch) → **process** (fusion + NN forward) → **output** (command send)
   → **log** (packed write) — with the control-critical slots (collect/process/output) preempt-
   protected and **log + non-critical work in the slack slot** (deferred / async-DMA; under overrun,
   **drop or coalesce the log rather than stall control** = graceful degradation). Two strategies,
   likely combined:
   - **Manage jitter** — size slots to worst-case, NVIC-prioritize the control path above non-critical
     ISRs, async/defer logging (ties to item 4 + the packed-log prework). Lower jitter, harder real-time.
   - **Tolerate jitter** — make the controller **dt-aware**: timestamp every sample and process with
     the *actual* dt (so history / span-rate / derivatives stay correct under variable cadence), and
     **train with cadence jitter in the sim** (the 020 `COMPUTE_LATENCY`-jitter item) so the NN is
     robust to variable timing. Removes the need for hard real-time.
   - Lean: **bound the control-critical path AND train dt-tolerant**, so residual jitter is harmless
     and logging degrades gracefully — don't chase hard-real-time perfection on a BLE-sharing MCU. The
     `inavSampleTimeMsec` timestamp already keyed through the system (see flight-join) is the hook for
     dt-aware processing.

## Out of scope (v1)

- Objective/fitness changes (035 territory — the loop rate is the lever here, not the objective).
- Airframe roll-authority changes (the X-wing / different-airframe backlog).
- 50 Hz (stretch above — gated on the latency analysis).
