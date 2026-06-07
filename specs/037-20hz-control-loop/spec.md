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

## Out of scope (v1)

- Objective/fitness changes (035 territory — the loop rate is the lever here, not the objective).
- Airframe roll-authority changes (the X-wing / different-airframe backlog).
- 50 Hz (stretch above — gated on the latency analysis).
