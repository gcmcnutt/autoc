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
- **Phase-delay budget:** quantify real-hardware command→response lag at 10 Hz first (see the
  most-recent flight analysis) so the 20 Hz target is grounded in the measured delay.

## Out of scope (v1)

- Objective/fitness changes (035 territory — the loop rate is the lever here, not the objective).
- Airframe roll-authority changes (the X-wing / different-airframe backlog).
