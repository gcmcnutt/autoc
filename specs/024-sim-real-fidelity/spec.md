# 024 — Sim/Real Fidelity & Cadence

**Status**: Scaffold
**Depends on**: 023 (direction cosines, engage delay, deterministic eval)
**Priority**: P0 — gates next flight; absorbs leftover 023 tech debt
**Related**: 025 (craft variations — formerly 024)

## Problem Statement

flight-20260417 diverged "wildly with some intention." The 023 postflight audit
ruled out the obvious convention bugs (quat ordering, NN output polarity on
pitch/roll, renderer perception, cmd→gyro timing on pitch/roll). Two real findings
remain unexplained by any convention bug, plus a pile of related tech debt that's
been accumulating:

1. **Training cadence drift.** Sim `data.dat` steps 117 ms/sample instead of the
   intended 100 ms. CRRCSim physics ticks at ~39 ms and the eval threshold
   `simTimeMsec > lastUpdateTimeMsec + 100` (strict greater) fires at the next
   physics tick past 100 ms (3 × 39 = 117). NN was trained at ~8.5 Hz while xiao
   runs at ~10 Hz, heading to 20 Hz. The sim and deployment cadences do not match.

2. **Flight AHRS quat disagrees with flight gyro on roll axis.** Post-flight
   analysis: flight cmd→gyro roll correlation is +0.76 at ~200 ms lag, but
   cmd→quat-derived-roll-rate is **negative at every lag 0–707 ms** (best −0.14).
   Sim shows +0.54 for the equivalent signal. Negating the flight quat-derived
   signal only gets to +0.14–0.27, much weaker than sim. The AHRS attitude
   tracking on the roll axis is materially different from what sim produces.

3. **Bench verification gap.** `docs/COORDINATE_CONVENTIONS.md` bench tests use
   only single-axis rotations. A compound-attitude test would distinguish several
   quat-convention bugs that isolated-axis tests cannot.

4. **Broader fidelity sweep pending.** Only body angular rates and attitude have
   been subjected to end-to-end cmd→response audit. Position, velocity, and
   acceleration pipelines between sim and flight have not been audited the same way.

5. **Accumulated tech debt around the NN input/output contract.** The 023
   topology change silently broke several post-flight scripts and the renderer
   (reconstructed rabbit from wrong indices) because the log format relies on
   index-based parsing without a schema version. Several scripts still carry
   old-layout assumptions. Renderer has a legacy INAV-blackbox direct path that
   is separate from the primary xiao `-x` flow.

6. **Workarounds and fallbacks rarely get revisited.** The `SIM_FPS = 25.0`
   with a comment claiming "~40 Hz" is a good example — a constant that doesn't
   match reality that's been sitting there. Similar latent items likely exist.

## Goals

1. **Cadence correctness**: sim eval fires at exactly 100 ms (10 Hz).
   Training-time and deploy-time cadence match exactly. 20 Hz (50 ms)
   infrastructure is a future-facing concern — keep in mind while designing
   the cadence fix so the step-up is low-friction later, but **do not** cut
   over to 20 Hz in this feature.
2. **Ground-truth sensor fidelity**: every signal the NN sees or acts on —
   position, velocity, acceleration, quaternion, angular rate, **and throttle
   response (airspeed, RPM)** — is traced to a ground-truth source and
   validated end-to-end, sim↔flight. The theme is "we stop guessing; every
   signal has a reference we can compare against."
3. **Quat sign-convention audit closed**: flight AHRS roll anomaly either
   root-caused or characterized and accepted.
4. **Tooling debt paid**: schema-versioned logs, post-flight scripts rewritten
   around literal NN I/O, renderer legacy path resolved, training run archive
   policy in place.
5. **Workarounds cleared**: known workarounds replaced with proper implementations
   where possible; documented where not.

## Work Items

### WI1 — Fix sim eval cadence to 100 ms exactly

**Root cause**: `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp:341`
`shouldEval = (simTimeMsec > lastUpdateTimeMsec + gEvalUpdateIntervalMsec)` with
physics ticking at ~39 ms lands on 117 ms. Header claims `SIM_FPS = 25.0`
("~40 Hz physics tick assumption") but actual step disagrees — first pass is to
understand why.

**Options**:
1. Change `>` to `>=` and align physics tick to divide 100 ms evenly.
2. Accumulate partial-tick virtual-time budget so long-run rate averages exactly
   100 ms.
3. Override physics rate in training so 100 ms lands on a tick.

**Verify**: `awk 'NR>1 {print $4}' data.dat | uniq -c` shows every Time delta
exactly 100 ms.

**Also document**: actual physics step path in a new `docs/CADENCE.md` (or append
to `COORDINATE_CONVENTIONS.md`). Resolve the `SIM_FPS` comment vs value disagreement.

### WI2 — Keep 20 Hz in mind (design, do not implement)

20 Hz is the next target after 024 ships. In WI1, design so the step-up is
cheap later — don't bake in assumptions that 100 ms is forever:

- `EVAL_UPDATE_INTERVAL_MSEC_DEFAULT` should become a real config knob, not
  a compile-time constant.
- NN temporal history offsets (`HIST_PAST`, `FORECAST_OFFSETS`) are in *ticks*.
  At tick=100ms vs 50ms, their wall-clock semantics differ. Decide now whether
  they should be expressed in ms (so they're tick-rate-independent) or left
  in ticks (NN window scales with rate). This decision affects WI1's API shape.
- Note in the code where 20 Hz will require additional care (MSP poll rate,
  INAV RC override cadence, training wall-time budget).

No training-run or xiao rebuild at 20 Hz in this feature — just the paper
design and the API shape that makes it possible.

### WI3 — Quat sign convention: compound-attitude bench verification

Existing table only tests isolated-axis rotations. Add compound attitudes; record
INAV raw, xiao post-conjugate, xiao log `q=` field, Eigen Euler, and our quat→body-axis
formula output for each:

| # | Physical hold | Expected Euler (φ, θ, ψ) |
|---|---|---|
| B1 | Level, nose north, canopy up | (0°, 0°, 0°) |
| B2 | Nose up 30°, right wing down 30°, heading 0° | (+30°, +30°, 0°) |
| B3 | Nose up 45°, heading east (yaw 90°) | (0°, +45°, +90°) |
| B4 | Right wing down 60°, heading south 180° | (+60°, 0°, +180°) |
| B5 | Nose down 30°, left wing down 45°, heading west | (−45°, −30°, −90°) |

Any row where extracted Euler ≠ expected within 5° = documented sign-convention
drift. Results append to `docs/COORDINATE_CONVENTIONS.md` as pass/fail matrix.

### WI4 — Full-sensor ground-truth audit on new flight

A flight test designed for this audit (not aerobatic, not just "fly a path").
Every NN-visible signal is reduced to ground truth and compared sim↔flight:

- **Position / velocity**: log INAV navPos, navVel over an engage span of a
  straight climb and a controlled circle. Compare against sim eval of the same
  path. World-frame trajectory should match within sim-to-real dynamics tolerance.
  Ground truth for position is GPS-derived (INAV nav), for velocity navVel.
- **Acceleration**: log `accSmooth[0..2]` during engage. Body-frame accel
  signature (centripetal in turn, vertical in climb) must match expected signs
  and magnitude. Ground truth is gravity (at rest / level) and derivable from
  velocity curvature during known maneuvers.
- **Quat continuity**: log quat every sample over a full engage span, overlay
  against gyro-integrated quat seeded from the span's initial quat. Divergence =
  AHRS filter contribution. Direct test for WI5. Ground truth is the
  gyro-integrated attitude trajectory.
- **Throttle → airspeed / RPM ground truth**: **deliberately command varying
  throttle during engage** (or allow pilot throttle override under autoc
  roll/pitch) so throttle→airspeed polarity and gain can be measured directly.
  The 20260417 flight pegged throttle near +1 throughout, leaving polarity
  unverified. Additionally log `escRPM` from blackbox so we have a second
  reference (RPM is closer ground truth for "motor is responding to command"
  than airspeed, which integrates airframe + prop + gravity). Confirm:
  - throttle cmd → ESC RPM correlation clean and positive at short lag (~50–100 ms).
  - ESC RPM → airspeed correlation clean and positive at longer lag (~500–1000 ms).
- **Angular rate**: gyrP/Q/R ground truth is the differentiated attitude from
  quat (or vice versa — either signal can sanity-check the other). Covered by
  WI5 investigation.

### WI5 — Flight AHRS/gyro axis convention: root-cause

**Updated finding (2026-04-18)**: The simple lag hypothesis doesn't hold after
the quat-delta-based rate extraction. Using `q_delta = q_prev.inverse() * q_curr,
(p,q,r) = 2 * q_delta.vec / dt` — the exact kinematics relation with no Euler
coupling — sim shows slope = 1.0 on all three axes (identity as expected), but
flight shows:

- **Yaw**: slope ≈ +0.93, correctly signed. Quat and gyro agree on yaw.
- **Roll**: slope ≈ +0.17, weakly positive (attenuated / noisy but not inverted).
- **Pitch**: slope ≈ **−0.22**, **sign-inverted**.

Only the pitch axis inverts. A global convention bug would flip all three.

**Working hypothesis**: msplink's `neuQuaternionToNed` applies a full conjugate
`(q[0], -q[1], -q[2], -q[3])` to convert INAV's body-to-earth to earth-to-body.
But INAV's *gyro* pitch (`gyroADC[1]`) is non-standard aerospace RHR — it's
"nose-down = positive" — so msplink separately negates `gyroADC[1]` to flip
it into aerospace RHR. If INAV's *quaternion* encodes pitch with the same
"nose-down = positive" convention as their gyro, then the conjugate alone
produces a quat where qy still represents "nose-down-positive rotation" — NOT
aerospace RHR. Our quat-delta rate extraction then gives `-q` (aerospace pitch
rate) while the gyro (already negated) gives `+q`. Exactly the observed inversion.

Yaw is unaffected because — assumption to verify — INAV's yaw encoding in the
quat happens to align with aerospace after conjugate (or the effect is small
on yaw-axis rotations because they're about world-down, a shared reference).

Roll is attenuated but not cleanly inverted. Possibly:
- Similar convention mismatch but partially cancelled by bank-angle coupling.
- Dominant noise from near-saturated roll commands with rate-limit clipping.

**Actions (folded into WI3 bench + here)**:
1. **WI3 compound-attitude bench** answers this definitively. At e.g.
   +30°-pitch +30°-roll, record INAV raw quat, msplink post-conjugate quat,
   and compare each component sign against standard aerospace q_EB computed
   from the known Euler. Any component that disagrees is documented as an
   INAV-specific flip requiring correction in msplink.
2. Depending on bench result, fix msplink `neuQuaternionToNed` to produce a
   true aerospace-RHR q_EB (not just conjugate). May need per-component sign
   handling matching the gyro-axis negations.
3. Once fixed, rerun `gyro_vs_quat.py` on a re-flown span — expect slopes
   near 1.0 on all three axes, matching sim.
4. If bench disproves the convention hypothesis, pivot to AHRS filter
   investigation (`inav_w_acc`, `inav_w_mag`, `imu_acc_ignore_rate`, centrifugal
   compensation, mag alignment in `xiao/inav-hb1.cfg`). Integrate flight gyro
   over each engage span, compare trajectory to AHRS quat.

**Diagnostic artifacts** (kept for reference, rerun after fix):
- `flight-results/flight-20260417/gyro_vs_quat.py` / `.png` — flight scatter
- `specs/024-sim-real-fidelity/gyro_vs_quat_sim.py` / `.png` — sim baseline

### WI6 — Xiao rabbit logging

`rabbit=[x,y,z]` field in NN log lines is the NN's direct target (virtual NED).
Verify it's consistently present in all engage spans and is the true
rabbit position from the NN's own state, not a reconstruction. Fix if missing.
Needed by renderer `-x` mode magenta ground-truth reference line.

### WI7 — NN input layout drift guard

Commit 1189782 (023) changed NN input layout; renderer and scripts silently
continued compiling because they parse the text log by regex, not through the
`NNInputs` type. Add:

- Schema version constant next to `NNInputs` struct (bumped on any field change).
- Version emitted in data.dat header and xiao log banner.
- Renderer + Python readers verify version and bail loud on mismatch.

Stronger alternative: replace index-based data.dat parsing with named-field
parsing (already done in xiao log: `tX=`, `g=`, `out=`). Apply same discipline
to data.dat header format so silent drift is structurally impossible.

### WI8 — Post-flight analysis scripts: rewrite around literal NN I/O

Several scripts rotted across the 023 layout change:
- `specs/018-flight-analysis/correlate_flight.py` — missing Z-flip and quat conjugate.
- `specs/019-improved-crrcsim/scripts/verify_flight_log.py` — reads old
  dPhi/dTheta indices.
- `specs/022-tracking-cone-fitness/flight_nn_polar_viz.py` — plots direction
  cosines as bearing angles.

Rewrite under a single principle: **the NN input/output in the log is canonical
truth.** Consume literal named fields; share a helper module with canonical
column/index constants; keep sim and flight analyses structurally comparable.
Folds with WI7 version check.

### WI9 — Renderer legacy INAV blackbox path: decide

`tools/renderer.cc:1720–1735` loads INAV blackbox CSV directly (pre-xiao flow).
Complies with conventions today but parallel to the primary xiao-log `-x` path.
Decide and execute: **keep-and-maintain** (add version check per WI7, keep as
fallback when xiao log unavailable) or **delete**. Removing it reduces surface
area; keeping it gives a conventions cross-check against an independent source.
Low priority but close it out either way.

### WI10 — Training run archive policy

We have `test4-data.dat` (6.3 GB) anchoring current analysis. Define policy for
which training runs to preserve long-term vs discard: a short note in
`docs/` or `specs/` covering naming, retention, and the current canonical run.

### WI11 — Workarounds & fallbacks audit

A cross-cutting sweep for "workaround shaped" code we've been working around
rather than fixing. Known or suspected items to check:

- `SIM_FPS = 25.0` with comment "~40 Hz physics tick assumption" — comment and
  value disagree; which is correct? (Connected to WI1.)
- `engageDelayMsec = 750` with stick-centered + cruise-throttle hold —
  is the delay masking a real startup instability in the NN or the
  AHRS-just-after-engage? Revisit once WI3–WI5 give cleaner attitude tracking.
- `XIAO_RABBIT_SPEED_MPS = 12.0f` as a xiao-side constant separate from
  sim's `autoc.ini RabbitSpeedNominal` — two sources of truth. Consolidate.
- `inav-hb1.cfg` board alignment quirks on bench hardware — document current
  values and why; decide whether they're still needed.
- `MSP_override_channels = 47` (bits 0,1,2,5) forcing CH6 = MANUAL — working
  as intended, but verify in the INAV CLI of the flight FC (came up as "top
  suspect" in the 023 output-side audit).

For each: is this a real workaround that should be removed, or a legitimate
boundary decision that just *looks* like a workaround? Document either way.

### WI12 — Test coverage (carried from 023 P3)

From `specs/023-ood-and-engage-fixes/tasks.md` P3:

- `tests/engage_reset_tests.cc` — 6 contract tests for `resetHistory()`.
- `tests/engage_delay_tests.cc` — 3 contract tests for delay window.
- `tests/nn_inputs_tests.cc` — unit-vector invariant, poison-value completeness,
  `sizeof(NNInputs) == 33 * sizeof(float)` contract (already `static_assert`ed,
  but test makes behavior visible in test output).

Selection-tests / 2-dim lexicase moved to 025 with effort-lexicase.

### WI13 — Minisim quat convention — fix in-scope

Previously backlogged. Fits 024's fidelity theme squarely: minisim currently
treats `AircraftState::aircraft_orientation` as body→world while every other
code path treats it as earth→body, and `tools/minisim.cc:148` plus
`aircraft_state.h:367,389` carry that assumption. Not flight-relevant today
(minisim is only plumbing-tests) but it's a latent footgun that will poison
any future reuse of `minisimAdvanceState` for training-adjacent work.

Full description of the bug, proposed fix, and acceptance test are in the
`[NEXT]` entry that was in BACKLOG — pulled into 024 in this feature. See
commit history for the original backlog prose.

Acceptance:
- `AircraftState::aircraft_orientation` has exactly one semantic (q_EB),
  documented at the field declaration.
- Minisim composition and velocity rotation use q_EB semantics.
- Locking unit test constructs an AircraftState at known non-identity
  attitude, asserts body-frame ↔ world-frame transformations round-trip
  correctly.

## Validation

1. **Cadence**: data.dat Time deltas all exactly 100 ms (then 50 ms after WI2).
   Flight xiao_ms deltas all 100 ms ± 5 ms (then 50 ms).
2. **Quat bench (WI3)**: all 5 compound attitudes Euler-extract within 5°, in
   both the sim init path and after xiao msplink conjugate.
3. **Flight cmd→response audit (WI4)**:
   - gyro-based cmd→rate correlation ≥ +0.5 at lag 100–300 ms on all 3 axes.
   - quat-derived cmd→rate within ±0.2 of gyro-based (no sign flip).
   - Position trajectory matches sim within 5 m RMS over 5 s span.
   - Throttle→airspeed clearly positive at ~700 ms lag.
4. **AHRS divergence (WI5)**: quantified divergence angle; either root-caused
   and fixed, or documented as known-acceptable bias below threshold.
5. **Tooling**: WI7 version guard trips on a deliberate layout-change test.
   WI8 rewritten scripts produce correct output on both 20260417 flight log
   and sim data.dat.
6. **Workarounds (WI11)**: for each listed item, disposition is one of
   {removed, replaced, documented as legitimate}. No "ambiguous, revisit later."

## Out of Scope

- Craft parameter variations, trim offsets, authority limit — 025.
- NN topology changes.
- Hardware (airframe, servo, prop) changes.
- Effort lexicase / smoothness-pressure training — 025 Change 5.

## Open Questions

1. Does AHRS roll divergence come from centripetal accel contamination or from
   a convention mismatch? WI3 + WI5 together answer.
2. Is the training-time / flight-time cadence mismatch actually affecting NN
   behavior meaningfully, or is 117 vs 100 ms within the noise? Compare NN
   control patterns before vs after WI1.
3. At 20 Hz, do history window tick offsets (−9, −3, −1, 0, +1, +5) still
   make sense or should they be rescaled to preserve wall-clock semantics?
4. If AHRS is noisy, should NN consume gyro-integrated attitude instead? Pro:
   cleaner. Con: drift over long engage.
5. For WI11 engage delay: does the 750 ms window mask a real NN startup
   instability, or is it just engage-transient smoothing?
