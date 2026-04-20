# 024 — Sim/Real Fidelity

**Status**: Scaffold
**Depends on**: 023 (direction cosines, engage delay, deterministic eval)
**Priority**: P0 — gates next flight with new training
**Related**: 025 (craft variations — formerly 024)

## North Star

**Close-out criterion: research → corrections → retrain → eval → flight
test showing sim-parity dynamics.** The feature closes when a flight
test of the retrained (unchanged-topology) NN, on a cadence-corrected and
sensor-verified pipeline, shows the same dynamic control response as sim
— same cmd→rate slope, same cmd→attitude-delta behavior, same trajectory
family. The corrections we expect to land (rabbit position logging,
sensor-axis fixes, sampling-rate alignment, history buffer verification)
are precisely the kind that a flight test is the only definitive proof
of; we keep them bundled so the feature's outcome is a flown, working
system, not a staged hand-off.

**Deployment rule**: **no changes to flight hardware until the retrained NN
is ready** (US6). Msplink convention fixes, cadence corrections, and all
other pipeline changes will invalidate the current NN weights — the
current weights were trained against pre-fix conventions, so flashing the
fixed xiao code with old weights would make flight behavior *worse*, not
better. Xiao rebuilds during US3/US5 are for bench verification only. The
single deployment event is US6: new weights + fixed pipeline flashed
together, then flown.

### Critical milestones

The work items collapse into **two ordered milestones** that together close
the feature. Everything else is either (a) prerequisite diagnostic work
feeding these milestones, or (b) nice-to-have cleanup that can defer.

- **M1 — CRRCSim is right against our aeronautical conventions**
  (reference correctness). The training simulator must match the standard
  aerospace conventions in `docs/COORDINATE_CONVENTIONS.md` before we can
  trust anything downstream — CRRCSim is NOT assumed correct; it's tested
  against first-principles aero conventions just like every other component.
  Acceptance: sim `data.dat` passes all 8 cross-checks at exactly 100 ms
  cadence; quaternion, gyro, position, velocity, attitude, accel (N/A if
  not logged), heading, and command→rate correlations all sign-correct.
  Covered by: **US2 + US4 + any CRRCSim-side fixes in US3.**
- **M2 — Xiao IO matches CRRCSim** (deployment parity). Every signal the
  NN sees and every command it emits must use the same convention on the
  xiao deployment target as on the CRRCSim training target. Acceptance:
  US1 cross-checks on flight-20260417 historical data pass after analysis-
  library fix; compound-attitude bench on flight FC reproduces the
  canonical signals expected by the (reference) sim for each pose.
  Covered by: **US1 + US3 msplink fixes + US5 bench.**

**Minisim q_EB fix (WI7) is P2**, nice-to-have for convention consistency
across our stack. It does not gate either M1 or M2 — minisim is not in
training or the xiao pipeline. Fold into the feature if time permits
(tasks.md subgroup 5.b); otherwise leave the BACKLOG entry and address
when minisim is actually reused.

The primary focus is going deep on sensor consistency: sim-to-real. Dynamic
control response in flight should match sim at the single-sample level
(slope ≈ 1 on cmd→rate, quat-delta ≈ gyro, position integrates cleanly from
velocity, attitude matches accel/gravity). We've been circling around
"something is off" — the fix is to stop guessing and methodically confirm
every NN-visible sensor signal is self-consistent with every other one,
conforms to [COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md),
and matches between sim and real at the interfaces we control.

Non-linear effects, CG differences, airframe wear, bang-bang control, and
craft variations are NOT gated here — those are 025.

## Clarifications

### Session 2026-04-19

- Q: Does 024 scope include a fresh NN training run, or does training wait until a later feature? → A: Full sequence is in-scope — research, corrections, retrain, eval, **and flight test**. NN topology stays the same for the next flight. The fixes expected from this work (rabbit position logging, sensor tweaks, sampling-rate alignment) mean the *outcome* of 024 is a new flight, and that flight is the closing proof point for the feature. (Revised from initial answer of "flight is separate" — clarified 2026-04-19: the corrections we expect to land are exactly the kind that require an end-of-feature flight to validate, so we keep them together.)
- Q: New pilot-only flight, or work with flight-20260417 data first? → A: Use 20260417 first; supplementary pilot-only flight only if WI1 inconclusive. Premise: we trust the INAV → blackbox → CSV pipeline end-to-end once plan-phase research verifies each column, so pilot-flight portions of 20260417 are as valid as xiao engage spans for convention cross-checks.
- Q: Pass/fail threshold policy for WI1 cross-checks? → A: Directional only — slopes must be positively signed (no sign inversions); correlation strength is reported and reviewed, not gated by a numeric threshold. Rationale: sim shows strong correlation despite cross-coupling, which demonstrates that correct conventions produce strong correlation; flight with correct conventions should too. We inspect correlation values to catch weak/drifting signals, but the hard pass/fail is "no axis is sign-inverted."
- Q: History buffer semantics — does a stored direction-cosine entry represent the direction "at sim tick now" or "at sim time of recording"? → A: Current sim tick. `recordErrorHistory(dir, dist, timeMs)` stores `dir` as computed at the caller's "now"; history index `[t]` = direction observed at sim tick t. Lock with a contract test in `tests/engage_reset_tests.cc` (WI15).
- Q: WI4 cadence fix approach — options A (tick alignment), B (partial-tick budget), C (50 Hz physics override), D (combo)? → A: Defer to plan-phase research. Research targets: (a) CRRCSim's inner loop / frame update rate in **headless mode** — what drives the tick, what knobs exist; (b) Xiao's timer — is its 100 ms sample interval on a sustained hardware timer (no drift) or software-accumulated (subject to drift from MSP response time)? Goal: hit exactly 10 Hz now with a path to 20 Hz later, on both sides, with the cleanest + lowest-risk mechanism.

## Problem Statement

flight-20260417 showed "wildly wrong with some intention." 023 postmortem
ruled out obvious convention bugs (quat scalar ordering, NN output polarity
on pitch/roll, renderer perception, cmd→gyro timing on pitch/roll). Two
findings survive and drive 024:

1. **Flight gyro vs quat-delta rate scatter** (`gyro_vs_quat.py` / `.png` in
   `flight-results/flight-20260417/` and `specs/024-sim-real-fidelity/`):
   sim is identity (slope ~1 on all three axes — expected since CRRCSim's
   gyro is integrated from the quat). Flight yaw agrees (slope +0.93,
   correctly signed), flight roll is attenuated/noisy (+0.17), flight pitch
   is **sign-inverted (−0.22)**. Only pitch flips — a global convention bug
   would affect all three. Working hypothesis: msplink's quat conjugate
   doesn't correct INAV's non-aerospace pitch sign the way the explicit
   `-gyroADC[1]` correction does for the gyro.
2. **Sim cadence drift**: training data.dat steps 117 ms/sample instead of
   100 ms. CRRCSim physics tick (~39 ms) + strict-greater eval threshold
   in `inputdev_autoc.cpp:341` lands at 3 ticks = 117 ms. NN trained at
   ~8.5 Hz while xiao runs ~10 Hz, heading to 20 Hz.

These findings are tips of a pile — likely other coordinate and pipeline
drift issues we haven't surfaced yet because the xiao engage spans during
flight-20260417 are short and bang-bang, leaving limited dynamic range to
expose bugs. The first work item exploits the **manual (pilot) flight
portions** of the same blackbox log for richer, non-saturated data.

## Goals

**Premise**: INAV's internal state is trusted — INAV configurator accurately
shows orientation and the map matches reality. The blackbox logger and
`blackbox-tools` decoder are faithful passthroughs of INAV's belief (cited
in [docs/INAV_BLACKBOX.md](../../docs/INAV_BLACKBOX.md)). The xiao pipeline
and our analysis library are where convention drift can enter — **that is
what we audit and fix**.

1. **Every NN-visible sensor signal is self-consistent** with every other
   signal it overlaps with (position↔velocity, quat↔gyro, attitude↔accel,
   heading↔track), on real flight data — including manual-flight portions
   outside xiao engage spans.
2. **Every signal conforms to COORDINATE_CONVENTIONS.md** in both sim
   (CRRCSim + minisim) and real (INAV → xiao pipeline). Where they don't,
   we fix the producer, not the consumer.
3. **History buffers are correct** — temporal indices, direction semantics,
   engage-reset behavior all verified in sim and the xiao pipeline.
4. **Cadence uniform** at 100 ms/sample end-to-end (sim training = xiao
   deployment). 20 Hz-ready in API shape but not cut over.
5. **Bench test confirms** sensor conventions on the actual flight FC so
   we predict flight behavior before the next test flight.

## Running Findings Log

As we work WI1–WI6 we will uncover bugs, workarounds, fallbacks, or tech
debt that isn't the thing being audited. Capture here **on the spot**,
categorized, with a pointer to which WI absorbs the fix. This log lives
with the spec so nothing drifts into a "cleanup phase at the end" pile.

Categories:
- **bug**: behavior is wrong against the convention doc or self-consistency.
- **workaround**: behavior is OK but only because of a hack; the underlying
  cause should be addressed.
- **fallback**: we silently use a default when something upstream is broken,
  masking the upstream issue.
- **debt**: test/doc/script rot, not a behavior bug per se.
- **question**: we need input from the human or a bench measurement.

Format: `[category] short description — found in WI_, fix in WI_ (or "logged
for later")`.

### Findings from first audit run (2026-04-19)

Artifacts:
- [flight-results/flight-20260417/sensor_self_check_lib.py](../../flight-results/flight-20260417/sensor_self_check_lib.py)
- [flight-results/flight-20260417/sensor_self_check.py](../../flight-results/flight-20260417/sensor_self_check.py)
- [flight-results/flight-20260417/sensor_self_check_blackbox_log_2026-04-17_173039.01.md](../../flight-results/flight-20260417/sensor_self_check_blackbox_log_2026-04-17_173039.01.md) — real flight blackbox
- `/tmp/sensor_self_check_gen400_p0_p2.md` — sim gen-400 slice

**Flight (blackbox CSV, 9593 samples, 161s):**

- **[bug] Check 2 (gyro↔quat-delta): FAIL on roll AND pitch.** p slope
  −0.17 r −0.20; q slope −0.24 r −0.24; r slope +1.05 r +0.60. The yaw
  axis works cleanly; roll and pitch are both sign-inverted vs gyro.
  Earlier xiao-log analysis showed ONLY pitch inverted — blackbox-side
  shows an ADDITIONAL roll inversion. Points at the INAV→aerospace
  quat transform in `blackbox_to_canonical()` being wrong on TWO axes
  (the simple conjugate currently used is insufficient; NEU→NED + pitch
  convention both need explicit handling). **Fix target: T041 (msplink)
  and T044 (analysis-library)** — WI5 bench derives the correct
  composition.
- **[bug] Check 4 (accel↔gravity): FAIL.** ax slope −4.83, az slope
  +6.05 (expected both slope ≈ 1). Scale magnitude ≈ 5 suggests a
  consistent mis-rotation chaining from check 2 (expected gravity
  direction comes from rotating world gravity to body via the same
  wrong quat). Will retest after WI3 quat fix lands; may reveal
  residual scaling issue (`acc_1G = 256` verification from
  INAV_BLACKBOX.md) separately if not.
- **[debt] Check 3 (Euler↔attitude): SKIP** — flight-20260417 log
  doesn't include `attitude[0..2]` columns. This is an INAV blackbox
  config setting (`blackbox_fields`). If we want this cross-check for
  US6, add `attitude` to the blackbox field set before the next flight.
- **[warn] Check 6 (mag↔heading):** mean offset −39.9°, stddev 98.75° —
  huge variance. Likely: mag calibration on the flight FC not refreshed,
  or dynamic disturbances (motor current, ferrous frame). Low priority
  unless WI6 post-retrain flight also shows bad mag.
- **[ok] Check 1 (pos↔vel): PASS.** Slope 1.07, 1.11, 0.92 with r ≥ 0.97.
  Position and velocity transforms are internally consistent (NEU→NED
  + cm→m both working). **Validates WI1 belief cascade layer 1.**
- **[ok] Check 5 (heading↔track): PASS.** Quat-yaw direction matches
  ground-track direction (slope +1.17, r +0.81). **Validates cascade
  layer 2 for yaw.**
- **[ok] Check 7 (attitude vector↔vel direction): PASS on N and E.**
  Nose direction in world agrees with velocity direction on horizontal
  axes (N +0.81 r +0.90, E +0.61 r +0.74). D axis weak (+0.08, noisy)
  because flight is mostly horizontal. **Strong check for quat
  convention — passing suggests quat yaw direction is correct in world
  frame.**
- **[skip] Check 8 (cmd↔attitude): SKIP** — blackbox doesn't have NN
  cmd; need the fusion join (T025c) with xiao log. Defer until fusion
  implemented.

**Sim (gen 400 data.dat, 350 samples, 23s):**

- **[bug] Check 1 (pos↔vel): FAIL on N (−0.25) and noisy D (r 0.28).**
  Unexpected — sim should trivially pass. The integration uses
  `rotate_body_to_world(q_EB, v_body)` on sim's `vxBdy/vyBdy/vzBdy`.
  If sim's body velocity convention doesn't match my rotate assumption
  (or if q_EB doesn't match expectations), integration drifts. **Suspect
  workaround/fallback zone** — sim's world pos comes from the FDM
  directly; sim's body vel comes from the FDM differently. Their
  relationship may assume a rotation direction that doesn't match
  research.md §3's claimed q_EB. Logged as [workaround] candidate —
  need to trace either CRRCSim velocity export path or our rotate
  convention. Fix target: T034 sim investigation.
- **[ok] Check 2 (gyro↔quat): PASS** slope +1.10 +0.97 +1.03 r +0.96
  +0.70 +0.99 — sim's gyro and quat are self-consistent (as expected,
  sim gyro = quat integration). Confirms sim side of the cascade.
- **[ok] Check 5 (heading↔track): PASS** slope +0.99 r +0.99.
- **[ok] Check 7 (attitude↔vel dir): PASS** slopes +0.93 +0.95 +0.85.
- **[ok] Check 8 (cmd↔rate): PASS** slopes +0.15, +0.29 (weak but positive
  correlation; r is low due to saturated bang-bang commands). Sign is
  correct.

### Key implications

- **Flight blackbox quat conversion is definitely wrong on 2 axes** (roll +
  pitch). Sim quat conversion is RIGHT (slope ≈ 1 on all axes in check 2).
  The fix is in `blackbox_to_canonical()` (T044) and `msplink.cpp`
  `neuQuaternionToNed()` (T041). WI5 bench derives the exact transform.
- **Belief cascade layers 1 (pos↔vel) and 2 (yaw direction) validate** on
  flight, giving us trust in the position, velocity, and yaw pieces.
- **Sim position integration surprise** suggests sim-side convention drift
  we didn't anticipate. Could be a real bug OR a subtle expectation
  mismatch on my rotate. To investigate in T034.
- **No xiao-log-fusion check ran yet** — T025c / T025b still pending;
  will reveal cmd↔rate flight behavior.

### Additional findings captured here as we proceed below.

```
# (subsequent findings from fixes and iterations below)
```

## Work Items (priority order)

### WI1 — Full-flight sensor self-consistency audit (first story)

Exploit the ENTIRE flight-20260417 blackbox log, not just the three xiao
engage spans. Pilot-flight portions have varied, non-saturated dynamics —
rich data for cross-validating conventions.

#### Belief cascade (audit philosophy)

The audit rests on trusting what INAV has already demonstrated to be
internally consistent:

- INAV configurator accurately represents aircraft orientation, and the
  flight path displayed on its map matches reality. Therefore INAV's
  internal convention is self-consistent and physically correct.
- The blackbox logger emits INAV's internal values (source-cited in
  [docs/INAV_BLACKBOX.md](../../docs/INAV_BLACKBOX.md)). `blackbox-tools`
  is a faithful passthrough.
- **Therefore**: the blackbox CSV content can be trusted as an accurate
  representation of INAV's belief at each logged instant.

What is NOT yet trusted: **OUR conversion boundaries** — msplink's
`neuQuaternionToNed`, the analysis library's `blackbox_to_canonical`,
and the xiao→autoc interface layer. This is where 024's fixes land.

The cross-checks therefore form a cascade, each layer gating the next:

1. **Position ↔ velocity integration** — if pos/vel transforms are correct,
   integrating vel over time recovers pos delta. No attitude needed.
2. **Quat direction sanity** — body-forward rotated by our canonical q_EB
   should align with world-frame velocity direction during coordinated
   flight. Confirms quat direction semantics.
3. **Quat ↔ gyro (d/dt)** — q_delta-derived rate matches logged gyro. Both
   come from INAV's independent measurement paths (fused attitude vs raw
   gyro); internal consistency confirms our conversion on both.
4. **Euler(our_quat) ↔ attitude[]** — independent of (3) because
   `attitude[]` is INAV's own Euler extraction from the same internal
   state. Mismatch points to our extraction formula.
5. **Accel ↔ gravity** — at quasi-steady flight, body-Z accel ≈ g×cos(θ)×cos(φ).
   Confirms accel frame and our Euler extraction agree with measured specific
   force.
6. **Mag ↔ heading** — body-frame mag vector rotated to world via our q_EB
   should point approximately magnetic north (plus declination). Confirms
   full world-frame reconstruction. Board-alignment and calibration
   complications kept minimal by requiring only "rough alignment."

If all six pass on the same data, the conversion boundary is trusted. If
a specific check fails, the cascade tells us which layer owns the bug.

**Working premise**: once plan-phase research nails down each blackbox
column's meaning from `~/inav` and `~/blackbox-tools` source (see below),
we trust the INAV → blackbox → CSV pipeline end-to-end. That means pilot-
flight portions of 20260417 are as valid as xiao engage spans for
convention cross-checks — no xiao/autoc code path runs during pilot flight,
but every sensor is still logged. **Decision (2026-04-19)**: work with
20260417 first. Only schedule a supplementary pilot-only flight if WI1 is
inconclusive after plan-phase research + cross-checks.

**Plan-phase research prerequisites** (before any analysis code): the exact
interpretation of each blackbox CSV column must be re-derived from source.
We believe gyro and position transforms are correct; quat is the most
suspect. Research targets:

- `~/inav/src/main/blackbox/blackbox.c` — emission site for each field.
  What units, what scaling, what frame? Compare with bench-verified behavior.
- `~/inav/src/main/navigation/*.c` — `navPos` / `navVel` compute path.
  Confirm NED vs NEU, cm vs m, sign conventions (especially Z).
- `~/inav/src/main/imu/*.c` — quaternion construction. Confirm whether
  the stored quat is earth→body or body→earth, Hamilton vs JPL, scalar
  position.
- `~/inav/src/main/sensors/gyro.c`, `acceleration.c` — raw-ADC → logged-
  value chain. Confirm scaling (gyro `× 16 deg/s`? acc_1G const?) and
  board-alignment application order.
- `~/blackbox-tools/src/parser.c` — CSV decode. Does it apply any
  transformation (reorder, rescale, sign-flip) that would differ from
  the raw blackbox frame?
- `~/inav/src/main/flight/imu.c` or similar — `attitude[0..2]` Euler
  extraction: angle ORDER in the array (some INAV versions store
  `[roll, pitch, yaw]`, others `[yaw, pitch, roll]`) and sign convention
  (doc notes INAV displays *negative* pitch for nose up — does this
  apply to the logged `attitude[1]` too?).
- Check the INAV fork `~/inav` is actually `gcmcnutt/inav` with any
  custom modifications vs mainline.

Output of plan-phase research: an append to `docs/COORDINATE_CONVENTIONS.md`
(or a new `docs/INAV_BLACKBOX.md`) that nails each column's meaning
source-by-source, with file:line citations, so future analysis isn't
based on guesses. This research is the precondition to the actual
cross-checks.

Using the blackbox CSV only (no dependency on xiao log), verify each signal
is self-consistent with the others per
[COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md):

- **Position ↔ velocity**: integrate `navVel` over time, compare to
  `navPos` delta. NED units should agree. Catches cm/m unit bugs or Z-sign
  drift.
- **Attitude ↔ angular rate** (the check we've started): for each sample
  `q_delta = q_prev.inv() * q_curr; rate = 2 * q_delta.vec / dt` and
  compare to `gyroADC` (post-negation). Expected: y=x on all axes. This
  is `gyro_vs_quat.py` extended to the full flight.
- **Quaternion Euler ↔ `attitude[0..2]`**: INAV already computes Euler
  angles and logs them. Eigen `eulerAngles(2,1,0)` of our post-conjugate
  quat must match `attitude[]` (decideg → deg). Any mismatch is a
  conjugate or convention drift.
- **Accel ↔ gravity + dynamics**: at steady level flight, `accSmooth`
  should be (~0, ~0, +1G) in body frame. In a coordinated turn at bank
  angle φ, body +Z accel ≈ g·sec(φ). Check a sustained turn from pilot
  flight.
- **Heading ↔ ground track**: during level flight, quat-derived heading
  should match `atan2(navVel_E, navVel_N)` (within wind). Any persistent
  offset = yaw-reference drift.
- **Mag ↔ heading**: `magADC` direction should match quat heading
  (within local declination). Confirms AHRS mag fusion is healthy and
  helps characterize WI5 AHRS roll anomaly.
- **If we claim quat says we're heading south** → `navPos` north should be
  decreasing, if climbing → `navPos` Z decreasing. Spot-check a few
  representative samples end-to-end.

Deliverable: a script (likely `flight-results/flight-20260417/sensor_self_check.py`
or similar) that produces a report + per-check scatter/time-series plots.

**Pass/fail policy**: each check reports Pearson correlation and linear-fit
slope. A check **fails** only if slope is sign-inverted (a negative slope
where we expect positive, or vice versa). Correlation strength is reported
and visually inspected but not a numeric gate. Sim's own correlation on
each check is the reference; flight shouldn't be dramatically weaker
without an explainable physical reason (e.g., AHRS filter noise). Any
axis that comes out sign-flipped becomes a hypothesis for WI3.

Output → feeds WI3 fixes.

### WI2 — Sim (CRRCSim + minisim) conformance audit

Run the same self-consistency checks on sim data.dat. Sim SHOULD pass
everything (it's the convention doc's reference), but we've been wrong
before. The exact same script machinery as WI1, just pointed at sim data.

If sim fails a check, fix the sim producer BEFORE comparing against
flight — otherwise we can't tell which side is the reference.

Covers both CRRCSim (inputdev_autoc bridge) and minisim explicitly —
minisim's q_EB vs q_WB confusion (known from 023) almost certainly fails
the quat↔gyro check. That confirms WI3 scope.

Output → feeds WI3 fixes.

### WI3 — Root-cause fixes (from WI1 + WI2 findings)

Fix everything that failed the cross-checks. Candidates (confirm / expand
after WI1, WI2):

- **msplink.cpp pitch-axis convention** (high probability): the flight
  pitch sign-inversion finding suggests `neuQuaternionToNed` needs more
  than a simple conjugate. Likely a per-axis sign handling matching how
  the gyro pitch/yaw are negated. Fix and re-verify via `gyro_vs_quat.py`
  slope → 1.
- **minisim q_EB alignment**: handled in WI7 (kept as a separate WI for
  visibility; implementation tasks are grouped with other US3 fixes in
  tasks.md subgroup 5.b).
- **Position/unit/sign drift** in any script or converter that failed WI1
  (history of rotted scripts in 023 postmortem suggests more lurk).
- **History buffer semantics**: if WI1 shows historical direction cosines
  point the wrong way, fix `recordErrorHistory` and related.

Each fix is its own small commit with a paired cross-check showing the
failure→pass transition.

### WI4 — Cadence fix to 100 ms exactly (both sides)

Root cause (sim side): `inputdev_autoc.cpp:341`
`simTimeMsec > lastUpdateTimeMsec + 100` combined with ~39 ms physics tick
lands at 117 ms (3 ticks past threshold).

**Plan-phase research prerequisites** (both sides, not just sim):

- **CRRCSim inner loop in headless mode**: what actually drives the physics
  tick when the renderer is disabled? Is it a hardware timer, a sleep-based
  loop, a tick-as-fast-as-possible virtual clock? What knobs exist
  (`SIM_FPS`, `Simulation::getSimulationTimeSinceReset`, etc.)? Why does
  the header say `SIM_FPS = 25.0` with a "~40 Hz" comment and we observe
  ~39 ms ticks? Where does the FDM `dt` actually come from?
- **Xiao timer source**: is the 100 ms NN eval loop driven by a sustained
  hardware timer (no drift, exact 100 ms), or software-accumulated (subject
  to drift based on MSP request/response timing)? Check `xiao/src/msplink.cpp`
  main loop cadence source.
- **20 Hz path**: at 20 Hz = 50 ms, are there new constraints on either
  side? MSP bus saturation, physics sub-step accuracy, xiao compute time
  per NN forward pass.

Candidate approaches (decide after research):
1. Change `>` to `>=` and align physics tick to divide 100 ms cleanly.
2. Accumulate partial-tick time budget so long-run average is 100 ms
   (per-tick jitter acceptable).
3. Override physics rate in training config (50 Hz = 20 ms tick, 5 ticks
   = 100 ms exactly).
4. Combo of 1 + 3.

Verify: `awk 'NR>1 {print $4}' data.dat | uniq -c` shows every Time delta
exactly 100 ms. Xiao log `xiao_ms` deltas jitter measured ≤ 2 ms from
100 ms nominal.

Also document the actual physics stepping path. Resolve the `SIM_FPS = 25.0`
constant with its "~40 Hz" comment (they disagree).

### WI5 — Compound-attitude bench verification

Existing bench table (docs/COORDINATE_CONVENTIONS.md lines 270–285) uses
single-axis rotations. Single-axis tests cannot distinguish several quat-
convention bugs. Add compound attitudes and verify each sensor field at
the known pose.

| # | Physical hold | Expected Euler (φ, θ, ψ) |
|---|---|---|
| B1 | Level, nose north, canopy up | (0°, 0°, 0°) |
| B2 | Nose up 30°, right wing down 30° | (+30°, +30°, 0°) |
| B3 | Nose up 45°, heading east | (0°, +45°, +90°) |
| B4 | Right wing down 60°, heading south | (+60°, 0°, +180°) |
| B5 | Nose down 30°, left wing down 45°, heading west | (−45°, −30°, −90°) |

For each hold, record INAV raw quat, msplink post-processed quat, xiao
log `q=` field, gyro values (with aircraft held still → gyro ≈ 0),
accel values (should show gravity vector in body frame), `attitude[]`
Euler. Each compared against expected.

Outcome: either confirms the WI3 msplink fix worked, or exposes more
convention drift.

### WI6 — Full-sensor ground-truth audit on next flight

Once WI3 fixes land and WI5 confirms, a flight test designed to exercise
the full signal chain and verify in-motion:

- Straight climb segment (exercises pos, vel, pitch attitude, accel).
- Controlled level circle (exercises yaw rate, bank angle, centripetal
  accel, heading tracking).
- Varied-throttle span during engage (verifies throttle→airspeed polarity
  that 20260417 couldn't — throttle was saturated at +1 throughout).
- Both pilot-flown and xiao-control portions, so we have high-dynamic-range
  pilot data plus the engage spans.

Validation: same cross-checks as WI1 must pass on the new flight, now with
the WI3 fixes applied.

### WI7 — Minisim q_EB canonicalization

Carried from BACKLOG. 024 owns it fully. `AircraftState::aircraft_orientation`
is documented as q_EB but `minisimAdvanceState` composes and rotates
assuming body→world. Fix composition (pre-multiply body delta), fix
velocity rotation (`.inverse() * body_vel`), fix `minisim.cc:148` initial
velocity. Add locking unit test that builds an AircraftState at known
attitude and asserts round-trip body↔world.

Sequence with WI2 and WI3 — WI2 sim audit will confirm or refute the bug;
fix is this.

### WI8 — Xiao rabbit logging sanity

`rabbit=[x,y,z]` field in xiao NN log lines should be the NN's direct
target in virtual NED. Verify present and correct on every engage sample
and matches what the NN reconstructs from direction cosines + distance.
Discrepancy = rabbit interpolation or log bug.

Needed by renderer `-x` magenta ground-truth reference.

### WI9 — NN input layout drift guard

023's topology change silently broke the renderer (fixed now — commit
5330af6 updated parser for named-field xiao log format). Prevent recurrence:

- Schema version constant in `nn_inputs.h`, emitted in data.dat header and
  xiao log banner.
- Renderer + Python scripts read version and bail loud on mismatch.

Stronger: make data.dat parsing use named-field headers (as xiao log already
does) so index-based drift is structurally impossible. Ties to WI10.

### WI10 — Post-flight analysis scripts: rewrite around literal NN I/O

Several scripts rotted across the 023 layout change:
- `specs/018-flight-analysis/correlate_flight.py` — missing Z-flip, missing quat conjugate.
- `specs/019-improved-crrcsim/scripts/verify_flight_log.py` — reads old
  dPhi/dTheta indices.
- `specs/022-tracking-cone-fitness/flight_nn_polar_viz.py` — plots direction
  cosines as bearing angles.

Rewrite under principle: **NN input/output in the log is canonical truth.**
Consume literal named fields; shared helper module with canonical field
names; sim and flight analyses structurally parallel. Folds with WI9.

### WI11 — Renderer legacy INAV blackbox path

`tools/renderer.cc:1720–1735` loads INAV blackbox CSV directly, parallel to
the xiao `-x` path. Complies today but duplicates convention work. Decide:
keep-and-maintain (adds version check per WI9) or delete. Low priority.

### WI12 — Training run archive policy

`test4-data.dat` (6.3 GB) is the current canonical analysis target. Define
short retention policy for training runs: naming, what to preserve, what
to discard. One-page note, low complexity.

### WI13 — 20 Hz future readiness (design, no cutover)

Don't cut over to 20 Hz in this feature. But design WI4's cadence fix so
step-up is cheap:

- `EVAL_UPDATE_INTERVAL_MSEC_DEFAULT` → real config knob, not compile-time
  constant.
- Decide whether `HIST_PAST` / `FORECAST_OFFSETS` remain in ticks
  (NN window scales with rate) or become explicit ms (rate-independent).
- Note in code where 20 Hz will require additional care (xiao MSP poll,
  INAV RC-override cadence, training wall-time).

### WI14 — Workarounds & fallbacks audit

Sweep for "workaround shaped" code. Known or suspected:

- `SIM_FPS = 25.0` with "~40 Hz" comment — value and comment disagree.
  Resolve via WI4.
- `engageDelayMsec = 750` with stick-centered + cruise-throttle hold —
  is this masking a real NN startup instability, or legitimate engage
  smoothing? Revisit after WI1/WI3 clarify attitude tracking.
- `XIAO_RABBIT_SPEED_MPS = 12.0f` duplicates sim's `autoc.ini
  RabbitSpeedNominal`. Consolidate single source of truth.
- INAV board alignment bench vs flight quirks — document current values
  and why.
- `MSP_override_channels` mask — confirm in actual flight FC CLI.

Per item: removed / replaced / documented as legitimate. No "ambiguous,
revisit later."

### WI15 — Test coverage (carried from 023 P3)

- `tests/engage_reset_tests.cc` — 6 contract tests for `resetHistory()`.
- `tests/engage_delay_tests.cc` — 3 contract tests for delay window.
- `tests/nn_inputs_tests.cc` — unit-vector invariant, `sizeof(NNInputs)`
  contract, poison-value completeness.

## Validation

Feature closes when:

1. **WI1 all cross-checks pass on flight-20260417** (after WI3 fixes to
   BOTH the msplink pipeline (xiao side) AND the analysis-library
   `blackbox_to_canonical()` quat transform, no axis on any cross-check
   is sign-inverted; correlation strengths reported and visually in line
   with sim baselines). Historical blackbox data is INAV-produced and
   cannot be retroactively fixed — the fix is in how our conversion code
   interprets INAV's emission.
2. **WI2 all cross-checks pass on sim** (CRRCSim and minisim; same
   sign-inversion pass/fail, sim is the correlation reference).
3. **Cadence**: sim data.dat and xiao logs both step at exactly 100 ms.
4. **WI5 bench**: all 5 compound attitudes verify per-field against expected.
5. **Retrain + eval**: one training run on the fixed-cadence sim completes,
   eval suite (tier0 repro through tier3 stress) passes, xiao build verifies
   with the new weights. NN topology unchanged from 023.
6. **WI6 flight test**: flight of the retrained NN on the corrected
   pipeline shows the same dynamic control response as sim — cmd→rate
   slope sign-correct on all three axes, quat-derived rate agrees with
   gyro on sign, position trajectory follows the intended path family.
   Not asking for aerobatic fidelity; asking for "sim-to-real is on the
   same footing" (non-linear effects and craft variation are 025).
7. **Workarounds (WI14)**: every listed item has a disposition.

## Out of Scope

- Craft parameter variations, trim offsets, authority limit — 025.
- Effort lexicase / smoothness-pressure training — 025.
- NN topology changes (stays fixed at 33,32,16,3 for next flight).
- Hardware (airframe, servo, prop) changes.
- Cut-over to 20 Hz (design only in WI13).

## Open Questions

1. Will WI3 msplink pitch-sign fix land at a simple per-axis negation, or
   will it require re-deriving the INAV→aerospace rotation from first
   principles? WI5 bench answers this.
