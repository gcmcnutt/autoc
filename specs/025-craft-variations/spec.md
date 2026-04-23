# 025 — Craft Parameter Variations & Control Smoothness

**Status**: Scaffold
**Depends on**: 023 (direction cosines, engage delay, deterministic eval), 024 (sim/real fidelity — cadence, sensor verification)
**Priority**: P1 — next training pass after 024 gate
**Renamed from**: 024-craft-variations (renumbered 2026-04-18 when 024-sim-real-fidelity was introduced)

Absorbs from 023:
- Effort lexicase (T123–T136) → integrate into Change 5 Option A (chatter penalty)
- 2-dim lexicase tests (P3 T-TEST-4) → add alongside effort lexicase implementation

## Status (2026-04-22)

**BLOCKED on 026-nn-temporal-state.** Flight-20260422 showed the
cadence7 NN collapses to full-throw bang-bang as the trained policy —
with current single-pass instantaneous-input MLP topology, evolution
lacks the machinery to build smoother control. Adding craft variations
on top of a bang-bang policy will mostly add selection noise; 025's
robustness gains will land only after 026 equips the NN with temporal
state. See [`specs/026-nn-temporal-state/spec.md`](../026-nn-temporal-state/spec.md)
and the flight report at
[`flight-results/flight-20260422/FLIGHT_REPORT.md`](../../flight-results/flight-20260422/FLIGHT_REPORT.md).

The work described below is scoped and valid — it just doesn't start
until 026 produces a smoother baseline to vary against.

## Problem Statement

The NN trained in 023 achieves strong fitness (-18k+) but exhibits bang-bang
control: outputs saturate at ±1 nearly every tick.  This is unflyable on real
hardware where extreme commands stress servos, produce unpredictable aero
responses, and don't match the smooth control the FDM was tuned to.

Two root causes:

1. **Overfit to exact FDM parameters**.  The NN has only ever seen one set of
   aircraft dynamics (hb1_streamer.xml).  It can exploit precise knowledge of
   control effectiveness to bang between extremes, knowing exactly how the
   aircraft will respond.  Real aircraft have rigging tolerances, CG shift,
   asymmetric trim, and wear — the NN needs to handle ±variation to be robust.

2. **No smoothness pressure in the objective**.  The fitness function rewards
   tracking quality and streak maintenance but has no penalty for control rate,
   output magnitude, or energy.  The NN has no reason to prefer gentle commands
   over extreme ones if both produce the same tracking score.

## Goals

1. Train a NN that tracks the rabbit across a range of aircraft parameter
   variations — robust to the sim-to-real dynamics gap.
2. Reduce bang-bang control to produce flyable command profiles.
3. Maintain or improve tracking fitness relative to 023 baseline.

## Changes

### Change 1 — Craft Parameter Variations (per-scenario)

Vary aircraft dynamics parameters per-scenario using Gaussian offsets seeded
by the scenario RNG (same architecture as wind/entry variations).

#### Aero Coefficient Variations

| Parameter | Description | Sigma | Range |
|-----------|-------------|-------|-------|
| `Cl_da` | Aileron roll effectiveness | 0.15 | ±40% |
| `Cm_de` | Elevator pitch effectiveness | 0.15 | ±40% |
| `Cl_p` | Roll damping | 0.10 | ±30% |
| `Cm_q` | Pitch damping | 0.10 | ±30% |
| `CD_prof` | Profile drag | 0.10 | ±30% |
| `Cn_dr` | Rudder yaw effectiveness | 0.10 | ±30% |
| `Cn_b` | Yaw stiffness (weathervane) | 0.15 | ±40% |
| `Cn_r` | Yaw damping | 0.10 | ±30% |

##### HB1 rudder-moment calibration (2026-04-21 observation)

cadence7 sim-video playback of an eval flight shows the craft tracking
the path correctly overall, but with apparent yaw/rudder wobble that
the real HB1 doesn't exhibit at equivalent conditions. The sim HB1
streamer model probably under-models the rudder's authority: the
physical HB1 is a flying-board layout with a **large rudder positioned
noticeably aft of the wing**, giving it more yaw moment arm and more
effective `Cn_dr` / `Cn_b` than a conventional low-moment-arm tail
surface.

025 should:

1. **Measure the real HB1 yaw-response first** (bench ground-run with
   rudder step + gyro log, OR flight rudder-step with blackbox capture).
   Compare sim cadence7 step response to the measured baseline; decide
   whether `hb1_streamer.xml` baseline values (`Cn_dr`, `Cn_b`, `Cn_r`,
   moment-arm scaling) need updating *before* applying variations.
2. **If the baseline is off**: update `Cn_dr` in the airframe XML to
   match measured authority. Retraining on corrected baseline will
   naturally reduce the wobble that cadence7 carries in as a
   compensation for under-authority sim.
3. **Then apply the variations table above** on top of the corrected
   baseline, so the NN learns both "authority is this strong" and "but
   varies ±30% per airframe."

Two of the variation rows in the table above are new for HB1 yaw
specifically (`Cn_b`, `Cn_r`) — conventional-tail aircraft might not
need them, but a flying-board layout with an oversized rudder makes
them worth exercising.

#### Mass/Inertia Variations

| Parameter | Description | Sigma | Range |
|-----------|-------------|-------|-------|
| `mass` | Aircraft mass | 0.05 | ±15% |
| `Ixx` | Roll inertia | 0.05 | ±15% |
| `Iyy` | Pitch inertia | 0.05 | ±15% |
| `Izz` | Yaw inertia | 0.05 | ±15% |

#### Control Timing

| Parameter | Description | Sigma | Range |
|-----------|-------------|-------|-------|
| `ctrl_phase_delay` | Command-to-servo delay | 50ms | 0-200ms |

### Change 2 — Trim Offsets (per-scenario)

Real aircraft have imperfect trim — CG not exactly on CL, slight asymmetry
left/right, pitch bias from incidence rigging.  The NN must learn to
compensate rather than assuming perfect trim.

| Parameter | Description | Sigma | Units |
|-----------|-------------|-------|-------|
| `pitch_trim` | Pitch moment offset (CG vs CL) | 0.02 | Cm units |
| `roll_trim` | Roll moment asymmetry (left/right) | 0.01 | Cl units |
| `yaw_trim` | Yaw moment bias (P-factor, torque) | 0.005 | Cn units |

These are additive moment offsets applied every FDM tick, simulating constant
aerodynamic biases the NN must counter.  The NN will need to learn a baseline
trim output rather than assuming zero-stick = straight flight.

### Change 3 — Authority Limit (from 023 US5)

Scale NN outputs before application: `output_scaled[i] = output_raw[i] * NNAuthorityLimit`.

- Default: 1.0 (no change, baseline comparison)
- Target: 0.5 for robustness training (forces NN to work in ±0.5 range)
- Applied in CRRCSim, minisim, and xiao identically
- Combined with craft variations, this forces the NN to find smooth solutions
  that work across parameter uncertainty with limited authority

### Change 4 — RC Smoothing Filter (MOVED TO 023 Phase 9a)

INAV's RC smoothing filter (pt3, 3rd-order cascade) is being implemented in
CRRCSim as a 023 experiment (Phase 9a, T114-T120).  If training with the
filter produces smooth commands and acceptable fitness, 023 can go to flight
test without waiting for 024.

The pt3 filter covers all channels including throttle, runs at the CRRCSim
FDM rate (333Hz), and uses the same algorithm as INAV's `rc_smoothing.c`.
The real INAV config will be updated to match: `rc_filter_lpf_hz = 20`.

See `specs/023-ood-and-engage-fixes/tasks.md` Phase 9a for full details.

024 assumes the RC filter is already in place from 023.  Craft variations
(Changes 1-3 above) layer on top of it.

### Change 5 — Control Smoothness Pressure (conditional, after Changes 1-4)

Only if Changes 1-4 don't naturally smooth out bang-bang.  Options:

**Option A — d²output² chatter penalty (lexicase dimension)**:
Add a third lexicase dimension measuring tick-to-tick output change:
`chatter = mean(|u[t] - u[t-1]|²)` across pitch/roll/throttle.
Lower chatter preferred at equal tracking score.

**Option B — Minimum turn radius constraint**:
Penalize scenarios where the aircraft's instantaneous turn radius drops below
a threshold (e.g., 5m).  This makes tight spiral solutions infeasible.

**Option C — Control rate penalty in step score**:
Subtract `k * |du/dt|` from the per-step fitness score.  Direct pressure on
every tick to minimize control rate.

Decision on A/B/C deferred to post-Change-1-4 observation.

## Implementation Approach

### CRRCSim FDM Integration

The hb1_streamer.xml parameters are loaded by the LaRCSim FDM at init.
Per-scenario deltas need to be applied AFTER model load, BEFORE the scenario
runs.  The FDM exposes these parameters through the EOM01 interface.

Key integration point: `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`
at scenario start, after `Global::Simulation->reset()`.

Trim offsets are applied as additive moments in the FDM force/moment
computation loop — they persist for the entire scenario duration.

### Variation Ramp

Same architecture as 023: `VariationRampStep` controls a linear ramp from 0%
to 100% of configured sigmas over the training run.  Craft variations ramp
alongside existing entry/wind variations.

### Config Knobs

All new parameters in `autoc.ini` with defaults of 0.0 (disabled).  Training
operator enables them incrementally:

```ini
# Craft parameter variation sigmas (0 = disabled)
CraftClDaSigma              = 0.0
CraftCmDeSigma              = 0.0
CraftClPSigma               = 0.0
CraftCmQSigma               = 0.0
CraftCDProfSigma            = 0.0
CraftMassSigma              = 0.0
CraftInertiaSigma           = 0.0
CraftCtrlPhaseDelaySigma    = 0.0

# Trim offset sigmas (0 = disabled)
TrimPitchSigma              = 0.0
TrimRollSigma               = 0.0
TrimYawSigma                = 0.0

# Authority limit (1.0 = full authority, no change)
NNAuthorityLimit            = 1.0
```

### RPC Protocol

Per-scenario craft deltas and trim offsets are transmitted in `ScenarioMetadata`
via the existing RPC protocol.  CRRCSim applies them at scenario start.

## Validation

### Milestone A — Baseline (no craft variations)
Run 023's best NN through eval with `CraftXxxSigma = 0`.  Record fitness as
the baseline to beat.

### Milestone B — Craft variations only
Enable `CraftClDaSigma=0.15`, `CraftCmDeSigma=0.15`, `CraftCtrlPhaseDelaySigma=0.05`.
Train 400 gens.  Compare fitness and control profiles.  If bang-bang persists,
proceed to Change 3/4.

### Milestone C — Authority limit + variations
Set `NNAuthorityLimit=0.5`.  Retrain.  Compare.

### Milestone D — Smoothness pressure (if needed)
Add chatter penalty or min-turn-radius.  Retrain.  Compare.

### Flight readiness gate
- Control outputs must show mean |command| < 0.6 (no persistent saturation)
- Tracking fitness within 80% of 023 baseline
- Robust across ±20% control effectiveness variation
- Zero bearing discontinuities (structural check from 023)

## Relationship to 023

| 023 Item | 024 Disposition |
|----------|----------------|
| US3 Throttle lexicase | Absorbed — throttle discipline via authority limit + variations |
| US4 Forcing paths | BACKLOG — not needed for next flight |
| US5 Authority limit | Change 3 above |
| US6 Craft variations | Changes 1-2 above (expanded with trim offsets) |
| Memory leak investigation | Remains in 023 — fix before 024 training |

## Carried over from 024

These items were flagged as not flight-critical for 024 and deferred to 025
(2026-04-21):

### Minisim conformance audit + q_EB canonicalization (WI7)

Minisim is a lightweight sim used for some offline analysis; it was flagged
in 024 as having a `q_EB` vs `q_WB` convention bug (see 024 WI7 and
tasks.md 5.b T050–T055). **Not on the critical path for 024 flight**
because minisim isn't in the training pipeline (CRRCSim is) and isn't on
the xiao. Carry into 025 as part of the overall convention-canonicalization
pass, since 025 touches minisim and aircraft_state anyway:

- Minisim conformance audit: add fixed-attitude-scenario support to
  `tools/minisim.cc`, pipe output through `sensor_self_check.py`,
  confirm the expected `q_EB` bug surfaces. (024 T033)
- `tests/minisim_convention_tests.cc` — compound-attitude bench in code.
  (024 T050)
- Fix `minisimAdvanceState` composition to `delta_body * aircraft_orientation`
  and velocity rotation to world-via-inverse. (024 T051)
- Drop or re-derive the initial-velocity pre-rotation in
  `tools/minisim.cc:148`. (024 T052)
- Verify gyro-rate sign at `aircraft_state.h:379-382`. (024 T053)
- Name `AircraftState::aircraft_orientation` as canonical q_EB in
  `docs/COORDINATE_CONVENTIONS.md`. (024 T054)
- Confirm T033 audit passes after the above. (024 T055)

### NN input layout drift guard + schema version (024 T081)

024 flagged that `sizeof(NNInputs) == 33 * sizeof(float)` is currently
enforced by a compile-time static_assert but not by a boot-time log line
or a version field in serialized artifacts. Easy to add alongside 025's
inevitable input-layout churn (new variation-aware features, etc.).

- Add a boot-time log line asserting the sizeof invariant. (024 T081)

### Path-Relative Smoothness / Control-Effort Pressure

See the 024 bang-bang evolution analysis
(`specs/024-sim-real-fidelity/cadence7_bang_bang_evolution.md`). cadence7
already shows a weak downward dCtrl trend (~30% over gens 90–400),
suggesting the current lexicase setup has residual selection for smoother
control — it's just faint because the airframe+servo are fixed. 025's
Change 1 (aero + mass + CG + servo variations) should amplify this signal
without any explicit smoothness term in fitness, consistent with the
"no tunables" project constraint. Re-run
`plot_control_aggressiveness.py` after 025's first full training to
measure the shift. If variations don't move the dCtrl/|out| plateau
meaningfully, revisit a structural selection change (e.g. NSGA-II on
tracking + robustness-proxy) — but variations first.

## Backlog Items Absorbed into 024

From `specs/BACKLOG.md`:

| Item | Disposition |
|------|-------------|
| **Path-Relative Smoothness** (DEFERRED) | Revisit as Change 4 Option C if variations don't smooth control |
| **Simulator Sampling Time Variation** (DEFERRED) | Natural fit — add sim tick jitter (±10ms) as another per-scenario variation |
| **Memory Leak Investigation** | Remains in 023 — RSS ~40GB needs heap analysis before 024 training |
| **Batch and Cache Deterministic Scenarios** | Performance optimization — consider if 024 training is too slow |
| **Phase 6 Aircraft Parameter Variation (015)** | This IS 024 Change 1 — closing the loop |

## Open Questions

1. Should trim offsets be constant per-scenario or time-varying (simulating
   CG shift from fuel burn)?  Constant is simpler and sufficient for first pass.
2. What sigma values produce meaningful variation without making training
   impossible?  Start conservative (suggested values above), widen iteratively.
3. Does the FDM expose all needed parameters through EOM01, or do some require
   deeper FDM surgery?
4. Population size — 023 test3 shows excellent results with deterministic eval.
   Can we drop from 10k to 2-5k pop and save 50-75% wall time?
5. Sim tick jitter — should we add ±10ms dither to the 100ms eval cadence as
   a variation?  Would harden NN against real MSP bus timing jitter.
