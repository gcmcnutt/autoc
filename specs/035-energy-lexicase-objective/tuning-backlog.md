# 035 modelling / incentive backlog — before exiting the feature

Observations from the t6 (035 lexicase) per-axis aggressiveness analysis, 2026-06-06.
The intent here is **proper incentive — better, proper modelling** — NOT hand-tuning penalty
knobs. These are physics/objective candidates to experiment with now that the plumbing
(buckets, zstd, dmp-dump, detached `scripts/train.sh`) is solid. Resolve / promote / drop
before closing 035.

## What we saw

Per-axis aggressiveness, t6 @ gen 111 vs 034-test4 @ gen 209 (the variation-stable
cross-run comparator; both M1, RNN topology):

| axis | 034-test4 dctrl / mag | t6 dctrl / mag | reading |
|---|---|---|---|
| throttle | 0.005 / 0.997 (frozen, pinned full) | 0.157 / 0.968 | now **modulating** |
| pitch    | 0.321 / **0.827** (saturated) | 0.429 / **0.578** | **de-saturated** (the real energy win) |
| roll     | 0.425 / 0.390 (bang-bang axis) | **1.126** / 0.867 | roll bang-bang **amplified** |

Substantive change vs 034 M1: lexicase escapes 034's "throttle-pinned + pitch-saturated +
roll-bang-bang" corner and **re-allocates** — frees throttle, de-saturates pitch,
concentrates change-rate on roll.

Cross-run survey of per_axis_time_series (M1 + M2): the bang-bang axis is **set at breakout
(~gen 50–100) and persists — no clean *within-run* late flips** (033-phase4 pitch spike ~gen
150 reverts; 034-origm1-5000x49 590-gen pitch narrows toward roll but never overtakes).
Flips occur **across** runs/basins (032-pop8000-wind36-r1 is pitch-dominant), consistent with
[[project_bangbang_axis_migration]] — each basin commits early.

## Energy-cost ranking of aggressiveness (operator physics call)

Aggressiveness is NOT equally expensive per axis:

1. **Pitch — most expensive, and the well-modeled one.** Elevator → AoA → induced drag ∝ CL²,
   a direct airspeed/energy loss the LaRCSim FDM computes accurately. The energy-relevant term
   is pitch **amplitude** (sustained AoA), not change-rate.
2. **Throttle — moderate, mostly averaged.** PWM/electric: energy ≈ time-average duty cycle, so
   command wiggle ≈ holding the mean. Real extra cost (motor inrush, prop spin-up inertia, ESC
   switching) is **un-modeled**.
3. **Roll — least, but still charged.** Ailerons redistribute lift; the *direct* cost is
   `CD_AIsq*aileron²` drag, which the FDM DOES model (`CD_AIsq=0.05`, = elevator). The indirect
   adverse-yaw/sideslip cost is small by design (`Cn_da=-0.010`) — correct for a rudderless
   foam flying wing, not an omission. See the FDM check below.

Consequence (revised after the FDM check): roll is NOT a free axis — aileron drag is modeled and
equals elevator drag. So roll bang-bang is likely **genuinely near-optimal**, not a sim-under-
charging artifact. The only plausibly-under-modeled cost is the **throttle transient** (power
model, not aero). Pitch is priced correctly (induced drag), which is why the controller respects
it.

## FDM check (2026-06-06): the aero costs are already modeled and tuned

Examined `crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp` + `crrcsim/models/hb1_streamer.xml`.
The "roll is an under-charged free axis" premise does **not** hold:

- **Aileron drag is modeled and non-trivial:** `CD_all` includes `CD_AIsq*aileron²` with
  **`CD_AIsq = 0.05` — equal to `CD_ELsq` (elevator) = 0.05** (induced `CD_CLsq=0.04`,
  `CD_prof=0.15`, `CD_stall=0.10`). Full-aileron costs ~0.05 CD, on par with parasitic drag.
  Roll is NOT free.
- **Yaw is fully modeled:** `Cn_b=0.07` (weathervane), `Cn_p=-0.008` (adverse yaw from roll
  rate, ×CL/CL_0), `Cn_r=-0.04` (damping), `Cn_dr=0` (no rudder), `Cn_da=-0.010` (adverse yaw
  from aileron). Adverse yaw is *deliberately small* per the 018→023 sim-to-real tuning history
  in the file — physically correct for a **rudderless foam flying wing with elevons**, not an
  omission.
- Craft variations already perturb `CD_prof` via `craftDragDelta`.

**Conclusion: this is "close enough."** Roll bang-bang is therefore most likely **genuinely
near-optimal** — the controller pays the aileron-drag toll and the maneuvering benefit still
wins — not a fidelity artifact. Do NOT hand-weight a per-axis dctrl budget: it's a non-physical
proxy and the corner-collapse trap of [[project_scalar_multiobjective_collapse]]. dctrl/mag stay
**diagnostics, not objectives.**

## What actually remains

1. **Throttle transient (the one real gap).** Spin-up/inrush + prop inertia + ESC switching
   live in the separate power model (`mod_fdm/power`), not the aero above, and are likely
   un-modeled (energy ≈ PWM time-average). Already downgraded by operator (mostly PWM). Verify
   if/when it matters — a "revisit pre-hardware" item like [[project_cep_realism_backlog]].
2. **If bang-bang is optimal but we want manned feel** → that's a **deliberate damping
   incentive** (a design choice), NOT a fidelity fix. Defer; discuss explicitly when/if we
   decide we want it. Do not pre-emptively penalize.
3. **(Open) does the autoc energy fitness actually integrate this aero drag?** The FDM charges
   it; confirm the fitness "energy" term reflects the resulting airspeed/altitude loss so the
   modeled cost actually reaches selection. (Quick check, not assumed.)

## Verification
- **Maturity-matched re-comparison.** Re-run the t6-vs-034-test4 per-axis comparison at t6
  **gen ~210** (test4 was gen 209) for the apples-to-apples verdict — t6's gen-111 numbers are
  only ~20 gens post-breakout and the roll dctrl may still be settling.
- After any modelling fix, the *test* is whether roll/throttle aggressiveness drops **without**
  an added smoothness penalty — i.e. the energy objective alone now charges for it.

## Bang-bang is roll-only and partly a 10 Hz artifact (2026-06-07)

Per-tick command traces (t6 gen 494, 10 Hz loop = evalIntervalMsec=100):

| axis | dctrl | % sat >0.9 | sign-flips/tick | lag-1 autocorr |
|---|---|---|---|---|
| roll | 1.07 | 61% | 0.56 | **−0.24** (anti-persistent) |
| pitch | 0.37 | 0% | 0.26 | +0.44 (smooth) |
| throttle | 0.18 | 97% | 0.08 | +0.80 (pinned-high, smooth) |

Only **roll** is bang-bang — held saturated banks interleaved with hard ±1 reversals.
Pitch and throttle are coherent/smooth. At 10 Hz with a saturating roll command and
airframe roll inertia/damping (`Cl_p=-0.47`), the ±1 dither is partly the controller
synthesizing intermediate average roll (PWM-like); the airframe low-passes it to a
bounded ~155 deg/s, so the *command* looks violent but the *motion* doesn't. deg/sec
rates are airframe-limited (~150–200 roll across all regimes) — the objective only
trims them ~15–20%, it can't change the airframe envelope.

**The lever to smooth roll is a faster control loop, not the objective.** Tracking is
already met at 10 Hz (0% crash, 294/294 on hard courses) — this is headroom, not a fix.
Tracked as feature **037** (`specs/037-20hz-control-loop/spec.md`): 20 Hz, blocker =
INAV MSP serial latency, candidate = Xiao onboard IMU as high-rate source with a slow
INAV sync loop.

## Watch items as t6 matures
- **pitch amplitude** must stay low (it's the real energy term); fine if pitch *dctrl* rises so
  long as amplitude doesn't follow it back up.
- whether roll dctrl refines down or stays ~1.1–1.2 (settled strategy vs early-breakout
  overshoot).

Related: [[project_pitch_only_smoothness]], [[project_smoothness_axis_grouping]],
[[project_cep_realism_backlog]] (sim-fidelity-before-hardware pattern).
