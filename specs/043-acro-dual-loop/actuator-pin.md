# T046 — actuator / airframe response, measured IN FLIGHT (2026-09-07)

Source: `flight-results/flight-20260907/blackbox_log_2026-09-07_103927.TXT`, **log 2 of 2** (3:38, 12,941
samples at 59 Hz, 82% `ARM|MANUAL`). Flown per [T046-alt-inflight.md](T046-alt-inflight.md): MANUAL
throughout, sharp roll/pitch steps, **airspeed as the load sweep** (8–25 m/s, median 14.3).

⛔ **MANUAL, so there is no NN and no ACRO loop in this measurement.** What is measured is
`rcCommand → achieved body rate`: the **actuator + airframe composite**, which is exactly the thing the
sim is judged on.

Method: 26 roll / 39 pitch step events detected (|Δ| > 250 counts inside 85 ms, MANUAL only, 0.5 s
refractory), each normalised by its own step size and sign, baseline-subtracted, then **averaged**.
Averaging beats per-step fitting at 59 Hz.

⚠️ Caveats, stated once: the pilot's step is a ~85 ms ramp, not an ideal step, so rise times are **upper
bounds** and dead-time is an **upper bound**; sampling is 16.8 ms so all timings carry ±17 ms; per-band
n is 5–16.

---

## 1. ROLL — first order, no overshoot

| | |
|---|---|
| steady gain | **133 °/s per 100 command counts** |
| response detectable by | **≤ 34 ms** (1–2 samples; the true dead-time is below the resolution) |
| 63% of final | **~134 ms** |
| shape | monotonic rise to a plateau, then gentle decay. **No overshoot, no oscillation.** |

Gain by speed band: slow (<12 m/s) 139, mid (12–17) 120, fast (≥17) **162** — rising at the fast end as
expected, though n = 5 at each edge.

## 2. ⭐⭐ PITCH — a lightly damped SHORT-PERIOD MODE at ~3 Hz

Pitch does **not** behave like roll. It overshoots hard, falls back, and rings:

| band | peak | t_peak | trough | t_trough | 2nd peak | period | **freq** |
|---|---:|---:|---:|---:|---:|---:|---:|
| ALL | 53.6 | 134 ms | 19.0 | 336 ms | 25.7 | 336 ms | **2.98 Hz** |
| slow <12 | 52.6 | 168 ms | 13.2 | 437 ms | 13.9 | 336 ms | 2.98 Hz |
| mid 12–17 | 48.0 | 134 ms | 7.2 | 336 ms | 20.2 | 370 ms | **2.71 Hz** |
| fast ≥17 | 64.2 | 134 ms | 23.5 | 302 ms | 47.6 | 302 ms | **3.31 Hz** |

⭐ **Peak/steady ≈ 2.5×** — the initial pitch rate is two and a half times what it settles to.
⭐ **Frequency rises with airspeed, 2.71 → 3.31 Hz**, which is what a short period must do physically.
That consistency is the main reason to trust the number.

This is the classic statically-stable short period: elevator step → pitch rate spikes → AoA builds →
static stability and pitch damping arrest it → rate falls back and rings. Estimated ζ ≈ **0.18** in the
fast band (log-decrement; the other bands don't settle inside the window, so no number is offered rather
than a bad one).

## 3. ⛔ Why this reframes 043

| measurement | where it came from |
|---|---|
| **~3 Hz lightly damped pitch short period** | this flight, **MANUAL** — airframe only |
| 67% of engaged pitch-rate power at **1–3 Hz**, RMS 110 °/s | 2026-09-05 flight, NN engaged |
| 3–5 Hz = 30.1% of real roll power vs sim's 12.6% | 041-t7 |

⭐ **The frequency the NN excites in flight is the airframe's own natural mode.** The policy is not
inventing an oscillation — it is pumping a lightly damped resonance that the aircraft already has, and
that MANUAL flight shows plainly with no controller involved.

⛔ **Consequence**: 043's premise was that the oscillation is a *phase-budget* problem. This says a large
part of it is a *plant* problem. A faster loop helps only if the sim's plant has the same mode; if the
sim's short period is better damped or at a different frequency, the policy never learns to avoid exciting
it, and no amount of latency tuning will transfer.

### ⇒ The next test is cheap, deterministic, and not a flight

**Run this exact step protocol in CRRCSim** — MANUAL, scripted roll/pitch steps at 10/15/20 m/s — and
compare, on the same axes:

1. roll steady gain and 63% time
2. **pitch peak/steady ratio** (real: 2.5×)
3. **pitch short-period frequency and its trend with airspeed** (real: 2.71 → 3.31 Hz)

That is a direct, falsifiable sim↔real dynamics comparison that needs no aircraft, no weather, and no
genome. ⭐ It should happen **before** the latency constants are touched: if the short period is wrong,
that is the bigger error and it changes what the latency numbers even mean.

## 4. Blackbox rate: the ladder was flown, and SPIFLASH holds

Answering the 2026-09-05 question, which was decided on a guess and has now been **measured**:

| log | rate | data rate | loop iterations missing |
|---|---:|---:|---:|
| `…103927` (both logs) | 59 Hz | 8.4 kB/s | **0** |
| `…104749` bench, varied controls + handling | **481 Hz** | 18.9 kB/s | **7 = 0.01%** |
| `…114406` (3 logs) | ~480 Hz | 18.6 kB/s | 4–6 = 0.02–0.07% |

⭐ **SPIFLASH sustains 481 Hz / ~19 kB/s at ~0.01% loss** — 2.3× the data rate of the current setting,
with the *full* 92-field set (the field strip in `fe661e4` was not even needed to get there). ⇒ The
dropout worry is retired; a future actuator sortie can run at 240–480 Hz and resolve dead-time properly
instead of bounding it at ±17 ms.

⚠️ What that does **not** change: the 2026-09-05 reasoning for keeping the *flight* at 59 Hz still holds
on its own terms — everything 043 is judged on was already well sampled there, and the high rate buys
resolution only for this actuator work.

---

# Addendum — the 2026-09-07 anomaly log, and how to run this same test in the sim

## 5. `…114406` (3 logs, arm/disarm, no flight): the FC was fine; the STALLS garble telemetry

Operator report: *"the telemetry link goes crazy as if corrupted data … acro, disarm, failsafe, gps
incessant messages"*, so the sortie was abandoned before flight. Hypothesis offered: loop timeout /
schedule overrun.

⭐ **The FC's control path was healthy.** Across all three logs: `FAILSAFE` never set, and log 02 shows
only **2 mode transitions in 20.1 s** (`ARM` → `ARM|MANUAL` → `MANUAL`) — a normal arm/ACRO/MANUAL/disarm
sequence, not the flapping the TX was announcing.

⛔ **But the scheduler does stall, and only at the high blackbox rate:**

| log | rate | intervals >1.5× nominal | >3× | **max stall** |
|---|---:|---:|---:|---:|
| `…103927` ×2 | 59 Hz | **0.00%** | 0.00% | 17.5 ms (= 1.04× nominal) |
| `…104749` | 484 Hz | 0.01% | 0.01% | **16.5 ms** (8×) |
| `…114406` ×3 | ~481 Hz | 0.02–0.07% | 0.01–0.07% | **10.5–12.3 ms** (5–6×) |

At 59 Hz there is **literally not one** interval beyond 1.5× nominal. At 481 Hz the SPIFLASH write
occasionally blocks the scheduler for **10–16 ms** — rare (≤0.07%), harmless to the blackbox itself
(0.01% of samples lost, §4), but **long enough to corrupt a timing-sensitive telemetry frame**, which the
TX then reads as spurious mode/failsafe/GPS events.

⇒ Consistent with everything observed: blackbox data intact, FC modes stable, TX shouting.
⚠️ **Mechanism, not proof.** Falsifying test, cheap: fly the same rate with telemetry disabled (or drop to
240 Hz) and see whether the TX quiets. ⇒ Practical rule for now: **the high rate is for
instrumentation sorties, not for flights where telemetry matters.**

## 6. ⭐ Running this identical step test in the sim — the instrument already exists

The comparison in §3 needs the sim's own step response. It does **not** need new physics logging:
`PhysicsTraceEntry` (`include/autoc/eval/aircraft_state.h:881`) already records, per FDM substep, in
**native SI doubles**:

| field | units | why it matters here |
|---|---|---|
| `omegaBody[3]` | **rad/s** | the body rates — the sim's counterpart to blackbox `gyroADC` |
| `omegaDotBody[3]` | rad/s² | angular acceleration, free rather than differentiated |
| `alpha`, `beta` | **rad** | ⭐ AoA — the state that *drives* the short period. The blackbox cannot give this. |
| `vRelWind` | m/s | true airspeed, i.e. the load-sweep variable |
| `Cl, Cm, Cn` | — | ⭐ moment coefficients: `Cm` vs `alpha` **is** the static-margin/damping question |
| `momentBody[3]`, `forceBody[3]` | N·m, N | |
| control inputs (`TSimInputs`) | — | the commanded step |
| `density`, `gravity` | SI | |

**Rate**: `Global::dt = 0.002777 s`, rounded to ms ⇒ **3 ms/step = 333 Hz**. Comparable to the 481 Hz
blackbox and far better than the 59 Hz flight log this measurement came from — so the sim side can resolve
dead-time the real side could only bound.

### What blocks it, and it is small

1. ⛔ **`MAX_TRACE_STEPS = 35`** (`fdm_larcsim.cpp:75`) — ~105 ms, set for RNG-divergence debugging. A step
   response needs ~600 ms ⇒ **200 steps**. ⚠️ Do not raise it globally for a bake: the trace is per
   scenario and 294 × 200 × ~400 B is large, which is exactly why the cap exists. Make it configurable and
   raise it only for the test.
2. **The collector is autoc-specific.** The FDM fills `gCurrentPhysicsTrace` **unconditionally**
   (`fdm_larcsim.cpp:956`), but only `inputdev_autoc` resets the counter and drains it into
   `evalResults.physicsTrace`. A hand-flown GUI session fills 35 steps once at startup and then stops.
   ⇒ Needs a small **CSV sink**: write each entry as it is produced when a flag is set. No buffering, no
   cap, no dependence on autoc mode. That is the whole change.

⇒ Then hand-fly the §1/§2 protocol in CRRCSim — MANUAL, sharp roll/pitch steps at 10/15/20 m/s — and run
the same averaging script. ⭐ `Cm` and `alpha` come along for free, so if the sim's short period is wrong
the trace says *why*, not just *that*.

### Unit mapping for the comparison

| quantity | real (blackbox) | sim (`PhysicsTraceEntry`) |
|---|---|---|
| body rate | `gyroADC[0..2]`, **deg/s** | `omegaBody[0..2]`, **rad/s** ⇒ ×57.2958 |
| command | `rcCommand[0..1]`, ±500 counts | `TSimInputs` aileron/elevator, ±0.5 ⇒ ×1000 for counts |
| airspeed | `navVel` magnitude, cm/s | `vRelWind`, m/s |

⚠️ One asymmetry to respect: the real `rcCommand → surface` path carries the **actuator** (50 Hz PWM latch
+ slew); the sim applies its own servo model. So compare **command → body rate end-to-end** on both sides
— the composite — exactly as §1/§2 did. Splitting actuator from airframe is what the bench rig was for and
is not what this comparison needs.


---

# ⛔ CORRECTION (2026-09-07, later) — §2's frequency table was window-limited

Re-running §1/§2 through the reusable tool
([`step_response.py`](step_response.py), which now runs on **both** sides) exposed a
methodology fault in the table above and in its commit message.

**What was wrong.** §2 used a 0.60 s post-step window and read the frequency from
peak→trough→peak. For three of the four bands the trough was **sitting on the window edge**, so the
"period" was an artefact of where the window ended, not a measured ring. Widened to 0.90 s, those bands
show the response still decaying with no second peak at all.

⚠️ **A second confound, also mine**: past ~0.5 s the averaged real response mixes in the **pilot releasing
the stick**. The real steps had no controlled hold duration. So anything read from the late part of that
average is suspect on principle. (The sim test controls this exactly — `holdSec = 0.8` — which is one more
reason the sim side is the better instrument.)

**What survives, and is now measured robustly.** For a second-order step, time to first peak is
`t_pk = π/ω_d`, so `ω_d = π/t_pk`. That needs **only the peak** — the single most reliable feature here —
and is immune to both the window edge and the stick release:

| band | n | peak | t_peak | **f = π/t_pk** | peak-to-peak (corroboration) |
|---|---:|---:|---:|---:|---|
| slow <12 | 12 | 52.6 | 166 ms | **3.01 Hz** | — (no ring in window) |
| mid 12–17 | 14 | 48.5 | 149 ms | **3.35 Hz** | 2.87 Hz ✅ agrees within 15% |
| fast ≥17 | 13 | 64.2 | 133 ms | **3.76 Hz** | — (no ring in window) |

⭐ **The headline stands and is now better supported**: pitch overshoots hard (peak at 133–166 ms, well
inside any plausible hold, so this part was never at risk) and the short-period frequency **rises
monotonically with airspeed, 3.01 → 3.35 → 3.76 Hz** — the trend physics demands. The one band with an
independent estimate agrees.

⛔ **What is NOT established**: a damping ratio. The ζ ≈ 0.18 quoted in §2 came from a log-decrement on the
degenerate fast-band trough and should be **disregarded**. Damping needs either the sim (where the hold is
controlled) or a real sortie with held steps.

⇒ §3's conclusion is unchanged — the NN excites the airframe's own ~3 Hz mode — and §3's recommended sim
comparison is now the way to pin the damping the real data cannot.

---

# §7 — SIM vs REAL, measured (2026-09-07): the sim has NO pitch short period

Ran the §3 comparison. Sim side: `Cntrl_StepTest` on `hb1_streamer_steptest.xml`, MANUAL-equivalent
(no `InavFwRate` in the chain), 13 cells / 85.7 s at **200 Hz** (`autoc_config.xml` sets
`flightModel dt="0.005"`), airspeed **9.6–26.7 m/s** (median 17.6) against the real flight's 8–25
(median 14.3). 12 of 13 cells clean; cell 10 clipped and is excluded.

## ROLL — the sim is close

| | sim | real |
|---|---:|---:|
| steady gain (deg/s per 100 counts) | 156 (fast band 172) | 133 (fast band 165) |
| 63% of final | 120 ms | 133 ms |

⇒ Within ~17% on gain and ~10% on rise. **Roll is not the problem.**

## ⛔ PITCH — the sim tracks the input; the real aircraft has dynamics

Raw trace, one clean mid-power cell, against the real averaged response:

| | **sim** | **real** |
|---|---|---|
| peak | **85 ms** — exactly the end of the 85 ms ramp | **133–166 ms** — *after* the input stops moving |
| peak/steady | **2.8×** | **~2.5×** |
| after the peak | monotonic decay, **settled by ~175 ms** | **rings at ~3 Hz**, still oscillating at 500 ms |
| short-period freq | none observable | **3.01 → 3.35 → 3.77 Hz**, rising with airspeed |

⭐ **The overshoot MAGNITUDE matches (2.8 vs 2.5) but the DYNAMICS do not exist.** The sim's pitch rate
peaks when the surface stops moving and then relaxes — a quasi-static response with strong damping. The
real aircraft keeps building rate for another ~60–80 ms after the input stops, then rings.

⇒ **§3's hypothesis is confirmed.** The ~3 Hz pitch energy the NN produces in flight is an airframe mode
the **sim does not have**. A policy trained in that sim cannot learn to avoid exciting a resonance that
does not exist in its world, which is why the oscillation appeared only in the air.

⛔ **This outranks the latency constants.** §6 listed the sim as ~20 ms too slow on compute and ~66 ms too
slow on the rate loop; those are real, but they are *tuning* against a plant whose pitch dynamics are
structurally absent. ⇒ **Fix the short period first.** The candidates are in the FDM aero block —
pitch damping (`Cmq`), static margin (`Cm_alpha`), and pitch inertia — and `PhysicsTraceEntry` already
records `alpha`, `Cm` and `momentBody`, so the diagnosis is available without new instrumentation.

## ⚠️ Reading limits of this run — stated so the next person does not over-trust it

1. ⛔ **The averaged sim pitch numbers from `step_response.py` are NOT reliable**; the raw per-cell trace
   above is. The ALL band mixes cells with different trim datums and both step polarities, and the
   frequency estimator then reports nonsense (16.7 Hz). ⇒ **Tool gap: the sim side needs a per-cell mode**
   rather than pooled averaging. Roll pooled fine because its cells share a trim; pitch does not.
2. The real side is a **pilot ramp with no controlled hold**; the sim holds exactly 0.8 s. Comparisons
   past ~0.5 s on the real side include the stick release.
3. `n = 1` airframe, `n = 1` sim model. This says the *shape* differs, which is a structural claim and
   robust; it does not pin a damping ratio on either side.
4. Two bugs were found by running it, both now fixed and worth remembering: **`v_rel_airmass` is ft/s**
   (crrcsim carries velocities in feet), and **crrcsim's elevator sign is inverted** relative to the rate
   it produces (`+elevator → −rate_q`, which is why `cntrl_inavfwrate` carries `pitchCmd = −2·elevator`).
   Un-fixed, the second silently cancels the sign-normalised pitch average to zero.

## Regression-test candidate

⭐ The operator's note — *"at some point we put this in regression tests on each build"* — is well aimed,
and the ROLL numbers are ready for it now: gain and 63% time are stable, pooled cleanly, and would catch
an FDM or servo-model regression immediately. ⇒ Suggested gate once the per-cell fix lands: assert roll
gain and 63% time inside a band, and assert the pitch peak/steady ratio, per cell. ⛔ Do **not** gate on
the pitch *frequency* until the sim actually has one — that is the open finding, not a regression.

---

# ⛔ §8 — RETRACTION: it is not a short period, it is a STALL CYCLE (operator, 2026-09-07)

Operator: *"this particular craft appears to stall on hard pitch up — and is nose heavy … the wing is
asymmetric and not that great."* Splitting the pitch steps by DIRECTION — which §2 and §7 did not do,
because sign-normalised averaging silently **assumes the airframe is symmetric** — shows they are right.

## Three independent signatures, all pointing the same way

**1. It happens in ONE direction only.** Averaged separately (n=19 down, n=20 up):

| | peak | after the peak |
|---|---|---|
| nose-DOWN (cmd +ve) | +49 @151 ms | smooth monotonic decay — **no ring** |
| nose-UP (cmd −ve) | −59 @151 ms | collapses to −10 @302 ms, then **swings BACK to −31 @504 ms** |

A linear short period is a property of the linearised airframe and appears in **both** directions.

**2. The load factor collapses.** `accSmooth[2]` through the nose-up response: rises to **+8229 @235 ms**,
falls to **+3848 @370 ms** (−53%), recovers to **+5564 @504 ms**. ⭐ Lift breaking down at peak AoA and
re-attaching as the nose drops. Airspeed barely moves (+0.4 then back), so this is an **AoA** event, not
an energy one.

**3. ⭐ The frequency depends on AMPLITUDE — which a linear mode's cannot.**

| nose-up steps | mean step | peak | recovery | 2nd dip | rebound | implied freq |
|---|---:|---:|---:|---:|---:|---:|
| smaller half (n=11) | 226 counts | −66 @168 ms | −13 @386 ms | −31 @655 ms | **26%** | **2.05 Hz** |
| larger half (n=12) | 351 counts | −64 @134 ms | −14 @319 ms | −37 @470 ms | **35%** | **2.98 Hz** |

## What this retracts, and what survives

⛔ **RETRACTED**: the §2/§7 framing of a "lightly damped **short period** at ~3 Hz", and with it the §7
headline *"the sim has NO pitch short period."* That comparison used ±0.15 surface around trim and
**never approached stall**, so it did not test the phenomenon at all. The `f = π/t_peak` numbers in the
§2 correction (3.01 → 3.35 → 3.77 Hz) describe the *first* peak of a nonlinear response; they are not a
modal frequency and the airspeed trend in them is confounded with step amplitude.

✅ **SURVIVES**: (a) roll is first-order and the sim matches it within ~17% gain / ~10% rise;
(b) the sim's near-trim pitch response is quasi-static — it peaks with the input and settles by ~175 ms
with **no rebound at all**, where even the *smaller* real steps rebound 26%;
(c) whatever it is, the real aircraft has pitch behaviour at 2–3 Hz that the sim did not reproduce in the
regime tested.

⭐ **And the reframing is more useful than what it replaces.** The 2026-09-05 flight had the NN **railed in
pitch** (38% saturation in span 4) with 67% of its pitch-rate power at 1–3 Hz. A policy repeatedly
commanding hard pitch-up into a stall/recovery cycle is a far more specific and actionable story than
"excites a resonance" — and it is consistent with a **nose-heavy** aircraft, which needs more up-elevator
to trim and therefore sits closer to the stall boundary on every pull.

## ⇒ What to test next, revised

1. ⛔ **Re-run the sim comparison at LARGE nose-up amplitude**, to stall, one direction. That is the
   experiment §7 should have been. `PhysicsTraceEntry` records `alpha`, `CL` and `Cm`, so the sim can be
   asked directly whether its wing breaks down at the same AoA.
2. **Split by direction everywhere.** ⚠️ `step_response.py` sign-normalises and therefore assumes symmetry —
   it must gain a `--split-sign` mode before it is trusted on pitch again, on either side.
3. ⭐ **The second wing is now a high-value data point, not just a spare.** Different planform, different
   servos, and — if it is less nose-heavy or less asymmetric — a direct test of whether this cycle is a
   property of *this* article rather than of the design. ⚠️ Until then, `n = 1` airframe, and the
   BACKLOG rule about the second article applies with more force than before, not less.

---

# §9 — STALL PROBE RUN: the sim DOES break down, and §7 was wrong

Ran the §8 experiment: large-amplitude pitch pulses, **both directions**, three power settings, from an
auto-trimmed state, 900 ft launch for headroom. `artifacts/stallprobe-sim-20260907.csv`.

## Result — the asymmetry reproduces

| direction | n (level start) | rate peak | **rebound** | **CL peak → min** |
|---|---:|---:|---:|---|
| amp −ve (nose-up sense) | 2 of 4 | **85 ms** | **19–46%** (mean 22.4) | **1.10 → 0.31–0.48**, collapse **57–72%** |
| amp +ve | 3 of 4 | ~1500 ms | **~0%** | 0.45–0.69, no early break |
| **REAL nose-up** | 20 | **133–166 ms** | **26–35%** | (load factor collapse 53%) |

⭐ **`flight_cl` reaches 1.10 and then collapses 57–72%.** That is a wing stalling, recorded directly
rather than inferred from load factor as the blackbox forced. And the rate rebound — **19–46% sim vs
26–35% real** — overlaps. **The sim has the mechanism, in the right direction, at roughly the right
strength.**

## ⛔ Correcting §7 again

§7 concluded *"the sim has NO pitch short period"* / *"the dynamics do not exist."* **That was wrong**, and
§8 correctly diagnosed why before this run: the ±0.15 near-trim matrix never approached the stall
boundary, so it measured the sim's *linear* regime and pronounced on a *nonlinear* phenomenon. At stall
amplitudes the sim behaves qualitatively like the aircraft.

**What the gap actually is, now that both sides are measured in the same regime:** the sim reaches its
rate peak at **85 ms** — exactly when the input ramp ends — while the real aircraft peaks at
**133–166 ms**, 50–80 ms *after* the input stops. ⇒ The sim's pitch response is **too fast to peak**, not
missing dynamics. That is a much narrower and more tractable defect, and it is consistent with §6's
independent finding that the sim is systematically quicker than the real plant.

## ⚠️ Confidence, stated plainly

- **n is small**: only 5 of 8 cells began within 3 m/s of level, and the nose-up group that did is **n=2**.
  The direction and the presence of the CL break are solid; the 22.4% mean is not a precise number.
- Trim converges but not tightly — three cells still had 5–10 m/s of descent at the step.
- ⇒ The claim that survives is **qualitative**: the sim stalls, in the same direction, with a comparable
  rebound. The claim that does **not** yet survive is any quantitative match of damping or frequency.

## Three real bugs this harness found by being run

Each was invisible until the thing was actually executed at scale, and each is now fixed and commented:

1. **No trim ⇒ ground.** First run pitched to −34° and flew into the ground in 6.75 s. Open-loop step
   tests need what the pilot was silently providing.
2. **Trim clamp starved trim.** Clamping the trim datum to `0.5 − |amplitude|` left 0.15 of authority at
   large amplitude — not enough to hold level, so it dove during *settle*. ⇒ Trim now has priority and the
   **pulse** is reduced to the remaining room (and says so).
3. ⭐ **The trim integrator had the sign backwards.** crrcsim's elevator is inverted (`pitchCmd =
   −2·elevator`), so nose-up is **negative** elevator. The integrator added where it should subtract,
   driving nose-down while the aircraft sank. Symptom: 5–12 m/s of descent still present after a 4 s trim.
   ⚠️ Same inverted convention that silently cancelled the pitch average in `step_response.py` — **that
   sign has now caused two separate defects; treat it as a known trap.**

## ⇒ Where this leaves the sim-fidelity list, revised

| finding | status | priority |
|---|---|---|
| sim compute latency 30 ms vs measured **9.9 ms** | measured, unambiguous | ⭐ **high — do it** |
| NN sensor filtering un-modelled (accel ~21 ms, gyro ~6.4 ms) | measured | ⭐ high, and cheap |
| pitch **peak timing** 85 ms sim vs 133–166 ms real | measured, both regimes | medium — narrow defect |
| roll gain/rise | within ~17% / ~10% | low — leave it |
| ~~"sim lacks pitch dynamics"~~ | ⛔ **retracted twice; it stalls** | — |

⭐ **The second airframe is now the highest-value single data point.** This whole thread rests on `n = 1`
aircraft with a known-asymmetric wing and a nose-heavy CG, and the stall cycle is exactly the kind of
behaviour that is an *article* property rather than a *design* property. Different planform and different
servos would separate those two in one flight.
