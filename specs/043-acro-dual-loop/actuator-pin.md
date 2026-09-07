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
