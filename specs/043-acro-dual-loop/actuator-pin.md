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
