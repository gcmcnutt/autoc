# 043-t2 — flight analysis (2026-09-05)

Genome **gen 800** (`weight_id=3af8e3ab787b75a5`), first flight of the ACRO dual-loop architecture.
Logs: `flight-results/flight-20260905/`. Four engagements, 1011 ticks, 0 gaps, 0 overruns, 0 drops.

⭐ **Headline: the architecture worked; a frame bug in the engage prefill fed the policy 800 ms of
6–8× out-of-scale input at the start of every engagement, and the sim does not have it.**

---

## 1. ⛔ ROOT CAUSE — the engage history prefill uses the wrong frame

At engage the xiao pre-fills all six NN history slots so the policy starts with consistent geometry
(`AircraftState::resetHistory`, `xiao/src/msplink.cpp:864`). It is being called **one MSP cycle too
early**:

| line | what happens |
|---|---|
| `msplink.cpp:768` | `test_origin_offset` = absolute INAV position, captured at engage |
| `msplink.cpp:864` | `resetHistory(flight_path[0].start, tangent)` — **fires here** |
| `msplink.cpp:1087` | `position_rel = position_raw - test_origin_offset` — **only on the NEXT MSP update** |

So at the moment of the prefill, `aircraft_state.position` still holds the **absolute** NED position
(≈ the origin) while `flight_path[0].start` is **arena-relative** (≈ 0). `resetHistory` computes
`craftToTarget = targetPos − position`, which evaluates to **−origin**.

⭐ The prediction is exact — the stale distance should equal ‖arena origin‖:

| run | arena origin (N,E,D) | ‖origin‖ | observed `dist_0` |
|---|---|---:|---:|
| flight span 1 | (143.4, −25.7, −70.0) | 161.6 | **162.2** |
| flight span 2 | (187.8, −29.5, −90.8) | 210.7 | **208.0** |
| flight span 3 | (157.4, −36.4, −84.9) | 182.5 | **183.2** |
| flight span 4 | (162.8, −7.6, −95.8) | 189.0 | **189.5** |
| bench 2026-09-04 | (−3.76, 4.01, 0.48) | 5.5 | **5.5** |

The residual is just the craft's own offset from the origin at engage. This is not a hypothesis.

**Consequence.** `kTargetDistScale_m = 26`, so the policy is fed `dist ≈ 6.2–8.0` in normalised units
against a trained range of roughly 0–2, in **5 of the 6 history slots**, together with the matching stale
`target_x/y/z` direction cosines — until the ring flushes at the 800 ms lag, i.e. **16 ticks**.
`closing_rate` is corrupted for the first ~2 ticks by the same mechanism.

⛔ **The network is recurrent (`45→32→16r→3`).** `recurrent_reset` fires correctly at tick 0, so the hidden
state starts clean — and is then driven for 16 ticks by inputs it has never seen. The corruption does not
end when the buffer flushes.

### ⚠️ Why the bench could not catch this

The magnitude of the garbage is ‖origin‖, and the bench sits **5.5 m** from the NED datum while the field
puts the aircraft **160–210 m** from it. The bench reproduces the bug *structurally* — the same fill
pattern is visible in both bench logs — but at an amplitude that is inside the trained range and therefore
harmless. ⭐ **A static bench near the datum cannot surface this class of fault. That is the lesson, not
that the bench was run badly.**

### The sim is correct, which is why training never saw it

`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp:754` assigns
`aircraftState = initialState` **before** the prefill, with a comment saying exactly why, and
`virtualInitialPos = initialPos − pathOriginOffset` is already arena-relative. The sim prefills all six
slots with the correct geometry and `closing_rate = 0`.

⇒ **This is a pure sim↔real divergence in the input vector**, not a policy failure and not a physics
mismatch. The policy was asked a question it was never trained on, four times.

---

## 2. Did ACRO work? — YES, unambiguously

| check | result |
|---|---|
| Mode, all four spans | **100.0% / 99.7% / 100.0% / 100.0%** `ARM\|MSPRCOVERRIDE`, no MANUAL. ACRO held for every engaged tick. |
| Clock join | **−944.5 ppm**, 1011/1011 ticks matched within 30 ms, median skew 4.2 ms. ⭐ Cross-validates 041-t7's −970 ppm. |
| Yaw silence | `axisRate[2]` = `axisF[2]` = **0.0** in all four spans (FR-018). |
| Loop health | 0 overruns, 0 resyncs, 0 drops, maxLate 15–25 ms across 1011 ticks (SC-006 ✅). |

### The rate loop tracks — with 68–84 ms of lag and about half the gain

Measured on the blackbox at its native 59 Hz (no join needed — both signals are INAV's own), sweeping lag:

| axis | best lag | r @ best | slope | r @ zero lag |
|---|---:|---:|---:|---:|
| ROLL | **84 ms** | +0.72 … +0.79 | 0.51 … 0.61 | +0.10 … +0.57 |
| PITCH | **68 ms** | +0.46 … +0.77 | 0.26 … 0.67 | +0.36 … +0.72 |

⚠️ Two things to sit with. The loop **delivers only ~50–60% of the commanded rate**, and the residual
delay is **68–84 ms** — the same order as the 81.6 ms phase budget 043 set out to defeat. Moving the
stabilisation inboard did not remove the phase; the NN command still arrives over a 20 Hz ZOH, and the
airframe/actuator lag is untouched. ⭐ This is the number that decides whether 043's thesis is complete.

### SC-001 — roll-rate spectrum: real improvement, not a fix

| roll-rate power | 0–1 Hz | 1–3 Hz | 3–5 Hz | 5–10 Hz | mean \|p\| |
|---|---:|---:|---:|---:|---:|
| 041-t7 **real** (baseline) | 30.6% | — | **30.1%** | 7.4% | 87.3 |
| 041-t7 **sim** (target) | 55.9% | — | 12.6% | 2.3% | 88.4 |
| **043-t2 real, all spans** | 40.2% | 33.3% | **17.7%** | 8.2% | 86.6 |
| per span | | | 30.3 / 33.6 / **15.2** / **5.7**% | | |

⭐ **3–5 Hz fell 30.1% → 17.7%**, about halfway to sim, at matched rate amplitude (86.6 vs 87.3 — the
amplitude match 041 established still holds). ⚠️ But **5–10 Hz did not improve** (7.4% → 8.2%), and the
per-span spread is enormous: spans 1–2 are no better than 041-t7, spans 3–4 beat the sim target. The
improvement is not uniform, and spans 1–2 are the two most affected by the prefill bug.

⛔ **Pitch is the new problem.** 67.2% of pitch-rate power now sits at **1–3 Hz** with RMS **110.7 °/s** —
against 039's references of 24 °/s for INAV acro *"on a rail"*, 50 for pilot MANUAL, and 141 for NN-direct.
The oscillation did not disappear; in pitch it moved **down** in frequency and stayed large.

---

## 3. Sim vs real dynamics — NOT similar (SC-002)

| measure | sim gen 800 | real | ratio |
|---|---:|---:|---:|
| body accel \|a\| median | 1.49 g | **2.33 g** | **1.56×** |
| body accel p95 | 4.19 g | 5.61 g | 1.34× |
| mean \|p\| (roll rate) | 97.9 °/s | 89.4 | 0.91× |
| mean \|q\| (pitch rate) | 65.1 °/s | **91.3** | **1.40×** |
| mean \|r\| (yaw rate) | 52.3 °/s | 29.9 | 0.57× |

⚠️ **Body acceleration is 1.56× sim** — the direct descendant of 041-t7's *"roll acceleration is 1.66× sim
at matching rate amplitude"*. ⛔ **043 did not close that gap.** Note this is the opposite sign to the
first read at the field: the real aircraft pulls **more** g than the sim, not less.

Pitch rate is 1.4× sim while roll is 0.9× and yaw 0.6× — the divergence is now **concentrated in pitch**,
which is where the open-loop stability question lives (research.md Finding 1a).

---

## 4. Tracking — clearly worse than sim (SC-003 ✗)

| distance to rabbit | median | mean | <5 m | <10 m | <20 m |
|---|---:|---:|---:|---:|---:|
| sim gen 800 | 4.1 m | 6.3 m | **60.7%** | 84.7% | 95.5% |
| **real** | 11.1 m | 16.2 m | **16.2%** | 44.3% | 81.0% |

Close-tracking occupancy collapsed **60.7% → 16.2%**. Per-span the craft starts on the rabbit and then
falls behind monotonically — median distance by tick window:

| span | ticks 0–15 | 16–40 | 41+ |
|---|---:|---:|---:|
| 1 | 3.3 | 13.7 | 21.9 |
| 2 | 3.1 | 12.8 | 14.9 |
| 3 | 1.6 | 8.0 | 9.3 |
| 4 | 2.0 | 6.8 | 32.1 |

⭐ Ticks 0–15 are exactly the corrupted window; the craft is close there only because the rabbit *starts*
on it. It never recovers the gap afterwards.

## 5. Throttle — the modulation is real and it is not in the sim

| | sign-changes | % railed |
|---|---:|---:|
| sim gen 800 (median over 294 scenarios) | 1.89 Hz | **22%** |
| **real, whole flight** | **3.96 Hz** | **73%** |
| span 1 / 2 / 3 / 4 | 1.21 / 2.41 / **6.51** / 1.31 Hz | 85 / 79 / 62 / 83% |
| bench 2026-09-04 (static) | 0.23 Hz | 85% |

Real throttle switches **2× faster** than sim and is railed **3.3×** as often; span 3 reaches **6.51 Hz**.
⭐ This is the same *class* of finding as 041's roll oscillation — a channel that is smooth in sim and
chattering in the air — now in throttle. It is a genuine sim↔real divergence and is not explained by the
prefill bug alone (it persists long past the 800 ms window).

## 6. Span 1 — the "idle dive", explained

66 ticks, 3.3 s, pilot took it back (`DISENGAGE_REASON=1`, servo switch). Throttle **74% at the idle
rail**, pitch mean −0.59 with a sustained nose-down hold, descent reaching **8.6 m/s**, airspeed decaying
21.3 → 14.6 m/s.

⭐ It engaged at **18.5 m/s while climbing at 2.9 m/s** — already fast against the 13 m/s trained cruise —
and then took 800 ms of `dist ≈ 6.2` garbage into a freshly-reset recurrent state. The response is the
documented OOD nose-down rail (`specs/BACKLOG.md`), the same signature the 2026-09-04 bench produced and
the same one that kills 218 of 294 `random` scenarios. **The pilot's read was correct.**

## 7. Span 4 discontinuity — INAV vertical estimator reset, NOT GPS

At t = 267.41 s, `pos_d` steps **−70.50 → −24.12 m** (46.4 m) and `vel_d` jumps −4.3 → **+20.1 m/s**,
while `pos_n`/`pos_e` step a normal 1 m. Horizontal is untouched; GPS is fine.

The blackbox shows the cause building for seconds beforehand: `navPos[2]` climbing while `BaroAlt` falls,
with `navEPV` growing monotonically **398 → 998** — then at the step, `navEPV` snaps back to **390**. That
is the estimator rejecting its own vertical state and re-initialising to the measurement.

| reset | t (s) | step | EPV before → after | navPos−baro gap |
|---|---:|---:|---:|---:|
| 1 | 265.51 | +23.9 m | 998 → 369 | +27.2 m |
| 2 | 267.37 | **−45.5 m** | 994 → 390 | **+85.6 m** |
| 3 | 277.80 | +13.1 m | 999 → 353 | +0.3 m |

⛔ **The vertical estimator was unhealthy for the whole flight, not just span 4.** `navPos[2] − BaroAlt` by
span: **+0.5 / +11.2 / +4.3 / +3.5 m** median, with maxima **−11.0 / +27.4 / +22.2 / +85.8 m**. A healthy
estimator tracks baro to a few metres. `accVib` is **2039** in span 1 and **4340–4596** in spans 2–4.

⚠️ This corrupts the NN's altitude-derived inputs directly: `specific_energy` is scaled by
`kEnergyScale_m = 145`, so a 27 m altitude error is **0.19** in normalised units, and `dist_to_boundary`
and the engage-arena floor/ceiling geometry ride on the same estimate. Two of the three resets land
**inside span 4**.

⭐ Suspect ordering: high `accVib` → accelerometer fusion drives vertical velocity drift → EPV grows →
reset. The 2026-09-04 accel recalibration left `ins_gravity_cmss` at **972.092** against a true ~979,
still ~0.7% low, which biases exactly this integration.

---

## 8. What this flight does and does not tell us

**Established.** ACRO engages, holds, and releases correctly; the NN commands INAV's rate loop end to end;
yaw stays silent; the 20 Hz loop is healthy; the clock-join method reproduces at −944.5 ppm; 3–5 Hz roll
power dropped meaningfully.

⛔ **Not established, because the input vector was corrupted at every engagement.** No conclusion about the
*policy's* competence — tracking, throttle discipline, or the OOD rail — is safe from this flight. The
comparison that SC-002 and SC-003 need is against a flight where the prefill is right.

**Ordered by expected value:**

1. ⭐ **Fix the prefill frame** (`msplink.cpp`: re-express `aircraft_state.position` into the engage frame
   before line 864, mirroring CRRCSim's ordering). One-line class of change; it invalidates nothing else.
2. **Re-fly.** Everything in §3–§5 needs re-measuring against a clean engage before it can be attributed.
3. **Investigate the vertical estimator** — `accVib` 4300+ and an 85 m excursion are their own problem and
   feed the NN directly.
4. Only then judge the 043 thesis on §2's 68–84 ms residual lag and 0.5–0.6 loop gain.

⚠️ **Add a non-static engage check to the bench protocol.** The bench passed every gate and could not have
caught this, because the fault scales with distance from the NED datum.
