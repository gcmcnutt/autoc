# 043 — checkpoint 2026-10-13 (after t5 and the fifth ACRO flight) — NOT the outcome; 043 continues

> ⚠️ Written as "outcome" on 2026-10-13 and **re-scoped the same day** (operator): *"sim pitch fidelity continues in 043 as we
> haven't really improved in 043 for the intent — smoother control. 043 should have a much better flight-proven exit."* So this is
> the mid-feature record; the outcome is written when a flight shows the pitch cycle reduced (tasks Phase 10e, T118). 044 M2
> follows that exit.

**Scope:** the NN commands **rotation rates** into INAV's 2 kHz ACRO loop instead of surface deflections at 20 Hz, to
remove the 2–5 Hz pitch/roll oscillation measured on the 041-t7 flight; one bake carries everything.
**State at this checkpoint: the architecture works and the aircraft tracks better than it ever has — and the pitch oscillation is
unchanged, because it was never the inner loop.** Five bakes (t1–t5) and five ACRO flights so far.
Spec of record: [spec.md](spec.md); the run records: [artifacts-t3](artifacts-t3/MANIFEST.md),
[artifacts-t4](artifacts-t4/MANIFEST.md), [artifacts-t5](artifacts-t5/MANIFEST.md).

---

## 1. Result

| | 041-t7 (flown 08-23, surfaces @ 20 Hz) | **043-t5 gen 488 (flown 10-11, rates @ 20 Hz → ACRO)** | |
|---|---|---|---|
| flight median tracking error | — (t3: 25–34 m) | **8–15 m** | ~3× tighter than t3 |
| flight `step_score` | — (t3: 0.06–0.08) | **0.16–0.34** | 3× |
| throttle +rail in flight | 68 % | **42–59 %** (t3 85–90) | halved vs t3 |
| roll railing in flight | — | **0.0–0.3 %** | |
| one-sided downwind bias | — | **gone** (+0.4 … +5 m; t3 +25–27 m) | calm days only |
| vertical estimator | 6 resets on t3 | **`navEPV` max 541 cm, zero steps** | first ACRO flight seen clean |
| loop health | 0 overruns | **0 gaps / overruns / resyncs over 3515 ticks** | |
| **pitch cycle** | 2.0–2.2 Hz | **2.0–2.3 Hz, ±200 °/s, ±7 g — unchanged** | ⛔ five flights, three genomes, two architectures |
| sim, trained regiment: `pctInStreak` / crashes | 38.3 % / 2.7 % | **51.1 % / 0/294** (t5 gen 800; t3 54.6 % on a calmer regiment) | |
| sim cold-start generalisation (tier2-random) | — | **144/144, score 245** (t3 122/144, 43) | |

⭐ **What improved, improved together**: tracking, crash rate, throttle discipline, estimator health and cold-start
generalisation. Those are the crash cost (t3), the wind envelope and the second random course (t4/t5), and crash-free
selection — not the rate architecture by itself, and not the smoothness objective.

⛔ **What did not**: the pitch cycle's amplitude, frequency and loop gain are the same as before 043 existed. SC-001 is
**not met on pitch**; it is met on roll (tick-to-tick roll flipping halved in the air, 5–10 Hz share 13–20 % vs 30–37 %).

## 2. What actually caused it — and what did not

1. **The oscillation is the outer loop, not the inner one.** The NN is a saturating relay (setpoint p5/p95 at the ±240 °/s cap,
   FF 98–100 % of the rate-loop output); the 2 kHz inner loop follows it faithfully. Moving the rate loop onto INAV removed a
   phase budget that was not the binding one. The −64 ms setpoint→gyro lag at 2 Hz is ZOH + actuator, as the spec budgeted.
2. **The sim does not exhibit the cycle, so no objective could price it.** Same genome, both worlds
   ([first-principles § 11](tempered-control-first-principles.md)): pitch-rate RMS 61–96 °/s broadband in the sim vs 124–153 °/s
   concentrated at 2 Hz in the air; the command at the cap 0–1.3 % of ticks vs rail-to-rail. Energy, excess rotation and
   crash cost each did what they could see — roll, throttle, altitude — and pitch was invisible.
3. **Why the sim lacks it — measured, not argued** ([proposal § 1](sim-pitch-fidelity-proposal.md)): the closed inner loop's
   frequency response agrees between sim and aircraft to 1 Hz in gain and to 2.5 Hz in phase; the aircraft then peaks |H| 1.6
   at 2.5 Hz and rolls off, the sim keeps rising to 1.54 at 6 Hz. The model file's short period is **6.9 Hz / ζ 0.81** against
   the aircraft's **~3.0 Hz / ζ 0.2–0.3**, and `I_yy = 0.0013 kg·m²` (a 5 cm pitch radius of gyration) is the first-order cause.
4. **The t4 lesson on objectives**: a standalone rotation case is won by a passive flyer (t4 stalled from gen 281); as a
   tie-break inside the tracking case (t5) it bit on ~half of selections and cut roll/yaw rotation 6–10 % — and made the pitch
   command ~24 % *more* bang-bang, because `Σ‖ω‖·dt` rewards dithering that the sim plant low-passes (MANIFEST-t5 § 7a).
5. **Not the cause**: vibration / accelerometer clipping (excluded in flight, `accClipCount` 0 with `accVib` at an all-time high),
   anything above 20 Hz but blade-pass (966 Hz log), compute latency (the sim carries *more* than the aircraft — one frame,
   tasks T111), sampling rate alone (same 20 Hz in both worlds; the sim does not cycle).

## 3. Sim-to-real

| axis | state at close |
|---|---|
| tracking | flight / sim per-path ratio 0.29 (Straight) … 0.65 (RandomB): the gap shrinks as the path gets harder — it lives in the dynamics, not the geometry |
| roll | close; the axis moved both worlds the same way |
| throttle | sim rail halved and flight rail halved |
| **pitch dynamics** | **the one structural gap** — mode placement (§ 2 item 3). Fixing it is the successor feature. |
| latency | sim applies NN commands one frame (50 ms) late by construction; the aircraft ~10 ms (T111) |
| entry | hand-off attitude is not a variation class; the first second of every span is out of distribution (T085/T086 deferred) |
| wind | the 0–8 m/s envelope reproduces the flight brittleness in sim (`pctInStreak` 60 → 27 % calm → 6+ m/s); windy-day flight on a t4/t5 genome still owed |

## 4. Verified, not assumed

- Determinism: `NN_ELITE_SAME` 800/800 on t3, 413/413 on t4, 800/800 on t5; eval tier0 bitwise on every flown genome; the
  t3 gate `−88026.619367` reproduced across the t4 and t5 code changes (axis off).
- Provenance: every flown genome re-extracted from S3 and hashed (`a5d097da…` t3, `1731d937…` t5 g488 — byte-identical);
  the one mis-label (gen9512 = gen **488**, not 512) caught and corrected in the record.
- Wind reset path audited to the Dryden integrator; the t4 stall was selection, not non-determinism.
- Unit slips found and fixed on both sides: flight gyro ÷10 in three scripts; sim gyro columns are rad/s ÷ 6.

## 5. Spec success criteria

| SC | status |
|---|---|
| SC-001 smoother in the air | **not met on pitch**; met on roll (flipping halved) |
| SC-001a magnitude stated | met — roll 5–10 Hz share 30–37 → 13–20 %; pitch 0 % change |
| SC-002 emulation divergence measured | met at close — the FRF and short-period numbers above |
| SC-003 tracking does not regress | met, 3× better |
| SC-004 trainability survives | met (t3, t5 trained; t4's failure was the objective, not the plant) |
| SC-005 041 agreement holds | partially re-verified (T071 clock-join routine; the formal cross-validation was not repeated) |
| SC-006 loop health | met |
| SC-008 / SC-013 variation regime legible, no dead sigma | **not completed** (T079/T080 → BACKLOG) |
| SC-009 IMU axes free when off | met (bitwise gates) |
| SC-010 actuator measured | pitch and roll step tests yes; servo bench step (T046) no |
| SC-011 arm's-length answered | met — D1 stands alone; the inner loop was not the lever |
| SC-012 zero command → zero rate | met (T043) |
| SC-014 flies sensibly, recovery is the policy's | met — five flights, all spans ended cleanly |

## 6. Carried forward

| item | to |
|---|---|
| **Pitch short period in the sim** (measure `I_yy` on both articles, fit `Cm_a`/`Cm_q`, honest sub-step latency, latency sweep, open-loop replay, gen 488 on article 2) → **t6 bake on the honest plant → a flight that shows the cycle reduced** | **043 Phase 10e** — this IS the feature's exit; M2 inherits the plant |
| entry dither / heading / path library; six random courses | BACKLOG (t6+ content) |
| command-side smoothness measures (tie-break only, after the plant is honest); relay-amplitude flight | BACKLOG |
| windy-day flight on g488; `GPS_ONLY` decision; `flash_erase` discipline | BACKLOG |
| observability audit (SC-008/013), FR-056 n = 2 review, INAV phase-delay params, roll dead-band, pitot | BACKLOG |
| M2 tracking and its two gates | **044** (after 043's flight-proven exit) |
