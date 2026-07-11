# Contract: Bench Validation (FR-002 observational span + FR-011 cadence soak)

**Scope**: the pre-flight gates. Per clarification 2026-07-10, verification is OBSERVATIONAL (the
stationary bench runs a real engaged span; the recorded log is reviewed) — there is no numeric
replay-parity harness.

## FR-002 — observational span (per firmware image + weight set)

Stationary bench, INAV live, full engaged span run to completion. The downloaded log MUST show:

1. Boot banner reports the regenerated topology (37 in / 2051 w) + firmware_id/weight_id matching
   the intended candidate.
2. EngageHeader present per span with the re-centered arena: origin ≈ bench position,
   `ceiling_Z = z_engage − 47.5`, `floor_Z = z_engage + 47.5` (pure ±K re-center — no
   minimum-elevation clamp this phase; bench z_engage ≈ 0 ⇒ floor_Z ≈ +47.5, ceiling_Z ≈ −47.5).
3. The generated path moves around the stationary craft: target-direction history + dist[6] evolve
   through the span; `recurrent_reset = 1` exactly on the first tick.
4. All 37 inputs + 3 outputs plausible every tick: unit-vector components in [−1,1], quat
   normalized, no NaN, no stuck outputs. Arena inputs read **center-of-arena** at engage — the
   pure ±K re-center puts the bench craft at band center both horizontally (radial boundary ≈ R
   away) and vertically (±47.5 m band centered on the bench). Off-center readings at engage
   indicate the re-centering is implemented wrong. (The limits are safety-only this phase;
   engaging too low in flight is operator error — undefined, not firmware-defended.)
5. Zero decoder errors; tick_counter contiguous.

## FR-011 — 20 Hz cadence soak

Full stack live (state fetch, NN eval every tick, RC send, binary logging): several consecutive
3–4 min engaged spans. The log's timing fields MUST show:

- Tick cadence 50 ms nominal; jitter bound REPORTED (the memo's bench section sets the number —
  research measured ±4.4 ms at 100 ms ticks as the prior).
- Zero tick overruns attributable to logging or eval (drop/coalesce counters zero or explained).
  The existing `loopStats` counters (ticks/overruns/resyncs/maxLate/avgLate) are the instrument —
  verified present in the span-summary EventRecord of the downloaded log (not console-only).
- Per-span MSP pipeline stats (fetch/eval/send) captured — these feed contracts/latency-memo.md §2.
- DWT cycle-count measurement of the unrolled eval recorded once per image (037 eval-cycle-harness
  design: `DWT->CYCCNT`, M1 shape).

## Pass/fail

Any NaN, stuck output, missing EngageHeader field, decoder error, or logging-induced overrun ⇒ FAIL
(fix before flight). The reviewed bench log is attached/referenced in outcome.md as the FR-002
record.
