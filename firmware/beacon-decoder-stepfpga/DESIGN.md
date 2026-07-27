# Beacon receiver — acquisition & tracking design

The locked-in design for the 031 beacon receiver's signal chain, as developed and validated on the in-FPGA
**correlator-sim** harness (`experiments/s3.v`). This is the "why" companion to the feature reference
([`SIM-FEATURES.md`](SIM-FEATURES.md)) and the milestone plan ([`correlator-sim-plan.md`](correlator-sim-plan.md)).

## 1. The problem

A camera-tracked aircraft carries optical beacons (Gold-coded LED flashes). The ground receiver must **acquire and
hold a confident lock** on each beacon under a hostile, *dynamic* channel:

- **Unsynchronized clocks.** Emitter (battery RC oscillator, ±5 %) and receiver (12 MHz xtal) share no clock. The
  beacon chip rate drifts vs the sample clock — acquisition must tolerate the offset and *track* it.
- **Wildly variable amplitude.** Range, aspect, and lens vignetting swing the return over orders of magnitude.
  Detection must not depend on brightness.
- **Frequent dropouts.** Sun, reflections, ground clutter, occlusion, and aircraft attitude blank the signal for
  spans from a few chips to seconds. The system must coast short gaps and re-acquire fast after long ones.
- **Multiple beacons on one pixel (CDMA).** Two codes can sum into one detector; each must be separated.
- **Noise + bit errors.** Photon/Johnson noise and the occasional corrupted chip must be ridden through.
- **Low latency.** This is a *control loop* (≈20 Hz). Re-acquire that takes many control cycles is a miss. The goal
  is graded confidence that arrives early ("candidate") and firms up ("hard lock").

The single guiding objective: **strong lock under dropouts, acquired with the lowest latency the physics allows.**

## 2. Signal model (decided)

| Parameter | Value | Why |
|---|---|---|
| Code | **Gold preferred pair**, N=31 active (N=63 fits at 85 % LUT) | bounded cross-corr for CDMA; N is a localparam |
| Chip rate | **200 Hz** | within the camera-frame budget; ties to FR sampling |
| Oversample | **2.4 samples/chip** (480 Hz) | camera cadence; stays above Nyquist with unsynced clocks |
| Window `L` | `round(2.4·N)` = 74 (N=31) | one full code period of matched filtering |

**N=31 vs N=63 (measured).** Latency scales linearly with N at fixed chip rate: cold acquire 308 ms (N=31) vs
629 ms (N=63); warm re-acquire 154 vs 315 ms. N=63 buys +3 dB processing gain and a lower CDMA floor (0.27 vs
0.29) but costs 2× wallclock and tightens the cold-skew cliff to ~±5 %. **N=31 is the active build** for the
latency-by-confidence phase; N=63 is the robustness option once a stable emitter clock removes the skew penalty.

## 3. The algorithm — a lock-defense pipeline

Each stage defends the lock against a specific failure mode. (Concise version + the lifecycle diagram live in
[`SIM-FEATURES.md` §6](SIM-FEATURES.md).)

1. **DC-removal + signed matched filter.** A slow IIR mean (τ≈256 samples) is subtracted; the window is correlated
   against the ±1 template by signed-accumulate (no multiplier). Two passes share one window/energy → both codes.
2. **AGC quality `q = |corr|/energy` (0–9).** *Decouples "is it the code?" from "how bright?"* — the single most
   important decision. A match ratios to ~9 regardless of amplitude; a wrong/absent code sits at the cross-corr
   floor (~3). `|corr|` makes bit-order/polarity irrelevant. This is what makes a *weak* return still confirm and a
   raised noise floor still hold.
3. **Lock ladder (FSM).** SEARCH → ACQUIRE (`MINLOCK=2` consecutive good periods) → LOCK → HOLD → re-acquire.
   `GOOD=5`. The MINLOCK requirement is a strong noise-rejector (pure noise hits GOOD <1 % of frames, never twice
   running → ~0 false confirms).
4. **Closed-loop DPLL (`Leff = L + slip`).** Once locked, an IIR of the per-period peak-phase slip estimates the
   beacon's true chip rate and *stretches the template's chip-advance* (Bresenham) to match it → the long matched
   filter stays coherent under ±5 % skew, retaining full processing gain instead of smearing it away.
5. **Fast-acquire snap (frequency aiding).** The lock is fast (~0.5 s) but the slip IIR's *frequency pull-in* across
   the RC-osc offset used to leave q yellow for ~5 s (N=31)/~10 s (N=63). Fix: on a **cold** lock edge, **snap** the
   slip straight to its steady-state estimate (`slip ≈ dlt·2^SLIP_SH`, HW-verified) → cold full-quality in <1 s
   across ±5 %. Conceptually A-GPS / SnapTrack: seed the rate, skip the search. Warm re-locks keep the held slip.
6. **HOLD coast (`HOLDMAX=2`).** Ride 2 bad periods without dropping → absorbs short burst dropouts outright
   (measured: continuous blanking up to ~1.5 code words is held with no drop).
7. **Frequency flywheel.** The rate estimate is **frozen through the outage** (coast window ~10 s wallclock). A
   re-acquire is therefore *phase-only* — full quality on the first good period, no cold search, no pull-in.
   **Coast is set by oscillator stability, not code length**: a 20–50 ppm emitter xtal would extend it far beyond
   10 s *and* remove the cold-skew cliff — strongly motivating a crystal over the bare RC oscillator.
8. **On-chip recovery counter.** Measures signal-return→confirmed-lock in samples (USB can't sample fast). The
   `Z`/`K` commands force true-cold / burst conditions for bit-exact latency measurement.

The shipped confidence is the **3-level ladder** (no_lock → tentative → confirmed). A 4th, lower-latency
"candidate" tier (partial/progressive correlation, A4d-2) was investigated and **deferred** — see §5.

## 4. Key decisions & tradeoffs (locked)

- **AGC ratio, not absolute threshold** — the only way to be amplitude-independent across the camera's dynamic range.
- **Closed-loop template stretch, not multi-hypothesis bank** — one correlator tracks the rate after lock; cheaper
  than N parallel rate hypotheses, at the cost of needing an initial lock first (see cold-skew cliff).
- **Snap fast-acquire, not a faster IIR** — the IIR's time constant is coupled to its output scale, so it can't just
  be sped up; pre-loading the integrator (snap) decouples acquire-speed from track-bandwidth.
- **Flywheel coast is an oscillator-stability budget**, not a fixed constant — documented as such so the emitter
  clock choice (RC vs xtal) is an explicit lever on re-acquire performance.
- **No silent fallbacks** (constitution VII): thresholds, clamps, and code length are explicit localparams; the host
  parser fails loud on malformed frames.

## 5. Known limits / open items

- **Partial/progressive candidate (A4d-2) — investigated, deferred.** Goal: a ½-code-word "candidate" tier earlier
  than tentative. Two FPGA designs were built and acceptance-measured: (a) per-sample "any strong half-correlation
  in the period" → fired on the max over 74 alignments → ~100 % false on noise; (b) the ½-window correlation at
  each code's own full-peak phase (noise-robust, code-specific) → better but still **50–70 % false-candidate on
  noise/wrong-code**. Root cause is fundamental at N=31: the ½-window has **3 dB less processing gain** *and* a
  **partial Gold code loses the full code's cross-correlation bound** (the {−9,−1,7} bound only holds over all 31
  chips). Any threshold low enough to lead the ladder also admits noise/cross-code; any threshold high enough to
  reject them fires no earlier than tentative. **Decision: ship the 3-level ladder; revisit the candidate at N=63**,
  where the half is a full 31-chip code that regains the cross-corr bound — or with a separate energy-only
  "coded-signal-present" gate. The existing latency is already strong (cold 250 ms, warm 154 ms, re-acq 25 ms).
- **Cold-skew cliff ~±5 % (worse at N=63).** The initial lock runs the nominal template (DPLL only corrects *after*
  lock), so a too-large offset smears the first correlation below threshold. Fix paths: multi-`Leff` cold search, or
  a stable emitter clock (preferred — also extends the flywheel).
- **Two equal-power skewed codes (CDMA corner).** With two beacons of equal power on one pixel, AGC energy-sharing
  drops each to q≈4–6; one can sag below lock. Eased by unequal power (the normal case) and by N=63's lower floor.
  Follow-up: per-code interference cancellation or power-aware thresholding.
- **S7 — real MCP3201.** The soft SPI master + bit-exact model mean the swap to real photons is just re-pointing the
  sample input.

## 6. Acceptance test

`host/acceptance.py` drives the full battery via `host/scenario.ps1` and scores each against a pass criterion
(transient flush/serial flakiness is retried up to 3×). Telemetry lock ladder: **0 no_lock · 1 tentative · 2 confirmed.**

| dimension | scenario | criterion | measured (N=31) |
|---|---|---|---|
| Time-to-signal | code A, cold (flush) | confirmed ≤ 600 ms | **250 ms** |
| Clock skew | A and B, ±5 % | confirmed ≤ 900 ms | 150–300 ms, all 4 |
| Variable amplitude | low / high magnitude | confirmed ≥ 60 % (AGC) | 96 % / 96 % |
| Noise | A + noise | confirmed ≥ 60 % | 97 % |
| Bit errors | 1-bit / 2-bit injection | confirmed ≥ 50 % | 95 % / 97 % |
| Weak signal | A + weak | confirmed ≥ 50 % | 95 % |
| Burst dropout (hold) | blank 1 code word | lock never drops | floor = confirmed |
| Burst dropout (re-acq) | blank 3 code words | recovers ≥ 50 % | 88 % |
| Re-acquire after outage | 8 s blackout, restore | confirmed ≤ 500 ms (flywheel) | **25 ms** |
| Two-code CDMA | A + skewed B | dominant confirmed ≥ 60 % | 96 % (capture: A 96 / B 56) |
| No false lock | noise only | false-confirm ≤ 2 % | 0 % |
| No false lock (wrong code) | B only, watch A | A false-confirm ≤ 2 % | 0 % |

**17/17 pass** at the locked design (3-level ladder, N=31). Re-run any time with `python3 host/acceptance.py`.
