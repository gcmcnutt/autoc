# The emitter is not at 120 Hz, q was saturating, and T081/T082's premises need revising (2026-08-29/30)

Session goal was operator's list: **(1) the `energy` saturation, (2) T081 sync-first, (3) T082 straddler
weighting** — "from playback data first if possible". (1) is done and measured. (2) and (3) both turned
out to rest on premises that the playback data contradicts, which is the more useful outcome.

All numbers are replay against `/data/{static_ir,pend3,pend_ir}.bcnr` with fiducial/geometric truth.

## 1. The correlator was saturating — q = 1.00 was mostly an artifact

Both accumulators were int32 with saturating clamps, and at the real operating point both railed:

| | before | after (int64) |
|---|---|---|
| `corr` railed at INT32_MAX | **73 %** of slot-ticks | 0 |
| `energy` railed | 80 % | 0 |
| both railed together | 73 % | — |
| `q == 1.000` | **75 %** | 22 % |

When both rail, `q = |corr|·256/energy = 2^31·256/2^31 = 256` exactly. **98 % of the q = 1.00 readings
were that artifact.** True maxima once widened: `|corr|` 2.15e9, `energy` 4.19e9 — the latter nearly
twice what an int32 holds. q gates promotion, HOLD entry and the ladder, so it was lying to three
consumers, and it is why q flipped 0.87 / 0.51 / 1.00 on consecutive ticks of a STATIC target.

Result: static `MEASURED` 80 → 98 %; `pend3` fixes 2593 → 3479 (+34 %), 3–5 °/s 52 → 81 %.

### and it unblocked the gate T085 could not find

The 64-bit fix raised the false-fix share (pend_ir 0.1 → 2.2 %). Those fixes are **not a competing
source** — scattered rather than clustered on a lamp, all in the high-rate part of the swing, median
45 M2 px off, and q p50 **0.45**. They are the tracker still asserting `MEASURED_FIX` after losing a fast
target. `measured_fix` never looked at q at all.

With honest q the separation is clean — on-beacon q p05 **0.86**, off-target q p95 **0.67** — so the new
`[bank] q_fix` (0.75) gives **zero** off-target fixes on both clips for 1.5 % of fixes on pend3.
**T085 was right about the data it had**: q railed to 1.00 on 75 % of slot-ticks, so the separation
genuinely was not visible. It only appears once the correlator stops saturating.

## 2. THE EMITTER IS AT ~120.75 Hz, NOT 120.0 — and the design's "fail loud" did not fire

Measured **two independent ways**, agreeing to 0.002 Hz:

- **Raw pixels, no tracker involved.** Beacon-window time series from `static_ir.bcnr`, swept against the
  Gold-B template over chip rates 119.0–121.5: best at **120.794 Hz**. At 120.0 the 30 s correlation is
  **3 % of** its value at the true rate.
- **The tracker's own reported phase.** Unwrapped `chip_phase` is a clean linear ramp, and its slope is
  the rate error:

| clip | date | drift | implied emitter | residual |
|---|---|---|---|---|
| `pend_ir` | 08-28 | +0.7433 chip/s | **120.743 Hz** | 0.36 chip |
| `pend3` | 08-30 | +0.7183 chip/s | **120.718 Hz** | 0.31 chip |
| `static_ir` | 08-30 | +0.7918 chip/s | **120.792 Hz** | 0.27 chip |

So the pod sits **~6200 ppm** above the configured 120.0, stable to a few hundred ppm across two days.
The bench journal's "119.940 Hz measured after the 2026-08-20 reflash" no longer describes this pod.

**The `[code]` block's stated safety property failed.** It says a wrong-rate pod "must present as NO LOCK,
loudly, so the answer is go look at the pod rather than a derated lock that quietly measures the wrong
operating point". It presented as a *perfectly healthy* lock — 99 % LOCK, q p50 1.00 — because a 258 ms
window only accrues 0.20 chip of drift. The derated lock is exactly what happened.

### where the rate error actually costs, measured

⚠️ **Correcting an overstatement made during the session.** The "3 % of available gain" above is the
correlation over a *30 s* span; it is NOT what the tracker loses, because the tracker integrates 258 ms.
Swept on `pend3`, the corrected rate changes almost nothing at the current window:

    chip_hz 120.0  3428 fixes    120.70  3247    120.79  3420    120.85  3436

The cost appears when the window gets long — which is precisely what long-range work needs:

| | K = 31 (258 ms) | K = 124 (1.03 s) |
|---|---|---|
| rate 120.0 | LOCK 99 %, q 1.00 | **LOCK 75 %, MEASURED 43 %, q 0.78** |
| rate 120.79 | LOCK 99 %, q 1.00 | LOCK 82 %, MEASURED 46 %, q 0.83 |

`integration_max_chips = 124` has therefore never been usable, and the AGC's "weak → double the window"
lever has been broken the whole time. Correcting the rate recovers part of it (75 → 82 %) but not all, so
**there is a second mechanism** — see §4.

## 3. T081's enabling invariant does not currently hold

Sync-first rests on: *"phase survives ~65 s blind (single-rate platform, pod 64 ppm off nominal), so a
beacon leaving and re-entering the field costs ONE 0.39 ms correlation."*

At the **measured** 6200 ppm rather than the assumed 64 ppm, half-chip drift arrives in **0.7 s, not
65 s** — a factor of ~93. A beacon out of frame for one second returns with an unusable phase, so
"re-detect with a known phase" is not available.

This does not kill T081; it **reorders it**. The rate must be tracked first (T083) — and T083's own stated
precondition ("sim-first against golden clips, never live tuning") is now not just met but *equipped*: the
unwrapped phase slope is a working rate discriminator with a residual of 0.27–0.36 chip, measured on three
clips across two days, and it cross-validates against a tracker-independent pixel sweep.

**Operator, 2026-08-30**: *"If we assume the clocks are reasonably stable, then we continuously measure
period of emitter."* The table above is the evidence that they are (a few hundred ppm across two days)
and that the measurement is cheap.

## 4. T082's premise does not hold at this exposure — and the real finding is bigger

T082 says 42 % of frames straddle a chip boundary, worth 2.34 dB. That is true **only if a frame
integrates over its whole period**. It does not:

| | measured on `pend3` |
|---|---|
| exposure | **45 µs** |
| frame period | 3474 µs |
| chip period | 8333 µs |
| exposure as a fraction of a chip | **0.54 %** |
| frames whose exposure window crosses a chip boundary | **0.54 %** |
| what it would be if exposure == frame period | 41.7 % |

The 41.7 % is exactly T082's number, reproduced — from the assumption it made. At 45 µs a frame is
effectively an instantaneous sample and lands in one chip 99.5 % of the time; misassigned light is
~0.27 %, worth about **0.01 dB**, not 2.34. **Do not build T082 at this operating point.**

**The much larger fact it exposes: we are discarding 98.7 % of the available photons.** Exposure is 1.3 %
of the frame period. That is an enormous unused SNR lever for the far-field case, and the outdoor work
already showed sky is nearly black through the 850 nm bandpass, so a long exposure may be affordable
outdoors. T082 becomes real *exactly when* that lever is used — the two are the same task.

**Operator's related point, unmeasured but now specified**: *"since we only integrate a small fraction of
the frame time that is another edge variable that can resonate with 2.4 frames per chip."* At 2.3992
frames/chip the sampling instant walks through the chip on a 12-frame / 5-chip cycle, so each chip is
sampled at a systematically different sub-chip position rather than uniformly. That is a candidate for the
**second mechanism** breaking K = 124 in §2, and it is testable offline on these clips.

## What to do next, in the order the evidence now supports

1. **T083 — track the emitter period.** Promoted from "someday, carefully" to the gating item: it is what
   makes long integration usable and what T081 needs. The discriminator exists and is validated.
   Interim: set `chip_hz_nominal = 120.75` so the constant is at least in the right place.
2. **Measure the sub-chip sampling-phase effect** (§4) on `static_ir` — it is the leading explanation for
   why K = 124 stays broken even at the corrected rate.
3. **Exposure as a range lever**, with T082 folded in, since straddling only becomes real there.
4. **T081** after (1), when its invariant is true.

---

# T083 built and measured — the loop converges, and it does NOT unlock long integration (2026-08-30)

Operator: *"is this setup told the chip rate or does it discover and continually update its pll?"* It was
**told**, and it never updated: `acquire_next_rate_q8()` returned the config constant unconditionally, and
the DPLL's rate half was parked with only the phase half running. The phase loop then *masked* the rate
error by re-indexing a chip whenever it drifted — which is exactly the +0.718 chip/s ramp measured above.
A drifting pod was structurally invisible.

## The design, and why the two 2026-08-19 attempts diverged

They adjusted `chip_hz` alone. `corr_chip_at()` computes `chips = (t - epoch) * hz`, so changing `hz`
rebases the chip index of **every past sample**: at `t - epoch` = 240 s, a 0.01 Hz nudge moves the index
by 2.4 chips and relabels the whole bin ring at once. That is a structural trap, not a gain-tuning
problem, and it is why "naive dhz walked 115→109" and the repair "sprayed 112..129".

`track_retune()` moves rate and epoch **together**, chosen so `chip_at(now)` is invariant:

    epoch_new = now - chip_now * 1e6 / hz_new

Past samples then shift only by what the correction itself implies over the window (a 0.02 Hz change over
258 ms is 0.005 chip) instead of by the whole epoch-to-now lever.

The estimator is **receiver-global** — one emitter, one clock — so it lives in the Engine, not in Track.
That is also the groundwork T081 needs. It is deliberately slow: 5 s accumulation, 0.02 Hz deadband,
0.25 Hz slew cap. Over 5 s the true drift is 3.6 chips against 0.3 chip of noise, so a windowed estimate
has the SNR a per-tick one never had.

## It converges, and stays bounded

| clip | residual drift after t = 30 s | was |
|---|---|---|
| `pend3` | **+0.0257 chip/s** | +0.718 |
| `pend_ir` | **+0.0118 chip/s** | +0.743 |

A **28–60× reduction**, with `chip_hz` staying inside 120.5–121.0 across every clip. Half-chip drift now
takes **19–42 s** instead of 0.7 s.

## What it buys, honestly: almost nothing today, and the prerequisite for tomorrow

`pend3`, on-beacon fixes:

| | Kmax = 31 | Kmax = 124 |
|---|---|---|
| `rate_track = 0` | 3537 | 3399 |
| `rate_track = 1` | **3561** (+0.7 %) | 3438 (+1.1 %) |

**+1 %. That is the whole direct gain**, and it is worth stating plainly rather than dressing up.

**More importantly: correcting the rate does NOT unlock long integration.** K = 124 remains *worse* than
K = 31 (3438 vs 3561) even with the rate tracked to 0.026 chip/s — which over a 1.03 s window is 0.027
chip, far too small to matter. So the rate was **not** what was breaking long integration. There is a
second mechanism, and the operator has named the candidate: *"since we only integrate a small fraction of
the frame time that is another edge variable that can resonate with 2.4 frames per chip."* At 2.3992
frames/chip the 45 µs sample walks the chip on a 12-frame / 5-chip cycle, so each chip is sampled at a
systematically different sub-chip position rather than uniformly. **That is the next measurement**, and it
is offline on `static_ir`.

**Where T083 does pay is T081.** Its invariant — "phase survives blind, so re-detection costs one 0.39 ms
correlation" — needed 65 s and had 0.7. It now has 19–42 s. That is the difference between sync-first
being impossible and being merely bounded, and it is the reason to keep this on by default even at +1 %.
