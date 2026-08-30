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

---

# Verification of `pend4`, and the sub-chip measurement (2026-08-30, operator away from bench)

## `pend4.bcnr` — the first clip recorded with the whole stack live

69093 frames, **0 seq gaps, 1 dropped frame** (0.0014 %), 240 s requested / **214 s of beacon** (clean
stop at full amplitude = the pod's UVLO, not a fade). Truth: `/data/pend4_truth.csv`, ROI
`279,380,164,208` from `--arc`. 32722 position samples (47 %).

| block | rate | decode | bearing |
|---|---|---|---|
| 100–120 s | 3.3 °/s | **100 %** | 0.59° |
| 120–140 s | 2.2 | **100 %** | 0.37° |
| 140–160 s | 1.5 | **100 %** | 0.34° |
| 160–180 s | 1.0 | **100 %** | **0.26°** |
| 180–200 s | 0.7 | **100 %** | **0.22°** |
| 200–220 s | 0.5 | 96 % | **0.20°** |

**2932 on-beacon fixes, ZERO false.** Relock p50 40 ms, 42/47 (89 %) inside the 400 ms bar.

Two firsts: four consecutive blocks at **100 % decode**, and bearing **inside the §3 0.3° bar** (0.20–0.26°
below 1 °/s) — previously the best block anywhere was 0.44°. The "overall present 79 %" the analyser
prints includes the 26 s after UVLO with no beacon in the room; the per-block table is the honest view.

Unchanged: **≥8 °/s is 18 %.**

## The sub-chip sampling phase — operator's "edge variable"

*"since we only integrate a small fraction of the frame time that is another edge variable that can
resonate with 2.4 frames per chip."*

**The resonance does not occur at the actual rate**, and the reason is a near miss. Lock-up to 5 sampling
positions needs exactly 2.400 frames/chip (12 frames = 5 chips). The measured ratio is
287.85 / 120.794 = **2.383**, whose per-frame sub-chip advance (0.4196) does not close, so the sampling
phase walks the chip uniformly:

    sub-chip positions visited: 101 distinct to 0.01
    histogram over 10 bins:     [866, 866, 862, 863, 866, 862, 863, 865, 863, 862]

Flat. So the concern is well founded in principle and **we are accidentally safe from it** — which is
worth knowing, because retuning the pod toward a "nicer" 2.400 ratio would *create* the resonance.

**The edge effect itself is real, and asymmetric** (`static_ir`, lit chips only):

| sub-chip position | amplitude vs lit mean |
|---|---|
| 0.0–0.1 (chip start) | **−3.5 %** |
| 0.2–0.8 (middle) | +1.3 … +1.8 % |
| 0.9–1.0 (chip end) | **−6.0 %** |

A sample landing on a boundary catches the LED transition and is attenuated, more at the end than the
start — so the fall is slower than the rise, or the assumed boundary sits slightly early. Cost is ~5–8 %
on ~20 % of samples. Real, modest, and **not** what limits long integration.

## Correcting an overstatement: "K = 124 has never been usable"

That was measured only on a **saturated** beacon (`static_ir` peaks at 255), where long integration has no
SNR to add and only costs. That is the wrong regime to judge a far-field feature in. Re-tested on the
unsaturated `rng_8m` (peak-to-peak 208, not railed):

| K | LOCK | MEASURED | q p10 |
|---|---|---|---|
| 31 | **90 %** | **88 %** | 0.93 |
| 62 | 74 % | 57 % | 0.47 |
| 124 | 86 % | 51 % | 0.36 |

Still worse — but `rng_8m` is only 5 s, so a 1.03 s window spends a fifth of the clip filling, and that
dominates. On the 240 s `pend3` the K = 124 penalty is only ~3 % (3438 vs 3561).

**The honest statement**: long integration costs a few percent where signal is ample, and its *benefit*
— the far-field case it exists for — is **untested**, because every clip we own has a strong beacon
(saturated, or 208 p2p at 8 m). Not "broken"; unproven, and mildly costly where it is not needed.
Testing it needs a genuinely weak target, which is a range/outdoor fixture, not a bench one.

---

# Attacking the coherence limit: two failed attempts that locate the real defect (2026-08-30)

Indoors is now excellent below ~3 °/s (100 % decode, 0.20–0.26° bearing, zero false locks) and **18 % at
≥8 °/s**. That band is the entire remaining indoor gap, so this is where the work went.

## The mechanism, measured

The ROI centre is `xp`, updated **once per tick (50 ms)** and frozen for the ~12 frames in between, while
`track_frame` deposits `roi[p]` into `bins[p]` — an **aperture-relative** index. So the beacon walks
across a stationary bin grid and the per-pixel chip bins accumulate a trail rather than a point:

| rate | drift within one tick | over a 258 ms word |
|---|---|---|
| 8 °/s | 0.7 M2 px | 3.8 |
| 16.5 °/s | **1.5 M2 px** | 7.8 |
| 19 °/s | 1.7 M2 px | 8.9 |

against a FINE aperture **5 M2 px across**.

## Attempt 1 — base the ROI on `xr`. BLEW UP, exactly as this file already warned.

`xr = x + v*tau` is the tracker's own "where it is NOW", so centring there looks strictly more correct.
It is not: the aperture position becomes a function of `v`, the measurement is taken inside that
aperture, and `v` grows from its own output. Golden clip: **velocity 48.5 M2 px/s against a true 30**,
position error 7.4 px, lock_health 75.

`track.c` already carries this warning — *"v*tau added to z (positive feedback through v, 3x blowup)"* —
and this is the same loop entered through the **ROI** instead of through **z**. Recorded here because the
existing note only names the `z` route.

## Attempt 2 — mean-preserving intra-tick smoothing. Also worse, and this is the informative one.

Sweep the centre through `xp` at the tick's midpoint, so the tick-average aperture is unchanged and only
the sawtooth goes. No `v*tau` anywhere. Golden improved (4/17 → 2/17 failing) but was still wrong, and on
the real clips it is clearly harmful:

| | baseline | ROI-follow |
|---|---|---|
| `pend4` fixes | **2932** | 2218 (−24 %) |
| `pend4` 5–8 °/s | **48 %** | 30 % |
| `pend3` fixes | **3561** | 2877 (−19 %) |
| `pend3` 5–8 °/s | **55 %** | 36 % |

Reverted.

## What that tells us — the missing operation is a BIN SHIFT

`m2_q8_to_plane_px()` truncates to an integer plane pixel, so re-centring the ROI moves the bin-to-sky
mapping in **whole-pixel jumps**. Every jump invalidates the accumulated per-pixel window, because
`bins[p]` silently changes which patch of sky it means. Moving the ROI *more often* therefore corrupts
the window *more often* — which is exactly the ordering measured above.

The current code gets away with one jump per tick. It does **not** get away with it for free: that jump
is already corrupting the window at every tick boundary, and at high rate the jumps are larger.

So the fix is not to move the aperture less, nor more smoothly — it is to **shift the bin array by the
same (dx, dy) whenever the ROI centre moves**, so `bins[p]` keeps meaning the same sky. Then the ROI can
follow the target continuously *and* the window stays coherent, which is the actual content of
"decode along track" for a case where the velocity is already known.

`track_frame` already records `roi_cx`/`roi_cy` every frame and **never reads them** — the hook is
sitting there unused.

Cost: shifting `E*E*TRK_WIN` int32 on a move. At E = 16 that is 127 KB, and at high rate the ROI moves
~30 times/s, so ~4 MB/s — affordable against the ~36 ms of tick margin. A rotating origin would make it
free but is harder to get right; measure the memmove version first.

**This is the next indoor step**, and it is the first thing measured all session that plausibly moves the
≥8 °/s band rather than the lifecycle around it.

## Attempt 3 — shift the bins with the aperture. WRONG PREMISE, and it rules out the whole class.

The reasoning in §"What that tells us" above is **incorrect** and is corrected here rather than deleted,
because the correction is the useful part.

Implemented: shift `bins[]` by the same (dx, dy) whenever the ROI centre moves, so `bins[p]` keeps meaning
a fixed patch of sky. Result on the golden clip: the track reaches **q = 1.00 with lock_health 0.02–0.36**
and can never promote — `lock_health` collapsed, because the shift was smearing the beacon's own
accumulated history across pixels.

**Why it is wrong.** The per-pixel bins are **aperture-relative**, and the aperture *follows the target*.
So a tracked beacon already sits at an approximately FIXED bin index — the aperture-relative indexing
**is** the motion compensation, and it is already doing its job. Sky-locking the bins actively undoes it:
it holds the grid still and lets the beacon walk across it, which is the very smear I set out to remove.

The residual smear in the existing scheme is therefore **not** the target's transit (7.8 M2 px per word at
16.5 °/s). It is only the **quantisation** of the ROI centre — `m2_q8_to_plane_px()` truncates to a whole
plane pixel, so the beacon dithers ±0.5 plane px within the aperture. That is a much smaller error, and it
is why all three attempts to "fix the bookkeeping" made things worse rather than better.

## What the three failures establish

| attempt | result | why |
|---|---|---|
| ROI based on `xr` | v ran to 48.5 vs true 30 | `v*tau` in the aperture ⇒ positive feedback through v |
| mean-preserving ROI follow | pend4 −24 % fixes | more re-centrings ⇒ more quantisation jumps |
| sky-locked bin shift | lock_health collapse | undoes the compensation the aperture already provides |

**The aperture bookkeeping is not the coherence limit.** It is already motion-compensated, correctly, by
construction. What actually limits high rate is a **chicken-and-egg on velocity**: the aperture follows
the *prediction*, the prediction needs a velocity, and the velocity comes from fixes that are themselves
failing at high rate. The tracker can only ever REFINE a velocity it already has; it cannot acquire one.

That is precisely the gap **T050 decode-along-track** exists to fill — hypothesise velocities and test
them, so a lock is not a prerequisite for a lock. And it is exactly the operator's framing of the event
formulation (2026-08-30): *"moving apertures vs event camera has candidates who changed polarity at
expected time compared to motion estimates."* An event detector tests (position, velocity, code-phase)
hypotheses against the **timing of polarity changes** along a trail, which needs no established track and
no accumulated per-pixel window — so it does not have the chicken-and-egg at all.

**Recommendation**: stop trying to improve the correlator's motion handling. The next indoor gain at
≥8 °/s comes from hypothesis-based detection (T050 / the event path), not from better bookkeeping around
an aperture that is already doing the right thing.

---

# THE PHYSICS CEILING IS 100 % — oracle trail decode settles the coherence limit (2026-08-30)

Operator's direction: *"let's see how far stationary chase and moving target can go"* — with attitude
estimators layered later. `tools/trail_decode.py` measures the ceiling of that case by cheating with
truth: sample the clip along the fiducial trajectory (a ±4 px box that follows the beacon exactly), build
the along-track amplitude series, correlate against the Gold code in sliding one-word windows. That is
decode-along-track with an ORACLE for the velocity search. The control is the same correlation with the
box FROZEN at each window's start — what a static aperture sees over one word, on the same photons.

## The result, on both clips independently

`pend4` (and `pend3` within a point everywhere):

| rate bin | windows | **TRAIL decode / q p50** | FROZEN decode / q p50 | real tracker |
|---|---|---|---|---|
| 0–2 °/s | 1237 | **100.0 % / 1.00** | 100.0 % / 1.00 | ~95 % |
| 2–5 | 1155 | **100.0 % / 1.00** | 99.7 % / 1.00 | ~80 % |
| 5–8 | 533 | **100.0 % / 1.00** | 80.9 % / 0.71 | 48 % |
| 8–12 | 376 | **100.0 % / 1.00** | 47.1 % / 0.57 | ~18 % |
| 12–18 | 305 | **100.0 % / 1.00** | 30.2 % / 0.58 | ~18 % |
| **18–40** | 177 | **100.0 % / 1.00** | 22.0 % / 0.60 | — |

**Three things this settles.**

1. **The code survives motion perfectly.** Along the true trail, decode is 100.0 % with q p50 = 1.00 at
   every rate measured, including 18–40 °/s — twice the engagement rate. There is no smear penalty, no
   exposure-sampling penalty, no LED-edge penalty large enough to dent a word. The photons are fine.

2. **The coherence limit is 100 % velocity acquisition and 0 % physics.** The frozen control reproduces
   the tracker's collapse from the same photons (22 % at 18–40 °/s against the tracker's 18 % at ≥8),
   which both validates the methodology and localises the entire gap: the only thing the tracker lacks at
   high rate is knowing where to look next.

3. **The hypothesis search is guaranteed to have a signal to find.** If a (position, velocity) candidate
   is right, its q is ~1.00 — not marginal, SATURATED. A wrong candidate scores like the frozen control's
   tail (~0.6 at best). The discriminator between right and wrong velocity is enormous, which is exactly
   what makes a search cheap: coarse steps, early rejection, no fine tuning.

## What to build, and the arithmetic that says it is small

Velocity quantisation: a hypothesis is good enough when its trail error stays inside the box over one
word — dv × 0.258 s ≤ 2 M2 px → **dv ≈ 8 M2 px/s ≈ 4.4 °/s per step**. Covering ±19 °/s needs ~9 steps
per axis = **~81 velocity hypotheses per candidate position**, each costing one 74-sample trail
correlation (~2.3 k MAC including the phase search). With the event stage nominating a handful of
candidate positions (measured: ~19 events/frame, pure list), the whole search is well under a millisecond
on one core — against the 36 ms tick margin.

Data structure: a ring of ~74 reduce4 planes (160×100 × 74 = **1.2 MB**) — reduce4 is already NEON'd at
0.45 ms/frame, and the beacon saturates even 4×4-binned. No new capture path, no WC-read problem.

This is T050 with the V² term removed by the event stage's nomination — the "candidates who changed
polarity at expected time compared to motion estimates" formulation, exactly.

---

# THE TRAIL SEARCH IS LIVE IN core/ — 100 % decode at every rate, on replay (2026-08-30)

Prototype validated first (`trail_search_proto.py`, no oracle): 100 % locked and on-beacon at 0–40 °/s,
err p50 0.5–0.85 M2 px. Then implemented in C: `src/core/trail.c` + `track_apply_trail_fix()` + engine
wiring + `[trail]` config. The design point that keeps both worlds: **the aperture path stays
authoritative whenever it measures** (it has the sub-pixel centroid); the trail search fires only on
ticks where the aperture failed — exactly where the coherence limit lives. A trail fix replaces
(position, velocity) outright — it measured both — so there is no alpha-beta feedback path, which is
what made the two failed ROI experiments unstable.

`pend4`, replay, trail on vs off (`pend3` within a point everywhere):

| block | rate | decode off → **on** | bearing on |
|---|---|---|---|
| 0–20 s | 14.7 °/s | 39 % → **98 %** | 0.74° |
| 20–40 s | 12.3 | 17 % → **100 %** | 0.77° |
| 40–60 s | 8.9 | 14 % → **100 %** | 0.84° |
| 60–80 s | 6.4 | 29 % → **100 %** | 0.79° |
| every block below | ≤4.7 | → **100 %** | 0.19–0.80° |

| | trail off | **trail on** |
|---|---|---|
| total on-beacon fixes | 2859 | **4192 (+47 %)** |
| false locks | 0 | **0** |
| by instantaneous rate, ≥8 °/s | ~18 % | **100 % (924/924)** |
| relock after occlusion | p50 40 ms, 89 % in bar | **p50 27 / p90 46 / max 50 ms, 47/47 (100 %)** |
| MEASURED fix overall | 61 % | **89 %** (rest = occlusions + UVLO tail + cold start) |

Live sanity (beacon off): 0/400 deadline misses, median margin 39.1 ms — the ring feed and search cost
nothing measurable; the search only runs when a precision track exists and its aperture failed.

**The coherence limit is closed indoors, on replay.** 18 % → 100 % at ≥8 °/s, with zero false locks and
byte-identical replay parity (the golden runs with trail enabled). What remains before "field": the same
number live at the bench (the crosshair demo), then the outdoor axes — range/weak-target SNR, ego-motion,
background clutter — which are different problems and were never this one.
