# AC coupling, differential frames, and rough exposure (2026-08-22)

**Operator**: *"We did similar with stepfpga and that thing is impressive… we toyed with dc bias and ac
coupling and it does a decent job of digging out the signal. Single pixel static so similar approach needs
to deal with motion. Almost like differential frames and look for candidate edges when we expect them. And
sort of set exposure so beacon is saturated at maybe 10ft then measure signal out to 100m regardless of
background. Just rough exposure control."*

This names the actual difference between the two receivers, and it explains every discriminator failure in
[the rung-1 work](results/stage1-noise-ladder-rung1.md).

## The difference: WHERE the DC is removed

| | StepFPGA (single PD) | camera receiver |
|---|---|---|
| DC removal | **analog, before the ADC** (AC-coupled TIA) | **digital, after the ADC** (per-bin mean removal in `corr.c`) |
| consequence | background never consumes dynamic range or quantisation | background consumes both, and is only subtracted afterwards |

That is why the 031 chain "digs out the signal" and why my rung-1 gates kept failing: **every statistic I
tried was contaminated by background because the background was still in the samples.** Subtracting a
pedestal after digitisation removes its *mean* but not the noise or the lost headroom it already cost.

Shot noise is the part that does not subtract:

| background | shot noise per frame |
|---|---|
| 0 ADU | 1.0 ADU |
| 10 | 3.2 |
| 50 | 7.1 |
| 150 | 12.2 |

## Differential frames = AC coupling in the digital domain, and sync makes it matched

*"look for candidate edges when we expect them"* is the important half. Once phase is known
([T081](sync-first-acquisition.md)), the frames that straddle a chip transition are known exactly — the
platform's 12-frames-per-5-chips pattern is deterministic ([T082](tasks.md)).

Differencing **across a known chip transition** gives:
- static background → **cancels exactly**
- a constant ceiling lamp → **cancels exactly** ← this alone kills the rung-1 defect
- the beacon at a transition → **full amplitude, signed, with known polarity from the code**

It is a matched differential rather than a blind one, and it needs no threshold on q, energy or modulation
depth — the quantities that all failed. There is already convergent evidence: the blind version (accumulated
|Δ²|, T080) moved the beacon from **rank 1842 → 1** on detection. The synced version should be strictly
better because it knows the sign.

**Motion is the part that does not come for free.** The StepFPGA differences in time at a fixed pixel; a
camera must difference *along the track*, which is exactly T050's decode-along-track. Sync-first makes that
cheaper (one phase, not 31), but it does not remove the need.

## Rough exposure: saturate near, integrate far

Fixed/coarse exposure calibrated so the beacon **saturates at ~10 ft**, then let range do the rest. The
arithmetic, with a 258 ms word (74 frames) and a ~1.5 ADU/frame read-noise floor:

| range | 1/r² vs 3.05 m | single-frame ADU | coherent SNR over one word |
|---|---|---|---|
| 3.05 m | 1.0 | 255 (saturated) | 1466 |
| 10 m | 0.093 | 23.7 | 136 |
| 30 m | 0.010 | 2.6 | **15** |
| 60 m | 0.0026 | 0.66 | **3.8** |
| 100 m | 0.0009 | 0.24 | **1.4** |

**The policy works to ~30–60 m on one exposure setting and runs out at 100 m.** The demanded dynamic range
3 m → 100 m is **1075×** against an 8-bit sensor's 255×; coherent integration over a word buys only
**√74 = 8.6×**. So 100 m needs either a longer integration (spec §4's adaptive length — the far field is
slow, so it can afford one), a larger aperture, or a nearer saturation point than 10 ft.

That is not an argument against the policy — it is the policy's operating envelope, and it matches §2.2's
"near field = strong and fast, far field = weak and slow" exactly.

## Why this is better than the AGC we have

Today's AGC is **reactive**: it watches ROI peak and servos exposure, which is why it sat pinned at 53 µs
through `lit60` while the beacon struggled. But the beacon is a **constant-brightness source**, so its
apparent brightness is a **range measurement**, not a disturbance to be rejected. Operator: *"maybe our AGC
is aided by our distance estimate more than anything."*

Feeding exposure forward from range (the scale/extent estimate is already a crude range proxy) closes the
loop on **geometry instead of photometry**, so it never chases the background at all. Combined with the IR
filter — which removes the background's contribution rather than merely subtracting its mean — that is the
combination that makes the far-field cells reachable.

## What to build, in order

1. **T081 sync-first** — phase is the prerequisite for "edges when we expect them".
2. **T082 sub-frame edge handling** — identifies the straddling frames, which ARE the transition frames.
3. **Differential-at-known-transition detection** — replaces the q/energy/modulation gates that failed.
4. **Range-fed-forward rough exposure** — replaces the reactive AGC.

Steps 3 and 4 are new and follow from this note; 1 and 2 were already filed.
