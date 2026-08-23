# Noise ladder — rung 1: bench lamp on (2026-08-22)

Same geometry as rung 0, **only the bench lamp added**. The ladder found a real defect — but the
investigation took four wrong turns first, and those are recorded below because the method failure is as
instructive as the result.

## The numbers

| | rung 0 (dark) | **rung 1 (bench lamp)** |
|---|---|---|
| frame mean | 0.14 | 0.49 |
| **K per pass** | **36** | **213** (×6) |
| 60 s clip: present / q p50 | **99 %** / 1.00 | 97 %, 92 % / 1.00 |
| 5 × 15 s live: present | **93/96/95/95/97** | **51/60/87/87/77** |
| 5 × 15 s live: q p50 | **1.00** every run | **0.43/0.94/0.98/0.99/0.98** |

Rung 0 is metronomic; rung 1 is **bimodal**. The 60 s clips look healthy because they average across both
modes. In `rung1b`, **101 of 1160 fixes landed on a decoy** rather than the beacon.

## THE CAUSE: q has no energy floor, so a saturated flat lamp scores arbitrarily high

The decoys are **recessed ceiling downlights**, visible in an auto-exposure frame. Measured at the pixels
the tracker actually reported:

| | mean | rms | |
|---|---|---|---|
| beacon | 157.1 | **104.8** | peaks at code harmonics k = 2, 3, 6, 7, 8, 12 × 3.871 Hz — textbook Gold spectrum |
| downlight (179, 36) | **255.0** | **0.00** | constant, saturated |
| downlight (355, 44) | **255.0** | **0.00** | constant, saturated |

`corr.c`:

```c
static uint16_t quality_q8(int32_t corr, int32_t energy)
{
    int64_t q = ((int64_t)(corr < 0 ? -corr : corr) * 256) / energy;
```

**q is a pure ratio**, and `energy` is the sum of per-bin deviations with only `| 1` guarding division by
zero. On a **saturated flat** region the deviations collapse, `energy` → ~1, and **noise-level `corr`
yields a high q**. That is why a constant lamp — which carries no code at all — reaches q ≥ 0.55 often
enough to promote.

**The C receiver has no min-energy gate. Both of its predecessors do:**
- s7 gateware: an explicit *"min-energy gate"*
- `pi/beacon_track.py`: `--min-peak 150`, documented as *"no LOCK unless correlation peak >= this (s7
  min-energy gate; noise ~20-60)"*, and applied as `if q >= a.qlock and pk >= a.min_peak`

So the gate was dropped in the C rewrite. Filed as **T085**: require an absolute correlation floor
alongside the ratio, exactly as the gateware and the prototype do.

## Four wrong turns, recorded so the method improves

1. **Called it a specular reflection** on the strength of "same x, same code" — before checking whether
   anything was even at that pixel.
2. **Tested for lamp flicker on the frame mean.** A single small lamp cannot move a whole-frame average;
   the test could not have detected what it was looking for. Worse, the peaks it *did* show (27/47 Hz) were
   the **beacon's own code harmonics**, which I briefly read as ambient.
3. **Searched `rung1.bcnr` for a decoy that is not in it.** The decoy fixes came from the *live* 15 s runs,
   which were never recorded. Its patch is zero for all 60 s of that clip.
4. **Mis-centred the analysis patch**, getting "flat" and "saturated 100 %" out of the same clip depending
   on a few pixels of window — because "the decoy" is not one object. In the live run it was at native
   (355, 44); in `rung1b` at **(179, 36)**. Different downlight each time.

The common thread: **inferring from a derived statistic instead of looking at the thing.** What finally
worked was taking the tracker's own reported positions, cropping the image there, and reading `corr.c`.

## What this does NOT show

Earlier I claimed the mirror-pair rule (T055) was inverted for this geometry and was promoting a reflection
over the beacon. **Withdrawn** — there is no reflection here, and no evidence the rule misfired. The
`MULTIPATH_SUSPECT` flag seen in the flags was the rule doing its job on two same-code tracks; the defect is
that one of those tracks should never have existed.

## For the remaining rungs

Report **which object is tracked**, not just q and present %. A run that is "50 % present at q 0.5" and one
that is "50 % present because half the time it sat on a lamp" are identical in the summary and are entirely
different faults.

---

## T085 attempted twice and REVERTED — plus the measurement error that caused the second failure

The diagnosis (§ above) stands: a constant saturated lamp promotes because q is a ratio whose denominator
collapses. Two gate designs were tried against it and both are reverted.

### Attempt 1 — absolute correlation floor (the predecessors' approach)

Gate `|corr|` normalised per chip and per aperture pixel, floor calibrated from the bench at 150 000.

| clip | present before → after | lamp fixes |
|---|---|---|
| dark60 | 99 % → 98 % | 7 → **0** |
| rung1 | 97 % → 98 % | 6 → **0** |
| rung1b | 92 % → 86 % | **101 → 0** |
| **static_fid** | **98 % → 0 %** | 0 → 0 |

It kills the lamps outright — and **silently zeroes tracking on the fiducial clip.** The cause is a flaw in
the premise: the ROI normalisation (`rawsum·2¹⁴ / (exposure_us·gain/256)`) is exposure-invariant **only
while the source is unsaturated**. Our beacon saturates at both 53 µs and 1499 µs, so at the fiducial
exposure its normalised amplitude falls ~28× and an absolute floor rejects everything. **A threshold that
can silently zero the tracker is not shippable.**

### Attempt 2 — dimensionless modulation depth

`mod = (energy/W) / level`, a ratio, so it cannot be broken by a scale change. Measured distributions with
the gate **disabled**:

| | beacon p05 | lamp p90 |
|---|---|---|
| rung1b | 0.027 | 0.012 |
| static_fid | 0.016 | — |

and a threshold sweep suggested **0.04** — keeping ~94 % of beacon fixes on *both* clips while rejecting
95 % of lamp fixes. (It also showed my first guess of 0.08 would have kept only 16 % of `static_fid`, so the
sweep was worth doing.)

**Enabled, it failed anyway**: `static_fid` 98 % → **0 %**, `pan2` 40 % → **2 %**.

### The error: I measured the distribution under one policy and applied the threshold under another

With the gate **off**, a track survives its weak early ticks, establishes, and then produces the high-`mod`
fixes that dominate the distribution. With the gate **on**, those same weak early ticks are rejected, the
track dies before it establishes, and the high-`mod` population **never comes into existence**. The
distribution I measured was conditioned on the very survival the gate removes.

That is a feedback loop, not a threshold-tuning problem, and no value of `min_mod` fixes it. Any future gate
here must either be applied only to *established* tracks, or be validated end-to-end rather than against a
distribution collected with it disabled.

### Why the fiducial clips are the ones that break

Modulation depth is `deviation / level`, and at 1499 µs the background contributes far more to `level` while
the beacon's excursion is clipped by saturation. So the same beacon reads as *less modulated* purely because
the background got brighter — which is exactly the coupling the IR filter is meant to remove.
