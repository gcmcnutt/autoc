# Noise ladder — rung 1: bench lamp on (2026-08-22)

Same geometry as rung 0, **only the bench lamp added**. The ladder immediately earned its keep: it exposed
a real defect that a single measurement would have read as noise.

## The numbers, and the split that gave it away

| | rung 0 (dark) | **rung 1 (bench lamp)** |
|---|---|---|
| frame mean | 0.14 | 0.49 |
| **K per pass** | **36** | **213** (×6) |
| 60 s clip: first lock / present / q p50 | 0.30 s / **99 %** / 1.00 | 1.05 s / **97 %** / 1.00 |
| 5 × 15 s live: present | **93/96/95/95/97** | **51/60/87/87/77** |
| 5 × 15 s live: q p50 | **1.00** every run | **0.43/0.94/0.98/0.99/0.98** |

Rung 0 is metronomic. Rung 1 is **bimodal** — some runs are as good as dark, some are half that. The 60 s
clip looks fine because it averages across both modes.

**Flicker was the obvious suspect and it is wrong.** An FFT of the frame mean shows no 120 Hz component in
either rung (peaks at 27/47 Hz, rms 0.024 → 0.032, spectra essentially identical). A lamp flickering at
mains-2f would land exactly on the 120 Hz chip rate, so this was worth ruling out — it is ruled out.

## What is actually happening: the tracker locks onto a REFLECTION

The beacon sits at M2 **(19, +30)** — that is what the 60 s clip tracks at q 1.00. The bad runs track a
different object at M2 **(18, −78)**: same x, 108 px above, q only 0.43–0.58.

Same x, same code, weaker, offset in y — a **specular reflection** of the beacon off something above it.

In the worst run, of 189 fixes: **101 landed on the decoy and 88 on the beacon.** Up to 7 bank slots in use,
and `BCN_F_MULTIPATH_SUSPECT` (0x10) present in the flags.

## The mirror-pair rule is inverted for this geometry

`bank.c`'s T055 rule (spec §9): *two CONFIRMED tracks, same code → the geometrically **UPPER** (smaller y)
keeps CONFIRMED, the lower is flagged MULTIPATH_SUSPECT.*

That heuristic is right for the **flight** case — a beacon in the air reflecting off water or ground, where
the ghost appears *below* the real source. On this bench the reflecting surface is *above* the emitter, so
the ghost is upper — and the rule **promotes the reflection and demotes the real beacon**.

Beacon at y = +30 (lower) → flagged as the suspect. Decoy at y = −78 (upper) → kept as truth. Exactly
backwards, roughly half the time, which is precisely the bimodality measured above.

## What this means

- **The rule is not wrong, its assumption is unstated.** "Upper is real" encodes *"the reflector is below
  the emitter"*. That holds over water and ground; it fails wherever the reflector is overhead. The
  assumption needs to be explicit, and the rule needs a discriminator that is not pure geometry.
- **q is the available discriminator and it is decisive here** — 1.00 for the beacon against 0.43–0.58 for
  the ghost. A reflection is attenuated by the surface, so the direct path is essentially always stronger.
  Preferring higher q, with geometry as a tie-break, would pick correctly on this bench *and* still pick
  correctly over water.
- **This is why rung 1 looks bimodal rather than degraded.** K rising 36 → 213 did not blunt the tracker;
  it gave a same-code ghost enough support to reach CONFIRMED, after which a geometric rule chose wrongly.

## For the remaining rungs

Watch for the decoy explicitly — report **which object is being tracked**, not just q and present %. A run
that is "50 % present at q 0.5" and a run that is "50 % present because it spent half its time on a ghost"
look identical in the summary numbers and are completely different faults.
