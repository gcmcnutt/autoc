# Fixing the acquisition detector by re-ranking — offline study on `pan2.bcnr` (2026-08-22)

[stage1-K-census.md](stage1-K-census.md) found that acquisition fails not because there are too many
candidates but because **the beacon is not among the strongest ones**, and `engine.c` seeds only the top 3.
This study re-ranks the same recorded frames under alternative detector statistics and measures where the
beacon lands, using the fiducial truth (validated to 0.18°) to locate it.

## Result

| detector | STILL rank p50 | STILL in top 3 | MOVING rank p50 | MOVING in top 3 |
|---|---|---|---|---|
| **current** — &#124;Δ&#124; between passes, reduce4 | **1842** | 25.4 % | **276** | 19.5 % |
| accumulated &#124;Δ²&#124;, 4 fr (1.7 chips) | 2 | 56.7 % | 2 | 56.6 % |
| accumulated &#124;Δ²&#124;, 6 fr (2.5 chips) | 2 | 56.7 % | 2 | 59.5 % |
| **accumulated &#124;Δ²&#124;, 8 fr (3.3 chips)** | **1** | **60.0 %** | **2** | **60.9 %** |
| accumulated &#124;Δ²&#124;, 12 fr (5.0 chips) | 1 | 60.9 % | 2 | 60.3 % |
| accumulated &#124;Δ²&#124;, 16 fr (6.7 chips) | 1 | 64.4 % | 3 | 57.2 % |

**Median rank 1842 → 1 when still, 276 → 2 when moving.** Top-3 seeding roughly triples, and — the part
that matters most — **the moving case becomes as good as the still case** (60.9 % vs 60.0 %). The motion
penalty in *detection* essentially disappears.

The plateau across 4–16 frames means this is robust, not a tuned knife-edge. **8 frames (3.3 chips)** is the
recommendation.

## Why it works, and why the window has an optimum

**Second difference kills linear ramps.** An edge sweeping across a cell at constant rate is a ramp in time,
and a ramp has zero second difference — so the dominant ego-motion artefact self-cancels. The beacon's
on/off alternation does not.

**Accumulation averages the noise down while the beacon keeps modulating** — but only while the beacon stays
in its cell. Push the window too far and it degrades badly (72 frames / 30 chips: rank 2491, top-3 14.3 %),
because at 15–18 °/s the beacon crosses ~3 reduce4 cells in 250 ms. **The window optimum is the same smear
limit measured everywhere else in this feature**, which is a satisfying self-consistency check: ~8 frames is
1.4 M2 px of travel at the measured rate, i.e. still inside one cell.

## A hypothesis that was WRONG, recorded because it was expensive-sounding

I expected a **finer detection plane** to help, reasoning from §2.2 that reduce4 dilutes a sub-pixel point
source across a 4×4 bin. Measured, it makes things **worse**:

| plane | MOVING rank p50 | in top 3 |
|---|---|---|
| reduce4 (current) | 276 | 18.4 % |
| reduce2 | 691 | 14.8 % |
| native | 1268 | 6.3 % |

More cells means more competitors, and the point source does not gain enough contrast to pay for them. The
discriminator was the problem, not the resolution. **Do not "fix" this by looking harder at finer planes.**

## Scope of the win — what this does and does not fix

**Does**: gets the beacon *into the bank*, in both regimes, for roughly one extra subtract per cell per
frame plus a running sum. No new plane, no registration, no track-before-detect. Static cold-acquisition
latency (T074 median **2.95 s**) should improve substantially, since seeding goes from a ~19–25 % lottery
per throttled pass to ~60 %.

**Does not**: fix *confirmation*. A seeded candidate still has to accumulate a word and clear `q_lock`, and
the full-field code match still degrades 5–20× under motion (stage1-pan1-analysis.md). So under motion this
should put the beacon in the bank and then watch it fail to confirm — the coherence work (registration,
decode-along-track) is still required, and this does not substitute for T071/T050.

Expect: **large improvement to static acquisition, partial improvement under motion.** Measure both rather
than assuming, on the bench and against this clip.
