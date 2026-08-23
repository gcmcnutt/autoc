# lit60.bcnr — validating the promotion fix on a fresh, harder scene (2026-08-22)

**Clip**: 60 s continuous, 17275 frames, 0 dropped, exposure 53 µs (the real bench operating point),
**200 W lamp ON**, bench re-assembled in camera config. Third distinct ambient condition of the day, so
nothing here is comparable to the earlier clips — only the within-clip A/B is.

## The promotion fix, measured on the same clip both ways

| | first lock | present | drop episodes | q mean |
|---|---|---|---|---|
| **baseline** (`lock_health ≥ 0.60` gate) | 5.80 s | **2 %** | 2 | 0.57 |
| **with fix** (veto at drop threshold) | **1.00 s** | **42 %** | 76 | 0.45 |

**The baseline essentially does not track this scene at all — 2 %.** The fix makes it 42 %. The 76 drop
episodes look alarming in isolation, but the alternative is not "fewer drops", it is "no tracking".

This is the strongest evidence yet for the change, precisely *because* the scene is marginal: the old gate
required a noisy statistic to clear a high bar three times running, and on weak signal that essentially
never happens.

## Why this scene is harder — and it is NOT what I first guessed

I assumed the 200 W lamp was raising the background and swamping the beacon. **Measured, it is the
opposite**: this scene is *darker* than the morning's (frame mean 2.1 vs 5.6, p99 49 vs 165). Saturation is
also ruled out — `BCN_F_SATURATED` set on **0** of 617 fixes, same as the morning's clip.

The difference is in the **scale ladder**:

| clip | q p10 / p50 / p90 | scales used |
|---|---|---|
| static_bench (morning) | 0.92 / 1.00 / 1.00 | **{2}** only — fine, always |
| lit60 (now) | **0.12** / 0.45 / 0.85 | **{0, 1, 2}** — churning across all three |

The beacon is *sometimes* strong here (q p90 = 0.85) but the track keeps falling to coarse scales, where q
is worse, and never settles.

## New finding: the scale ladder has a positive-feedback failure mode

The ladder widens (scale down) when q is weak. That is right when the beacon has been **lost spatially** —
a wider aperture is the only way to find it again. It is exactly **wrong** when the beacon is **present but
weak**, because spec §2.2's own arithmetic says binning costs **√k in background**: widening *reduces* SNR,
which reduces q, which widens further. A death spiral.

The two conditions are different and the code currently conflates them, because **q is used as the trigger
for both**:

- *lost position* → widen (correct)
- *weak signal* → widen (harmful; should integrate longer instead, which the AGC's integration-length loop
  already knows how to do)

The discriminator between them exists: **`cep`** (position uncertainty) says whether the beacon is lost,
while q says whether it is weak. A track with small cep and low q is weak-but-located — widening it is the
wrong move.

Filed as T084. This is likely the dominant remaining cause of churn, and it is separate from the promotion
gate: the fix above gets a track *started*, and this is what stops it *staying*.
