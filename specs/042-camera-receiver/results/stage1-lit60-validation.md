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

---

## T084 attempted and REVERTED — a negative result worth keeping

The diagnosis was right; the fix was not. Recording both, because the diagnosis stands and the reason the
fix failed is more useful than the fix would have been.

### The diagnosis held up

Of 76 drop episodes on `lit60`, measured at the last tick before each drop:

| | |
|---|---|
| scale | **66 at scale 0** (coarsest), 10 at scale 1 |
| cep | p50 **0.58**, p90 0.77 — *never lost*, well-located throughout |
| age_ms at death | **exactly 150** — the `hold_max_age_ms` bound |
| q into the drop | 0.54 → 0.51 → 0.41 → 0.30 → 0.30 |

So tracks walk down to the coarsest scale and die of age, while being perfectly well-located. Seeds start at
scale 1, so the walk comes from the **HOLD-entry** widen, not the ladder — my first patch gated the ladder
and therefore did nothing (79 % → 79 %, 26 → 26 drops).

### The fix, and why it did not pay

Gating the HOLD-entry widen on `cep` too — keep both the scale and the bins when a track is
weak-but-located, so it spends its 150 ms with a warm window instead of a wiped one:

| clip | present | drops | q p50 |
|---|---|---|---|
| static_bench | 79 % → **74 %** | 26 → 34 | 0.79 → 0.56 |
| static_fid | 99 % → 99 % | 0 → 0 | 1.00 → 1.00 |
| pan2 | 40 % → **36 %** | 14 → 13 | 0.96 → 0.93 |
| lit60 | 42 % → **51 %** | 76 → 82 | 0.45 → 0.44 |

Net across four clips: **zero**, with variance. Reverted.

**Why**: the economics changed underneath the hypothesis. Preserving a struggling track is only worth it
when re-acquisition is expensive — and the promotion fix made cold acquisition **0.40 s**, its theoretical
floor. Holding a weak track now *blocks the slot* for a re-acquire that would have produced a better one.
**Dropping fast and re-acquiring is now the cheaper strategy**, which inverts the premise T084 was built on.

### What the four clips actually say

| clip | q p50 | present | drops |
|---|---|---|---|
| static_fid | **1.00** | **99 %** | 1 |
| pan2 | 0.96 | 40 % | 15 |
| static_bench | 0.79 | 79 % | 27 |
| lit60 | **0.45** | **42 %** | 77 |

Outcome tracks **signal quality**, not tracker logic — and monotonically, apart from `pan2` where the
motion is the confound. At q ≈ 1.00 the tracker is essentially perfect: 99 % present, one drop in 60 s.

**So for the operator's stated goal — "static scene, varying illumination, reliably lock on the beacon" —
the remaining lever is photometric, not algorithmic.** The tracker already delivers a reliable static lock
*when the signal is adequate*. What varies is whether the illumination leaves the beacon adequate, which is
exactly what the 1.56 mm lens and the IR filter are for: more beacon light, and rejection of everything that
is not 850 nm.
