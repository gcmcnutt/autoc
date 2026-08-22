# T072 — K measured, and the acquisition failure mechanism found (2026-08-22)

`acquire_pass()` caps its **return** at 3 seeds, so K — the underlying candidate count — has never been
visible from outside. Measured now with `tools/candidate_census.py`, which replicates the same statistic:
reduce4 coarse plane, temporal diff, count cells clearing **4× the field's mean |diff|**.

## K is not the problem

| condition | exposure | K (p50) | K² pairs |
|---|---|---|---|
| static | 53 µs | 1277 | 1.63 M |
| static, **StepFPGA ON** | 53 µs | 1292 | 1.67 M |
| static | 1499 µs | 276 | 76 k |
| **moving** (`pan2`, pan/tilt/circular) | 1499 µs | 582 | 339 k |

**K = 276–1292 across every condition — comfortably inside the ≲1000–2000 that
[compute-budget.md](compute-budget.md) says the O(K²) proto-track stage needs.** That stage is viable; the
operator's "every pixel is a candidate" limiting case does not occur, because the 4×-mean gate is
self-normalising and holds the count to 1.7–8 % of the plane.

Two secondary results:
- **The StepFPGA contributes nothing measurable.** 1277 vs 1292 — a 1.2 % difference over 719 and 287
  passes. Powering it down for camera work is tidiness, not a fix. (The first-lock A/B agreed and was
  useless on its own: medians 2.95 s vs 3.80 s but Mann-Whitney **U = 9 against a critical 2** at n=5 —
  indistinguishable. First-lock time is a high-variance instrument; K gives one sample per pass instead of
  one per run.)
- **Exposure moves K more than motion does**: 53 µs quadruples it versus 1499 µs (1277 vs 276), while motion
  merely doubles it (276 → 582).

## The actual mechanism: the beacon is not among the strongest blinks

`engine.c` calls `acquire_pass(..., 3)` — it seeds the **3 strongest** detections. Where does the beacon
actually rank? Using `pan2`'s fiducial truth (validated to 0.18°) to locate it in every diff plane:

| segment | beacon rank among cells (p50 / p90) | beacon ÷ threshold (p50) | **in the top 3** |
|---|---|---|---|
| still | 126 / 2234 | 1.71 | **15.9 %** |
| moving | 230 / 600 | 1.88 | **6.2 %** |

**The beacon is typically the ~126th–230th strongest blink in the field, and only the top 3 are ever
seeded.** It clears its own gate by just 1.7–1.9×, not the 12× the field's peak does.

That single fact explains the whole acquisition picture: a static-scene cold acquire needing a median 2.95 s
(T074) is the tracker *waiting to get lucky* at ~16 % per throttled pass; and under motion, at 6.2 %, it
essentially never gets lucky — which is exactly the 22 s of continuous motion in `pan2` with **zero**
reacquisitions.

### This corrects compute-budget.md

That document argued the binding constraint was K, and that registration's job was to keep K small. **Both
halves are wrong.** K was never large, and it *falls* relative to the plane under motion because the
self-normalising threshold rises with the motion-induced noise floor (mean |diff| goes 7.7 → 59.2, ×7.7).
The failure is **not combinatorial, it is a ranking/sensitivity failure**: the beacon is a weak, diluted
detection sitting far down a list dominated by background edges.

### Why the beacon is weak there, and what it suggests

Detection runs on the **reduce4** plane, so a sub-pixel point source is diluted across a 4×4 bin — precisely
the √k background penalty §2.2 warns about. The coarse plane is the worst place to look for a point source;
it is chosen for cost, not for SNR.

Candidate fixes, cheapest first — none of which is the full TBD build, and any of which may be worth trying
before it:
1. **Detect on a finer plane** (reduce2 or native) — 4–16× more cells, but far better point-source contrast.
2. **Rank by periodicity at the chip rate**, not raw |diff| magnitude. Background edges under ego-motion are
   broadband; the beacon is the only thing modulating at 120 Hz. This is a discriminator the beacon *wins*.
3. **Raise `max_seeds` and the bank's candidate cap** (currently 3 seeds, 4 candidates, 16 slots). Catching
   the p50 rank alone would need ~256 seeds, so this cannot be the whole answer — but it is one line.
4. Registration still matters, but for a different reason than claimed: it removes the motion-induced term
   from the noise floor, restoring **sensitivity**, not tractability.
