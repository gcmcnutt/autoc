# pan1.bcnr — the first measured motion result (2026-08-21, DGX bench)

**Clip**: `beaconpi5:/data/pan1.bcnr`, 640×400, 2800 frames, 59.3 s span, exposure 53 µs, gain 2.0.
**Raw data**: [stage1-pan1-envelope.csv](stage1-pan1-envelope.csv) — one row per burst.
**Tools**: `firmware/beacon-receiver/tools/oracle.py` (T044) + the replayed tracker.

## Headline

**The spec's central prediction is confirmed on real data, and it is the harsher reading.** spec.md §1
claimed *"above ~1–2 °/s the processing gain is gone."* Measured crossover: **between 1 and 3 °/s**, against
a theoretical 1.18 °/s (one M2 pixel of smear in one 258 ms word). The full-field static matched filter —
*no ROI, no aperture, no ladder, nothing the tracker does* — loses coherence at walking-pace apparent
motion.

| apparent rate | n bursts | median q(33 chips) / q(5 chips) | smear per 258 ms word | verdict |
|---|---|---|---|---|
| 0.0–0.5 °/s | 14 | **1.83** | 0.0 px | long integration **wins** |
| 0.5–3 °/s | 1 | **1.32** | 0.9 px | long integration **wins** |
| 3–10 °/s | 2 | **0.51** | 4.5 px | long integration **loses** |
| 10–30 °/s | 9 | **0.62** | 12.4 px | long integration **loses** |
| 30–80 °/s | 7 | **0.52** | 39.8 px | long integration **loses** |
| 80–200 °/s | 2 | **0.58** | 95.3 px | long integration **loses** |

### Scope: this bounds ACQUISITION, not tracking

Stated precisely, because the distinction decides what to build. The measurement above is of a **static
full-field** matched filter — the pixel grid does not move. That is exactly what **acquisition** is
(`acquire_pass` + a fresh candidate's first word), so the 1–3 °/s crossover is an **acquisition** limit:
above walking pace the receiver **cannot cold-acquire at all**, by either of its two mechanisms — the blink
detector is swamped because a moving camera makes the whole field differ frame-to-frame, and now the code
correlator too, because the signal leaves the pixel mid-word.

**Tracking** is not bounded by this number, because a confirmed track's ROI *follows* the prediction — in
the ROI's frame the beacon can stay put while the field sweeps past, which is what makes §3.2's "≥23° of
apparent motion per word" reachable at all. Tracking is bounded instead by aperture, innovation gate and
prediction error (see the 2026-08-21 tracker analysis).

**This is why "random reacquire is essential" is the hard requirement**, not a softer one: the operator's
stated need is precisely the case the receiver currently cannot do at all. T050/T051 (proto-track,
decode-along-track) are therefore not a tracking nicety — they are the *only* route to acquiring a moving
target, and the sole enabler of reacquire-while-slewing.

**The sign of the slope flips, and that is the whole finding.** Stationary, a longer coherent window buys
processing gain exactly as designed (33 chips beats 5 chips by 1.8×). Moving, a longer window *costs* — 5
chips beats 33 chips by up to 4.5× (burst 25). A ratio below 1 cannot happen from noise; it only happens
when the signal is leaving the pixel during the integration. This is the measured form of "the integration
model is what has to change".

## What the tracker did

| | bursts | got a track |
|---|---|---|
| stationary (≤5 °/s) | 16 | 5 = **31 %** |
| moving (>5 °/s) | 19 | 1 = **5 %** |

Even in the stationary bursts it managed only **3 ticks of the ~7 in the burst** — it spends essentially
the whole burst acquiring (≥258 ms for one word) and the burst ends just as it succeeds.

## THE CLIP IS BURST MODE — and that limits what it can answer

`bcnr_info` says `mode burst`: 35 bursts × 80 frames, 278 ms each, ~1.74 s of dead air between, **16 % duty**
over the 59 s span. `beacon_trackd --record` takes its mode from `[record] mode` in `beacon-bench.ini`,
which is `burst` — the bench default, not what a motion campaign wants.

Consequences, all structural:
- **Time-to-relock cannot be measured at all.** Reacquisition spans the gaps, and the container contract
  forbids correlating across a burst boundary.
- **Tracking *through* a pan cannot be demonstrated.** A burst is 278 ms = **1.08 words**; the tracker gets
  barely one word per burst and must cold-acquire every time.
- The 11 % "track present" figure over the whole clip is an artifact of the dead air — the honest
  denominator is ticks that had frames.

**Owed: re-capture with `mode = continuous` before any envelope run.** Everything above still stands — the
coherence measurement only needs within-burst data — but the tracker-side numbers are floors, not results.

## Caveats

- The rate fits for bursts **2 and 33** disagree with their own coherence ratio (46.9 °/s and 21.2 °/s but
  ratios 1.29/1.53, i.e. unsmeared) and should be treated as failed fits, not data. The cause is the same
  limitation being measured: under motion the oracle's short windows have low q and occasionally slip onto
  clutter. Aggregate bins are used above for exactly this reason.
- **The oracle needs decode-along-track too.** A static matched filter cannot measure the moving case
  precisely because that is the case it fails at. T044 and T051 are therefore coupled: the ground truth for
  the fast bands only gets sharp once integration follows the track.
- Scene clutter is heavy (see the `--field-map` viewfinder: multiple bright point sources besides the
  beacon). The first version of `oracle.py` took the brightest high-pass pixel per frame and produced
  6 500–19 000 °/s — the tell that it was hopping between fixed objects, not tracking the beacon. Only the
  code discriminates. That is the same fact the tracker rests on (§2.4).

---

## Addendum 2026-08-21 — the fiducial ground-truth path is verified end to end

The circular dependency this document flagged (truth for the fast bands needing the very
decode-along-track it was meant to score) is now broken in practice, not just in principle. Four printed
ArUco `DICT_4X4_50` markers were placed around the emitter and a 15 s clip recorded with
`beacon-fiducial.ini` (`exposure_min_us` pinned at 1500 so paper is visible — at the bench 53 µs it is
black). Both halves work **simultaneously**:

| | result |
|---|---|
| beacon | track present **81 %**, q mean **0.88**, `chip_hz` **120.00/120.00**, cep 0.29 |
| markers, ≥3 detected | **100 % of frames** — full 2D pose available on *every* frame |
| markers, all four | **97 %** (id0 is the weak one: 97.1 %, 35.3 px) |
| recorder | 4320/4320 frames, **0 dropped**, 0/300 deadline misses |

**Truth is therefore per-frame at 288 Hz**, independent of the code, the tracker, and the apparent rate —
which is exactly what the fast bands could not otherwise be measured in.

Two measured corrections to earlier reasoning in this file:
- **Marker detection was contrast-limited, not size-limited.** With room lighting only, the upper pair
  (~27 px) did not detect and I attributed it to angular size. Focus was ruled out — the failing region was
  the *sharpest* in frame (Laplacian variance 2139 vs 1978) — and adding 200 W of illumination brought all
  four to 100 % at unchanged size. Both limits are real; on this rig contrast bound first.
- **The beacon does not saturate at 1500 µs.** `BCN_F_SATURATED` was set on 0 of 243 fixes, so §5's
  flat-top estimator never engages at the fiducial exposure.

Still open on the fixture: all four markers sit at 27–36 px, below the ~40 px comfort floor, and they span
only ~19 % of the field width. Expect the weakest to drop first under **motion blur**, which is precisely
when truth matters most — so the pan clips must publish per-frame truth *coverage* alongside the envelope,
not assume it.
