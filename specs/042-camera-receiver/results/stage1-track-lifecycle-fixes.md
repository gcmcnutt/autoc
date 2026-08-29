# The tracker was fighting itself — three lifecycle defects, measured on `pend_ir` (2026-08-29)

Operator: *"make the known improvements and head toward higher performance."* This is what the known
improvements turned out to be worth, and what one of them cost before it was rebuilt.

All numbers are `beacon_trackd` replaying **`/data/pend_ir.bcnr`** — 240 s, 69094 frames, 0 seq gaps,
the flight optics (1.26 mm + 850 nm bandpass), exposure pinned at 53 µs — scored against
`pendulum_truth.py` at `--deg-per-px 0.548`, `--limit 210`. Same clip, same truth, same
`beacon-event.ini` for every build; only `core/` differs.

> **Truth for this clip is now persisted**: `beaconpi5:/data/pend_ir_truth.csv`, ROI `293,408,170,218`
> measured with `--arc` (native x 313..388, y 190..198 — a single isolated arc, no competing saturating
> source in the room). 38237 position samples over 69094 frames (55 %). Regenerate with `--arc` first if
> the rig moves; the default ROI in the tool is the **2.31 mm** rig's and is wrong for this clip.

## The headline

| | base | **gate fix** | + rebin widen | **+ evidence split** |
|---|---|---|---|---|
| on-beacon fixes | 1555 | 1752 | 1726 | **1770** |
| false locks (>20 M2 px) | 27 (**1.7 %**) | 17 (1.0 %) | 29 (1.7 %) | **18 (1.0 %)** |
| relock p50 | — | 209 ms | **88 ms** | 159 ms |
| within the 400 ms bar | — | 7/9 | 7/9 | **8/9** |
| 0–1 °/s | 57 % | 59 % | 58 % | **61 %** |
| 2–3 °/s | 55 % | 65 % | 63 % | **66 %** |
| 3–5 °/s | 50 % | **57 %** | 52 % | 56 % |
| 5–8 °/s | 33 % | 36 % | **39 %** | 36 % |
| ≥8 °/s | 14 % | 18 % | **20 %** | 18 % |

**+14 % more real decodes and 33 % fewer false locks, from two correctness fixes and no tuning.**

## 1. The innovation gate was a LATCH — the big one

`track.c` gates a position measurement against the prediction:

```
gate = max(4 * cep, 2 M2 px)
if (|innovation| > gate) reject -> coast
```

and `cep` was inflated **only in `TRK_HOLD`**. For a CONFIRMED track it froze at the last accepted
centroid spread (0.5–1.0 px). So once the state fell further behind than ~4 px, **every** subsequent
measurement was rejected, the state was therefore never corrected, and it fell further behind still.
Rejecting a fix is evidence the state is WRONG, and the code answered that evidence by rejecting harder.

Caught by instrumenting the golden clip, where it is stark: a PRECISION track sat frozen at x=45 with
velocity stuck at (4.1, 21.1) for **350 ms**, `measured_fix=0` on every tick, while the beacon walked to
x=65 — still reporting `cep` 1.0. It escaped only when `q` had decayed enough to force HOLD, ~7 ticks of a
dead track later, and on the third occurrence it did not escape at all.

The fix is one line of placement: **uncertainty grows whenever we coast**, in CONFIRMED exactly as in
HOLD, with the growth law HOLD already used. The gate then reopens progressively instead of latching.

Two things fall out. `cep` stops lying — it is the number both the envelope scorer and spec §3.1's
validity bound read, and a flywheeling track's uncertainty genuinely is growing. And the ladder's climb
gate (`cep < 2 px`) stops letting a coasting track climb to a scale it cannot hold.

> ⚠️ **This had been silently failing the golden test since `6452bd6`** (the lock_health-veto change,
> which was itself a real live win: acquisition 8.30 s → 0.40 s). Bisected. The defect was always there;
> that commit changed promotion timing enough to expose it. `beacon_golden_test_replay_parity` was
> reporting **1/8 checks FAILED** — "a confirmed track must be reporting by the end" — and the suite was
> 9/10 at the start of this session despite `continue.md` recording 10/10 at the pause.
> **Run `ctest -R beacon` before believing a baseline.**

## 2. T076(b) — the widen, and a wrong first answer worth keeping on the record

Widening `memset` both `bins` and `counts`: every sample the track had collected, gone. Rebuilding a
31-chip window takes **258 ms** at 120 Hz; `hold_max_age_ms` is **150**. HOLD was structurally incapable
of succeeding, and the widen — the *recovery* move — was what guaranteed it.

**First attempt: re-bin the per-pixel window onto the coarser plane.** A coarse pixel is exactly the sum
of the r×r fine pixels over the same native area, so this is lossless arithmetic, and it did what it was
designed to do: **relock p50 209 → 88 ms, present 81 → 88 %.**

It also broke bearing: **12.4 °/s block 1.46 → 2.60 °**, false share **1.0 → 1.7 %**. The cause is
specific and worth remembering, because it is invisible in the arithmetic: re-binning drops the preserved
evidence into the **centre** of the new aperture, so the per-pixel position surface peaks at the aperture
centre regardless of where the beacon is — and the aperture is centred on the *stale prediction*. Every
such "measured fix" was just re-reporting the prediction. Bearing error and false locks degraded together
because they were the same defect.

**What was actually wanted** is that the chip window holds two kinds of evidence with different lifetimes
across a scale change:

| | what it is | across a widen |
|---|---|---|
| `apsum[b]` | **temporal** — aperture flux per chip; feeds q, the phase search, identity re-verification | **survives** — scale-independent in meaning |
| `bins[p]` | **spatial** — the per-pixel surface the centroid comes from; aperture-RELATIVE | **discarded** — meaningless on another plane about a stale centre |

So two window starts: `first_chip` bounds `apsum`, `px_first_chip` bounds `bins`, and the spatial window
gets its own clamp and therefore its own phase. The track keeps correlating through the widen without the
position surface inheriting a lie.

**Honest reading of the result**: against the gate fix alone this is +18 fixes (noise) and relock
209 → 159 ms on **9 occlusions** — this clip's swing barely reaches the pole (0.3 % duty), so the
reacquisition axis is thin here. What justifies it is that it is the correct split, it costs nothing, and
it is *not* the first attempt's trade. A fixture with real occlusion duty (`pend60`, 11.5 %) would
measure it properly.

Free side effect: maintaining `apsum` incrementally in `track_frame` retires the per-tick re-summation of
`E*E*TRK_WIN` bins — per track 576·124·20 = 1.43 M adds/s replaced by 576·288 = 166 k, ~8.6× less on that
term. Arithmetic, not a measured speedup: replay is NVMe-bound at 478 MB/s and cannot show it.

## 3. The guard rescue throws away what it just collected — FOUND, NOT YET FIXED

`bank.c`'s guard rescue copies position, velocity and `cep` from a healthy GUARD onto a PRECISION track
in HOLD. It does **not** copy `last_fix_us`. So the rescued track gets a correct position and a small cep
and is then killed by `age_ms > hold_max_age_ms` on the very next tick, because age is measured from its
own stale timestamp — exactly the staleness the rescue just repaired.

Seen on the golden clip: a PRECISION track in HOLD at **q = 1.00**, rescued onto the beacon at age
150 ms, dead one tick later. Spec §2.4's "2 ticks instead of 1+ s" purchase never survived to be spent.

**A one-line fix was tried and backed out**, and the reason is recorded because it is a trap:
`last_fix_us = g->last_fix_us` assigned *unconditionally* can move the timestamp **backwards** when the
guard's own last fix is the older of the two, which aged the precision track prematurely and killed it
**earlier** than no rescue at all (golden: death moved f425 → f400). The guard must be
`if (g->last_fix_us > s->trk.last_fix_us)` — evidence only accumulates.

**Tried monotone, on top of the evidence split, and it is STILL a net loss.** I expected the split to
unblock it — a rescued track can now correlate through the widen — and predicted a win. Measured on
`pend_ir`, it is not:

| | evidence split | + monotone guard rescue |
|---|---|---|
| on-beacon fixes | **1770** | 1722 |
| false share | **1.0 %** | 1.3 % |
| within the 400 ms bar | **8/9** | 7/9 |
| relock p90 | **867 ms** | 1021 ms |

So the prediction was wrong and the mechanism is not simply "the rescue needs a correlatable track".
The likely reason, unverified: keeping the precision track alive holds a slot **and** keeps a CONFIRMED
track in the bank, and `engine.c` tests `have_confirmed` across *all* roles — so acquisition stays
suppressed while a rescued-but-not-recovering track coasts. On this fixture **dying and re-acquiring
cleanly beats being rescued**, which is a real statement about the current acquisition path being fast
(0.40 s) rather than about the rescue being wrong in principle.

**Do not try this a third time without first changing what it competes with.** The two candidates:
make `have_confirmed` role-aware so a HOLD-ing precision track does not suppress acquisition, or land
T081 (phase-known re-detection at 0.39 ms), after which the comparison is completely different because
re-acquisition stops being the expensive option. Reverted; not in the tree.

## What this does and does not move

It does not touch the coherence limit. **16.5 °/s is still 12 % decode** and the engagement is ~19 °/s;
that gap is T081/T082/T050's, and nothing here pretends otherwise. What it removes is the tracker losing
tracks it had already decoded — which is why the gains are largest at 2–5 °/s, where the beacon *is*
decodable and the lifecycle was throwing it away.

Bearing still fails the §3 0.3 ° bar at every rate on this lens: best block 0.43 ° at 10.6 °/s (the
0–20 s block is early, high-rate but well-lit), 0.50 ° at 2.5 °/s.

## Next, in order

1. **Guard rescue, monotone, on top of the evidence split** (§3) — the piece that is now unblocked.
2. **T084 ladder death-spiral** — `cep` says lost, `q` says weak; today `q` triggers both, and widening a
   weak-but-located track cuts SNR by √k, which cuts q, which widens further.
3. **T081 sync-first / receiver-global phase** — still the largest single win available, and still the
   event detector's prerequisite.
