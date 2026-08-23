# Sync-first acquisition — split the synced and unsynced regimes (2026-08-22)

**Operator**: *"Is there a strategy where we first grab a good sync and then execution motion both in and
out of field? In this case we know what to look for while searching. And then we work the unsync case
differently. Eg if our pll is locked we can now do broad and narrow search continuously."*

Yes — and the cost arithmetic says it is not a refinement, it is the answer. **A phase-known full-field
matched filter costs 0.39 ms.** Everything follows from that.

## The proof of concept already exists and we have been using it all day

`tools/oracle.py` is exactly this detector, and it is how every ground truth in this feature was produced.
On the same scenes where the live blink detector ranks the beacon **126th–230th** and acquisition is a
lottery, the oracle finds it unambiguously at q_rel ~5000. Verified again today: with the tracker unable to
hold lock at all, a 4 s recording gave **q_rel 4860–5293** at a rock-steady M2 (4.00, 28.00).

The receiver therefore already contains a detector that works and a detector that does not. The strategy is
to promote the former to full-field and let the latter go.

## Why it is affordable — cost by what is KNOWN

| what is known | cost per full-field evaluation | on 1 core |
|---|---|---|
| nothing — 31 phases × 2 codes (cold) | 123.0 M MAC | **24.3 ms** |
| code only (bench is code B) | 61.5 M MAC | 12.1 ms |
| **phase + code — i.e. SYNCED** | **2.0 M MAC** | **0.39 ms** |

Supporting costs: one add per pixel per frame to bin it = **18.4 M adds/s**; full-field chip bins =
64 000 px × 31 × 4 B = **7.9 MB**. Both trivial on a Pi 5.

**0.39 ms means the synced search can run EVERY TICK over the WHOLE FIELD** — the operator's "broad and
narrow search continuously", literally, with ~36 ms of tick margin to spare.

## Cold acquisition stops being a lottery

| | |
|---|---|
| fill one code word | 258 ms |
| full 31-phase × 2-code search | 24.3 ms |
| **total, deterministic** | **283 ms** |

Against the §3 relock bar of **400 ms** — met, cold, with no ranking, no seeding lottery, and no dependence
on the beacon being among the top-3 strongest blinkers. Compare today's measured **2.10–8.30 s**.

Note what this kills: the 60–120 ms full-field figure in spec §1 that justified detect-first was an **A53**
estimate. On the A76 it is 24 ms, and the whole "acquisition is too expensive to do directly" premise
weakens considerably.

## Phase survives being blind — this is the enabling invariant

The platform is **single-rate** (120 Hz, pinned in pod, gateware and receiver), so the only temporal unknown
is phase. The pod runs 120.0077 Hz against a receiver nominal of 120.0 — **64 ppm**:

| time out of field | phase drift | |
|---|---|---|
| 1 s | 0.008 chip | fine |
| 10 s | 0.077 chip | fine |
| 60 s | 0.462 chip | fine |
| **65 s** | 0.5 chip | **resync needed** |

**A beacon can leave the field for a minute and come back to a still-valid phase.** That is precisely the
"execution motion both in and out of field" case, and it costs one 0.39 ms correlation to re-detect.

## The regimes, and what each one does

**UNSYNCED (cold, once):** accumulate one word, run the 24 ms full search over 31 phases × 2 codes, take
the phase. Off-thread. Happens at startup and after >65 s blind.

**SYNCED (the normal state):** phase is receiver-global state, not per-track — there is one emitter and one
clock. Every tick, cheaply and continuously:
- **broad** — full-field, phase-known, coarse plane: "is it anywhere?"
- **narrow** — fine plane around each track's prediction: "exactly where?"

Re-acquisition after a lost track, an occlusion, or a field exit is then a **broad hit**, not a cold start.

## What this changes in the plan

- **It largely supersedes T080** (the |Δ²| detector). That was a good fix to candidate *ranking* — measured
  rank 1842→1 still, 276→2 moving — but ranking only matters because seeding is a lottery. A phase-known
  matched filter has no ranking step: the beacon wins by construction, because nothing else in the room
  modulates Gold-31 at that phase. Keep the study as evidence; do not build it if this lands first.
- **It makes T050 (track-before-detect) ~31× cheaper**, because velocity hypotheses need testing at ONE
  phase instead of 31. TBD is still required for the fast-motion case — sync does not fix smear.
- **It does not fix the coherence limit.** The 258 ms window fill still requires the beacon to sit in a
  pixel for a word, so the measured 1–2 °/s knee stands. Sync-first is the answer for cold start,
  re-acquisition and in/out-of-field; decode-along-track remains the answer for tracking through motion.
- **The `chip_phase` hook is already half-built**: `bank.c`'s guard allocation does
  `slots[g].trk.chip_phase = s->trk.chip_phase; /* inherit the lock — that is the point */`. This
  generalises that from guard-inherits-precision to receiver-global.

## Honest risks

- **False alarms over the whole field.** Correlating 64 000 pixels at a known phase means 64 000 chances
  for noise to clear threshold. Gold-31 autocorrelation is bounded at 9/31 (~10 dB) which is the defence,
  but the false-acquire rate must be measured (T058) — this makes that task load-bearing rather than
  optional.
- **Memory bandwidth, not MACs.** 7.9 MB of bins touched every frame will not sit in cache; the measured
  `hipass M2` at 0.672 G op/s is already memory-bound and is the warning. The MAC arithmetic above is a
  floor, and the real cost needs measuring before this is promised.
- **Phase must be re-verified, not assumed.** A 65 s validity window is a *drift* bound, not a correctness
  one — an emitter reset or a pod reflash invalidates it instantly and silently. Re-affirm continuously
  (§2.6 already wants this) and fall back to the cold search on sustained failure.

---

## Addendum 2026-08-22 — the pattern, after two operator corrections

### Correction 1: my 65 s holdover figure was open-loop, and therefore wrong for a PLL

**Operator**: *"We have a pll so if clocks are reasonably stable. We prob get more than 65 secs."* Correct.
The 65 s came from the raw nominal-vs-actual offset (64 ppm) with **no correction at all**. A loop that
estimates frequency **nulls that constant** — what remains is the *residual*:

| residual frequency error | time to 0.5 chip of drift |
|---|---|
| 64 ppm (uncorrected — **today**) | 65 s |
| 10 ppm | 7 min |
| 1 ppm | **69 min** |
| 0.1 ppm | 11.6 h |

So holdover is **minutes to hours**, not a minute — *once a rate estimate exists*.

**But it does not exist today.** The DPLL's rate half is deliberately **PARKED** (bench journal trap #4):
two live attempts destabilised it — naive `dhz` walked 115→109 Hz, and the epoch-re-anchor repair sprayed
112–129 Hz. Only phase adoption runs. So **today's real holdover IS the open-loop 65 s**, and the long
figures above are the prize for un-parking it, not a current property.

The unlock: the journal's own condition for revisiting was *"sim-first work against golden clips, never live
tuning."* **That precondition is now met** — `pan2.bcnr` and the static clips are deterministic replay
fixtures with fiducial ground truth at 0.18°. The rate loop can now be developed offline against a known
answer, which is exactly what it lacked when it burned two attempts.

### Correction 2: the camera is the sampler, and 42 % of frames straddle a chip edge

**Operator**: *"since camera is sampler we need to continuously reduce on whatever edge we see. Carefully."*

At 2.3992 frames per chip the frame grid and the chip grid are incommensurate, and the 120 Hz retune made
the pattern **deterministic: 12 frames per 5 chips**. Within that cycle:

- **5 of 12 frames straddle a chip boundary — 42 % of all samples.**
- `track_frame()` currently **hard-assigns** each frame to one chip via `corr_chip_at()`, so a straddler can
  put up to **50 % of its light in the wrong chip**.
- Simply discarding straddlers costs **2.34 dB** of the word's 14.9 dB processing gain — not acceptable.

The right treatment weights each frame by its overlap with the chips it spans. And the useful part:
**the straddlers ARE the sub-chip phase discriminator.** A straddling frame's value is a mix of two chip
polarities, and the mixing ratio says where the edge sits *within* the frame — sub-frame phase resolution
from data already being collected. That is the "continuously reduce on whatever edge we see", and it is the
natural error signal to drive the rate loop with.

**"Carefully" is the operative word**: this discriminator is what the parked rate loop lacked, and feeding a
loop from a noisy edge estimate is precisely how the last two attempts diverged.

### The pattern

**Temporal before spatial, and temporal is GLOBAL.** There is one emitter and one clock, so phase and rate
are receiver-level state, not per-track. Spatial estimators then run inside a *solved* temporal frame —
which is the operator's *"if we assume locked then motion should work well as predictors can work from
there."* The predictor stops having to solve when-and-where simultaneously.

Three temporal states, each with a different search:

| state | what is known | search | cost |
|---|---|---|---|
| **UNSYNCED** | nothing | full field × 31 phases × 2 codes | 24 ms, once, off-thread |
| **PHASE-LOCKED** | phase | full field, phase-known, broad + narrow | **0.39 ms every tick** |
| **RATE-LOCKED** | phase + frequency | as above, plus **long holdover** through blind periods | same |

Re-entry to the field, occlusion recovery and lost-track re-acquisition are all **broad hits in the
phase-known regime**, never cold starts — as long as holdover has not expired.
