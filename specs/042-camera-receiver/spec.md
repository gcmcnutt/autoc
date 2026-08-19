# 042 — camera receiver: a measured, robust beacon tracker

**Status**: shaped 2026-08-17 in the strategy discussion (operator + assistant). Supersedes the seed
(the seed is recoverable at `git show d96c293:specs/042-camera-receiver/spec.md`). Platform, exit criterion, and architecture are DECIDED below; the
phasing carries explicit split seams because this is a large feature.

**The ask, in the operator's words**: *"the outcome is a measureable robust tracker at our reduced frame
rates. get the algo in place and prove it … acquire the beacon as we saw in 031 studies, dropouts handled
as expected, all while estimating position of next beacon on the 2d frame of the camera … something like
20 Hz updates smoothly tracking items coming and going across the screen."*

---

## Clarifications

### Session 2026-08-19

- Q: In the §3 bar "≥95 % of 20 Hz ticks valid", does an extrapolated/HOLD tick count as valid? →
  A: **Option D — report two metrics** (bounded-extrapolation *valid-output rate* ≥95 %, plus a
  *measured-fix rate* floor ≥60 %). Operator rationale: acquisition is expensive and the target is
  crossing the field, so **the estimator must continuously re-affirm lock — confirming a chip is seen
  where it is expected, wherever the track is on the screen** — rather than inferring health once per
  word. See §2.6 and §3.1.
- Q: What is the end-to-end latency requirement (photon → record on the wire)? →
  A: **Option C — a delivery *deadline*, not a lag bound** ("C for now", revisitable): the record whose
  prediction targets tick N must be on the wire ≥5 ms before tick N; publish deadline-miss rate. See §11.1.
- Q: Where does 042 stop, given §13's seams? → A: **Option D — split E at its natural seam.**
  E1 = record + emit path, E2 = xiao transport + AHRS feed-forward. **042 = A, B, C, E1, D**; E2 → 043.
  Rationale surfaced by Q1/Q2: the envelope campaign (D) publishes §3.1's two metrics and §11.1's
  deadline-miss rate **through the record**, so E1 is a *prerequisite* of D, not a successor — §13 as
  originally written had D depending on its own successor. See §13.
- Q: What is the recorder's flight default, given the 3A+ cannot sustain full raw? →
  A: **None of the offered options — restructure.** Operator: *"if we plan for flight hardware at 453 Hz on
  a Pi 5 with NVMe storage, maybe this simplifies a bit."* It does: **continuous full raw is the flight
  design point**, and the 3A+'s RAM-ring + triggered dump becomes the *degraded bench mode*, not an
  architecture. The reduced/ROI/compressed-stream machinery is **not built unless something forces it**.
  See §8.
- Q: Confirm or move the §3 exit-band numbers, now that the flight article is a Pi 5 at 453 fps? →
  A: **Option D — the invariant is primary, °/s is derived.** Publish apparent motion per coherent
  integration interval, plus the °/s at the 250 fps bench point *and* the ×1.65 flight-scaled knee, in
  every envelope cell. Operator framing: *"we can do initial experiments on the RPi 3 we have today and
  the slower chip rate — but this is to prove **algorithms**, not absolute tracking time; sure, we'll slow
  the slew a bit to make it an honest test."* See §3.2.

---

## 1. The fact that reorganizes the feature

**The 031 tracker is a *static-beacon* correlator.** `beacon_track.py` integrates one full word per fix by
stacking *the same pixel* across M frames. At 121 Hz chips a word is **256 ms**; in the M2 grid
(0.304°/px) that is **25 px of smear at 30°/s and 1.7 px at 2°/s**. Above ~1–2 °/s the processing gain is
gone — the beacon sits under the pixel for 2–3 of 64 frames. No ROI, DPLL, or hold tweak fixes this; the
*integration model* is what has to change.

Two consequences ripple through everything:

- **DC removal moves from temporal to spatial.** Per-pixel ring-mean subtraction assumes a static scene
  under the pixel. Under slew — and especially under ego-motion, where the whole field streaks — it is
  wrong. Replace with a per-frame **spatial high-pass** (difference-of-boxes / annulus subtraction) at the
  sample point, then correlate the temporal code. Background edges move but do not blink Gold-31; the code
  does the clutter rejection.
- **Acquisition of a mover cannot be a full-field static matched filter.** Split it:
  **detect blink → form proto-track → decode along the track.** Frame-to-frame temporal difference is
  cheap and motion-agnostic (at 250 fps even 400°/s is 5 px/frame); a linear-motion RANSAC over ~one word
  yields proto-tracks; correlation along each gives code ID and phase. The full-field correlator survives
  only as the weak-signal / static fallback.

**Correction to a carried-forward estimate**: `firmware/beacon-receiver/README.md` predicts "~30 ms
full-field acquire" in C. Real arithmetic: 320×200 px × 64 taps × (31 phases × 2 codes) ≈ **123 MMAC per
pass** → **60–120 ms on one A53 core**, times several chip-rate hypotheses. Acquisition must be threaded
off the capture path regardless — another argument for detect-first.

---

## 2. Architecture — the movable multi-scale correlator bank

Carried in from the operator's FPGA-era design (2026-08-17): *"16 movable correlators which track
candidates and reject the invalids but have their own trackers — something without needing AHRS input —
if a big sub section shows a beacon in there and the tracker's goal is to center it, that naturally
builds momentum, especially if it's a stack of large, med, small, fine correlators."*

### 2.1 It is the spatial twin of the DPLL

Each instance owns a spatial aperture and a servo whose job is to **center** the beacon. Centering error
integrates into a velocity state — an **alpha-beta (type-2) loop**, zero steady-state lag under constant
slew, and the "momentum" falls out for free. The temporal side already does exactly this: the DPLL pulls
in chip rate and phase. Build them as the same shape — same lock/hold/drop state machine, same q-driven
bandwidth schedule, two axes instead of one.

### 2.2 The scale ladder — what it actually trades

Not aperture-sum vs aperture-sum. For a sub-pixel point source, correlating each candidate pixel
independently and taking the max costs **no SNR** (only a max-of-k threshold shift, ~√(2 ln k)·σ), whereas
*binning* to cover the same extent cheaply costs **√k** in background. So the ladder trades
**compute vs SNR vs precision**:

| scale | plane | extent | cost | penalty |
|---|---|---|---|---|
| coarse | 4× SW-binned | wide (reacquire / candidate) | cheap | √k background, coarse position |
| medium | 2× binned (the M2 grid, 0.304°/px) | ±16 px | moderate | √2 |
| fine | native 640×400 (0.148°/px) | ±4 px | k× per px, tiny extent | none |

**The regimes line up.** Near field = strong signal (can absorb the coarse penalty) and brutal apparent
rates (248°/s at 3 m). Far field = weak signal, but v/r makes apparent motion slow → small extent → the
fine scale is affordable. The ladder closes on its own.

**Where it does not close: weak + fast**, which is exclusively an ego-motion problem (039 flight data:
pitch-rate RMS 128–141°/s, roll to ~500°/s — [BACKLOG.md §Question 2c](../BACKLOG.md)). That single cell is
what AHRS feed-forward buys, and it is the reason to keep the hook — not the reason to start with it.

### 2.3 AHRS is optional feed-forward, never a dependency

The bank learns apparent motion whatever its source, so **build the whole tracker AHRS-free first**. Bolt
feed-forward on afterwards as a measured improvement (shortens pull-in, rescues large transients).
Discipline for the coordinate-convention swamp the operator flagged: **one convention doc, one transform,
one sign test on the rig** (command +pan, assert the predicted track moves the direction the beacon
actually goes). Because the term is feed-forward only, **a wrong sign degrades rather than breaks** —
that is the whole point of the ordering.

### 2.4 Bank management — two patterns committed

- **Candidate pool with lifecycle.** 16 slots; born from detection, promoted on sustained q, evicted by
  priority when full. The *code* kills false candidates (clutter, glints, room IR); Gold-31 cross-
  correlation is bounded at 9/31 (~10 dB), so A/B separation is solid. Metric: **false-acquire rate with
  the rig slewing over a cluttered room**.
- **Each confirmed track carries a pair: a fine "precision" correlator and a coarse "guard" at wider
  extent.** One extra slot buys the difference between *"transient knocked the fine tracker off → guard
  re-centers in 2 ticks"* and *"→ full-field reacquire, 1+ s outage."* Given 500°/s roll transients are a
  normal flight event, this is likely the highest-value robustness feature in the design.

The operator's jittery pan/pitch rig is a **feature**: jitter is broadband disturbance, and the q-driven
bandwidth schedule exists precisely so a narrow loop rejects it while a wide loop chases it. A rig that
was too smooth would not test the thing that matters.

### 2.5 The word-transit constraint — and why it re-frames the 453 fps gate

A word at 121 Hz is 256 ms; the field is 95° H. The beacon is in-field for **one** full word only if
ω < 95/0.256 = **371°/s**, and for **four** words (enough to actually track through) only if
**ω < ~93°/s**. At 453 fps / 200 Hz chips the word drops to 155 ms → **613°/s** and **~153°/s**.

**Therefore the 453 fps / Pi 5 step is a linear slew-tolerance multiplier, not primarily an SNR or
samples-per-chip story.** That makes the held Pi-5 gate quantitative: 042 measures the °/s knee at 250 fps,
and if the knee lands below the 128–141°/s body-rate RMS from 039, **the frame rate is the fix, not the
algorithm.**

### 2.6 Continuous chip-level re-affirmation (decision-directed lock health)

**Operator requirement, 2026-08-19**: *"acquisition time is quite long and the target is moving across
screen — we want the estimator to be able to continuously reaffirm, regardless of position on screen, that
a chip is seen where we expect and is thus solidly locked still."*

Acquisition is genuinely expensive: **1–2 words = 256–512 ms = 64–128 frames at 250 fps**, plus 60–120 ms
per full-field search pass × chip-rate hypotheses (§1, §10). Losing lock is therefore costly, and must be
detected *fast* and re-affirmed *cheaply*.

Once code, phase and chip rate are known the receiver is in a **decision-directed** regime: it knows
exactly which frames should be lit and which dark, and — from the track state — at which pixel. Every chip
boundary is therefore a **testable event**, giving a lock-health signal at the chip rate (~121 Hz) rather
than at the fix rate (20 Hz). Requirements:

- **Per-chip re-affirmation** at up to the chip rate, not per-word inference. Accumulate a fast
  lock-health statistic (exponentially-weighted agreement between predicted and observed chip polarity)
  alongside the slow full-word `q`.
- **Scale-free and field-position independent** — it must behave identically at the corner of the field as
  on axis, which means normalizing against the *local* noise floor (RI corner falloff is ~52 %, so a raw
  threshold would silently tighten off-axis). The same property `q` already has.
- **It drives the hold/drop decision**, replacing the 031 prototype's "ride 6 low-q reports" countdown:
  chip-level evidence says *when* lock actually broke, so a hold is justified by evidence rather than by a
  timer, and a genuine break is detected in ~1 chip instead of ~1 word.
- **It is also the `measured-fix` test of §3.1** — a tick is measurement-backed if chip re-affirmation
  covered it, which is why the two-metric bar is cheap to compute.

---

## 3. Exit criterion — three rate bands, delivered as a measured envelope

Not one number. A surface (validity vs slew × SNR), plus:

| band | regime | bar |
|---|---|---|
| **Tracked, full-word** | to ~90°/s | ≥95 % of 20 Hz ticks valid, ≤1 px (0.3°) RMS bearing residual, **zero code-ID swaps** |
| **Tracked, short-integration** (strong signal, adaptive length) | to ~150°/s | degradation *documented*, not asserted |
| **Transient survival** | 300–500°/s roll | a **reacquire event**: metric is time-to-relock ≤ 400 ms, not continuity |

Plus, in every campaign: **false-acquire rate** over cluttered background under ego-motion, and an
occlusion/relock ladder (0.3 / 0.6 / 1.0 s blocked).

> The °/s figures above are the **bench** column of §3.2 — the transferable form of each bar is the
> per-integration invariant, because 042 measures on the 3A+ while the flight article is a Pi 5 at 453 fps.

### 3.1 What "valid" means (clarified 2026-08-19) — two metrics, both published

A single validity number can be gamed by coasting: a tracker that extrapolates through an entire transit
without re-measuring would score 100 %. So **every envelope cell publishes both**:

| metric | definition | bar |
|---|---|---|
| **valid-output rate** | ticks carrying a usable estimate — a measured fix, **or** an extrapolation bounded at **age ≤150 ms (3 ticks) AND predicted CEP ≤3 px**. Outside either bound the tick is invalid regardless of flags. | **≥95 %** |
| **measured-fix rate** | ticks whose estimate is backed by a correlation measurement in that tick (not extrapolation) | **≥60 %** floor |

Both come free from the same run. The first reflects what 041 actually consumes; the second keeps a
degrading correlator from hiding behind a healthy Kalman filter.

### 3.2 The bar is an invariant; °/s is derived (clarified 2026-08-19)

**042 measures on a host that is not the flight article.** The bench is a 3A+ at 250 fps / 121 Hz chips
(word = 256 ms); the flight design point is a Pi 5 at 453 fps / 200 Hz (word = 155 ms). Raw °/s therefore
does not transfer — but **apparent motion per coherent integration interval** does, because what breaks a
correlator is smear *within* an integration, and °/px is fixed by the optics.

**Operator framing (2026-08-19)**: *"initial experiments on the RPi 3 we have today and the slower chip
rate — but this is to prove **algorithms**, not absolute tracking time; sure, we'll slow the slew a bit to
make it an honest test."* The reduced bench slew is **dimensional consistency, not a weakened bar.**

| band | **invariant (primary)** | bench °/s (3A+, 250 fps, word 256 ms) | flight °/s (Pi 5, 453 fps, word 155 ms) |
|---|---|---|---|
| **1 — full-word** | **≥23° of apparent motion per word** (≈76 px in the M2 grid) | ~90 °/s | ~148 °/s |
| **2 — short integration** (8 chips) | **≥10° per integration** (≈33 px) | ~150 °/s | ~248 °/s |
| **3 — transient** | survive a full-field (95°) transit; **relock ≤400 ms** — already host-independent | 300–500 °/s | 300–500 °/s |

The ≤1 px (0.3°) RMS residual and §3.1's two rates are dimensionless already and carry across unchanged.

**Every envelope cell publishes all three columns.** The bench number is the measurement; the invariant is
the claim; the flight number is the ×1.65 word-duration extrapolation that §2.5's gate reads.

**Honesty clause on "20 Hz"**: with full-word integration, consecutive fixes share ~92 % of their samples —
the *effective* information bandwidth is ~4 Hz and the Kalman prediction supplies the rest. Near field with
short adaptive integration gives genuinely independent 20 Hz fixes. The output record must carry
**age / independence** so 041 is not fooled about what it is holding.

---

## 4. The three coupled controllers (the "dynamic AGC" framework, generalized)

| controller | actuates | fights | cost |
|---|---|---|---|
| exposure | µs | well depth / saturation (near, sunny) | √t SNR |
| gain | analog | read noise (far, dark) — measured to buy **zero** against background (031 gain series) | headroom only |
| **integration length** | chips per fix | processing gain | **latency + motion smear** |

The third is the new one and it is the responsiveness lever. A full-word fix is inherently ~½ word old
(128 ms ≈ 2.5 ticks at 20 Hz). Once the DPLL holds phase, tracking does **not** need a full word: coherent
over K chips at known phase gives ×√(K·fpc) with unambiguous amplitude; **re-verify identity on a full word
every N ticks**. Integration length therefore becomes an SNR-driven loop output — near field runs 4–8 chips
(low latency, low smear), far field runs a full word.

**Two hard requirements that fall out of the design:**
- **Exposure/gain changes must not restart capture.** Today `--agc` kills and restarts `rpicam-raw`,
  dumping the ring. See §6.
- **Exposure must be driven by the tracked ROIs, not a global field statistic** — otherwise one sun glint
  (see §9) pins exposure down and starves every real track. Per-frame metadata gives the *actual*
  exposure/gain, so the sample series is normalized by (exposure × gain) before correlating and **AGC
  becomes transparent to the correlator**.

---

## 5. Descaling, defocus, saturation

Established in 031: the stock 640×400 pixel is a **2×2 analog bin on a 4×2 photosite footprint with 50 %
horizontal fill** — a sub-pixel source *amplitude-modulates* as it drifts across the column comb.

- **Defocus to PSF ≥ 2 output px** (√k background cost) — one answer.
- **Make the estimator comb-aware** — the ripple is a *known* function of sub-pixel phase, so estimate it
  jointly with position rather than fighting it. This is the better answer and it is free at the fine scale.
- **Measure it**: amplitude ripple at the 4-photosite pitch vs defocus, directly on the slew rig.
- **Saturation-aware estimator**: at high power a railed beacon spreads over several pixels — *helps*
  centroiding, *breaks* amplitude linearity. Switch from intensity-weighted centroid to a flat-top /
  disk-center estimator once the peak clips. (Operator: *"interesting to see honest field tests at high
  power whether the target blooms beyond one pixel"* — this is the estimator that answers it.)
- **8-bit is the tight axis**; 10-bit buys 4× headroom for ~20 % fps. Keep it a knob.

### 5.1 The goal state (10 ft trail) is the camera's worst cell — and the pod pair solves half of it

`TrailDistance = 3.048 m` (040 input-data-checklist) is the mission goal state, and **three things go
wrong there at once** (operator 2026-08-17):

- **Apparent rate peaks**: 244°/s for a 13 m/s crossing at 3 m (BACKLOG §Q2c says 248°/s) ⇒ the full 95°
  field crosses in **0.39 s** — barely one and a half words at 121 Hz. §2.5's constraint bites hardest
  exactly at the goal state.
- **Integration time is therefore minimal** — the adaptive short-integration mode (§4) is not an
  optimization here, it is the only mode that works.
- **The beacon saturates hard.** Extrapolating the 031 exposure ladder (bench current, worst aspect, 2 m →
  ~200 µs), a flight cube at 306 mA face-on at 3 m wants roughly **10–20 µs** — a couple of line times
  above the OV9281's floor. Inside ~2 m the exposure controller **bottoms out and saturation is
  permanent**. Not fatal (a railed signal still modulates, so the code correlates fine); what breaks is
  amplitude linearity and the intensity-weighted centroid ⇒ **§5's flat-top / disk-centre estimator is
  load-bearing at the goal state, not optional.**

**The compensation: at this range the two-code pod pair is a rangefinder.** Separation subtends
2·atan(b/2r). For a 1 m pod baseline, at ~0.7 px combined centroid error:

| range | subtended | M2 px | range precision |
|---|---|---|---|
| 3.05 m | 18.6° | 61 | **±1.1 % (±3 cm)** |
| 10 m | 5.7° | 19 | ±3.7 % |
| 30 m | 1.9° | 6.3 | ±11 % |
| 100 m | 0.57° | 1.9 | ±37 % |

**Monocular — no stereo, no ToF.** It needs only what 042-B already produces: two coded tracks with
sub-pixel centroids. Aspect foreshortening (b_apparent = b·cos aspect, biasing range long) is the usual
killer of this technique and it **vanishes exactly where the measurement is wanted** — in a dead-astern
trail the aspect is ~0 by definition. Useful to ~30 m; excellent inside 10 m.

**Consequence for the sensor ledger**: for a *cooperative* target the pod pair already covers endgame
ranging better than the VL53L9CX does at that distance, so **the ToF's unique justification collapses
cleanly onto the non-cooperative case** (§9) — the one thing no other sensor here can supply. Two reasons
to add a sensor became one, and it is the durable one.

*(042 does not build the rangefinder — it just must not preclude it: emit both tracks' sub-pixel centroids
and the pod baseline `b` as a config constant, which the §11 record already does.)*

---

## 6. Camera path — direct libcamera (DECIDED)

`beacon_trackd` drives the camera directly rather than consuming an `rpicam-raw` pipe (operator
2026-08-17: *"we'll likely be driving the camera directly"*). What that buys:

- **per-frame metadata** (exact exposure/gain → the normalization in §4),
- **request-level control with no capture restart** (glitchless AGC),
- **zero-copy DMA buffers**.

The `rpicam-raw` pipe path is **kept as the replay/regression harness** and as a fallback.

Real-time hygiene on the 3A+: SCHED_FIFO on the capture + front-end thread, core pinning (dedicate one of
four A53s to capture/front-end, one to the correlator bank), `mlockall`, pre-allocated buffers, no
allocation in the loop. Thermals: sustained `rpicam-raw` already pegs a core; the 3A+ has no heatsink.

---

## 7. Prove-it — ground truth without building a simulator

Operator direction: *"most of this doesn't need to be simulated so we can focus on reliable tracking."*
Accepted — no renderer, no physics. But an envelope still needs truth, and there are three cheap sources,
all riding on the replay harness:

1. **Offline oracle on the same recording.** Unlimited compute, full-field every frame, **non-causal**
   smoothing over the whole clip. Score the real-time causal tracker against it. Same data, no latency
   budget, no camera model required. *This is the primary truth source.*
2. **Signal injection into real recorded background.** Add a coded point source on a known trajectory at a
   chosen amplitude into real frames. Not a simulation — a test-vector generator, a few dozen lines — and
   the only way to reach the weak-signal end of the envelope before range and filters exist. Gives exact
   CEP ground truth.
3. **The pan/pitch rig's commanded profile** — good rate truth, weaker position truth given the jitter.

---

## 8. Raw capture — the recorder (restructured 2026-08-19)

Operator: *"we do want all flight tests to capture all raw data for ground replay analysis"* — and
*"if we plan for flight hardware at 453 Hz on a Pi 5 with NVMe storage, maybe this simplifies a bit."*
It does. **There are two hosts, and the recorder is designed for the flight one.**

### 8.1 The arithmetic

| host / mode | rate | sink capability | verdict |
|---|---|---|---|
| **Flight: Pi 5 + NVMe, 640×400 @ 453 fps** | **116 MB/s** | NVMe PCIe 2.0 ×1 ≈ 450 MB/s (~26 % used) | **continuous full raw — comfortable** |
| Flight: Pi 5 + NVMe, 640×200 @ 453 fps | 58 MB/s | same | trivial |
| Bench: Pi 3A+, 640×400 @ 250 fps | 64 MB/s | SD ≪; USB2 ~35 MB/s; 512 MB RAM | **~8 s RAM ring only** |
| Bench: Pi 3A+, 640×200 @ 250 fps | 32 MB/s | as above | ~16 s ring, narrower V field |

Capacity at the flight design point: a 10-minute sortie ≈ **70 GB**; a 1 TB drive ≈ **2.4 hours** at full
rate. Storage is not a constraint on the flight article.

### 8.2 The design that falls out

**One recorder, one pluggable sink, one bounded in-RAM queue.**

- If the sink keeps up (NVMe) → **continuous full raw**. This is the flight default.
- If it cannot (3A+ SD/USB2) → the queue *is* the RAM ring, dumped on trigger (lock-loss, event, manual).
  Same code path, one config knob; the degraded mode is a consequence of the sink, not a separate feature.
- **Do NOT build** the ROI+context crops, decimated-context frames, high-passed streams or LZ4/delta
  compression contemplated earlier. They were bandwidth management for a host being replaced. *(Noted for
  the record: full-field compression was weaker than first assumed anyway — in flight the camera is always
  moving, the whole field streams, and temporal-delta collapses to ~1.5–2× on raw. And a high-passed
  stream discards the absolute levels that the later photometry, saturation and multipath-phenomenology
  work needs.)*

### 8.3 Two consequences to design for, not discover late

- **IO jitter vs the §11.1 deadline.** Writing ~116 MB/s continuously on the same SoC as a hard-real-time
  correlator can threaten the delivery deadline. Mitigations, specified now: writer on its own core,
  pre-allocated extents / `O_DIRECT`, large aligned writes, and the bounded RAM queue absorbing burst
  stalls. Deadline-miss rate (§11.1) is the metric that proves it.
- **Mass and power of the flight host.** Pi 5 + active cooler + NVMe HAT + SSD is roughly **100–130 g**
  against the 3A+'s ~30 g, with a materially higher sustained draw. That is an airframe budget question for
  the flight article — filed as an open item (§15), **not a 042 question**.

---

## 9. Multipath / the pond — research thread (operator 2026-08-17)

*"our park does have a small body of water … chasing a target across the pond and the tracker locking onto
reflections from the pond surface or even damp grass."*

**Why it is a real hole, not a corner case**: every other discriminator in this design is the Gold code —
and a specular reflection **carries the same code**. The extra path is a few metres, ~10⁻⁹ of a chip at
115–200 Hz, so **code phase is blind to it**. The DPLL will lock the ghost and report high q. And Fresnel
reflectance → ~1 at grazing incidence, which *is* the low-across-the-pond geometry — so expect a
**near-equal-brightness twin**, not a faint artifact. **Amplitude thresholds will not save you.**

**What discriminates, ranked (the first two are free from the ladder already being built):**

1. **Spatial extent — `q_fine / q_coarse`.** A real beacon is a point source: q peaks at the *fine* scale.
   A water reflection is a glitter path — elongated, boiling, smeared along the specular direction by
   ripple — so it peaks at the *coarse* scale. Costs nothing; both are computed anyway. Probably the best
   single-camera discriminator.
2. **Scintillation.** Ripple makes the specular return glint in and out as facets align → far higher
   amplitude variance and centroid jitter than the direct path. Also free from state already kept.
3. **Mirror-pair geometry.** Two tracks, same code, similar azimuth, elevations straddling a common
   surface, **anti-correlated vertical motion** ⇒ a direct/reflected pair; with the camera above the water
   the real one is the **upper**. Pure track-level logic. **Flag, do not delete** — if the direct path is
   momentarily occluded, a flagged ghost fix beats no fix as long as 041 knows what it holds.
4. **Stereo disparity — the decisive one, already on the roadmap.** The virtual source sits below the water
   plane at longer path length ⇒ different disparity; the pair resolves both to 3-D and one is four metres
   underground. **This answers the "too many sensors" worry: the multipath solver is the dual-CSI birded
   pair already wanted for parallax ranging and photon doubling.** Not another sensor — the one already
   committed, earning a third keep.

**Polarizer** — the one cheap passive experiment: near Brewster (~53° incidence for water) the p-polarized
reflection collapses, so a correctly oriented linear polarizer suppresses the ghost for a 3 dB cost on the
unpolarized direct signal. It helps *least* at grazing — least in the worst case — so: bench experiment,
not a commitment.

**The pond as a background**: water is near-black in NIR (it is the standard remote-sensing water mask), so
diffuse background over the pond is *low* — good contrast for the direct path and equally good for the
ghost. And **the pond hosts a second sun**: solar specular glint at 850 nm is brutal and it *moves*. The
031 sun-transit max-energy gate (bench-journal 2a) anticipated one sun; the pond makes it two, one of them
scintillating. → the ROI-driven exposure requirement in §4.

**The one nice thing**: the direct/reflected angular split is a **height-above-water measurement** (the same
geometry radar altimetry exploits) — a free height-over-terrain cue in the endgame. Not worth chasing now;
worth not designing out.

### What 042 actually does about it (three cheap things; everything else parked)
1. **Record the fields now** — per-track `extent` (q_fine/q_coarse), `scintillation` (q variance over a
   window), `multipath_suspect` flag in the 20 Hz record. Free while the record is being defined; painful
   to retrofit once 041 consumes it.
2. **The same-code pair rule** in track management — flag the lower, keep the upper.
3. **Get the data.** The recorder exists anyway (§8), so a **pond session costs an afternoon** and makes
   later research possible without re-flying. And the phenomenology is available *this week* on the bench:
   **a tray of water under the emitter on the pan rig** gives a real specular pair at controlled geometry —
   enough to measure whether `q_fine/q_coarse` actually separates them before committing to it.

### ToF — parked to M3/M4 (operator 2026-08-17: **ST VL53L9CX, ~9 m / 30 ft**)
The physics favours ToF: metres of path difference is exactly what it resolves and code phase cannot.
The part is the **ST VL53L9CX** — 2-D zone array, **~9 m (30 ft)** range (corrected from the initial
"30 m"; *"an impressive piece of hardware to use for target locked-in mode"*). Two roles, both M3/M4:

- **Terminal / locked-in ranging.** 9 m at 13–25 m/s closure is ~0.4–0.7 s of engagement — this is a
  *last-second* sensor, for impact geometry and terminal guidance, not for tracking.
- **Non-cooperative targets — and after §5.1, this is the *only* durable justification.** The entire
  031/042 chain discriminates by Gold code, so it only ever sees a target that *broadcasts*. A 2-D ToF
  array sees geometry, not cooperation. (For a cooperative target the monocular pod-pair rangefinder of
  §5.1 beats it at the ranges the ToF can reach.)

**The fit is better than it first looks: the ToF's window is exactly where the beacon correlator is
weakest.** Near field means maximum apparent angular rate (248°/s at 3 m) and therefore minimum usable
integration time — the corner §2.5 says the camera cannot integrate through. ToF is per-frame geometry
with no code lock to lose, so the two sensors fail in opposite directions. (It would also resolve pond
multipath trivially at that range — but stereo already covers that, so it is not the justification.)

⚠ **Still to check against the datasheet** before any design leans on it: FOV (this family is typically
~60–65° diagonal — narrow vs the 95° camera field), zone count, frame rate vs the 20 Hz control tick,
performance against bright sunlight and against water, mass and power. **M3/M4 scope, not 042.**

---

## 10. Compute budget and the FPGA question

| item | cost | note |
|---|---|---|
| tracking (phase + code known) | 31 MAC / position hypothesis → 16 trackers × 64 hyp × 20 Hz ≈ **0.6 MMAC/s** | free |
| identity re-verification (31 phases × 2 codes) | ~123 kMAC / tracker at ~1 Hz ≈ **2 MMAC/s** | free |
| per-frame front end | 64 Mpx/s × ~10 ops/px ≈ **640 Mops/s** ≈ 20 % of one A53 with NEON | the real per-frame budget |
| **cold full-field acquire** | **~123 MMAC ⇒ 60–120 ms/pass**, × chip-rate hypotheses | **the actual cost** |

**Provisional verdict: the correlator bank does not need an FPGA on this host.** The binding constraints
are **capture fps** and **cold-acquire latency** — a Pi 5 addresses both. 042's deliverable is the measured
table that turns this from an opinion into a decision, with the trigger stated explicitly (§2.5).

---

## 11. Interfaces

**The 20 Hz record** — versioned, fixed-size, fixed-shape *every* tick (never a gap), with explicit
validity flags:

`t_us · seq · n_tracks ·` per track: `code_id · x · y` (M2 grid, 320×200 @ 0.304°/px, centre (0,0),
+x right / +y down — 041 `camera_projection.h`) `· vx · vy · x_pred/y_pred at the NEXT control tick ·
cep · q · lock_health (fast, chip-rate; §2.6) · extent · scintillation · flags{lock, hold, extrapolated, multipath_suspect, saturated} ·
age_ms · independence`

- **JSON lines** for the DGX display (existing `beacon_display.py` convention: **code A = PORT/red,
  code B = STARBOARD/green**).
- **Binary over UART now, I2C slave when the xiao takes sensor inputs.** The link is **bidirectional** in
  the end state: fixes out, AHRS rates in (§2.3).
### 11.1 Latency is a delivery deadline, not a lag bound (clarified 2026-08-19, "for now")

For a control-loop sensor what matters is not absolute lag but whether the estimate **arrives before it is
needed**. The record already predicts to the next tick (§11), which removes the lag by construction, so
the constraint is on delivery:

| requirement | value |
|---|---|
| **Deadline** | the record whose prediction targets tick N must be on the wire **≥5 ms before tick N** |
| **Deadline-miss rate** | **<0.1 %**, published with every envelope cell alongside §3.1's two metrics |
| internal budget — capture + front end | **≤1 frame (4 ms at 250 fps)**, hard real-time (§6) |
| internal budget — correlate + emit | **≤1 tick (50 ms)**; may run off the capture thread behind a queue *provided the deadline holds* |

Consequences: `age_ms` in the record remains **reported** (041 still wants to know how stale the
measurement behind the prediction is) but is no longer a *constraint* — the deadline is. And adaptive
integration length (§4) is bounded from above by nothing but this deadline plus the §3.1 measured-fix
floor, which is the intended coupling.

*Marked "for now" by the operator — revisit once 041's control-loop sensitivity to sensor lag is measured.*

- **To 041**: the important handoff is not a camera model — it is the **measured tracker error model**
  (CEP and dropout statistics vs SNR, slew rate, field position, latency), so 041 injects a measured error
  process on top of the ideal bearing rather than running a correlator in sim. That is also how the sun
  comes back in: crrcsim gives sun-in-field geometry; one outdoor session through the 850 filter gives
  background-vs-sun-angle → SNR → dropout probability. **Two measured parameters instead of a story.**

---

## 12. Platform decision — two hosts, and they are not the same host

**042 development / bench host: Pi 3A+ at 250 fps / ~115–121 Hz beacons.** Every algorithmic question is
per-frame and scales with the host, so 042 proves the *algorithm* here (§3.2).

**Flight-article design point: Pi 5 + NVMe at 453 fps / 200 Hz chips** (dual CSI, ~3× CPU, continuous full
raw — §8). Stated 2026-08-19 so the recorder and the exit-bar arithmetic are designed against the right
target.

**These are not in conflict, and the gate is unchanged**: the Pi 5 is still *bought* on the °/s knee
measured in 042-D (§2.5) — to confirm scaling and unlock dual CSI, not to discover whether the tracker
works. What changed on 2026-08-19 is that the flight article is now *planned* as a Pi 5, which is what
lets §8 stop engineering around a 512 MB / SD-card sink and §3.2 publish a flight-scaled column.

---

## 13. Phasing, with split seams

**042-A — recorder + camera app skeleton.** Direct libcamera, per-frame metadata, SCHED_FIFO + core
pinning, RAM ring, triggered dump, reduced continuous stream, replay path byte-identical to live. *Ships
the flight recorder as a side effect; everything downstream leans on it.*

**042-B — the correlator bank.** Movable multi-scale instances, alpha-beta centering loop, q-driven
bandwidth/scale schedule, DPLL, adaptive integration length, candidate lifecycle, guard+precision pairing,
`q_fine/q_coarse` extent output. Proven on replayed rig data against the offline oracle. *(Bench water-tray
multipath probe rides here.)*

**042-C — acquisition.** Blink-detect → proto-track → decode-along-track, multi-rate, threaded. Plus the
**second code** — a breadboard ATtiny412 + 2 LEDs is enough at 2–5 m, **do not wait on the A7 cubes** — and
the near-far test (railed near beacon leaking into the weak one's ROI; stretch: successive interference
cancellation).

> **— split seam 1: A+B+C is "a tracker that provably tracks" —**

**042-E1 — the record + emit path.** The 20 Hz versioned fixed-shape record of §11, its §11.1 delivery
deadline and miss-rate instrumentation, and the §3.1 two-metric scoring that reads it. JSON lines to the
DGX display. **Moved ahead of D (clarified 2026-08-19)**: the envelope campaign publishes its metrics
*through* this record, so it is a prerequisite of the measurement, not a successor to it.

**042-D — the envelope.** Rig campaign across the three rate bands, cluttered-background false-acquire
rate, occlusion/relock ladder, per-frame and per-fix cost table, **the °/s knee that gates the Pi 5**. The
deliverable the operator asked for: a measured surface, not an anecdote. *(Pond raw-capture session rides
here.)*

> **— split seam 2: end of 042 —**

**042-E2 → 043 — transport + AHRS feed-forward.** Binary over UART / I2C slave to the xiao, AHRS rates in
as optional feed-forward with the sign test (§2.3). Deferred because nothing in the 042 exit criterion
measures through it, and because AHRS is deliberately the *last* thing built (§2.3).

**Scope of record (decided 2026-08-19): 042 = A · B · C · E1 · D.** Everything photometric, plus E2, to
043 — A–E1–D is the coherent claim ("robust tracker, measured") and D is what makes the 043 gates
quantitative.

---

## 14. Explicitly deferred

**To 043**: daylight — sky background through the 850 filter (031 empirical #1, **still open**); range;
the saturation/blooming and defocus/descaling study at high power; exposure/gain controller *tuning*
against real conditions (the framework is 042); 453 fps on Pi 5 (042 does not *build* on the Pi 5 even though §12 names it the flight
design point); the FPGA decision, closed by the 042-D table; the tracker-error-model handoff that lets 041 put sun behaviour back in the sim; true 200 Hz chip
rate with Gold-31.

**To backlog / M3–M4**: ST **VL53L9CX** multizone ToF (~9 m) for terminal locked-in ranging and
**non-broadcasting targets**; stereo multipath
rejection; the polarizer experiment; the height-above-water inversion; the 120° single-lens question
beyond measuring the ELP-L156; the flight recorder as a product.

**Not 042**: Pi 5 / FPGA host; simulation of the camera (operator: not needed — focus on reliable
tracking).

---

## 15. Open items

1. ~~Phasing seam~~ — **RESOLVED 2026-08-19**: 042 = A · B · C · E1 · D (§13).
2. ~~Exit-band numbers~~ — **RESOLVED 2026-08-19**: invariant primary, °/s derived, all three columns
   published (§3.2).
3. ~~Recorder default~~ — **RESOLVED 2026-08-19**: continuous full raw on the Pi 5 + NVMe flight host;
   3A+ ring is the degraded bench mode (§8). **New open item**: Pi 5 flight-host mass/power (~100–130 g
   vs the 3A+'s ~30 g) against the airframe budget — flight-article question, not a 042 one (§8.3).
4. ST VL53L9CX (§9): pull the datasheet for FOV / zones / frame rate / sunlight — M3-M4, no rush.

---

## Inputs / references

- `specs/031-beacon-camera/camera-era-knobs.md` — the physics contract (photon/SNR, ¼-power code-rate law,
  blind-time table, filter verdict, sampling/defocus, measured f·θ 95×61, gain series)
- `specs/031-beacon-camera/handoff-041-camera-model.md` — the 041 camera-model handoff
- `specs/031-beacon-camera/bench-journal.md` — living bench state (instruments, traps, field tests)
- `specs/031-beacon-camera/WRAP.md` — what leaves 031
- `firmware/beacon-receiver/` — README, HIGH-FPS-PLAN (per-frame ceiling + Pi 5 verdict), `pi/` tools
  (`beacon_track.py`, `beacon_display.py`, `live.sh`, `focus_view.py`, `fps_probe.py`, patched `ov9282.c`)
- `specs/BACKLOG.md` §Question 2c — 039 flight body rates (pitch RMS 128–141°/s, roll ~500°/s) and target
  angular rates (248°/s @3 m, 74°/s @10 m, 30°/s @25 m)
