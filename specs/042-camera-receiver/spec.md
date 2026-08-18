# 042 — camera receiver: a measured, robust beacon tracker

**Status**: shaped 2026-08-17 in the strategy discussion (operator + assistant). Supersedes the seed
(the seed is recoverable at `git show d96c293:specs/042-camera-receiver/spec.md`). Platform, exit criterion, and architecture are DECIDED below; the
phasing carries explicit split seams because this is a large feature.

**The ask, in the operator's words**: *"the outcome is a measureable robust tracker at our reduced frame
rates. get the algo in place and prove it … acquire the beacon as we saw in 031 studies, dropouts handled
as expected, all while estimating position of next beacon on the 2d frame of the camera … something like
20 Hz updates smoothly tracking items coming and going across the screen."*

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

## 8. Raw capture — the recorder, and a reality check on this host

Operator: *"we do want all flight tests to capture all raw data for ground replay analysis."* Committed.
The arithmetic that shapes it:

- 640×400 × 8 bit × 250 fps = **64 MB/s**. The SD card cannot take it; the 3A+ is USB2 (~35 MB/s real);
  512 MB RAM ⇒ about an **8-second** ring.
- 640×200 (the patched mode) halves it to 32 MB/s ⇒ ~16 s ring, at the cost of vertical field.

Design (config-selectable, and the shape survives the host upgrade):
- **RAM ring + triggered dump** — last N seconds, dumped on lock-loss / event. Works today, catches the
  interesting moments.
- **Continuous reduced stream** — the high-passed frame is mostly zeros; temporal-delta + LZ4 on an
  IR-filtered scene should reach 5–10×. Lossy w.r.t. raw truth, lossless w.r.t. what the tracker consumes.
- **Full continuous raw is a Pi 5 + NVMe capability**, not a 3A+ one — another line in the Pi 5 column.

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

### ToF — parked to M3/M4 (operator note 2026-08-17)
The physics does favour ToF: metres of path difference is exactly what it resolves and code phase cannot.
**Operator note: ST has a ~30 m-range multizone ToF with a 2D array** *(part number to VERIFY — the
commodity VL53L5/L7/L8CX multizone family is ~4 m class; the 30 m-class part is a newer/longer-range SKU
and must be confirmed against a datasheet before any design leans on it).* Noted before as
**the mode to use close-in, and for the case where the target does NOT broadcast its position** — i.e. a
non-cooperative-target sensor. **That is M3/M4 scope, not 042.** Filed to the backlog.

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
cep · q · extent · scintillation · flags{lock, hold, extrapolated, multipath_suspect, saturated} ·
age_ms · independence`

- **JSON lines** for the DGX display (existing `beacon_display.py` convention: **code A = PORT/red,
  code B = STARBOARD/green**).
- **Binary over UART now, I2C slave when the xiao takes sensor inputs.** The link is **bidirectional** in
  the end state: fixes out, AHRS rates in (§2.3).
- **To 041**: the important handoff is not a camera model — it is the **measured tracker error model**
  (CEP and dropout statistics vs SNR, slew rate, field position, latency), so 041 injects a measured error
  process on top of the ideal bearing rather than running a correlator in sim. That is also how the sun
  comes back in: crrcsim gives sun-in-field geometry; one outdoor session through the 850 filter gives
  background-vs-sun-angle → SNR → dropout probability. **Two measured parameters instead of a story.**

---

## 12. Platform decision (carried in, unchanged)

**Pi 3A+ at 250 fps / ~115–121 Hz beacons** until the tracker proves itself. The Pi 5 (dual CSI, ~3× CPU,
NVMe, likely 453+ fps with the patched 640×200 mode) is the **scaling step, gated on the °/s knee measured
in 042-D** (§2.5) — bought to confirm scaling, not to discover whether the tracker works.

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

**042-D — the envelope.** Rig campaign across the three rate bands, cluttered-background false-acquire
rate, occlusion/relock ladder, per-frame and per-fix cost table, **the °/s knee that gates the Pi 5**. The
deliverable the operator asked for: a measured surface, not an anecdote. *(Pond raw-capture session rides
here.)*

**042-E — interfaces + AHRS feed-forward.** The 20 Hz record, UART now / I2C later, AHRS as optional
feed-forward with the sign test.

> **— split seam 2: D/E ride with 042, or start 043 —**

**Working preference (2026-08-17): 042 = A through D**, with E folded in if small; everything photometric
to 043 — because A–D is the coherent claim ("robust tracker, measured") and D is what makes the 043 gates
quantitative.

---

## 14. Explicitly deferred

**To 043**: daylight — sky background through the 850 filter (031 empirical #1, **still open**); range;
the saturation/blooming and defocus/descaling study at high power; exposure/gain controller *tuning*
against real conditions (the framework is 042); 453 fps on Pi 5; the FPGA decision, closed by the 042-D
table; the tracker-error-model handoff that lets 041 put sun behaviour back in the sim; true 200 Hz chip
rate with Gold-31.

**To backlog / M3–M4**: ST multizone ToF for close-in and **non-broadcasting targets**; stereo multipath
rejection; the polarizer experiment; the height-above-water inversion; the 120° single-lens question
beyond measuring the ELP-L156; the flight recorder as a product.

**Not 042**: Pi 5 / FPGA host; simulation of the camera (operator: not needed — focus on reliable
tracking).

---

## 15. Open for the morning

1. Confirm the phasing seam (042 = A–D, E folded in?) before tasks are cut.
2. Exit-band numbers (§3) — 90 / 150 / 300–500 °/s and ≤1 px RMS — confirm or move.
3. Recorder default: RAM-ring+trigger vs continuous reduced stream as the flight default.
4. Verify the ST ToF part number before it is quoted anywhere downstream (§9).

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
