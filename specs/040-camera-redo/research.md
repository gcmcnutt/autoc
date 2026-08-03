# Phase 0 Research — 040 Camera Redo

**Date**: 2026-07-28 | **Plan**: [plan.md](plan.md) | **Spec**: [spec.md](spec.md)

Measured hardware values are **not** repeated here — they live in
[input-data-checklist.md](input-data-checklist.md), the input of record. This document resolves *design*
unknowns and records ground truth established by reading the tree.

---

## R1 — Which tracker tick is the production path?

**Decision**: `crrcsim_tracker_helper.cpp` is production. `TrackerStepper` is a **test-only reference
implementation**. Extract the shared per-tick rule into one unit that both consume.

**Rationale**: The constitution's Architecture section states crrcsim FDM is the *"sole worker since 034;
minisim retired"*. Searching the tree, `TrackerStepper` appears only in `tests/` (`tracker_stepper_init_tests.cc`)
and in comments; no production call site remains. The two implementations currently duplicate the CEP gate,
span computation, and situational-awareness update.

**This corrects the spec's stated rationale for FR-031.** The spec says the rule is "duplicated across two
execution paths and would otherwise diverge", implying two production paths. It is one production path plus
a test-only twin. The work is still required — arguably more subtly important, because *the tests are the
behavioural contract*, and a test-only implementation encoding stale semantics would silently certify wrong
behaviour — but the risk profile is lower than the spec implies.

**Alternatives considered**: (a) Delete `TrackerStepper` outright — rejected: it is the only harness that
exercises tracker init geometry and per-scenario reset without standing up crrcsim, and FR-020a's reset
requirement needs exactly that. (b) Leave both and update in lockstep — rejected: that is the divergence
the requirement exists to prevent.

---

## R2 — Bearing representation and separation metric

**Decision**: quantise to the 320×240 pixel grid, then present **angles in radians**, isotropically scaled.
Retire the int8 encoding entirely.

**Rationale**: The int8 step is 0.472°/LSB horizontally against a 0.375° pixel — **26% coarser than the
sensor it claims to model** — while being 6% *finer* vertically, and the two axes carry different scales.
Radians are the honest sim↔hardware contract (the front-end's output is an angle; pixels are a sensor
implementation detail), isotropic by construction, and land in a well-conditioned range (±1.047 H / ±0.785 V).
Separation then becomes a true angular quantity, so range follows from the pair geometry directly — and
exactly, once the metric is spherical: `R = (L/2)/tan(ψ/2)` for a baseline `L` subtending `ψ`, with no
small-angle step left in the chain.

**AMENDED 2026-07-29 (T033a) — the residual is corrected, not documented.** This paragraph previously read
"*Residual to document, not fix*" and accepted a tangential over-read on the grounds that it "makes a
separate ray-angle span computation unnecessary". **That was the wrong call**, for a reason the original
note did not weigh: the over-read is position-dependent and it sits in the *only* channel conveying range.
Operator decision (2026-07-29) was to reopen it rather than reword SC-001 around it.

**The geometry, restated correctly**: under equidistant mapping, Euclidean distance in (θx, θy) equals the
great-circle angle **exactly** radially and over-reads tangentially by θ/sin θ — +8.6% at 40° off-axis and
**+35% at the 75° frame diagonal**. *The +21% originally recorded here was θ/sin θ at the 60° horizontal
edge, not at the true diagonal — the worst case was understated.* This is the generic
flat-metric-on-a-curved-surface error, not a property of the encoding, which is why it is fixable without
touching the representation.

**Correction**: `compute_pair_span` reconstructs both unit rays from the quantised bearings — the exact
inverse of the forward projection, so no information is added or lost — and returns the angle between them
as `2·asin(chord/2)`. That form rather than `acos(u_L · u_R)` because `gp_scalar` is **float** and our
operating regime is small spans: at 25 m the pair subtends ~0.031 rad, where the dot product sits at
0.99952 and `acos` sheds roughly half its significant digits. The result is exactly position- and
orientation-invariant by construction, so **SC-001 holds as literally worded** and there is no residual
left to pin. Cost is one sqrt + one sincos per beacon per tick, negligible against the FDM.

**Bearing representation is unchanged** — still equidistant (θx, θy), still 4 slots, so `TrackerInput::COUNT
== 58` and FR-006 are untouched. Only the metric moved. **Tilt deliberately stays planar**: on a sphere the
pair's angle needs a reference direction and there is no global horizontal, so defining one means a
parallel-transported tangent basis at the pair midpoint. Tilt conveys roll/aspect, so its residual is a
*direction* error — far more benign than a *magnitude* error in the range channel. Recorded as a known
approximation.

**Alternatives considered**: (a) Raw pixel indices as controller inputs — rejected: large, non-zero-centred,
poor conditioning. (b) Direction cosines `(sin θ·dir)` — rejected as written, and the rejection was *too
broad*: that is a **two-component** encoding with the along-boresight component discarded, and it is the
truncation that compresses toward the frame edge, not ray geometry as such. The **full three-component**
unit ray has no compression whatsoever, which is what T033a now uses internally — as a measurement stage,
not as an NN input, so (b)'s conditioning objection never arises. (c) Keep int8 — rejected: it is a
resolution model, and the pixel grid *is* the resolution model, so it is redundant and wrong.

---

## R3 — Obstruction primitives

**Decision**: three primitives — **thin wing slab**, **pod nose box**, **propeller disc as a static angular
region**. Replace `defaultAirframeProxyHB1()` entirely.

**Rationale**: The current single AABB models a 30″×7″×1″ wing as a solid brick spanning
y ∈ [−0.6, +0.6], z ∈ [−0.05, +0.20] — it would over-obstruct grossly. Worse, it is **degenerate**: the
camera mount sits at z = −0.05, exactly on `box_min_z`, and the slab test counts a surface touch as a hit, so
enabling obstruction today obstructs essentially every forward ray. That defect must be fixed before anything
depends on obstruction.

At the baseline mount (leading edge, 8″ outboard, ~1″ above the thrust line) the wing contributes **no**
obstruction — nothing sits ahead of the leading edge — and the propeller shadow lands 41–61° inboard. The
propeller therefore needs no blade phase or engine speed: a static angular region with a representative mean
attenuation is sufficient and costs no propulsion data.

**Alternatives considered**: (a) Corrected single AABB — rejected: cannot represent a thin plate and a
fuselage simultaneously without over-obstructing. (b) Full mesh — rejected: enormous cost for a model whose
job is now design validation. (c) Engine-speed-resolved propeller — deferred to backlog with an explicit
trigger (a mount that puts the camera behind the disc).

---

## R4 — Signal-budget shape

**Decision**: a per-beacon chain — emitter drive × emission pattern × `1/r²` × obstruction attenuation ÷
ambient floor → per-chip SNR → quality. Seed from 031 bench measurements; every term a configured value.

**Rationale**: Today range does not enter perception at all: a beacon at 5 m and 500 m are identical.
`1/r²` is the single highest-value physical term, and the 031 bench provides a measured flux constant to
anchor it. Emission uses a **flat-top-with-shoulders** profile (flat to ≈±45°, half-power at ±75°) per the
Lumileds DS190 datasheet — **not** `cos^m`, which under-reads the flat region ~16% and over-reads the skirt.

**Calibration honesty**: two known discrepancies are carried as *stated uncertainty*, not silently absorbed —
(a) the bench's five co-aimed emitters do not represent a flight enclosure aiming five directions, so the
current-scaling in `optical-link-outcome.md` overstates field range by ~1.4×; (b) the 100 m link budget
assumes a narrow camera, and a 120° optic is ~40 dB down. Both are recorded as calibration targets per
FR-035, which is exactly what plumbing-first is for.

**Alternatives considered**: (a) Keep the position-only placeholder — rejected, it is the defect the feature
exists to fix. (b) Full radiometric model with sensor QE, well depth, read noise — rejected as premature: it
needs the photon budget, which needs article 1 and raw capture, both deferred.

---

## R5 — Acquisition state machine

**Decision**: per-beacon **chip-credit integrator** feeding a four-state machine (searching → acquiring →
tracking, plus holding through brief loss). Fully deterministic — the 031 probability curves become
**thresholds and time constants**, never Bernoulli draws.

**Rationale**: Determinism is non-negotiable (FR-020) and bit-replay is a project gate, so sampling is
disqualified outright. Timings come from the shipped gateware, hardware-measured at N=31 — see **R12**
below, which supersedes the earlier N=15 figures used when this section was first written.

**Tentative lock reports a bearing** with a large-variance quality value (FR-017a), matching the 031
soft-threshold recommendation. Quality therefore spans three regimes: small (confident), large (tentative),
and the distinct not-visible indication.

**The state machine is internal** (FR-017b) — never a controller input. The interface stays bearing plus
quality; the controller infers state from how quality behaves across its history window.

**Per-scenario reset is a known trap** (FR-020a): the existing situational-awareness state carries an
explicit warning that failing to reset it leaks across scenarios and breaks the bitwise gate. The new
carried state has identical exposure, in both the production path and the test-only twin.

**Alternatives considered**: (a) Monte-Carlo the 031 curves — rejected, breaks determinism. (b) Expose
lock state as an input — rejected by operator direction; it also grows the 58-input vector, which FR-006
forbids. (c) Instant acquisition — rejected, it is the current (wrong) behaviour and erases the
reacquisition cost that is the documented M2 bottleneck.

---

## R6 — Camera-variation plumbing

**Decision**: follow the **existing craft-variation pattern** exactly — draw from the reserved `camera`
sub-seed, record raw pre-scale draws in `ScenarioMetadata`, expose sigmas as ini keys.

**Rationale**: Slot 5 is already reserved and seeded in `deriveClassSubSeeds` (frozen order
wind/rabbit/entry/craft/camera) and `ScenarioMetadata` documents the `cameraSeed` append point after
`craftSeed`. The craft variation (034 US4) is a working, tested precedent for draw-and-record. Recording
raw pre-scale draws is what makes variation verifiable ramp-independently via `dmp-dump --meta-only`.

Camera variation stays **chase-specific** even when environment seeds are shared with the target
(FR-022) — perception belongs to the chase alone.

**Magnitudes** (operator): boresight and roll each σ = 10° **hard-clipped at 20°** — clipped, not
tail-sampled, so training never sees an implausibly misaligned camera. Position within a 1 cm box.

**Position splits by consumer**: ±5 mm is 0.03° at 10 m — negligible for bearing — but swings propeller
clearance ~15%, because the clear cone is `atan((h − r_tip)/d)` with a small numerator. **Feed translation
into obstruction only.**

**Alternatives considered**: (a) A new PRNG class — rejected, the slot exists and the order is frozen.
(b) Time-varying ambient within a scenario — rejected as scope; static per scenario matches the craft pattern.

---

## R7 — dmp diagnostic fields

**Decision**: append pixel-coordinate and diagnostic fields (quality measure, tracking state) to
`BeaconObservation` / `CameraViewSample`, following the in-code convention.

**Rationale**: `protocol.h:428-447` documents the established practice — append at the end of the v≥2
block, no `CEREAL_CLASS_VERSION` bump, old dmps orphaned. Orphaning satisfies Principle V's fail-loud
requirement: old files fail to parse rather than mis-parsing into plausible-but-wrong values. The M1 source
format is untouched, so M1 dmps stay readable and no rebake is triggered.

**Confirmed no new fields needed for throttle or range**: `DebugSample` already carries
`throttleCommand`/`throttleSim`/`position`, and `CopiedTargetSample` carries target position, so
chase-to-target range is derivable today.

---

## R8 — Throughput measurement

**Decision**: benchmark **total evaluation throughput** against the **prior M2 run**, not against a
micro-benchmark of the perception function.

**Rationale**: Operator direction, and it is the right comparator — the prior M2 run is what generations
are actually being traded against. Perception is not the dominant per-tick term (the FDM step and NN
forward pass dominate), so isolating it would over-constrain a minor contributor while missing whole-loop
effects. Operator's expectation is that the added work lands well under 10% of the per-step physics cost;
the ≤10% ceiling (FR-038) is therefore a guard rail rather than a binding constraint.

**A breach escalates** to an explicit accept-or-optimise decision rather than being absorbed silently,
because throughput converts directly into generations reached.

---

## R9 — M1 source provenance and retention (T003a, resolved 2026-07-28)

**Both M1 sources the feature depends on are now pinned `retain=keep`.**

| Role | Config | Bucket / key | Tag |
|---|---|---|---|
| M2 **training** source | `autoc-tracker.ini` | `autoc-m1` · `autoc-9223370253553029228-2026-07-06T01:35:46.579Z/gen9200.dmp.zst` | already `keep` |
| M1 **novel-path eval** source | `autoc-eval-tracker.ini` | `autoc-eval` · `autoc-9223370253134917267-2026-07-10T21:44:18.540Z/gen9999.dmp.zst` | **was `expire` → set to `keep`** |

**The eval source was ~12 days from deletion** — created 2026-07-10, and the Principle VIII lifecycle
expires `retain=expire` objects 30 days after creation (≈2026-08-09). It is the novel-path source from the
038 t10 wrap exercise, i.e. the paths the prior M2 baseline's generalisation was measured on. Losing it
would not have broken the retrain; it would have removed the ability to evaluate against the *same* novel
paths as the baseline — silently destroying the comparison SC-008 rests on, and surfacing weeks later as
"the baseline comparison cannot be reproduced" with no obvious cause.

Single 9.4 MB object in that prefix, so one tag preserves the run.

**Interpretation caveat (Assumption 13a)**: the current best M1 is acknowledged mediocre, and M1 fidelity
work (less pitch aggressiveness) is deferred. The chase can only track as well as the target flies, so
**absolute** M2 competence here is bounded by target flight quality rather than perception. Only the
aggregate delta against this same-source baseline is interpretable — which is exactly why both sources are
pinned rather than refreshed.

## R10 — Throughput baseline for FR-037/038 (T002, captured 2026-07-28)

Prior M2 training run: **`logs/autoc-038-t9-m2-spherical.log`** — the t9 spherical/equidistant run whose
elite fed the 038 t10 wrap exercise, i.e. the baseline SC-008 compares against.

| Metric | Value |
|---|---|
| Window | 2026-07-09 11:32:11 → 2026-07-10 14:24:43 (**26.88 h**) |
| Generations | 430 |
| **Wall throughput** | **16.00 gen/hour** (225.0 s/gen) |
| Per-gen sim duration | mean 225.0 s (min 90.6 / max 275.8) |
| Sim rate | mean 7359 sims/s; **late-run ≈5530–5730 sims/s** (gens ~425–430) |

**Use `sims/s` at a comparable generation as the primary comparator, not gen/hour.** Wall throughput is
sensitive to population and scenario-count configuration, and the per-gen figure drifts *within* a run as
scenario length grows — the mean (7359) is inflated by faster early generations, so comparing a new run's
early gens against this mean would flatter it. The late-run band (~5600 sims/s) is the honest reference.

**FR-038 ceiling**: ≤10% regression ⇒ a new run must hold **≥ ~5040 sims/s** in the same late-run regime.

## R11 — Bit-identity oracle for the Stage B gate (T003, partial 2026-07-28)

**Candidate elite**: prior M2 run `autoc-038-t9-m2-spherical`, **gen 430** (final).

| Field | Value |
|---|---|
| Elite fitness (training) | **`-13949.366286`** |
| Corroboration | `NN_ELITE_SAME: gen=430 fitness=-13949.366286` — the run's own elite re-eval already reproduced it |
| Competence context | `pctInStreak=7.5`, `avgMaxStreak=21.1`, 293/294 rabbitComplete, 0 hull strikes |
| S3 | bucket `autoc-m2`, run `autoc-9223370253134917267-…`-class prefix (confirm exact key before pinning) |

**Why this value is a usable oracle**: the run's own `NN_ELITE_SAME` line shows the elite re-evaluates to
the training fitness bit-exactly, so it is already a proven determinism anchor rather than merely a
recorded number.

⚠️ **Remaining step is operator-driven**: actually running the eval to confirm `NN_EVAL_SAME` against a
rebuilt binary is the eval-vs-training bitwise gate, which the operator drives rather than an assistant
kicking off. The value above is what T021 must reproduce.

## R12 — Acquisition timing model, first pass (decided 2026-07-28)

Supersedes the N=15 figures in R5. Source is the **shipped gateware**, not the paper study:
[`firmware/beacon-decoder-stepfpga/SIM-FEATURES.md`](../../firmware/beacon-decoder-stepfpga/SIM-FEATURES.md),
recovery-counter measurements on real hardware.

### Rate stack

Controller **20 Hz** (50 ms) · camera **480 fps** (2.08 ms) · chips **200 Hz** (5 ms) · code **N=31**
(154 ms). Per controller tick the camera captures **24 frames** and advances **10 chips** — about a third
of a word. Frames-per-chip is 2.4: Nyquist margin, not information.

### Measured timings (N=31; N=63 reads 315/629 ms)

| | ms | ticks |
|---|---:|---:|
| Code word | 154 | 3.1 |
| **Warm re-acquire** (flywheel held) | **154** | 3.1 |
| **True-cold** (rate stale, needs `MINLOCK`) | **308** | 6.2 |
| HOLD coast (`HOLDMAX` 2 bad periods) | 308 | 6.2 |
| **Coast window** (`COASTMAX=65`) | **~10 000** | 200 |

### Decisions

1. **N=31 baseline** (operator: all 031 tests moved to 31; 63 is next). Code length is a **config value**,
   so the 63 upgrade is a value change rather than a re-derivation.
2. **Mirror the gateware FSM** — `SEARCH → ACQUIRING → LOCKED → HOLD` — rather than invent one. It is
   already the right shape, it is what the hardware does, and it is ~50 lines.
3. **Analytic advance, once per controller tick.** No sub-stepping. Exact for constant SNR, phase-free
   (154 ms against 50 ms is 3.08, so the code boundary drifts relative to the tick as free-running hardware
   does), and 24× cheaper than frame-stepping — which matters against the FR-038 ceiling.
4. **Warm/cold gated on the coast timer** — the single highest-value feature in the model.
5. **Quality from a `q` proxy.** The hardware emits `q = |corr|/energy` on 0–9, **signal-level
   independent** (AGC-normalised), GOOD ≥ 5. CEP derives from a simulated `q`: a monotonic map from
   modelled per-chip SNR, ramping with integration and degrading in HOLD. This *is* the operator's
   "likely-will-lock representation in CEP ≈ SNR" — it falls out rather than being bolted on.
6. **Cold path modelled**, despite being rare in M2 (operator: it does occur in flight — sun, reflections
   — at losses beyond ~10 s). Nearly free once the timers exist.

### The behaviour-defining finding

**The coast window is wallclock-driven, not code-length driven** — set by emitter↔receiver oscillator
stability. The documented M2 worst-case blind window is **~8 s**, which sits **inside** the ~10 s coast.
So **most M2 reacquisitions are warm (154 ms), not cold (308 ms)**, and "N=31 triples acquisition" holds
only for the cold path M2 rarely takes. **Sim difficulty hinges on the coast window far more than on code
length** — it is the parameter to vary if reacquisition cost turns out to matter.

### Deferred (recorded, not dropped)

- **DPLL slip/skew pull-in** — snap-to-estimate reaches cold full quality in <1 s; the first pass gives
  cold lock full `q` immediately
- **Per-chip erasure/flip accounting** — the measured 2:1 flip-vs-erasure asymmetry
- **2.4 frames/chip oversampling** — Nyquist margin, carries no information the tick-level model needs

## R13 — Propeller blade-passage envelope (T046, FR-011: RECORDED, NOT RESOLVED)

**Status**: deliberately *not* a decision. FR-011 asks for the envelope arithmetic to be preserved because
it is cheap to keep and expensive to re-derive; resolving it is a **decoder-design** research project, not
a controller-training one, and it is **moot at the 040 baseline mount**. Recorded here so it survives
outside the checklist that produced it — see [input-data-checklist.md](input-data-checklist.md) §C for the
full derivation and its assumption ledger.

**Blade-passage frequency**, 2-blade prop: `f_bp = 2 × RPM/60 = RPM/30`. Across level flight this runs
**261–679 Hz** (7 833 rpm at stall → 20 365 rpm at 92% throttle), and down to ~187 Hz static at low
throttle. Cruise (13.0 m/s, ~62% throttle, ~14 660 rpm) sits at **~489 Hz**.

**The commensurabilities that matter**, against the 480 fps camera / 200 Hz chip / N=31 code baseline:

| coincidence | RPM | airspeed |
|---|---:|---:|
| `f_bp` = 2× chip rate (400 Hz) | 12 000 | ≈10.6 m/s |
| `f_bp` = **frame rate** (480 Hz) | 14 400 | **≈12.8 m/s — on autoc cruise** |
| `f_bp` = 3× chip rate (600 Hz) | 18 000 | ≈15.9 m/s |

**Exact lock is benign; the NEAR-MISS is the hazard.** At exact lock the occlusion phase is fixed every
frame, so attenuation is uniform and the energy-normalised AGC absorbs it (031 shows lock holding at 3%
duty). But when the beat period approaches the 75 ms code word, the phase walks a full cycle *within* one
code and produces a systematic attenuation ramp across the 15 chips — structured error, and flips cost 2×
erasures. At cruise the beat is |489 − 480| ≈ 9 Hz (114 ms), so phase advances ~66% of a cycle per word.
**Predicted danger band: `f_bp` = 480 ± 13 Hz ⇒ 14 010–14 790 rpm ⇒ 12.4–13.2 m/s.**

**Why it is moot here, and the two things that would change that.** At the baseline mount the disc sits
**42–61° inboard** (measured by `dmp-dump --obstruction-report`, T045), so the shadow is nowhere near a
tail-chased target and a static disc is sufficient. It stops being moot if either (a) the camera moves back
behind the disc — a wing-top or centreline mount, or any build that lets the propeller re-enter the useful
field, or (b) a bench measurement shows margin ripple synchronised to engine speed.

**Cross-check worth noting**: the checklist's independent duty estimate — ~13 mm chord + 8 mm pupil at
r ≈ 35 mm, 14 400 rpm ⇒ 0.40 ms transit against a 2.08 ms blade-pass period ⇒ **~19% duty** — lands on the
`AirframePropAttenuation = 0.18` the model ships. That agreement is reassuring but **not** independent
confirmation: both rest on the same unmeasured pupil and chord figures, and the constant stays classified
**assumed** (FR-035) until a bench measurement replaces it.

**One control-theoretic wrinkle, recorded because it is easy to forget**: throttle is an NN *output*, so
the controller selects its own position relative to these bands. A trained M2 could in principle learn to
avoid — or to sit in — a band, which makes this a closed loop rather than an external disturbance. Nothing
in 040 exploits or prevents that.

---

## Open items carried into implementation

Values, not structure — none blocks the design (per the plumbing-first contract, FR-034/035):

| Item | Handling |
|---|---|
| Entrance pupil diameter | Configured, classified **assumed**; only affects the propeller attenuation constant, which is itself representative |
| Exposure duty | Configured with a stated default; its influence is reported as a sensitivity, not a fixed result |
| Filter bandwidth, flux anchor, ambient pedestal | Configured, classified **assumed**, seeded from 031 |
| Airframe CG station, wing camber | Affects obstruction geometry marginally; classified **assumed** |
| Wing-thickness variation sigma | Folded into camera variation as an obstruction-side term |

---

## R14 — What the shipped signal model actually delivers vs range (measured 2026-07-30, post-T057)

Probe against `hb1SignalConfig()` through the shipped `computeSignal()`, tail-chase geometry (chase aft of
target, so the enclosure's aft face is on-axis). This is the answer to "will the sim show signal at 100 m
even if that isn't realistic?" — **yes, by assertion, and here is the size of the lie.**

| range | received | SNR | q (0–9) | cep the NN sees | regime |
|---:|---:|---:|---:|---:|---|
| 5 m | 15.5 nA | 27.6 dB | 9.0 | 0.02 | saturated |
| 10 m | 3.9 nA | 21.6 dB | 9.0 | 0.02 | saturated |
| 25 m | 0.62 nA | 13.6 dB | 6.1 | 0.32 | GOOD |
| **33 m** | — | 11.1 dB | **5.0** | 0.44 | **GOOD threshold** |
| 50 m | 0.16 nA | 7.6 dB | 3.4 | 0.62 | degraded |
| 100 m | 0.039 nA | 1.6 dB | 0.7 | 0.92 | barely above floor |
| >100 m | — | — | — | sentinel | hard cut (FR-033a) |

### The number that matters for the hardware work

**Modelled received at 100 m is 0.039 nA. The 031 bench decode floor is ≤10 nA. That is ~256× short,
≈24 dB.** Taken against the measured floor, the honest detection range at today's per-emitter flux with no
collection optics is **≈6 m**, and every range past that would sit equally at the floor — perfectly
monotonic and completely uninformative, which is why the floor is back-solved instead (see
`signal_model.h`).

So the sim's 100 m is an **assertion that the hardware will get there** (FR-033a), and **closing the ~24 dB
is the camera-perf / emitter-power work**. When any of it is real it moves `SignalOpticsGain` /
`flux_constant` / the floor — and per FR-036 it must move **without structural change**, which is what T092
rehearses.

### ⚠️ CORRECTED 2026-08-02 by 031 field test #4 — emitter drive is NOT a free lever

This section originally listed the candidate contributions as "`SignalOpticsGain`, **emitter drive**,
entrance-pupil area, narrower field, longer integration". **The emitter-drive entry was wrong**, and the
bench measured it rather than argued it
([`specs/031-beacon-camera/bench-journal.md`](../031-beacon-camera/bench-journal.md) field test #4,
2026-08-02):

- 6 LEDs @ 50 mA (~300 mA, nominally ~6× current), bare PD, no filter.
- **PD shaded → locks at ~20 ft. PD in direct sun → fails at any distance tested.**
- Same emitter, same distance: **shadow alone flips it.**

The mechanism is **ambient compression at the photodiode** — ambient forward-biases the PD, its dynamic
resistance collapses, and beacon current is *shunted at the sensor*. That loss sits **upstream of every
downstream multiplier**, and it is far larger than any realistic current increase can recover, so **you
cannot out-power the sun**. Consequence recorded on the 031 side: the **850 nm bandpass moves from an
optimisation to a gate** on the whole range roadmap — optics multiply signal but do nothing about
compression, and only after the filter restores headroom does extra current show up as range.

**What this cost the 040 model, and the fix.** `signal_model.h` treated ambient as a purely ADDITIVE noise
term (`snr = received / (ambient_floor + noise_floor)`). Under that shape more `flux_constant` or
`optics_gain` always buys SNR at any ambient — i.e. the model would cheerfully predict you can out-power
the sun. A `SignalAmbientKnee` compression term was added 2026-08-02 so the *transfer* degrades with
ambient as well as the noise. See `signal_model.h`; the knee is ASSUMED and is the value the
**lens + filter field tests (~week of 2026-08-03) are expected to pin**.

⚠️ **Confounds the 031 journal flags itself, and which this note inherits** — the defensible claim is the
shaded/exposed split alone (same emitter, same distance), NOT the absolute range: (1) ~20 ft may be the
*yard* limit rather than the signal limit; (2) splayed LEDs are **not** 6× on-axis, plausibly 1-3×;
(3) conditions and build differ from test #3, which logged 15-20 ft *in* direct sun at 51 mA.

### Two properties worth keeping

1. **The two degradations nearly coincide.** GOOD tracking (q ≥ 5) ends at ~33 m; separation-derived range
   dies at ~28 m. So "I trust this" and "I know how far" fade together rather than as two unrelated
   cliffs — a coherent perceptual story for the controller to learn.
2. **The five-face enclosure is nearly flat across useful aspects.** Gains: tail 1.44, beam peak 1.59,
   quarter 2.29, head-on 1.44, inboard (the mounted face) 0.59. Tail-chase sits only 0.4 dB below beam
   peak. Under the single-outboard-axis model the tail chase — the dominant M2 geometry — would have read
   ~10 dB down.
