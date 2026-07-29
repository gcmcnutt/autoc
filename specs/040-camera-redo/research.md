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

## R2 — Bearing representation and residual anisotropy

**Decision**: quantise to the 320×240 pixel grid, then present **angles in radians**, isotropically scaled.
Retire the int8 encoding entirely.

**Rationale**: The int8 step is 0.472°/LSB horizontally against a 0.375° pixel — **26% coarser than the
sensor it claims to model** — while being 6% *finer* vertically, and the two axes carry different scales.
Radians are the honest sim↔hardware contract (the front-end's output is an angle; pixels are a sensor
implementation detail), isotropic by construction, and land in a well-conditioned range (±1.047 H / ±0.785 V).
Separation then becomes a true angular quantity, so range is `separation ≈ wingspan / angle` directly.

**Residual to document, not fix**: under equidistant mapping, Euclidean distance in (θx, θy) equals the
great-circle angle **exactly** radially, and over-reads tangentially by θ/sin θ — **+21% worst case at the
frame corner**. This is the *only* remaining position dependence, down from the tan-stretch removed at t9,
and it is what makes a separate ray-angle span computation unnecessary.

**Alternatives considered**: (a) Raw pixel indices as controller inputs — rejected: large, non-zero-centred,
poor conditioning. (b) Direction cosines `(sin θ·dir)` — the literal ray geometry, but compresses toward the
frame edge, re-introducing position-dependent separation scale in the opposite direction from t9. (c) Keep
int8 — rejected: it is a resolution model, and the pixel grid *is* the resolution model, so it is redundant
and wrong.

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
disqualified outright. The 031 data supplies the shape directly: full code 75 ms ≈ 1.5 ticks at 20 Hz,
partial-code early lock ≈ 55 ms, hold ≈ 2 code periods, warm relock ≲ 1 period.

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

## Open items carried into implementation

Values, not structure — none blocks the design (per the plumbing-first contract, FR-034/035):

| Item | Handling |
|---|---|
| Entrance pupil diameter | Configured, classified **assumed**; only affects the propeller attenuation constant, which is itself representative |
| Exposure duty | Configured with a stated default; its influence is reported as a sensitivity, not a fixed result |
| Filter bandwidth, flux anchor, ambient pedestal | Configured, classified **assumed**, seeded from 031 |
| Airframe CG station, wing camber | Affects obstruction geometry marginally; classified **assumed** |
| Wing-thickness variation sigma | Folded into camera variation as an obstruction-side term |
