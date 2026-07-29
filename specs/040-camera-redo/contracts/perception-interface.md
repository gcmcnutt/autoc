# Contract — Perception Interface

**What the controller consumes, and what is deliberately withheld.**

Consumers: the tracker input assembly (`gather_tracker_inputs`), the production tick
(`crrcsim_tracker_helper`), the test-only reference (`tracker_stepper`), the renderer, and `dmp_dump`.

---

## 1. The interface is (bearing, quality) — nothing more

Per beacon, per tick:

| Output | Type | Range | Meaning |
|---|---|---|---|
| bearing x | angle | ±(fov_h/2) rad, ≈±1.047 | right positive; quantised to the pixel grid |
| bearing y | angle | ±(fov_v/2) rad, ≈±0.785 | down positive (pixel convention) |
| quality | scalar | see regimes below | positional uncertainty |

**Both axes carry the same angular scale.** A given angular separation reads identically regardless of
orientation or position in frame (FR-002). Bearing is quantised on the sensor's pixel grid — never finer,
never coarser (FR-001).

### Quality regimes

| Regime | Meaning | Bearing valid? |
|---|---|---|
| small | confident, tracking established | yes |
| large | **tentative lock** — reported early, explicitly untrusted | yes |
| sentinel | not visible | no (zeroed) |

The sentinel MUST remain distinguishable from every in-range value.

---

## 2. What is NOT exposed

**The tracking state machine is internal** (FR-017b). No categorical state — searching, acquiring,
tracking, holding — reaches the controller. Its effect arrives *solely* through the quality value.

Rationale: the controller infers tracking state from how quality behaves across its history window, rather
than being handed a category. This keeps the inference in the network where it belongs, and keeps the input
vector at 58 (FR-006).

Also not exposed: range (inferred from separation), signal-to-noise, obstruction flags, engine speed,
propeller phase.

---

## 3. Two envelopes, not one (FR-033)

Detection and range-inference have **different reach**, and both must be modelled:

| Quantity | Reach | Failure mode |
|---|---|---|
| bearing | to the design detection range (~100 m) | quality → sentinel |
| separation-derived range | ≈25 m | separation falls below the resolving limit → range unavailable |

At 120° over 320 px the beacon pair subtends ≈1 px at 100 m. **Neither quantity may be reported as usable
outside its own envelope** — reporting range where the sensor cannot resolve it is the failure this rule
prevents.

Consequence: the controller experiences a genuine perceptual regime change as it closes — bearing-only at
range, bearing-plus-range inside. This is intended, physically honest, and behaviour M2 has not previously
trained against.

---

## 4. Invariants

1. **Determinism** (FR-020) — identical inputs produce bit-identical outputs. No PRNG anywhere in the
   signal path. The 031 probability curves enter as thresholds and time constants, never as draws.
2. **Per-scenario reset** (FR-020a) — all carried state (chip credit, tracking state, hold timers) resets
   at every scenario boundary, identically in both execution paths. Unreset state leaks between scenarios
   and breaks the bitwise gate.
3. **Input-vector stability** (FR-006) — 58 inputs, unchanged. Diagnostics never become inputs.
4. **No M1 dependency** — perception runs chase-side at M2 train time; the M1 source format is untouched
   and no rebake is triggered.
5. **Angle encoding** (FR-005) — quantities that wrap are represented free of discontinuity (target tilt
   stays sin/cos); bounded bearings are not, because they cannot wrap.

---

## 5. Single-sourcing (FR-031)

The per-tick rule — CEP gating, separation, situational-awareness update, and now the acquisition state
machine — MUST exist in exactly one unit consumed by both:

- `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` — **production**
- `src/eval/tracker_stepper.cc` — **test-only reference** (see research R1)

Both must reset state identically. The test-only path matters because the tests are the behavioural
contract; a reference implementation encoding stale semantics would silently certify wrong behaviour.

---

## 6. Physical-camera assumption (unchanged, restated)

The hardware is a **planar sensor**. The perception front-end applies known intrinsics to remap pixel
centroids to angles — a deterministic, information-free calibration step. The simulator skips the
**photometric and centroiding** stages and emits the angle domain directly, but it does model the grid's
*resolution*: bearings are quantised to pixel centres (T031), so reported precision never exceeds what the
sensor can deliver.

**Angular bearing is a representation choice for the interface, not a fisheye claim.**

**Separation is measured on the sphere, not in the bearing plane (amended 2026-07-29, T033a).** Under
equidistant mapping, Euclidean distance in (θx, θy) equals the great-circle angle exactly radially and
over-reads tangentially by θ/sin θ — +8.6% at 40° off-axis, **+35% at the 75° frame diagonal** (the figure
previously given here, +21%, was θ/sin θ at the 60° horizontal edge, not the true diagonal). That residual
was originally accepted as documented-not-fixed. It is now **corrected**: `compute_pair_span` reconstructs
both unit rays and returns the angle between them, which is exactly position- and orientation-invariant by
construction. There is **no remaining position dependence in the separation channel**, and SC-001 holds as
literally worded, to within grid quantisation.

Bearing itself remains the equidistant (θx, θy) pair — the change is to the *metric*, not the
representation. Tilt is still measured in the bearing plane by deliberate choice; see research R2.
