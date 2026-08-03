# Optics Record — why 120°, what it costs, and what would overturn it

**Feature**: 040 camera-redo · **Tasks**: T078–T080 (US7, FR-027) · **Written**: 2026-08-02

**Purpose** (the independent test for US7): a reader who was not party to the scoping conversation should
be able to state **why 120° was retained, what it costs, and which measurements would overturn it** —
without reading the code.

This is a *durable artefact*, not a decision. The lens and sensor are not chosen. What is recorded here is
the reasoning as it stood when 040 closed, so that whoever does choose inherits it rather than re-deriving
it.

---

## 1. The decision: 120° × 90° was RETAINED, and it was not really a choice

The simulated camera is 320 × 240 at 0.375°/px ⇒ **120° × 90°**, unchanged from 030. 040 changed how that
field is *expressed* — it is now derived from the grid rather than configured beside it (FR-003), so field
and resolution can no longer disagree — but not its value.

**It was retained by default, not selected on evidence.** No candidate lens was priced against it, and no
alternative was simulated. That is the honest status, and it is why this document exists rather than a
decision record.

---

## 2. What a wide field costs: the ~24 dB shortfall

This is the load-bearing number, and it comes from running the shipped model rather than arguing
(research [R14](research.md)):

| range | received | SNR | q (0–9) |
|---:|---:|---:|---:|
| 5 m | 15.5 nA | 27.6 dB | 9.0 |
| 25 m | 0.62 nA | 13.6 dB | 6.1 |
| **33 m** | — | 11.1 dB | **5.0 — the GOOD threshold** |
| 100 m | **0.039 nA** | 1.6 dB | 0.7 |

Against the **031 bench decode floor of ≤10 nA**:

> **Modelled received at 100 m is 0.039 nA. The floor is 10 nA. That is ~256×, ≈24 dB short.**
>
> Taken against the measured floor, the honest detection range at today's per-emitter flux with no
> collection optics is **≈6 m**.

The simulator nonetheless reports bearings out to 100 m, because **FR-033a asserts the detection envelope
rather than deriving it** — the budget is not calibrated well enough to be trusted as a *limit*, but is
good enough to shape a *gradient*. The 100 m figure is a claim about what the hardware must become, not a
measurement of what it is.

### Where the 24 dB could come from — and one place it cannot

| Lever | Status |
|---|---|
| **Collection optics** (`SignalOpticsGain`, shipped at 1.0) | Untried. The C-14 lens is the obvious first multiplier. |
| **Narrower field** | More photons per pixel and a longer effective range for the same sensor — the direct trade this document is about. |
| **Larger entrance pupil** | Untried; couples to mass and to the C-14 mechanical envelope. |
| **Longer integration** | Bounded by the 480 fps frame and by motion blur at closing speed. |
| ~~**Emitter drive**~~ | ❌ **RULED OUT in direct sun — measured, not argued.** |

**The emitter-drive row is the finding.** 031 field test #4 (2026-08-02): ~6 LEDs at 50 mA (~300 mA), bare
photodiode, no filter — a **shaded** PD locks at ~20 ft, a **sun-exposed** PD fails at any distance, and
*shadow alone flips it at fixed emitter and distance*. Ambient forward-biases the PD, its dynamic
resistance collapses, and beacon current is **shunted at the sensor**, upstream of every downstream
multiplier. You cannot out-power the sun.

**Consequence, and it reorders the roadmap**: the **850 nm bandpass is a gate, not an optimisation**.
Optics multiply signal but do nothing about compression. Only once the filter restores headroom does extra
current — or extra aperture — show up as range. 031 now lists C-14 (lens + bandpass) as the **sole
remaining critical path**.

*(040 modelled this as `SignalAmbientKnee`; before it, the link budget treated ambient as purely additive
noise and would have cheerfully predicted that you can out-power the sun.)*

---

## 3. What a real 120° build demands of the sensor

Two constraints that a narrow-field build does not face:

**Resolution.** At 120° over 320 px the 0.772 m beacon pair subtends **≈1 px at 100 m**. Range inferred
from separation is therefore unusable well before detection fails — measured crossover **≈28 m**
(T056), against a 100 m bearing envelope. That asymmetry is not a defect; it is why FR-033 models two
envelopes. But it means **a wide field buys angular coverage by spending range resolution**, and the
controller experiences a genuine perceptual regime change as it closes: bearing-only far out,
bearing-plus-range inside.

**Photon budget.** A wide field spreads the same collected flux over a wider angular acceptance. The
~24 dB above is the wide-field penalty made concrete against a link budget that assumed a narrow camera.

**Format.** A real 120° build at the current 0.375°/px needs 320 × 240 with a lens that actually delivers
120° onto that array without vignetting the corners — where, per T044, obstruction already lives. Nothing
has verified that a candidate lens does so.

---

## 4. Assumptions each rests on

Each is a live `A` row in [contracts/config-surface.md](contracts/config-surface.md); none is measured.

| Assumption | Value | If wrong |
|---|---|---|
| Sensor format | 320 × 240 | Changes both fields and the resolving limit; every range-from-separation number moves |
| Angular pitch | 0.375°/px | Same |
| Detection envelope | 100 m, **asserted** | The gradient's endpoint moves; the *shape* survives |
| Noise floor | back-solved, not measured | The whole SNR scale shifts; **q's mapping to range moves with it** |
| Ambient knee | 100× the shade floor | Sets how hard sun bites. **The single value the filter tests should pin** |
| Per-emitter flux | bench ÷ 5 | The bench used five **co-aimed** emitters; flight aims five directions |
| Optics gain | 1.0 | Any real lens moves this first |

---

## 5. What would overturn this analysis (T079)

Ordered by how decisively each would settle it:

1. **Lens + bandpass field test at range, sun-exposed and shaded** *(~week of 2026-08-03, parts in hand
   bar the lens)*. **The decisive one.** If the filter restores sun operation, `SignalAmbientKnee` gets a
   measured value and the emitter-current lever comes back into play behind it. If it does not, wide-field
   daylight operation is in question regardless of optics, and the field decision reopens immediately.
2. **A calibrated received-current measurement at known range with the flight enclosure** — collapses
   `SignalFluxConstant`, the ÷5 co-aiming correction, and `SignalOpticsGain` into one measured number, and
   would replace the back-solved floor with a real one.
3. **A candidate lens's measured MTF and vignetting at the corners.** The corners are where obstruction
   already lives; a lens that dies there costs field twice.
4. **A narrow-field A/B in simulation.** Cheap, and never run: halve `CameraDegPerPixel` and retrain.
   Because FOV is now derived (FR-003), this is a **one-key change** — which is the plumbing-first claim
   T092 rehearses. It would price coverage against range resolution *in controller competence* rather
   than in dB.
5. **The pod measurement** (checklist A1b) — bounds the nose box, currently the entire residual 2.9%
   blockage and the least trustworthy geometry in the model.

**What would NOT settle it**: more bench work at short range with a bare PD. Field test #4 already showed
the limiter is ambient compression, and a shaded bench cannot see it.

---

## 6. Deferred directions

Per Principle X these live in `specs/BACKLOG.md` and are **not** re-described here — narrower fields, dual
field-of-view, a second camera, and the raptor binocular arrangement are all recorded under the 031/040
optics entries. This record is what they should be read against: any of them is a way of buying back some
of the ~24 dB, and none of them helps until the compression gate is cleared.
