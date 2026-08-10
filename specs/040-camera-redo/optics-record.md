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
| ~~**Emitter drive**~~ | ❌ **RULED OUT in direct sun on a PHOTODIODE — measured, not argued. See §3a: this may not carry to an imaging array.** |

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

## 3a. ⚠️ EVERY BENCH NUMBER ABOVE IS FROM A 1-PIXEL PHOTODIODE

**Added 2026-08-02 (operator): camera trials are imminent, and they are a different sensor.** All 031
bench and field work to date — including field test #4 and the ≤10 nA decode floor — uses a **single
photodiode**. The 040 model is of a **320 × 240 imaging array**. Those differ in the one respect that
dominates this whole analysis.

**A PD integrates the entire field onto one junction.** Every ambient photon inside its acceptance angle
lands on the same device as the beacon. **An array spreads ambient across 76,800 pixels while the beacon
concentrates onto a few.** That asymmetry *is* the reason to use a camera at all, and it is worth ~48 dB:

| | solid angle |
|---|---:|
| one pixel (0.375°) | 4.28 × 10⁻⁵ sr |
| full 120° × 90° field | 2.989 sr |
| **ratio** | **69,780 ⇒ 48.4 dB** |

*(Sanity check: the pixel count is 76,800; the gap to 69,780 is the sin θ/θ Jacobian — the sphere subtends
slightly less than the naive angular product. The two agreeing to ~9% is the arithmetic checking itself.)*

### This inverts §2's conclusion

The ~24 dB shortfall is computed against a **PD-anchored** budget. An array recovers roughly **twice**
that from spatial ambient rejection alone — even with a badly blurred spot:

| beacon spot | rejection | headroom vs the 24 dB shortfall |
|---|---:|---:|
| 1 × 1 px | 48.4 dB | **+24.4 dB** |
| 2 × 2 px | 42.4 dB | +18.4 dB |
| 3 × 3 px | 38.9 dB | +14.9 dB |
| 5 × 5 px | 34.5 dB | +10.5 dB |

**What transfers from the PD work and what does not:**

- ✅ **The SIGNAL side transfers.** The same aperture collects the same beacon flux, so
  `SignalFluxConstant`, the ÷5 co-aiming correction, and the 1/r² law are all still the right shape.
- ❌ **The NOISE and FLOOR side does not.** The ≤10 nA decode floor is a *PD* decode floor for a
  whole-field integrator. An array's limits are per-pixel well depth, read noise and per-pixel shot
  noise — a different noise model in different units.
- ❌ **`SignalAmbientKnee` is anchored to the wrong sensor.** Junction forward-bias from whole-field flux
  is a PD failure mode; an array pixel sees ~1/70,000 of that flux, so compression in that specific form
  is largely a non-issue for an array, which saturates wells instead.
- ⚠️ **"You cannot out-power the sun" is a PD result.** It is solid for a bare photodiode and may well
  **not** carry to an imaging array, where the sun occupies its own pixels and the beacon does not share
  them. §2's struck-out emitter-drive row should be read as *PD-specific pending the camera trials*, not
  as a general law.

### Honest disclosure about how this model got here

`SignalAmbientKnee` was added to the 040 signal model on 2026-08-02 in direct response to field test #4.
The **structure** is right — ambient can compress, and modelling it only as additive noise let the model
claim you can out-power the sun. The **magnitude** is anchored to a photodiode and is very likely far too
pessimistic for an array. It is an `A` row and the camera trials are what will set it.

**No harm to the runs**: ambient variation ships at sigma 0 and the nominal knee sits 100× above the
nominal floor, so compression contributes −0.09 dB in t1 and t2. The wrong anchor is in the *record*, not
in the trained controllers.

### ⏭️ A THIRD configuration is coming first: a 16 mm lens on the single-pixel PD

**Operator 2026-08-02: "we have a 16 mm lens coming with the single pixel PD — the math will be all
different — we'll work those details later in 031."** Recorded here only so the reader knows this record
describes *two* sensors while the next bench data will come from a *third*. **The details belong to 031,
not to 040.**

The one thing worth stating now is what that rig is and is not. A 16 mm lens collapses the PD's acceptance
angle from a bare device's near-hemisphere to a few degrees:

| PD | active area | field with a 16 mm lens | ambient vs a bare 120°×90° view |
|---|---:|---:|---:|
| BPV10NF | ~0.78 mm² | ~3.2° | ~1,250× less (**~31 dB**) |
| BPW34 | ~7.5 mm² | ~9.8° | ~131× less (**~21 dB**) |

So a lensed PD attacks the *same* compression problem as an array — by rejecting out-of-field ambient —
but geometrically rather than spatially, and it gains aperture on the signal at the same time. It is
therefore an excellent instrument for **isolating the link budget and the filter question**, which is
exactly what 031 needs next.

**But a 3–10° detector is not a 120° tracking sensor.** Whatever range it demonstrates is a statement about
the *link*, not about the *field*, and it cannot be read directly as a range for the flight camera. The
array analysis above stands on its own for the flight sensor; the lensed-PD result will bound the terms
that both share (`SignalFluxConstant`, `SignalOpticsGain`, and whether the bandpass restores sun
operation at all).

### What the camera trials should measure

1. **Per-pixel ambient photocurrent in direct sun**, with and without the 850 nm bandpass — this is what
   actually sets `SignalAmbientKnee` for an array, and it is the number the PD cannot provide.
2. **The beacon's spot size in pixels** at representative range. It converts the table above from a range
   of possibilities into a single number, because the whole 48 dB rests on concentration.
3. **Whether the sun-exposed failure reproduces at all** on an array. If it does not, field test #4's gate
   is a PD artefact and the bandpass drops from *gate* back to *optimisation*.
4. **Blooming / smear extent around a saturated sun**, in pixels, at the working exposure. The 48 dB above
   assumes the sun stays in *its own* pixels. Charge bleeding into neighbours, or a smear column, is the
   mechanism that would break that assumption — and it is a sensor property, not a lens property.
5. **Sun-transit recovery time from a saturated start.** Not the same entry condition as a dark occlusion,
   and nothing on the PD bench or in any field test so far has measured it.

### ⚙️ Two camera REQUIREMENTS this analysis promotes out of "preference" (operator 2026-08-04)

Both are recorded here because they are consequences of §3a's argument, not shopping preferences. A
candidate sensor lacking either is disqualified. They are mirrored on the D1 line of
[`specs/031-beacon-camera/verified-bom.md`](../031-beacon-camera/verified-bom.md).

- **Global shutter.** A rolling shutter converts a saturated sun from a ~2 px event into a full-height
  smear column that can cross the beacon track. Global shutter keeps the event local. (The shipped D1
  choice — Arducam OV9281 — already satisfies this; it simply was not written down as binding.)
- **Manual / fixed exposure and gain.** This one is load-bearing in a way that is easy to miss:
  **frame-averaged auto-exposure discards the entire 48 dB of spatial ambient rejection that justifies
  using a camera at all.** A sun anywhere in a 120° field crushes global exposure and blanks the beacon
  everywhere — the array's spatial advantage is undone by a global control loop. Fixed exposure, or ROI
  metering locked to the track.

This is the same failure the single-pixel rig has in the decoder's AGC: a control loop chasing a pedestal
it should instead detect and freeze on. The 031 fix shape is a max-energy gate that freezes AGC and
asserts HOLD, mirroring the existing min-energy gate
([031 bench journal](../031-beacon-camera/bench-journal.md) item 2a). **Operator intent is that these
converge — the decoder eventually drives camera exposure/gain directly**, making saturation detection and
exposure command one control problem rather than two. Worth carrying into whatever spec owns the camera
bring-up.

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
