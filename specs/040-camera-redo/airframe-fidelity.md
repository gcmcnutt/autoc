# Airframe Fidelity Check — simulated vs measured

**Task**: T004 (FR-024) | **Date**: 2026-07-28 | **Gate**: this document feeds the T005 decision (FR-025)

**Purpose**: establish, before any perception work begins, whether the simulated flight model must be
regenerated. A "regenerate" verdict forces an M1-source rebake *first*, because the M2 controller trains
off the M1 source — so this runs at Stage A and gates everything downstream.

---

## The structural finding that frames everything

**`crrcsim/models/hb1_streamer.xml` is a stability-derivative model, not a geometric one.**

It carries `Cl_b`, `Cl_p`, `Cl_da`, `Cn_b`, `Cn_r`, `Cm_q` and friends — coefficients **tuned against
flight data across features 021 and 023**, with the tuning history recorded inline as comments. The
vertical fin, for instance, is not a shape anywhere; it is represented as `Cn_b=0.07` with the note *"Real
HB1 has vertical fin behind motor — significantly more yaw stability than a pure flying wing."*

**Consequence**: most measured airframe geometry does **not** feed the flight model at all. Span, chord,
thickness, station stack and mount positions feed the **obstruction model**, which is 040's new work and
carries no plant risk. Only mass, reference dimensions and the propulsion block touch the plant.

---

## Parameter comparison

| Parameter | Simulated (`hb1_streamer.xml`) | Measured | Δ | Verdict |
|---|---|---|---|---|
| Mass | `Mass="0.515"` kg | 520 g (eCalc entry) | +1.0% | ✅ **agree** |
| Wing span | `ref span="0.762"` m | 30″ = 0.762 m (sketch) | 0% | ✅ **agree, exact** |
| Wing chord | `ref chord="0.178"` m | 7″ = 0.178 m (sketch) | 0% | ✅ **agree, exact** |
| Wing area | `ref area="0.136"` m² | 7″×30″ = 0.1355 m² geometric | −0.4% | ✅ **agree** |
| **Prop diameter** | `propeller D="0.127"` m (5.0″) | **0.1397 m (5.5″)** — photo, Windsor/Master Airscrew | **+10% dia, +21% disc area** | ❌ **disagree** |
| **Prop pitch** | `propeller H="0.114"` m (4.49″) | **0.1016 m (4.0″)** — photo | **−11%** | ❌ **disagree** |
| CG arm | `CG_arm="0.28"` (MAC units) | not measured | — | ⚠️ **unknown** |
| Motor reference | `automagic omega_p="2827"` (≈27 000 rpm) | EMAX EcoII 2207-2400, 3S 1000 mAh | — | ⚠️ **model reference, not a max-rpm claim** |
| Vertical fin | `Cn_b=0.07`, `Cl_b=−0.05` (derivative) | 6.5″ × 4″ on a twin 5/16″ dowel boom | — | ✅ **represented, non-geometrically** |

### Third-party data discrepancy (not a simulator defect)

The eCalc propulsion sheet lists **wing area 17.42 dm² (270 in²)**, implying a ~9″ mean chord against a
measured 7″. The simulator's 0.136 m² matches the measured geometry; **eCalc is the outlier**, almost
certainly a data-entry error. It does not affect the RPM-vs-throttle relation but does contaminate eCalc's
*airspeed* axis (lift/drag balance), which is why the propeller work — now backlogged — was scoped to fit
**throttle→RPM** rather than airspeed→RPM.

---

## Analysis: does the propeller discrepancy warrant regenerating M1?

The propeller is the only genuine plant-affecting disagreement. Three considerations, and they point the
same way:

**1. The discrepancy is coupled to the tuning, not independent of it.** The derivative set was fitted
against *observed flight* with these propeller values already in place. `Cl_da` alone was revised four
times across 021/023 to match measured roll rates. Correcting `D` and `H` in isolation would break that
fit — the model could become **less** faithful to the real article, not more, because the tuning silently
absorbed whatever thrust error the wrong propeller introduced. A propeller correction is only meaningful
as part of a re-tune against flight data, which is a different feature.

**2. There is a standing decision against exactly this.** The 039 wrap records **"NO sim recalibration from
this n=1 airframe"** pending additional articles, and the pitch-lever work (including reduced pitch
aggressiveness) is explicitly gated on n>1. Operator confirmation 2026-07-28: *"the current best M1 is
so-so… at some point we go back to improved fidelity there, but for now we are refining camera track."*

**3. The cost is disproportionate and lands on the wrong axis.** A rebake would:
- serialise 040 behind a full M1 training run, ending the parallelism with flight-test work that the
  feature was scoped to preserve;
- **invalidate the pinned baseline** — both M1 sources were just pinned `retain=keep` specifically so
  SC-008's delta isolates *perception*. A new source makes the only outcome measure confounded;
- change nothing about perception, which runs chase-side at M2 train time and is what this feature exists
  to improve.

### Recommendation

**DEFER**, and file the propeller discrepancy against the flight-model feature that will re-tune with
n>1 articles in hand.

The reasoning that matters is (1), not (3): this is not "too expensive to fix now" but **"fixing it in
isolation would probably make the model worse."** The propeller value and the tuned derivatives are one
fitted system.

---

## Decision (T005 — operator)

> ### ✅ **DEFER** — decided by operator, 2026-07-28
>
> - [x] **DEFER** — proceed to Stage B; propeller discrepancy filed for the flight-model feature
> - [ ] ~~REGENERATE~~
>
> **Consequences now binding on 040**:
>
> 1. **No M1 source regeneration anywhere in this feature** (FR-025, SC-009). The existing sources stand,
>    and the `retain=keep` pins applied at T003a remain the basis for SC-008's comparison.
> 2. **Stage B opens.** Everything downstream may assume the existing M1 source is valid.
> 3. **The propeller discrepancy is filed, not forgotten** (T006) — it belongs to the flight-model re-tune
>    gated on n>1 articles, not to a perception feature.
> 4. **The plant is unchanged**, so 040 remains parallel to the M1-controller flight-test track, which is
>    what the feature was scoped to preserve.
>
> **Reasoning of record**: the propeller value and the tuned derivative set are one fitted system.
> Correcting `D`/`H` in isolation would break a fit that was made against observed flight, and could make
> the model *less* faithful rather than more. Deferral is therefore the technically correct call, not
> merely the cheap one — and it is reinforced by the standing 039 "no recalibration from an n=1 airframe"
> decision and by the acknowledged mediocrity of the current best M1 (spec Assumption 13a), which places
> flight-model fidelity work in its own feature.
