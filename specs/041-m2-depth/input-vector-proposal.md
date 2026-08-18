# Proposed NN input vector — post-TA01/TA03

**2026-08-17.** Revises the 42/63 layout on the evidence in
[ablation/ta01-t1-elite-findings.md](ablation/ta01-t1-elite-findings.md). Every range below is either a
measured value from the t1 run or a stated design choice — none are guesses carried forward.

## The structural fact worth building on

Operator 2026-08-17: *"sensor inputs to m1 and m2 are the same except for target representation."*
Confirmed exactly:

| block | M1 | M2 |
|---|---:|---:|
| **shared** (craft state, limits, energy) | **17** | **17** — identical names, identical semantics |
| target representation | 25 | 46 |
| total today | 42 | 63 |

⚠️ Today those 17 shared slots are declared **twice**, in two unrelated structs (`NNInputs`,
`TrackerInputs`). That is the same parallel-definition hazard the index-coupling inventory exists to
retire — a change to `ACCEL_Z`'s meaning must be made in two places or the modes silently diverge.
**Recommend a shared `CraftCommonInputs` sub-struct** embedded in both, so the shared block has one
definition. Layout order is preserved; only the declaration is factored.

---

## SHARED block — proposed 20 slots (was 17)

| # | field | source | range (measured / by construction) | change |
|---|---|---|---|---|
| 1–4 | `QUAT_W/X/Y/Z` | `AircraftState::getOrientation()`, q_EB aerospace FRD | [−1, 1], unit norm | — |
| 5 | `AIRSPEED` | `getRelVel()`, **raw m/s** | ~8–25 m/s (cruise 13) | — |
| 6–8 | `GYRO_P/Q/R` | `getGyroRates()`, body rad/s | ±~6 rad/s | — |
| 9–11 | `ACCEL_X/Y/Z` | `specific_force.h` ÷ `kAccelScale_g` (8) | measured: X [−0.19, +0.21], Y [−0.07, +0.07], **Z [−1.39, +0.50]** | ⭐ **RETAINED incl. ACCEL_Y** |
| 12 | **`SPECIFIC_ENERGY`** | `Es = h_hd + v²/2g` ÷ `kEnergyScale_m`, where `h_hd` = height above the **arena floor (hard deck)** | measured p05 **27.1** / med **43.5** / max **98.0** m ⇒ **[0.19, 0.71]** at scale 139 | 🆕 **NEW** |
| 13 | **`BOUNDARY_CLOSURE_RATE`** | outward radial velocity `(p·v)/‖p‖` ÷ `kCruiseSpeed_mps` | measured p05/p95 **−17.1 / +17.0 m/s**, std 11.4 ⇒ ~[−1.3, 1.3] | 🆕 **NEW** (replaces the time-to-boundary idea) |
| 14 | `DIST_TO_BOUNDARY` | `tanh(d / 20 m)` | measured [0.12, 1.0], >0.95 for 92.8% of ticks | ⭐ **KEEP — see below** |
| 15–17 | `INWARD_BODY_X/Y/Z` | `inwardBodyDirection()`, unit vector | [−1, 1] | — |
| 18–20 | **`SCORE_GRAD_X/Y/Z`** | ∂score/∂position (body frame) × streak multiplier | design target ±1 after scaling | 🆕 **NEW**, replaces the envelope pair |
| — | ~~`IN_ENVELOPE`~~ | — | — | ❌ **REMOVE** |
| — | ~~`ENVELOPE_SECS`~~ | — | — | ❌ **REMOVE** |

### Why each change

**`ACCEL_Y` retained.** I proposed dropping it (lowest contribution, 0.014, because coordinated turns pin
lateral accel near zero). **Ablation reversed that**: −4.5% on path 5, *worse than all three accel axes
together*. All-attitude flight has sideslip; that is exactly where lateral accel earns its place, and path 5
is the field-representative path.

### ⚠️ TIME-TO-BOUNDARY IS RETRACTED — the vehicle never flies straight

The earlier proposal (FR-035) was `distance_along_vel / speed`, i.e. time-to-impact assuming straight
flight. **Operator 2026-08-17: *"time to boundary is tricky given that neither target, nor chase will be
flying very often in straight line at any attitude."*** Measured on path 5, that is an understatement:

| | path 5 |
|---|---|
| median turn radius | **11.9 m** (arena radius 80 m — 14.8%) |
| ticks with turn radius < 20 m | **66.6%** |
| full circle at median turn rate | **4.8 s** |

A 3-second straight-line ray reaches 46 m, but the craft turns ~225° in that time. The ray points somewhere
it will never be, so time-to-boundary would be a **systematically wrong input** — worse than none, because
it looks authoritative. Retracted.

**Replacement: `BOUNDARY_CLOSURE_RATE`** — the outward radial velocity component, `(p·v)/‖p‖`. It makes
**no trajectory assumption whatsoever**: it states how fast the wall is being approached right now,
whatever the path shape. Two further arguments for it:

1. **It is informative exactly where distance is blind.** `DIST_TO_BOUNDARY` is saturated (>0.99) on 83% of
   ticks with std 0.042; on those same ticks closure rate still spans **−17.3 to +16.5 m/s**. Position says
   nothing there; rate says plenty.
2. **It mirrors a pattern already in the vector.** The target block carries `DIST_*` **and**
   `CLOSING_RATE` — position plus rate. The boundary had only position. This makes the boundary block
   consistent with the target block rather than inventing a new idiom.

⚠️ An arc-based projection (using turn rate to follow the actual circular path) is the more sophisticated
option and is **not** proposed: it swaps a straight-line assumption for a constant-turn-rate assumption,
and at 3.9 rad/s p95 the turn rate is not constant either. The derivative is assumption-free; prefer it
until something proves it insufficient.

**`DIST_TO_BOUNDARY` kept, NOT replaced.** FR-035 originally said "replace or supplement". Ablation settles
it: **−40.7% pooled / −25.0% on path 5, the third most important input in the vector.** The closure rate is
an **addition**, not a substitution. ⚠️ The two are complementary, not redundant: distance says *where the
wall is*, rate says *whether it is getting closer*. The saturation problem (93% of ticks above 0.95) is
fixed by adding the rate, not by removing the distance.

**Envelope pair removed.** Ablation: zeroing `IN_ENVELOPE` **improves** path-5 score by 0.3%;
`ENVELOPE_SECS` costs 0.2%. Both inside noise. They occupy two slots and buy nothing.

**`SCORE_GRAD_*` replaces them** (FR-039). Operator: *"streak was a crude proxy for rewarding in-track
range."* A binary flag is a **state label** — a controller can only switch on it. A gradient is an
**improvement direction**: which way to move, in body axes, to score more. Analytically available from the
Lorentzian in `FitnessComputer::decomposeStepScore`, so no estimation is required in M1. Multiplying by the
streak multiplier weights it by how much reward is currently at stake.

⚠️ **`SCORE_GRAD_*` is a shared SLOT with a mode-specific SOURCE**, exactly as `IN_ENVELOPE` was:
- **M1** — exact. The virtual target's geometry is known in sim *and* in flight, so there is no sim-to-real
  gap.
- **M2** — must be **perception-derived**, never the true geometry. Feeding true ∂score/∂position to a
  tracker that cannot see it in the air would be an oracle, and would train a policy that cannot fly.
  This is the same discipline T038 applied to the M2 envelope estimator.

**`SPECIFIC_ENERGY` — the missing observation.** TA03 found the vector carries `AIRSPEED` but **no
altitude/height/AGL term at all**, so `Es = h + v²/2g` was unobservable: the policy had the `v²` half and
never the `h` half. That is the mechanical explanation for 035's energy objective muting the whole
regiment.

### The datum: height above the HARD DECK, not AGL and not the virtual origin

Operator 2026-08-17: *"agl vs the virtual origin — probably ok for now — or elevation above hard deck, the
bottom of the arena perhaps?"* **Take the hard deck**, and the code says the reason is stronger than
convenience:

⚠️ **AGL is NOT reproducible in flight.** In sim, `checkArenaBounds` derives
`alt_agl = -(pos.z + SIM_INITIAL_ALTITUDE)` — ground-referenced, because the sim knows where the ground is.
In flight, `resolveEngageArena` builds the band as `floor_z_ned = z_engage + K` — **engage-centred**,
because the aircraft does not know ground height. An AGL input would therefore have been a sim-only
quantity: exactly the same class of error as the virtual-origin datum it was meant to avoid.

**Height above the arena floor is defined identically in both**, because both carry an explicit floor.
`h_hd = floor_z − z` in each frame's own terms. Three further arguments:

1. It is the **operationally meaningful** quantity — energy above the deck is energy you can actually
   spend; below it is a crash.
2. It **unifies FR-034 with FR-037**: running out of energy and hitting the floor are the same failure, so
   containment and energy management stop being separate concerns.
3. "Hard deck" is the air-combat term for exactly this, which keeps it consistent with the Boyd E-M framing
   the rest of the proposal borrows.

Measured on t1 with the hard-deck datum: `h_hd` p05 14.1 / med 28.4 / max 77.7 m; `v²/2g` med 14.2, max
43.6 m; **`Es` p05 27.1 / med 43.5 / max 98.0 m**. Zero ticks below the deck — and negative `Es` is
*meaningful* (below the deck), not an error to clamp.

### ⚠️ PRE-EXISTING GAP FOUND WHILE CHECKING THIS — the band is placed differently in sim and flight

| | floor relative to engage | ceiling relative to engage |
|---|---:|---:|
| **sim** (5–100 m AGL, engage at 25 AGL) | **20 m below** | 75 m above |
| **flight** (`resolveEngageArena`, K = 47.5) | **47.5 m below** | 47.5 m above |

Same 95 m band, **different placement**. A policy trained with 20 m of room beneath it will fly with 47.5 m.
This is **not introduced by `SPECIFIC_ENERGY`** — it already affects `DIST_TO_BOUNDARY`, which the ablation
just showed is the third most important input in the vector. It deserves its own decision (centre the sim
arena on engage, or ground-reference the flight arena when altitude is trustworthy) and is filed rather
than fixed here.

**Mitigation for the input itself**: normalise by the **band** (`ceiling − floor` = 95 m in both), so the
slot means *"energy height as a fraction of my usable vertical band"* — identical semantics either side,
even while the placement question is open.
⚠️ Give **`Es` (the state), not only `Ps` (the rate)**. `Es` is the integral; making the recurrent layer
accumulate it wastes capacity already unfilled (effective rank 11.3 of 16). `Ps` remains available to the
*objective* without being an input.

---

## M1 target representation — 25 slots, unchanged

| # | field | source | range |
|---|---|---|---|
| 21–38 | `TARGET_X/Y/Z_TM5…NOW` | direction cosines, body frame, 6 ms-based lags | [−1, 1] |
| 39–44 | `DIST_TM5…NOW` | raw metres to rabbit | ~0–120 m |
| 45 | `CLOSING_RATE` | ΔDIST over the NOW↔TM1 lag gap | ±~25 m/s |

**M1 total: 45** (was 42).

## M2 target representation — 46 slots, unchanged

Ranges measured on the pinned 040-t4 tracker run (131 802 ticks). ⚠️ **That run was captured at the OLD
120° × 90° field**; at the measured 97.3° × 60.8° the bearing bounds tighten and the blind fraction rises.
Treat the bearing extremes as an upper bound and the blind fraction as a **lower** bound.

| # | field | source | range (measured on 040-t4) |
|---|---|---|---|
| 21–32 | `BEACON_L_X/Y_TM5…NOW` | left-wingtip beacon bearing, camera frame, **radians**, 6 ms-based lags | bounded by the half-field: ±0.849 H / ±0.531 V at 97.3°×60.8° |
| 33–38 | `BEACON_L_CEP_TM5…NOW` | per-beacon circular error probable, normalised | **[0.02, 1.5]**; **≥1.25 = the visibility sentinel** (invisible) |
| 39–50 | `BEACON_R_X/Y_TM5…NOW` | right-wingtip beacon, same | as left |
| 51–56 | `BEACON_R_CEP_TM5…NOW` | same | **[0.02, 1.5]** |
| 57–62 | `BEACON_PAIR_SPAN_TM5…NOW` | ‖right.xy − left.xy‖ in **radians**, CEP-gated (0.0 when either endpoint is untrusted) | med **0.039**, p95 **0.153**, max 0.99 |
| 63 | `SPAN_RATE` | d(span)/dt — the range-closure proxy | Δspan p05/p95 **−0.008 / +0.009** per tick |
| 64–65 | `TARGET_TILT_SIN/COS` | port→starboard tilt of the beacon pair (target roll cue) | [−1, 1]; `cos` med **0.88** (mostly upright) |
| 66 | `TIME_SINCE_SEEN` | `tanh(blind_seconds / 2.0)` | med **0.0**, p95 **0.52**, max 1.0 |

⚠️ **20.3% of ticks had the left beacon at or beyond the CEP sentinel** — i.e. blind — at the *old, wider*
field. This is the number FR-024b's blind-gap distribution quantifies, and the reason `TIME_SINCE_SEEN`
and the span-gating exist at all. It will be **worse** at the measured field.

**Structural note**: M2 has **no direct distance and no direct bearing to the target** — only beacon
bearings and their separation. Range is inferred from `SPAN` (wider = closer), which is why span and its
rate carry the load that `DIST_*`/`CLOSING_RATE` carry in M1. That is the whole of the M1↔M2 difference:
**M1 is told where the target is; M2 must infer it from two points of light.**

**M2 total: 66** (was 63).

---

## New constants required

| constant | proposed | rationale |
|---|---|---|
| `kEnergyScale_m` | **139.0** | band (95 m) + max observed kinetic term (43.6 m). Measured Es lands in **[0.19, 0.71]** — comfortably inside the unit without saturating, and with headroom above the observed max |
| *(none — `kCruiseSpeed_mps` reused)* | 13.0 | `BOUNDARY_CLOSURE_RATE` normalises by cruise speed, an existing constant. Measured ±17 m/s ⇒ ~±1.3, which is tanh-friendly without a new knob |
| `kScoreGradScale` | **TBD from data** | ∂score/∂position magnitude ≈ 1/`FitDistScale` (~0.14 m⁻¹) × multiplier (1–5). Measure on recorded ticks before fixing, the way `kAccelScale_g` was sized from the ±11 g record |

⚠️ `BOUNDARY_CLOSURE_RATE` is **rate-derived** ⇒ millisecond-denominated and cadence-invariant, with a
two-cadence test (the `ENVELOPE_SECS` T040(e) pattern). Same for anything built on `Ps`.

## Fitness vector — separate from the inputs

TA03: `corr(Ps, closure rate) = −0.048`. The energy axis is **orthogonal** to the tracking axis, which is
the ideal case for **lexicase** — complementary selection pressure, not the Pareto-corner collapse that
scalar-aggregated smoothness produced in 033. Add `Ps`-based efficiency as a **lexicase axis, never as a
scalar penalty term**, and never without slot 12 in place: an axis for an unobservable is what muted 035.

## Net effect

| | before | after |
|---|---:|---:|
| M1 | 42 | **45** (+3: Es, closure-rate, grad×3, −2 envelope) |
| M2 | 63 | **66** |
| shared block | 17 (duplicated) | **20 (one definition)** |
