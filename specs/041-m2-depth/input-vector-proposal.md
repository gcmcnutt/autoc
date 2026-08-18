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
| 12 | **`SPECIFIC_ENERGY`** | `Es = h_agl + v²/2g`, ÷ `kEnergyScale_m` | Es measured 5–110 m ⇒ **[0.05, 1.1]** at scale 100 | 🆕 **NEW** |
| 13 | **`TIME_TO_BOUNDARY`** | `distanceToBoundary(along vel) / speed`, `tanh(t / kTtbScale_s)` | t ~0–12 s in an 80 m arena at 13 m/s ⇒ [0, 1) | 🆕 **NEW** |
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

**`DIST_TO_BOUNDARY` kept, NOT replaced.** FR-035 originally said "replace or supplement". Ablation settles
it: **−40.7% pooled / −25.0% on path 5, the third most important input in the vector.** Time-to-boundary is
an **addition**, not a substitution. ⚠️ The two are complementary, not redundant: distance says *where the
wall is*, time says *how long until you hit it at current velocity*. The saturation problem
(93% of ticks above 0.95) is fixed by adding the time signal, not by removing the distance one.

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
⚠️ **Compute `h` as AGL, not from the sim's virtual-frame z.** The dmp's position is engage-relative
(z ≈ 0 at start); AGL is what exists in flight. Using the virtual datum would train against a quantity the
aircraft cannot reproduce.
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

`BEACON_L_X/Y/CEP` ×6, `BEACON_R_X/Y/CEP` ×6, `BEACON_PAIR_SPAN` ×6, `SPAN_RATE`, `TARGET_TILT_SIN/COS`,
`TIME_SINCE_SEEN`. Bearings in radians against the measured 97.3° × 60.8° field; CEP normalised with 1.25
as the visibility sentinel.

**M2 total: 66** (was 63).

---

## New constants required

| constant | proposed | rationale |
|---|---|---|
| `kEnergyScale_m` | **100.0** | arena ceiling 100 m AGL + cruise kinetic term ~8.6 m ⇒ Es lands in ~[0.05, 1.1], tanh-friendly without saturating |
| `kTtbScale_s` | **3.0** | `tanh(t/3)` keeps resolution across 0–6 s, the window where corrective action is still cheap; beyond ~10 s the distinction stops mattering |
| `kScoreGradScale` | **TBD from data** | ∂score/∂position magnitude ≈ 1/`FitDistScale` (~0.14 m⁻¹) × multiplier (1–5). Measure on recorded ticks before fixing, the way `kAccelScale_g` was sized from the ±11 g record |

⚠️ `TIME_TO_BOUNDARY` is **rate-derived** ⇒ millisecond-denominated and cadence-invariant, with a two-cadence
test (the `ENVELOPE_SECS` T040(e) pattern). Same for anything built on `Ps`.

## Fitness vector — separate from the inputs

TA03: `corr(Ps, closure rate) = −0.048`. The energy axis is **orthogonal** to the tracking axis, which is
the ideal case for **lexicase** — complementary selection pressure, not the Pareto-corner collapse that
scalar-aggregated smoothness produced in 033. Add `Ps`-based efficiency as a **lexicase axis, never as a
scalar penalty term**, and never without slot 12 in place: an axis for an unobservable is what muted 035.

## Net effect

| | before | after |
|---|---:|---:|
| M1 | 42 | **45** (+3: Es, TTB, grad×3, −2 envelope) |
| M2 | 63 | **66** |
| shared block | 17 (duplicated) | **20 (one definition)** |
