# ⛔ TASK RESET 2026-08-17 — 041 = "a fresh full M1 toolchain, flown"

**THIS BLOCK GOVERNS.** Everything below is the historical 136-task M2-depth list, retained for provenance.
Scope: [spec.md § SCOPE RESET](spec.md) · Phases: [plan.md § PLAN RESET](plan.md).

⚠️ **Checkbox convention after the 2026-08-18 cleanup.** `[X]` means **"no open work in 041"** and carries
one of four dispositions, always stated first in the line:

| marker | meaning |
|---|---|
| ✅ **DONE** | actually completed, with evidence |
| ⛔ **SUPERSEDED** | the same work, renamed into the P-task list — follow the pointer |
| ⛔ **MOVED→043** | M2 tracking, out of scope since the rescope |
| ⛔ **VOID** | the rescope removed the reason for it |

**`- [ ]` therefore means genuinely open, and there are exactly 18 of them — all `Pn-n`.** Original task
text is kept beneath each disposition so cross-references from S3 provenance, the ablation docs and 043's
seed still resolve.

⛔ **MOVED OUT — do not work these here**: T060, T081–T098 and the whole predictor/M2-bake thread go to
**[043](../044-m2-tracking/README.md)**. Camera follow-ups go to **042**. Boundary redesign is already in
`specs/BACKLOG.md`.

## Phase 0 — PREP (no code, no bake)

- [X] P0-1 ✅ **DONE 2026-08-18 — datum inventory MEASURED; no UNVERIFIED cells remain.** Table + evidence in
  [toolchain-datum-validation.md](toolchain-datum-validation.md). Headline: the chain closes to **1.2 mm** —
  crrcsim launches the CG at `82 + zLow(0.125) − ground(0.1) = 82.025 ft = 25.0012 m`, so `<launch
  altitude="82">` and `SIM_INITIAL_ALTITUDE = −25` never disagreed; the gap was an unstated unit (crrcsim is
  foot-native) plus an unstated `zLow`. One real defect found and one bias recorded: `checkArenaBounds`'s
  `alt_agl` is **datum**-referenced, not ground-referenced, and under-reads true AGL by a uniform **30.5 mm**.
  ⛔ **The surprise was elsewhere — the sim runs THREE arenas** (see P0-2).
  ~~Original:~~ ⛔ **Datum inventory — measure, do not infer.** ➕ **Half the answer is known (operator
  2026-08-18)**: *"The initial position in sim is the autoc xml file in CRRCSim we reference plus some aiding
  from variations which are zeroed these days."* So `crrcsim/autoc_config.xml`'s `<launch altitude="82">` **is
  the sim's initial-position source**, and the *position* entry variations are currently off
  (`EntryPositionRadiusSigma = 0.0`, `EntryPositionAltSigma = 0.0`) — note the *attitude/speed* ones are NOT
  (`EntryConeSigma` 18, `EntryRollSigma` 30, `EntrySpeedSigma` 0.06), so entry state still varies, just not
  entry *position*. ✅ **RESOLVED 2026-08-18 — there was no discrepancy, only a unit assumption.** CRRCSim is **foot-native**
  (`FEET_TO_METERS = 0.3048`, `inputdev_autoc.h:53`), so `<launch altitude="82">` is **82 FEET =
  24.9936 m**, matching `SIM_INITIAL_ALTITUDE = 25 m` **to 6.4 mm (0.03%)**. What remains for P0-1 is the
  scenery ground reference and the remaining hops — not this. Fill every UNVERIFIED cell in
  [toolchain-datum-validation.md](toolchain-datum-validation.md). **Ends with the
  `<launch altitude="82">` vs `SIM_INITIAL_ALTITUDE = −25` reconciliation answered by measurement.** (was TD01)
- [X] P0-2 ✅ **DONE 2026-08-18 — HAT analysis, and it found the real defect.** Davis Field's ground is a
  **flat plane at −0.1 ft (−0.03048 m)** — `BuiltinSceneryDavis::getHeight()` returns a constant, so there is
  no field-elevation or terrain-relief question to answer at all. What the analysis actually surfaced:
  ⛔ **`DIST_TO_BOUNDARY` describes a cylinder the M1 aircraft cannot reach and does not describe the one that
  kills it** — kill = `checkAircraftOOB` at 70/7/120, input = `FlightArena` defaults at 80/5/100 (10 m wider,
  20 m shorter). TA01 ranks that input the **3rd most important in the vector**.
  ⭐ And the measurement that reopened P0-4: **the M1 targets never descend below the entry altitude** and
  climb to **+49.97 m** above it ([measure/path_altitude_extents.cc](measure/path_altitude_extents.cc)).
  ~~Original:~~ **HAT analysis** — crrcsim's ground, field elevation and starting height-above-terrain: what they
  are, and where they must land under a unified datum. Operator: *"we need to make sure we twiddle the
  offsets for crrcsim too."*
- [X] P0-3 ✅ **DONE 2026-08-18 — `kScoreGradScale` = 0.78 m⁻¹**, the in-envelope p95 of `|∇score| × streak
  multiplier` over **131 127 recorded t1 ticks** ([measure/README.md](measure/README.md)). Sized from data, not
  picked: at that divisor the in-envelope median (0.2016) reads 0.26 and 95 % of in-envelope ticks fall below
  `tanh(1)` — the same shape `kAccelScale_g` was given by the ±11 g record.
  ⚠️ **The measurement also changed the ENCODING.** `|∇θ| = 1/d` diverges as the chase closes; every tick
  above 2.0 has `d < 1.52 m` (median 0.23 m). The quantity is genuinely unbounded, so a plain divide is wrong
  for this slot. Encode as a **direction-preserving tanh of the norm** —
  `v_body = ĝ_body · tanh(|∇score|·mult / kScoreGradScale)` — not per-component tanh, which would bound the
  slot while **rotating the vector** the input exists to communicate.
  ~~Original:~~ **Size `kScoreGradScale` from recorded ticks** — the last unsized constant in the new input
  vector. ⚠️ Size it from *data*, the way `kAccelScale_g` was sized from the ±11 g flight record, never
  picked as a round number. Expected magnitude ≈ 1/`FitDistScale` (~0.14 m⁻¹) × streak multiplier (1–5), but
  measure rather than assume. *(restored 2026-08-18 — this task was lost in an edit that replaced a
  neighbouring block; the loss is noted rather than silently repaired.)*
- [X] P0-4 ✅ **DONE 2026-08-18 — REOPENED by P0-2's measurement, then re-decided by the operator.**
  The first decision ("entry at the 3D centre" + "deck above ground", at today's 25 m entry) turned out to be
  **unsatisfiable**: the targets reach 74.98 m AGL, and a band centred on 25 m that reaches 75 m needs its
  floor 25 m underground. Operator resolution — *"Raise sim frame… We do want Xiao arm at virtual origin.
  Pilot responsible for that being way above terrain so that bottom of cyl is above ground. So really sim
  should do identical."* Geometry: *"70m radius. Hat of perhaps 10 (bottom of cyl) top at 100m above bottom."*
  ⇒ **radius 70 / floor 10 AGL / ceiling 110 AGL, K = 50, entry & arm at 60 m AGL = exact mid-cylinder**,
  `SIM_INITIAL_ALTITUDE` −25 → **−60**, `<launch altitude>` 82 → **196.825 ft**. ⭐ Under those numbers
  `resolveEngageArena(...).virtual_arena` **equals the sim's `FlightArena` identically** — sim and flight stop
  agreeing by convention and start agreeing by construction. Implementation is P2-3.
  ➕ **REVISED TWICE MORE by operator messages during implementation** — final geometry is **R = 70, K = 48**,
  landing in sim at floor **25** / entry **73** / ceiling **121** m AGL, `SIM_INITIAL_ALTITUDE` **−73**,
  `<launch altitude>` **239.476 ft**. ⛔ The arena is **RELATIVE** (±K about the arm origin); staying above
  terrain and inside the site's 400 ft envelope are **arm-time operator responsibilities**, deliberately not
  encoded (*"Not the models problem. Is operator."*).
  ⚠️ **The chase still does not fit and that is accepted**: measured entry **+66.2 m** against K = 48, so
  ~1 % of t1-like ticks now egress at the ceiling — intended pressure against the zoom the `Ps` axis also
  charges for. **First thing to check in the P3-4 smoke** (`ArenaEgressKind::CEILING`); if it bleeds
  scenarios, **K is the knob, not the objective**.
  Full derivation, the three near-misses, and the acceptance criteria:
  [toolchain-datum-validation.md](toolchain-datum-validation.md).
  ~~Original:~~ ➕ **ARENA — DECIDED 2026-08-18, now implementation not research.** Operator: *"Make it the same
  size for both cases. And generally entering at the origin in the 3d center of the cylinder. For sim we need
  to ensure manually that the hard deck bottom of arena is above ground. And this is a manual decision in
  flight for now. We just start way above ground in these early runs."*
  1. **Same arena size both sides** — one radius, one height, sim and flight.
  2. **Entry at the 3D centre of the cylinder** — mid-band vertically *and* centred horizontally. Flight
     already does the vertical half (`resolveEngageArena`, ±K); **the sim does not** (engages 21% up its
     band) — that is the change.
  3. **Hard deck placement is MANUAL, both sides.** Sim: ensure by configuration that the arena bottom sits
     above the crrcsim ground. Flight: an operator decision, mitigated by **starting well above ground** in
     these early runs.
  ⛔ **No HAT sensor, and no sensing question to answer.** The earlier framing asked whether baro/GPS/
  rangefinder could supply a deck; the answer is that none is needed yet — start high enough that a
  mis-placed deck cannot be reached. Revisit only when flights get low or terrain varies.
  ⚠️ Still true: whatever deck value is used **must be recorded in the flight log**, or `Es` cannot be
  reconstructed post-flight.
  ➕ **INAV is the source of truth for position/altitude in these phases** (operator 2026-08-18) — *"Don't
  worry about gps/baro drift for now."* So no drift budget, no altitude-fusion work, and no second opinion:
  what INAV reports is what the arena and `Es` are computed against.

- [X] P0-5 ✅ **DONE 2026-08-18 — ramp DISABLED. Decision, not an experiment.** `VariationRampStep` → **0**
  in `autoc.ini`, `autoc-basic-m1.ini` and both their eval counterparts. ⚠️ **All four variation classes stay
  ON** — this was never a variations change.
  **Effect**: `computeVariationScale()` returns `1.0` when `numSteps <= 1`, so variation is at **full scale
  from generation 1** rather than climbing 0→1 over 20 steps. Harder early, **stationary** throughout — and
  stationarity is the point.
  **Why no A/B** (I had proposed one; operator overrode): *"Nah disable ramp. Arguably the m1 paths are
  simpler given they are computed geometry."* ⚠️ **That inverts my caveat.** I had argued M1 might be more
  fragile than M2 under full variation because M1 generates its own paths. The opposite is true where it
  matters: M1's targets are **computed analytic geometry** (StraightAndLevel, SpiralClimb, FigureEight,
  AngledLoop, HighPerchSplitS, SeededRandomB) while M2 tracks a **recorded flight trajectory** with all its
  noise. M1 is the *easier* tracking problem, and M2 has tolerated ramp-free since t8.
  **Supporting**: the ramp caused all 11 of t1's elite-fitness regressions, and a moving objective swamps the
  small energy/aggressiveness signals 041 exists to measure.

### ➕ Methodology note (operator 2026-08-18) — match the test to the size of the change

*"We are looking for improvements in many dimensions. So the eyeball test is good for bigger changes like nn
inputs."*

⛔ **Do not A/B every change.** A controlled comparison is for **small signals** — an ablation delta, a
regression band, an energy effect measured against noise. For **large** changes (the input vector, the ramp,
the objective's shape) the difference is visible in the ordinary report set, and an A/B costs a full run pair
to confirm what the charts already show. This applies directly to Phase 4: judge the new M1 on its charts
against the pinned prior M1, not on a purpose-built controlled pair.

## Phase 1 — ANALYSIS (reads only) — closing, not opening

- [X] P1-1 ✅ TA01 ablation on the pinned t1 elite — **H1a fails**; `DIST_TO_BOUNDARY` 3rd most important;
  screens mis-rank in both directions. [findings](ablation/ta01-t1-elite-findings.md)
- [X] P1-2 ✅ TA03 `Es`/`Ps` from existing dmps — spiral bleeding energy; `Ps` orthogonal to progress
  (r = −0.048), 24–32 m/s spread at matched progress.
- [X] P1-3 ⛔ **CLOSED 2026-08-18 without doing it — reassessed, as the task asked.** A conditional
  contribution view would have prevented **one** of TA01's two mis-rankings, not both, and the one it misses
  is the one that matters.
  * ✅ `DIST_TO_BOUNDARY` (saturated >0.95 on 93% of ticks, ranked low by weight, actually **3rd most
    important**) — conditioning on the unsaturated 7% WOULD have surfaced it.
  * ⛔ `IN_ENVELOPE` (ranked **2nd by contribution**, yet ablation shows zeroing it *improves* path-5 score)
    — **no conditioning fixes this.** Contribution measures weight × variance; it cannot measure causal
    dependence, and no subset of ticks turns a correlational metric into a causal one.
  Since ablation is now cheap (the elite is pinned, T049/T052 masking is implemented) and SC-015 already
  demotes contribution to *"at most a screen"* with ablation as the verdict, a half-fix to the screen buys
  nothing. Rely on ablation. ~~Original:~~ **[LOW — reassess before doing]** Conditional contribution for
  limit-class inputs. ⚠️ Value
  dropped since it was filed: SC-015 now demotes contribution to *"at most a screen"* and names **ablation as
  the verdict**, after the pooled metric mis-ranked inputs in **both** directions. Worth doing only if a
  cheap conditional view would have prevented that — otherwise close it and rely on ablation. Original:
  conditional contribution for limit-class inputs — pooled averages nearly deleted the third most
  important input in the vector. (was TA02)

## Phase 2 — IMPLEMENT (ONE format break, one owed re-bake)

⛔ **Constitution I (Testing-First) applies to every task below — currently the largest gap in this plan.**
Phase 2 changes the input vector, the arena frame and the persistence schema at once; the 041 T041e episode
showed what that costs untested — four projection tests carried literals silently encoding a retired
60° half-field, and one passed *only* because `0.375` and `120` happen to be exactly representable in binary.
Per task, before the implementation:

| task | test obligation |
|---|---|
| P2-1 `CraftCommonInputs` | layout/count assertions hold for BOTH modes after the split; struct size == COUNT × 4, no padding |
| P2-2 inputs | semantics per slot (the T040 pattern): `Es` non-negative and reconciling with airspeed; `BOUNDARY_CLOSURE_RATE` sign convention (+ = toward wall) and **cadence-invariance at two rates**; `SCORE_GRAD_*` zero at the score maximum and pointing uphill off it |
| P2-3 arena | egress fires at exactly the configured floor/ceiling/radius; **engage lands mid-cylinder** in sim as it does in flight |
| P2-4 recording | round-trip: every emitted column reads back; `Es`/`Ps` survive serialisation |
| P2-5 `Ps` axis | ⚠️ **the muting guard** — a policy that climbs toward a HIGH target must not score worse than one that does not. This is the one test that would catch 035's failure re-entering |
| P2-6 land it | full suite green with **banner count verified**, not just exit status |


- [X] P2-1 ✅ **DONE — `CraftCommonInputs`, 20 slots, one definition.** Embedded as the last member of both
  `NNInputs` (M1: 25 target + 20 common = **45**) and `TrackerInputs` (M2: 46 target + 20 common = **66**).
  ⚠️ **`TrackerInputs` was REORDERED to make this possible** — the craft slots used to be split around the
  derived perceptual block, and a shared sub-struct cannot be non-contiguous. Legal inside the format break.
  ⭐ **The refactor immediately found a live divergence**: `AIRSPEED` was RAW m/s in M1 and CRUISE-NORMALIZED
  in M2 — the same "shared" slot carrying two different scales. Unified on raw. That is precisely the hazard
  the shared struct exists to retire, and it was already present.
  Also folded: `DIST_TO_BOUNDARY_ALONG_VEL` → `DIST_TO_BOUNDARY` (two names, one quantity, one function).
  Tests: `CraftCommonBlockIsIdenticalInBothModes` walks both meta tables asserting name / display-name /
  width / relative order, plus `offsetof` in both structs — the enums are still written twice (C++ enums do
  not compose) and nothing else stops them drifting. ~~Original files:~~ `include/autoc/nn/nn_inputs.h`,
  `include/autoc/nn/topology.h` (counts, weight `static_assert`s). Tests: `tests/contract_evaluator_tests.cc`,
  `tests/nn_sensor_interface_tests.cc`.
  Shared **`CraftCommonInputs`** sub-struct: the 17 slots M1 and M2 share get **one definition**
  instead of two. Operator: *"sensor inputs to m1 and m2 are the same except for target representation."*
- [X] P2-2 ✅ **DONE — input vector 42→45 / 63→66 (net +3 each).** Added `SPECIFIC_ENERGY`,
  `BOUNDARY_CLOSURE_RATE`, `SCORE_GRAD_X/Y/Z`; removed `IN_ENVELOPE`, `ENVELOPE_SECS`; retained `ACCEL_Y` and
  `DIST_TO_BOUNDARY` on ablation evidence. Weight counts recomputed: **2307** (M1), **3047** (M2).
  New shared headers: `include/autoc/eval/energy_state.h` (Es / Ps / `heightAboveDeck`) and
  `include/autoc/eval/craft_observations.h` (the producer-side writer both modes call).
  `FitnessComputer::scoreGradientWorld` lives next to `decomposeStepScore` so the gradient and the score
  cannot be derived from different constants.
  ⚠️ **Two design decisions came out of the P0-3 measurement, not out of the proposal**:
  1. **`SCORE_GRAD_*` is encoded as a direction-preserving `tanh` of the NORM**, not a plain divide.
     `|∇θ| = 1/d` diverges near the rabbit — measured, every t1 tick above 2.0 had d < 1.52 m — so the
     quantity is genuinely unbounded. Per-component `tanh` would bound it while **rotating the vector**,
     and the direction is the entire content of the input.
  2. **`SCORE_GRAD_*` is DELIBERATELY ZERO IN M2**, written explicitly rather than left to a default.
     Today's M2 tracks a recorded flight through a camera; the exact ∂score/∂position would be an oracle it
     cannot reproduce in the air — the same refusal T038 made for the M2 envelope estimator. 043 decides the
     source (phase 1 exact, phase 2 span-proxied).
  ⚠️ Also retired: the `EnableEnvelopeInputs` config knob, which gated inputs that no longer exist.
  Tests: 8 new semantics tests in `envelope_accel_inputs_tests.cc`. The gradient is verified by **central
  finite difference against the objective itself**, not against a second copy of its own algebra —
  agreement to 1e-4 relative across behind / ahead / at-the-π/2-clamp offsets. ~~Original files:~~
  (42→45 / 63→66, recompute asserts), `src/nn/evaluator.cc` (BOTH gathers). **Producers** must set the new
  state: `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`, `src/eval/tracker_stepper.cc`,
  `include/autoc/eval/aircraft_state.h` (carriers). New shared header for `Es`/`Ps` alongside
  `include/autoc/eval/specific_force.h`. Tests: `tests/envelope_accel_inputs_tests.cc` (extend),
  `tests/contract_evaluator_tests.cc`.
  Input vector per [input-vector-proposal.md](input-vector-proposal.md): **add**
  `SPECIFIC_ENERGY`, `BOUNDARY_CLOSURE_RATE`, `SCORE_GRAD_X/Y/Z`; **remove** `IN_ENVELOPE`,
  `ENVELOPE_SECS`; **retain** `ACCEL_Y` and `DIST_TO_BOUNDARY` on ablation evidence.
- [X] P2-3 ✅ **DONE — ONE arena, 70 / 10 / 110, entry at its exact vertical centre.**
  `SIM_INITIAL_ALTITUDE` −25 → **−60**; `<launch altitude>` 82 → **196.825 ft**; `FlightArena` defaults and
  all eight .ini files updated; **M1 now terminates on `checkArenaBounds`** against the same struct the
  gather reads (`checkAircraftOOB` and `SIM_MIN/MAX_ELEVATION` deleted outright, no shim).
  ⭐ **`resolveEngageArena(...).virtual_arena` is now IDENTICAL to the training `FlightArena`** — with entry
  at the band centre it comes out an identity, so sim and flight stop agreeing by convention and start
  agreeing by construction. `ArenaDatum.ResolveEngageArenaReproducesTheTrainingArenaExactly` asserts it at
  three different engage altitudes.
  ⛔ **`SIM_INITIAL_ALTITUDE` DID move, against this task's original "unchanged" instruction** — the P0-2
  measurement showed the alternative was unsatisfiable, and the operator re-decided (see P0-4). It is NOT
  the retracted origin→hard-deck proposal: z = 0 is still the entry point, so no dmp reader's frame changes
  meaning; the scene simply sits 35 m higher over the terrain.
  ⚠️ The two constants that must agree (`SIM_INITIAL_ALTITUDE`, a compile-time macro, and the arena bounds,
  runtime config) are coupled by nothing in the type system, so
  `ArenaDatum.EntryPointIsTheExactVerticalCentreOfTheBand` is the coupling. Move one without the other and
  it fails.
  ➕ **MAX-EXTENT CROSS-CHECK** (operator ask) — new suite
  [`tests/arena_path_fit_tests.cc`](../../tests/arena_path_fit_tests.cc), which links the real
  `generateSmoothPaths` and MEASURES rather than trusting recorded numbers. It found three things nothing
  else would have:
  1. SpiralClimb cleared the proposed ceiling by **3 cm**; shrunk 50 → 35 m (operator-authorized).
  2. Fixing that promoted **HighPerchSplitS** (8.2 m) and then the **seeded** path (3.0 m). Shrunk to
     15 + 15 m, and the seeded path's vertical draw split out as **`SIM_PATH_HEIGHT_BOUNDS = 30`** — the
     arena is not a cube, and one bound for both axes made the seeded path binding.
  3. ⛔ **The xiao generated a DIFFERENT random rabbit from the sim** (`height=100` vs 40 ⇒ an analytic
     envelope of entry +112.5 m against +45 m). A live sim/flight divergence in the TARGET. Both generators
     now share the constants.
  ⭐ The suite also carries the **analytic** bound, not just a seed sweep: uniform Catmull-Rom overshoots its
  control hull by exactly **1/8** at t = ½, so radius ≤ R × 1.250 (a norm) and altitude ≤ entry + H × 1.125
  (a component). ⚠️ Conflating those two factors is an error the test caught while being written — a claimed
  45 m radial bound against a measured 47.43 m. A seed sweep can only ever report the largest thing seen so
  far; a bake runs far more than 400 generations. ~~Original files:~~
  `crrcsim/autoc_config.xml` (`<launch altitude>`, **feet**), `autoc.ini` + the three tracker inis
  (`FlightArenaRadius`, floor/ceiling), `include/autoc/eval/aircraft_state.h` (`SIM_INITIAL_ALTITUDE` —
  **unchanged**, see below). Tests: `tests/arena_tests.cc`, `tests/arena_recenter_tests.cc`.
  ⚠️ **REVISED 2026-08-18 — this is NOT an origin move.** Operator: *"Arena should be same geometry.
  Same radius. Same height. Xiao trigger should be halfway up the cylinder. And sim should be similar… in the
  general sense z sign should never matter… So revisit the need to change arena origin."*
  **Do**: (a) verify sim and flight use the **same arena geometry** — radius and height — and fix the *sizes*
  if they differ; (b) make **engage sit mid-cylinder in the sim**, as it already does in flight
  (`resolveEngageArena`, ±K). Today the sim engages at 25 m AGL in a 5–100 m band, i.e. **21% up, not
  half-way** — that asymmetry is the actual defect, not the coordinate origin.
  **Do NOT**: move `SIM_INITIAL_ALTITUDE` / the virtual origin. That was my proposal (TD02) and the operator
  questioned the need; z sign should not matter to any consumer, and a frame move is a large, risky change to
  buy something the geometry fix already delivers.
  ⚠️ **Why the hard deck must not be baked in as permanent**: *"hard deck or ground will eventually vary"*,
  and *"at some point we are at trigger and craft enters arena and stays there is quite plausible"* — today's
  mid-band entry is a **flight-safety convention**, not physics. Keep `Es`'s deck datum a *computed* quantity
  (height above the configured floor) so a varying deck or an outside-in entry does not invalidate the frame.
- [X] P2-4 ✅ **DONE — `EvalResults` v3 → v4, and dmp-dump now emits EVERY slot.**
  ⭐ **The hand-picked column subset is gone.** `dmp_dump` walks `kPathgenInputMeta` / `kTrackerInputMeta`,
  so the header labels and the values come from ONE source: a slot cannot appear without its value, or be
  emitted under the wrong label, and adding an input to `nn_inputs.h` adds its column with no edit at all.
  Several recomputed-in-the-reader columns (`spn0`/`dspn`/`tltS`/`tltC`/`dBnd`/`inX..`) were deleted for the
  same reason the gathers were consolidated — the reader was guessing a CEP gate threshold that
  `EvalResults` does not carry.
  Plus the UNSCALED sources — `Es_m`, `Ps_mps`, `bClR_ms`, `sgx/sgy/sgz` — recorded on `AircraftState`
  separately from the slots that carry them. Not redundancy: an ablation mask zeroes the slot **by design**,
  `SCORE_GRAD`'s slot is a saturating tanh so its magnitude does not survive, and Ps must be differenced
  from a true Es. `Ps` is computed by `energy_state.h`, **the same function the lexicase axis uses**, over
  the actual recorded interval.
  Tests: `NewCraftObservationsSurviveSerialization_P2_4` (incl. a negative closure rate — the sign is the
  slot's whole content) and `EveryNNSlotSurvivesSerializationInBothModes_P2_4` (each slot filled with its
  own index, so a shift by ONE is visible). ~~Original files:~~
  `tools/dmp_dump.cc` (emit all 45/66 slots + `Es`/`Ps`), `src/autoc.cc` (stamp provenance). Tests:
  `tests/tracker_dmp_roundtrip_tests.cc`, `tests/cereal_version_anchor_tests.cc` (re-anchor).
  Recording: full input vector + `Es`/`Ps` per tick. (was TA04/TA05)
- [X] P2-5 ✅ **DONE — `energy_score` is now metres of specific energy DESTROYED, `Σ max(0, −Ps)·dt`.**
  Replaces 035's convex throttle-command integral, which penalised an ABSOLUTE quantity — and full power is
  genuinely correct when far behind, in a sustained spiral, and under pitch-induced drag, so uniform
  pressure could only quiet everything. Stays a **lexicase axis**, never a scalar penalty; measured
  `corr(Ps, closure rate) = −0.048`, i.e. orthogonal to tracking, which is the ideal case for lexicase and
  the worst case for aggregation.
  Three properties, each load-bearing and each pinned by a test:
  1. ⭐ **Climbing costs EXACTLY zero** (Ps > 0 contributes nothing) — the muting failure, stated as a
     property. `ClimbingTowardAHighTargetIsNotPenalised_MUTING_GUARD`.
  2. ⭐ **Trading height for speed is free** — Es is conserved through the trade, so only drag losses and
     deliberately destroyed energy appear. That is "waste" as opposed to "activity".
  3. **Not telescoping** — `Σ Ps·dt` would collapse to `(Es_end − Es_start)`, gameable by finishing high;
     the clip at zero breaks the cancellation. `IsNotTelescopingAndCannotBeGamedByFinishingHigh`.
  ⚠️ **`energy=` in the per-gen log is the same column name carrying a DIFFERENT quantity and different
  units.** It is not comparable across the 041 boundary; a cross-run plot that mixes them is comparing
  throttle effort with energy waste. Labelled at both emission sites. ~~Original files:~~
  `Ps`-based), `src/eval/fitness_decomposition.cc` (compute), `src/eval/selection.cc` (lexicase axis list).
  ⚠️ Downstream readers of `energy_score` must follow: `tools/dmp_dump.cc`, `src/autoc.cc`,
  `src/analytics/plot_evolution_progress.py`. Tests: `tests/energy_metric_tests.cc`,
  `tests/selection_tests.cc`, **plus the muting guard**.
  Objective: `Ps`-based efficiency as a **lexicase axis**. ⛔ Never a scalar penalty; never without
  P2-2's energy input — an axis for an unobservable is what muted 035.
- [ ] P2-6 [OP] Land it: `rebuild-perf.sh` with banners counted, Constitution VI audit.
  ➕ **Partially prepared 2026-08-18 — here is what is already done and what is genuinely left.**
  * ✅ **Version bump**: `EvalResults` v3 → v4, `CEREAL_CLASS_VERSION` + `kSchemaVersion` + the anchor test
    all moved together.
  * ✅ **Desktop**: incremental build clean (autoc, crrcsim, renderer, dmp-dump, nn2cpp); **46/46 ctest
    suites pass**.
  * ✅ **Tool chain verified end-to-end 2026-08-18**: `nnextractor` extracts `45→32→16→3 / 2307 weights`;
    `nn2cpp -u` generates a 45-input unrolled program carrying the fail-loud guard and **bakes the live arena
    automatically** (70 / 25 / 105 — `BakedArena` derives from `FlightArena` rather than restating it);
    the xiao then builds **SUCCESS** (RAM 53.5%, Flash 44.8%) against that generated file. The placeholder was
    restored byte-exact afterwards (md5 verified), so the guard is back in place.
  * ⚠️ **xiao host compile — TWO blockers cleared, ONE deliberately left failing.**
    - CLEARED (both PRE-EXISTING at HEAD, not introduced by 041 P2): `evaluator.h` dragged
      `fitness_decomposition.h` → `protocol.h` → **cereal**, which does not exist in the embedded
      toolchain; and `NNControllerBackend::setInputMask` **throws**, against `-fno-exceptions`. Both are
      desktop concerns — the firmware runs the nn2cpp-generated program and never selects a genome — so both
      are now fenced with `#ifndef ARDUINO`. ⚠️ The throw was NOT removed: a short ablation mask would ablate
      the wrong columns and produce a clean-looking number answering a different question. Fence the
      instrument, not its safety check.
    - ⛔ **STILL FAILING, ON PURPOSE**: `xiao/src/generated/nn_program_generated.cpp` is baked at **37
      inputs** while the gather now writes **45**. That file compiled CLEAN before this change and the
      unrolled forward pass would have read **past the end of `nn_weights`** — flying on whatever was next
      in flash, with nothing at build or run time saying so. `tools/nn2cpp` now emits a fail-loud
      `static_assert` on every regeneration, and the guard has been hand-inserted into the existing artifact
      so the current state announces itself. **Compiling clean was the dangerous state.**
      **Clears at P4-2**, by regenerating from the pinned genome. Everything else in the firmware builds at
      45 inputs (verified: RAM 53.5%, Flash 44.7%).
  ⛔ **NOT DONE, and it gates P4-3 not P2-6**: the xiao does not PRODUCE any of `SPECIFIC_ENERGY`,
  `BOUNDARY_CLOSURE_RATE`, `SCORE_GRAD_*` — **nor `ACCEL_*`, which has been a silent zero on the xiao since
  041 US4 landed**. All four are computable there (it has position, velocity, the engage-resolved arena and
  the virtual target), but the MSP accel plumbing and the fitness-cone constants are Phase 6 board work
  (T074/T075/T078). **A flight before that wiring lands would fly a policy seeing zeros in four channels the
  sim trained with.**

⚠️ **RNN architecture is NOT changing.** Effective rank 11.1–11.8 of 16, flat over 608 generations —
capacity is not the constraint. Settled; reopen only on new evidence.

## Phase 3 — VALIDATE (per hop, before spending a bake)

- [ ] P3-1 Per-hop datum checks, each against something independent. (was TD05)
  ➕ **PARTIALLY DONE 2026-08-18 — here is the coverage, hop by hop, and what each one is checked
  AGAINST.** The obligation is "each conversion checked against something independent", so the third column
  is the point of the table.

  | hop | status | checked against |
  |---|---|---|
  | 1–3 crrcsim scenery → launch → FDM | ✅ | the code that computes it, read out rather than assumed; closes to **1.2 mm** ([toolchain-datum-validation.md](toolchain-datum-validation.md)) |
  | 4–5 bridge → virtual z | ✅ | same arithmetic, independently: `82.025 ft × 0.3048 − 60` vs the compiled `SIM_INITIAL_ALTITUDE` |
  | 6 `checkArenaBounds` AGL | ✅ | `ArenaDatum.EntryPointIsTheExactVerticalCentreOfTheBand` — the macro against the runtime config, which nothing in the type system couples |
  | 6 egress bounds | ✅ | `arena_tests` — floor/ceiling/radius each fire at exactly the configured value, and the descent/ascent pair now read the SAME 50 m, which IS the centring property |
  | 9 `resolveEngageArena` | ✅ | `ArenaDatum.ResolveEngageArenaReproducesTheTrainingArenaExactly` — flight's arena against sim's, at three engage altitudes |
  | 12 `Es` datum | ✅ | `SpecificEnergyIsMeasuredFromTheHardDeck_P2_2f` + `...ReconcilesWithAirspeed_P2_2f` — the kinetic term against a hand-computed height trade |
  | 7 dmp → renderer | ⏳ | needs a run to look at — **P3-2** |
  | 8 INAV → xiao | ⏳ | bench, Phase 6 |
  | 10 xiao → flight log | ⏳ | blackbox clock-anchor fit, Phase 6 |
  | 11 renderer `'a'` | ⏳ | **P3-2**, the acceptance test |
  | `Es` sim vs flight | ⏳ | **P3-3** (subjective, de-scoped) |

  ⚠️ **The remaining five all need a run, a board or a flight** — none is closeable at a desk, which is why
  they are listed rather than quietly folded into "done".
- [ ] P3-2 ⭐ Renderer **`'a'` mode** acceptance test — the one place actual flight is shown in world
  coords, so the only place a datum error is *visible* rather than inferred. (was TD06)
- [X] P3-4 ✅ **DONE 2026-08-18 — smoke run TWICE, and the first one found a real defect.**
  **t2** (`logs/autoc-041-t2-smoke-m1.log`, arena +60/−10): ⛔ **16 of 16 scenarios died on the hard deck**
  (terminal AGL 25.01–25.51 against a 25 m deck), mean survival **4.9 s** of a 272-step path,
  `rabbitComplete=0`, elite frozen at −161.468690 for 11 generations. The ceiling was never approached
  (48 m of 95). Diagnosis: the band had been sized to the RABBIT (flat, never descends) when it is the CHASE
  that is contained. → arena re-cut to **+50 / −30** (operator).
  **t3** (`logs/autoc-041-t3-smoke-m1.log`, arena +50/−30): `rabbitComplete=` **3/16**, egress mix
  **6 floor / 7 radius / 0 ceiling**, elite moving −166.10 → −171.52 → −178.25 over 20 gens.
  ✅ **All the P3-4 column checks pass** (gen 5, 4734 ticks): `Es_m` 13.6→66.1 and **all ≥ 0**;
  `Ps_mps` −43.4→+23.3; `bClR_ms` **signed both ways** (1750 neg / 2984 pos); `SCORE_GRAD` non-zero on
  **4734/4734** ticks. `#GenCrash boot=0 sim=0` throughout.
  ⚠️ `rbHhd`/`rbZ` are **CONSTANT — correctly so**: `longSequential` is a flat path, which is the smoke's
  known blind spot (it exercises none of the vertical question). 040's trap 3 says a diagnostic that does not
  vary is telling you something; here it is telling you which config you ran.
  ⛔ **A diagnostic that lied, and the fix.** t2's per-gen line read `egFloor=0 egCeil=0 egRadius=0` while
  100% of deaths were the deck. The attribution re-ran `checkArenaBounds` on the terminal state, which sits
  **0.01–0.51 m INSIDE** the bound that tripped (the check and the state push happen at different points in
  the tick). Now attributed by **NEAREST bound**. A diagnostic reporting "none of the above" is worse than
  none — it looks like an answer.
  ⚠️ **Watch `pctInStreak`, not `best`**: at t3 gen 20 the elite improves while `pctInStreak` falls
  2.1 → 1.6 and `avgMaxStreak` 4.6 → 3.9 — the length-confounding of raw score
  ([project_038_wrap](../../.claude/projects/-home-gmcnutt-autoc/memory/project_038_wrap.md)). 20 generations
  is far too early to read either way (035-t6 took off at gens 125–200).
  ~~Original:~~ ⭐ **SMOKE before the production bake** (added 2026-08-18). `scripts/train.sh autoc-basic-m1.ini
  logs/autoc-041-t2-smoke-m1.log` — pop 3000 / 1 path / 16 winds, killed once it is clearly climbing.
  **Checks**: `#GenCrash` shows `boot=0 sim=0` (workers survive the new input vector and the new arena);
  the new columns **vary** in the dmp — `Es` plausible, `BOUNDARY_CLOSURE_RATE` signed both ways,
  `SCORE_GRAD_*` non-zero off the score peak; and it **climbs at all** on `pctInStreak`/`avgMaxStreak`.
  ⚠️ **A diagnostic that does not vary is telling you something** (040's trap 3).
  **Why this exists**: the 041 t1 smoke found no fault, and that was the point — it proved 26 workers came
  up clean on 42 inputs across 294 scenarios *before* committing ~15 h. This bake changes the input vector,
  the objective **and** the arena at once, so the cheap check matters more, not less.
  ⛔ **Gates P4-1.** No production bake until the smoke is clean.

- [ ] P3-3 **`Es` sanity check — SUBJECTIVE, not a numeric gate (de-scoped 2026-08-18).** Operator: *"Prob
  defer. The variations work signals that comparison is perhaps hard to quantify across the scenarios."* With
  entry / wind / rabbit-speed / craft variations all active, "the same state in sim and flight" is not well
  defined, so a paired numeric tolerance has no clean denominator.
  **Do**: confirm `Es` is plausible and self-consistent — non-negative, kinetic term reconciling with measured
  airspeed, no sign or unit error, no gross constant offset between a sim run and the flight.
  **Do not**: invent a tolerance to gate on. ⚠️ The bar is judged **subjectively at first**; the real target is
  *good fitness with less aggressiveness*, with energy as the indicator. Refine after the strategy shows
  signal. (was TD08)

- [X] P2-9 ✅ **DONE 2026-08-22.** `nn2cpp` now emits a 9-constant scale signature beside the layout guard:
  `kCruiseSpeed_mps`, `kDistToBoundaryScale_m`, `kTargetDistScale_m`, `kClosingRateScale_mps`,
  `kGyroScale_radps`, `kAccelScale_g`, `kEnergyScale_m`, `kScoreGradScale`, `kTimeSinceSeenScale_s` — each
  `static_assert`ed against the firmware tree's live value.
  **Verified both directions**: the emitted asserts compile against the live header, and perturbing one
  value fires it with the intended message. ⚠️ A first cut emitted `13f` (via `std::defaultfloat`), which
  is not a valid C++ literal and would have been a hard compile error in every generated file — caught by
  the positive test, fixed to `std::fixed`.
  ⚠️ **WHAT IT DOES NOT CATCH, stated so nobody over-trusts it**: it pins the *codegen tree's* scales
  against the *firmware tree's* scales. It does NOT pin the scales the genome was TRAINED with — the NN01
  file carries weights and topology, not the config that shaped its inputs. Normally nn2cpp is built from
  the same tree as the trainer so the check is meaningful, but a genome carried across a scale change and
  regenerated with a matching-era nn2cpp still passes. Closing that needs the scales inside the genome
  file: a format change, and a separate task if it is ever wanted.
  ⛔ **The article flashed for the 2026-08-23 flight PREDATES this** (`firmware_id=fb3866080c0df02d`, gen
  633). Do not regenerate before that flight unless you intend to re-flash — the signature changes the
  generated text and therefore the `firmware_id`. Any field update or later bake picks it up automatically.
  *(original)* ⛔ **nn2cpp must bake a SCALE SIGNATURE, not just an input count.** Found 2026-08-20 while
  applying P2-8. `nn2cpp` emits `static_assert(kGeneratedNNInputCount == NN_INPUT_COUNT)`, which catches a
  LAYOUT change — but P2-8 changed input SCALES with the layout untouched (still float[45]). A genome baked
  before P2-8 therefore has the right count and the wrong units: it compiles clean, passes the assert, and
  flies wrong. That is precisely the silent-failure class the count assert was added to prevent, one level
  down.
  * Emit the four P2-8 constants plus `kAccelScale_g` / `kEnergyScale_m` / `kScoreGradScale` /
    `kDistToBoundaryScale_m` into the generated header, and `static_assert` each against the live value.
  * ⚠️ **Do this at flight-prep time, NOT during a bake** — it touches `tools/nn2cpp.cc`, and rebuilding
    overwrites `build/autoc`, which the workers re-exec. `nn2cpp` is not needed until Phase 6 anyway.
  * Deferred deliberately at 2026-08-20 so the t7 rebuild was not disturbed.

- [ ] P2-10 ⭐ **VARIATION SWEEP EVAL — find the knee, i.e. where the craft/sensor complex actually runs out.**
  Operator 2026-08-22: *"if we ran a genome through eval setting the ramp to a fraction of full across say
  every gen or every 5 gens of ramp with the same best-of we might see good fitness maintained to a point and
  then it climbs. That knee says something. We could run to 150% variation."*

  **Shape**: hold ONE genome fixed, sweep the variation scale, plot fitness vs scale. A flat region then a
  knee is the limit; where the knee sits, and whether it MOVES between genomes, is the finding.

  * ⭐ **Sweep several genomes, not one** — e.g. best-of at gen 100 / 300 / 500 / 800. One genome tells you
    where *it* breaks; the family tells you whether evolution is **moving the wall** or merely **walking
    toward a fixed one**. That distinction is the actual question behind "physical limit".
  * ⭐ **CRAFT VARIATION IS ALREADY IN THE SAMPLE** (operator 2026-08-22: *"we are already running 300 or
    more variations so we do get craft variation fixed against a ramp of env"*). Each of the 294 scenarios
    carries its own craft draw at FULL magnitude, and the seeds hold those draws identical across the whole
    sweep. So the design is **a fixed craft-variation DISTRIBUTION × a ramped environment** — not a single
    airframe. That is strictly better: the knee is then a property of the craft *population* the controller
    must handle, not of one lucky or unlucky draw.
  * ⭐ **SIM COST IS NEGLIGIBLE; STARTUP IS NOT.** 294 scenarios per genome-eval; 4 genomes × 16 scale
    points = ~18.8k scenario-sims ≈ **seconds** of actual sim. The cost is per-eval-process startup (S3
    genome fetch, worker spawn), which lands N times in a shell loop. ⚠️ This REVERSES the loop-vs-internal
    recommendation above: prefer the operator's *"eval where it runs a ramp so all in one run"* — one
    process, one worker pool, sweeping scale internally. Go wide on points (31, i.e. 0.05 steps) since the
    resolution is nearly free; it is the knee's SHARPNESS that carries the information.
  * ⛔ **Same master seed, same scenario seeds, across the whole sweep.** Only the variation MAGNITUDE may
    change. Re-drawing scenarios per point confounds magnitude with which-scenarios and destroys the curve.
  * Implementation: `gEvalVariationScaleOverride` already exists (`src/autoc.cc:238`) but is only ever set
    from the genome's stored `variation_scale` (`:1262`). Add an ini/CLI override accepting **0.0 … 1.5+**,
    default −1 = keep current behaviour. Then a shell loop over scale values, one full 294-scenario eval per
    point. ⚠️ Prefer the loop over an all-in-one internal ramp: each point stays independently reproducible,
    and a failed point does not poison the run. ⛔ SUPERSEDED — see the startup-cost note below; the
    internal sweep wins. Keep per-point results independently identified in the output so a bad point is
    still discardable.

  ⚠️ **THREE CONSTRAINTS THAT CHANGE WHAT THIS CAN MEASURE — read before interpreting any curve:**

  1. ⭐ **The ramp scales ENV variations ONLY — and for THIS experiment that is exactly right.**
     Operator clarification 2026-08-22: *"given a fixed craft config set and a genome, ramp the env around
     it to see where the craft/controller reduce performance."* `applyVariationScale`
     (`include/autoc/eval/scenario_meta_apply.h`) touches entry pose, wind direction, entry position and
     speed factor; **craft variations are deliberately NOT ramped** (037, operator 2026-06-10 — the ramp
     exists so a fresh population is not killed by unflyable difficulty, and *"that risk is environmental
     … not the airframe"*), and camera draws sit at full magnitude from gen 0.
     ⛔ **Do NOT "fix" this.** Holding the airframe constant is what makes the curve readable: craft and
     genome are the fixed system under test, env is the independent variable, and degradation is therefore
     attributable to the environment alone. Ramping the craft too would confound the two.
  2. ⭐ **A craft/sensor-tolerance sweep is a SEPARATE, later experiment** — multiplier on the craft/camera
     sigmas, holding env fixed instead. Precedent: `eval_suite.sh` tier3-stress already runs *"12 paths ×
     12 winds at 120% sigmas"*. Same shape, orthogonal axis: env-scale → "how bad can the weather get",
     sigma-scale → "how wrong can the airframe/camera be". ⚠️ Keep them as two runs; one sweep varying both
     answers neither.
  3. ⚠️ **Position offsets CLAMP** at `ENTRY_SAFE_RADIUS` / `ENTRY_SAFE_ALT_MIN/MAX`. Past roughly scale 1.0
     the entry-position component stops growing, so a knee near 1.0 may be **the clamp**, not the aircraft.
     Angular/wind/speed terms keep scaling linearly. ⛔ Report the clamp fraction alongside the curve or the
     knee will be misread.

  **Output**: fitness (and `pctInStreak`, mean target distance) vs variation scale, one line per genome.
  Then the standard evolution PNG pass for context.

  ⛔ **BLOCKED until t7 finishes** — the override is a code change, and rebuilding overwrites `build/autoc`
  which live workers re-exec. Queued 2026-08-22, deliberately not started.

## Phase 5 — HARDWARE LONG-LEAD ✅ **CLOSED 2026-08-22 — the flight article is ready to fly**

> **P5-1 … P5-5 are DONE.** The accel is on the MSP wire, its convention is measured rather than argued
> (three independent parsers, and INAV's own blackbox agreeing to **0.34 milli-g**), all four formerly-zero
> NN channels are computed and bench-verified on flight hardware, and the flight article passed the
> three-attitude check with the 170-vs-180 alignment concern cleared. **P4-3 (the flight) is unblocked.**
> P5-6 (scale signature) is open and knowingly not blocking — see its note.
> ⛔ Read the **TOOLCHAIN + FORMAT STATE** block below before touching flight tooling: the log format, the
> `nn2cpp` CLI and the renderer all changed underneath this phase on 2026-08-22.
> ⚠️ One thing deliberately left unresolved: the **control oscillation** seen on the bench (P5-5). It is
> open-loop/off-distribution and probably not representative — but the sim-vs-bench comparison that would
> prove that was declined in favour of flying, so it is the FIRST thing to run if the flight looks jittery.

*(original header)* ⭐ **STARTS NOW, IN PARALLEL WITH THE BAKE**

➕ **Picking this up in a fresh context? Start at [`phase5-handoff.md`](phase5-handoff.md)** — verified
environment state, the three previously-resolved traps, the ⛔ do-not-rebuild-autoc boundary while t7 bakes,
and the toolchains.md INAV gap to close first. This section still GOVERNS; the handoff is orientation.

➕ **Added 2026-08-18 (operator)**: *"we have an inav extension backlog item to get additional sensor data
through the single msp call — that can start soon and be proven convention by bench testing — this is
required ahead of all given the complexity of setting up the programming — and then the xiao in prep for
flight."*

⛔ **Why this is its own phase rather than a step of P4-3.** It was folded into P4-3 (the flight task), which
made it look like it happens *after* the bake. It does not: it is the **longest-lead item in the feature**,
it needs a bench and two INAV targets, and it gates the flight completely. It also touches **neither autoc
nor the running bake** — different repo, different hardware — so it can proceed in parallel today.

➕ **BENCH EVIDENCE OF RECORD (2026-08-22 stationary bench, gen 633 firmware).**
* **IN-REPO / portable**: [`artifacts/bench-20260822/autoc-041-bench2-gen633-xiao-v4.bin`](artifacts/bench-20260822/autoc-041-bench2-gen633-xiao-v4.bin)
  — the run-2 xiao log, 315 ticks, 0 drops, all four 041 channels live. This is the first real v4 file and
  doubles as the format's regression fixture. Decode:
  `python3 src/analytics/flightlog_decode.py <bin> -o <csv>`.
* **MACHINE-LOCAL** (`eval-results/` is gitignored, so this does NOT travel): `eval-results/bench-20260822/`
  also holds both INAV blackboxes and the run-1 pair. The INAV side needs `blackbox_decode --index 2` —
  that file contains two logs, and index 1 is the earlier run. The derived result is recorded rather than
  the raw file: INAV `accSmooth` vs the xiao's logged `ACCEL_*` agree to **0.34 milli-g**, `|a|` 0.9976
  both sides (full table in `docs/COORDINATE_CONVENTIONS.md`).

➕ **TOOLCHAIN + FORMAT STATE after the 2026-08-22 bench session — read before touching flight tooling.**
Several things changed underneath Phase 5 that are not tasks but WILL bite a fresh context:

* **Flight log is v4** (`xiao/include/flight_log_format.h`). The NN input block was a hand-maintained 37
  against a 45-slot `NNInputs`, with nothing asserting it. That silently truncated 8 channels AND
  mis-assigned every quantization scale from slot 33 on (the old table put `dist_to_boundary` /
  `inward_body` where 041 has `ACCEL_*`). Now `kNumInputs = NN_INPUT_COUNT` with a `static_assert`, the
  table indexes by ENUM NAME, and the telemetry bases are derived. TickRecord 114 → **130 B**, FileHeader
  320 → **352 B**.
* **Post-P2-8 scales in the log**: `dist`/`gyro`/`airspeed`/`closing_rate` are O(1) NN units, not physical.
  They were still quantized at physical scales — `dist` at 1/32 NN unit = **0.81 m** granularity. Now
  `kScaleNN8` (±8 NN units): dist resolution 0.0063 m. ⚠️ **No version bump was needed** — the scale table
  travels inside the CRC-guarded FileHeader and decoders use THAT copy, so older v4 files still decode with
  their own table. That property is why it was safe.
* **FOUR decoders had to move together**, and three of them were independently wrong in the same way
  (literal field offsets sized for 37 inputs, which read outputs as inputs and telemetry as outputs):
  `src/analytics/flightlog_decode.py`, `tools/renderer.cc`, `xiao/web/flight_logger.html`. All now derive
  offsets from the widths. Verified agreeing on the real bench log: 130/352 B, telem scale base 48.
* ⚠️ **`nn2cpp` CLI CHANGED**: `-i` is now the **config file** (`--config` alias, default `autoc.ini`) and
  the genome moved to **`-w`/`--weights`**. Arena and cone come from the ini and NOWHERE else — the CLI
  overrides were removed deliberately, so the printed provenance line is the whole answer. An old
  `-i <weights>` is caught by an NN01 magic check with a migration message.
* ⚠️ **The renderer read POST-P2-8 NN units as physical** — `DIST_NOW` × direction cosines with no
  `kTargetDistScale_m`, so every chase vector was 26× too short and collapsed onto the craft. Fixed; the
  reconstruction now matches the log's own rabbit telemetry to 0.19 m mean. **Rebuild the renderer** — a
  stale binary loud-fails on v4 rather than mis-parsing (`cmake --build build --target renderer` is safe
  during a bake: separate executable target, static libs, `build/autoc` untouched — verified by dry run).
* ⚠️ **The USB console drops bytes.** Confirmed across two runs and a very wide terminal: the same
  heartbeats lose exactly one space mid-line. `printf` cannot do that and the 512 B buffer has room, so it
  is the CDC path; the mechanism was NOT identified. **The binary flight log is unaffected** (separate
  path, fixed-size records, CRC-guarded header) — trust the log, not the console.

- [X] P5-1 [OP] ✅ **DONE 2026-08-22** — 13 lines appended to `case MSP2_INAV_LOCAL_STATE` in `~/inav/src/main/fc/fc_msp.c` (after the gyro block), sourcing `acc.accADCf` as milli-g `int16`, INAV-native FLU unflipped. Payload **58 → 64 bytes** (note: the `63cffaf4f` commit message's "38 → 44" was already stale — the real pre-accel payload was 58). Built + flashed to the bench target `MAMBAF722_2022A`. ⚠️ Flight target `MATEKF722MINI` NOT yet built/flashed. **INAV: carry accel in the SAME single MSP round trip.** Full spec is in T072 (kept below for
  provenance); it is complete and does not need re-deriving. The load-bearing parts:
  * Extend `MSP2_AUTOC_STATE` in `~/inav/src/main/fc/fc_msp.c` (the `MSP2_INAV_LOCAL_STATE` case). Copy the
    shape of fork commit **`63cffaf4f`** ("extend MSP2_AUTOC_STATE with filtered gyro rates") — append at
    payload end, fixed integer scale stated at the write site.
  * ⚠️ **Source `acc.accADCf`** — the TRANSFORMED field. `acceleration.c:563-568` applies
    `applySensorAlignment` then `applyBoardAlignment` then divides by `acc.dev.acc_1G`, so it is
    board-alignment corrected and already in **g**. ⛔ **NOT** the file-static `accADC` nor `acc.dev.ADCRaw`
    — those are pre-alignment and would bake each board's own misalignment in differently (bench roll = −16
    vs flight).
  * Wire encoding **milli-g `int16`** = `lrintf(acc.accADCf[axis] * 1000.0f)` — ±32 g range against ±11 g
    observed.
  * ⚠️ **The wire carries INAV's native FLU, UNFLIPPED**, exactly as the quat and gyro already do. The
    FLU→FRD y/z flip belongs at the msplink boundary, once, beside the other two (settled 2026-08-11).
  * ⚠️ Both targets, **bench first** (`MAMBAF722_2022A`), then flight (`MATEKF722MINI`). **Disconnect the
    GPS before flashing.**
- [X] P5-2 [OP] ✅ **PASSED 2026-08-22, bench target — all three attitudes, measured on the wire.** Read host-side off the FC's USB VCP by [`msp_state_probe.py`](msp_state_probe.py) (no xiao in the loop, so a convention error could not hide behind a firmware bug). FRD after the flip: level `[−0.011, −0.032, −0.997]`, nose up `[+0.998, −0.023, −0.091]`, right wing down `[+0.001, −1.000, +0.003]` — all matching, `|a|` within 0.3% of 1 g. Full table + bias/scale residuals recorded in `docs/COORDINATE_CONVENTIONS.md` → "MEASURED ON THE WIRE 2026-08-22". ⚠️ **Bench board ONLY** — the flight FC has different `align_board_*` and must be re-measured after flashing. ⭐ **BENCH-PROVE THE CONVENTION before trusting it in the air.** This is the point of doing
  it early: the three static attitudes have a measured, recorded answer, so the sign convention is checkable
  on a table rather than inferred from a flight.
  Expected in **FRD, after the msplink flip**: level `[0, 0, −1]`, nose up `[+1, 0, 0]`, right wing down
  `[0, −1, 0]`.
  ⚠️ The bench table in `docs/COORDINATE_CONVENTIONS.md` is in INAV **FLU counts** (`acc_1G ≈ 2048`) —
  **divide by 2048 AND flip y/z** before comparing. Getting this backwards is invisible in sim and wrong in
  the air; it was already resolved once against `~/inav @ 63cffaf4` and the earlier "flip it" instruction was
  **withdrawn**. Do not re-derive it from scratch — confirm against the recorded table.
- [X] P5-3 ✅ **COMPLETE 2026-08-22 — all four channels live and bench-verified.** No NN input is a
  constant zero on the xiao any more.
  **Part 2 (the three non-hardware channels), added 2026-08-22:** `writeCraftObservations()` supplies Es +
  boundary closure; `SCORE_GRAD_*` is computed from `FitnessComputer::scoreGradientWorld` × the streak
  multiplier from a span-scoped `EnvelopeState`, world→body at the same boundary as `inwardBodyDirection`.
  ⚠️ **All three use the SHARED code the sim producer uses** — `fitness_computer.cc` is now compiled into
  the firmware (`platformio.ini` `build_src_filter`) rather than copied, so the gradient's closed form has
  ONE definition. `HALF_PI` → `kHalfPi` there: the Arduino core `#define`s the former (rename only).
  ⚠️ **The gradient uses SEGMENT geometry** (`flight_path[i].start` + next segment), matching
  `inputdev_autoc.cpp`'s score path — NOT the interpolated `targetPos` the direction-cosine inputs use.
  Under one 0.4 m segment apart, but it must describe the rabbit the OBJECTIVE scored.
  **Cone constants** are baked by `nn2cpp` beside the arena template, via `autoc/eval/cone_constants.h`
  (no default member initializers — it must not become a third source of truth).
  **BENCH-VERIFIED 2026-08-22 (bench run 2, 315 ticks, gen 633 firmware):**
  | channel | recorded | check |
  |---|---|---|
  | `SPECIFIC_ENERGY` | 30.02 → 30.52 m | matches the engage header's `floor_z = 30.5` |
  | `BOUNDARY_CLOSURE_RATE` | −0.075 … +0.287 m/s | correct for a craft drifting cm/s on a table |
  | `SCORE_GRAD_*` | non-zero on **315/315** ticks | mean 0.108 inside 10 m vs 0.0007 beyond 30 m — **146×** |
  ⭐ The gradient trace is the real evidence: small AT the rabbit (0.042 at 0.62 m — score is maximal, so
  there is no uphill), peaking at **0.152 at 4.8 m** where the tail-chase band and the ramping streak
  multiplier compound, then vanishing as the rabbit runs the racetrack away and reviving as it loops back.
  Body direction is +x dominant — "uphill is forward" for a craft being left behind. ⚠️ **The channel
  carries decision signal only near the cone** (<0.01 NN beyond ~30 m). Geometrically honest — the
  objective IS flat out there — but do not read a flight as "ignoring the input" without checking range.
  **Cost**: eval 0.8/1.3/10.6 → 1.1/1.8/13.1 ms; total 12.8/21.5/55.1 ms in a 50 ms tick, 0 overruns/drops.
  **Part 1 (`ACCEL_*`), 2026-08-22:** ✅ With the 45-input firmware flashed, the console heartbeat (which now prints all five 041 channels) read level `z = −1`, nose up `x = +1`, right wing down `y = −1` — agreeing with the host probe through a completely separate code path, so the flip in msplink is confirmed, not just the wire. Done: `msp_autoc_state_t` grew `int16_t accel[3]` (`xiao/include/MSP.h`); the FLU→FRD flip `(+x, −y, −z) × 0.001` landed at the ONE boundary beside the quat and gyro (`msplink.cpp`, `convertMSPStateToAircraftState`) feeding `setSpecificForceG` unscaled in g; and `performMspRequest` now **rejects a short reply** — `MSP::recv` zero-fills an undersized payload and still returns true, so a xiao flashed against un-upgraded INAV would have read `accel = [0,0,0]` silently. Guard is `recvSize >= size`, not `==`, so a LATER INAV appending fields stays compatible (surplus is discarded, offsets unchanged) — that asymmetry is what makes flashing INAV ahead of the xiao safe. `msplink.cpp.o` compiles; the build still stops at the by-design `static_assert` in `src/generated/nn_program_generated.cpp` (generated file is the old **37-input** program), which is P5-4.
  ⛔ **Still zero on the xiao**: `SPECIFIC_ENERGY` (`setSpecificEnergy`), `BOUNDARY_CLOSURE_RATE` (`setBoundaryClosureRate`), `SCORE_GRAD_*` (`setScoreGradBody`). The shared gather in `src/nn/evaluator.cc` only COPIES these from `AircraftState`, so they stay zero until msplink computes and sets them — no hardware needed for any of the three. The cone-constants catch below still applies.
  *(original text follows)* **xiao: produce the four channels it currently zeroes.** ⛔ The MSP extension only supplies ONE of
  them. The split matters for scoping:
  | channel | source on the xiao | needs P5-1? |
  |---|---|---|
  | `ACCEL_X/Y/Z` | MSP, via P5-1 | ✅ yes |
  | `SPECIFIC_ENERGY` | position + airspeed + the engage-resolved arena floor — **all already on board** | ❌ no |
  | `BOUNDARY_CLOSURE_RATE` | position + velocity — **already on board** | ❌ no |
  | `SCORE_GRAD_*` | the virtual target's geometry + the cone constants — **the xiao knows the target** | ❌ no |
  So three of the four can be written **now**, against `autoc/eval/craft_observations.h` and
  `FitnessComputer::scoreGradientWorld`, and only the accel waits on hardware. ⚠️ The cone constants
  (`FitDistScaleBehind/Ahead`, `FitConeAngleDeg`, `FitStreakThreshold/RampSec/MultiplierMax`) are **not on
  the xiao today** — they have to be baked in alongside the arena template, or the gradient cannot be
  computed there.
  ⛔ **A flight before P5-3 lands would fly a policy seeing ZEROS in four channels the sim trained with** —
  and `ACCEL_*` has been a silent zero on the xiao since 041 US4, so this is a pre-existing gap, not a new one.
- [X] P5-4 ✅ **DONE 2026-08-22 — regenerated, flashed, and running on the flight article.** `nn2cpp` regenerated from gen 633
  (`…2026-08-20T22:22:41.333Z/gen9367.dmp.zst`, fitness −77,698, `45->32->16r->3`, 2307 weights) and the
  firmware BUILDS (flash 45.5%, RAM 53.5%). Identity for the boot-banner check:
  `weight_id=4123bd342058553a`, `firmware_id=fb3866080c0df02d`, `arenaTemplate=[R=70 F=25 C=105]`.
  ⚠️ **gen 633 is the LATEST dmp, not the best** — `nnextractor` without `-k` takes the most recent. Choose
  the flight genome deliberately (t7's final best, or a pinned gen), don't inherit whenever the extract ran.
  ⚠️ Rebuild + reflash needed to pick up the post-run heartbeat change (`--` for uncomputed channels).
  ⛔ **Flight article (`MATEKF722MINI`) is NOT built or flashed** — see the gate below.
  *(original)* Regenerate + flash: `tools/nn2cpp` from the P4-2 pinned genome, then the xiao build. The
  fail-loud `static_assert` in the generated file is what currently blocks the firmware, by design.

- [X] P5-5 ✅ **COMPLETE 2026-08-22 — FLIGHT ARTICLE IS READY TO FLY** (operator call after the prop-off
  bench run below). All three steps done.
  **Prop-off bench validation on the flight article** (run 3,
  [`artifacts/bench-20260822/autoc-041-bench3-flightarticle-gen633-xiao-v4.bin`](artifacts/bench-20260822/autoc-041-bench3-flightarticle-gen633-xiao-v4.bin),
  341 ticks, path 0 run to natural completion, 0 drops, 0 overruns):
  * ⭐ **Quaternion ↔ accel agree to 0.8–3.5°.** At rest the accel must read the body-frame UP direction,
    which the AHRS quaternion independently predicts. Two INAV outputs that travel different paths and meet
    only at the xiao — this is the strongest end-to-end check available without flying. The airframe sat
    nose-up 4.0° / left-wing-down 9.8°, and BOTH channels report that same attitude.
  * **Command path verified faithful**: 95.6% of sent RC values appear EXACTLY in INAV's stream (zero-order
    hold test, all three channels). ⚠️ **Index-convention trap**: the xiao sends MSP order, the blackbox
    logs FLIGHT-DYNAMICS order (ROLL, PITCH, **YAW**, **THROTTLE**) — so throttle correctly appears at
    `rcData[3]` and the constant 1500 at `rcData[2]` is uncommanded yaw, NOT a lost channel. `servo[0]/[1]`
    sweep the full 1000–2000 µs, so the servos have full authority.
  * **Es / bClR / sg all live on flight hardware.** Es 30.0 → 31.2 m tracking a 1.1 m altitude drift;
    `sg = [0.09, 0.00, −0.01]` at engage, decaying as the rabbit departs.
  * ⚠️ **`bClR` is NOISY on a bench (±0.6 m/s) and this is geometric, not a fault** — the craft sits
    0.5–1.3 m from the arena axis and `boundaryClosureRate` divides by that horizontal radius, so ordinary
    velocity noise is amplified near the centre. In flight the radius is tens of metres. Do not chase it.
  * ⚠️ **CONTROL OSCILLATION observed and deliberately NOT resolved.** Roll reverses direction on 77% of
    ticks (mean 175 µs/tick), pitch 69%, throttle at a rail 74% (the known throttle bang-bang).
    Autocorrelation shows a period-2 (10 Hz) component riding a slow trend — lag-1 0.451 vs lag-2 0.937 on
    roll, weaker on pitch. ⛔ **A STATIC BENCH IS AN OPEN LOOP AND FAR OFF-DISTRIBUTION**: airspeed reads
    0.35 m/s against a 13 m/s training cruise, commands produce no state change, and the rabbit recedes to
    48 m past where the gradient carries signal. A recurrent controller whose outputs do not move the world
    will wind up and hunt, so this OVERSTATES what the air will look like. It is good evidence the plumbing
    and servo authority are sound; it is weak evidence about flight control quality.
    ➕ **The diagnostic that would settle it** (offered, declined 2026-08-22 in favour of flying): compare
    the same genome's per-tick outputs in SIM via `dmp-dump` against this bench trace, using the per-axis
    `dCtrl`/aggressiveness comparator from the 038 control-quality gate. If sim shows the same 10 Hz roll
    component it is the policy; if sim is smooth it is the open loop. **Do this before blaming the policy
    for anything seen in flight.**
  * ➕ **Latency datum for `project_sim_latency`**: command → `rcData` median **40 ms**, but up to 17 ms of
    that is blackbox sampling phase, so the true figure is **~23–40 ms**. The sim model assumes ~30 ms. Not
    a blocker; worth a purpose-built measurement rather than this by-product.
  *(original)* **FLIGHT ARTICLE GATE — steps 1 and 2 DONE 2026-08-22; step 3 (xiao flash) pending.**
  ✅ INAV `MATEKF722MINI` built + flashed (FC enumerates as `INAV_…_206132853456`, distinct from the bench
  board's `…203739535333`), payload confirmed 64 B. ✅ Three attitudes measured with `msp_state_probe.py
  --watch`: level `[+0.009, +0.038, −0.997]`, nose up `[+1.003, +0.048, −0.014]`, right wing down
  `[+0.029, −1.003, −0.043]` — matching the bench board pose-for-pose, which is exactly the transparency the
  in-INAV alignment is supposed to give. ⭐ **The 170-vs-180 roll concern is CLEARED**: largest off-axis
  component across all three poses is **0.048 g** (~2.8°, hand-holding) against the ~0.17 g a 10° residual
  would produce. Full table in `docs/COORDINATE_CONVENTIONS.md` → "FLIGHT ARTICLE VERIFIED".
  ⚠️ Port contention (INAV Configurator on the same VCP) shows up as pyserial "readiness to read but
  returned no data"; the probe now rides through it rather than aborting a held pose.
  *(original)* ⛔ **FLIGHT ARTICLE GATE — the only hardware gap left before flying.**
  1. Build + flash INAV `MATEKF722MINI` (clean build dir per target — `rm -rf build && mkdir build && cmake
     .. && make MATEKF722MINI`; **disconnect the GPS**).
  2. Re-run the three static attitudes on the FLIGHT board with
     [`msp_state_probe.py`](msp_state_probe.py) `-p <port>`. ⚠️ **The bench pass does NOT transfer.** Board
     alignment is applied inside INAV before MSP, so a correct config makes the two boards read identically
     — that is the test. The flight FC's `align_board_roll` is the 170-vs-180 case (auto-memory
     `project_board_alignment`): a 10° residual puts ~0.17 g on the wrong axis and, unlike a fixed offset,
     it ROTATES with attitude, so it does not average out in flight.
  3. Flash the xiao with the chosen flight genome; confirm the boot banner's `weight_id` is the intended one.

- [X] P5-6 ✅ **CLOSED 2026-08-22 — implemented, see P2-9.** The hazard it named is now guarded for every
  FUTURE regeneration; the article flying on 2026-08-23 predates the signature, which is fine for the
  reason below.
  *(original)* ⚠️ **OPEN, and knowingly not blocking the first flight**
  (operator 2026-08-22): the flown genome and firmware were generated from one tree in one session and the
  boot banner's `weight_id`/`firmware_id` were checked against the intended build, so the hazard does not
  apply to THIS article. It applies to the next re-flash of a snapshot genome. (carried from the P2 list — restated here because it is a
  flight-safety item, not a tooling nicety). `nn2cpp` bakes `weight_id` / `firmware_id` / arena / cone, but
  NOT the input scale constants (`kAccelScale_g`, `kGyroScale_radps`, `kTargetDistScale_m`, …). Those live
  in `nn_inputs.h` and are outside the hashed code text, so **a genome + different scales produces an
  IDENTICAL `firmware_id`** — a pre-P2-8 genome loads clean and flies wrong with nothing to say so. Not
  blocking a same-session build (genome and firmware come from one tree), but it is exactly the hazard a
  SNAPSHOT genome that outlives today creates.

⛔ **P4-3 depends on all of Phase 5.** It is the flight; this is what makes the flight mean anything.

## Phase 4 — BAKE AND FLY

- [ ] P4-1 [OP] Pre-run gate (Constitution IX — launch via `scripts/train.sh`, never a background task),
  then the production M1 bake.
  ⚠️ **Judge against AC-1's THREE parts, not fitness alone** (corrected 2026-08-18 — the previous wording
  said only *"judge on `pctInStreak` / `avgMaxStreak`"*, which is the fitness half and omits the actual
  target):
  1. **fitness holds** — tracking occupancy within noise of the pinned prior M1's **30.9%**
     (`stpPt ≥ FitStreakThreshold`, the one definition Study A and the per-gen logs share). Do not
     substitute a differently-defined tracking metric.
  2. ⭐ **aggressiveness DOWN** — per-axis `dCtrl` and `⟨|u|⟩` versus the prior M1. **This is the target.**
     Both come from the existing report set (`per_axis_aggressiveness`, `run-summary` `dctrl_*`/`mag_*`) and
     from [study-a/](study-a/) for the prior-M1 side — no new tooling.
  3. **energy improving** — `Ps` better while fitness holds, which is what distinguishes a genuinely calmer
     controller from a **muted** one (035's failure).
  ⚠️ Judged **subjectively at first** per AC-1 — no numeric thresholds until the strategy shows signal.
  ⚠️ Compare on the **charts against the pinned prior M1**, not a purpose-built controlled pair: large
  changes are visible in the ordinary report set (methodology note, Phase 0).
- [ ] P4-2 [OP] ⭐ **RETAIN THE t7 BUILD IN S3 — this run is the baseline for M2 and camera work, not just
  for 043.** Operator 2026-08-22: *"that m1 may likely be the baseline for m2 and camera work — we'll
  definitely want to retain this build in s3 once the run is done."* Scope grew accordingly: it was
  "pin + archive weights"; it is now **preserve the whole reproducible article**.
  * **Run**: `autoc-9223370249590214474-2026-08-20T22:22:41.333Z` in `autoc-m1`, master seed `1787264561`.
    Pin `retain=keep` (milestone-preserve prefix, per `reference_autoc_storage_keeper_runs`) so no lifecycle
    rule can expire it.
  * ⛔ **Wait for gen 800.** The pinned generation is the FINAL one — see [`baseline.md`](baseline.md),
    which deliberately records a mid-run snapshot as NOT the pin.
  * Archive alongside the weights, because the weights alone are not reproducible:
    - `autoc.ini` **as the run used it** (the ramp-on, P2-8-era config),
    - the **commit hash** of the binary that produced it (clean `rebuild-perf.sh`, 48/48),
    - the crrcsim submodule pointer,
    - the four P2-8 scale constants (`kCruiseSpeed_mps`, `kTargetDistScale_m`, `kClosingRateScale_mps`,
      `kGyroScale_radps`) — ⛔ **without these the genome loads cleanly and flies WRONG**; see P2-9.
  * ⭐ **Also extract the per-tick CSV now, while the reader matches.** The v3→v4 break already cost 041 a
    comparator (T011a), and a future schema move would do the same to this one. A `--csv-only` and a
    `--physics` dump into `artifacts/` costs minutes and removes the dependency on building an old worktree
    later. ⚠️ The prior archive's physics columns turned out to be 4 ticks/scenario — check coverage on
    the way in this time, do not assume it.
  * **Hard dependency of**: 043, and now the M2 / camera baseline too.
- [ ] P4-3 [OP] M1 flight → playback → energy validation. ⚠️ Remove the GPS before flashing; both targets,
  bench first.
- [ ] P4-4 Outcome doc: every hypothesis with its evidence, including the refuted ones.

---

# Tasks: 041 — M2 Depth (observation-side objectives)

**Input**: Design documents from `/specs/041-m2-depth/`

## 📖 READ THIS FIRST — order, and which document wins

A fresh context should read in this order. The ordering is not cosmetic: two documents disagree with each
other on purpose, and one of them is superseded.

| # | document | why, and how much |
|---|---|---|
| 1 | **this file** | the execution list. Each task carries its file paths and its own warnings — you can work from it directly |
| 2 | [plan.md](plan.md) | stack, constitution gates, phase sequencing |
| 3 | [spec.md](spec.md) **§ Clarifications** | ⭐ **GOVERNS.** On any conflict between documents, this wins. 11 decisions across two sessions live here |
| 4 | [contracts/](contracts/) + [data-model.md](data-model.md) | binding interface detail — read the relevant one per task, not all up front |
| 5 | [research.md](research.md) | the *why* per decision, R1–R14. Read when a task's rationale is unclear |
| 6 | [hypothesis.md](hypothesis.md) | ⚠️ **SUPERSEDED WHERE IT CONFLICTS.** The derivation of record and the only place the *retractions* are written down — genuinely worth reading for **why**, but **not for what**. It carries a supersession table at the top; trust that over its body. Optional |

**If you read only one thing beyond this file, read spec.md § Clarifications.** Several decisions reversed
earlier ones, and the reversals are the load-bearing part: no load axis in 041; the in-envelope *flag* (not the
duration accumulator) is the primary input; the predictor is a horizon-free current-bearing estimator with its
error fed back, not a fixed-horizon forecast.

**Three traps that cost real money if missed** — each is called out at its task, listed here because they are
the ones you cannot recover from:

1. **T011a before T044** — extract the pre-break comparator CSVs, or the prior-M1 baseline (SC-007a) and the
   blind-gap distribution (FR-024b) are gone permanently.
2. **T036** — moving the step score into the tick loop must not silently change the objective. *(Reframed
   2026-08-10: the gate is **determinism + "materially the same or better"**, not literal bit equality —
   see spec.md § Clarifications. Silence is the failure mode, not difference.)*
3. **T040(a)** — steady level flight must read ≈1 g on the normal accel channel, not ≈0. The only guard
   between specific force and kinematic acceleration.

**Tests**: REQUIRED, not optional — Constitution I, and FR-003 mandates the zero-answer pattern for every
paired-series fitness term. Test tasks are interleaved with implementation per Constitution I's TDD ordering.

**Organization**: grouped by user story. US1–US4 are P1, US5–US6 are P2.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable (different files, no dependency on incomplete work)
- **[Story]**: US1–US6 per spec.md
- **[OP]**: operator-driven — an assistant MUST NOT run this (clean rebuilds, bakes, S3 mutation, flight)

## ⏸️ PAUSED 2026-08-17 — t1 stopped at gen 608; the APPROACH is the blocker, not the bake

**Operator call**: *"feels we need to get some improvement here in approach before working m2."* The camera
toolchain proceeds in parallel — it only fixes constants for the M2 sensor and does not gate this.

### What t1 settled (none of it wasted)

| question | answer |
|---|---|
| does the 42-input vector run at production scale? | **yes** — 294 scenarios, 26 workers, 608 gens, zero worker deaths |
| is scoring deterministic? | **yes** — 487+ `NN_ELITE_SAME`, zero `ELITE_DIVERGED` |
| were the fitness regressions a bug? | **no** — 11 of 11 land exactly on 40-gen variation-ramp boundaries |
| is NN capacity the constraint? | **no** — `W_hh` effective rank 11.1–11.8 of 16, flat all run |
| is `IN_ENVELOPE` used? | by **contribution** (weight × std) it ranks **2nd of all measurable inputs**; by weight alone it looked weakest — the weight-only read was wrong |
| what load does the policy pull? | nz median **3.17 g**, peak **11.13 g**, 34% of ticks >4 g — at the airframe's standing record, with nothing in the objective restraining it |

t1 final: best −23651, `avgMaxStreak` 34.4, `pctInStreak` 16.1, 288/294 complete. It was still climbing.

⚠️ **The gen-608 elite is pinned and REMAINS ABLATABLE.** T049/T052 masking is implemented, so the H1a
matrix can still run against it. Stopping the run did **not** forfeit the verdict — it deferred it.

### The diagnosis that triggered the pause

Every energy/smoothness objective tried so far has *muted the whole regiment* rather than trimming waste,
because each penalises an **absolute** quantity. Full power is genuinely correct when far behind, in a
sustained spiral, and under pitch-induced drag — so uniform pressure can only quiet everything. What is
missing is energy **relative to what the situation demands**, i.e. a state-conditioned baseline. Operator:
*"if we knew an optimal path and then could figure out the energy to make that step, then this can be per
step reinforcement."*

See **[approach-proposal.md](approach-proposal.md)** for the proposed way forward (per-tick reward shaping,
a computable energy reference, and the input-vector trim).

### Consequences for the remaining 041 tasks

- **T063–T067, T071 are VOID as written** — they describe an 800-gen t1 and its non-regression read. A
  re-baked M1 under a revised objective is a different run; do not report t1 against them.
- **T068/T069/T070 (the ablation matrix) are STILL LIVE** and now cheap — the elite exists.
- **T045 remains blocked on T046** (parked), unchanged.
- The M2 chain (T095–T098) is unblocked only by a *pinned M1 source*, which t1 could still supply if we
  chose to; the pause is about wanting a better M1 first, not a broken one.

## ▶️ RESUME HERE (state as of 2026-08-11)

**Progress: 47/111 marked.** Phases 1–3 complete; Phase 4 (US2) through T041c. Build clean on both
surfaces (autoc + crrcsim), **43/43 ctest suites pass** (+1: `envelope_accel_inputs_tests`, the T040
semantics suite). ⚠️ All Phase 3/4 work is **checkpointed WIP, not landed** — it squashes into T045.
Task count is 111, not 110: **T074a** added 2026-08-11 (bench-observable NN inputs, operator ask).

### What landed 2026-08-11

- **The accel sign datum is SETTLED** (see the next section) — and the answer was "no flip", so nothing
  was flipped. `docs/COORDINATE_CONVENTIONS.md` now carries the derivation and the evidence.
- **T037–T040 complete.** The step score now computes **pre-eval in both modes**; `IN_ENVELOPE` /
  `ENVELOPE_SECS` / `ACCEL_*` reach both gathers; 8 new semantics tests.
- New shared header **`include/autoc/eval/envelope_state.h`** — `EnvelopeState` (accumulator mechanics,
  identical in both modes) + `perceivedInEnvelope` (the M2 direct-perception estimator). `EnvelopeState`
  joins `resetPerceptionState()`, so both tracker paths reset it without either being edited.
- `CrrcsimTrackerHelper::peekTargetGeometry()` — reads `samples[cursor_]` **without advancing**, which is
  what let the tracker score pre-eval. The clamp at exhaustion is not padding: it reproduces the old
  `lastTargetSample()` value exactly, which is what keeps the move a relocation.

### State after the 2026-08-11 evening pass

**Phase 4 is code complete except T046**, which is deliberately parked (below). Landed this pass:
T042 + T044a tests (`tests/recording_fidelity_tests.cc`), T043's real remainder (041 knobs recorded,
`renderer.cc:4651` un-half-flipped, tests), and T044 (version 2→3 with a **real** fail-loud check).
**44/44 ctest suites pass**; autoc, crrcsim, dmp-dump and renderer all build.

⚠️ **T044's headline correction**: the old comment claimed *"v=3+ dmps fail loudly via cereal's
class-version mechanism"*. **Cereal does no such thing** — it forwards the stored version to `serialize()`
and rejects nothing, which is exactly why mismatches died in the allocator as `vector::_M_default_append`
instead of naming the artifact. The check is now explicit in `EvalResults::serialize`, keyed to
`EvalResults::kSchemaVersion`, static_asserted against `CEREAL_CLASS_VERSION`, and covered by three
hermetic tests (older / newer / current) that replaced a `SUCCEED()` placeholder asserting nothing.

**T046 is PARKED** — operator 2026-08-11: *"that part of validate needs to wait."* It is the sole
remaining T045 precondition, so **T045 cannot land until T046a is done**. See T046 for the split and for
the silent 37-vs-42 layout hazard that must be made loud as part of it.

### ⚠️ Next, and the one thing to be careful about

**T042 → T043 → T044 → T046 → T045.** Ordering constraint 6 reduced to T046-only (T047 is inverted), so
the remaining gate order is: T046 regenerates `nn_program_generated.cpp` for 42 inputs, **then** T045
commits and builds xiao. T011a is already done, so T044's version bump is safe to take.

⚠️ **The T037 move is unverified by anything an assistant can run.** Be precise about which "T036" is meant,
because the two are not interchangeable:

| | what it is | catches the T037 move? |
|---|---|---|
| **T036 unit test** (`fitness_decomposition_tests.cc`) | in ctest, no rebuild, passing | ❌ **No.** Its fixtures fill `stepScore` via `fillStepScores()` — the post-hoc reference — so the worker is never invoked. The test says so itself at `:895-900`. It proves the objective READS the recorded series, deterministically. That is orthogonal to where the worker computes it. |
| **eval-vs-training bitwise gate** [OP] | clean `rebuild-perf.sh` + a run, fitness compared exactly | ✅ **Yes** — it is the only thing that compares a worker-produced dmp against the objective end to end. |

So the argument for the move being value-preserving (the NN eval writes only pitch/roll/throttle; position,
attitude and the target lookup are all fixed before it runs; `peekTargetGeometry`'s exhaustion clamp
reproduces the old `lastTargetSample()` exactly) is **an argument, not a measurement** — and "silence is the
failure mode" is exactly T036's point. **The operator's bitwise gate is what closes it, before T045.**
Re-running the unit suite is not a substitute and should not be mistaken for one.

### ✅ SETTLED 2026-08-11 — the accel sign needs NO flip; do not re-open it

The open datum is **resolved against `~/inav` @ `63cffaf4`**, and the answer is that
`include/autoc/eval/specific_force.h` and its five tests in `tests/dmp_dump_tests.cc` are **already
correct**. Full derivation and evidence: `docs/COORDINATE_CONVENTIONS.md` → "Accelerometer as an INTERFACE
quantity (041)" → *RESOLVED 2026-08-11*.

- **INAV's body frame is FLU** (x fwd, y **left**, z **up**) and `acc.accADCf` is plain **proper
  acceleration**, which at rest points UP. Under that reading **all three** bench rows fit — including
  nose-up. Nose-up was the only *discriminating* row because FLU→FRD flips y and z but **shares x**, so on
  level and RWD the frame flip and the sign flip cancel and both hypotheses agree.
- **The msplink converter is the same y/z flip already used for the quat and gyro**:
  `accel_FRD = (accADCf[0], −accADCf[1], −accADCf[2])` (T074).
- **In FRD, level flight reads `ACCEL_Z = −1`.** That matches spec.md § Clarifications (which GOVERNS —
  session 2026-08-10, "the sim's `ACCEL_Z` in steady level flight is **−1** in body axes") and matches the
  header as written.
- ⚠️ **The earlier "flip it" instruction is WITHDRAWN.** It was derived against the unresolved datum and
  asserted the opposite convention. Flipping the header would have put the NN's load axis backwards
  relative to flight — invisible in sim, wrong in the air.
- **All three attitudes are now safe to assert** at T073, in FRD: level `[0,0,−1]`, nose up `[+1,0,0]`,
  right wing down `[0,−1,0]`. The bench table is in INAV FLU **counts** (`acc_1G ≈ 2048`); divide *and*
  flip y/z before comparing.

### Then T037–T040, which are now fully specified

- **Ordering is SOLVED** (spec.md Clarifications, "the tracker target is a PRELOADED PATH"): the tracker
  target is `source_->samples[cursor_]` (`crrcsim_tracker_helper.cpp:208`) — a preloaded trajectory
  indexed by a cursor, structurally identical to pathgen's `path[pathIndex]`. So the step score is a
  LOOKUP in both modes, knowable before the NN acts ⇒ compute **pre-eval in both**. No tick k−1 fallback.
  Currently the worker scores AFTER the eval (`inputdev_autoc.cpp`, "SCORE THE TICK") — move it up.
- **T039 plumbing is DECIDED**: body-frame specific force in g stored on `AircraftState` **beside
  `gyroRates_`**, computed worker-side via `specific_force.h`; both gathers only COPY. Criterion was
  "closest to what INAV presents to real hardware NN inputs" — on hardware the value arrives finished and
  the gather does no transformation, so the sim must match that shape.
- **T040** is wiring, not derivation: the shared math and its five tests already exist from T010/T011.

### ✅ Filed 2026-08-16 — no longer unfiled

The **M2 flight architecture** (operator 2026-08-10) is now in `specs/BACKLOG.md` → *Post-041 direction*,
where it belongs as future firmware scope. It was extended there by the operator on 2026-08-16: M2 is
**two-fold** — phase 1 a virtual M1 target on the flight hardware with a synthetic camera (beacon
projection + CEP on the nRF52840), phase 2 chasing a real craft with real beacons — demonstrating
all-attitude flight control and a generalized controller. **M3 then forks**, deliberately undecided:
target-stops-broadcasting (vision-only) vs strategy (offense/defense).

Still not 041 — 041's flight test is M1 (T079/T080).

### Decisions taken 2026-08-10 that changed the plan

| | outcome |
|---|---|
| T062 | **Reframed, closed.** No retry budget, no lottery framing — "a good bake is good enough". Early failures are **diagnostics** (missed assumptions / non-determinism), not bad luck. T064 folds into T063 |
| T067 / R7 | Judge against the **pinned prior M1 alone**, loose bar, "materially worse" — one comparator cannot make a band |
| T087 | **DROPPED** — one tracker run removed |
| "calibration" | = re-establish the **comparator set**. Closes with no new work (T065 / FR-010 / T097) |
| Camera pitch | ~~**0.375 stays** (120°×75°). Revisit from measurement, not a second estimate~~ → **SUPERSEDED 2026-08-16 (T041d)**: the measurement arrived. **0.304** ⇒ 97.3°×60.8°. The reserved trigger fired; see spec.md § Clarifications |
| crrcsim builds | **Build freely; gates stay with the operator.** `crrcsim/CLAUDE.md` updated |
| T036 | Gate is **determinism + materially-same**, not literal bit equality |
| T047 | **INVERTED** — `data.dat` retired at 035 FR-P05; its parser must NOT be updated |

---

## ⛔ Hard ordering constraints (violating these costs a run)

1. **One contract-break commit** (FR-005). US1's grouped record and all of US2 land **together**, before any
   bake. T045 is that commit. *(One narrow exception: **FR-005a** permits the M2-only tracker innovation
   channels at T094, because N is unknown until T088 and the T023 serialize split makes it M1-safe.)*
2. **Never rebuild while a bake is live** — workers re-exec `build/autoc`.
3. **Nothing fitness-affecting after the bundle** until the M1 bake completes.
4. **Submodule pointer bump before parent merge** (crrcsim tick stamping).
5. ⛔ **T011a MUST precede T044.** Studies A and B both read the **pre-break** pinned comparators, and T044's
   version bump + fail-loud read makes those dmps unreadable to a 041 binary. Extract the CSVs first or lose
   the prior-M1 baseline (SC-007a) and the blind-gap distribution (FR-024b) permanently.
6. **T046 MUST precede T045.** T045's gate builds xiao, which compiles the generated forward pass —
   a stale `nn_program_generated.cpp` against the new input count fails or, worse, misleads.
   *(T047 was the second half of this constraint and is now INVERTED — `data.dat` is no longer generated, so
   its parser must NOT be updated. See T047.)*
7. All training via `scripts/train.sh` (Constitution IX). Never a background task.

---

## Phase 1: Setup

**Purpose**: confirm the build surfaces and establish artifact homes.

> ↪ **INAV bring-up (T001, T001a) is deferred to Phase 6** (operator decision 2026-08-10): it is hardware
> work that wants the operator near the gear, and nothing in Phases 2–5 depends on it. It is now the first
> item of Phase 6's hardware sub-phase, triggered once the M1 read (T067) looks decent. The IDs are
> unchanged so cross-references (T102) still resolve. Phase 1 is therefore desktop-only.

- [X] T002 ⛔ **VOID** — pre-change xiao byte-size reference; the 2026-08-17 rescope replaced the input vector wholesale, so there is nothing to compare against. *(original text below, kept for provenance)*  [P] [OP] Confirm the xiao baseline builds unchanged: `cd xiao && ~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed`, and record the byte size of `xiao/src/generated/nn_program_generated.cpp` as the pre-change reference. **Host compile — no board attached.** It stays in Phase 1 (operator decision 2026-08-10, when INAV moved out) for two reasons: the byte size is only a *pre-change* reference if taken before T046 overwrites the file, and T045's gate compiles this same generated forward pass (ordering constraint 6). The xiao tasks that need the board — T074, T075, T078 — are in Phase 6 with INAV.
- [X] T003 [P] Create `specs/041-m2-depth/artifacts/` with a `README.md` stating what belongs there (final M1/M2 `data.dat` snapshots per FR-022, archived `nn_weights*.dat` per FR-010) and what does not (anything re-derivable from S3).
- [X] T004 [P] Record the pinned comparator prefixes and their verified retention state in `specs/041-m2-depth/artifacts/README.md`: `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` and `autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/`, both 800/800 `retain=keep` verified 2026-08-07 (Constitution VIII.3 — provenance lives in the repo).

---

## Phase 2: Foundational (blocking prerequisites)

**Purpose**: config-surface and reporting fixes that would otherwise fail for reasons unrelated to correctness,
plus the physics reader that unblocks Study A on existing data.

- [X] T005 Raise or remove the `INI_MAX_LINE` cap in `src/util/config.cc` so a legal `key = value  # long comment` never fails on length. This trap already aborted startup once on `EnablePredictorHead = 1` plus its comment (216 bytes) with no diagnostic, costing a bisect — and 041 adds several commented knobs.
- [X] T006 Surface `ini_parse`'s error line in `src/util/config.cc`: replace the bare `FATAL ERROR: Cannot parse configuration file '<f>'` with `Cannot parse '<file>': line <N>: <line text>` plus a hint for the common causes (over-length line, missing `=`, stray `[`).
- [X] T007 [P] Add a test in `tests/contract_config_tests.cc` that a deliberately malformed temp ini fails with the offending line number in the message (fixture-owned ini, never the production file).
- [X] T008 Strip the mutable-production-value pins from `tests/contract_tracker_config_tests.cc` (`FitStreakThreshold == 0.5`, `FlightArenaRadius == 80`, `CepGateThreshold == 1.25`, `BeaconEmissionConeDeg == 270`, `BeaconLeftMountY == -0.45`). Keep at most a structural guard: production ini parses clean and required keys (`Mode`, `TrackerSourceRun`) are present. 041 changes streak config, so these pins would fail for a reason unrelated to correctness.
- [X] T009 [P] Time-denominate the streak metrics in `src/analytics/` so `pctInStreak` / `avgMaxStreak` are surfaced in seconds consistently (037 P-O11). These are 041's primary progress signal and raw tick-denominated counts read 2× at 20 Hz — fix before they are used to judge a bake.
- [X] T010 Add physics columns to `tools/dmp_dump.cc`: per-tick `acc[3]`, `omegaDotBody[3]`, `alpha`, `vRelWind` from `PhysicsTraceEntry`, plus derived body-frame normal acceleration from `acc[]` + `quat[]`. ⚠️ This data is **already recorded** for every elite reeval (`inputdev_autoc.cpp:1047`) and has **no consumer anywhere** — this is a reader, not a recording change, and it works on **current** dmps.
- [X] T011 [P] Add a test in `tests/dmp_dump_tests.cc` (or the nearest existing suite) that the derived normal acceleration equals 1 g for a synthetic steady-level-flight tick and the documented sign for a synthetic pull-up.
- [X] T011a ⛔ **HARD PREREQUISITE OF PHASE 4 — extract the comparator data while it is still readable.** Run `dmp-dump --physics` (and the standard per-tick CSV) over **both pinned comparators** — the prior M1 `autoc-m1/…2026-07-06T01:35:46.579Z/` and 040-t4 `autoc-m2/…2026-08-04T03:43:24.586Z/` — and archive the CSVs under `specs/041-m2-depth/artifacts/pre-break/`.
  **Why this cannot wait**: T044 bumps the `EvalResults` version and makes reads **fail loud** on older artifacts, so from that commit onward a 041 binary **cannot read either comparator**. Two things become permanently unobtainable if this is skipped:
  1. **The prior M1's per-regime profile** (SC-007a, FR-011c) — its 37-input genome cannot be loaded by a 041 binary either, so the recorded dmp is the *only* route to that baseline. Lose the read and the baseline is gone for good.
  2. **The blind-gap distribution** (FR-024b) — which *defines* the predictor go/no-go bins and feeds the lens-purchase decision.
  Extracting to CSV (rather than keeping a pre-break binary around) means the data outlives any build.

**Checkpoint**: config edits no longer fail spuriously, streak metrics are honest, and load is readable from
existing pinned runs. Study A (Phase 5) is now unblocked without touching the schema.

---

## Phase 3: US1 — Retire the index-coupled failure class (P1)

**Goal**: make the parallel-index bug class unrepresentable rather than tested-for.
**Independent test**: the inventory exists and every entry is fixed, structurally eliminated, or covered by a
zero-answer test; no known instance remains asserted only by a comment (SC-001).

### A0 — the scan

- [X] T012 [US1] Sweep for collection pairs indexed by a shared loop variable across a producer/consumer boundary: `grep -rn "List\.at(\|List\[" src/eval/ src/nn/ tools/ crrcsim/src/mod_inputdev/`, then **read each hit** rather than trusting the pattern. Record findings in a new `specs/041-m2-depth/index-coupling-inventory.md`.
- [X] T013 [P] [US1] Sweep for structs serving two lifetimes (RPC-only vs persisted) across `include/autoc/rpc/` and `include/autoc/eval/`; `ScenarioMetadata` in both roles already cost a launch on 2026-08-02. Add to the inventory.
- [X] T014 [P] [US1] Sweep for values duplicated across two definitions (the `CameraConfig` default vs `hb1AirframeObstruction()` pattern — that one HAS a test and is the model to copy). Add to the inventory.
- [X] T015 [P] [US1] Sweep for "compiled-in default vs recorded config" reads; note which are resolved by the US2 config block and which are not. Add to the inventory.
- [X] T016 [US1] Complete `index-coupling-inventory.md`: every entry marked **fixed**, **structurally eliminated**, or **covered by a zero-answer test**, with the grouped-record migration list as an appendix. This is A0's deliverable and the gate on everything downstream.

### The zero-answer test pattern (write these BEFORE the refactor — they must pass identically after)

- [X] T017 [US1] Add a zero-answer test for the M2 objective in `tests/fitness_decomposition_tests.cc`: construct data whose correct score is **exactly 0**, assert exactly 0. Add the companion assertion that a deliberately one-tick-shifted input scores **visibly worse** — a test that passes either way would be worse than none here.
- [X] T018 [P] [US1] Add the same zero-answer + shifted-worse pair for `vis_frac` in `tests/fitness_decomposition_tests.cc`.
- [X] T019 [P] [US1] Add the same pair for `prediction_score` in `tests/fitness_decomposition_tests.cc`. ⚠️ **Timing exception to this block's "write before the refactor" rule**: `prediction_score`'s pairing is *currently wrong*, so a zero-answer test cannot pass until T022 lands the grouped record. Write it here, expect RED, and confirm it goes green at T022 — that transition is the evidence the pairing was actually fixed. ⚠️ Two fixture traps make these silently vacuous, both already paid for: an empty `pathList` makes `computeScenarioScores` **skip the scenario** so every variant scores 0 and the comparison looks passed without running; and a bare `TEST()` misses the `ConfigManager` fixture so the run proceeds on defaults. Verify each new test **fails** when the fix is reverted.

### The structural fix

- [X] T020 [US1] Define the grouped per-tick record in `include/autoc/rpc/protocol.h`: `tickList[i][k] = { state, cameraView, targetSample }`, with the pre-loop initial state as a **separate named field** beside the list (research.md R5) — **not** `tickList[0]` with sentinel members, which would recreate the hazard as "slot 0 is special". Tracker-only members absent (not zero-filled) in pathgen records.
- [X] T021 [US1] Update the push sites in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` to emit grouped records, and store the initial state into its named field once before the loop.
- [X] T022 [US1] Migrate `src/eval/fitness_decomposition.cc` to the grouped record. **Delete** the `stepIndex - 1` clamp rather than relocating it — if any consumer still needs an offset, the grouping is wrong. This subsumes FR-004: the prediction-score pairing becomes correct by construction rather than by a fix.
- [X] T023 [P] [US1] Migrate `tools/dmp_dump.cc` to the grouped record (coordinate with T010's physics columns). ⚠️ **Also `tools/tracker_dmp_inspect.cc`** — 11 parallel-index hits, found by the T012 sweep and absent from the original task list; it breaks at T045 if skipped (index-coupling-inventory.md appendix).
- [X] T024 [P] [US1] Migrate `tools/renderer.cc` to the grouped record.
- [X] T025 [P] [US1] Migrate `src/eval/source_dmp_loader.cc` to the grouped record.
- [X] T026 [P] [US1] Migrate `src/eval/tracker_stepper.cc` and `crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp` to the grouped record.
- [X] T027 [P] [US1] Migrate any `src/analytics/` reader that consumes per-tick arrays.
- [X] T028 [US1] Add a grouped-record round-trip test in `tests/serialization_tests.cc` (or nearest): serialize → deserialize preserves every member and the tick count, and the initial-state field survives without being confused for tick 0.

**Checkpoint**: the objective is correct by construction, the tests have teeth, and the inventory says what
else shares the shape. ⚠️ Do **not** commit yet — this lands with Phase 4 as one commit (T045).

---

## Phase 4: US2 — One clean-slate contract break (P1)

**Goal**: every model- and schema-incompatible change in a single commit, so exactly one M1 rebake is owed.
**Independent test**: a fresh build records a self-describing dmp with realized wind and exact tick stamps,
read by every consumer, with input-count assertions and metadata tables in agreement (SC-002, SC-003).

### NN input slots

- [X] T029 [US2] Add the five new slots to `include/autoc/nn/nn_inputs.h` in **both** `PathgenInput`/`NNInputs` and `TrackerInput`/`TrackerInputs`, in the order given by [contracts/nn-input-layout.md](contracts/nn-input-layout.md): `IN_ENVELOPE`, `ENVELOPE_SECS`, `ACCEL_X`, `ACCEL_Y`, `ACCEL_Z`. Place `ACCEL_*` **adjacent to `GYRO_*`** so the 6-DOF inertial block is visible. Annotate each `// raw-ok: NN-byte-format buffer`.
- [X] T030 [US2] Add the matching rows to `kPathgenInputMeta` and `kTrackerInputMeta` (name, short name, width). The existing `static_assert` on table length vs `COUNT` enforces this — do not weaken it.
- [X] T031 [US2] Update counts and recompute weight-count `static_assert`s in `include/autoc/nn/topology.h`: `NN_INPUT_COUNT` 37→42, `TRACKER_NN_INPUT_COUNT` 58→63, `TRACKER_NN_TOPOLOGY_STRING` → `"63,32,16r,<out>"`. **Recompute**, never relax.
- [X] T032 [P] [US2] Add `kAccelScale_g` alongside `kCruiseSpeed_mps` / `kDistToBoundaryScale_m` in `include/autoc/nn/nn_inputs.h`, with the rationale comment (±11 g observed must land in a tanh-friendly range).
- [X] T033 [US2] Add config knobs via the `AUTOC_CONFIG_FIELDS(X)` X-macro in `include/autoc/util/config.h`: `EnableEnvelopeInputs`, `EnableAccelInputs`, `AccelScaleG`, and the reserved M2 estimator knobs `EnvelopeSpanLo` / `EnvelopeSpanHi` / `EnvelopeCentroidRadius`. Per Constitution VII, no in-class default initializers for constructor-supplied members.
- [X] T034 [P] [US2] Update the field-count assertion in `tests/contract_config_tests.cc` for the new knobs.

### The step score moves into the tick loop (FR-018a — the single-source-of-truth refactor)

- [X] T035 [US2] Move the per-tick step-score / streak computation out of post-hoc `computeScenarioScores` in `src/eval/fitness_decomposition.cc` into the eval tick path, and record the per-tick result into the tick record. One computation feeds **both** the NN input gather and the fitness accumulation.
- [X] T036 [US2] ⚠️ **Prove DETERMINISM and characterise the delta** in `tests/fitness_decomposition_tests.cc`. *(Reframed by the operator 2026-08-10 — see spec.md § Clarifications "what bit-identical actually has to mean". The original wording said **bit-identical**; the real gate is determinism plus "materially the same or better".)*
  Three obligations, in priority order:
  1. **Determinism** — the same genome and scenario scored twice give exactly the same number, and the move introduces no worker-order or FP-associativity sensitivity. This one IS an equality assertion.
  2. **Materially the same or better** — the inline result is compared against the post-hoc one and the delta is *measured and reported*, not asserted to be zero. Post-hoc reads float-rounded `AircraftState` (`gp_scalar` is `float`); inline reads live state, so a last-bits difference is expected and acceptable. A changed *objective* is not.
  3. **Silence is the failure mode, not difference.** What this task prevents is a rounding change nobody notices. Measuring and reporting satisfies that; asserting `==` and then loosening the tolerance when it fails does not.
  ⚠️ Unchanged: Constitution IV, the pre-run build gate, and the eval-vs-training bitwise regression gate.
- [X] T037 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `src/nn/evaluator.cc` `gather_inputs` from the tick-loop step score. Accumulator is an **external counter** in the stepper, resets **on envelope exit only** (not on regime change), millisecond-based against `FitStreakRampSec`, **linear** normalization — no log, no tanh.
- [X] T038 [US2] Populate `IN_ENVELOPE` / `ENVELOPE_SECS` in `gather_tracker_inputs` from the M2 direct-perception estimator: both beacons CEP-visible AND separation within `[EnvelopeSpanLo, EnvelopeSpanHi]` AND pair centroid within `EnvelopeCentroidRadius`. Same accumulator mechanics as M1 — only the flag's source differs.
- [X] T039 [US2] ✅ **Sign convention SETTLED 2026-08-11 — `specific_force.h` is correct as written, NO flip (see the RESUME block). FRD level flight ⇒ `ACCEL_Z = −1`.** Plumbing is decided — store on `AircraftState` beside `gyroRates_`, computed worker-side; both gathers only COPY. Populate `ACCEL_*` in both gather functions as **body-frame specific force including gravity**: `R(quat)ᵀ · (a_world − g_world) / kAccelScale_g`. ⚠️ **Not FDM kinematic acceleration** — that would put a constant ~1 g offset in the most load-relevant axis, invisible in sim and wrong in flight.
- [X] T040 [US2] Add input-semantics tests in `tests/nn_evaluator_tests.cc`: (a) **steady level flight → normal channel ≈1 g, not ≈0** (the test that catches the kinematic-vs-specific error — and **empirically confirmed on hardware**: the bench table in `docs/COORDINATE_CONVENTIONS.md` reads `+2050` on the normal axis in level attitude — blackbox `accSmooth` counts, `acc_1G ≈ 2048`, so `≈ +1.0 g` — meaning INAV already reports specific force including gravity, exactly the sim semantics required); (b) documented sign on a pull-up; (c) `IN_ENVELOPE` ∈ {0,1}; (d) `ENVELOPE_SECS` monotone within a streak, 0 immediately after exit, saturates at 1; (e) `ENVELOPE_SECS` identical for a given wall-clock duration at two cadences; (f) M1 `IN_ENVELOPE` agrees tick-for-tick with the objective's own threshold decision.
- [X] T041 [P] [US2] Update layout assertions in `tests/contract_evaluator_tests.cc` for both modes' new counts.

### Camera model (M2-only, fidelity to ordered hardware)

- [X] T041a [US2] Set `CameraPixelsV = 240 → 200` in `autoc-tracker.ini`, `autoc-eval-tracker.ini`, and `autoc-eval-tracker-visual.ini`, giving V = 200 × 0.375 = **75°** (H unchanged at 120°). Leave `CameraDegPerPixel` alone so `radPerPx`, per-pixel quantisation and CEP are untouched. Rationale in the ini comment: the ordered 1.8 mm fisheye on OV9281 estimates ~124°×78° equidistant, 120×75 is the conservative split, and 320:200 = 1.6 matches the real 1280×800 aspect (the prior 240 px was a 4:3 invention, optimistic by 15° vertically). ⚠️ **Fitness-affecting** → A1 bundle only.
- [X] T041d [US2] ✅ **DONE 2026-08-16 — supersedes the pitch half of T041a.** Set `CameraDegPerPixel = 0.375 → 0.304` in `autoc-tracker.ini`, `autoc-eval-tracker.ini` and `autoc-eval-tracker-visual.ini`, plus the two struct defaults (`CameraConfig::deg_per_px`, `AutocConfig::cameraDegPerPixel`) — five definitions of one value, the E1 hazard in `index-coupling-inventory.md`. Grid unchanged at 320 × 200 ⇒ **97.3° H × 60.8° V**. **Trigger**: T041a explicitly reserved this knob for measurement (*"revisit from MEASUREMENT (the real grid image, an 031 deliverable when hardware lands), not from a second estimate"*), and 031's ruled-mat calibration of the 1.8 mm fisheye on the OV9281 landed on the `031-beacon-camera` merge — equidistant **confirmed** (no action; the sim has been f·θ since 038 t9), native pitch **0.076°/px**, FOV **95° H × 61° V** by tape. 4 × 0.076 = 0.304 makes the sim grid a true 4× bin of the real 1280 × 800. ⚠️ The estimate was wrong in the **narrow** direction, ~19% on both axes — the single-fisheye-at-120° assumption is retired for this lens. Consequences: CEP/quantisation ~19% **finer** (separation crossover 27.75 → 34.25 m); field ~19% narrower, which is the **fitness-affecting** half → A1 bundle with the rest, never mid-comparison. Comment fallout fixed at the same time in `camera_projection.h`, `derived_features.h`, `signal_model.cc`, `camera_projection.cc` and the ini `SeparationMinResolvablePx` note.
- [X] T041f [US2] ↩️ **PROPOSED then REVERTED 2026-08-16 — the grid stays 320×200.** T041f briefly set `CameraPixelsH = 320 → 312` so the DERIVED field landed on 031's tape-measured 95° H instead of the f·θ extrapolation's 97.3°. **Operator reversed it, and the reasoning is better**: *"the actual dump from camera will be 320x200 — we know that — so seems we really should go with this and assume some slight measurement errors."* The sensor dumps 320×200 (a 4× bin of 1280×800); the pixel count is the one quantity here known **exactly**, and bending it to make a derived angle match a tape reading models a sensor that does not exist. The residual ~2.4° is absorbed as **tape measurement error** — the calibration itself only claims agreement within 3°.
  **Kept from the exercise** (the reversal did not undo these):
  1. The **pod-nose obstruction sweep**, now recorded in the test: onset between 46.2° and 46.8° half-H, then 0.214% @ 47.42°, **0.628% @ 48.64° (shipped)**, 2.57% @ 54.72°. The blockage sits at the OUTER EDGE of the horizontal field, so it is steeply field-dependent — one degree moves it ~2×.
  2. ⚠️ **`EffectiveFieldDiffersFromNominalByAJustifiedAmount` passes by a HAIR** at the shipped grid: 0.628% against a 0.5% floor. That floor and its "~3% hard-blocked" comment both date from the retired 120° era. The comment now says so, and says to set any future bound from the sweep rather than by nudging the floor.
  3. `CameraPixelsH/V`, `CameraDegPerPixel`, `CameraDetectionRangeM` added to the train/eval ini-agreement guard — M2-only keys, skipped on M1 pairs, but a tracker mismatch would otherwise evaluate a policy against a different optic than it trained on.
  **Net config change: none.** 320 × 200 @ 0.304 °/px ⇒ 97.3° H × 60.8° V, as T041d left it. 46/46 suites green.
- [X] T041e [US2] ✅ **DONE 2026-08-16.** Re-derive the projection tests against the measured field — **46/46 suites green**. Four broke, each a literal that had silently encoded the retired 60° half-field, and all four are now **computed from the config** so they follow the one knob (FR-003) rather than needing a human to re-evaluate a tangent: the ±FOV-edge target positions (were `±17.32` = 10·tan 60°), `SeparationIsInvariant`'s outer sweep sample (was a literal 40°, whose ψ/2 pushed the pair to 50° — outside the new field, so the pair gated and the teeth-guard silently disarmed), and `BearingIsQuantisedToPixelCentres`' ±50° sweep. ⚠️ One finding worth keeping: `FieldOfViewIsDerivedFromGridAndPixelPitch` used `EXPECT_DOUBLE_EQ` between the accessor's **float** multiply and the test's **double** one. That passed exactly only because 0.375 = 3/8 and 320 × 0.375 = 120 are both exactly representable in binary — luck, not a property. 0.304 is a repeating binary fraction, so the two now differ by ~2e-6; the check compares at float precision. Also widened the separation-crossover sweep 40 → 60 m: the crossover scales as 1/`deg_per_px`, and at 34.25 m the old start had under 6 m of headroom, past which the loop would have reported **its own start range** as the answer instead of failing.
- [X] T041b [P] [US2] Update the derived-FOV assertion comment in `include/autoc/eval/camera_projection.h` (currently cites ±0.785 rad / 45° for V) so the documented half-angles match the grid, and add/extend a test asserting derived V = 75° and derived H = 120° from the configured grid — the FR-003 "field and resolution cannot disagree" property.
- [X] T041c [P] [US2] Record in `specs/041-m2-depth/artifacts/README.md` that the projection is **already equidistant** (`camera_projection.cc:158-184`, since 040 T031) and that `CameraDetectionRangeM = 100.0` is now **independently corroborated** by the 031 photon budget (bright-day post-correlation SNR ≈22 @100 m, ÷4 at 4×4 defocus → ≈5.5 vs ×4.5 threshold). Neither is a change; both are facts a later reader will otherwise re-derive.

### Recording changes

- [X] T042 [US2] ⚠️ **IMPLEMENTATION ALREADY DONE — verified 2026-08-11. The task premise is STALE.** It says the field is "serialized-but-never-set, zero in every dmp"; that was true when 041 was specced but was fixed at **038 P0-D-3** (`e6108cc`), and `inputdev_autoc.cpp` has set it from `eom01->getLastLocalAirmass()` (NED ft/s → m/s, NaN-guarded) ever since. **Do not re-implement it.**
  Outstanding: **only the test** — that a non-zero wind survives to the dmp. Note the honest limit of a unit test here: recording is a worker-side effect, so unit level can prove the field round-trips through serialization, and the end-to-end "a run in wind records wind" claim is closed by the **T061 smoke inspection**, not by ctest.
- [X] T043 [US2] ⚠️ **MOSTLY DONE ALREADY — verified 2026-08-11. Scope is much smaller than this text implies.** The 038 P0-D-2 work landed `RecordedRunConfig` (`protocol.h:432`), serialized it into `EvalResults` (`:583`), and flipped `dmp_dump.cc` to read it with **no ConfigManager fallback** (`:655-660`). Do not redo any of that. What actually remains:
  1. **`tools/renderer.cc` is only half-flipped.** It reads `runConfig` at `:3313` but still reads the **live ini** at `:4651` (`ConfigManager::getConfig().fitStreakMultiplierMax`, the HUD streak-colour scale). Display-only, but it is precisely the "reader with a drifted ini" case this task exists to close, and a half-flipped reader is worse than an unflipped one because the inconsistency is invisible.
  2. **Add 041's knobs to `RecordedRunConfig`** — `enableEnvelopeInputs`, `enableAccelInputs`, `accelScaleG`, `envelopeSpanLo/Hi`, `envelopeCentroidRadius`. Without them a dmp cannot say whether its `ACCEL_*` columns were populated or ablated, which is exactly the question the T068 matrix asks of it.
  3. **The test**: a reader with a deliberately-drifted ini still replays the recorded numbers.
- [X] T044 [US2] Bump the `EvalResults` version field and implement **fail-loud** reads naming both the artifact and reader versions (Constitution V; research.md R6). No migration path, no shim. Add a test that a prior-version artifact errors with both numbers and does **not** crash in the allocator — the 038 baseline currently dies as `vector::_M_default_append`, which is exactly the diagnosis this prevents.
- [X] T044a [US2] ⚠️ **IMPLEMENTATION ALREADY DONE — verified 2026-08-11. The task premise is STALE.** `SimStateHandler::getSimulationTimeSinceReset()` already returns `llround(sim_steps * Global::dt * 1000.0)` (**038 P0-D-1**, `e6108cc` — the same commit as T042), replacing the truncation that produced 49/50/51 ms jitter. **Do not re-implement it.**
  **Two consequences, both worth stating because they change the plan:**
  1. ⛔ **The `[OP]` marking and the "determinism-affecting / pointer-bump the submodule first" warning NO LONGER APPLY to this task.** There is no determinism-affecting change owed here. An earlier 2026-08-11 note claimed T044a was still pending and would land *between* two runs of the bitwise gate — that was wrong, and it made the gate look more fragile than it is.
  2. The consumer-side invariant is already enforced **fail-loud at runtime**: `crrcsim_tracker_helper.cpp::initScenario` throws when the first source gap ≠ `SIM_TIME_STEP_MSEC`, and `tracker_stepper.cc` mirrors it.
  Outstanding: **only the test** asserting exact gaps.

### Land it

- [X] T045 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  [US2] [OP] **THE SINGLE COMMIT** — everything from Phase 3 and Phase 4 together (FR-005).
  ⛔ **Preconditions**: **T046** (`nn2cpp` regenerated) and **T047** (`sim_response.py` parser) must be done *first* — this gate builds xiao, which compiles the generated forward pass, so a stale generated file against the new input count either fails to build or builds something wrong. Also **T011a** must be done (see Phase 2), since this commit makes the pre-break comparators unreadable.
  Gates, all of which must pass: `bash scripts/rebuild.sh` green; `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` green; Constitution VI audit clean on touched paths (`grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'`). Nothing lands after this until the M1 bake completes.
- [X] T046 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  [US2] Regenerate `xiao/src/generated/nn_program_generated.cpp` via `tools/nn2cpp.cc` for the new layout, and add a parity test that the generated forward pass matches the desktop forward pass on a fixed input vector.
  ⛔ **ORDERING TRAP found 2026-08-11 — read before starting.** `nn2cpp -i FILE` requires an **NN01 weight file**, i.e. a *trained genome*. The current generated file is baked from the **pinned prior M1** (`generatedNNProgramSource` = `…2026-07-06T01:35:46.579Z/gen9200.dmp.zst`, `generatedNNInputCount = 37`). That genome is 37-input and **cannot be loaded by a 041 binary** (FR-011c), and **no 42-input genome exists until the new M1 bake lands at T065.** So T046 as literally written cannot be done before T045.
  **Resolution — T046 splits in two:**
  1. **T046a, before T045**: regenerate from a **placeholder 42-input genome** at the new topology (Xavier-init or zeroed — `nn_xavier_init` exists). Its only job is to make the xiao compile and the parity test run against the *new layout*. It is **not flyable** and must say so at the top of the generated file.
  2. **T046b, after T065**: regenerate from the **real new M1 weights** archived at T065. This is the one that flies, and it gates T078/T080.
  ⚠️ **Second, worse gap — the layout mismatch is currently SILENT.** The generated forward pass hardcodes `for (i = 0; i < 37; i++)` and fixed `nn_weights + offset` slices, while the gather now fills **42** floats. That combination **compiles cleanly** and quietly reads the first 37 slots against weights trained for a different layout — precisely the "builds something wrong" case T045's precondition warns about. `generatedNNInputCount` is exported but **never checked against `PathgenInput::COUNT`**. Add that assertion as part of this task (compile-time in the generated file, or a boot-assert in firmware): it converts an invisible wrong-flight into a loud failure, which is the whole point of US1.
- [X] T047 [P] [US2] ⛔ **INVERTED — do NOT update `specs/019-improved-crrcsim/sim_response.py`. Verified 2026-08-10.** The task assumed `data.dat` is a live output. It is not: **035 FR-P05 retired the per-step writer** (`src/autoc.cc:1969` "data.dat output file retired; the dmp (S3) is the single training trace"), and nothing in the tree writes one. Operator 2026-08-10: *"data.dat is gone from being generated, but the content in s3 remains."*
  So `sim_response.py` is a **reader of historical artifacts only**, and its documented field map is frozen at the **021-era** layout (`F0..F49`, 33-input era — pre-038's four arena inputs, let alone 041's five). Teaching it the 041 column set would make it mis-parse every file it can still be pointed at, which is the exact opposite of the intent. It is also a pre-035 script under [[feedback_historical_scripts_immutable]].
  **Consequence for ordering**: hard constraint 6 reduces to **T046 only**.
- [X] T048 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  [US2] [OP] If `CMakeLists.txt` was touched by this phase, run a clean `bash scripts/rebuild-perf.sh` (Constitution IV — an incremental reconfigure can leave stale link state and miss test registration).

**Checkpoint**: one commit, one owed rebake. All three build targets green.

---

## Phase 5: US3 — Instruments that can answer the questions (P1)

**Goal**: build the measurement tools before spending compute.
**Independent test**: empty-mask ablation reproduces baseline fitness exactly; Study A reports per-regime load
from a pinned run with no recording change (SC-004, SC-005).

### The ablation tool

- [X] T049 ⛔ **SUPERSEDED** — folded into **P1-1 (ablation ran via autoc's built-in `--zero-input`; the standalone tool was never needed)** of the P-task list. *(original text below, kept for provenance)*  [US3] ⚠️ **PARTIALLY DONE 2026-08-13 — the MASKING ENGINE ships, the standalone binary does not.** What exists and is validated (`specs/041-m2-depth/ablation/instrument-validation.md`): the mask is plumbed `WorkerInit.nnInputMask` → worker → `NNControllerBackend::setInputMask`, applied after the gather and immediately before the forward pass in BOTH modes; slot-name resolution lives in `include/autoc/nn/input_mask.h` against the existing metadata tables, with a hard error listing valid names; `autoc --zero-input NAMES` drives it, and `scripts/train.sh` forwards extra args so an ablation runs through the Constitution IX detached path. Empty-mask identity, determinism, degrade-on-known-critical, wrong-length and off-by-one are all tested (11 tests, `tests/input_mask_tests.cc`). **Still owed**: the `tools/nn_ablate.cc` front-end that runs baseline+ablated and emits the T051 report. Original text: Create `tools/nn_ablate.cc` per [contracts/ablation-cli.md](contracts/ablation-cli.md): `-i <ini> --genome <dmp-key|weights-file> [--zero-input NAME[,...]] [--out csv]`. Slot names resolve against `kPathgenInputMeta` / `kTrackerInputMeta` — **no new naming infrastructure needed**, it already exists. Masked columns forced to 0.0 every tick after gathering, before the forward pass. Unrecognised slot name is a **hard error** listing the valid set, never a silent no-op.
- [X] T050 ⛔ **SUPERSEDED** — folded into **P1-1** of the P-task list. *(original text below, kept for provenance)*  [US3] Register the `nn_ablate` target and its test in `CMakeLists.txt`.
- [X] T051 ⛔ **SUPERSEDED** — folded into **P1-1** of the P-task list. *(original text below, kept for provenance)*  [US3] Report fields per the contract: Δfitness; per-axis Δ`dCtrl` / Δ`⟨|u|⟩`; Δ`pctInStreak` / Δ`avgMaxStreak`; Δ peak and mean normal load; per-scenario Δ distribution; and **per-regime breakdown** (`{tracking, intercept, patrol}`) — required, not optional (FR-011a), because the hypothesis predicts a signal in *one* regime and pooling would hide it.
- [X] T052 [US3] ✅ **DONE as `tests/input_mask_tests.cc` + the end-to-end run** (empty-mask identity verified against the real eval, not a stub: `-1045.136851` both sides). Original text: Add `tests/nn_ablate_tests.cc`: **empty-mask identity** — with no mask, reproduces the source run's fitness **exactly** (SC-004; a tool that quietly perturbs the eval path makes every finding worthless); unknown slot name errors with the valid list; masking a known-critical input degrades measurably; two identical invocations are bit-identical.
- [X] T053 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  [US3] [OP] Clean `bash scripts/rebuild-perf.sh` for the `CMakeLists.txt` touch (Constitution IV).

### Study A — regime and load (report-only; no load axis in 041)

- [X] T054 [US3] Create `src/analytics/regime_load_study.py`: classify every tick into `{tracking, intercept, patrol}` using the **existing** rule (`stpPt ≥ 0.5`; below that, smoothed `d(dist)/dt < 0` is intercept, else patrol) — reuse `dynamics_progress.py:74-80`'s definition rather than writing a new one, so numbers stay comparable with every prior report.
- [X] T055 [US3] Report per regime, per axis: `dCtrl`, `⟨|u|⟩`, and load distribution **plus peak** (peak is the damage-relevant statistic; a mean hides ±11 g excursions entirely). Emit machine-readable CSV alongside any plot. State sample sizes and any excluded ticks.
  ⚠️ **Load comes from the recorded NN input column `ACCEL_Z`, NOT from `physicsTrace`** (spec.md Clarifications, session 2026-08-10). `nz_g = −ACCEL_Z × kAccelScale_g`, the same line flight uses. The physics trace is capped at 175 ms per scenario (0.89% of ticks) and is staying that way. Consequence: **load exists only for runs baked after T039** — the two pre-break comparators have no usable load at all, so T058 reports their CONTROL half only.
- [X] T056 [US3] Add the H2 test to the study: within each regime, does pitch/roll `dCtrl` predict throttle level and load? Report correlation with a stated confidence, not a scatter plot alone.
- [X] T057 ⛔ **SUPERSEDED** — folded into **P1-3** of the P-task list. *(original text below, kept for provenance)*  [US3] ⏳ **IMPLEMENTED 2026-08-13, NOT YET RUN** — the code is in `regime_load_study.py` and emits `<label>-autocorr.csv`, but it needs `ACCEL_*` and so cannot execute until the new M1 (T071). Emitted empty for both pre-break comparators, correctly. Add normal-load **autocorrelation at the history lags** to the study — ⚠️ runs on the **new M1** (T071), not on the pre-break CSVs, which carry no load. Sampling is 20 Hz by decision, so the shortest observable lag is 50 ms — this is the cheap evidence that decides whether the accel channels ever need temporal depth (research.md R1 fallback ladder). Strong autocorrelation at 50–100 ms ⇒ one instantaneous sample suffices and neither fallback is warranted.
- [X] T058 [US3] Run Study A's **control half** on the **pre-break CSVs already archived at T011a** (`artifacts/pre-break/*.csv.gz`, extracted 2026-08-10), covering the pinned prior M1 and 040-t4, producing `specs/041-m2-depth/study-a/`. Per-regime `dCtrl` / `⟨|u|⟩` and the regime classification: 100% tick coverage, fully available. This is the **prior M1's per-regime profile**, the only obtainable form of that baseline (FR-011c: its 37-input genome cannot be loaded by a 041 binary, so it can never be re-evaluated or ablated).
  ⚠️ **No load half here.** The prior M1 predates `ACCEL_*` and its physics trace covers 0.89% of ticks, so old-vs-new load comparison is impossible — accepted by the operator 2026-08-10 ("old M1 is what it is"). Every 041 load number is a single-run profile of the NEW M1 (T071), never a delta. ⚠️ Do **not** plan to re-extract from S3: after T044 these comparators are unreadable by a 041 binary.

**Checkpoint**: both instruments exist and are validated. Study A's findings are recorded as input to the
**follow-on** aggressiveness feature — 041 builds no load axis.

---

## Phase 6: US4 — Give the controller the tracking state it is paid for (P1)

**Goal**: bake a new M1 carrying the envelope inputs; answer whether the policy uses them.
**Independent test**: the bake clears non-regression, and ablation says whether the learned policy depends on
the new inputs (SC-006, SC-007, SC-007a).

### Smoke — plumbing and ballpark only, NOT comparison arms

- [X] T059 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Pre-run build gate, then M1 smoke: `scripts/train.sh autoc-basic-m1.ini logs/autoc-041-smoke-m1.log` (pop 3000, fast). Check: builds, runs, climbs at all, new inputs carry sane values. **No delta is being measured.**
- [X] T060 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US4] [OP] M2-mode smoke — the topology change is **generic**, so prove it runs in both modes: `scripts/train.sh autoc-tracker.ini logs/autoc-041-smoke-m2.log`, killed once ticking cleanly.
- [X] T061 ⛔ **SUPERSEDED** — folded into **P3-1** of the P-task list. *(original text below, kept for provenance)*  [US4] Inspect the smoke runs' `data.dat` / dmp for the new columns: `IN_ENVELOPE` toggling plausibly, `ENVELOPE_SECS` ramping and resetting, `ACCEL_Z` ≈1 g in level flight. A diagnostic that does not vary is telling you something (040's trap 3).

### Production bake

- [X] T062 [US4] [OP] ⚠️ **REFRAMED by the operator 2026-08-10 — this is not a retry-budget task.** Asked how many attempts to allow before re-drawing, the answer was: *"a good bake is good enough — forget the random as we aren't quite at the brittle edge of NN capacity. Often the early runs uncover missed assumptions or non determinism — that's the key search."*
  So: **no fixed attempt count, and no lottery framing.** Run until a bake is good; a good bake is good enough. The important correction is what an early failure MEANS — it is a **diagnostic**, not bad luck. The first runs are the cheapest place to discover a missed assumption or a non-determinism, and treating a bad one as "re-draw and hope" throws away the signal we are actually there to collect. R8's 3-attempt proposal is superseded.
  The **abort signature** below still stands, but as an *investigate* trigger rather than a *re-draw* trigger: abort on the stuck-basin signature — throttle amplitude exactly 1.000 with **σ = 0.000** and `dCtrl` 0.000, plus `avgMaxStreak` frozen and best-sigma not annealing past ~0.14 by gen 200. Note that throttle *saturation* (mean ≈0.85, σ>0) is **not** the tell; climbers pass through it.
- [X] T063 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Pre-run build gate, then `scripts/train.sh autoc.ini logs/autoc-041-t1-m1-envelope.log` (pop 8000 / 49 winds). Judge climb on `pctInStreak` / `avgMaxStreak`, **not** completions.
- [X] T064 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Repeat as a lottery re-draw if the abort criterion fires, within the declared budget, incrementing the artifact index (`t2`, `t3`) per `autoc-<feature>-t<N>-<details>`.
- [X] T065 ⛔ **SUPERSEDED** — folded into **P4-2** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Immediately on completion: tag all dmp objects `retain=keep`, and archive `nn_weights*.dat` beside the dmp (FR-010 — the dmp preserves numbers, only NN01 preserves a controller you can re-fly). Record the S3 prefix in the outcome doc. ⚠️ **The `data.dat` snapshot FR-022 asks for is MOOT** — 035 FR-P05 retired the writer, so there is no per-tick file for "the next launch" to overwrite. The dmp IS the trace, and `retain=keep` is what preserves it. FR-022 is satisfied by the pinning, not by a file copy.

### Reads

- [X] T066 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [US4] Generate the report set: `scripts/generate_pngs.sh m1 logs/autoc-041-t1-m1-envelope.log`.
- [X] T067 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [US4] Assess non-regression against **the pinned prior M1 alone** (`autoc-m1/…2026-07-06T01:35:46.579Z/`), with a **loose bar**: the trigger is *materially worse*, not a numeric band. ⚠️ **Operator decision 2026-08-10, superseding research.md R7's "historical band".** Reasons, both worth keeping: we have **one** pinned comparator, so a "band" would be a spread invented from a single sample; and per the T062 reframing the early runs exist to surface missed assumptions, which a noise-triggered bar actively works against. Same spirit as T036 — measure, report, and judge materially, rather than assert a threshold that gets loosened when it fires. Compare on: `pctInStreak` / `avgMaxStreak` within noise of or above the band; crash/OOB counts not worse; per-axis `dCtrl` / `⟨|u|⟩` per regime; peak load not higher. ⚠️ Absolute fitness sums are **not** comparable across runs with different scenario counts — use per-scenario or per-step rates.
- [X] T068 ⛔ **SUPERSEDED** — folded into **P1-1 ✅ (ran 2026-08-17 — H1a **fails**)** of the P-task list. *(original text below, kept for provenance)*  [US4] Run the **ablation matrix** on the new elite: flag alone (`--zero-input IN_ENVELOPE`), duration alone (`ENVELOPE_SECS`), both, and the accel channels — per FR-014b. Expectation is that the **flag** carries the effect; a duration-only effect would be distinct and more surprising.
- [X] T069 ⛔ **SUPERSEDED** — folded into **P1-1 ✅** of the P-task list. *(original text below, kept for provenance)*  [US4] Run the **control-input ablations** for calibration (FR-011b): `DIST_NOW` (known-critical end), `GYRO_P,GYRO_Q,GYRO_R`, `INWARD_BODY_X,INWARD_BODY_Y,INWARD_BODY_Z` (plausibly marginal). State the envelope verdict as a **position on this spectrum**, never against an assumed absolute threshold.
- [X] T070 ⛔ **SUPERSEDED** — folded into **P1-1 ✅** of the P-task list. *(original text below, kept for provenance)*  [US4] Record the H1a verdict as **pass** (fitness drop **and** behavioural shift, beyond the marginal end of the control spectrum), **partial** (one but not both), or **fail** (neither) — per regime. All three close the hypothesis; only an unclassifiable result fails SC-007.
- [X] T071 ⛔ **SUPERSEDED** — folded into **P1-2 ✅** of the P-task list. *(original text below, kept for provenance)*  [US4] Run Study A on the new elite and produce the **per-regime intent comparison** against the prior M1's profile from T058 (SC-007a). Per-axis aggressiveness is comparable across the two runs; **peak load per regime** (SC-008) is **new-M1-only** — there is no prior-M1 load to compare against, and peak is bounded by the 20 Hz sampling decision (a between-tick structural peak is not observed). Report as a **ballpark read**, not an attributable effect size — it is a cross-run profile comparison, not a controlled delta.

### Hardware deployment (conditional on the M1 result)

> ⚠️ **Standing hardware procedure — applies to EVERY INAV flash in this feature (T001, T072, T078, T080)**:
> **remove the GPS before flashing the INAV controller.** Known quirk, not up for debate. Forgetting it costs
> a debugging session, so it belongs in the runbook rather than in somebody's memory.

**This sub-phase is where every gear-attached task lives** — both INAV targets *and* the xiao board work
(T074, T075, T078). Nothing before this point requires hardware; the xiao tasks outside this phase (T002,
T046, T045's gate) are host compiles only.

**INAV bring-up** (moved here from Phase 1, 2026-08-10 — operator is near the gear at this point). Trigger:
the T067 non-regression read looks decent. Do this **before** T072, so the baseline is known-good before it
is modified.

- [X] T001 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Confirm the INAV baseline builds in `~/inav` for **both** established targets — **bench = `MAMBAF722_2022A`** (STM32F722; `xiao/inav-bench.cfg`) and **flight = `MATEKF722MINI`** (`xiao/inav-hb1.cfg`), both currently at `63cffaf4`. Routine and precedented: **021 T041 already did exactly this** ("INAV builds for bench (MAMBAF722_2022A) and flight (MATEKF722MINI)", closed), and the commands are recorded in `specs/020-pre-flight-pipeline/plan.md`: `cd ~/inav && mkdir -p build && cd build && cmake .. && make MAMBAF722_2022A`. **Bench first.**
- [X] T001a ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Record the two-variant build/deploy sequence in `specs/041-m2-depth/artifacts/README.md`, pointing at `specs/020-pre-flight-pipeline/plan.md` for the commands rather than restating them. Every later INAV task (T072, T078, T080) builds and flashes **both** targets, bench first — a change validated only on the bench target is not validated for flight.

- [X] T072 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Extend `MSP2_AUTOC_STATE` in `~/inav/src/main/fc/fc_msp.c` (the `MSP2_INAV_LOCAL_STATE` case) to carry accel in the **same single round trip**. Copy the shape of fork commit `63cffaf4f` ("extend MSP2_AUTOC_STATE with filtered gyro rates"): append at payload end, fixed integer scale stated at the write site, and document the axis/sign convention for the consumer. ⚠️ **Source `acc.accADCf` — the TRANSFORMED field, never a raw sensor read.** `acceleration.c:563-568` applies `applySensorAlignment` then `applyBoardAlignment` then divides by `acc.dev.acc_1G`, so `acc.accADCf` is board-alignment-corrected and **already in g units**. This mirrors `gyro.gyroADCf` (`gyro.c:438-442`, same two alignment calls), which is exactly why the existing gyro extension is correct. **Do NOT use** the file-static `accADC` (`acceleration.c:73`) or `acc.dev.ADCRaw` — those are pre-alignment and would bake in each board's misalignment differently (bench roll = −16 vs flight). Wire encoding: milli-g `int16` = `lrintf(acc.accADCf[axis] * 1000.0f)`, giving ±32 g against ±11 g observed. ⚠️ **The wire carries INAV's native FLU, unflipped** — exactly as the quat and gyro already do. The FLU→FRD y/z flip belongs at T074's msplink boundary, once, beside the other two (settled 2026-08-11). Build **both** targets, bench first; disconnect GPS before flashing.
- [X] T073 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] ✅ **The bench table is fully resolved (2026-08-11) — all three rows are consistent and all three are safe to assert.** INAV's frame is FLU and `accADCf` is proper acceleration; nose-up `+2050` on X is correct, and it was the discriminating row precisely because FLU→FRD shares the x axis. Assert in **FRD** (post-msplink): level `[0,0,−1]`, nose up `[+1,0,0]`, right wing down `[0,−1,0]` — see `docs/COORDINATE_CONVENTIONS.md` → "Accelerometer as an INTERFACE quantity (041)" for the `~/inav` evidence. ⚠️ Board alignment DIFFERS bench vs flight (different mounting), so verify on **both** targets — a bench-only check is not a flight check. Pin the accel axis and sign convention against the **already-measured bench table** in `docs/COORDINATE_CONVENTIONS.md` ("Ground Verification Results, bench 2026-03-30", `MAMBAF722_2022A`, board alignment roll = −16): level → `[~0, ~0, +2050]`; right wing down 90° → `[~0, +2060, ~0]`; nose up 90° → `[+2050, ~0, ~0]`. ⚠️ **Units**: those are **blackbox `accSmooth` counts** (`acc_1G ≈ 2048`), *not* the runtime `acc.accADCf`, which is the same vector already divided by `acc_1G` — i.e. `+2050 counts` ⇔ `+1.0 g`. Convert before comparing. The **axes and signs** transfer directly; only the scale differs. T073 is therefore *match the table*, not derive the convention — add a test asserting all three attitudes. ⚠️ Board alignment differs between bench (roll = −16) and flight, so verify on **both** targets; do not assume one target's result transfers.
- [X] T074 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Consume the new accel fields in `xiao/src/msplink.cpp` and feed `ACCEL_*` into the input vector with the same specific-force semantics as sim. **The conversion is `accel_FRD = (a[0], −a[1], −a[2])`** — the identical y/z flip already applied to the quat (`inavQuatToAerospaceEB`) and the gyro (`msplink.cpp:964-966`), because INAV's frame is FLU. Put it beside them so the three cannot drift. Level flight must land on `ACCEL_Z ≈ −1`, matching sim.
- [X] T074a ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] ⭐ **Bench-observable NN inputs — operator requirement 2026-08-11: "we will definitely want the NN inputs logged in xiao, and in the sim as usual, to help troubleshoot; bench where we move the craft in various directions to confirm hypothesis."** The engaged path already satisfies this and needs nothing: `TickRecord.inputs[kNumInputs]` is *"post-gather values ACTUALLY fed to the NN (honest)"*, so growing `kNumInputs` 37→42 logs `ACCEL_*` + envelope per tick for free.
  **The gap is the bench posture itself.** Moving the craft by hand happens **armed-but-not-engaged**, and in that state only `FlightStateRecord` is written — pos, vel, quat, and **no accel**. So the one test that confirms the convention is the one state that cannot see it. Close it:
  1. Add `int16_t accel_frd[3]` to `FlightStateRecord` (`xiao/include/flight_log_format.h`) — **post-flip aerospace FRD**, the value the NN would receive, same convention as its `quat[4]` (already `q_EB`, post-`neuQuaternionToNed`).
  2. Add `int16_t accel_inav[3]` alongside it — the **raw INAV FLU** value, pre-flip. ⚠️ Not redundant: with only the post-flip number a wrong reading cannot be attributed to the sensor, the wire, or the flip. With both, the bench says *which step* is wrong. The operator's standing constraint is that this must be right on the **first** flight, and one extra `int16[3]` is the cheapest possible way to make the failure legible.
  3. **Bump `kFormatVersion` 3 → 4** and update the decoder (Constitution V — every decoder loud-fails on unknown version).
  **Bench acceptance** (the hypothesis under test, settled 2026-08-11): hold the craft level → `accel_frd ≈ [0, 0, −1]`; nose up 90° → `[+1, 0, 0]`; right wing down 90° → `[0, −1, 0]`; and `accel_inav` shows the un-flipped FLU counterparts `[0,0,+1]`, `[+1,0,0]`, `[0,+1,0]`. Verify on **both** targets — board alignment differs (bench roll = −16, flight roll = 1700 / yaw = 900).
  **Sim side needs no work**: `AircraftState::nnInputs_` already records the full input vector per tick, so `ACCEL_*` and the envelope slots reach the dmp as a consequence of being inputs, and `dmp-dump` labels them from the metadata short names. Confirm the columns appear at T061 rather than building anything.
- [X] T075 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Implement `IN_ENVELOPE` / `ENVELOPE_SECS` on-target in `xiao/src/`: the step-score cone geometry (`FitDistScaleBehind`/`Ahead`, `FitConeAngleDeg`) thresholded at `FitStreakThreshold`, plus the duration accumulator with a **reset on engage** as well as on envelope exit (FR-022a). This is firmware work, not codegen.
- [X] T076 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Verify the added payload does not push the MSP cycle past its loop budget. 039 measured zero overruns at 115200 with the prior payload; if headroom is marginal, the unexercised 460800 baud-raise lever is the documented next step rather than dropping the field.
- [X] T077 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] Decide explicitly whether the queued `mspOverrideInit` first-frame patch (backlog C1 — MSPRCOVERRIDE engage pays a spurious 200 ms floor) rides along, since INAV is being built and flashed anyway. Record the decision either way — this is the same "now is the time" logic as the format break, and the window closes when the flash does.
- [X] T078 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Bench parity before flight (SC-011a): generated forward pass reproduces the desktop forward pass on a fixed input vector, and the new inputs read sane values on-target against a known geometry. **Includes the T074a attitude sweep** — move the craft through level / nose-up / right-wing-down and read `accel_frd` + `accel_inav` back out of the log, on **both** targets. This is the empirical confirmation of the 2026-08-11 convention finding; do not treat the desk derivation as sufficient on its own.
- [X] T079 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] Flight-test go/no-go per FR-022d: proceed **unless** the M1 result is an utter fail/reject (non-regression failed, or H1a a clear fail with no behavioural change). **Record the decision and its reason either way** — silence on this point is not an acceptable outcome (SC-011b).
- [X] T080 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [US4] [OP] If go: fly, and produce a flight report in `flight-results/flight-<date>/` with per-flight clock-anchor fit (standing practice). Watch the standing load trend — +11.2 g / −8.4 g is the current record and loads have crept up flight-over-flight.

**Checkpoint**: M1 rebaked and pinned; the central hypothesis has a verdict; the new M1 is the M2 source.

---

## Phase 7: US5 — Decide the predictor's fate on evidence (P2)

**Goal**: settle offline whether the head can produce usable signal, before committing a bake.
**Independent test**: the study returns a verdict against a no-information baseline, binned by gap age
(SC-009).

- [X] T081 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US5] Extend `src/analytics/regime_load_study.py` (or a sibling in the same package) to extract per-tick target-bearing truth and visibility from the **T011a pre-break tracker CSVs** (not from S3 — see constraint 5), and to compute the **hold-last-seen** baseline (dead-reckon at zero rate).
- [X] T082 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US5] Produce the **blind-gap distribution** — frequency, duration histogram, exit→re-entry bearing offset. ⚠️ **This must come first**, because it *defines* the go/no-go criterion (FR-024b), and it is a deliverable in its own right regardless of the predictor verdict — **it is an input to the lens purchase** (1.8 mm vs 2.x mm depends on how useful the predictor turns out to be), and nobody has measured it.
  ⚠️ **Measured at V = 90°, applied at V = 75°.** The recorded runs available (040-t4) predate FR-029, so their gap distribution is **optimistic** — narrowing V shifts mass toward longer gaps. This is conservative in the right direction (bins qualified against t4 are a harder bar than reality), so use it, but say so. Predictability itself (T083) is largely FOV-independent and needs no caveat.
  Cross-check against the physics: [camera-era-knobs.md](../031-beacon-camera/camera-era-knobs.md) §3 predicts a 3 g target exits a ±36° half-field in ~1.1 s @50 m / ~1.5 s @100 m — so expect the relevant timescale to be **order 1 s, not sub-second**. A measured distribution far from that wants explaining before it is trusted. ⚠️ **OPERATOR DECISION 2026-08-16 — report this as a STATED LOWER BOUND.** The pre-break CSVs were captured at **120° × 90°**; the measured flight lens is **95° × 61°** (T041d), roughly HALF the solid angle. Every gap frequency and duration here is therefore **optimistic** versus the real optic — the predictor's measured value is a floor, not an estimate. Say so in the deliverable, because it feeds a hardware purchase and a floor argues for the lens differently than a point estimate does. Chosen over re-measuring at the new FOV, which would have blocked the lens decision behind a full run. Refreshing it against the A1 run later is cheap and stays available.
- [X] T083 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US5] Fit an offline regressor for **target (a) — the continuous current-bearing estimate**: from the perception history window, predict the target's bearing *now*, including through blind ticks. Score as **r² against hold-last-seen**, **binned by seconds since last truth**, with per-bin sample counts. ⚠️ A pooled number is misleading by construction — on visible ticks the truth is an input, so both head and baseline are near-perfect.
- [X] T084 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [P] [US5] Fit target **(b) — Δspan at 50/100/150 ms** as the *control* that confirms why the old head failed. Confounded until the pairing is correct; T022 satisfies that structurally.
- [X] T085 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [P] [US5] Fit target **(c) — discounted future `stepPoints`** vs a constant-mean baseline (the value-head fallback).
- [X] T086 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US5] Apply the go/no-go rule (FR-024a): the head must beat hold-last-seen **in the gap-age bins where real excursions occur**, per T082's measured distribution, by a margin stated in advance, with adequate per-bin samples. Bound the claim by two physical facts (FR-030, FR-031): the drift budget Δθ ≈ ½·a·t²/r + 1–2° IMU error sets what hold-last-seen's error *should* look like as a gap ages, and **warm code relock has a ≈155 ms floor** — so the predictor's value is in **pointing, not latency**, and any claimed reduction in time-to-reacquire below 155 ms is unphysical. ⚠️ The verdict feeds a hardware decision (which lens gets bought), so record it with its evidence rather than as a verdict alone.
- [X] T087 [US5] ⛔ **DROPPED — operator decision 2026-08-10. Do not run E1.** T086 settles the head's fate on offline evidence, and T088 then re-targets or retires it; either way the "dead head" configuration E1 exists to measure **stops existing**, so its answer cannot change anything downstream. Removes one tracker run from the plan. *(Original task text kept below for provenance.)* ~~is E1 (`EnablePredictorHead` 0 vs 1, short tracker runs) still worth a run, given that T086 decides the head's fate anyway? If the head is being retired or re-targeted regardless, E1's question ("is the dead head taxing the search") is moot and this task should be **dropped**. Flagged outstanding at the second clarify pass; resolving it may remove a tracker run from the plan.~~
- [X] T088 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US5] Record the **C3 decision** in `specs/041-m2-depth/predictor-decision.md`: re-targeted continuous-estimate head, value-head fallback, or **retire**. Retirement is an accepted outcome (FR-027) and shrinks the output topology 7→3, reclaiming 119 output weights and a third of the lexicase pool.

**Checkpoint**: the predictor question is answered before any M2 compute is spent.

---

## Phase 8: US6 — One M2 bake, scoped to the predictor question (P2)

**Goal**: report whether prediction produced signal.
**Independent test**: the predictor verdict is reportable independent of any aggressiveness outcome (SC-010).

*Tasks T089–T094 apply only if T088 decided to build a head. If it decided to retire, do T089 and T093–T096 only.*

- [X] T089 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Apply the T088 decision to `include/autoc/nn/topology.h`: output count 7 (head retained) or 3 (retired), with `TRACKER_NN_TOPOLOGY_STRING` and weight-count `static_assert`s recomputed.
- [X] T090 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Implement the continuous current-bearing estimate outputs in `src/nn/evaluator.cc`, scaled into the target quantity's domain (FR-026) so the usable range is not a small fraction of one output unit — the failure that turned 040-t1's error curve into a saturation readout.
- [X] T091 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Implement scoring in `src/eval/fitness_decomposition.cc`: score against truth wherever truth exists — every visible tick **plus the reacquisition tick** — and **do not** exclude blind ticks by visibility gating (FR-025a). The current objective CEP-gates both endpoints, scoring only where prediction is information-free; that exclusion is the defect being corrected.
- [X] T092 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Implement the **innovation feedback** (FR-025c–f): compute `error = truth − estimate` in the stepper when truth arrives, write it into the *next* tick's tracker input vector, signed per axis. **Hold the last value during blindness** — zeroing would falsely assert "my model is correct". No new staleness slot: the existing `TIME_SINCE_SEEN` covers it. No output-layer recurrence required.
- [X] T093 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Add tests in `tests/fitness_decomposition_tests.cc` and `tests/gather_tracker_inputs_tests.cc`: a perfect estimator scores exactly zero error; blind ticks are included in scoring; the innovation input holds its value through a synthetic blind gap and updates on reacquisition.
- [X] T094 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Update `include/autoc/nn/topology.h` tracker input count `63 → 63 + N` for the innovation channels (N = estimate dimensionality, fixed at T088) and add the metadata rows. ⚠️ This is the **one permitted post-A1 layout change** (FR-005a) — legal because `TrackerInputs` is a separate struct with a separate genome, and 040's T023 `AircraftState::serialize` split means **no M1 source rebake** is needed, so the M1 source dmp stays readable. Recompute the weight-count `static_assert`s; do not relax them.
- [X] T095 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] [OP] Repoint `autoc-tracker.ini` at the **new pinned M1 source** from T065. ⚠️ Check the scenario **shape** explicitly (`SimNumPathsPerGeneration`, `WindScenarios`), not just the run id — reproducing a run once needed four hand-aligned fields, and the 1:1 seed-table guard catches count mismatches only. A config differing in sigmas or enables produces a plausible wrong number.
- [X] T096 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] [OP] Pre-run build gate, then `scripts/train.sh autoc-tracker.ini logs/autoc-041-t<N>-m2-predictor.log`.
- [X] T097 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] Report in priority order: (1) **predictor signal or its absence** — the deliverable, judged as variance explained against the no-information baseline, binned by gap age; (2) whether the M1 aggressiveness change **carried through** — a free observation, not a designed experiment (FR-028); (3) **novel-geometry generalization** on the pinned 038-t10 source plus the training set, against the T085 baseline (13.03 m median, 15.3% inside 5 m trained; 15.00 m, 8.4% novel). ⚠️ State the single-pinned-set limitation: the second novel source was deliberately left to expire, so a difference cannot be cross-checked against an independent sample.
- [X] T098 ⛔ **MOVED→043** — M2 tracking — moved to [043](../044-m2-tracking/README.md) by the 2026-08-17 rescope. *(original text below, kept for provenance)*  [US6] [OP] Pin the M2 run `retain=keep` and archive its weights (FR-010). ⚠️ No `data.dat` snapshot — retired at 035 FR-P05; see T065.

---

## Phase 9: Polish & wrap

- [X] T099 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  Run the Constitution VI type-domain audit on every touched path and either annotate `// raw-ok: <reason>` or convert: `grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ | grep -v -- '// raw-ok:'`. No milestone is done with unannotated raw-type hits in its diff.
- [X] T100 ⛔ **SUPERSEDED** — folded into **P4-4** of the P-task list. *(original text below, kept for provenance)*  [P] Write `specs/041-m2-depth/outcome.md`: every hypothesis (H1a, H1b, H2, H3, predictor) recorded as supported or refuted **with its evidence** (SC-012 — a refuted hypothesis is a successful outcome), plus all pinned S3 prefixes (Constitution VIII.3).
- [X] T101 ⛔ **SUPERSEDED** — folded into **P4-4** of the P-task list. *(original text below, kept for provenance)*  [P] Return deferred items to `specs/BACKLOG.md` in order (Constitution X): accelerometer noise/bias modelling; the accel temporal-depth fallback ladder (differentiation before history); the load/aggressiveness feature seeded by Study A's findings; anything Study A removed from scope. The visibility-filter entry is already filed.
- [X] T102 ⛔ **SUPERSEDED** — folded into **P4-4** of the P-task list. *(original text below, kept for provenance)*  [P] Add the INAV build section to `docs/toolchains.md` from T001's recorded sequence, if not already done there.
- [X] T103 ⛔ **SUPERSEDED** — folded into **P4-4** of the P-task list. *(original text below, kept for provenance)*  [P] Update `~/.claude/.../feedback_no_cereal_versioning.md` to reflect the resolved practice — bump the version field, maintain no compatibility — since it currently reads as an unqualified "never bump" and the constitution outranks it.
- [X] T104 ⛔ **SUPERSEDED** — folded into **P4-4** of the P-task list. *(original text below, kept for provenance)*  Update `specs/BACKLOG.md` routing header with the 041 outcome and the next feature's pointer.

---

## Dependencies & story order

```text
Phase 1 (setup — desktop only; INAV bring-up deferred to Phase 6)
   │
Phase 2 (foundational: config surface, physics reader, ⛔ T011a pre-break CSV extraction)
   │
   ├─────────────► Phase 5 partial: T054–T058 (Study A on the T011a CSVs — must be extracted
   │                                            BEFORE T044's version bump, see constraint 5)
   │
Phase 3 (US1 structural) ──┐
                           ├──► T045 THE SINGLE COMMIT ──► Phase 5 rest (instruments) ──► Phase 6 (US4 bake)
Phase 4 (US2 bundle) ──────┘                                                                   │
                                                                                               ▼
                                                              Phase 7 (US5 predictor decision, offline)
                                                                                               │
                                                                                               ▼
                                                                          Phase 8 (US6 M2 bake) ──► Phase 9
```

**Story independence**:

- **US1** delivers value alone (a bug class retired) even if 041 stops there.
- **US2** is coupled to US1 by the one-commit rule — they are separate work but a single landing.
- **US3** is independent and its Study A half runs on existing data before any change.
- **US4** depends on the commit; its hardware sub-phase is conditional on its own result.
- **US5** is independent of US4's outcome and could run in parallel with the bake (it is offline).
- **US6** depends on US4 (source) and US5 (design decision).

**Parallel opportunities**:

- Phase 1: T002, T003, T004 together.
- Phase 2: T007, T009, T011 alongside T005–T006, T010.
- Phase 3: T013–T015 together; T018–T019 together; T023–T027 together (distinct consumers).
- Phase 5: Study A (T054–T058) fully parallel with the ablation tool (T049–T053).
- Phase 7: T084, T085 alongside T083.
- **Phase 7 can run entirely during the Phase 6 bake** — it is offline and touches no build state. ⚠️ But it must not *rebuild* while the bake is live; analysis only.
- Phase 9: T100–T103 together.

---

## Implementation strategy

**MVP = US1 + US2.** A retired bug class, a correct objective, and one clean contract break is a complete,
defensible increment even if no bake ever runs.

**Incremental delivery**:

1. **Phase 1–2** stops config edits failing spuriously and makes load readable from existing runs. The INAV
   unknown is deliberately **not** retired here — it moved to Phase 6, where the operator is at the bench
   anyway, and it blocks nothing before then.
2. **Study A early** (T054–T058, on current dmps) — cheapest possible information, and it can *remove* work
   from the feature by showing no load problem exists.
3. **US1 + US2 → T045** the single commit.
4. **US3 instruments**, validated before they are trusted.
5. **US4** the bake and its verdict; hardware only if the result earns it.
6. **US5 offline during the bake**, then **US6** if the predictor earns a head.

**What can cheaply kill work — by design**:

| gate | can remove |
|---|---|
| T058 Study A | the entire load/aggressiveness thread from the follow-on's scope |
| T057 load autocorrelation | the accel temporal-depth fallbacks |
| T070 H1a verdict | the whole envelope hypothesis, for the cost of an eval |
| T086 predictor go/no-go | Phase 8's head work, before a 27 h bake |
| T087 E1 decision | one tracker run — **exercised 2026-08-10: dropped, run removed** |

**Total**: 110 tasks. US1 17 · US2 24 · US3 10 · US4 24 · US5 8 · US6 10 · setup/foundational/polish 17.
*(US2 grew by 3 at the 2026-08-10 camera-model pass: T041a–T041c. T011a added at the same pass's ordering
review — it is the one task whose omission is unrecoverable. T001/T001a moved from setup into US4's hardware
sub-phase the same day — same IDs, later position; the count is unchanged.)*

---

## Phase A′ (amendment 2026-08-17) — containment + step-wise cost. **BOTH MODES.**

Implements spec FR-033…FR-039 and the amendment plan. Ordering is load-bearing: **A1–A2 spend no bake
time and can change everything after them.**

### A — read what already exists (zero bake)

- [X] TA01 ⛔ **SUPERSEDED** — folded into **P1-1 ✅ (ran)** of the P-task list. *(original text below, kept for provenance)*  Run the **T068/T069 ablation matrix on the pinned gen-608 elite** (`autoc-m1/autoc-9223370249927095135-2026-08-17T00:48:00.672Z/`, seed 1786927680). Masking is implemented (T049/T052). Arms: `IN_ENVELOPE`; `ENVELOPE_SECS`; both; `ACCEL_X,ACCEL_Y,ACCEL_Z`; `ACCEL_Y` alone; plus the calibration spectrum `DIST_NOW` (known-critical) and `GYRO_P,GYRO_Q,GYRO_R`. ⛔ **Gate on SC-015: no input may be dropped before this runs.** The contribution ranking is a screen; this is the verdict.
- [X] TA02 ⛔ **SUPERSEDED** — folded into **P1-3** of the P-task list. *(original text below, kept for provenance)*  ⚠️ **Conditional contribution for limit-class inputs.** Extend the contribution panel to report contribution **restricted to the states where the input is active** — for `DIST_TO_BOUNDARY`, ticks below a saturation threshold. Measured 2026-08-17: it is >0.95 for **92.8%** of ticks and its near-edge std is **2.4×** its pooled std, which is precisely why the pooled number (0.050) nearly got it deleted. **This task exists because that analysis was wrong once already.**
- [X] TA03 ✅ **DONE 2026-08-17 — spiral confirmed bleeding energy; SC-014 passes decisively (corr(Ps,closure) = −0.048, 24–32 m/s spread at matched progress).** Compute `Es = h + v²/2g` and `Ps = (T−D)·V/W` from the **existing t1 dmps** and plot energy state along representative trajectories. Answers, as a READ rather than a bake: is the spiral bleeding energy into induced drag (operator's hypothesis), and does `Ps/Ps_max` separate "expensive for what it achieved" from "expensive"? ⛔ **If it cannot separate those two on recorded data, it is not ready to drive an objective (SC-014).**

### B — instrument (zero bake, but needs a rebuild; the window is open NOW)

- [X] TA04 ⛔ **SUPERSEDED** — folded into **P2-4** of the P-task list. *(original text below, kept for provenance)*  **`dmp-dump`: emit the FULL input vector** — all 42 pathgen / 63 tracker slots, named from `kPathgenInputMeta` / `kTrackerInputMeta` so columns and metadata cannot drift (FR-038). Today only 9 of 42 are emitted, which is the direct reason the boundary input was nearly trimmed on a partial picture and why contribution ranking covers 9 of 42.
- [X] TA04a ⛔ **SUPERSEDED** — folded into **P2-2** of the P-task list. *(original text below, kept for provenance)*  ⭐ **ADD AN ALTITUDE / `Es` INPUT — the missing observation.** TA03 found the 42-slot vector carries `AIRSPEED` but **no altitude/height/AGL term at all**, so `Es = h + v²/2g` is *unobservable* to the policy: it has the `v²` half and never had the `h` half. That is the mechanical explanation for why 035's energy objective muted the whole regiment — it optimised an unobservable, and the only lever available was reducing output everywhere. ⚠️ Give **`Es` (the state)**, not only `Ps` (the rate): `Es` is the integral, and making the recurrent layer accumulate it wastes capacity that is already not filled (effective rank 11.3 of 16). Both modes.
- [X] TA05 ⛔ **SUPERSEDED** — folded into **P2-4** of the P-task list. *(original text below, kept for provenance)*  **Record `Es` and `Ps` per tick** (FR-038). Derived from state already recorded; put the derivation in ONE header shared by the recorder and every reader, the `specific_force.h` pattern — reader and input must not be able to disagree.
- [X] TA06 ⛔ **SUPERSEDED** — folded into **P2-2** of the P-task list. *(original text below, kept for provenance)*  **Time-to-boundary input** (FR-035): `distance_along_vel / speed`, seconds, replacing or supplementing the saturated `tanh(d/20 m)`. ⚠️ Rate-derived ⇒ **millisecond-denominated and cadence-invariant**, with a test at two cadences (the `ENVELOPE_SECS` T040(e) pattern). Both gathers; both modes.
- [X] TA07 ⛔ **SUPERSEDED** — folded into **P2-6** of the P-task list. *(original text below, kept for provenance)*  [OP] Land B as **one commit with an `EvalResults` version bump** — same FR-005 one-break discipline as the A1 bundle. Gates: `rebuild-perf.sh` clean with the banner count verified, xiao host compile, Constitution VI audit on touched paths.

### C — reshape (one M1 bake, and it is also the M2 source)

- [X] TA08 ⛔ **SUPERSEDED** — **DROPPED**: boundary shaping is not built; arena exit is a terminal fail for safety and the objective is unchanged. *(original text below, kept for provenance)*  **Boundary containment as potential-based shaping** (FR-034, FR-036): `F(s,s') = γ·Φ(s') − Φ(s)` with Φ a function of time-to-boundary. ⚠️ **Potential-based specifically** — an ad-hoc "stay away from the edge" penalty is the exact shape that creates a centre-hugging attractor, and the tight spiral already proves this system finds unintended attractors. Keep the terminal egress crash as the backstop; the shaping is what gives the policy something to act on *before* it.
- [X] TA09 ⛔ **SUPERSEDED** — folded into **P2-2** of the P-task list. *(original text below, kept for provenance)*  **Track-score gradient input** (FR-039): ∂score/∂position in body frame, from `FitnessComputer::decomposeStepScore`'s Lorentzian (closed form), scaled by the streak multiplier. ⚠️ **Keep the binary `IN_ENVELOPE` alongside for one bake** so the ablation can attribute any change to the reshape rather than to the removal.
- [X] TA10 ⛔ **SUPERSEDED** — folded into **P2-5** of the P-task list. *(original text below, kept for provenance)*  **Step-wise cost against `Ps_max(state)`** (FR-037) — state-conditioned, never absolute. ⛔ Gated on TA03 showing the discrimination exists.
- [X] TA11 ⛔ **SUPERSEDED** — folded into **P2-2** of the P-task list. *(original text below, kept for provenance)*  Apply TA02's verdict + TA01's ablation: drop what both agree is inert. ⚠️ **TA01 LANDED AND REVERSED THIS.** `ACCEL_Y` is **RETAINED** — it costs **−4.5% on path 5**, more than all three accel axes together, and had the *lowest* contribution score; the proposal to drop it was wrong. The removal candidates are now `IN_ENVELOPE` and `ENVELOPE_SECS` (**+0.3% / −0.2% on path 5** — removing the flag is *better*), pending whether TA09's reshape is a better use of those slots than deletion. ⛔ `DIST_TO_BOUNDARY` and `INWARD_BODY_*` are **RETAINED** by FR-033 regardless of pooled contribution.
- [X] TA12 ⛔ **SUPERSEDED** — folded into **P4-1** of the P-task list. *(original text below, kept for provenance)*  [OP] Pre-run gate, then the M1 bake. Judge on `pctInStreak` / `avgMaxStreak`, and on SC-013: egress not worse than t1's 6/294, with time-to-boundary minima **rising** — i.e. turning before the edge rather than being terminated at it.

### D — per-tick advantage (later; see specs/BACKLOG.md → 042 candidate)

- [X] TA13 ⛔ **SUPERSEDED** — folded into **043 (per-tick advantage / critic)** of the P-task list. *(original text below, kept for provenance)*  Critic/value head reusing 041's extra-head machinery + the recorded per-step reward, giving `A(s,a) = Q − V`. ⚠️ The point of the learned baseline is that **the optimum never has to be known** — `V(s)` only has to RANK actions. `Ps/Ps_max` is the hand-built baseline; do it first so we know what the learned one must beat.

### B′ — datum unification, proven end-to-end (see [toolchain-datum-validation.md](toolchain-datum-validation.md))

⛔ **Belongs INSIDE the TA04–TA07 format break.** Moving the frame is fitness-affecting and orphans every
recorded dmp; done separately it costs a second re-bake. Operator 2026-08-17: *"is prob a good time to do
this."*

- [X] TD01 ⛔ **SUPERSEDED** — folded into **P0-1** of the P-task list. *(original text below, kept for provenance)*  ⚠️ **MEASURE every altitude datum in the chain before changing any of them.** Eleven hops, at
  least four distinct references (scenery ground, `<launch altitude="82">`, the −25 m virtual mid-band
  origin, the arm point, the engage point). ⛔ **The 82-vs-25 question is open and must be answered by
  measurement, not inspection**: it is not established what `autoc_config.xml`'s launch altitude is
  relative to, nor that it reconciles with `SIM_INITIAL_ALTITUDE`. Produce the filled-in table in
  toolchain-datum-validation.md, replacing every UNVERIFIED cell with a measured number.
- [X] TD02 ⛔ **SUPERSEDED** — **DROPPED**: the virtual origin is NOT being moved (operator 2026-08-18). *(original text below, kept for provenance)*  Move the virtual origin from mid-band to the **arena floor (hard deck)**: `SIM_INITIAL_ALTITUDE`
  and every consumer (`arena.h` ×6, `scenario_stepper.h`, `inputdev_autoc.cpp` `pathOriginOffset`, the
  renderer's add-back, the arena tests). ⚠️ **`z = 0` becomes a defined physical surface**, so `h_hd = −z`
  needs no conversion and `Es ≥ 0` holds by construction.
- [X] TD03 ⛔ **SUPERSEDED** — folded into **P0-4** of the P-task list. *(original text below, kept for provenance)*  Resolve the **band-placement** disagreement in the same commit (sim floor 20 m below engage vs
  flight 47.5 m) — see specs/BACKLOG.md. ⛔ Do NOT ship TD02 without deciding TD03: they touch the same
  frame, and fixing one while leaving the other creates a *new* inconsistency between the definition and
  the placement.
- [X] TD04 ⛔ **SUPERSEDED** — folded into **P0-2 + P0-4** of the P-task list. *(original text below, kept for provenance)*  Adjust the crrcsim side — scenery ground, field elevation, starting HAT — so the FDM's launch
  state lands where the unified datum expects. Operator: *"we need to make sure we twiddle the offsets for
  crrcsim too."*
- [X] TD05 ⛔ **SUPERSEDED** — folded into **P3-1** of the P-task list. *(original text below, kept for provenance)*  Per-hop validation, each against something independent (table in the plan doc). ⚠️ Not "it
  compiles" — each conversion checked: launch height reads back, egress fires at the configured floor,
  playback sits at the dmp's altitude, bench height reads through MSP, logged `pos_raw` matches blackbox on
  the clock-anchor fit.
- [X] TD06 ⛔ **SUPERSEDED** — folded into **P3-2** of the P-task list. *(original text below, kept for provenance)*  ⭐ **Renderer `'a'` mode is the ACCEPTANCE TEST.** It is the one place actual flight is shown in
  world coords (`renderer.cc:2518` uses `pos_raw`), so it is the only surface where a datum error is
  *visible* rather than inferred — a craft flying underground, or a hovering arena. Everywhere else a wrong
  offset produces plausible numbers. Flown trace must sit on the arena with the ground plane at the ground.
- [X] TD07 ⛔ **SUPERSEDED** — folded into **P4-3** of the P-task list. *(original text below, kept for provenance)*  [OP] **M1 flight**, then playback, as the end-to-end proof: sim → renderer → xiao flight →
  playback → energy. ⚠️ Standing procedure: **remove the GPS before flashing INAV**, and build/flash BOTH
  targets, bench first.
- [X] TD08 ⛔ **SUPERSEDED** — folded into **P3-3** of the P-task list. *(original text below, kept for provenance)*  ⚠️ **NUMERIC energy check — the one that cannot be eyeballed.** `Es` from the flight log vs `Es`
  from a sim run in comparable states. A trace can look perfect on the arena while `Es` carries a constant
  offset, because a constant offset is invisible in a picture and fatal in an objective. Requires a number,
  not a screenshot.
