# Toolchain datum unification, proven end-to-end by an M1 flight

**Proposed 2026-08-17.** Operator: *"we make these changes and prove we got the whole toolchain right with
an m1 flight — sim, renderer, xiao flight, playback in renderer, valid energy calculations — is prob a good
time to do this."*

## Why now

Three things converged:

1. **`SPECIFIC_ENERGY` makes altitude an NN input**, so its datum stops being a display detail and becomes
   a trained-against quantity (`docs/COORDINATE_CONVENTIONS.md` → *Altitude DATUM as an INTERFACE
   quantity*).
2. **Two datum defects are already filed** — the sim/flight band **placement** disagreement (20 m vs 47.5 m
   below engage) and the virtual origin sitting **mid-band**. Both are frame-wide and fitness-affecting, so
   both are only free inside a format break.
3. **A format break is already owed** — TA04/TA05/TA06 bump `EvalResults` anyway. Doing the datum work
   outside that bundle means paying for two re-bakes instead of one.

## The chain, and every datum in it

✅ **MEASURED 2026-08-18 (P0-1 / P0-2). No UNVERIFIED cells remain.** Every number below was read out of
the code that produces it, not inferred from a comment, and the arithmetic closes to 1.2 mm. The
instrument for the path-extent half is [`measure/path_altitude_extents.cc`](measure/path_altitude_extents.cc).

⚠️ **The one surprise is not in this table** — it is that the sim runs *three different arenas* at once.
See "The arena the M1 policy is actually flying in" below.

| # | stage | quantity | datum today | ✅ measured |
|---|---|---|---|---|
| 1 | crrcsim scenery (Davis Field) | terrain height | scenery-defined ground plane | **flat, −0.1 ft = −0.03048 m**, everywhere. `BuiltinSceneryDavis::getHeight()` returns the constant `-0.1` (`crrc_builtin_scenery.cpp:709`). Units are **feet** — the same expression adds `launch.altitude` and the FDM's `zLow`, and the sibling Cape Cod scenery returns `100.1` for a cliff that is ~30 m |
| 2 | `crrcsim/autoc_config.xml` | `<launch altitude="82">` | what is 82 relative to? | **82 ft above the terrain, measured to the aircraft's LOWEST POINT.** `crrc_main.cpp:246` computes `Altitude = launch.altitude + zLow + height`, so the CG sits `zLow` higher. `zLow` = max wheel z = **0.125 ft** (`hb1_streamer.xml` `<wheels units="0">`; `units=0` ⇒ feet, and `enabled="0"` disables gear *physics* only — `wheels.init()` still runs, so `dZLow` is computed either way) |
| 3 | crrcsim FDM | `v_P_CG_Rwy` z, ft | runway/scenery frame | **CG altitude = 82 + 0.125 − 0.1 = 82.025 ft = 25.0012 m** above crrcsim's z = 0 datum ⇒ `v_P_CG_Rwy(2) = −82.025 ft` |
| 4 | autoc bridge | `pathOriginOffset = (0,0,SIM_INITIAL_ALTITUDE)`, −25 m | subtracted at `inputdev_autoc.cpp:932` to make **virtual** z | raw NED z = **−25.0012 m** (ft→m at `inputdev_autoc.cpp:900`) |
| 5 | `AircraftState::position` | virtual NED z | origin at the launch point | **virtual z at launch = −0.0012 m.** ⭐ The reconciliation P0-1 existed to settle: `<launch altitude="82">` and `SIM_INITIAL_ALTITUDE = −25` agree **to 1.2 mm (0.005%)**. There was never a discrepancy — only an unstated unit (crrcsim is foot-native) and an unstated `zLow` |
| 6 | `checkArenaBounds` | `alt_agl = −(z + SIM_INITIAL_ALTITUDE)` | "ground-referenced" | ⚠️ **it is DATUM-referenced, not ground-referenced.** It returns height above crrcsim's z = 0, and the Davis ground plane sits **0.03048 m BELOW** that. So autoc's `alt_agl` **under-reads true AGL by 30.5 mm**, uniformly. Immaterial at a 5 m floor; recorded because an unlabelled 3 cm bias is exactly the sort of thing that later gets attributed to something else |
| 7 | dmp → renderer | display z | renderer **adds `SIM_INITIAL_ALTITUDE` back** | inverse of (4); exact, same constant (`renderer.cc:455/467/2199`) |
| 8 | INAV → xiao | raw NED from home/arm | **arm point**, not ground | unchanged — and per the 2026-08-18 operator call, **INAV is the source of truth** for position/altitude in these phases (no drift budget, no altitude fusion) |
| 9 | `resolveEngageArena` | `floor_z_ned = z_engage + K` | **engage-centred** | K = (ceiling_agl − floor_agl)/2 = **47.5 m** from the baked 80/5/100 template ⇒ flight engages at **exactly 50% of the band**. The sim does not — see below |
| 10 | flight log | `pos_raw` | raw NED from arm | unchanged |
| 11 | renderer **'a' mode** | world coords on the arena | ⭐ **the validation surface — see below** | P3-2 |
| 12 | `SPECIFIC_ENERGY` (new) | `h_hd + v²/2g` | height above the **arena floor** | ⚠️ blocked on the arena decision below — `h_hd` is only well defined once one floor exists |

### ⚠️ The arena the M1 policy is actually flying in — THREE geometries, not one

P0-2's real finding. "Sim vs flight" was the wrong axis; the sim disagrees with **itself**.

| | radius | floor AGL | ceiling AGL | band | entry, as % up the band |
|---|---:|---:|---:|---:|---:|
| **M1 sim — what KILLS you** (`checkAircraftOOB`, `scenario_stepper.h:65`) | **70** | **7** | **120** | 113 | 15.9% |
| **M1 sim — what the NN is TOLD** (`FlightArena` defaults; `autoc.ini` sets no `FlightArena*` key) | **80** | **5** | **100** | 95 | 21.1% |
| **M2 sim** (`autoc-tracker.ini`) | 80 | 5 | 100 | 95 | 21.1% |
| **flight** (`resolveEngageArena` off the baked 80/5/100 template) | 80 | engage−47.5 | engage+47.5 | 95 | **50.0%** |

⛔ **`DIST_TO_BOUNDARY` describes a cylinder the M1 aircraft cannot hit and does not describe the one that
kills it.** The input's cylinder is 10 m WIDER (80 vs 70) and 20 m SHORTER (100 vs 120) than the
termination envelope. The TA01 ablation ranked `DIST_TO_BOUNDARY` the **3rd most important input in the
vector** — so the policy leans hard on a boundary signal that is wrong in both directions. This is
independent of the entry-placement question and is a defect either way.

### ✅ MEASURED — what vertical room the M1 targets actually demand

`measure/path_altitude_extents.cc`, run against the configured generators (`aeroStandard`, 6 paths,
`RandomPathSeedB = 13337`):

| path | agl_min | agl_max | radius_max |
|---|---:|---:|---:|
| 0 StraightAndLevel | 25.00 | 25.00 | 48.28 |
| 1 SpiralClimb | 25.00 | **74.98** | 48.28 |
| 2 HorizontalFigureEight | 25.00 | 25.00 | 43.74 |
| 3 FortyFiveDegreeAngledLoop | 25.00 | 46.21 | 21.21 |
| 4 HighPerchSplitS | 25.00 | 64.85 | **57.14** |
| 5 SeededRandomB | 25.25 | 65.69 | 41.80 |
| **all** | **25.00** | **74.98** | **57.14** |

⭐ **The targets never descend below the entry altitude, and climb up to 49.97 m above it.** Entry is at the
**floor** of the target envelope, not its middle. That is the fact the "entry at the 3D centre of the
cylinder" decision has to be reconciled with: a band centred on a 25 m entry cannot reach 75 m AGL without
its floor going 25 m underground.

(`autoc-basic-m1.ini`'s `longSequential` single path is flat at 25.00 m AGL, radius_max 43.85 m — so the
P3-4 smoke exercises none of the vertical question.)

## ✅ DECIDED 2026-08-18 (operator) — ONE arena, RELATIVE, entry at its centre

Settled across four operator messages, each of which moved it:

1. *"Raise sim frame… We do want Xiao arm at virtual origin. Pilot responsible for that being way above
   terrain so that bottom of cyl is above ground. So really sim should do identical."*
2. *"Should be the same for both. 70m radius. Hat of perhaps 10 (bottom of cyl) top at 100m above bottom."*
3. *"we really should not exceed 400 AGL at our test site — so if possible we want something like 25m HAT
   for the floor… the flight area origin is at 0 and the radius from there is constant AND the top bottom
   are +/- meters — operator responsible for being above ground at all times."*
4. ⛔ *"if operator arms a path at 100m then it is on them to not go above 400ft… Not the models problem.
   Is operator."*

5. ⭐ *"Maybe we should go back to non uniform vertical extent for arena. 60m up and 10 down?"*
6. ⭐ *"I like the hard deck. Chase cannot swing low and will eventually be tricked by target moving close
   to the line (top gun episode 1)…"*

### The one geometry

| quantity | value |
|---|---:|
| radius R | **70 m** |
| up-extent | **+60 m** above the arm point |
| down-extent | **−10 m** below it |
| arm point — the virtual origin | 10 m above the deck, **NOT** the band's centre |
| where that lands IN SIM | floor **25** / arm **35** / ceiling **95** m AGL (312 ft) |
| `SIM_INITIAL_ALTITUDE` | **−35 m** (was −25) |
| `<launch altitude>` | **114.804 ft** (was 82) |

### ⭐ Why asymmetric, and why the deck is a FEATURE

Measured from the cylinder centre, the M1 rabbit climbs **34.98 m** and descends **2.74 m**. A symmetric ±K
band therefore spent half its height on airspace nothing ever enters, while being tight at the top. 60/10
matches the shape of the actual flight: the rabbit uses **58%** of the up-extent and **27%** of the down.

⚠️ **The deck is not merely a safety bound — it is a tactical element**, and that changes how a floor egress
must be READ. Operator: *"Chase cannot swing low and will eventually be tricked by target moving close to the
line."* A pursuer denied the airspace below its target cannot use altitude as an energy reservoir, and a
target flying near the deck can bait it into the ground. That is what a hard deck does in air combat.

⛔ **So `ArenaEgressKind::FLOOR` has two opposite meanings** — the deck is too tight, or the policy was drawn
down onto it — and widening the deck in response to the second one would remove the lesson. Distinguishing
them requires knowing where the RABBIT was at the time, which is why P2-4 now emits **`rbX/rbY/rbZ/rbHhd`**
(the rabbit's position and its own height above the deck) for pathgen. That data was already serialized on
every recorded state and had simply never been readable.

⚠️ **Expect the chase to meet both bounds.** t1 measured **+66.2 m** above and **17.9 m below** the arm point
— against +60 / −10 — and the below figure was itself floor-limited, so the policy wanted more. Both are
intended pressure (the `Ps` energy axis charges for the zoom independently), and both are watched at the
P3-4 smoke rather than asserted.

⭐ **THE ARENA IS RELATIVE, NOT ABSOLUTE**: radius R about the arm origin, floor and ceiling at ∓K. The AGL
numbers are only where that band sits in sim, where the ground is known. Keeping it above terrain and inside
the site's 400 ft working envelope are **both operator responsibilities exercised at arm time** — nothing in
the code enforces an absolute altitude, and nothing should.

⭐ `resolveEngageArena(...).virtual_arena` comes out **identical** to the training `FlightArena`, at any
engage altitude and any asymmetry. Sim and flight stop agreeing by convention and start agreeing by
construction.

⛔ **The asymmetry broke the old placement rule, silently.** `resolveEngageArena` computed a single half-band
`K = (ceiling − floor)/2` and placed the band at ±K — correct only when the arm point is the vertical centre.
Under +60/−10 that yields **±35 m**: 25 m less room above than the policy trained with and 25 m more below,
in flight, with every logged number looking reasonable. It now derives the two extents separately, and
`arena_recenter_tests` asserts the placement is *not* half-band symmetric so the old shape cannot return
quietly.

### ⚠️ Two training-content changes this forced, both measured

The max-extent cross-check (operator: *"make sure the pathgen paths range fit in the arena with room to
spare. I think we are ok but a max extent cross check is a good idea"*) found we were **not** ok. Everything
below is measured from the generator by [`tests/arena_path_fit_tests.cc`](../../tests/arena_path_fit_tests.cc),
every build — not transcribed.

| path / bound | was | now | ceiling margin |
|---|---:|---:|---:|
| SpiralClimb climb | 50 m (realized **+49.97**) | **35 m** (+34.98) | — |
| HighPerchSplitS climb | 20 + 20 m (realized **+39.85**) | **15 + 15 m** (+34.98) | **13.0 m** |
| seeded path vertical draw | `SIM_PATH_BOUNDS` 40 (realized **+45.0**) | **`SIM_PATH_HEIGHT_BOUNDS` 30** (+33.75) | **15.5 m** |
| widest path (HighPerchSplitS) | 57.14 m radius | unchanged | **12.9 m** radial |

⛔ **Three separate near-misses, none visible without measuring**:
* SpiralClimb cleared the first proposed ceiling by **3 cm** — a coincidence, not a clearance, and the chase
  flies *above* the rabbit.
* The moment SpiralClimb stopped binding, **HighPerchSplitS** took over at 8.2 m, then the **seeded** path at
  3.0 m. Fixing one just promotes the next.
* ⛔ **The xiao generated a DIFFERENT random rabbit from the sim** — `height=100` in
  `embedded_pathgen_selector.h` against the desktop's 40, an analytic envelope of entry **+112.5 m** vs
  **+45 m**. A live sim/flight divergence in the target itself. Both sides now use the shared constants.

### ⚠️ The chase does NOT fit, knowingly

Measured on 131 127 t1 ticks the chase reached **entry +66.2 m** against K = 48. Roughly **1 % of t1-like
ticks would now egress at the ceiling**. Accepted rather than designed around: a binding ceiling is pressure
against the zoom that the new `Ps` energy axis charges for anyway, and the chase's extent is a property of
the policy, not of the generator. ⛔ **First thing to look at in the P3-4 smoke**: `ArenaEgressKind::CEILING`
counts. If the run bleeds scenarios there, **K is the knob, not the objective**.

### ✅ Acceptance criteria (operator 2026-08-18)

> *"What I expect to see is a m1 train and a renderer that make sense. And a visual eval in CRRCSim that
> shows relative to camera the sensible locations. And of course real m1 flight tracks as today and its
> ground playback has both virtual origin centering above the checkerboard and the absolute playback shows
> real flight from launch time. Same look as today with improved coordinate transforms."*

| # | acceptance item | where it is closed |
|---|---|---|
| 1 | M1 train that makes sense | P3-4 smoke, then P4-1 |
| 2 | renderer that makes sense | P3-2 |
| 3 | CRRCSim **visual eval** — sensible locations relative to camera | P3-2 (run `scripts/crrcsim-visual.sh`, not headless) |
| 4 | real M1 flight tracks as today | P4-3 |
| 5 | ground playback: **virtual-origin centring above the checkerboard** | P3-2 — the whole scene now sits 48 m higher over the ground plane; the renderer adds `SIM_INITIAL_ALTITUDE` back at 18 sites and none of them hard-codes the old value |
| 6 | ground playback: **absolute playback from launch time** | P3-2 / P4-3 — renderer `'a'` mode, `pos_raw` |

⚠️ Item 5 is the one the datum work actually changed, and it is a **visual** check by design: a wrong offset
produces plausible NUMBERS everywhere else and a craft flying underground here.

**Eleven hops, at least four distinct references** (scenery ground, launch, virtual mid-band origin, arm
point, engage point). Every conversion is a place to be wrong, and today only some of them are written down.

## ⭐ The 'a' key is the validation surface

Operator: *"in real playback mode in renderer we can hit the 'a' key and see the actual flight displayed
from the ground (e.g. on the arena) — this is the one place where actual flight is shown in world coords."*

That makes it the **only** place a datum error becomes *visible* rather than inferred. `renderer.cc:2518`
uses `pos_raw` for exactly this reason. Everywhere else a wrong offset produces plausible numbers; here it
produces a craft flying underground, or a hovering arena, and you see it immediately.

**So 'a' mode is the acceptance test, not a nice-to-have.** If the flown trace sits correctly on the arena
with the ground plane where the ground is, the chain is right end to end.

## ⚠️ TARGET REVISED 2026-08-18 — geometry, not the origin

**The origin move proposed below was NOT adopted.** Operator: *"Arena should be same geometry. Same radius.
Same height. Xiao trigger should be halfway up the cylinder. And sim should be similar. Now maybe we should
keep it as is because in the general sense z sign should never matter… So revisit the need to change arena
origin."*

**Adopted scope instead:**
1. **Same arena geometry both sides** — radius and height. Fix the *sizes* if they differ.
2. **Engage mid-cylinder in the sim**, as it already is in flight. Today the sim engages 21% up its band
   (25 m AGL in a 5–100 m band), which is the real asymmetry.
3. **Virtual origin unchanged.** z sign should not matter to any consumer, and a frame move is a large risky
   change to buy what (1)+(2) already deliver.
4. `Es` still measured as **height above the configured floor** — a *computed* quantity, so a varying deck
   or an outside-in entry does not invalidate the frame.

⚠️ Do not anchor the frame to today's deck: *"hard deck or ground will eventually vary"*, and entry from
outside the arena is *"quite plausible"*. Today's mid-band entry is a flight-safety convention, not physics.

### (superseded) Original proposal — move the origin to the hard deck

Move the virtual origin from mid-band to the **arena floor**, and make every stage express height as
"metres above the hard deck":

- `z = 0` becomes a *defined physical surface* rather than "wherever a scenario started";
- altitude inputs need **no conversion** — `h_hd` is just `−z`;
- **`Es ≥ 0` by construction**, not by observation;
- sim and flight agree on the *definition*, and the placement decision (backlog) becomes the only remaining
  disagreement, in one place instead of scattered.

⚠️ **Not free**: it moves the frame every recorded dmp is expressed in, so every reader shifts together and
old dmps are orphaned. That is precisely why it belongs in the TA04–TA07 format break rather than after it.

## Validation plan — each hop proven, then the whole thing flown

The point is not "the code compiles"; it is **each conversion checked against something independent**.

| stage | what proves it |
|---|---|
| crrcsim → autoc | craft launched at a known height reads that height in `AircraftState` |
| autoc → arena | egress fires at exactly the configured floor/ceiling, verified by a scripted approach |
| autoc → dmp → renderer | a sim run played back sits on the arena at the altitude the dmp says |
| INAV → xiao | a known bench height reads back correctly through `MSP2_AUTOC_STATE` |
| xiao → flight log | logged `pos_raw` matches the INAV blackbox for the same instant (per-flight clock-anchor fit, standing practice) |
| **flight log → renderer 'a'** | ⭐ **the flown trace sits on the arena, ground plane at the ground** |
| **energy** | `Es` from the flight log matches `Es` from a sim run in comparable states — the number that has to be right for the objective to mean anything |

⚠️ **The energy check is the one that cannot be eyeballed.** A trace can look right on the arena while `Es`
is offset by a constant, because a constant offset is invisible in a picture and fatal in an objective. It
needs a numeric comparison, not a screenshot.
