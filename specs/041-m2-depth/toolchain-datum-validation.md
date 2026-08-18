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

⚠️ **This table is the deliverable of task TD01, not an established fact.** Several cells are marked
UNVERIFIED because the numbers do not obviously reconcile and must be *measured*, not assumed.

| # | stage | quantity | datum today |
|---|---|---|---|
| 1 | crrcsim scenery (Davis Field) | terrain height | scenery-defined ground plane — **UNVERIFIED** |
| 2 | `crrcsim/autoc_config.xml` | `<launch altitude="82">` | **UNVERIFIED** what 82 is relative to; it is not obviously the same reference as (4) |
| 3 | crrcsim FDM | `v_P_CG_Rwy` z, ft | runway/scenery frame |
| 4 | autoc bridge | `pathOriginOffset = (0,0,SIM_INITIAL_ALTITUDE)`, −25 m | subtracted at `inputdev_autoc.cpp:932` to make **virtual** z |
| 5 | `AircraftState::position` | virtual NED z | **origin 25 m AGL, mid-band** |
| 6 | `checkArenaBounds` | `alt_agl = −(z + SIM_INITIAL_ALTITUDE)` | ground-referenced |
| 7 | dmp → renderer | display z | renderer **adds `SIM_INITIAL_ALTITUDE` back** |
| 8 | INAV → xiao | raw NED from home/arm | **arm point**, not ground |
| 9 | `resolveEngageArena` | `floor_z_ned = z_engage + K` | **engage-centred** — the placement gap |
| 10 | flight log | `pos_raw` | raw NED from arm |
| 11 | renderer **'a' mode** | world coords on the arena | ⭐ **the validation surface — see below** |
| 12 | `SPECIFIC_ENERGY` (new) | `h_hd + v²/2g` | height above the **arena floor** |

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

## Proposed target: one datum, the hard deck

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
