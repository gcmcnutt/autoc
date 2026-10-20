# 044 — roll honesty ladder (FR-006, research R11, tasks T022–T024)

The plant change must reproduce the flown genome's real roll behaviour **before** any baseline or smoke is taken on it — the 043 pitch
ladder pattern. **If (1) or (3) fails: stop; revise the class (centre, σ, `Cl_da` scaling) in research R1 and re-run.**

## (1) Step test — gated (T022)

Roll steps at 15 m/s, `crrcsim/models/hb1_streamer_steptest.xml`.
**Pass**: corner within ±20 % of target; steady roll rate per aileron within ±5 % of the t6 plant; rate-loop roll output unsaturated
at the fast clamp.

| `craftClP` | target corner | measured corner | p_ss / δa vs t6 | max \|servo\| | pass |
|---|---|---|---|---|---|
| −0.36 (centre) | 3 Hz | | | | |
| −0.60 (fast clamp) | 5 Hz | | | | |
| −0.18 (slow clamp) | 1.5 Hz | | | | |
| t6 plant (−0.135) | 1.1 Hz | | (reference) | | — |

## (2) Closed-loop roll FRF of t6 g752 vs the 2026-10-18 flight — recorded, not gated (T023)

Setpoint → roll rate, 1–6 Hz, departed scenarios excluded. `roll_frf_compare.py`.

## (3) Equal-environment eval of t6 g752 — gated (T024)

`EvalVariationScale = 1.0`, seed 20261018, t6 build (`043-final`) vs new plant. **Gate**: 3–5 Hz roll share higher on the new plant,
calm and all (flight: ~50 %).

| plant | 3–5 Hz roll share, all | calm | roll-rate RMS | pass |
|---|---|---|---|---|
| t6 (`043-final`) | | | | — |
| 044 new plant | | | | |
