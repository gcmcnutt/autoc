# Contract: Sim data.dat

**Reader role**: `cmd_response_scatter_sim_lagged.py`, `gyro_vs_quat_sim.py`,
`sensor_self_check.py` WI2 (sim conformance), `sim_polar_viz.py`, renderer.
**Producer**: `autoc.cc` main loop, row emitted per `stepState` of each
simulation / eval.

## Format

Whitespace-separated text with header on line 1. One row per NN evaluation
tick. Sample cadence driven by CRRCSim physics frame boundary + eval
threshold in `inputdev_autoc.cpp:341`.

**Current cadence (pre-WI4)**: 117 ms per row (frame-boundary quantized).
**Post-WI4 cadence**: 100 ms exact per row (accumulator time budget).

## Columns (in order)

```
Scn  Bake  Pth/Wnd:Step:  Time  Idx  tgX-9 tgX-3 tgX-1 tgX0 tgX+1 tgX+5
  tgY-9 tgY-3 tgY-1 tgY0 tgY+1 tgY+5  tgZ-9 tgZ-3 tgZ-1 tgZ0 tgZ+1 tgZ+5
  ds-9 ds-3 ds-1 ds0 ds+1 ds+5  dd/dt  qw qx qy qz  vel gyrP gyrQ gyrR
  outPt outRl outTh  pathX pathY pathZ  X Y Z  vxBdy vyBdy vzBdy
  dhome dist along rabVl stpPt mult rampSc
```

| Column | Units | Frame | Canonical |
|---|---|---|---|
| `Scn` | int | — | scenario counter (generation indexing) |
| `Bake` | int | — | bakeoff counter |
| `Pth/Wnd:Step:` | `PPP/WW:SSSS:` | — | path / wind-variant / step |
| `Time` | ms | — | `simTimeMsec` |
| `Idx` | int | — | NN run index |
| `tgX-9`..`tgX+5` | unit | body FRD | TARGET direction cosines, x-component, 6 history |
| `tgY-9`..`tgY+5` | unit | body FRD | y-component history |
| `tgZ-9`..`tgZ+5` | unit | body FRD | z-component history |
| `ds-9`..`ds+5` | m | — | distance history |
| `dd/dt` | m/s | — | closing rate |
| `qw qx qy qz` | unit quat | q_EB aerospace | QUAT |
| `vel` | m/s | — | airspeed (groundspeed proxy) |
| `gyrP gyrQ gyrR` | rad/s | aerospace RHR body | GYRO |
| `outPt outRl outTh` | [−1,+1] | — | CMD |
| `pathX pathY pathZ` | m | NED virtual | path point aircraft is tracking |
| `X Y Z` | m | NED virtual | aircraft position |
| `vxBdy vyBdy vzBdy` | m/s | **body frame** | aircraft velocity in body frame |
| `dhome` | m | — | distance from virtual origin |
| `dist` | m | — | distance to rabbit (duplicate of `ds0`) |
| `along` | m | — | along-path progress |
| `rabVl` | m/s | — | rabbit speed |
| `stpPt`, `mult`, `rampSc` | mixed | — | fitness accounting internals |

## Notes on frames

- `X Y Z` is **virtual NED meters** (raw minus `pathOriginOffset`).
- `vxBdy vyBdy vzBdy` is **body-frame velocity**, not world. To get world:
  `vel_world = q_EB.inverse() × vel_body` — matches evaluator's convention.
- All quaternion, gyro, and direction-cosine columns are already in our
  canonical stack convention. No transforms needed at read boundary.

## Header parsing

Header is whitespace-separated with multi-token names (`Pth/Wnd:Step:`). Reader
must split on whitespace and not assume single-token names. Column ordering
matches the `sprintf` format string in `src/autoc.cc:658` — can re-derive by
reading that file if parsing is uncertain.

## Cadence

- **Pre-WI4**: median `Time` delta = 117 ms (not 100 ms as intended). Root
  cause in research.md §3.
- **Post-WI4**: median `Time` delta = 100 ms exactly. Verification command:
  `awk 'NR>1 {print $4}' data.dat | uniq -c`.
- Per-scenario: Time starts at some offset (first row usually Time=117 in
  the pre-fix state); successive rows increment by the same delta.

## Scenario bucketing

Readers that show multiple scenarios should split on the `Pth/Wnd:Step:`
column's path/wind portion (everything before the first `:`), e.g.
`000/13:0090:` → scenario = `000/13`. Within a scenario, rows are sorted by
`Time`.

## File naming / location

- Training runs: root or `eval-results/<timestamp>/tier*/data.dat`.
- For archival analysis: `/home/gmcnutt/autoc/test4-data.dat` (6.3 GB, last
  big training run — retained per WI12 archive policy).

## Error modes

- **Missing required column** → reader fails loud with the missing name.
- **Time non-monotonic within scenario** → reader warns (shouldn't happen but
  possible if a scenario restart is ill-formed).

## Testing

- Smoke: read header, verify all required columns present.
- Content: for `/tmp/gen400_p0_p2.dat` (gen 400, path 0+2, var 0 from
  test4-data.dat), 350 rows across 2 scenarios. Cadence: 117 ms (pre-WI4).
- Contract cross-check: `gyro_vs_quat_sim.py` on same slice produces
  slope ≈ 1.0 on all 3 axes, r > 0.7 — sim is its own identity reference.
