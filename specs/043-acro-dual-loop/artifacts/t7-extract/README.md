# 041-t7 extract (043 T004 · FR-057) — ⛔ the irreversible-if-skipped gate

**Why this exists**: 043 T011 changes `ScenarioMetadata`'s cereal wire format (new craft/IMU axes),
which orphans **every dmp ever written** — including the `retain=keep` 041-t7 baseline that every 043
comparison is measured against. Same failure mode as 041's T011a. This directory is that baseline,
extracted **before** the break, into a format (plain CSV/YAML) that does not depend on the dmp loader.

Extracted 2026-08-25 with the pre-break `build/dmp-dump`, from the pinned source of record:

```
s3://autoc-m1/autoc-9223370249590214474-2026-08-20T22:22:41.333Z/
```

(800/800 objects `retain=keep`; see specs/041-m2-depth/artifacts/t7-baseline/MANIFEST.md for the
scale constants — without them the genome loads clean and flies wrong.)

## Contents

| file | what | how |
|---|---|---|
| `t7-gen800-per-tick.csv.gz` | per-tick sim trace of the **baseline-of-record** elite (gen 800, best fitness −81,413), all 294 scenarios | `dmp-dump <s3> --gen 800 --csv-only --physics` |
| `t7-gen633-per-tick.csv.gz` | per-tick sim trace of the **flown** genome (gen 633, flown 2026-08-23) | `dmp-dump <s3> --gen 633 --csv-only --physics` |
| `t7-gen800-meta.yaml` | full per-scenario metadata + scores for gen 800 (294 scenarios: seeds, scores, streaks, crash reasons) | `dmp-dump <s3> --gen 800 --meta-only` |
| `t7-gen633-meta.yaml` | same for the flown gen 633 | `dmp-dump <s3> --gen 633 --meta-only` |
| `t7-gen800-runsummary.csv` | one authoritative per-gen aggregate row for gen 800 (shared-code `computePerPathRates`/`computeAggr`) | `dmp-dump <s3> --run-summary --since-gen 800` |
| `verify_extract.py` | T005 independent verification (reads only the CSV, no dmp loader) | — |
| `VERIFY.md` | T005 verification results | — |

### CSV columns (per-tick)

`scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,wN,wE,wD,
dist,along,stpPt,mult,rampSc,rbX,rbY,rbZ,rbHhd,tg{X,Y,Z}{-5..0},ds{-5..0},dd/dt,qw,qx,qy,qz,vel,
gyr{P,Q,R},ac{X,Y,Z},Es,bClR,dBnd,in{X,Y,Z},sg{X,Y,Z},Es_m,Ps_mps,bClR_ms,sg{x,y,z},phyMs,
acc{X,Y,Z},odb{P,Q,R},alpha,vRelWind,sf{x,y,z}_g,nz_g`

Orientation quaternion for attitude/rate work = the **first** `qw,qx,qy,qz` (positions 5–8). Tick dt =
`SIM_TIME_STEP_MSEC` = **50 ms** (20 Hz recorded cadence).

## ⚠️ Scope of the extract — what was deliberately NOT pulled, and why

- **Only gen 800 and gen 633 per-tick traces**, not all 800 gens. The full run is ~800 × 41 MB ≈ 33 GB.
  No 043 comparison phase needs t7's per-generation *evolution history* — they compare the **new** flight
  against t7's **final** behavior (gen 800 = baseline of record; gen 633 = what actually flew). The
  evolution history also already survives as the 041 report PNGs on disk (`specs/041-m2-depth/*.png`).
- **The `--physics` FDM columns (`gyrP/Q/R`, `acX/Y/Z`, `sfx_g`…) populate only the first ~200 ms of each
  scenario** — the FDM caps its physics trace at `MAX_TRACE_STEPS` (`fdm_larcsim.cpp`). This is expected,
  **not** a join failure (`dmp-dump` prints the note itself). Body rotation rates for the full trajectory
  are therefore derived from the **quaternion** trace (present on every tick), which is exactly how
  `dmp_dump.cc::computePerPathRates` and the outcome.md rate figures were computed. See VERIFY.md.
- **The flight-side band-power reference (30.1% / 7.4%, 2.39× / 3.22×) is NOT in the dmps** — it comes from
  the flight blackbox in `flight-results/flight-2026*/…csv`, which is already on disk and not at risk from
  the wire break. This extract preserves the **sim** side.
