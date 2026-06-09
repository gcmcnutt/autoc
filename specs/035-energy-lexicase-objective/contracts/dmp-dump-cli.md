# Contract — `dmp-dump` CLI (FR-P02)

A tool that reads a cereal `EvalResults` dmp (zstd or legacy plain) and emits a
human/Python-parseable text representation. Primary input is S3; local is a dev convenience.

## Invocation
```
dmp-dump <s3-uri | local-path> [--gen N] [--scenarios a,b,c] [--meta-only] [--csv-only]
```
- `<s3-uri>`: `s3://<bucket>/<run-id>/gen<N>.dmp.zst` (or `.dmp`). Bucket from URI, not ini.
- `<local-path>`: a `.dmp[.zst]` file.
- `--gen N`: when given an S3 *run prefix* (no gen), select gen N; else inferred from key.
- `--meta-only` / `--csv-only`: emit just one block.

## Output (stdout)
Two segregated blocks:

1. **YAML metadata** (first):
```yaml
run:
  bucket: autoc-m2
  key: autoc-<id>/gen9410.dmp.zst
  gen: 590                 # 10000 - 9410
  mode: tracker
  gp_hash: <hex>
  scenario_count: 294
scenarios:
  - idx: 0
    seeds: {path: ..., wind: ..., craft: ..., rabbit: ...}
    crash_reason: NONE
    score: -158.2
    energy_score: 12.7
    max_streak: 41
  - ...
arena: {...}      # tracker-mode camera/beacon/hull config when present
```
2. **CSV per-tick** (after a `\n---\n` separator):
```
scenario,tick,px,py,pz,qw,qx,qy,qz,vx,vy,vz,pitchCmd,rollCmd,thrCmd,out_pt,out_rl,out_th,dhome,dist,along,stpPt,mult,rampSc[,hull]
0,0,...
```
- Columns 1:1 with `AircraftState` per-tick fields **plus recomputed derived** columns
  (`dhome,dist,along,stpPt,mult,rampSc`; `hull` only in tracker mode).
- Derived columns recomputed in-tool via shared `autoc_common` math (no drift).

## Guarantees
- **Round-trip:** decompressed cereal blob is byte-identical to the pre-compression blob
  (unit-tested). CSV column set is stable and documented in the tool `--help`.
- **Fail-loud:** missing S3 key / unreadable file / version-mismatch → non-zero exit + clear
  stderr (Principle V/VII). Never emits partial-but-plausible output on a load failure.
- **`.zst` transparency:** `.zst` keys auto-inflate; legacy `.dmp` accepted.

## Consumers
`specs/03[2-5]*/*.py` plot scripts invoke via subprocess/pipe:
`dmp-dump s3://autoc-m1/<run>/gen<N>.dmp.zst --csv-only | python3 plot_per_axis_time_series.py`
(FR-P03 — scripts rewritten to the new column names; no data.dat byte-compat required).
