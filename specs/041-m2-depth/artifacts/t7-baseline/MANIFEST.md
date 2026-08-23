# 041-t7 — M1 BASELINE OF RECORD, reproducibility manifest

**Run complete 2026-08-23 07:28:59, gen 800/800. Final best fitness −81,413.**
Pinned `retain=keep` in S3 (P4-2).

## The run

| | |
|---|---|
| bucket / prefix | `s3://autoc-m1/autoc-9223370249590214474-2026-08-20T22:22:41.333Z/` |
| objects / size | 800 dmps, 33.2 GB |
| master seed | `1787264561` |
| final generation | **800** (file `gen9200.dmp.zst` — filenames encode 10000 − gen) |
| flight-baked genome | **gen 633** (`gen9367`), fitness −77,699 |

## Binary provenance

| | |
|---|---|
| autoc commit | `59a83e1a70feb2c1bd685b252d721c5b4a1ca64a` |
| crrcsim pointer | `c85e5d69eefacb9626cd06852a275eb0d2042d81` |
| build | clean `scripts/rebuild-perf.sh`, 48/48 suites |
| ini as run | [`autoc.ini.as-run`](autoc.ini.as-run) |

## ⛔ INPUT SCALE CONSTANTS — THE GENOME IS UNUSABLE WITHOUT THESE

041 P2-8 changed input SCALES while the layout stayed `float[45]`. A genome
restored against different constants **loads cleanly and flies wrong**, and the
`nn2cpp` assert checks the input COUNT only — it cannot catch this (that is what
open task P2-9 is for).

| constant | value |
|---|---|
| `kCruiseSpeed_mps` | `13.0` |
| `kTargetDistScale_m` | `26.0` |
| `kClosingRateScale_mps` | `16.0` |
| `kGyroScale_radps` | `6.0` |
| `kAccelScale_g` | `8.0` |
| `kEnergyScale_m` | `145.0` |
| `kScoreGradScale` | `0.78` |
| `kDistToBoundaryScale_m` | `20.0` |

## Reproducing

⚠️ Bit-replay is only valid **within this build** — cross-version seed reuse
produces a different run by design (PRNG cousin of the no-cereal-versioning
rule). To replay: check out the commit above, `rebuild-perf.sh`, and set
`Seed=1787264561` in an eval-mode ini.

## Result summary

| | 041-t7 | prior baseline 038-t5 |
|---|---:|---:|
| best fitness | **−81,413** | −45,254 |
| vs all-time record (034-origm1 −55,270) | **+47%** | — |
| throttle pegged (>0.99) | 21.3% @ g699 | 37.7% |
| load p50 / p95 | 1.45 / 4.26 g | 1.92 / 5.21 g |
| ticks ≥ 8 g | 0.036% | 0.118% |

⭐ The run finished **on an improvement** (−78,996 at g790 → −81,413 at g799) at
full variation scale 1.0 — it had not converged.
