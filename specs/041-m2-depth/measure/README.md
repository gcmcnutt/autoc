# 041 Phase 0 — measurement instruments and what they measured

Phase 0 is "settle what is known before touching anything". These are the instruments, kept in the feature
directory per the scripts-dir convention (top-level `scripts/` is cross-version utilities only).

| instrument | task | what it answers |
|---|---|---|
| [`path_altitude_extents.cc`](path_altitude_extents.cc) | P0-2 | how much vertical and radial room the M1 *targets* demand |
| [`size_score_grad_scale.py`](size_score_grad_scale.py) | P0-3 | the realized magnitude of `SCORE_GRAD_*`, from recorded ticks |

## Building / running

```sh
# path extents — links the same generator autoc uses, so this is the real geometry
g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 \
    -I build/_deps/cereal-src/include -I build/_deps/inih-src -I build/_deps/inih-src/cpp \
    specs/041-m2-depth/measure/path_altitude_extents.cc build/libautoc_common.a -lzstd -o /tmp/path_ext
/tmp/path_ext aeroStandard 6 13337        # autoc.ini
/tmp/path_ext longSequential 1 13337      # autoc-basic-m1.ini

# score-gradient sizing — needs a pathgen per-tick CSV
build/dmp-dump "s3://autoc-m1/autoc-9223370249927095135-2026-08-17T00:48:00.672Z/" \
    --gen 608 --csv-only > /tmp/t1-gen608.csv
python3 specs/041-m2-depth/measure/size_score_grad_scale.py /tmp/t1-gen608.csv
```

⚠️ **The CSV extraction above must happen BEFORE P2-4's `EvalResults` version bump**, for the same reason
T011a had to precede T044: after the bump the current binary cannot read these dmps, and the t1 tick record
is the only 42-input M1 record in existence. The extracted CSV is the durable artifact, not the dmp.

---

## P0-2 result — the arena, and the room the targets need

Full table in [`../toolchain-datum-validation.md`](../toolchain-datum-validation.md). Two findings:

1. ⛔ **The sim runs three arenas.** What kills an M1 aircraft is `checkAircraftOOB` at **70 / 7 / 120**;
   what `DIST_TO_BOUNDARY` describes is `FlightArena`'s defaults at **80 / 5 / 100**. The input's cylinder
   is 10 m wider and 20 m shorter than the one that terminates the scenario. TA01 ranked that input **3rd
   most important in the vector**.
2. ⭐ **The M1 targets never go below the entry altitude** and climb to **49.97 m above** it. Entry sits at
   the *floor* of the target envelope. A band centred on today's 25 m entry cannot contain a 75 m AGL
   target without putting its floor 25 m underground.

## P0-3 result — sizing `kScoreGradScale`

Measured on **131 127 ticks** of the pinned t1 elite (gen 608).

`score = 1/D`, `D = 1 + (d/S)² + (θ_c/C)²`. The world-frame gradient splits into two **orthogonal** parts —
radial `∇d = û` and tangential `∇θ` with `|∇θ| = 1/d` — so the norm needs only scalars the dmp carries:

```
|∇score| = (1/D²)·sqrt( (2d/S²)² + (2θ_c/(C²·d))² )        [tangential term = 0 where θ is clamped]
```

| statistic | all ticks | in-envelope ticks (`stpPt ≥ 0.5`, n = 21 143) |
|---|---:|---:|
| p05 | 0.0032 | 0.0919 |
| p50 | 0.0316 | **0.2016** |
| p95 | 0.3118 | **0.7812** |
| p99 | 0.7698 | 2.6373 |
| max | 19.0528 | 19.0528 |

### ⚠️ The distribution has a singular tail, and it is not noise

`|∇θ| = 1/d` diverges as the chase closes on the rabbit. **Every** tick above 2.0 has `d < 1.52 m`
(median 0.23 m); that is 0.27 % of ticks. The quantity is genuinely unbounded, so:

- ⛔ **a plain divide is the wrong encoding for this slot.** `ACCEL_*` gets away with `÷ kAccelScale_g`
  because physics bounds it; nothing bounds this.
- ✅ **Encode as a direction-preserving `tanh` of the norm**:
  `v_body = ĝ_body · tanh(|∇score|·mult / kScoreGradScale)`. Applying `tanh` per-component would bound the
  slot but *rotate the vector* — and the whole point of this input is that it is a **direction**.
  This also matches the existing idiom: `DIST_TO_BOUNDARY` is already `tanh(d / kDistToBoundaryScale_m)`.

### ✅ **`kScoreGradScale` = 0.78 m⁻¹**

The **in-envelope p95** of `|∇score| × streak multiplier`. Not a round number and not the pooled p95 — the
in-envelope regime is the one the input exists to serve, and pooling it with the 84 % of ticks that are
nowhere near the cone would size the constant against ticks whose gradient the policy cannot act on.

This reproduces the `kAccelScale_g` precedent exactly: at that divisor the common regime (in-envelope
median 0.2016) reads **0.26**, well inside tanh's near-linear zone; 95 % of in-envelope ticks fall below
`tanh(1) = 0.76`; and the near-field excursions saturate toward 1.0 — which is the correct message, since
"the gradient here is enormous" is all the controller needs to know.

⚠️ Sized on the **norm**, so it is conservative: no individual body-axis component can exceed it.

⚠️ The estimate carries the t1 policy's own state distribution. It is the best available basis — it is the
only 42-input M1 record there is — but it is a *policy-conditioned* measurement, not a property of the
scoring surface alone. Re-check it against the P3-4 smoke's dmp before the production bake.
