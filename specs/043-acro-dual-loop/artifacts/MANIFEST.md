# 043-t2 — M1 bake MANIFEST (T068, FR-061, Constitution VIII)

**Purpose**: reproduce the exact genome flown in T070. ⛔ A genome without its **input scale constants**
loads clean and flies wrong — that is why they are transcribed below rather than referenced.

Written 2026-09-04. Facts verified against S3 and git at that date; each row says how it was checked.

---

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370248704297747-2026-08-31T04:27:58.060Z/` |
| Retention | ✅ `retain=keep` — **verified 2026-09-04**, 800 objects, sampled `gen9999` / `gen9900` / `gen9500` / `gen9201` / `gen9200`. The 30-day `retain=expire` lifecycle will not reach it. |
| Master seed | **1788150478** (logged as "Effective master seed" at run start; `autoc.ini` has `Seed = -1`, so the seed is resolved at launch and this value is the only record of it) |
| Regiment | pop **5000** × **800** generations × **294** scenarios (6 `SimNumPathsPerGeneration` × 49 wind ⇒ `ExpectedScenarioCount = 294`) |
| Launched | 2026-08-30 21:27, detached via `scripts/train.sh autoc.ini logs/autoc-043-t2-m1-acro.log` (T065) |
| Completed | gen 800 uploaded 2026-09-02 15:55 (`gen9200.dmp.zst`, the lowest-numbered object under the prefix) |

## 2. The flown genome

| Field | Value |
|---|---|
| Object | `gen9200.dmp.zst` |
| Generation | **800** — ⚠️ two labels for one net: `nnextractor` prints `Generation: 800` (filename-derived, `10000 − 9200`), `autoc` eval mode prints **`799`** (`genome.generation`, the trainer's 0-indexed counter). **Match on fitness, not on the generation label.** |
| Fitness | **−88013.840878** |
| Mutation sigma | 0.051642 |
| Topology | **45 → 32 → 16r → 3**, 2307 weights |
| Scenarios completed | 281 / 294 (per T068 task note) |
| `nn_weights.dat` | 9371 bytes, sha256 `3af8e3ab787b75a5…` |
| ⭐ **`weight_id`** | **`3af8e3ab787b75a5`** — sha256[0..7] of `nn_weights.dat`, emitted as `generatedNNWeightId` and written into every flight-log header. **This is the field that proves the flown firmware carries THIS genome.** (Sept-1 bench ran gen 554 = `610e0eba3506b149`.) |
| `firmware_id` | `95dd8904c0ae04b8` — hash of the generated code text |

Extract with:

```bash
./build/nnextractor -k autoc-9223370248704297747-2026-08-31T04:27:58.060Z -o nn_weights.dat
./build/nn2cpp   -w nn_weights.dat -i autoc.ini -o xiao/src/generated/nn_program_generated.cpp
```

⚠️ `nn2cpp`: `-w` is the genome, `-i` is the config. The generated header must read source
`…2026-08-31T04:27:58.060Z/gen9200.dmp.zst` and topology `45 -> 32 -> 16r -> 3`.
⛔ `xiao/src/generated/` is **git-ignored** (`.gitignore:24`) — the generated file does not survive a
checkout and must be regenerated from this manifest on any fresh working copy.

## 3. Commits

| Commit | What it pins |
|---|---|
| `c8d00ab` | autoc at launch (2026-08-30 21:29, 2 min after `train.sh`) |
| `ac4d796` | **crrcsim submodule pointer** (2026-08-30 21:22) — the arm-C model, 5 min before launch. Tree clean, pointer unmoved since. |
| `400b217` | rates 36/24 into both cfgs (post-bake, 2026-09-01) |
| `2d4b27a` | final t2 PNGs at gen 800 |

`autoc.ini` is unchanged since `82e45f6` (2026-08-25), i.e. **before** launch — so the working-copy ini
*is* the as-run ini. Snapshotted alongside as [`autoc.ini.as-run`](autoc.ini.as-run),
sha256 `2f34f0054a72fc84…`.

## 4. ⭐ Arm C — the action space this genome was trained against

⛔ **This is the rate-parity gate (T051b). Flying an FC set otherwise diverges in exactly the axis 043
exists to fix, and wastes the bake.**

| Axis | Model `maxRate` (`crrcsim/models/hb1_streamer.xml`) | INAV `rates` | `commandScale` |
|---|---|---|---|
| roll | 360 | **36** | 1.0 |
| pitch | **240** (arm C: was 120, i.e. `pitch_rate` 12 → 24) | **24** | 1.0 |
| yaw | — | 4 (flight article, 2026-09-04 dump) | — | 

⭐ Yaw is **immaterial** to the parity gate: the model declares no yaw `maxRate`, and T056 bench-verified
that no surface responds to the yaw axis (`axisRate`/`axisF` yaw = 0.0). The bench rig's `3` and the flight
article's `4` are both fine.

FC-side config of record: `xiao/inav-hb1.cfg`, **`control_profile 1`** (INAV 8 folds rates into the control
profile; profiles 2 and 3 are stock 20/20/20 and unused — the dump's trailing `control_profile 1` selects 1). ✅ Parity confirmed 2026-09-01 (FC 36/24 ==
model 360/240); ⚠️ re-assert on the flight article in T069, since flashing is what could silently move it.

Accepted for this flight (T051c): roll stays at **36**, leaving the top **14%** of roll stick clipped
(full-stick FF = 360 × 1.613 = 581 against the fixed ±500 `pidSumLimit`).

## 4b. Flight-article FC firmware (verified 2026-09-04)

Config of record is now [`xiao/inav-hb1.cfg`](../../../xiao/inav-hb1.cfg) — as of 2026-09-04 a **genuine
flight-article capture** (target `MATEKF722MINI`), no longer the Apr-2 dump with `pitch_rate` hand-edited.
⚠️ It supersedes the note in [`bench-notes.md`](../bench-notes.md) that no flight-article dump existed.

The intermediate CLI dumps were deleted once folded in here; their unique content was the FC state
**before** the rate change, preserved below:

| Line | Before (2026-09-04 19:58) | After (`inav-hb1.cfg`, config of record) |
|---|---|---|
| `pitch_rate` | 12 | **24** ✅ arm C |
| `roll_rate` | 36 | 36 (unchanged) |
| `yaw_rate` | 4 | 4 (unchanged, immaterial) |
| `gyro_zero_x/y/z` | −27 / 30 / 13 | −34 / 34 / 14 |
| `ins_gravity_cmss` | 948.609 | **972.092** |

⭐ The accel calibration was redone alongside the rate change: `ins_gravity_cmss` moved from **948.609**
(≈3% below true g) to **972.092** (≈0.7% below). This matters because the 45-input genome consumes
accelerometer specific force scaled by `kAccelScale_g = 8.0`, delivered over the `MSP2_AUTOC_STATE`
extension that build `38ff0d29e` added.

⚠️ The **bench** board (`MAMBAF722_2022A`, `inav-bench.cfg`) runs the older `63cffaf4` build, which predates
that MSP extension — a reason bench results do not transfer to the NN's accel path.

| Field | Value |
|---|---|
| Flashed build | `38ff0d29e` (2026-08-22) — *"feat(041): extend MSP2_AUTOC_STATE with accelerometer specific force"* |
| Fork | `~/inav` branch `autoc`, HEAD `c412bfd76` (2026-09-01) |
| Source delta flashed → HEAD | **empty** — the only two commits between are `52374cb37` (T057 MSP-override fix) and `c412bfd76` (its revert), which cancel exactly ⇒ **no reflash needed** |

⛔ Correction to `bench-notes.md`: it states HEAD is `67ba0919e` and that `52374cb37` / `c412bfd76` "do not
exist in `~/inav`". On **this** host all four commits exist and `c412bfd76` *is* HEAD. The other host's fork
clone is stale; trust this record.

⚠️ The flashed build is what supplies the NN's accelerometer specific force over `MSP2_AUTOC_STATE` — the
`63cffaf4f` (2026-04-02) build the config-of-record was captured from **predates that**, so the
config-of-record dump is not a valid firmware reference for a 45-input genome.

## 5. ⛔ Input scale constants (`include/autoc/nn/nn_inputs.h`)

Without these the genome loads clean and flies wrong.

| Constant | Value |
|---|---|
| `kCruiseSpeed_mps` | 13.0 |
| `kDistToBoundaryScale_m` | 20.0 |
| `kTargetDistScale_m` | 26.0 |
| `kClosingRateScale_mps` | 16.0 |
| `kGyroScale_radps` | 6.0 |
| `kAccelScale_g` | 8.0 |
| `kEnergyScale_m` | 145.0 |
| `kScoreGradScale` | 0.78 |
| `kTimeSinceSeenScale_s` | 2.0 |
| `kNNHistoryLagsMsec[6]` | 800, 400, 200, 100, 50, 0 |
| `kNNHistoryLayoutVersion` | **3** |

Arena and cone are baked into the generated evaluator from `autoc.ini`:
arena `70.0, 25.0, 105.0`; cone `7.000, 2.000, 45.000, 0.500, 5.000, 5.000`.

## 6. ⚠️ Known deviations carried into the flight

- **Phase 6 (T046–T050a) did NOT run before the bake** ⇒ **SC-010 unmet**. The genome trained against a
  *modelled* actuator, not a *measured* one. Consequence is attribution, not safety: a sim↔flight
  divergence in T077 cannot be cleanly split between "the ACRO model is wrong" and "the actuator model is
  wrong". T050a is moot — there was no plant change to re-gate.
- **Crash rate 4–7%** of scenarios versus 041-t7's **0.7%** (`specs/BACKLOG.md` § streak-outbids-crash).
  Operator ruled it FLYABLE 2026-09-04. It is the thing to watch in the air, and it argues for altitude in
  hand on the first engagement.

## 7. Firmware state at manifest time

Regenerated from gen 800 and gated 2026-09-04 (T059):
`pio run -e xiaoblesense_arduinocore_mbed` **SUCCESS** — RAM 53.5% (127204 / 237568), Flash 45.5%
(369164 / 811008). Built, **not yet flashed**.
