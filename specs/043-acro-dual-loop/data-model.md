# 043 — data model

Only what **changes**. Everything else in the eval pipeline is untouched — which is itself the headline:
the NN input/output vectors do not move.

## 1. `ScenarioMetadata` — grows (⛔ format break)

`include/autoc/rpc/scenario_metadata.h`. New fields **appended last**, after `craftServoPwmPhase`, and
added to the `serialize()` walk in the same position. `gp_scalar` per Constitution VI.

| field | default (no-op) | notes |
|---|---|---|
| `craftImuMisalignRoll/Pitch/Yaw` | 0.0 | deg |
| `craftGyroScaleX/Y/Z` | 1.0 | multiplicative — defaults to nominal, like `craftThrustScale` |
| `craftAccelScaleX/Y/Z` | 1.0 | multiplicative |
| `craftAccelBiasX/Y/Z` | 0.0 | g |
| **`craftCmQ`** | **−4.2** | pitch damping — ⭐ absolute physical value + clamp (like `craftServoSlew`), **not** a delta. Clamped to [−5.0, −3.6], the range `hb1_streamer.xml` already derives from the streamer's effect |

⛔ **Break consequences**: every existing dmp becomes unreadable (fail-loud, Constitution V). **FR-057
extraction from the pinned 041-t7 run must complete first** — permanent if missed. No cereal version bump
(project practice).

## 2. `CraftSigmas` / `CraftDeltas` — grow in parallel

`include/autoc/eval/craft_variation.h`. Same fields; draws **appended at the bottom** of
`generateCraftFromClassPRNG` so every existing draw keeps its value. Sigmas from `autoc.ini`
(`CraftImuMisalignSigma`, `CraftGyroScaleSigma`, `CraftAccelScaleSigma`, `CraftAccelBiasSigma`,
`CraftCmQSigma`).

⛔ **No inner-loop gain variation** (decided 2026-08-25). The gains are known exactly from the config;
`craftGyroScale` alone carries the commanded-rate-≠-achieved-rate uncertainty.
⛔ **No separate static-margin axis** — `craftCGDelta` already varies CG against the neutral point.
`craftCmQ` carries the **dynamic** side only.

⚠️ `applyVariationScale` leaves them **untouched** — craft is not ramped (FR-055).

## 3. Model XML — new `<controllers>` node

`crrcsim/models/hb1_streamer.xml`. Carries every constant in
[contracts/inav-fw-rate-loop.md](contracts/inav-fw-rate-loop.md) so they change without a rebuild (FR-014).
⚠️ Controllers currently load from the **global** config (`crrc_fdm.cpp:38`), not per-model — if
per-scenario gain variation is ever wanted, follow the `fdm_mcopter01` per-model pattern instead.

## 4. `Cntrl_InavFwRate` — new controller state

Per axis, reset at scenario init (determinism): `integrator`, `prevGyroRate`, `dtermLpfState`,
`ptermLpfState`, `targetOverThresholdTimeMs`. ⛔ No attitude state of any kind — see FR-019a.

## 5. Unchanged, and deliberately

| | |
|---|---|
| **NN inputs** | 45, same order, same scaling (spec assumption 11) |
| **NN outputs** | 3, same magnitude **and polarity** — only the interpretation changes |
| **NN01 weight format** | unchanged |
| **xiao log records** | ⛔ **entirely unchanged** — US1 dropped 2026-08-24 and FR-005 cut 2026-08-25. ⭐ 043 makes **no xiao log-format change at all** |
| **servo/mixer path** | unchanged — MANUAL and ACRO both feed ±500 into the same mixer |

⚠️ **Conditional**: if R2 (arm's length) returns "add an input", the NN input vector and every downstream
consumer move. That is why R2 is sequenced **before** the bake (plan phase 7).
