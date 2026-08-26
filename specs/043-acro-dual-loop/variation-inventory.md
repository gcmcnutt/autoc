# 043 — variation-class inventory (FR-050 / FR-051 · SC-008)

Every per-scenario variation class, what it varies, its magnitude, its enable knob, whether it is
per-scenario, and **whether the ramp applies**. The ramp is `applyVariationScale`
(`include/autoc/eval/scenario_meta_apply.h`) — the single source of truth for what ramps. ⭐ It scales
**only the environmental pose/wind classes**; everything else is full-magnitude from gen 0.

## What actually ramps (read from the code, not the comments)

`applyVariationScale(meta, scale)` mutates **only**: `entryHeadingOffset`, `entryRollOffset`,
`entryPitchOffset`, `windDirectionOffset`, `entrySpeedFactor`, and the entry position offsets
(`entryNorthOffset`, `entryEastOffset`, `entryAltOffset`, with the arena clamp). It touches **nothing
else** — not rabbit speed, not any craft field, not any camera field.

## Inventory

| class | what it varies | magnitude (2.5σ envelope) | enable knob | per-scenario | **ramped?** |
|---|---|---|---|:---:|:---:|
| **wind** | wind direction offset; per-seed thermals/gusts | `WindDirectionSigma` 45° | `EnableWindVariations` | yes | ✅ **direction ramps**; the per-seed thermal/gust field is drawn at the wind sub-seed and is not itself scaled |
| **rabbit** | rabbit (target) traversal speed | `RabbitSpeedSigma` about `RabbitSpeedNominal` 12 m/s | `EnableRabbitSpeedVariations` | yes | ❌ **not ramped** (not in `applyVariationScale`) |
| **entry** | entry pose (cone heading, roll, pitch), entry speed factor, entry position (N/E/Alt) | cone 18° (2.5σ→45°), roll 30°, speed 0.06 (2.5σ→15%), position 0 m by default | `EnableEntryVariations` | yes | ✅ **ramped** — the difficulty curriculum |
| **craft** (034/037) | CG, drag, Cm_0 trim, thrust scale, pitch/roll effectiveness, servo slew, thrust tau, PWM latch phase | CG 0.02, drag/thrust/pitchEff/rollEff ±5%, trim 0.02 rad, servoSlew 4 /s (clamp 16–32), thrustTau 0.06 s (clamp 0.05–0.30), pwmPhase U[0,20 ms) | `EnableCraftVariations` | yes | ❌ **not ramped** — diversity, not difficulty (FR-055) |
| **craft** (043 US5 — NEW) | IMU mount misalign (R/P/Y), gyro scale (X/Y/Z), accel scale (X/Y/Z), accel bias (X/Y/Z), `craftCmQ` pitch damping | misalign 2.0° (2.5σ→±5°), gyro/accel scale 0.02 (2.5σ→±5%), accel bias 0.04 g, `craftCmQ` center −4.2 clamp [−5.0,−3.6] (σ 0.32) | `EnableCraftVariations` + per-axis sigma | yes | ❌ **not ramped** (FR-055; verified by test `CraftVariationImu.NewAxesAreNotRamped`) |
| **camera** (040) | boresight yaw/pitch/roll, mount translation X/Y/Z, wing thickness | boresight/roll 4° (2.5σ→10°), translation 2–4 mm, wing 0.8 mm | `EnableCameraVariations` | yes (rides `WorkerInit`, not the dmp) | ❌ **not ramped** — diversity, not difficulty |

⛔ **FR-051 contradiction fixed (T008)**: `autoc.ini` previously claimed craft *"RAMPS with wind/entry
(same VariationRampStep)"*. That was false — the code has never ramped craft. The `autoc.ini` comment is
corrected to state craft is full-magnitude from gen 0.

## Regiment (T008a · FR-058)

⛔ **The scenario regiment is 294 = 6 paths × 49 winds, and the population is unchanged.**
`SimNumPathsPerGeneration = 6`, `WindScenarios = 49`, `PopulationSize = 5000`. A new opt-in pre-run gate
`ExpectedScenarioCount = 294` (autoc.ini) makes autoc **FATAL** if `paths × winds != 294`, so a silent
bump to either knob cannot reach the 27 h bake. `0` disables it for other experiments.

## Notes / provisional items carried into later phases

- ⚠️ **`CraftAccelBiasSigma = 0.04 g` is provisional on the acc_1G scale.** Derived from the 041-t7 flight
  blackbox (`accSmooth[1]` mean ≈ −279 raw counts; at ICM `acc_1G ≈ 4096` that is ≈ −0.068 g, treated as
  the contract's 1.70σ anchor ⇒ σ ≈ 0.04 g). If the board's `acc_1G` differs, rescale. This is a
  non-ramped **diversity** knob — being in the right ballpark is what matters, not the exact value.
- ✅/⛔ **T015 is split (operator decision 2026-08-25 — fold into Phase 5).** The 043 US5 draws, metadata,
  config, plumbing, tests and `craftCmQ → Cm_q` (FDM, no-op verified) are all landed and green, and the
  Global carriers for all 13 IMU/cmQ fields are set per-scenario. The *observation-path* application of the
  IMU misalign/scale/bias — rotating the gyro/accel/target-geometry the **policy** sees via a **sensed
  copy** distinct from the true state (fitness/arena-bounds/physics must keep truth; `getGyroRates()` feeds
  both NN inputs and `fitness_decomposition.cc`) — is **deferred to Phase 5**, where `Cntrl_InavFwRate` is
  the other consumer of the misaligned gyro and the T055 bench polarity check validates signs end-to-end.
  Until then the IMU axes are drawn/recorded/replayable but do not yet bias the observation.
- **T047a (FR-056) craft-realism review at n=2 articles** — AHRS alignment, control-surface trim/bias,
  control response gains/rates, per-axis *measured vs assumed* — is a Phase-6 (US2 plant-pin) deliverable
  and is intentionally not filled here. Operator note 2026-08-25: build repeatability is coming, so
  **characterise, do not chase**.
- **T079/T080 observability audit (SC-013 / FR-059/059a)** — whether each axis is observable, absorbed, or
  observable-only-at-limits — is a Phase-11 deliverable; verdicts there are sim verdicts, provisional on
  the flight.
