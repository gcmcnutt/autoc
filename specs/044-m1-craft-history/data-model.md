# 044 — data model

## CraftHistoryWindow (new, per craft state)

| field | type | notes |
|---|---|---|
| `gyro_p[3]`, `gyro_q[3]`, `gyro_r[3]` | `gp_scalar` rad/s | readings at 100 ms, 50 ms, now (oldest first) |
| `accel_z[3]` | `gp_scalar` g | body specific force z (incl. gravity), same lags |

- Lags from one constant `kNNCraftHistoryLagsMsec = {100, 50, 0}`; ring depth = 100/50 + 1 = 3 ticks; signals from one list.
- Written once per decision tick in `recordErrorHistory(target, dist, t, craftSample)` — same call as the target ring, both targets.
- `resetHistory` pre-fills every slot with the current reading (scenario start, engage, companion reboot).
- Validation: lags integral multiples of the tick (static_assert, as the target table); no slot ever reads zero after reset.

## CraftCommonInputs v2 (shared by NNInputs and TrackerInputs) — 20 → 28 floats

`quat_w, quat_x, quat_y, quat_z, airspeed, gyro_p_tm2, gyro_p_tm1, gyro_p, gyro_q_tm2, gyro_q_tm1, gyro_q, gyro_r_tm2, gyro_r_tm1,
gyro_r, accel_x, accel_y, accel_z_tm2, accel_z_tm1, accel_z, specific_energy, boundary_closure_rate, dist_to_boundary,
inward_body_x, inward_body_y, inward_body_z, score_grad_x, score_grad_y, score_grad_z`

- Scales unchanged per signal (gyro ÷ 6 rad/s, accel ÷ 8 g) — history slots use their signal's scale.
- `NNInputs` = 25 target + 28 craft = **53**; `TrackerInputs` = 46 target + 28 craft = **74**.
- Weights: M1 53·32+32 + 32·16+16 + 16·3+3 + 16·16 = **2563**; M2 74·… = **3235**.
- Invariants (static_assert): struct sizes; craft block last in both enums; the two enums' craft tails identical; metadata table
  length = enum COUNT.

## ScenarioMetadata (appended field)

| field | type | default | notes |
|---|---|---|---|
| `craftClP` | `gp_scalar` | `kCraftClPCenter` (−0.36) | absolute roll damping; clamp [−0.60, −0.18]; appended LAST |

## CraftVariationDraw / CraftSigmas

| field | change |
|---|---|
| `craftClP` | new draw: centre −0.36 + N(0, `CraftClPSigma`), clamped |
| `craftClPSigma` | new ini key `CraftClPSigma` (0.10) |
| `craftCmQ` | centre −2.25, clamp [−3.5, −1.0]; `CraftCmQSigma` 0.5 |

## crrcsim globals and FDM

- `Global::craftClP` (NaN = unset → xml nominal), set per scenario by `inputdev_autoc` next to `craftCmQ`.
- `fdm_larcsim`: `Cl_p = craftClP`; `Cl_da = Cl_da_ref · (craftClP / Cl_p_ref) · (1 + craftRollEffDelta)` with the t6 reference pair
  (−0.135, 0.0633) kept as named constants of the model.
- Sensor filters: per-scenario state for gyro PT1 25 Hz and accel biquad 15 Hz, stepped every 5 ms sub-step; reset per scenario.

## Variation streams

- `deriveClassSubSeeds(seed)` → `{wind, rabbit, entry, craft, camera}` = `fmix32(root.next())` each, zero-guarded.
- Invariant (test): no class's first 64 draws contain another class's sub-seed or first draw (10 000 seeds).

## WindDraws (logged)

Scenario table gains `windMps, turbX, gustLx, thermX` — full-strength values of the four wind-envelope draws.

## Eval config

| key | type | default | meaning |
|---|---|---|---|
| `EvalVariationScale` | double | −1 | −1 = weights file's stored scale (today); 0…1 overrides; anything else rejected at startup |

## Versioned artifacts

| artifact | before | after |
|---|---|---|
| `EvalResults` (dmp) `kSchemaVersion` | 4 | **5** |
| xiao flight log `kFormatVersion` | 5 | **6** |
| NN input layout marker | v3 | **v4** |
| `data.dat` header | 45 inputs | 53 (derived from the metadata table) |
