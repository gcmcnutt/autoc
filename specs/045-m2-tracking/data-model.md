# 045 — Data model

Entities from [spec.md](spec.md) § Key Entities, with fields, validation and state. Wire and storage layouts are in
[contracts/](contracts/).

## Configuration change set (FR-007, FR-007a/b, FR-008/8a, FR-027)

One per article per flight. Lives in the repo as the article's config of record (`xiao/inav-hb1.cfg`, `xiao/inav-hb1-art2.cfg`),
plus a change-set table in the flight's analysis doc.

| field | meaning | validation |
|---|---|---|
| setting | CLI name | exists in the fork's `settings.yaml` |
| old → new | values | `new` read back after `save` + reboot (FR-009) |
| class | A rate loop / B observation (accuracy or delay) / C nav estimate / D fallback / E logging / hardware | B-delay and A require a sim change + g800 re-evaluation first (FR-017) |
| evidence | research.md item or flight analysis § | non-empty |
| applies when | boot-only / live / needs save | from research R5 |

Calibration rows (6-point accel, gyro zero, `ins_gravity_cmss`, compass offsets, board and compass alignment) carry the **date,
article, level reference** (flat wing bottom) and **conditions** (outdoor, away from metal).

## Estimator fault flags (FR-011, FR-012) — per xiao tick, advisory only

> ⏸ Per-tick companion flags deferred (spec § Clarifications 2026-11-12). The same conditions are flagged offline from the blackbox
> by `nav_span_scan.py`; the FROZEN / STEP definitions below apply to that tool.

| field | type | source |
|---|---|---|
| `est_eph`, `est_epv` | u16 cm | INAV `posEstimator.est.eph/epv` (state message v2) |
| `est_flags` | u8 bits: XY_VALID, Z_VALID, GPS_XY_VALID, GPS_Z_VALID, BARO_VALID | `posEstimator.flags` |
| `gps_numsat`, `gps_fix` | u8, u8 | `gpsSol` |
| `acc_weight` | u8 (×100) | `posEstimator.imu.accWeightFactor` |
| `gps_age_ms` | u16 | now − `posEstimator.gps.lastUpdateTime` |
| `fault_flags` | u8 bits: INVALID_XY, INVALID_Z, FROZEN, STEP_XY, STEP_Z, GPS_STALE (> 200 ms), LOW_SATS (< 6) | derived on the xiao |

- **FROZEN**: armed and engaged, horizontal velocity exactly 0 and position unchanged versus the previous tick, for **≥ 2
  consecutive ticks** (0.1 s at 20 Hz). *Pinned by N1 (drift-report.md § Pinned values, 2026-11-12)*: in 4614 engaged ticks across
  11 spans, the only occurrence is the 11-08 span-1 freeze (2.30 s).
- **STEP**: |Δpos − v̄·Δt| > **5 m horizontal / 5 m vertical** in one 50 ms tick. *Pinned by N1*. The clean-tick residual maxima
  are 1.00 m horizontal and 1.59 m vertical (p99.9 0.74 / 1.38 m, `n1/nav_residuals.csv`); the real faults are 106 m (XY) and
  25–101 m (Z). The vertical threshold moved from the provisional 3 m, which left < 2× margin over clean data.
- Until the per-tick companion flags land, `nav_span_scan.py --blackbox` applies these same definitions to the blackbox, resampled
  to 20 Hz (`n1/faults.csv`).
- Flags never feed engagement, disengagement or the NN (FR-012). A test asserts that the policy output is bit-identical with flags
  forced on and off.

## Span-local inertial frame (shadow) — FR-015, FR-016

State (error-state filter, propagated at IMU rate): attitude quaternion (body → span-local NED), velocity, position, gyro bias;
optional accel bias. Covariance diagonal logged.

Lifecycle:

```
DISARMED ──arm──▶ ARMED_IDLE ──engage──▶ HANDOVER ──(ring buffer caught up)──▶ PROPAGATING ──disengage──▶ ARMED_IDLE
                                              │                                   │  GPS update accepted → smooth correction
                                              │                                   │  GPS rejected / stale → COASTING (no reset)
                                              └── IMU fault (FIFO overrun, I2C error) ──▶ FAULTED (logged; never feeds the NN)
```

- Initialized once per span at engage from INAV's attitude, velocity and position, propagated forward from INAV's sample time with
  a gyro ring buffer.
- **No state reset inside a span**, ever. A large GPS innovation is gated and coasted, not jumped to.
- Never read by the policy (shadow).

## Flight log v7 budget (xiao QSPI, 2 MB log region)

> ⏸ Tick fields deferred (2026-11-12). If only the ShadowStateRecord lands: 148 + 31 = 179 B/tick.

| record | v6 bytes | v7 bytes | rate |
|---|---|---|---|
| TickRecord (N = 53 inputs) | 148 | **158** (+ the 10 B of estimator fields + `fault_flags`) | 20 Hz while engaged |
| ShadowStateRecord (new, only if N4 flies) | — | **31** (type, tick id, quat 4×i16, vel 3×i16 cm/s, pos 3×i16 dm, gyro bias 3×i16, innovation u8, status u8) | 20 Hz while engaged |
| per-tick total | 148 | **189** | |

- The v6 assert (148 × 9600 ≤ 70 % of 2 MB) leaves ~47 KB. At 189 B, 70 % of 2 MB (1 468 006 B) holds **7767 ticks**.
- **Decision**: budget **7200 ticks** (6.0 min engaged ≈ four flights of 4 × 22 s spans). The assert moves to
  `189 × 7200 ≤ 0.7 × region`. **Erase the log before each field session** (already a backlog practice; it moves into the
  quickstart checklist).
- Raw xiao-IMU bursts (optional, for the mount survey) are a bench-only record type, off in flight.

## Drift report (FR-006, FR-006a)

Per flight × span × source (INAV AHRS, gyro-only from engage, shadow frame when flown): attitude error versus the GPS-specific-force
reference at 5 / 10 / 17 s / end; velocity and position drift for the span-local propagator; INAV faults (freezes, resets, steps,
with times and magnitudes); the span's gross and net rotation and load (the R3 table columns). Output: a markdown table in the N1
report plus a CSV.

## Camera parameter record (FR-020, FR-020b) — after G2

Per parameter or variation: name, value, unit, source (measurement id or bench-journal §), class (M / D / A / provisional), fate
(kept / updated / added / deferred), phase (4 geometry / 5 signal). Single definition per parameter (removes the five
`CameraDegPerPixel` copies).

## Signal mode (FR-020a) — after G2

Ini key `TrackerSignalMode = PERFECT | REALISTIC`. **No default**: a missing key fails at startup (Constitution VII). Logged at
startup and recorded in the run's as-run ini, **not in the dmp**: `source_dmp_loader` deserializes the whole `EvalResults` with
cereal, so a schema change would orphan the `retain=keep` 044-t1 tracker source (FR-026; analyze U1, 2026-11-12). `PERFECT` forces TRACKING / q = 9 / no CDMA penalty / no emission-darkness gate /
no 100 m cutoff for any beacon passing the geometry gates.

## M2 performance result (FR-024) — after G2

Fixed-evaluation report: tracking occupancy, per-step error, reacquire time after the target leaves view, crash rate (040/041
definitions), labelled with signal mode and navigation-state mode, with the 040/041 comparison caveat.
