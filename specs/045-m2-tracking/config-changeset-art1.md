# 045 — Configuration change set, article 1 (MATEKF722MINI, INAV 8.0 `38ff0d29`)

Format: [data-model.md](data-model.md) § Configuration change set (FR-007, 007a, 007b, 008, 008a, 009, 027). One change set for
the re-fly. Base = the configuration of record, [`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg) @ `91abd4d` (the 11-08 flight).

⚠️ **Status (2026-11-12)**: the ✅ rows are **on the FC** (applied with [`inav_cli_apply.py`](inav_cli_apply.py): set → get read-back
→ save → reboot → read-back). The repo's config of record is **deliberately not updated** until T049 (operator: not before the
research settles). ⏳ rows are pending, each with the task that settles it. **Failsafe settings are not in this change set**: they
change only through the safety ladder (tasks Phase 10, L7). The companion-UART setting (R-7) is ladder step L8.

Classes: **A** inside the ACRO rate loop (dynamics) · **B-acc** what the policy observes, accuracy only · **B-delay** what the
policy observes, delay / filtering · **C** navigation estimate / GPS · **D** fallback · **E** logging · **HW** hardware.
FR-017: no A or B-delay change on the re-fly unless the simulator models it first. "Trained-on effect" says whether the change
moves flight away from what g800 was trained on. The simulator feeds perfect state, so an accuracy-only change moves flight
*toward* training.

## Rows

| # | setting | config of record → re-fly | class | evidence | applies | read-back | trained-on effect |
|---|---|---|---|---|---|---|---|
| 1 ✅ | `gps_ublox_use_galileo` | OFF → **ON** | C | R5; 5–7 sats in 11-08, every drop in a pull | sent to the receiver at boot (`gps_auto_config ON`) | `get` ON; `status` after reboot: **Galileo 1/0 active** (11-11) | none (nav frame quality) |
| 2 ✅ | `gps_ublox_use_glonass` | OFF → **ON** | C | R5; M8 runs 3 GNSS concurrently | boot | `get` ON; `status`: **GLONASS 1/1 active** | none |
| 3 ✅ | `gps_ublox_use_beidou` | OFF → OFF *(guarded)* | C | a 4th GNSS exceeds the M8's 3; the NAK reverts all three silently (R5) | boot | `get` OFF; `status` BeiDou 0/0 | none |
| 4 ✅ | `gps_dyn_model` | AIR_2G → **AIR_4G** | C | R3: > 2 g for 54–83 % of each span; AIR_4G is u-blox's highest | boot | `get` AIR_4G | none |
| 5 ✅ | `init_gyro_cal` | ON → **OFF** | calibration (B-acc) | FR-007a / R5: the boot calibration inherits whatever attitude the aircraft sat at (gravity 909 / 943 / 979 on successive boots); also needed for an in-flight re-arm after an FC reboot (R17) | boot | `get` OFF | none: removes a boot-dependent bias. ⚠️ the gyro zero no longer tracks temperature: re-record per session (quickstart § 0) |
| 6 ✅ | `inav_w_acc_bias` | 0.010 → **0.000** | C | R5: the accel-bias learner is fed by faulted GPS / baro residuals rotated through a wrong attitude, unclamped to 0.25 g | live | `get` 0.000 | none |
| 7 ✅ | `ins_gravity_cmss` | 943.520 → **979.700** *(baseline)* | calibration (C) | still \|a\| = 0.9990 g (σ 0.0015, 79 MSP samples) × 980.665 (11-11). ⏳ re-record after the 6-point accel calibration (T035 / T036) | live | `get` 979.700 | none (the estimator's 1 g reference) |
| 8 ✅ | `gyro_zero_x/y/z` | −35 / 34 / 13 → **−35 / 32 / 9** *(baseline)* | calibration (B-acc) | last still boot with `init_gyro_cal ON` (11-11); boots spread 0.3–0.6 °/s. ⏳ deliberate levelled record (T036) | boot (used live once cal is off) | `get` −35 / 32 / 9 | none: a bias correction (it also feeds the rate loop, with no change of dynamics) |
| 9 ✅ | `align_board_roll` | 1700 → **1736** | B-acc | gravity-solved on the levelled flat wing bottom (11-12): attitude roll / pitch −1.35 / +3.73° → **+0.06 / −0.21°**; the FC sits ~6.4° back-tilted, not 10° | boot | `get` 1736 | removes the **3.7° nose-up bias** in the policy's quaternion inputs: toward training. ⚠️ board alignment also rotates the gyro axes the rate loop uses, here by 3.6° about the pitch axis (≈ 6 % roll ↔ yaw cross-coupling removed). Judged an accuracy correction toward the true airframe axes, not a dynamics change: **T045 to confirm** |
| 10 ✅ | `align_board_pitch` | 0 → **7** | B-acc | as row 9 (the 0.7° roll component) | boot | `get` 7 | as row 9 |
| 11 ✅ | `align_mag_roll / pitch / yaw` | 0 / 0 / 0 (⇒ CW0 + board alignment) → **1736 / 7 / 900** | B-acc (heading) | R19 / FR-008a: explicit external-compass path with the same rotation as the board (CW270FLIP + the panel back-tilt), so a board change can't move the compass silently. Heading at magnetic north read 12.9° = the local declination (11-12). ⏳ refine from the measured GPS-panel tilt (T038) | boot | `get` 1736 / 7 / 900; read-back PASS ([log](bench-20261111/art1_apply_align_20261112_log.txt)) | none (heading accuracy) |
| 12 ⏳ | `inav_max_eph_epv` | 1000 → **2000–3000** cm | C | R5: the cap turns a coast into a freeze + reset; the value is pinned from N1's coast durations (T029) | live | — | none. ⚠️ trades INAV's validity for continuity; the offline flags record EPH regardless |
| 13 ⏳ | `inav_default_alt_sensor` | GPS → **GPS_ONLY**, only if multi-GNSS EPV ≤ ~5 m | C | R5: removes the baro fade (the 26 m Z reset); baro uncompensated for dynamic pressure. Decided by the outdoor 10 min static (T043) | live | — | none |
| 14 ⏳ | blackbox groups | `-MAG -NAV_ACC` → **`MAG NAV_ACC`** | E | FR-014 / R9 (B1): heading and compass analysis; earth-frame acceleration for N1 without reconstruction (T051) | save | — | none |
| 15 ⏳ | `blackbox_rate_denom` | 8 → **8, 16 or 32** | E | the flash write bandwidth with rows 14 on: the fastest with no dropped frames; 1:32 acceptable (operator 2026-11-12) (T051) | save | — | none. At 1:32 gyro-only propagation is coarse (R4 sampling term) |
| 16 ⏳ | `debug_mode` | VIBE → VIBE *(kept deliberately)* | E | the blackbox's only source of accWeight (R9) (T051) | — | — | none |
| 17 ⏳ | accelerometer 6-point (`acczero_*`, `accgain_*`) | 52 / 25 / −68, 4106 / 4071 / 4050 → *(new)* | calibration (B-acc) | \|a\| read 0.986 g on 11-12 vs 0.999 on 11-11 ⇒ redo; verify ≥ 12 poses with `cal_verify.py accel` (T035) | save | — | none (accuracy) |
| 18 ⏳ | compass calibration (`magzero_*`, `maggain_*`) | −271 / −393 / −510, 1409 / 1347 / 1370 → *(new, outdoors)* | calibration (B-acc) | dip read ~18° below the site's ~61° indoors after the alignment fix ⇒ outdoor check, then calibration if needed (T044, after T038) | save | — | none (heading accuracy) |

Not in this change set (and why): `failsafe_*`, `motorstop_on_low`, `failsafe_procedure` → safety ladder L7. Any rate-loop gain or
filter (`gyro_main_lpf_hz`, `fw_*` rates, notch) → class A / B-delay, held at the config of record (FR-017). `msp_override_channels`
(47) unchanged: the companion's ACRO-through-RC6 is intended; the escape is RC9 off (spec clarify 2026-11-11).

## Article 1 receiver record (FR-008, US4 AS2)

| item | value | source |
|---|---|---|
| module | Matek **M8Q-5883** = u-blox **SAM-M8Q**, with a QMC5883 compass | research R5 (web, 2026-11-11) |
| firmware | `status`: **HW Version UBLOX8, Proto 18.00** ⇒ ROM SPG 3.01 (Galileo-capable); ROM-only, no flash: settings live in battery-backed RAM, and INAV re-sends them every boot (`gps_auto_config ON`). No firmware update possible or needed | bench A5, 11-11 |
| GNSS capabilities | Galileo supported (was 0/0: off), GLONASS default on (config of record had it off ⇒ article 1 flew **GPS-only**), BeiDou 0/0, **max concurrent 3** | bench A5 |
| after rows 1–3 | `status`: GPS 1/1, Galileo 1/0, GLONASS 1/1 active, BeiDou 0/0; **13 sats** indoors (9–10 GPS-only), HDOP 1.37, EPH 1.76 m, EPV 2.65 m | bench, 11-11 |
| nav rate | message interval **100–101 ms ⇒ 10 Hz held** with 3 GNSS (no 5 Hz fallback). ⏳ confirm in an outdoor blackbox `*.gps.csv` (T043) | bench E2 (MSP), 11-11 |
| live read 2026-11-12 | `msp_bench_read.py --watch` (indoors, USB, motor off): fix 3D, **16–18 sats**, HDOP 1.20–1.53, EPH 2.0–2.3 m, EPV 2.3–2.8 m, GPS errors 1, timeouts 0; I2C errors 50 (constant: the boot-probe count); cycle ~510 µs, CPU 15 % | this file's T032 check |
| per-satellite C/N0 | not available over MSP on INAV 8.0 (`MSP_GPSSVINFO` is a stub). D4 uses the CLI `gpssats` (disarmed; `exit` reboots) or `gpspassthrough` + u-center; the MSP proxy (sats, HDOP, EPH) covers motor-on comparisons | `fc_msp.c` MSP_GPSSVINFO, `cli.c` `gpssats` |

## Calibration rows (template; one row per calibration, FR-007b / data-model)

| date | quantity | method | level reference | conditions | result | independent check (`cal_verify.py`) | read-back |
|---|---|---|---|---|---|---|---|
| 2026-11-12 | board alignment | gravity-solved, yaw fixed at 90° | flat wing bottom levelled spanwise + chordwise, nose at magnetic north | indoors, bench | `align_board_roll/pitch` 1736 / 7; attitude +0.06 / −0.21° | ⏳ in-flight `nav_accel_fit.py` on the re-fly | PASS (log) |
| *(T035)* | accelerometer 6-point | INAV Configurator | flat wing bottom | bench | `acczero_*`, `accgain_*` | `accel`: ≥ 12 poses, \|f\| 1.000 ± 0.005 g, cross-axis < 1 % | |
| *(T036)* | gyro zero + 1 g reference | `init_gyro_cal ON` for one still boot, values recorded, then OFF | flat wing bottom | bench, note temperature | `gyro_zero_*`, `ins_gravity_cmss` | `gyro-static` (⚠️ integer °/s blackbox: needs dither, see the tool) | |
| *(T037)* | gyro scale factor | 10 turns per axis on a jig, CW then CCW | jig | bench, blackbox denom 1–2 | per-axis scale error | `gyro-scale --turns 10` (CW/CCW pair, bias-free) | n/a (INAV has no gyro scale setting) |
| *(T038)* | compass alignment | measured GPS-panel back-tilt + arrow direction | flat wing bottom | bench | `align_mag_*` | `heading` at 8 bearings (T044) | |
| *(T044)* | compass calibration | outdoor tumble, away from metal and vehicles | — | outdoors | `magzero_*`, `maggain_*` | `mag`: \|m\| variation < 5 %, dip ≈ 61°; `heading` < 5° at 8 bearings | |
