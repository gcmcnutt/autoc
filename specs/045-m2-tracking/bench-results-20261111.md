# 045 bench results — 2026-11-11, article 1 ([USB] only, read-only)

Setup: FC + xiao on this host's USB; battery, motor and servos disconnected. INAV Configurator was closed first. Queries:
[`msp_bench_read.py`](msp_bench_read.py) (MSP, read-only) and a read-only CLI capture (`version`, `status`, `aux`, `serial`,
`feature`, `blackbox`, `diff all`, then `exit`, which rebooted the FC). Raw captures: [`bench-20261111/`](bench-20261111/).

| item | result |
|---|---|
| **A1 identity** | `INAV/MATEKF722MINI 8.0.0 Aug 22 2026 (38ff0d29)`, MPU6000 / SPL06 / QMC5883. **Article 1, flight build** ✔ |
| **A2 config vs repo** | `diff all` = `xiao/inav-hb1.cfg` except **4 boot-calibration values**: `gyro_zero_x/y/z` −25 / 29 / 10 (repo −35 / 34 / 13) and `ins_gravity_cmss` **909.391** (repo 943.520). All four come from `init_gyro_cal = ON` recalibrating at boot |
| **A3 mode bits** | ✔ **bit 19 = NAV ALTHOLD** (permanent id 3), **FAILSAFE = bit 29**, ARM = bit 0, ANGLE = 3, POSHOLD = 8, NAV RTH = 10, MANUAL = 20, **MSP RC OVERRIDE = bit 40** (outside the 32 bits the state message carries). ⇒ the xiao's `MSP_MODE_FAILSAFE = 19` watches ALTHOLD (R16 B-3 confirmed); the xiao cannot see the override bit at all |
| **A4 arming flags** | `NAV RX CLI` (MSP: 0x00040800 = NAVIGATION_UNSAFE + RC_LINK, before the CLI session); "DISABLE NAVIGATION FIRST", "FIRST WP TOO FAR". TX off on the bench |
| **A5 receiver** | `GPS: HW Version: UBLOX8 Proto: 18.00` (SAM-M8Q, FW 3.01); **Galileo 0/0** (supported, off), **Glonass 0/1** (module default on, config off), BeiDou 0/0, **Max concurrent: 3**. GPS-only now: 9–10 sats indoors, HDOP 1.5–1.7, EPH 2.0 m, EPV 2.1–2.5 m, message interval 101 ms (10 Hz) |
| **A6 plan settings (live)** | `failsafe_procedure` DROP, `failsafe_delay` 10, `failsafe_recovery_delay` 5, `failsafe_off_delay` 200, `failsafe_throttle` 1000, `motorstop_on_low` ON, `msp_override_channels` 47, `init_gyro_cal` ON, `inav_w_acc_bias` 0.01, `inav_max_eph_epv` 1000, `inav_default_alt_sensor` GPS, `gps_dyn_model` AIR_2G, `gps_ublox_nav_hz` 10, `gps_min_sats` 6, `align_mag_*` 0 with `align_mag` CW0 (⇒ board alignment applied to the external compass), `align_board_roll/yaw` 1700 / 900, `reboot_character` 'R', `blackbox_rate_denom` 8, `debug_mode` VIBE, `fw_p_level` 20, `nav_rth_altitude` 5000, `nav_rth_climb_first` ON |
| **other** | ⚠️ **`I2C Errors: 50`** in `status` (uptime 2.8 days): the I2C bus carries the external compass (and likely the baro). New bench item: watch the error count during a session and with the motor running |
| **A8 xiao (listen only)** | 1 Hz heartbeat, `mspOK=Y`, disarmed. Stationary on the bench: GPS position wanders ~1 m, velocity ±0.6 m/s (GPS-only), accel [0.02, 0.09, −1.00] g (the bench is ~5° off level), heading creeping ~2–3° over 9 s shortly after the FC's reboot (AHRS converging) |

## What it changes

1. **B-3 is no longer hypothetical**: the xiao's failsafe stop has been watching NAV ALTHOLD. Map the bits by permanent id at boot
   (`MSP_BOXIDS`), and carry the failsafe state explicitly in the state message (B-5). The override box (bit 40) isn't visible in
   the 32-bit field.
2. **The boot gravity reference swings 7 % with the bench's tilt** (909 vs 943 cm/s²) and the **gyro zeros shift 5–10 LSB per boot**
   (~0.3–0.6 °/s at ±2000 °/s full scale). That makes per-session, levelled, recorded values (`init_gyro_cal = OFF`, checklist C4)
   matter more, and means N4 must estimate gyro bias in-filter.
3. **Multi-GNSS needs no firmware work** on article 1: Galileo is supported, the max is 3 concurrent. Plan: GPS + Galileo + GLONASS,
   with the 10 Hz check (E2).
4. **New item: the I2C error count** (50 since boot). Check whether it grows with the motor running (D2) — it would also explain
   compass trouble.

## Change set applied to article 1 (2026-11-11, operator-approved) — ⚠️ NOT yet in `xiao/inav-hb1.cfg`

Baselines, not the deliberate calibration: the article was on the bench, close to level but not precise (operator). The repo's
config of record is **deliberately not updated** until the research settles. This table is the record of what is on the FC.
Applied by a whitelisted `set` → `get` read-back → `save` script (now [`inav_cli_apply.py`](inav_cli_apply.py); read-only capture: [`inav_cli_capture.py`](inav_cli_capture.py)); log in [`bench-20261111/art1_apply_log.txt`](bench-20261111/art1_apply_log.txt).
Post-change `status` + `diff all` in [`bench-20261111/art1_cli_after_changeset.txt`](bench-20261111/art1_cli_after_changeset.txt).

| setting | config of record | now on the FC | class / why |
|---|---|---|---|
| `gps_ublox_use_galileo` | OFF | **ON** | C — sats; research R5 |
| `gps_ublox_use_glonass` | OFF | **ON** | C — sats (3 concurrent max with GPS + Galileo) |
| `gps_ublox_use_beidou` | OFF | OFF | would exceed the M8's 3 concurrent |
| `gps_dyn_model` | AIR_2G | **AIR_4G** | C — > 2 g for 54–83 % of each span |
| `init_gyro_cal` | ON | **OFF** | calibration policy (operator-approved); also needed for in-flight re-arm after an FC reboot (R17) |
| `inav_w_acc_bias` | 0.010 | **0.000** | calibration policy: no bias learning from faulted GPS / baro residuals |
| `ins_gravity_cmss` | 943.520 | **979.700** | measured: still \|a\| = 0.9990 g (σ 0.0015, 79 samples over MSP) × 980.665. Boot-calibrated values seen: 943.5 (saved), **909.4**, **979.9** on successive boots ⇒ the boot calibration is unreliable, not the accelerometer |
| `gyro_zero_x/y/z` | −35 / 34 / 13 | **−35 / 32 / 9** | from the last boot with `init_gyro_cal = ON` (INAV accepts only a still calibration). Successive boots gave −25 / 29 / 10 and −35 / 32 / 9 ⇒ a ~0.3–0.6 °/s spread. To be re-recorded deliberately (checklist C4) |

Verified after save + reboot: all values read back. `status`: **GPS 1/1, Galileo 1/0, GLONASS 1/1 active**, BeiDou 0/0. **13 sats**
indoors (9–10 before), HDOP 1.37, EPH 1.76 m, EPV 2.65 m. **Nav rate stayed 10 Hz** (message interval 100–101 ms; no 5 Hz
fallback). `I2C Errors: 50` is the same immediately after a reboot ⇒ a boot-time probe count, not accumulating. Recheck it under
motor load (D2).

**Still owed before this becomes the config of record**: the outdoor 10-min multi-GNSS static (E3), the deliberate levelled
calibration (C1–C5), the compass alignment and calibration (C6, E4), and the failsafe / recovery decisions (R16 / R17).

## BLE download, measured remotely (F2 / R2)

Driven from the training host: Chromium 156 over DevTools ([`ble_cdp_probe.py`](ble_cdp_probe.py)), the picker answered
programmatically, external timing probes on the page's Web Bluetooth calls. The page and firmware are unchanged. 60 s and 30 s
samples of `flight_001.bin` (307 968 B); download only, no erase.

| run | chunk size | rate | round trip / chunk | errors |
|---|---|---|---|---|
| clean BlueZ state, 30 s | 240 B (no truncation) | **1.58 kB/s** | median **98 ms**, p90 244 ms | 0 |
| stale second BlueZ session, 60 s | 240 B | 1.56 kB/s unique | as above | **every notification delivered twice ⇒ 2 `NEXT` per chunk ⇒ 582 / 780 writes "GATT operation already in progress"**, then the download aborted |

Also: BlueZ caches the GAP name **"Arduino"** after a connection, so the page's `XIAO-GP` name filter misses the device on a host
that has connected before. Operator: Edge on a Windows laptop in the field is also slow. ⇒ the protocol (stop-and-wait, no
duplicate tolerance), not the phone. Research R2 updated.

## 2026-11-12 — board and compass alignment on the levelled reference (operator-approved) — ⚠️ NOT yet in `xiao/inav-hb1.cfg`

Setup: article 1 levelled on the **flat bottom of the wing** (the ≈ 0° angle-of-attack reference), **nose pointed at magnetic
north** (operator). FC now on `/dev/ttyACM0` (the ports swapped since 11-11). A >90 % pass; the deliberate calibration comes later
(operator). Location from GPS: 37.378 N, −122.035 E (20 sats indoors with multi-GNSS); field model for the site ≈ declination 12.9° E,
dip ≈ 61° (approximate).

**Method**: solved the board alignment from the measured gravity vector: the body-frame accel with the old alignment, rotated back to
the raw sensor frame (INAV applies `Mᵀ`), then the roll/pitch (yaw fixed at 90°) that maps it to straight down. Applied with the
whitelisted set → read-back → save script ([log](bench-20261111/art1_apply_align_20261112_log.txt)). INAV applies compass offsets and
gains in the **sensor frame before alignment** (`compass.c:461`), so the existing compass calibration stays valid.

| setting | config of record | now on the FC | why |
|---|---|---|---|
| `align_board_roll` | 1700 | **1736** | gravity-solved; with yaw 90° the board's roll is the aircraft pitch axis ⇒ the FC is tilted back **~6.4°**, not 10° |
| `align_board_pitch` | 0 | **7** | gravity-solved roll component |
| `align_mag_roll / pitch / yaw` | 0 / 0 / 0 (⇒ CW0 + board alignment) | **1736 / 7 / 900** | external path with the same rotation as the board (≡ CW270FLIP + the panel back-tilt). Explicit, so a future board change can't silently move the compass. Assumes the compass panel's tilt = the FC panel's tilt; refine with the C6 measurement |

| reading (levelled, nose magnetic north) | before | after |
|---|---|---|
| INAV attitude roll / pitch | −1.35° / +3.73° | **+0.06° / −0.21°** |
| accel tilt (body) | 3.6° (pitch), −0.7° (roll) | **0.04° / −0.03°** |
| heading | 11.4° | **12.9°** = local declination ⇒ magnetic north reads correctly with auto-declination |
| compass dip | 39.1° | 42.6° |
| \|a\| | 0.985 g | 0.986 g |

**What it settles**:
- The **3.7° nose-up attitude bias** that the NN's quaternion inputs carried relative to the airframe is gone (sim-vs-flight parity).
- Compass **heading** at north is right. The **dip is still ~18° below the field model** and moved only by the alignment change
  (+3.5°), so it is **not alignment**. It is the compass calibration (likely the vertical axis: `magzero` z −510, gains 1409 / 1347 /
  1370) and/or indoor field distortion. ⇒ Outdoor check (E4): dip ≈ 61° and the same \|m\| at N / E / S / W; if not, recalibrate
  outdoors.
- \|a\| = 0.986 g today vs 0.999 g on 11-11 ⇒ redo the 6-point accel calibration (C3), then re-record `ins_gravity_cmss`.
