# 045 — Research (plan phase, 2026-11-11)

Scope and division of labour: [README § Plan research scope](README.md#-plan-research-scope-agreed-2026-11-10-for-speckitplan).
Only **remote** research is done here: code and source reading, analysis of the committed logs, arithmetic. Items that need the
article are written as procedures in [quickstart.md](quickstart.md) and become bench tasks. Decisions that depend on them are
written as conditionals. This file is revised when those results come in.

Each item: **Decision**, **Rationale**, **Alternatives considered**, then the evidence.

---

## R3 / R4 — attitude over a span, and the attitude-stability budget

### Evidence (11-08 and 10-18 blackbox, engaged spans; scratch analysis `span_attitude.py`, numbers reproducible from the committed CSVs)

Engaged spans (`mspOverrideFlags` = 2): 11-08: 373.07–389.78, 406.18–428.07, 455.55–476.61, 493.05–514.30 s. 10-18: 399.81–406.08,
423.15–439.69, 460.04–482.18, 499.91–521.45 s. The blackbox runs at **240 Hz** (`blackbox_rate_denom` 8).

| flight · span | length | gross rotation roll / pitch / yaw | **net** rotation roll / pitch / yaw | max rate r / p / y (°/s) | specific force mean · p95 · time > 2 g |
|---|---|---|---|---|---|
| 11-08 · 1 | 16.7 s | 1554 / 1969 / 505° | −599 / **+1315** / +240° | 423 / 401 / 120 | 3.80 · 6.57 g · 83 % |
| 11-08 · 2 | 21.9 s | 1749 / 2215 / 588° | −54 / **+1350** / +19° | 345 / 346 / 82 | 2.85 · 5.36 g · 70 % |
| 11-08 · 3 | 21.1 s | 1920 / 2160 / 621° | −392 / **+1075** / +271° | 341 / 324 / 104 | 2.87 · 5.69 g · 68 % |
| 11-08 · 4 | 21.2 s | 1873 / 1977 / 593° | −483 / **+962** / +208° | 301 / 244 / 104 | 2.43 · 4.88 g · 58 % |
| 10-18 · 1 | 6.3 s | 1016 / 728 / 276° | −14 / +404 / −35° | 553 / 276 / 154 | 3.93 · 8.28 g · 74 % |
| 10-18 · 2 | 16.5 s | 1705 / 1512 / 510° | −448 / **+962** / +149° | 427 / 245 / 121 | 2.85 · 5.77 g · 66 % |
| 10-18 · 3 | 22.1 s | 2808 / 1959 / 736° | −167 / **+1108** / +194° | 454 / 301 / 149 | 2.73 · 6.01 g · 58 % |
| 10-18 · 4 | 21.5 s | 1989 / 1918 / 643° | −647 / **+858** / +172° | 477 / 302 / 116 | 2.55 · 5.78 g · 54 % |

- **The policy loops continuously one way**: net pitch +860 to +1350° per full span (≈ 3–4 loops), net roll −50 to −650°.
- Peak rates (≤ 553 °/s) are well inside the ±2000 °/s gyro range. Range is not a constraint.
- **Gyro-only attitude vs INAV's attitude**, propagated from engage with the axis signs fitted by least squares (roll +1, pitch −1,
  gyro FLU→FRD +1/−1/−1, matching `nav_accel_fit.py`): divergence of **1–15°**, not monotonic in time (e.g. 11-08 span 2:
  14.6° at 5 s, 1.4° at the end). The comparison's own floor is **~2.6–3.3° at 1 s**: filter lag (25 Hz PT1 ≈ 6.4 ms ≈ 2.6° at
  400 °/s), 240 Hz decimated logging of 400 °/s rates, and INAV's own corrections. ⇒ **INAV's attitude is not a usable truth
  reference for drift.** The truth reference has to be the GPS-derived specific-force direction (the `nav_accel_fit.py` method)
  or a later, faster-logged flight.

### Budget arithmetic (FR-006a)

Attitude error over a span from an un-aided gyro, per error source:

| source | model | magnitude over a 17–22 s span | what it takes to reach < 1° |
|---|---|---|---|
| **scale factor** | ε_s × net rotation (pitch net ≈ 1000–1350°) | 1 % → **10–13°**; 0.1 % → 1.0–1.3° | pitch scale known to **≲ 0.07 %** |
| cross-axis (misalignment) | ε_x × gross rotation of the other axes (~2000°) | 1 % → ~20° (partly cancelling); 0.1 % → ~2° | axis alignment ≲ 0.03° *and* in-run cross-axis calibrated |
| bias | b × T | 0.1 °/s → 1.7–2.2°; 0.02 °/s → 0.4° | in-run bias ≲ 0.04 °/s, i.e. bias re-estimated at engage |
| g-sensitivity | k_g × ∫\|f\| dt (mean 2.4–3.9 g) | 0.1 °/s/g → 4–8° | g-sensitivity ≲ 0.01 °/s/g or calibrated |
| sampling / coning | rate² × dt² terms; the integrator rate | small at ≥ 1 kHz; visible at 240 Hz (the ~3° floor above) | integrate at ≥ 1 kHz on raw samples |
| vibration rectification | foam-tape mount resonances × rate | unknown (R7: cannot be measured at 240 Hz) | measure first |

⚠️ Datasheet values (scale tolerance, cross-axis, g-sensitivity) for MPU6000 (article 1 FC), ICM42688P (article 2 FC) and the
xiao's LSM6DS3TR-C are owed in the N1 task. The table shows the shape of the budget; it does not certify a part.

### Decision

1. **Un-aided gyro integration cannot meet < 1° per span** on any of the three MEMS parts. Scale factor on the pitch axis
   dominates, because the policy accumulates ~1000°+ of net pitch rotation per span. Bias, the term people usually budget first, is
   second. INAV's AHRS is no better in manoeuvres. It ignores the accelerometer above 15 °/s, which is nearly all the time here, so
   it is effectively gyro plus mag plus GPS course. What attitude-dependent observation it gets comes from a compass whose alignment
   is in doubt (spec FR-008a).
2. **The N4 attitude source must be aided, and the aiding that works under load is GNSS velocity.** In a loosely coupled GNSS/INS
   filter, attitude becomes observable *through acceleration*: the specific force rotated to the earth frame must match the GPS
   velocity change. Sustained high-g manoeuvres make tilt *more* observable this way, not less (gravity-only levelling is what
   fails under load). Design target for N4: a span-local error-state filter (gyro + accel propagation at ≥ 1 kHz, 10 Hz GPS
   velocity updates, optional mag) that applies **smooth** corrections and never resets inside a span. GPS dropouts are coasted
   on the inertial solution.
3. **Calibration owed before any source can be judged**: gyro scale factor per axis (rotation-table or turntable-equivalent test,
   e.g. a measured number of hand turns on a level jig), bias at engage (the aircraft is not stationary at engage, so this is a
   filter state), and the board-to-airframe alignment (FR-007b).
4. **The < 1° target is revisited with the N1 numbers.** Its purpose is a stable path frame. A path that drifts smoothly by a few
   degrees is tolerable; a reset is not (spec § Why). If N1 shows 2–3° smooth drift is the realistic floor, the target becomes
   "smooth and < 3°", with the 1° number kept as the design aim.

### Alternatives considered

- **INAV's AHRS as the attitude source** (status quo): measured 10–25° heading/attitude scatter in pulls (044 analysis), compass in
  doubt, no tuning that makes it observe tilt under load (accelerometer is ignored above 15 °/s by design). Kept as the fallback
  and as a comparison stream.
- **Un-aided gyro from engage**: fails on scale factor (above).
- **An inertial part with on-chip fusion** (e.g. parts with a built-in game-rotation-vector engine): on-chip fusion is
  gravity-and-mag aided, i.e. the same failure under sustained load. It does not help the core problem; skipped unless N1 shows a
  sensor-quality problem rather than an aiding one.

## R7 — mount resonance (foam tape)

### Evidence

`gyroRaw` spectra over the engaged spans (both flights, all axes): the top peaks above 15 Hz sit at **102–113 Hz**, just under the
120 Hz Nyquist of the 240 Hz blackbox. That is **aliasing** of higher-frequency prop/motor and mount content, not a resolvable
mount mode.

### Decision

> 2026-11-12: the flight rate is set by the write bandwidth (possibly 1:32 with MAG + NAV_ACC), so (b) below is unlikely; the
> mount survey is bench-only.

The committed logs **cannot measure mount resonances**. Measure them with (a) a bench motor run-up per article with the blackbox at
a high rate (denominator 1–2), and (b) the re-fly logged at a higher rate if write bandwidth allows. The operator noted on
2026-10-04 that write bandwidth is limited, so trim the logged field groups to fit: [quickstart.md](quickstart.md) § Mount
survey. The xiao's own IMU has no logging today (R10), so its mount response is unmeasured until N4's shadow logging exists.

### Alternatives considered

Inferring resonance from the 240 Hz data (aliased; ambiguous). Accelerometer-only survey (accSmooth is 15 Hz-filtered;
useless for this).

---

## R1 — dmp cache chokepoint (F1)

**Decision**: implement the cache inside `s3GetDmpBlob` / `s3PutDmpBlob` (`src/util/s3_run_selector.cc:131` / `:151`) as the
README designs. Cache the **compressed** bytes: Get has the raw body before inflating (`:139`); Put compresses internally (`:145`).
Key it `<cache>/<bucket>/<key>`. The bucket matters: `source_dmp_loader` reads `trackerSourceBucket`, and `dmp-dump` takes the
bucket from the URI. Read `AUTOC_DMP_CACHE` once, in `s3_run_selector.cc`, as a function-local static beside the two functions.
Every tool links `autoc_common`, so all of them inherit it. This is the first production `getenv`; the env var is justified
because the Python reports spawn the tools (README § FRONT TASK).

**Rationale**: verified as the only `GetObject` / `PutObject` sites in `src/ tools/ include/ tests/`. Callers: `dmp_dump.cc:418,557`,
`nnextractor.cc:141`, `renderer.cc:433,437`, `source_dmp_loader.cc:72`; writers `autoc.cc:1513` (eval) and `:1738` (training). Python
makes **no** direct S3 calls (no boto3 / aws CLI); every report shells out to `dmp-dump` or `nnextractor`. The README suspected
`rnn_capacity`, but it reads only nn2cpp `.cpp` files.

**Known gaps (acceptable)**: `ListObjectsV2` in `listAll` (`s3_run_selector.cc:71`) and in the renderer (`renderer.cc:2097,2126,3156`)
bypasses the cache. LIST requests are small and are not egress-heavy; SC-001 counts `GET.OBJECT` only. Local-path reads
(`dmp_dump.cc:284`, `source_dmp_loader.cc:41`) already avoid S3.

**Alternatives considered**: a Python-side cache (misses the C++ tools); an S3 proxy / local MinIO (operational weight for no gain).

## R2 — BLE flight-log download (F2)

**Evidence** (code read, `xiao/src/bluetooth.cpp`, `xiao/web/flight_logger.html`, ArduinoBLE 2.1.0 over mbed Cordio):

- Protocol: `DL:<file>` → `READY:<size>:240`, then **strict stop-and-wait**. The page writes `NEXT` (write *with response*;
  control characteristic is `BLEWrite` only), the firmware reads 240 B from QSPI and sends one notification, repeat. The end is
  `DOWNLOAD_DONE:<crc>:<n>`; the page checks a whole-file CRC. One chunk in flight, ever.
- **MTU is never negotiated.** The firmware is a peripheral and never starts an MTU exchange (`exchangeMtu` exists only in the
  central path, `ATT.cpp:168`). **Notifications are silently truncated to MTU − 3** (`ATT.cpp:619`). The default MTU is 23. The
  firmware ignores `writeValue`'s returned length (`bluetooth.cpp:232`), and the page never checks `byteLength` or uses the parsed
  chunk size (`html:260`).
- Connection interval, data length and PHY are never requested.
- The main loop keeps running during a download. `controllerUpdate` does a blocking MSP request with a 50 ms timeout, so one HCI
  poll can wait 50–100 ms with the flight controller absent or slow.
- **No resume, no retry, and a wedge.** A failed `NEXT` write stops the page (`html:300-304`). The firmware keeps
  `transferInProgress` set and flash in `FLASH_DOWNLOADING`, so a new `DL:` is rejected (`flash_logger.cpp:491`) until reconnect.
  The HCI receive ring is 256 B and drops packets when full (`HCICordioTransport.cpp:317-319`). A lost completed-packets event can
  make TX flow control spin (`HCI.cpp:638`) until the 30 s watchdog resets the board.
- Throughput as written: 240 B per round trip ≥ 2 connection intervals ⇒ ~1.6–3.4 kB/s *if* the MTU were 247. That matches the old
  bench figure of 1–1.6 kB/s (`specs/archive/ZZZ-FLASH_SPEEDUP.md`). **0.5 kB/s ≈ 20 B per ~40 ms round trip**, which is exactly a
  23-byte MTU. A truncated stream should also fail the CRC, which fits "every field attempt failed".

**Measured 2026-11-11 (remote, from the training host)**: the downloader page driven in Chromium 156 over DevTools
(`ble_cdp_probe.py`): the device picker answered programmatically, and the page's Web Bluetooth calls wrapped with timing probes
from outside, so neither the page nor the firmware was modified. Captures in `bench-20261111/ble_probe_*.json`.
- **No MTU truncation from this host**: every notification was the full 240 B (BlueZ negotiates a large MTU). ⇒ The 23-byte-MTU
  suspect is **not** what limits a Linux central. It may still apply to some phones, and the instrumented page will show
  (`byteLength`). The operator reports **Edge on a Windows laptop in the field is also slow**, so the protocol is the common cause.
- **Clean run (30 s)**: 195 chunks, **1.58 kB/s**, stop-and-wait round trip **median 98 ms, p90 244 ms** per 240 B, 0 errors. A
  ~300 kB log would take ~3.3 min at the bench. Field range and RF retries stretch every round trip toward the field's ~0.5 kB/s.
  98 ms ≈ two connection intervals; the 244 ms tail matches the xiao's blocking 50 ms MSP poll stalling `loop()`.
- **Duplicate-delivery failure (60 s, earlier run)**: with a stale BlueZ session from an earlier connection, every notification
  reached the page **twice** (status and data alike). The page sent **2.01 `NEXT` per chunk**, and **582 of 780 writes failed** with
  "GATT operation already in progress", until the download aborted. A clean BlueZ state removed it. ⇒ The page must tolerate a
  duplicate or out-of-order chunk: offset-tagged chunks, one outstanding request, duplicates ignored. **This is a plausible
  mechanism for "every field attempt failed".**
- **Name filter**: after a GATT connection, BlueZ caches the GAP device name, which is still the Arduino default **"Arduino"**;
  the page filters on `name: 'XIAO-GP'`, so on a host that has connected before, the picker can't find the device. ⇒ Firmware:
  set the GAP device name too (`BLE.setDeviceName("XIAO-GP")`). Page: also filter by service, as a fallback.
- **Remote checks work**: connect, list and download can be driven from the training host with the xiao on the bench (no pairing
  or bonding needed). The F2 work can be measured remotely, before and after.

**Decision**:
1. **Instrument first** (operator's direction). Page: t0 at READY, per-chunk send→receive time, `byteLength` against the chunk
   size (detects truncation), offset, error name + offset on failure, disconnect offset and elapsed time, an in-memory log with a
   **Copy log** button. Firmware: log the negotiated ATT MTU at connect (`ATT.mtu(handle)`), compare `writeValue`'s return with
   bytes read, max loop duration, and report them in a `STAT:` status line and in `DOWNLOAD_DONE`. One bench download with the
   phone used in the field is the measurement of record.
2. Then fix, expected order: (a) negotiate MTU 247 and size chunks to MTU − 3, and **fail loud** on a short `writeValue`;
   (b) replace stop-and-wait with **windowed streaming**: notifications back-to-back up to the TX buffer limit, with the page
   acknowledging every N chunks by offset, **offset-tagged so duplicates are ignored** (the 2026-11-11 duplicate-delivery failure); (c) **resume** by offset (`DL:<file>:<offset>`) plus a per-chunk or per-window CRC, and
   a firmware inactivity timeout that clears `FLASH_DOWNLOADING`; (d) request a short connection interval and 2M PHY where the
   phone allows; (e) skip `controllerUpdate`'s MSP traffic while a download runs (disarmed only).
3. Acceptance as in spec SC-002 (< 60 s for ~300 kB; byte-identical after an interrupted resume).

**Alternatives considered**: L2CAP connection-oriented channels (not exposed by ArduinoBLE); USB-serial download in the field
(needs a laptop; does not solve the phone workflow).

## R5 / R6 — INAV estimator, AHRS, GPS, compass and calibration (from the fork source)

Facts below are read in `~/inav/src/main` (spot-checked where noted ✔); inference is marked *(inf.)*.

### What actually happened on 11-08, corrected

- ✔ **`inav_allow_dead_reckoning` played no part.** It is consulted only when no GPS sensor is present at all
  (`navigation_pos_estimator.c:471-474`). Losing satellites while the module still talks leaves the sensor present. The README /
  BACKLOG line "`inav_allow_dead_reckoning = OFF` ⇒ position frozen" is wrong. The freeze is EPH crossing `inav_max_eph_epv`,
  which makes XY invalid.
- When satellites drop below `gps_min_sats`, GPS_FIX clears (`gps.c:370-379`), but **EST_GPS_XY/Z_VALID stays set for 1500 ms**
  (✔ `INAV_GPS_TIMEOUT_MS`). For that time the estimate is pulled toward the **frozen last GPS position and velocity**, with gains
  divided by accWeight. Then XY integrates acceleration, velocity decays at `w_xy_res_v`, EPH grows as eᵗ (2 m → 10 m in ~1.6 s), XY
  goes invalid and position freezes. On reacquire, after two fixes, position and velocity **hard-reset** to GPS (`:656-661`). That
  is the 11-08 105 m step.
- **accWeight** = vibration factor (1 → 3 g of "vibe" maps to 1.0 → 0.30) × clip factor (0.5 on any axis > 15.9 g). The true
  floor is therefore 0.15. All corrections are **divided** by it. A hard pull's onset reads as vibration *(inf.)*, so correction
  gains spike 3–7× exactly when the estimate is weakest.
- **Altitude, `GPS` mode**: baro weight fades 1 → 0 as |GPS − baro| goes 10 → 20 m, with `max_eph_epv` = 10 m. That fade is the
  26 m Z reset. Baro is a fixed 1 Hz PT1 (~160 ms lag), climb rate is its derivative, and there is no dynamic-pressure or
  angle-of-attack compensation *(inf.: several metres of static error at speed and high AoA)*.

### AHRS under load

- Accelerometer weight = bell(|f_comp| − 1 g, σ 0.2 g) × rate-ignore. At 1.4 g it is 0.13, at 2 g ≈ 0. Rate-ignore runs between
  40 and 80 °/s with trustworthy GPS (×4 multiplier), 10 and 20 °/s without. **In this flying, roll and pitch are gyro-only nearly
  all the time.** The R3 table shows > 2 g for 54–83 % of every span.
- ✔ **Inertia-compensation fallback bug**: without trustworthy GPS (fix and **≥ 6 sats, hard-coded**), `currentspeed` stays 0
  and the pitot / reference-airspeed fallbacks test `currentspeed < 0` (`imu.c:687,694`), so they never fire. Compensation is
  zero exactly when GPS is weak.
- The gyro-bias integral freezes above 20 °/s spin (`imu.c:466,496`): **in-flight bias is never learned** here *(inf.)*.
- Yaw: mag weight by field magnitude; GPS course used when ≥ 6 sats and ≥ 3 m/s, weighted by `ahrs_gps_yaw_weight`. *(inf.)* GPS
  course lags 100–200 ms, which is 9–18° at 90 °/s, and on a plane at high AoA / sideslip course ≠ heading. With kp = 0.2 /s the
  heading is pulled toward a lagged, slipped course in sustained turns. A plausible source of the 10–25° scatter.

### Calibration (FR-007a)

- ✔ `init_gyro_cal` gates **both** the boot gyro zero and the boot gravity calibration. When OFF, `gyro_zero_*` and
  `ins_gravity_cmss` are used live. `setGravityCalibration` writes RAM only and persists on a later `save`, which is how 943.52
  reached the cfg. *(inf.)* 943.5 vs 980.7 cm/s² is 3.8 % low; accgain is within ±1 % of nominal, so the likely cause is a boot
  gravity calibration taken ~16° off level. The 1 g reference is in the earth frame, so an attitude error at boot reads as a gain
  error. With `init_gyro_cal = ON` it is recomputed every boot anyway, so **every boot inherits whatever attitude the aircraft sat
  at**.
- 6-point accel calibration: offsets plus per-axis scale from a sphere fit, no cross-axis term. ⚠️ Its result validation can never
  reject (`isnan && isinf` in `sensorCalibrationValidateResult`), so check the result by hand.
- Compass calibration: offsets from a sphere fit, a crude per-axis "gain", **no soft-iron matrix**, samples only after ~8° steps.

**Decision (calibration policy, conditional on the bench)**: (1) bench 6-point accel calibration on the levelled article (flat wing
bottom = 0° reference), result checked by hand; (2) **`init_gyro_cal = OFF`** with a recorded gyro zero and a recorded
`ins_gravity_cmss` taken level on the bench. Then the estimator's 1 g reference no longer depends on how the aircraft sat at boot;
(3) **`inav_w_acc_bias = 0`**: the bias accepts corrections up to 0.25 g, is fed by faulted GPS / baro residuals rotated through a
wrong attitude, and is unclamped; (4) outdoor compass calibration *after* the alignment fix (FR-008a). ⚠️ `init_gyro_cal = OFF`
means the gyro zero no longer tracks temperature or ageing: re-record it each session (quickstart).

### Compass alignment (FR-008a), confirmed

With all `align_mag_*` = 0 the controller applies `align_mag` (CW0) **then the board alignment** (roll 170°, yaw 90°) to the
external QMC5883 (`compass.c:465-482`). That is correct only if the GPS-mast compass is mounted parallel to the flight controller.
Fix: measure the compass's actual orientation and set it explicitly through `align_mag_roll/pitch/yaw`. Any non-zero value selects
the external path, which applies that rotation alone.

### GPS

- `gps_dyn_model` AIR_2G = u-blox 7, **AIR_4G = 8** (the highest u-blox offers; peaks of 5–7.5 g still exceed it). Sent once at
  boot.
- Multi-GNSS on M8: INAV's old-API path sends Galileo / BeiDou / GLONASS blocks (`configureGNSS`), with no cap on nav rate.
  ⚠️ *(u-blox docs)* M8 runs ~10 Hz with concurrent GNSS (18 Hz GPS-only), so **10 Hz is the ceiling with Galileo on**; keep
  `gps_ublox_nav_hz = 10`. The NAK path resets all three constellations to OFF in RAM, and a later `save` persists that: check
  with `status` after the save and reboot.
- **Article 1's receiver (web research 2026-11-11)**: the Matek M8Q-5883 is a u-blox **SAM-M8Q**, ROM SPG 3.01 (Galileo-capable):
  GPS, GLONASS, Galileo, BeiDou and QZSS, **up to 3 major GNSS concurrently**. Nav rate: 18 Hz single GNSS, **10 Hz with 2
  concurrent**; the rate with 3 concurrent is not stated in the summaries found. Galileo is **off by default**. The part is ROM-only
  with no flash: settings live in battery-backed RAM and are not persistent. That doesn't matter here: `gps_auto_config = ON` makes
  INAV re-send the GNSS configuration every boot. **No firmware update is possible or needed.** The module is discontinued, replaced
  by the SAM-M10Q (article 2's M10Q-5883). Note: the config of record sets GLONASS **off** explicitly, so article 1 has been flying
  **GPS-only** (the module's own default is GPS + GLONASS). That explains the 5–7 sats.
- **Decision (article 1)**: enable **GPS + Galileo + GLONASS**, and leave **BeiDou off**: a 4th major GNSS exceeds the M8's 3, the
  NAK silently reverts all three, and a later `save` persists the revert. Keep `gps_ublox_nav_hz = 10`. ⚠️ If the module NAKs 10 Hz
  with 3 GNSS, INAV falls back to **5 Hz** silently (`gps_ublox.c` rate path), and `status` does not print the nav rate. **Verify the
  rate** from the GPS frame interval in a bench blackbox (`*.gps.csv` time deltas). If it is 5 Hz, drop GLONASS (GPS + Galileo,
  2 concurrent, 10 Hz).
- `gps_min_sats` gates only GPS_FIX. AHRS trust, the inertia-compensation choice and CoG heading use a **hard-coded 6**.

### Decisions for the re-fly configuration (class C, plus calibration)

| setting | now | re-fly | why |
|---|---|---|---|
| Galileo (± GLONASS) | off | **on**, receiver permitting | 5–7 sats; every drop came in a pull |
| `gps_dyn_model` | AIR_2G | **AIR_4G** | > 2 g for 54–83 % of each span |
| `gps_ublox_nav_hz` | 10 | 10 | M8 ceiling with concurrent GNSS |
| `inav_default_alt_sensor` | GPS | **GPS_ONLY**, if the multi-GNSS EPV is ≤ ~5 m on the bench; else stays GPS | removes the baro fade (26 m reset); baro uncompensated for dynamic pressure |
| `inav_max_eph_epv` | 1000 cm | **2000–3000 cm** *(to pin from N1)* | the cap is what turns a coast into a freeze + reset. Raising it keeps the estimate valid through short dropouts; the reset on reacquire remains. ⚠️ trades validity for honesty — N3's flags record EPH regardless |
| `inav_allow_dead_reckoning` | off | off | irrelevant to sat loss (above) |
| `inav_w_acc_bias` | 0.01 | **0** | above |
| `init_gyro_cal` | on | **off** + recorded zeros / gravity | above |
| compass alignment | via board | **explicit** | FR-008a |

The hard reset on reacquire and the 1500 ms frozen-GPS pull are code behaviour; no setting changes them. They are N4's argument
(a companion frame that never resets), not something INAV configuration can fix.

**Alternatives considered**: tuning `w_*` fusion weights first (rejected: 043 warned not to lean harder on a baro that reads
wrong; weights do not address the reset); patching the fork's estimator (reset smoothing, the inertia-compensation fallback bug):
a candidate for N6 after G2, not before the re-fly (it would be a fourth build surface before the gate).

## R10 — companion inertial frame feasibility (N4, shadow)

**Evidence**: the LSM6DS3TR-C is unused today (no IMU code or library). It sits **alone on the internal Wire1** (SCL P0.27, SDA
P0.07, power-enable P1.08 must be driven high, INT1 P0.11). Everything runs in Arduino `loop()`: the 20 Hz tick accumulator, a
blocking MSP request (50 ms timeout), non-blocking QSPI logging, BLE (stopped at arm). Measured flight pipeline p50 / p95 / max =
12.4 / 21.6 / 42 ms; 0 overruns on recent flights. RAM 53.6 %, flash 45.8 %; NN ~1.5 ms mean.

**Decision**:
- Read the IMU at **416 Hz** (208 Hz fallback) through its **FIFO with an INT1 watermark**, drained in small batches every loop
  pass *and* inside the MSP wait loop, so a 35–50 ms MSP stall (~21 samples) is absorbed. Stock mbed `Wire` blocks; a 250 B burst
  is ~6 ms, so batches stay small, or use async TWIM DMA if needed. Integrator + filter: ~30–60 µs per sample ⇒ 2–3 % CPU.
- **Engage handover**: INAV's state at engage is 3–35 ms old (fetch time) and its timestamp is ms-truncated. Keep a short gyro ring
  buffer and propagate INAV's attitude forward from its sample time to "now".
- **Logging**: IMU-rate records do not fit (~7 KB/s vs ~3 KB/s today). Log the shadow state at 20 Hz in a **new record type**
  (FlightStateRecord precedent, ~25–30 B), plus optional decimated raw IMU bursts. ⚠️ **The v6 budget is nearly full**: 148 B/tick ×
  9600 ticks against 70 % of 2 MB leaves ~47 KB (`flight_log_format.h:399-404`). Log v7 (N3 + B1 + shadow) needs a re-budget:
  fewer ticks per session, a larger log region, or tighter encoding. Decided in data-model.md.
- Owed: the xiao board's orientation in the airframe (undocumented; 037 proposed auto-calibrating it from INAV), gyro bias /
  scale calibration (R4), and a watchdog for IMU stalls (today's is 30 s).

**Alternatives considered**: a separate RTOS thread for the IMU (priority interplay with the Cordio BLE thread; not needed at
2–3 % CPU); fusion at 20 Hz on decimated samples (fails the R4 sampling term).

## R12–R15 — readiness scan (phases 2–5; depth deferred to the post-G2 plan revision)

- **R12 camera**: analytic equidistant projection, 320×200 at 0.304 °/px (97.3° × 60.8°, 1.8 mm lens), `CameraDegPerPixel` defined
  in **five** places. 040 variations: boresight σ 4°, roll σ 4° (the ini itself doubts their "M" class), mount translation σ
  2 / 4 / 1.2 mm, wing thickness 0.8 mm (A), ambient 0 (deferred). Gaps vs measurement: detection range 100 m asserted (measured
  locks 4.5–12.5 m, photodiode only); ambient knee anchored to a photodiode while the sim models an array; 1.56 mm lens unmeasured;
  warm relock possibly pessimistic (sim 154 ms credit vs bench "0–2 frames" inside HOLD); no narrow-field / rotation-rate lock
  loss; filter FWHM unknown (sun pedestal); prop attenuation computed but unused (`camera_projection.cc:152-156`).
- **R13 perfect signal**: one branch in `projectPerceptionTick`'s `refine` lambda (`tracker_tick_rule.cc:82-118`). When perfect and
  visible: force TRACKING, q = 9, skip CDMA, CEP = 0.02. Plumbed as a `TickRuleConfig` field with no default (Constitution VII),
  a WorkerInit field, an ini key and a startup log line. **Open decision**: the emission-darkness gate (`camera_projection.cc:128`)
  and the 100 m cutoff (`:139`) are *signal*, so they are off in perfect mode. FOV, behind-camera and obstruction stay on as
  *geometry*. The shared-element (two beacons on one pixel) identity flag is also *geometry* and stays on.
- **R14 input-domain check**: `dmp-dump --csv-only` gives every slot in NN units with names; `flightlog_decode.py -o` gives the
  flight side. Nothing reports bounds or fraction-at-bound for all inputs, so a small new script under `specs/045-m2-tracking/` is
  needed. ⚠️ **Correction to spec FR-022**: only CEP is an int8 quantity. Bearings are fp32 radians quantised to the pixel grid,
  `beacon_pair_span` is raw radians, and `span_rate` is Δspan / 0.05 s. So the tracker inputs are *not* already in a small integer
  range, and the domain check matters for them.
- **R15 tracker pipeline**: tracker unit tests are registered and green on layout v4 (ctest 55/55 at fb4d3ba), but there has been
  **no end-to-end tracker run since 040 (2026-08-06)**. `autoc-tracker.ini:39` and `autoc-eval-tracker.ini:62` pin pre-schema-6
  source dmps, so tracker startup will **fail loud** until they are repointed. Repoint to **044-t1 g800**
  (`autoc-9223370244339188361-2026-10-20T16:59:47.446Z/gen9200.dmp.zst`, already 800/800 `retain=keep` per 044 outcome.md). With
  `TrackerChaseUseSourceScenarioSeed = 1` its path × wind table must match 1:1. Open 041 hand-over: predictor head (T081–T088),
  output topology 7 ↔ 3, innovation channels (T092–T094), repoint and bake (T095–T098). **No M2 smoke ini exists**; G4 needs one
  (`autoc-basic-m2.ini`, logging the signal mode).

## R8 — INAV's role during autoc (N6)

**Evidence** (fork source, `~/inav/src/main`; config `xiao/inav-hb1.cfg`):

- The "autoc override" is INAV's **standard** `USE_MSP_RC_OVERRIDE`; the fork adds only the state message. The xiao sends
  `MSP_SET_RAW_RC`, which is merged in `rx/msp_override.c:148-213` when `BOXMSPRCOVERRIDE` is active and the override is not in its
  own failsafe.
- **The ACRO rate loop reads no estimator.** rcCommand (`fc_core.c:389-436`) → `pidRcCommandToRate` (`pid.c:438`; rates
  360 / 240 / 40 °/s, clamp ±1800) → the fixed-wing rate controller (`pid.c:791-843`: gyro, target, gains only) → mixer / servos.
  Attitude readers in the PID path (level, turn assist, heading hold, yaw I-term bank freeze) are all inactive in ACRO or disabled
  by config. Nav still runs every loop but does not touch rcCommand in NAV IDLE. **⇒ Nav or attitude faults cannot change how the
  rate loop flies.** They reach the policy only through the state message to the xiao: the quaternion is GPS-influenced through
  centripetal compensation and course-over-ground yaw.
- **MSP loss alone is not a failsafe**: the override times out after 200 ms and needs 200 + `failsafe_delay`×100 = 1200 ms to drop
  out. *(inf.)* So if the xiao stalls, INAV holds its **last command for ~1.4 s** (200 ms signal timeout + 1200 ms override failsafe; R16) before passing the pilot's sticks through.
  `failsafe_procedure = DROP` fires only on loss of the main receiver, and disarms.
- **RTH needs** pos + vel TRUSTED, heading USABLE, `GPS_FIX_HOME` and position fresh within `nav_position_timeout`. With a
  faulted estimate it falls back to emergency landing.
- ⛔ **Safety finding: MANUAL is on an overridden channel.** MANUAL (permanent id 12) is `aux 2 12 1 900 1200`, i.e. RC6 low band.
  `msp_override_channels = 47` includes bit 5 (RC6), which the xiao holds at 1500. **While the override is active the MANUAL switch
  does nothing.** The xiao comment at `xiao/src/msplink.cpp:1093-1095` ("MANUAL's own switch (aux 4 on RC8) stays with the pilot")
  is wrong: `aux 4` is permanent id 10, **NAV RTH**, and RC8 carries POSHOLD / RTH. The working pilot escape is the **RC9 MSP RC
  OVERRIDE switch** (`aux 6 50 4`), which is also the xiao's engage switch: off ⇒ pilot sticks pass through at once, and MANUAL is
  selectable again. RTH on RC8 is *not* a safe escape with a faulted estimate.
- Gates that do not depend on the estimator and could bound the aircraft during autoc: rate caps (already 360 / 240 / 40 °/s),
  `rate_accel_limit_roll_pitch` (0 = off), pidsum / I-term limits, throttle limits (`throttle_idle`, max throttle,
  `throttle_scale`), servo travel. Angle limits apply only in ANGLE / HORIZON; geozone and `nav_max_altitude` need the estimate or a
  nav mode. **INAV has no estimator-free altitude floor in ACRO.**
- The xiao hard-codes box bit positions ARM = 0 and FAILSAFE = 19 (`xiao/include/MSP.h:77,96`), but the bitmask is ordered by the
  *configured* boxes (`fc_msp_box.c:468`). *(inf.)* A mode added or removed in the config can silently shift them.

✔ **Verified in source 2026-11-12 (RC9 escape)**: `calculateRxChannelsAndUpdateFailsafe` (`rx/rx.c` ~442–512) fills the channels
from the receiver, then merges the MSP values only `if (IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) && !mspOverrideIsInFailsafe())`. The
override box's own aux channel is removed from the override mask (`mspOverrideCtrlChannels`, `rx/msp_override.c:99,219`), so the
companion can never hold RC9. The box is re-evaluated in `updateActivatedModes()` (`fc/fc_core.c:669`) just after the channel
calculation (`:634`), so the hand-back takes about one RX frame. ⇒ **With the TX link up, RC9 off is an INAV-side escape that works
whatever the xiao is doing**, including a xiao that rebooted and re-engaged by itself (R18). Without the link, RC9 is frozen at its
last good value and the receiver-failsafe path applies (R17).

**Decision**:
1. **Before the re-fly** (class D, no fork change): correct the xiao comment and the pilot brief. The escape is RC9 override-off,
   then MANUAL. Bench-verify in [quickstart.md](quickstart.md) that RC9-off passes the pilot's sticks through immediately while
   engaged. Verify, or derive at boot, the box bit positions. Record the ~1.4 s "last command held" behaviour on xiao stall in the
   pilot brief.
2. **The reduced-role design (written now, built after G2)**: INAV during autoc = rate loop + override + receiver failsafe +
   estimator-free gates. An estimator-free altitude floor (baro-relative to engage, with its lag stated) and a stall watchdog on
   the xiao side are the candidates INAV lacks. The full estimator is kept only for RTH after disengage. Deferring the fork change
   keeps the fork out of the re-fly's build surface.
3. Moving MANUAL off RC6 (so it stays with the pilot during autoc) is a **config change to record and bench-test**, operator's call.
   It changes the pilot's switch layout, so it is not made silently. Listed as an open operator decision in quickstart.

## R9 — estimator fault fields (N3) and blackbox fields (B1)

**Evidence**:
- The message the xiao reads is `MSP2_INAV_LOCAL_STATE` (0x210E, `fc_msp.c:672-725`): 64 bytes — timestamp_us, pos (cm, NEU),
  vel (cm/s), quaternion, box bitmask, rc ch8 / ch9, gyroADCf (×10), accADCf (mg). The xiao mirrors it in `msp_autoc_state_t`
  (`xiao/include/MSP.h:261-283`). `state_valid` means *the fetch succeeded*. A false value **does** stop autoc
  (`kReasonMspStateFailure`), which is existing behaviour and unchanged by N3.
- Available to add, with no new estimator code: `posEstimator.est.eph/epv` (or the u16 `navEPH/navEPV`), `posEstimator.flags`
  (EST_GPS_XY_VALID / Z_VALID …), the packed `navFlags`, `posEstimator.imu.accWeightFactor`, `posEstimator.imu.accelBias`,
  `gpsSol.numSat / fixType / hdop / eph / epv`, `posEstimator.gps.lastUpdateTime`, baro altitude.
- Blackbox today: navEPH / navEPV (NAV_POS group, on), navFlags (always), numSat (G-frame, on change), at **250 Hz** effective (2 kHz
  loop, denom 8). Nav fields refresh at 50 Hz and repeat. **Not logged**: accWeight (only under `debug_mode = VIBE`, which costs the
  debug slots), the accel bias, `posEstimator.flags`. NAV_ACC, MAG and QUAT groups are off.

**Decision** *(N3 companion side deferred 2026-11-12: flagged offline from the blackbox instead; B1 stands)*:
- **N3 (state message, INAV fork + xiao)**: append to the state message: `est_eph` (u16 cm), `est_epv` (u16 cm), `est_flags` (u8:
  XY valid, Z valid, GPS-XY valid, GPS-Z valid, baro valid), `gps_numsat` (u8), `gps_fix` (u8), `acc_weight` (u8, ×100) and
  `gps_age_ms` (u16, since the last GPS update). That is 10 bytes, 64 → 74. The xiao derives *frozen* (velocity exactly zero and
  position unchanged while armed and airborne) and *stepped* (Δpos − v̄·Δt residual > threshold) per tick, the `nav_span_scan.py`
  logic moved on-board. It sets advisory flag bits in the tick record. **No new disengage**; `kReasonMspStateFailure` stays as
  today. Layout in [contracts/msp-autoc-state-v2.md](contracts/msp-autoc-state-v2.md).
- **B1 (blackbox)**: turn on **MAG** (heading and compass analysis) and **NAV_ACC** (earth-frame acceleration, for N1 without
  reconstruction). accWeight and the bias stay unlogged in the blackbox; they travel in the state message (N3) and land in the xiao
  log at 20 Hz, which is enough. For the mount survey, a separate high-rate logging configuration ([quickstart.md](quickstart.md)
  § Mount survey). `debug_mode = VIBE` is not needed once accWeight travels in the state message.
- **Log v7 consumers to update** (Constitution III/V, all in one change): `xiao/include/flight_log_format.h` (structs, static_asserts,
  version check), `xiao/src/flight_log.cpp`, `src/analytics/flightlog_decode.py` (versions, struct formats), `xiao/web/flight_logger.html`
  (versions, offsets, CSV columns), `tools/renderer.cc`, `tests/flightlog_roundtrip_tests.cc` + the CMake fixture, and the scripts
  importing the decoder (044 `bench_join.py`, 039 `correlate_blackbox.py` / `flight_report.py`, three 043 scripts — the historical
  ones per the immutable-scripts rule read only old logs and are left alone). v5 / v6 readers must refuse v7.

## R11 — re-fly test card and analysis protocol

**Decision** (draft; the card itself is in [quickstart.md](quickstart.md) § Re-fly):
- Article 1, g800 unchanged (`a5ed3521ea1e70b7`). Same streamer configuration as 11-08 and 10-18. Four or more engaged spans, plus
  a **≥ 60 s straight / box / circle segment at altitude in ANGLE or MANUAL** before or after the spans, for the heading check
  (`nav_heading_check.py` needs ≥ 4 course sectors; 11-08 had two).
- Optional, operator's call, costs seconds: MANUAL pitch / roll doublets for the airframe fit (the SC-002 discriminator in the
  backlog).
- Analysis: the 11-08 tools, plus the N3 flags (faulted spans listed and dropped with `flight_gates.py --drop`), plus the shadow frame
  vs INAV vs GPS-derived specific force (if US6 is adopted), plus the class-C configuration effect on EPH / EPV / sat count against
  11-08.
- Article 2, if it flies: its two outstanding config fixes (`align_mag_yaw = 3600`, `vbat_scale = 1110`) applied and read back first,
  plus the same calibration and alignment pass; analysed separately.

## R6b — closer recalibration of the flight hardware (operator 2026-11-11)

Operator: turn off INAV's automatic gyro and gravity calibration (approved: `init_gyro_cal = OFF`, `inav_w_acc_bias = 0`), and take a
closer look at calibrating the flight hardware (accelerometer, alignment, compass, …).

**Principle**: INAV's calibrations are crude and self-certifying. The accel fit has no cross-axis term and a validation that can
never reject; the compass fit has offsets plus a crude per-axis gain and no soft-iron matrix. So **calibrate with INAV's tools, then
verify independently from logged raw data**, and record the residuals. Residuals INAV cannot correct (cross-axis, soft iron) are
recorded as known errors for the R4 budget, and the companion frame (N4) can correct them for its own sensor.

| sensor / item | calibrate (INAV) | verify independently (offline, logged data) | pass |
|---|---|---|---|
| **accelerometer** | 6-point on the levelled article | static poses in **≥ 12 orientations** (the 6 faces plus 6 diagonals, 10 s each, blackbox ACC); an offline ellipsoid fit gives the residual offset, scale and **cross-axis** terms | \|f\| = 1.000 ± 0.005 g in every pose; cross-axis < 1 % (else record) |
| **board alignment** (roll / pitch) | `align_board_*` until attitude reads 0 / 0 on the level reference | (a) level-boot gravity on the reference; (b) the re-fly's in-flight fit (`nav_accel_fit.py`: roll / pitch sign and heading offset) | ≤ 0.3° on the bench; in-flight best offset ≈ 0° |
| **board alignment** (yaw) | the FC mounting arrow vs the fuselage centreline, measured | in flight: GPS course vs AHRS heading in straight, unaccelerated flight (`nav_heading_check.py`, constant term) | constant offset < 3° after wind crab |
| **gyro zero** | recorded level and still, per session (`init_gyro_cal = OFF`) | 60 s static log: the residual mean rate per axis | < 0.05 °/s |
| **gyro scale factor** | INAV has no gyro scale calibration | 10 turns per axis on a jig: ∫gyro vs N·360° | record per axis; > 0.5 % ⇒ note in the R4 budget (INAV can't correct it; N4 can) |
| **compass alignment** | explicit `align_mag_*` from the measured module orientation | heading at 8 known bearings on the ground, nose level | < 5° everywhere |
| **compass calibration** | outdoor tumble, away from metal | log MAG (blackbox) through a second outdoor tumble; offline ellipsoid fit ⇒ residual hard-iron and **soft-iron** | \|m\| variation < 5 % over the sphere; soft-iron recorded |
| **compass vs motor current** | — | bench, props on, restrained: heading drift vs throttle 0 → 100 % (logged MAG + motor) | < 5° at full throttle, else move the mast / twist the battery and ESC leads, or note it |
| **baro** | — | static 10 min (drift) and a prop-on run-up (prop-wash pressure step) | record; feeds the `GPS_ONLY` decision |
| **xiao IMU** (N4) | none in INAV | 6 static faces (orientation + accel offset / scale), 10 turns per axis (gyro scale), 60 s static (bias) | recorded as the shadow frame's calibration file |

In-flight verification on the re-fly (needs the card's box / circle segment): heading harmonics vs course (mag residual), the
alignment fit, and |f| statistics in unaccelerated segments.

**Tooling owed**: `cal_verify.py` (static-pose ellipsoid fits for accel and mag from a blackbox decode, gyro static residual,
turn-count scale factor) in `specs/045-m2-tracking/`. The procedures are in [quickstart.md](quickstart.md) §§ 2–5.

**Alternatives considered**: trusting INAV's calibrations (self-certifying, no cross-axis or soft-iron); a full external calibration
rig (overkill: hand jigs plus logged verification reach the needed accuracy, except gyro scale below ~0.1 %, which is N4's
in-filter job).

## R16 — autoc failure-mode behaviour (operator 2026-11-11: "dig deeper on what behaviour is required")

Traced in source (INAV `autoc` @ 1fc3b257c, xiao, `inav-hb1.cfg`). ✔ = spot-checked by hand; *(inf.)* = inference. The xiao forcing
ACRO through RC6 during `autoc=y` is **intended** (operator). The question is what happens when something goes wrong.

### Core mechanisms

- **Held command after the last frame**: the MSP signal drops 200 ms after the last frame, then the override failsafe needs
  200 + `failsafe_delay`×100 = 1200 ms more ⇒ **INAV holds the xiao's last command ≈ 1.4 s** after the xiao goes quiet. Recovery
  of the override needs 700 ms of good frames.
- **The xiao never sends a neutral or hand-back frame.** It sends only while `rabbit_active`; every `stopAutoc` just goes quiet.
  ⇒ **Every stop except RC9-low and disarm is followed by ~1.4 s of the last command held**, in ACRO, then the pilot's channels
  (including the pilot's RC6 mode switch).
- **Nav modes win over the override**: nav runs after the stick and failsafe processing and overwrites roll, pitch and throttle;
  nav also forces ANGLE.
- **Servos stay live when disarmed**: the rate PID has no armed gate, so the servos keep tracking the held command (P + FF + D, no
  I), and then the pilot's sticks. They are neither centred nor sent to failsafe values.
- ✔ **Box bits**: `MSP_MODE_FAILSAFE = 19` is hard-coded (`xiao/include/MSP.h:96`), but the bitmask is ordered by the *configured*
  box list. *(inf.)* For this config bit 19 is **NAV ALTHOLD**, and FAILSAFE is ~bit 29. So the xiao's "failsafe" stop actually fires
  when RTH / POSHOLD engage, and it **never sees a real failsafe**. Bench-verify with `MSP_BOXIDS`.

### Scenario table

| # | event during autoc | INAV | xiao | net aircraft | hazard |
|---|---|---|---|---|---|
| S1 | pilot selects **RTH** (RC8 high) | with valid nav: nav takes over at once (ANGLE, nav roll / pitch / throttle). With invalid nav or < 5 m from home: **silently refused** | stops via bit 19 *(inf.)*; it cannot see RC8 | RTH, or nothing happens | no feedback on refusal; RTH is not a dependable escape with a faulted estimate |
| S2 | pilot selects **POSHOLD** (RC8 mid) | as S1 (needs pos / vel / alt valid); the held xiao pitch / roll nudge climb and loiter | as S1 | loiter, or nothing | as S1 |
| S3 | **ground transmitter / receiver link lost** | 0–~1.2 s: autoc continues (pilot channels frozen). Then ✔ **DROP ⇒ in-air disarm** in the same cycle; motor off; servos follow the held xiao command ~1.4 s, then the frozen pilot sticks and mode | stops on the disarm (not via the failsafe bit), closes its log | **unpowered, surfaces where the last commands left them, no pilot** | in-air disarm by design of DROP; re-arm needs ≥ 3 s of link + ARM toggle + low throttle |
| S3-RTH | same, with `failsafe_procedure = RTH` | forced RTH. Bad heading / alt / home ⇒ emergency landing at once; bad position ⇒ 5 s then emergency landing: motor off, ~10° dive or held descent, a **20° left-bank spiral glide** if position unusable, until landing is detected | irrelevant | glide / spiral down | relies on the nav estimate this feature distrusts; failsafe exit can hand control back to a still-sending xiao *(inf.)* |
| S3-LAND | `failsafe_procedure = LAND` | the same emergency controller, then **disarm at `failsafe_off_delay` (~21 s), airborne or not** | — | spiral glide, then a timed disarm | timed in-air disarm |
| S4 | **xiao stops sending** (crash, reset, UART) | last command held ~1.4 s, then the pilot's channels incl. RC6 | ✔ after a reboot: `servo_reset_required` boots **false** ⇒ if still armed with RC9 high, **autoc re-engages by itself** ~8 s later with a new origin. The watchdog is 30 s ⇒ ≥ ~38 s after a hang | ~1.4 s constant rate, then pilot; then a surprise re-engage | **> 1 s constant command; unexpected re-engage** |
| S5 | **xiao live but commanding garbage** (logic bug, NaN) | nothing detects a constant or bad command (only frame arrival and pulse range) | bounded only by the 60 s run timeout, path end, RC9, disarm, MSP failure or bit 19. *(inf.)* An early return on `flight_path.empty()` skips the timeout | up to **60 s** of a bad rate command | **escape = RC9** |
| S6 | any `stopAutoc` (timeout, path complete, MSP failure, …) | ~1.4 s last command held, then pilot | quiet; re-engage needs an RC9 cycle | ~1.4 s constant rate | **> 1 s hold on every stop** |
| S7 | pilot **RC9 low** | override off at the next RX frame; pilot channels pass straight through | stops ≤ 50 ms | clean hand-back | — (the escape that always works) |
| S8 | GPS / nav estimate bad, no pilot action | ACRO unaffected; estimator reports vel_xy = 0 | flies on stale position until the path or 60 s ends (N3 flags only, by decision) | policy on a bad frame | known; N3/N4 |
| S9 | battery, auto modes | no automatic battery action; FW_LAUNCH finishes before engage; `motorstop_on_low` normal | — | — | ⚠️ adding bit 4 (RC5 = ARM) to the override mask would read 1500 ⇒ disarm |

**Pilot escapes during autoc**: **RC9 low** (always works); the ARM switch (disarm after 250 ms); RC8 RTH / POSHOLD (only with a
valid estimate); RC6 MANUAL / ANGLE (**not available** while overridden).

### Required behaviour — proposals (operator decisions)

> ⚠️ **Staged (operator 2026-11-12, spec § Clarifications)**: nothing below flies on the re-fly. The changes form a ladder after
> it, one per step, each bench-proven and flown on a happy-path flight before the next; R-3 starts with a measurement. The "when"
> column records the 2026-11-11 answer.

| # | requirement | proposal | when |
|---|---|---|---|
| B-1 | a stop must not hold a turning command | on every `stopAutoc`, the xiao sends **neutral rate frames** (roll = pitch = 1500 ⇒ zero rate, "stop rotating"; **throttle minimum ⇒ motor stop**, per the operator's throttle-to-0 principle). ⚠️ Zero rate *holds the current attitude* (it may be inverted or vertical). The operator's first goal is only "no continued powered flight"; the surface behaviour is a separate, per-attitude decision (spec FR-015b (2), R17) until the override times out, so the 1.4 s window holds wings-steady instead of the last command. INAV cannot shorten the override timeout separately from `failsafe_delay` | before the re-fly |
| B-2 | no re-engage after a xiao reboot in the air | boot with `servo_reset_required = true` (RC9 must be cycled after any boot) | before the re-fly |
| B-3 | the failsafe stop must see real failsafe | map ARM / FAILSAFE / OVERRIDE / NAV modes by **permanent id** from `MSP_BOXIDS` at boot, fail loud if missing (contracts/msp-autoc-state-v2.md) | before the re-fly |
| B-4 | a wedged or garbage-producing xiao is bounded tightly | fix the `flight_path.empty()` timeout skip; NaN / out-of-range NN output ⇒ stop (+ B-1 neutral); a loop-alive watchdog of ~1 s for the control loop (the 30 s watchdog stays for the board) | before the re-fly |
| B-5 | the xiao knows about receiver failsafe | add `rx_link_ok` and `failsafe_phase` to the state message (contracts v2) ⇒ stop + neutral on failsafe | before the re-fly |
| B-6 | **receiver-loss procedure** | keep **DROP** (in-air disarm: predictable, no reliance on the nav estimate) *or* **RTH** (glide / spiral when nav is bad; relies on the estimate) *or* **LAND** (spiral then a timed disarm). Field rules and the flying site decide | **operator** |
| B-7 | escape brief | RC9 low is the escape; RC6 does nothing while engaged; RC8 only with a valid estimate | before the re-fly |
| B-8 | MANUAL reachable during autoc (optional) | move MANUAL off RC6, or accept RC9-then-MANUAL | ✅ **decided 2026-11-11: keep; escape = RC9 off** |

None of B-1 … B-5 changes how the policy flies while engaged. They change only what happens at and after a stop, so the re-fly
remains a g800 test (FR-017). They are bench-testable (quickstart § 6).

## R17 — rapid recovery and "no continued powered flight" (operator 2026-11-11)

Operator priorities (spec FR-015b): **(1) no continued powered flight: throttle to 0 first, in every fault**; (2) surface
behaviour chosen per attitude with care (all-attitude flight); recover as fast as the configuration allows; in-flight re-arm wanted;
the companion must not be able to load the flight controller. Traced in the fork source (✔ = checked by hand).

### Time budget (11-08 / 10-18 GPS)

Peak descent rate 23–24 m/s (p99 14–20), peak 3-D speed ~28 m/s. From 100 m (330 ft) at 24 m/s down: ~4.2 s to the ground. A
pull-out uses ~16 m at 4 g, ~32 m at 2 g. ⇒ **~2.9 s from the fault to the start of a recovery at 100 m, ~3.75 s at 120 m.**
Every timeline below is measured against that.

### Mechanisms

- **The stage-1 window**: for 200 + `failsafe_delay`×100 ms after link loss (1.2 s now), there is **no failsafe action**. The
  pilot's channels are held and a live companion keeps flying **with its throttle**. The same `failsafe_delay` also sets the
  companion-loss window (200 ms + 200 + `failsafe_delay`×100 = 1.4 s). **`failsafe_delay` is the only setting that shortens both.**
- Loss detection: SBUS (article 1) flags failsafe in-frame, so detection is immediate. CRSF (article 2) has no flag: loss is "frames
  stop" + 100 ms. A **transmitter reboot** looks like frames stopping; ELRS resync takes ~1–3 s on the radio side, then INAV needs
  200 + `failsafe_recovery_delay`×100 ms (0.7 s now) of good data.
- ✔ **The flight controller has no watchdog** (`HAL_IWDG_MODULE_ENABLED` is commented out, `target/stm32f7xx_hal_conf.h:67`). A
  HardFault stops the motors and halts. A **hang** with no fault leaves the PWM timers putting out the last pulses: **surfaces frozen
  and the motor still running** *(inf.)*. This is a powered fly-away with no recovery path.
- **FC brownout or reboot**: ~2–2.5 s to the main loop; servos centred once the outputs initialise (limp or held before that,
  depending on the servo); boots **disarmed**; while disarmed and calibrating, INAV forces MANUAL passthrough (the pilot can glide
  on the sticks, motor off). Re-arm is blocked in practice: with `init_gyro_cal = ON` the gyro and gravity calibrations restart on
  movement and never finish in flight. `OFF` (already decided, R5) clears them, but **the baro calibration is not gated by it** and
  only finishes below ~2.5 m/s vertical speed, so it never finishes in a dive. Emergency arming cannot override
  SENSORS_CALIBRATING. **There is no in-flight-restart support.**
- **In-air re-arm paths**: after DROP: link up + 3 s of good data + ARM switch off, then on with low throttle (≈ 4 s after the link
  returns). Emergency arming: 10 arm-switch toggles in 10 s, which **cannot** override FAILSAFE_SYSTEM, SENSORS_CALIBRATING, THROTTLE
  or RC_LINK, so it doesn't help after DROP or a reboot. The in-flight re-arm window: a hard-coded 5 s, only after a *switch*
  disarm, and only with a valid GPS heading.
- **ANGLE recovery rate**: rate target = angle error × `fw_p_level`/6.56 (≈ 3.05 /s at 20), clamped to 360 / 240 °/s (roll /
  pitch), 5 Hz LPF. From 90° bank: within ~5° in ~1 s. ⚠️ From inverted or vertical, the path ANGLE takes (roll upright vs pitch
  through) has not been traced. This is the operator's all-attitude caution (FR-015b (2)).
- **MSP flood**: the PID loop is protected (REALTIME tasks first; SERIAL is LOW priority at 100 Hz, one MSP command per call, 256 B
  RX buffer). A flood delays or starves the RAW_RC frames instead. Channels not sent are invalid. ✔ **While disarmed**, non-MSP
  bytes are evaluated (`fc_tasks.c:113`). A stray `reboot_character` followed by 100 ms of silence **reboots into the bootloader**
  (`msp_serial.c:425-447`), and a `#` enters CLI. After a DROP (disarmed in the air), a misbehaving companion can therefore
  **brick the controller mid-air**. While armed this cannot happen.
- **GPS brownout**: marked lost after 1 s, restarted after 0.5 s, ~1–3 s to re-establish comms, then time to first fix (~1–2 s hot,
  ≥ 25 s cold). The estimator drops GPS after 1.5 s. LAND does not depend on GPS (below). RTH does.

### Throttle to 0, by procedure

| loss-of-signal procedure | motor | armed? | surfaces | pilot control on link return | needs the nav estimate |
|---|---|---|---|---|---|
| **DROP** (current) | off at ~1.2 s (by disarm) | **no, disarmed in the air** | **live on stale sticks / held companion command** (no armed gate in the PID) | re-arm ≈ 4 s after link return | no |
| **LAND** (emergency landing) | **mincommand from activation, still armed** (`failsafe_throttle = 1000` + `motorstop_on_low`) | yes, until `failsafe_off_delay` or landing detection | forced **ANGLE**: fixed angles (`failsafe_fw_*`: −20° roll, +10° pitch, −45 °/s yaw ⇒ a circling glide) when there is no estimate; uses alt / pos if usable | **immediate**: link up + stick motion ≥ `failsafe_stick_threshold`, no re-arm | **no** (uses it if valid) |
| RTH | **powered** (climb, cruise) | yes | nav | stick motion | yes, else emergency landing |
| NONE | **frozen at the last value** (a powered fly-away) | yes | frozen | — | no |

⚠️ LAND caveat: the stick-motion check reads the **override-merged** channels. If the override is still on and the companion
still sending when the link returns, control goes back to the companion. B-5 (the companion stops on failsafe) closes this.

### Decision — proposals for the operator (each bench-tested before it flies; none adopted silently)

| # | proposal | what it buys | cost / risk |
|---|---|---|---|
| R-1 | **`failsafe_procedure` DROP → LAND**, with `failsafe_throttle = 1000`, `motorstop_on_low = ON`, `failsafe_fw_roll_angle / yaw_rate` chosen by the operator (circle vs wings-level), `failsafe_off_delay` chosen to never disarm in the air (semantics to verify on the bench) | motor off **and still armed**; surfaces in ANGLE instead of stale commands; **control back without a re-arm**; no bootloader exposure (stays armed); no nav dependence | ANGLE from inverted / vertical is untraced; a glide circle rather than a "drop" over the park; the operator's prior is "DROP is generally the safest" |
| R-2 | **`failsafe_delay` 10 → 3–5** (0.5–0.7 s) | shortens the powered stage-1 window **and** the companion-held window (1.4 s → 0.6–0.8 s) | more failsafes on short glitches; cheap with LAND (recoverable), costly with DROP (disarm) |
| R-3 | **companion MSP caps**: RAW_RC ≤ 50 Hz with all overridden channels, total ≤ 50 commands/s, one request outstanding, **never** setting / config / reboot commands in flight, **never** non-MSP bytes, and **stop all traffic** when INAV reports disarmed in the air (xiao side) | a misbehaving companion can't starve RAW_RC or brick the controller | xiao code only |
| R-4 | **flight-controller watchdog**: enable IWDG in the fork (timeout ~100–250 ms), so a hang resets and the motor stops (outputs off during the reboot) instead of a frozen powered fly-away | removes the worst case: a hang with the motor running | a fork change: the fourth build surface. The reboot then lands in "disarmed, MANUAL passthrough", and re-arm stays blocked by the baro calibration |
| R-5 | **fork: in-flight restart path**: on a watchdog / brownout boot (reset reason is persisted), skip or bypass the baro calibration block and allow arming in flight with low throttle | the in-flight re-arm the operator wants after an FC reboot | a fork change; it needs its own test plan. After G2 |
| R-6 | **transmitter-side re-arm** (operator's earlier approach: EdgeTX global variable / logical switch that re-asserts arm when motion is detected) | fast re-arm after a TX reboot | transmitter configuration; it interacts with R-1 (with LAND the aircraft stays armed, so less is needed) |
| R-7 | `reboot_character` / CLI exposure on the companion's UART: the fork skips non-MSP evaluation on that port even when disarmed | belt-and-braces with R-3 | small fork change |

**Operator decisions 2026-11-11** *(staged after the re-fly on 2026-11-12, see R16 note)*: R-4 (FC watchdog) + R-7 **before the re-fly**; `failsafe_delay` bench-tested at **5** (R-2); DROP stays for
now (R-1 LAND bench-tested in RAM); and **re-enabling after a failsafe must take fewer pilot actions** (today: throttle 0, disarm, arm,
throttle), which favours LAND (armed throughout, control back on stick motion) and needs a re-arm path designed against an action
count. Original recommendation for the re-fly: **R-2 + R-3 now** (config + xiao). **R-1** bench-tested on the bench and,
if accepted, flown. **R-4 / R-7** as the first fork change after G2, or before the re-fly if the operator rates the hang case high
enough. **R-5** after G2. Intentional in-flight fault testing waits for a large open site (spec FR-015b).

## R18 — what the xiao does on a reboot in flight (operator 2026-11-11)

Traced by hand in `xiao/src/main.cpp`, `xiao/src/msplink.cpp`, `xiao/include/fault_guard.h`, `xiao/include/main.h`.

**Boot sequence**: `faultGuardInstall` (snapshots RESETREAS + the fault record) → LED → console → **`delay(8000)`** (so the bench can
read the first boot lines over USB) → `faultGuardReport` → flash logger → MSP → BLE → controller → **watchdog armed last** (30 s,
`kWdtTimeoutS`; long because the nRF52 WDT keeps counting through the soft reset into the UF2 bootloader, and a 4 s dog killed a
DFU on 2026-07-12).

**Timeline of a xiao reboot while the aircraft is flying under autoc** (RC9 high, INAV armed):

| t | xiao | INAV / aircraft |
|---|---|---|
| fault | HardFault ⇒ breadcrumb + `SystemReset` at once. A **wedge** with no fault ⇒ silent until the **30 s** dog fires. A brownout ⇒ power-on reset | MSP frames stop |
| +0–1.4 s | rebooting / in the 8 s delay | holds the last MSP command (incl. throttle) ~1.4 s, then **the pilot's channels** (incl. the pilot's RC6 mode) |
| +1.4 s → +~8.5 s | still in `delay(8000)` | pilot flying |
| +~8.5 s (a wedge: +~38.5 s) | first loop: INAV armed ⇒ the arm transition fires: **a new flight file is opened** (the pre-reboot file is left as last flushed; up to ~4 KB of metadata-interval data at risk), BLE off | — |
| same loop | `servo_reset_required` boots **false**; RC9 > 1600 and armed ⇒ `autoc_countdown` 1 > **`MSP_ARM_CYCLE_COUNT = 0`** ⇒ **autoc re-engages immediately**: a new origin at the current position, the path from the cached RC selector, the NN state reset | INAV accepts the override after **0.7 s** of good frames ⇒ **the NN takes the aircraft back from the pilot without any pilot action** |

**So yes, in theory it re-arms itself.** After any xiao reboot in the air, if the pilot has left RC9 high, autoc re-engages about
9 s later (about 39 s after a wedge), from wherever the aircraft is, mid whatever the pilot is doing. With INAV disarmed (e.g. after
a DROP) it does not, because engagement requires armed.

**Decision proposals** (B-2, reframed):
- **B-2a: in-air warm-start detection.** If INAV reports *armed* on the xiao's first valid state after boot, this is a reboot in
  flight: **block engagement until INAV reports a disarm → arm cycle** (operator 2026-11-11: control stays with INAV whatever the
  RC9 position; an RC9 cycle is not enough), log `kEventInFlightReboot` with
  RESETREAS + the fault breadcrumb, and **continue** the flight file (or open a new one flagged "continuation"). Re-engaging the NN
  after a reboot is a pilot decision, not an automatic one.
- **B-2b: drop the 8 s delay when no USB host is present.** Gate the delay on USB VBUS (nRF52840 `POWER->USBREGSTATUS`) or on a
  host DTR, at most 8 s. In flight there is no USB, so the xiao is back in ~1 s, which matters for logging and for the shadow frame.
  The bench keeps its boot messages.
- **B-2c: a short watchdog in flight without breaking DFU.** The WDT cannot be reconfigured once started. Start it **at the first
  arm** rather than at boot, with a ~1–2 s timeout, and require a power cycle before any DFU after a flight. Disarmed bench sessions
  never start the dog, so DFU is unaffected. The B-4 control-loop check is complementary: it catches a loop that runs but has
  stopped making progress.
- With B-1 (neutral, minimum-throttle frames on stop) and B-2a, a xiao fault becomes: ≤ 1.4 s motor-off zero-rate hold, then the
  pilot, with no surprise re-engage.

**Alternatives considered**: keeping auto re-engage as a "recovery" feature (rejected: the NN restarting on a fresh origin mid-pilot
recovery is not recovery); `MSP_ARM_CYCLE_COUNT > 0` as a delay (doesn't fix it; it only delays the surprise).

## R19 — article 1 sensor layout, compass alignment and EMI (photos 2026-11-11)

Photos: [`photos/art1-top-level-20261111.jpg`](photos/art1-top-level-20261111.jpg) (top, on a level surface) and
[`photos/art1-underside-bay-20261111.jpg`](photos/art1-underside-bay-20261111.jpg) (underside, electronics bay).

**Layout (from the photos; distances scaled from the 0.762 m span, ±20 %)**:
- **GPS / compass (Matek M8Q-5883)**: flat on the **top of the wing, on the centreline, mid-chord**, between the two
  wing-retention bands, connector and wires toward the tail. Patch antenna up: clear sky view.
- **Electronics bay directly underneath**: a cut-out through the wing centre holds the **FC (MATEKF722MINI)**, the **xiao**, the
  SBUS receiver and the wire bundle that runs up through the foam to the GPS. The compass is **~2.5 cm (the wing thickness)**
  from the FC board and its wiring.
- **High-current path**: the battery lead (red + black, routed together) and the XT30 sit in the pod **just ahead of the leading
  edge, ~10–15 cm** forward of and below the compass. The ESC is in the pod; the motor is at the nose, ~20 cm away. The aileron
  servos are in the wing, ~14 cm either side.

**Compass alignment, settled numerically** (INAV `rotationMatrixFromAngles` + `rotationMatrixRotateVector`, which applies the
transpose): board alignment **roll 180° + yaw 90° is exactly `CW270FLIP`**, the rotation Matek specifies for this module's QMC5883.
The configuration of record applies `align_mag = CW0` **then** board alignment **roll 170° / yaw 90°**. That is CW270FLIP **plus a
10° tilt**: y → (−0.985, 0, −0.174), z → (0.174, 0, −0.985). The FC sits 10° off flat; the compass sits flat on the wing.
- Effect: the 10° tilt rotates part of the large vertical field into a horizontal axis. With a magnetic dip of ~60° (vertical
  ≈ 42 µT, horizontal ≈ 23 µT, approximate for the site), the horizontal error reaches ≈ 42 × sin 10° ≈ 7 µT, i.e. **heading
  errors of up to ~15–18° that depend on heading and attitude**. It explains both 11-08 observations: no constant offset (median
  0°) and 10–25° scatter in manoeuvres.
- **Fix**: select the external-compass path with the exact rotation, independent of board alignment: `align_mag_roll = 1800`,
  `align_mag_pitch = 0`, `align_mag_yaw = 900` (≡ CW270FLIP). Then the outdoor calibration and the 8-bearing check (checklist C6,
  E4). Confirm the module's printed arrow points forward before applying it.
- ⚠️ **Correction (operator 2026-11-11): the compass is probably not flat.** The FC *and* the GPS both sit on the wing's top panel
  **aft of the thickest point of the chord**, so both are tilted **back** slightly. With yaw 90° applied after roll, the board's
  "roll 170°" is a ~10° tilt about the **aircraft pitch axis**, i.e. the FC tilted back, as the photos suggest. If the GPS panel
  has a similar tilt, the current configuration (board alignment applied to the compass) is roughly **correct**, and the residual
  error is only the **difference** between the two tilts. ⇒ **Measure both panel angles** (inclinometer or phone level on the panel
  at each module, aircraft on the level reference) before choosing. Equal tilts: keep the effect but make it explicit
  (`align_mag_roll/pitch/yaw` = CW270FLIP with the measured back-tilt). Different tilts: the measured compass tilt goes into
  `align_mag_*`, independent of the board. The 15–18° estimate above applies only to a 10° mismatch.
- **Priority (operator)**: when M2 tracks a real target optically, direction to the target is **relative** (camera). The compass
  then matters mainly for **arena containment** (is the aircraft staying inside the field?) and possibly for **IMU aiding**. Worth
  getting right, but not the critical path for tracking. Cable bundling is the practical fix if D2 shows coupling.

**EMI / current coupling, what to test**:
- **Battery lead at ~12 cm**: a single conductor at 15 A would give ~25 µT, comparable to the horizontal Earth field. The paired
  red + black routing cancels most of it (~1 µT for a 5 mm pair at 12 cm). ⇒ D2 (heading vs throttle) decides it; any separation
  of + and − at the XT30 or ESC matters.
- **The FC and wire bundle 2.5 cm under the compass**: servo current (0.5–2 A peaks in aerobatics) on the FC's servo rail and in any
  servo or power leads in that bundle can produce several µT at this distance. ⇒ Add a **servo sweep** to D2 (heading drift with
  the servos cycling, motor off).
- **GPS noise floor**: the FC (216 MHz), the xiao (BLE off in flight, but its clock and flash), the SPI flash and the receiver sit
  under the patch antenna. ⇒ New bench item: per-satellite **C/N0** (MSP_GPSSVINFO) with the xiao powered vs unpowered, motor off
  vs running. A drop of several dB-Hz means EMI is costing satellites, and that showed up as drops in pulls on 11-08.
- Mitigations if needed (hardware, operator's call): raise the GPS on a short mast, add a ground-plane shim under it, route the
  bundle away from under the module, twist the battery leads end to end.
