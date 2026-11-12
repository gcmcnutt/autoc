# 045 — Quickstart: bench, outdoor and flight procedures

The operator runs these with the article. Each procedure records its result in the article's configuration change set
([data-model.md](data-model.md)) or in the N1 / re-fly analysis doc. ⚠️ **Props off** for everything except § Mount survey.
Disconnect the GPS before flashing an INAV controller (standing project rule).

## 0. Session checklist (every bench or field session)

1. Erase the xiao log before the session (v7 budget: 7200 engaged ticks ≈ 6 min).
2. If `init_gyro_cal = OFF` is in force: re-record the gyro zero (§ 3, step 4) at the session's temperature.
3. `diff all` from each controller, compared against its config of record in the repo. Any difference is a finding.

## 1. Receiver identity and multi-GNSS (FR-008) — bench, sky view

1. CLI `status` → record `HW Version`, `Proto`, the GNSS Capabilities block (Galileo / BeiDou / Glonass active/default, Max
   concurrent).
2. Article 1 (SAM-M8Q): `set gps_ublox_use_galileo = ON`, `set gps_ublox_use_glonass = ON`, **BeiDou stays OFF** (M8 max 3 GNSS), `set gps_dyn_model = AIR_4G`, `save`. After the reboot,
   `status` again. **The constellations must read active**: a NAK silently reverts them, and a later `save` would persist the
   revert.
3. **Nav rate check**: a short bench blackbox with GPS; the GPS frame interval in `*.gps.csv` must be ~100 ms (10 Hz). If ~200 ms, the module NAKed 10 Hz with 3 GNSS and INAV fell back to 5 Hz: turn GLONASS off and recheck.
4. Outdoors, 10 min stationary: record sats, HDOP, EPH, EPV, GPS-only vs multi-GNSS. The `GPS_ONLY` altitude decision needs EPV
   ≤ ~5 m (research R5 table).
5. If the constellations will not stay active ⇒ swap the receiver from stock (e.g. an M10Q, as on article 2), then redo § 4 for its compass.

## 2. Level reference (FR-007b)

Support the article so the **flat bottom of the wing is level** (≈ 0° angle of attack), checked with a level on the wing bottom
spanwise and chordwise. This is the reference for § 3 and the board-alignment check.

## 3. Accelerometer, gyro and gravity calibration (FR-007a) — bench

1. 6-point accel calibration (Configurator). ⚠️ INAV's result validation cannot reject a bad fit: check by hand. After saving, in
   each of the 6 orientations, |accel| must read 1.00 ± 0.01 g.
2. On the level reference, read `attitude` roll / pitch. A non-zero reading is **board misalignment**: correct `align_board_*`
   until it reads 0 / 0 ± 0.3°, then record it.
3. Gyro scale factor (R4): on the levelled support (or by hand on a flat table), rotate the article **N = 10 full turns** about
   yaw, logging the blackbox at denom 1–2. Repeat for roll and pitch on a jig. Scale error = (∫gyro − N·360°) / (N·360°) per axis.
   Record it. This feeds the shadow frame's calibration and the R4 budget.
4. Gyro zero and gravity: with `init_gyro_cal = ON`, power up **level and still**; read `gyro_zero_x/y/z` and `ins_gravity_cmss`
   (it should be ~980; record it). Then `set init_gyro_cal = OFF`, `set inav_w_acc_bias = 0`, `save`, reboot, and read back.
5. Repeat step 4's read on the next session. If the gyro zero moves more than ~10 LSB between sessions, keep re-recording per
   session (session checklist).

## 4. Compass alignment and calibration (FR-008a) — measure indoors, calibrate outdoors

1. Measure the compass module's physical orientation relative to the airframe (arrow direction, which face is up). Set
   `align_mag_roll / pitch / yaw` explicitly. A non-zero value selects the external path; for an identity mount use yaw = 3600.
   Do **not** rely on board alignment.
2. **Outdoors, away from metal and vehicles**: run the compass calibration, tumbling the article through all orientations (INAV
   samples only after ~8° steps).
3. Check: point the nose at four known bearings (N / E / S / W by phone compass or landmarks); heading error < 5° at each. Record
   it.

## 4b. Independent verification (R6b) — bench + outdoor

Log the blackbox (denom 1–2, groups ACC + GYRO_RAW + MAG) for each, then run `cal_verify.py`:
1. Accel: ≥ 12 static poses (6 faces + 6 diagonals), 10 s each.
2. Gyro: 60 s static (zero residual); 10 turns per axis on the jig (scale).
3. Compass: a second outdoor tumble (residual hard / soft iron); heading at 8 known bearings, nose level.
4. Compass vs motor current (props on, restrained): throttle sweep, heading drift.
5. Baro: 10 min static; prop-on run-up.
Pass criteria: research.md R6b table. Record the residuals in the change set.

## 5. Mount survey (FR-007b, R7) — bench, props ON, restrained

⚠️ Article restrained and clear of people; props on. Blackbox denom 1 (2 kHz), groups GYRO_RAW + ACC only (to fit write bandwidth).
Throttle sweep idle → full over ~20 s, then three steady points. Analysis: spectra per axis; mount modes are the peaks that do not
scale with motor RPM. If the shadow driver is built, also log raw xiao-IMU bursts (bench-only record) in the same run.

## 6. Pilot escape and override (R8) — bench, props off

1. With autoc engaged (bench test mode): flip **RC9 (MSP RC OVERRIDE) off** ⇒ the pilot's sticks pass through at once (watch the
   servos). This is the escape.
2. While engaged, flip the MANUAL switch (RC6 low): **it has no effect** (RC6 is overridden). Record this in the pilot brief.
   Operator decision: move MANUAL to a non-overridden channel, or keep RC9 as the escape (research R8).
3. Stop the xiao (pull its MSP TX): INAV holds the last command ~1.4 s (R16), then passes the pilot's sticks through. Time it.
4. Box bits: confirm the xiao maps ARM / FAILSAFE / OVERRIDE by permanent id (boot log line), not by fixed index.

## 7. Satellite-block test (N3, SC-005) — outdoors, props off

1. Outdoors with a 3D fix and ≥ 8 sats; armed in bench test mode, autoc engaged.
2. Cover the GPS antenna (foil hood or a metal can) for ~15 s, then uncover.
3. Download the xiao log. Expected: LOW_SATS and GPS_STALE while covered; INVALID_XY after EPH crosses the cap; STEP_XY on
   reacquire; **no change** in engagement or outputs (the policy keeps running). The blackbox shows the same interval.

## 8. BLE download, instrumented (F2, R2) — bench, the field phone

1. Flash the instrumented firmware + page (step 1 of the contract). Download a full ~300 kB log at ~3 m and at field distance.
2. **Copy log** → paste it into the session. That is the measurement of record (MTU, per-chunk times, truncation, drop point).
3. After the v2 protocol lands: repeat; interrupt mid-transfer (walk out of range); reconnect; resume; compare the file with a
   wired read (byte-identical).

## 9. Shadow frame (N4, if adopted) — bench

1. Static 60 s: attitude drift (bias) and overrun count (0 tick overruns allowed).
2. 10 slow turns about each axis: shadow attitude vs the turn count (scale-factor check on the xiao IMU).
3. The xiao board's orientation: static in 6 faces, solve the mount rotation, record it.

## 10. Re-fly card (US7, R11)

- Article 1, g800 (`a5ed3521ea1e70b7`) unchanged; streamer as 11-08; the N2 change set applied and read back; § 0 checklist done.
- ≥ 4 engaged spans. Plus **≥ 60 s at altitude in ANGLE or MANUAL**: a box or a circle (≥ 4 course sectors) for the heading check.
- Optional (operator): MANUAL pitch and roll doublets for the airframe fit.
- Escape = RC9 override off (§ 6).
- After landing: the BLE download (the v2 protocol if landed), the blackbox, notes on wind and conditions.
- Article 2 in the same window if ready: its NOTES fixes (`align_mag_yaw = 3600`, `vbat_scale = 1110`) read back, and § 1–4 done
  for it.

## 11. After G2 — M2 smoke and run (phases 3–4)

```bash
# smoke (G4): perfect signal, perfect nav state, repointed source (044-t1 g800)
scripts/train.sh autoc-basic-m2.ini logs/autoc-045-t1-m2-smoke.log
# full run, after the smoke passes and the pre-run build gate (Constitution IX)
scripts/train.sh autoc-tracker.ini logs/autoc-045-t2-m2-perfect.log
```

The operator drives the clean `rebuild-perf.sh` and the regression gate before either.
