# 045 — Bench checklist (itemized)

## ▶ Operator TODO — next time at the bench (kept current; updated 2026-11-12)

The desk work for these is done; each needs the article, the TX, a phone, the outdoors, or the admin AWS profile. Task IDs are
in [tasks.md](tasks.md); results go in `bench-results-YYYYMMDD.md` and the [MANIFEST](MANIFEST.md).

**State of the hardware**: the article 1 xiao runs the **045 dev firmware** (BLE download v2, USB console `LIST` / `DUMP:` /
`QPROF`), not the 11-08 build; its log still holds `flight_001.bin` (11-08). The FC carries the 11-11/12 bench change set, which is
not yet in `xiao/inav-hb1.cfg`. The page that matches the firmware is the current `xiao/web/flight_logger.html` (v1 is gone).

**Desk review (any time)**: read [`pilot-brief.md`](pilot-brief.md) before the next flight (⚠️ new finding: after a
receiver-loss DROP the surfaces hold the **xiao's last command** until the link returns, not your sticks); approve or reorder the
proposed steps in [`safety-ladder.md`](safety-ladder.md); skim [`fault-timelines.md`](fault-timelines.md), whose bench cells T056
fills.

**[USB] / phone — BLE download (US2)**
- [ ] **T017 / T023 phone**: with the **field phone** and with the **Windows / Edge laptop**, phone ~3 m from the aircraft,
  download `flight_001.bin` with the new page. Target < 60 s (Linux measured 19 s). Walk out of range mid-download, come back,
  reconnect, press Download again ⇒ it resumes. If anything is slow or fails: **Copy log** and paste it back.
- [ ] **T023 arm mid-download** (TX on, motor disconnected): start a download, arm ⇒ BLE drops at once and the xiao's console shows
  normal MSP polling again.

**[USB] — calibration and configuration (US4)**: T034 (A7 receiver failsafe mode, A8 boot banner), T035 (C3 accel 6-point + 12
poses, `cal_verify.py accel`), T036 (C4 gyro zero / gravity; `init_gyro_cal` OFF), T037 (C5 gyro scale, CW + CCW turn pairs),
T038 (C6 panel tilts → `align_mag_*`, before T044), T039 (C7 xiao orientation photo).

**[SERVO], motor disconnected — safety "before" record (US7-A)**
- [ ] **T056**: bench-checklist B1–B10 and H1–H4, H6, H7, ×3 each, on today's configuration; `failsafe_delay` 10 / 5 / 3 and LAND
  vs DROP in RAM only (never saved), with pilot-action counts and timings. H5 (MSP flood) stays held.

**[PROP], restrained (US4 / US5)**: T040 (D1 mount survey, `mount_survey.py`), T041 (D2 compass vs current + servo sweep, I2C
errors via `msp_bench_read.py --watch`), T042 (D3 baro; D4 C/N0 via CLI `gpssats`, xiao on vs off, motor off vs on), T051 (blackbox
MAG + NAV_ACC on, `debug_mode` VIBE kept, `blackbox_rate_denom` 8 / 16 / 32 with the motor running, no dropped frames; 1:32 is OK).

**[OUT]**: T043 (E1–E3 GNSS active, 10 Hz, 10 min static → `inav_default_alt_sensor`), T044 (E4 dip / |m| at N/E/S/W, compass
calibration, 8 bearings; check the declination sign first), T054 (satellite-block test with autoc engaged, motor disconnected).

**Then**: T049 (apply and read back the final change set; approve writing it into `xiao/inav-hb1.cfg`), T050 (article 2, only if it
flies).

**Not bench — needs the admin AWS profile**: T009, T013 (`s3_egress_check.py --profile <admin> --run <id> --day 2026-11-12` for
mini runs A and B; run IDs in the MANIFEST), T113 (month-end egress < 100 GB).

---

What to check at the bench, before `/speckit.tasks` fixes the work. Each item says what it settles, how to do it, and when it can
be done. Procedures with more steps are in [quickstart.md](quickstart.md); the reasoning is in [research.md](research.md).

**Setup classes**:
- **[USB]**: FC + xiao on USB only (no battery, motor or servos). Possible right now.
- **[SERVO]**: servos powered, motor disconnected or props off.
- **[PROP]**: props on, article restrained.
- **[OUT]**: outdoors with sky view.

**Access**: the FC's USB VCP speaks MSP and CLI. ⚠️ Only one program can hold the port: close INAV Configurator before any
scripted probe. Leaving the CLI with `exit` **reboots** the FC (harmless on the bench, but the xiao logs an MSP failure). Read-only
first; nothing is written until its change set is agreed.

Results go in the article's configuration change set ([data-model.md](data-model.md)) or in a `bench-results-YYYYMMDD.md` next
to this file.

---

## A. Identity and configuration — [USB], read-only

- [x] **A1 Which article and build.** `version` and `status` (CLI) → board target, INAV build hash. Article 1 should be
  `MATEKF722MINI` at `38ff0d29`; article 2 `HGLRCF722LITE` at `c412bfd7` + the target (now `1fc3b257`).
- [x] **A2 Config vs the repo.** *(2026-11-11: matches except 4 boot-cal values — [bench-results-20261111.md](bench-results-20261111.md))* `diff all` → saved, then compared with `xiao/inav-hb1.cfg` (or `-art2`). Every difference is a
  finding.
- [x] **A3 Mode-bit layout (B-3, critical).** `aux` (CLI) and the box list (MSP_BOXNAMES 116 + MSP_BOXIDS 119): record **bit
  position → permanent id → name**. Confirm or refute: the xiao's hard-coded `MSP_MODE_FAILSAFE = 19` is actually NAV ALTHOLD;
  where ARM and FAILSAFE really sit.
- [x] **A4 Arming flags now.** `status` → "Arming disabled flags". Records the baseline blockers.
- [x] **A5 Receiver identity (FR-008).** `status` GPS block: `HW Version`, `Proto`, GNSS Capabilities (Galileo / BeiDou / Glonass
  active/default, Max concurrent). Needs the GPS powered: check whether USB powers the GPS rail on this FC.
- [x] **A6 Current values of every setting the plan touches** (`get <name>`): `gps_ublox_use_galileo/glonass/beidou`,
  `gps_dyn_model`, `gps_ublox_nav_hz`, `gps_min_sats`, `gps_auto_config`, `init_gyro_cal`, `gyro_zero_*`, `ins_gravity_cmss`,
  `inav_w_acc_bias`, `inav_max_eph_epv`, `inav_default_alt_sensor`, `inav_w_*`, `failsafe_*` (all), `motorstop_on_low`,
  `msp_override_channels`, `align_mag*`, `align_board_*`, `reboot_character`, `blackbox_rate_denom`, the blackbox field groups,
  `debug_mode`, `fw_p_level`, `max_angle_inclination`, `nav_rth_*`, `nav_disarm_on_landing`, `switch_disarm_delay`,
  `disarm_always`, `serial` (which UART carries the xiao's MSP and at what baud).
- [ ] **A7 Receiver protocol and its failsafe mode.** Article 1 SBUS / article 2 CRSF; the receiver's own failsafe setting
  ("no pulses" vs "hold" vs custom). ⚠️ A receiver set to *hold* without a failsafe flag is **never detected** by INAV (R17).
- [~] **A8 xiao build and boot log** *(heartbeat captured; boot banner / genome hash still owed)* (`/dev/ttyACM0`, listen only; don't script-race the 8 s boot window). Firmware build, the
  NN genome hash (expect `a5ed3521ea1e70b7`), flight-log format version, the MSP poll rate actually used.
- [x] **A9 State message on the wire** *(2026-11-11: 64 B, \|a\| 0.999 g, gyro ~0 — consistent)* (`specs/041-m2-depth/msp_state_probe.py`, Configurator closed): 64-byte length, box
  bitmask value with the current switch positions, timestamp advancing.

- [~] **A10 I2C error count** (`status`): 50 after 2.8 days of uptime **and 50 right after a reboot** ⇒ a boot-probe count; recheck with the motor running (D2).

## B. Behaviour with servos powered — [SERVO], props off (motor disconnected or props off)

- [ ] **B1 Escape: RC9 off while engaged** ⇒ pilot sticks pass through at once (watch the servos). Time the hand-back.
- [ ] **B2 MANUAL (RC6) while engaged** ⇒ no effect (overridden). Confirms R8 / R16.
- [ ] **B3 RTH / POSHOLD switch (RC8) while engaged, indoors (no GPS fix)** ⇒ refused silently; record what the xiao logs (does
  bit 19 fire?).
- [ ] **B4 xiao stops sending** (pull its MSP TX or reset it) ⇒ time the last-command hold (expect ~1.4 s), then the pilot's
  channels including RC6. Watch the throttle channel output in the Configurator Outputs tab (motor disconnected).
- [ ] **B5 xiao reboot while armed with RC9 high** ⇒ does autoc re-engage by itself (expected today, R16 S4)? Record the delay.
  ⚠️ Motor disconnected.
- [ ] **B6 Transmitter off (LOS) while engaged** ⇒ today's DROP: time to disarm, servo behaviour after the disarm, re-arm sequence
  and time after the TX is back on. Then the same with **LAND** as a candidate (R-1), *in RAM only, not saved*: motor output at
  minimum while armed, ANGLE on the surfaces, and control back on stick motion. Note the companion-regains-control caveat.
- [ ] **B7 `failsafe_delay` candidates** (R-2) in RAM: measure the stage-1 and companion-hold windows at 10 / 5 / 3.
- [ ] **B8 FC reboot while "flying"** (`reboot` from the CLI, or power-cycle the FC only): time to the first servo response,
  servo behaviour during the boot, arming blockers after the boot (`status`), MANUAL passthrough while calibrating, with
  `init_gyro_cal` ON vs OFF.
- [ ] **B9 Emergency-arming toggle sequence**: verify what it can and can't override (R17), with the motor disconnected.
- [ ] **B10 MSP traffic from the xiao**: measure its request rate (INAV MSP stats or a UART tap); compare with the R-3 caps.

## C. Calibration and alignment — [SERVO] or [USB] with the article level

- [x] **C1 Level reference** *(2026-11-12: flat wing bottom levelled, nose magnetic north)*: support the article with the **flat wing bottom level** (spanwise and chordwise).
- [x] **C2 Board alignment** *(2026-11-12: gravity-solved 1736 / 7 / 900 ⇒ attitude +0.06° / −0.21°; FC back-tilt ~6.4°)*: attitude roll / pitch on the level reference → 0 / 0 ± 0.3°, else correct `align_board_*`. Cross-check the FC panel's measured back-tilt (C6) against `align_board_roll` 170° (with yaw 90° that is a ~10° back-tilt).
- [ ] **C3 Accel 6-point** + **≥ 12-pose verification** (R6b; `cal_verify.py`).
- [ ] **C4 Gyro zero + gravity reference** recorded level and still; then `init_gyro_cal = OFF`, `inav_w_acc_bias = 0`. Explain
  the stored 943.52.
- [ ] **C5 Gyro scale factor**: 10 turns per axis on a jig, blackbox denom 1–2.
- [~] **C6 Compass and board tilt** *(2026-11-11 photos + INAV math, R19: config = CW270FLIP + the FC's ~10° back-tilt; the GPS is on the same aft top panel, so probably tilted back too)*: on the level reference, **measure the back-tilt of the FC panel and of the GPS panel** (inclinometer / phone level at each module); confirm the module's arrow points forward. Then set `align_mag_*` explicitly from the measured GPS tilt (CW270FLIP + tilt), and verify with the 8-bearing check.
- [ ] **C7 xiao board orientation** in the airframe (photo + axes); 6 static faces once the IMU driver exists.

## D. Props on, restrained — [PROP]

- [ ] **D1 Mount survey**: blackbox denom 1, GYRO_RAW + ACC; throttle sweep + steady points (R7).
- [ ] **D2 Compass vs current**: heading drift across the throttle range **and with the servos sweeping, motor off** (MAG logged). The compass is 2.5 cm above the FC and bundle, and ~12 cm from the battery lead (R19).
- [ ] **D4 GPS noise floor**: per-satellite C/N0 via the CLI `gpssats` command (disarmed; leaving the CLI with `exit` reboots the FC) or u-center through `gpspassthrough`. INAV 8.0's `MSP_GPSSVINFO` is a stub that reports no satellites (found 2026-11-12); `msp_bench_read.py --svinfo` falls back to sats / HDOP / EPH, with the xiao powered vs unpowered, and motor off vs running (R19). Read-only except the power changes.
- [ ] **D3 Baro prop-wash step**: the baro altitude change at run-up.

## E. Outdoors — [OUT]

- [~] **E1 GNSS config** *(applied 2026-11-11 on the bench: GPS + Galileo + GLONASS active, AIR_4G; outdoor confirmation owed)* (article 1: GPS + Galileo + GLONASS, BeiDou off; AIR_4G) → `status` after save and reboot: the
  constellations **active**.
- [~] **E2 Nav rate check** *(2026-11-11 MSP: message interval 100–101 ms with 3 GNSS ⇒ 10 Hz held; confirm in a blackbox outdoors)*: the GPS frame interval in a short blackbox ≈ 100 ms (10 Hz); if 200 ms, drop GLONASS.
- [ ] **E3 10 min static**: sats, HDOP, EPH, EPV, GPS-only vs multi-GNSS → the `GPS_ONLY` decision (EPV ≤ ~5 m).
- [ ] **E4 Outdoor compass check + calibration** (after C6): first the **dip and \|m\| at N / E / S / W, level** (good calibration ⇒ dip ≈ 61° and the same \|m\| at every heading; indoors on 11-12 the dip read ~18° low after alignment, R19). Then the outdoor calibration if needed, then the heading at 8 known bearings < 5°.
- [ ] **E5 Satellite-block test** (N3, once flags exist): cover the antenna ~15 s with autoc engaged, motor disconnected → the
  expected flags, no behaviour change.

## F. Firmware-dependent (after the corresponding code lands)

- [~] **F1 BLE download, instrumented** (R2) *(2026-11-11: baseline measured remotely with `ble_cdp_probe.py` — 1.58 kB/s, 98 ms/chunk stop-and-wait, duplicate-delivery failure mode found; the field-phone run is still owed)*: a full ~300 kB log at 3 m and at field distance, the field phone, **Copy log** pasted
  back. Then v2: an interrupted download resumes byte-identical.
- [ ] **F2 B-1 / B-4 / B-5** (neutral minimum-throttle frames on stop, watchdog, failsafe stop): repeat B4–B6 and confirm the
  hold window is wings-steady and motor-off.
- [ ] **F3 Shadow IMU (N4)**: static 60 s, 10 turns per axis, 0 tick overruns.

## H. Safety proof tests — simulated faults on the bench (R16 / R17 / R18)

Each is run **before** and **after** the safety changes (B-1…B-5, B-2a…c, R-1…R-3), so the record shows the old behaviour and the
fix. Setup [SERVO], **motor disconnected**, INAV armed through the transmitter, autoc engaged in bench mode. Observe the servos,
the Configurator Outputs (motor channel), INAV `status`, the xiao console and both logs. ⚠️ Motor disconnected for every item.

- [ ] **H1 Link failure**: transmitter off for 0.5 s / 2 s / 10 s while engaged. Measure the stage-1 window, the DROP disarm, servo
  behaviour after the disarm, re-arm after the TX returns. Then LAND (R-1, in RAM): motor output at minimum while armed, ANGLE on
  the surfaces, control back on stick motion; the companion-regains-control caveat.
- [ ] **H2 Transmitter reboot**: power-cycle the TX while engaged: the time INAV takes to see loss and recovery (SBUS vs CRSF).
- [ ] **H3 xiao reboot**: (a) reset button / pull its power; (b) a forced watchdog (a test hook, if added); (c) a forced HardFault
  (a test hook). Measure the last-command hold, the hand-back to the pilot's channels, the 8 s delay, and **whether autoc
  re-engages by itself with RC9 left high** (expected today, R18), plus log continuity.
- [ ] **H4 INAV reboot while armed**: cut the FC's power (the USB hub port power cycle on this host is a backup hard-restart, per
  the operator) or brown it out. Measure the servo behaviour while it boots (limp / centred), boot-to-first-servo time, arming
  blockers after the boot with `init_gyro_cal = OFF`, MANUAL passthrough while disarmed, and what the xiao does on MSP loss and
  when the FC returns disarmed.
- [ ] **H5 MSP flood** ⛔ *(operator 2026-11-11: hold until a verified hard power-cycle exists — it may wedge the FC; the hub (VIA 2109:2813 on port 3-1) needs `sudo lsusb -v -s 3:12` / `uhubctl` to confirm per-port power switching, else pull the cable)*: drive the FC's MSP port at a high request rate (from the host on USB, or a xiao test hook on the UART):
  INAV cycle time and CPU load (`status`), and the latency of the companion's RAW_RC.
- [ ] **H6 GPS brownout**: unplug and re-plug the GPS lead while armed: INAV's GPS restart and reconfigure time, the estimate's
  validity, and what the xiao's N3 flags show.
- [ ] **H7 RC9 escape** (= B1) and **stop frames** (after B-1): the hold window is motor-off and zero-rate.
- [ ] **H8 Repeatability**: each H item ×3, timings recorded in `bench-results-YYYYMMDD.md`.

## G. Session checklist (every session, before the re-fly)

- [ ] Erase the xiao log. Re-record the gyro zero if `init_gyro_cal = OFF`. `diff all` vs the repo. Pilot brief: **escape = RC9
  off**; RC6 does nothing while engaged; RC8 only with a valid estimate.
