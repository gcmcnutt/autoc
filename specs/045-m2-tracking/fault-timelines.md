# 045 — Fault timelines (FR-015b; T058)

For each fault: how long until the aircraft is in stabilised, pilot-controllable flight under **today's** configuration, what each
candidate change would do to that, whether the throttle goes to 0 first, and the re-arm path with its pilot action count. This is
the input to the failsafe-configuration debate ([safety-ladder.md](safety-ladder.md)).

**Configuration**: the configuration of record [`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg) (INAV 8.0 fork `38ff0d29` on article
1; `failsafe_procedure` DROP, `failsafe_delay` 10, `failsafe_recovery_delay` 5, `failsafe_off_delay` 200, `failsafe_throttle`
1000, `motorstop_on_low` ON, `msp_override_channels` 47, feature `FW_LAUNCH` on, SBUS receiver). The 045 change set now on the FC
([config-changeset-art1.md](config-changeset-art1.md)) changes no failsafe setting. It matters here in two places only:
`init_gyro_cal` OFF (the FC reboot row) and multi-GNSS (the GPS row). **xiao**: flight firmware, safety behaviour unchanged since
11-08 (operator 2026-11-12: left alone for the re-fly).

**Sources**: [research.md](research.md) R8, R16, R17 (incl. § ANGLE recovery, T057), R18; fork source (`~/inav`, `autoc` @
1fc3b257c). *(inf.)* = inference. **⏳ T056** = filled by the bench "before" record (bench-checklist B1–B10, H1–H4, H6, H7, ×3);
no number in those cells is invented here.

**Budget** (R17): **~2.9 s from the fault to the start of a recovery at 100 m** (~3.75 s at 120 m), from a 24 m/s dive.

## Mechanisms shared by several faults (today)

- **M1, stage 1**: after the receiver flags a loss, nothing happens for 200 + `failsafe_delay`×100 = **1.2 s**
  (`flight/failsafe.c:156`). The pilot's channels are frozen at their last good values (`rx/rx.c:503-507`), **RC9 included, so the
  RC9 escape cannot act while the link is down**. A live companion keeps flying, with its throttle. Detection: SBUS flags the loss
  in-frame (immediate). ⚠️ A receiver set to *hold* without the flag is never detected (A7 / T034 open).
- **M2, DROP**: at the end of stage 1, an in-air disarm, motor off (`failsafe.c:473-476, 585-591`). **No ANGLE** (R17 § ANGLE
  recovery). The servos stay live on the ACRO rate loop (`flight/servos.c:460-474`) and the overridden channels keep the companion's
  last merged values until the link returns (`rx.c:503-512`, `rx/msp_override.c:216-222`): **the surfaces hold the companion's
  last rate command, unpowered**. From the first good frame after the link returns, the pilot's sticks steer (motor off), in the
  pilot's RC6 mode, if the xiao is quiet.
- **M3, companion-held window**: when the xiao goes quiet, INAV keeps its last frame for 200 ms (MSP signal timeout,
  `rx/msp.c:72`) + 200 + `failsafe_delay`×100 ms (override failsafe, `msp_override.c:200-211`) = **~1.4 s, throttle included**,
  then passes the pilot's channels (TX link up). Taking the override back needs 700 ms of good frames.
- **M4, RC9 escape**: with the TX link up, RC9 off hands the sticks back within about one RX frame, whatever the xiao does (R8 ✔).
- **M5, xiao auto re-engage**: after a xiao reboot, if INAV is armed and RC9 is high, autoc re-engages by itself ~8.5–9 s later
  (~38.5 s after a wedge: 30 s watchdog + 8 s boot delay); INAV accepts it after 0.7 s (R18).
- **M6, launch-mode trap on re-arm** (from source; ⏳ T056 B6): with `FW_LAUNCH` on, INAV sets up launch mode for the next arm
  whenever it is disarmed and the GPS heading is invalid (no 3D fix with ≥ 6 sats, or ground speed < 3 m/s;
  `navigation/navigation.c:4613`, `io/gps.c:637-643`). An in-flight re-arm then enters launch mode: forced ANGLE with roll locked
  at 0 and pitch locked (0°, then the 25° climb once the throttle is raised), and the motor held at launch idle (stopped) until a
  launch is detected (`navigation/navigation_fw_launch.c:325-331, 335-353, 414-436`; `navigation.c:985, 1001`). Before detection,
  stick movement does not abort it (abort exists only from MOTOR_DELAY on, `navigation_fw_launch.c:451-455`). Way out: disarm and
  glide on the sticks. With a valid GPS heading (normal in flight) it does not happen. On the bench indoors (ground speed 0) a
  re-arm takes this path too: read B6 / B9 results with that in mind.

## Summary (today)

| fault | throttle to 0 first? | pilot steering back | powered flight back | vs ~2.9 s | pilot actions to full control | bench (T056) |
|---|---|---|---|---|---|---|
| RX glitch < 1.2 s | no (by design: the companion keeps flying) | when the link returns (RC9 frozen meanwhile) | never lost | ok if the companion is flying well | 0 | ⏳ T056 (H1 0.5 s) |
| RX loss ≥ 1.2 s | at 1.2 s, by the DROP disarm | at link return (unpowered) | ≥ link return + ~3.7 s + re-arm | steering only if the link is back by ~2.9 s; power: no | 5 (re-arm, below) | ⏳ T056 (H1 2 s / 10 s, B6) |
| TX reboot | at 1.2 s | TX boot + resync ⏳ | + ~3.7 s + re-arm | no *(inf.: a TX boot alone likely exceeds 2.9 s)* | 5 | ⏳ T056 (H2) |
| FC reboot | yes: outputs stop, boots disarmed | ~2–2.5 s *(inf.)*, MANUAL passthrough | no in-flight re-arm (config of record) | steering marginal; power: no | — | ⏳ T056 (B8, H4) |
| FC HardFault | yes: `stopMotors()`, then a halt (`fc/fc_hardfaults.c:93-110`) | never: surfaces frozen at their last pulses *(inf.)* | never | **no recovery path** (no watchdog to reboot it) | none possible | needs a fault hook (L9) |
| FC hang | **never**: motor frozen at its last value *(inf.)* | never | — | **no recovery path** | none possible | needs a hang hook (L9) |
| xiao reboot / wedge | no: ~1.4 s at the last command, then the pilot's throttle stick | ~1.4 s | ~1.4 s | ok, but 1.4 s at the last policy command | 1: RC9 off (else M5) | ⏳ T056 (B4, B5, H3) |
| GPS brownout | no: nothing declares a fault | RC9 off, ~1 RX frame | not lost | depends on the pilot noticing | 1: RC9 off | ⏳ T056 (H6) |
| MSP flood | no; if RAW_RC is starved, as M3 | ~1.4 s if starved *(inf.)*; RC9 off | not lost (armed) | *(inf.)* | 1: RC9 off | ⛔ H5 held; ⏳ T056 (B10) |

## RX glitch and RX loss

| t | INAV | xiao | aircraft | pilot | measured |
|---|---|---|---|---|---|
| 0 | the receiver flags failsafe; channels frozen (M1) | flying | the policy flies on | RC9 frozen: no escape | ⏳ T056 |
| < 1.2 s, link back | nothing happened; channels resume | flying | unchanged | RC9 works again | ⏳ T056 |
| 1.2 s | DROP: in-air disarm (M2) | sees the disarm, stops, goes quiet (R16 S3) | motor off; surfaces hold the last rate command, ACRO; may keep rotating | nothing | ⏳ T056 |
| link back (t_L) | pilot's channels from the first good frame | quiet | pilot steers, motor off | glide; start the re-arm | ⏳ T056 |
| t_L + 0.7 s | link "up" for the failsafe | — | — | — | ⏳ T056 |
| ≥ t_L + 3.7 s | re-arm allowed (ARM off at the end of the 3 s) | — | — | ARM on, throttle up | ⏳ T056 |

**Candidate changes**:

| change | what it does to this timeline |
|---|---|
| L7 `failsafe_delay` 10 → 5 (→ 3) | stage 1 and the DROP disarm at 0.7 s (0.5 s). Glitches of 0.7–1.2 s now cause an in-air disarm and a re-arm: costly with DROP, cheap with LAND |
| L7 DROP → LAND (R-1) | at 1.2 s: motor off **still armed**, forced ANGLE (T057: half roll from inverted, full pull from vertical; fixed −20° / +10° glide with no estimate). On link return, control comes back on stick motion ≥ `failsafe_stick_threshold` 50 once the link is up (`failsafe.c:550-555`): **no re-arm**. ⚠️ The stick check reads the merged channels (`failsafe.c:325-339`), and the xiao keeps sending through a LAND (it watches bit 19, not FAILSAFE): control returns to the **companion** unless L1 is in or the pilot flips RC9 off first. `failsafe_off_delay` 200 disarms in the air after 20 s, and a re-arm then needs **30 s** of good link (`failsafe.c:576-580`); 0 disables the timed disarm (`failsafe.c:227-230`) |
| L1 (mode bits by permanent id) | the xiao stops on the real FAILSAFE bit, set when stage 2 starts (`failsafe.c:242`), not during stage 1. No change under DROP (it already stops on the disarm); required before LAND |
| L4 (neutral, motor-off frames on every stop) | the xiao's stop on the disarm sends zero-rate frames, and those are what M2 then holds: the surfaces **hold attitude** instead of the last rate command. No change in stage 1 |
| R-6 (TX-side re-arm) | the re-arm actions drop to 0–1; the ~3.7 s wait stays (below) |

**Throttle to 0 first**: not in stage 1 (powered by the companion for 1.2 s, by design for a glitch); at 1.2 s by the disarm. With
`failsafe_delay` 5: at 0.7 s. With LAND: at 1.2 s (0.7 s), still armed.

## TX reboot

As RX loss, with t_L = the transmitter's boot plus the link resync: **⏳ T056 H2** (SBUS on article 1; CRSF on article 2 has no
in-frame flag, so the loss is "frames stop" + 100 ms, and ELRS resync takes ~1–3 s on the radio side, R17). The candidates act as in
the RX-loss table. R-6 runs on the transmitter, so it must survive the transmitter's own reboot: a runtime flag in a logical switch
does not *(inf.; ⏳ check in EdgeTX)*.

## FC reboot and FC hang

| t | INAV | xiao | aircraft | measured |
|---|---|---|---|---|
| 0 (reboot) | outputs stop | MSP failure ⇒ stops, quiet (it cannot engage while INAV is disarmed) | motor off; servos limp or held, servo-dependent | ⏳ T056 (H4) |
| ~2–2.5 s *(inf.)* | main loop; boots disarmed; MANUAL passthrough while calibrating (`fc/fc_core.c:745-749`) | quiet | servos centred, then the pilot's sticks, motor off | ⏳ T056 (B8, H4) |
| later | re-arm blocked: SENSORS_CALIBRATING (R17) | — | unpowered glide on the sticks to a landing | ⏳ T056 (B8) |
| 0 (HardFault) | `stopMotors()` writes min command, then loops forever (`fc/fc_hardfaults.c:93-110`); servo outputs untouched | MSP failure ⇒ quiet | motor off, **surfaces frozen** at their last pulses *(inf.)*, never recovers | not testable without a hook |
| 0 (hang) | outputs keep their last pulses *(inf.)* | MSP failure ⇒ quiet | **surfaces frozen, motor running: a powered fly-away** | not testable without a hook |

Re-arm after a reboot: with `init_gyro_cal` ON (config of record) the gyro and gravity calibrations restart on movement and never
finish in flight. With OFF (on the FC now, change set row 5) the baro gate remains: its 2 s window passes only when the pressure
altitude's standard deviation is under `baro_cal_tolerance` 150 cm (`sensors/barometer.c:305-309`, `common/calibration.c:76-95`),
i.e. a steady sink below ~2.6 m/s (derived). That never happens in a dive, but **may in a steady glide**, so a re-arm after an FC
reboot is possible in principle on the re-fly FC: ARM off → on, throttle low, sticks centred, link up for 0.7 s, plus M6 (the GPS
heading is likely invalid just after the boot). ⏳ T056 B8 / H4.

| change | effect |
|---|---|
| L9 FC watchdog (R-4) | a hang, or a HardFault's endless loop, becomes a reboot (~100–250 ms timeout + boot): motor off, then the passthrough glide above. Re-arm as above |
| R-5 in-flight restart path (fork, after G2) | the reboot skips the calibration block ⇒ a designed in-flight re-arm |
| L8 (R-7), L5 (R-3) | the companion can no longer reboot the FC into the bootloader, or enter CLI, while it is disarmed in the air (see MSP flood) |
| L7 (DROP / LAND / delay) | no effect |

**Throttle to 0 first**: yes on a reboot or a HardFault (outputs stop; after a HardFault the surfaces stay frozen and the FC never
comes back); **no on a hang** until L9.

## xiao reboot or wedge

| t | INAV | xiao | aircraft | pilot | measured |
|---|---|---|---|---|---|
| 0 | frames stop | HardFault ⇒ reset now; wedge ⇒ silent; brownout ⇒ power-on reset | last command held (M3) | RC9 off works at once (M4) | ⏳ T056 (B4, H3) |
| ~1.4 s | pilot's channels, incl. RC6 | in its 8 s boot delay | pilot flying, pilot's throttle | flies | ⏳ T056 (B4) |
| ~8.5–9 s (wedge ~38.5 s) | accepts the override after 0.7 s | if armed and RC9 high: **re-engages by itself**, new origin (M5) | the policy takes the aircraft back | no action needed from the pilot: that is the hazard | ⏳ T056 (B5, H3) |

xiao live but commanding garbage (R16 S5): bounded only by the 60 s run timeout, path end, RC9, disarm or MSP failure, and a
`flight_path.empty()` early return may skip the timeout. Escape: RC9 off.

| change | effect |
|---|---|
| L2 (no resume after an in-flight reboot) | the ~9 s / ~39 s re-engage is gone until a disarm → arm |
| L6 (1 s control-loop watchdog) | a wedge resets in ~1 s instead of 30 s (then L2 blocks the re-engage) |
| L3 (bounded run on bad output) | garbage output stops within a tick; the timeout skip is fixed |
| L4 (neutral, motor-off frames) | covers stops the xiao *chooses* (timeout, path end, bad output); a crash, wedge or brownout sends nothing, so no change there |
| L7 `failsafe_delay` 5 (3) | the held window 1.4 s → 0.9 s (0.7 s) |

**Throttle to 0 first**: no. ~1.4 s at the last throttle, then whatever the pilot's throttle stick says. **Re-arm**: none needed
(INAV stays armed). **Pilot actions**: 1, RC9 off, at once after any xiao glitch (the pilot brief's mitigation for the re-fly).

## GPS brownout

GPS marked lost after 1 s, restarted after 0.5 s, ~1–3 s to re-establish comms, then ~1–2 s (hot) to ≥ 25 s (cold) to a fix; the
estimator drops GPS after 1.5 s (R17). ACRO is unaffected (R8); the policy keeps flying on a drifting frame, and the state message
carries no validity (R8 table), so nothing stops it before the path end or 60 s (R16 S8). RC8 RTH / POSHOLD is refused. A DROP
during the outage is unaffected; a re-arm during it hits M6. **Throttle to 0**: no (no fault is declared). **Pilot actions**: 1,
RC9 off. Multi-GNSS on the re-fly (change set rows 1–2) gives more satellites; its effect on reacquisition: ⏳ T056 (H6).
Candidates: no ladder step targets it; the offline flags (US5, T053) show it after the flight, and the deferred state message v2
(BACKLOG § 045 deferrals) is what would let the xiao act on it. LAND does not need GPS; RTH does.

## MSP flood

The PID loop is protected (REALTIME), and the RX task (HIGH, `fc/fc_tasks.c:527-532`) runs ahead of the MSP serial task (LOW,
`fc_tasks.c:488-492`), so RC9 should still work *(inf.)*. A flood delays or starves the companion's RAW_RC frames: fully starved ⇒
M3, then the pilot; partly starved ⇒ late, irregular frames with the override still on *(inf.)*. **While disarmed** (after a DROP),
non-MSP bytes are evaluated (`fc_tasks.c:113`): a stray `reboot_character` reboots into the bootloader, a `#` enters CLI (R17), and
either ends the flight with no recovery. ⛔ H5 stays held until a verified hard power cycle exists.

| change | effect |
|---|---|
| L5 (R-3, measure first) | today's traffic against the caps (B10); a guard only on a path that can exceed one |
| L8 (R-7) | no bootloader / CLI from the companion's UART, armed or not |
| L7 `failsafe_delay` 5 | the starved-override hold 1.4 s → 0.9 s |

## In-flight re-arm paths

| path | applies after | gates (source) | earliest | pilot actions today | with R-6 |
|---|---|---|---|---|---|
| **re-arm after DROP** (today's only path) | RX loss, TX reboot | link up (0.7 s of good data) + 3 s, with the ARM switch off when the 3 s end (`failsafe.c:594-611`); then throttle low (`fc_core.c:228-236`), roll / pitch centred (`FW_LAUNCH`, `fc_core.c:238-244`), RC link; M6 if the GPS heading is invalid | link return + ~3.7 s | **5**: RC9 off, throttle low, ARM off, ARM on, throttle up (4 without RC9; RC9 off guards against a xiao that rebooted meanwhile, M5) | the transmitter holds throttle low and cycles ARM after a fixed wait (≥ ~4 s, since it cannot see INAV's timer): 0–1 actions; the wait stays |
| control back under LAND (R-1, not today) | RX loss, TX reboot | none: still armed; stick motion once the link is up (`failsafe.c:550-555`) | link return + 0.7 s | 1 (sticks) with L1; 2 (RC9 off, sticks) without | not needed |
| re-arm after LAND's timed disarm | a LAND longer than `failsafe_off_delay` (20 s) | as DROP, but **30 s** of good link (`failsafe.c:576-580`) | link return + ~30.7 s | 4–5 | as DROP |
| in-flight emergency re-arm | a **switch** disarm only | within 5 s, landing detector active and a valid GPS heading; then **every arming blocker is bypassed** (`fc_core.c:514-534, 559-562`) | at once | 2 (ARM off, ARM on) | — |
| emergency arming (10 toggles in 10 s) | any disarm | overrides only GEOZONE, NOT_LEVEL, NAVIGATION_UNSAFE, compass / accel not calibrated, ARM_SWITCH, HARDWARE_FAILURE (`fc/runtime_config.h:68-74`; `fc_core.c:377-387, 478-511`). **Not** FAILSAFE_SYSTEM, RC_LINK, THROTTLE or SENSORS_CALIBRATING | — | no help after DROP or an FC reboot | a toggle burst from the transmitter cannot get past those either |
| re-arm after an FC reboot | FC reboot / L9 reset | SENSORS_CALIBRATING (above), RC link, ARM off → on, throttle low, sticks centred, M6 | config of record: never in flight; re-fly FC: maybe in a steady glide ⏳ | 4 | no help with the calibration gate; R-5 is the fix |

**R-6, the transmitter-side re-arm** (operator's earlier approach: re-assert arm when motion is detected, held in transmitter
global variables). It can automate the ARM toggle and the low throttle, taking the DROP re-arm from 4–5 actions to 0–1. It cannot
shorten INAV's 0.7 + 3 s, cannot pass the calibration gate after an FC reboot, must survive a transmitter reboot to help in that
case, and still meets M6. With LAND it is needed only after the timed disarm. It is transmitter configuration, not firmware, but
it changes the re-arm path, so it is bench-tested and flown as its own ladder step (L7).
