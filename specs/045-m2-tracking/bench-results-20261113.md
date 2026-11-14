# 045 bench results — 2026-11-13, article 1 (BLE download v2 acceptance, T017 / T023)

Setup: xiao (045 dev firmware, download v2) and FC on the DGX's USB; for the arm test, the flight battery connected for the
receiver, **prop off**, throttle at minimum, TX linked, 21 satellites indoors. Downloads from the operator's field laptop:
**Edge 157 on Windows 10** (the operator downloads with Chromium on the DGX and Edge on Windows; no phone). The xiao console was
watched live from the DGX. File: `flight_001.bin` (11-08, 307 968 B, CRC `645AC948` = the wired USB dump = the committed log).

| test | result |
|---|---|
| Edge with the **old page** (before 3d6bc62) | hangs at "Downloading... (wait for status notification)": the old page sends `DL:`, the v2 firmware answers `ERROR:Unknown command`, and the old page overwrites that line. Fixed by loading the current page (heading now says "download v2 · resumable"; 8014ae9) |
| Edge, full download | **20.9 s, 14.4 kB/s**, 1 284 chunks, 0 dup / 0 gap / 0 errors, CRC OK; MTU 247 negotiated by Windows; xiao: 20 782 ms, 0 rewinds ([log](bench-20261113/ble_edge_windows_full_download.txt)) |
| interrupt by **page reload**, reconnect | restarted from 0 (the partial lived only in page memory) ⇒ fixed: the partial now persists in localStorage (83c1354) |
| interrupt by **walking out of range / Disconnect button**, reconnect | dropped at 106 080 / 113 520 / 204 960 B with 7 rewinds as the link faded; **resumed** from 113 520 (`DL2 … from 113520`) |
| interrupt by **xiao reset** (console `RESET`, 6 s in) | the page kept its buffer; after the reboot the download **resumed** and finished OK (operator) |
| **arm mid-download** (T023): ARM on the TX 6 s in | within the 2 Hz poll the xiao logged `BLE disconnecting active central due to arm state`, `BLE advertising disabled`, then `Flash logger armed - recording flight #2` (~150 ms later; BLE stops before the flight file opens); heartbeat `armed=Y` |
| disarm, reconnect, resume | flight #2 finalized, `BLE advertising enabled`; LIST shows 2 files; `flight_001.bin` **resumed from 79 440**, 953 chunks, 17.3 s, CRC `645AC948` |

Not exercised on the bench: a > 5 s ACK stall without a disconnect (the inactivity path is unit-tested in `ble_dl2_tests`), and
the download-time MSP slow poll directly (shown indirectly: the arm was noticed within ~0.5 s during a download). Laptop distance
not recorded (desk, a few metres).

## Safety "before" record (T056), started — receiver failsafe mode (A7)

Tool: [`fc_watch.py`](fc_watch.py) — read-only MSP watch of the FC's USB port at ~20 Hz (arm state, FAILSAFE / MSP RC OVERRIDE /
MANUAL / ANGLE by permanent box id, RC_LINK / FAILSAFE_SYSTEM arming flags, merged channels); CSV
[`bench-20261113/fc_watch_safety.csv`](bench-20261113/fc_watch_safety.csv). Box positions read back: ARM 0, ANGLE 3, MANUAL 20,
FAILSAFE 29, MSP RC OVERRIDE 40 (as bench A3).

| test | result |
|---|---|
| **A7: TX off (disarmed), then on** — article 1, SBUS receiver | TX off at 16:21:46 ⇒ INAV set **`rc_link_lost=1` and `FAILSAFE=1`**; channels kept their last good values (INAV's own hold; throttle 987). TX on ⇒ cleared at 16:22:03. ⇒ The receiver signals the loss (**no pulses on signal loss**, set intentionally by the operator), not a canned "hold" pattern INAV can't detect. |

### Test B — xiao reboot while engaged (B4, B5 / H3), and the hold after a normal stop (S6)

Setup: battery on, **prop off**, throttle low, TX linked; arm in MANUAL, "launch", RC9 low → high (autoc engages); the console
`RESET` sent automatically 3 s (path 0) / 5 s (path 6) after `NN Control: Switch enabled`; RC9 held high and armed through the
reboot. Timings from `fc_watch.py` at ~25 Hz ([path 0 CSV](bench-20261113/fc_watch_testB.csv) — its header predates the column-order
fix: the four mode columns are ANGLE, MANUAL, FAILSAFE, OVERRIDE in that order; [path 6 CSV](bench-20261113/fc_watch_testB2_path6.csv)).
"Held" = from the last change of the xiao's command to the hand-back (RC6 back to the pilot's switch).

| run | event | measured |
|---|---|---|
| path 0 (StraightAndLevel) | normal **path-complete stop** (17.1 s) | INAV held the last command **1.43 s** — roll 1916, pitch 1980 (near full rates), throttle 1021 — then the pilot's sticks |
| path 0 | **xiao RESET** 3 s after engage | held **1.40 s** — roll 1420, pitch 1159, **throttle 1987** — then the pilot's sticks |
| path 0 | after the reboot (boot banner 8 s after the reset) | **the xiao re-engaged by itself 9.6 s after the reset** (armed, RC9 high, no pilot action), new origin, opening with roll 1023 |
| path 0 | pilot disarm while re-engaged | motor off; channels held ~1.3 s, then the pilot's |
| path 6 (index 5, SeededRandomB) | **xiao RESET** 5 s after engage | held **1.35 s** — roll 1136, pitch 1117, **throttle 1991** |
| path 6 | after the reboot | **re-engaged by itself 9.9 s after the reset**, throttle 1963; flew the whole path (40.2 s) |
| path 6 | path-complete stop of the re-engaged run | held **1.38 s** — roll 1169, pitch 1129, **throttle 1992** |
| both | **RC9 low** (the escape) | override off within one sample (~40 ms); the xiao logged "Servo reset detected - autoc re-arm allowed" |

**Confirmed hazards (today's firmware and configuration)**:
1. **Every stop** (path end, xiao reset; by R16 also a timeout or MSP failure) leaves INAV flying the xiao's last command,
   **throttle included (up to 1992 µs here)**, for **1.35–1.43 s** (×4), as R16 predicted (200 ms + 200 + `failsafe_delay` × 100).
   ⇒ ladder **L4** (neutral, minimum-throttle frames on every stop).
2. **A xiao reboot while armed with RC9 high re-engages autoc by itself ~9.6–9.9 s later** (×2), from a new origin, with no pilot
   action (R18). ⇒ ladder **L2** (no resume until disarm → arm). For the re-fly the mitigation is the pilot brief: **RC9 off after any
   xiao glitch**.
3. RC9 low hands control back at once, every time (the escape works, R8).

### B6 — transmitter off while engaged (today's DROP, `failsafe_delay` 10)

Setup as Test B (battery on, **prop off**, TX linked); arm, "launch", RC9 low → high, then the TX switched off by hand a few seconds
into the run. No reset script. [`fc_watch.py`](fc_watch.py) at ~24 Hz, CSV
[`bench-20261113/fc_watch_testB6_txoff.csv`](bench-20261113/fc_watch_testB6_txoff.csv) (columns as the header); xiao flights #9, #10.

| run | event | measured |
|---|---|---|
| 1 (16:47, path 6 = index 5, SeededRandomB) | TX off; the link came back 18.3 s after the disarm | **autoc flew on until the disarm**: the merged roll ramped smoothly 1318 → 1379 through the last armed sample |
| 1 | **DROP disarm** (t = 40.02 s) | disarm, `rc_link_lost` and FAILSAFE in the same ~40 ms sample; the xiao logged "Autoc disabled (disarmed)" in the same second |
| 1 | after the disarm | channels **frozen at the xiao's last command** (roll 1379, pitch 1164, throttle 1991, motor off) until the link returned; the operator saw the surfaces stay put |
| 1 | link back (t_L = 58.34 s) | the pilot's channels from the first sample (sticks centred, RC6 MANUAL, RC9 still high) |
| 1 | t_L + 0.72 s | `rc_link_lost` cleared |
| 1 | t_L + 3.72 s | FAILSAFE cleared (the pilot had put the throttle down and the ARM switch off while the TX powered up) |
| 2 (16:54) | **re-arm with RC9 still high** (from run 1, never lowered) | **no re-engage** for 2.2 s while armed (the stop latch survives the disarm → arm), then RC9 low ⇒ "Servo reset detected"; RC9 high ⇒ engaged normally |
| 2 (path 0) | TX power-cycled, then left alone | DROP disarm at t = 455.85 s (7 s after engage); surfaces frozen at **roll 1123, pitch 1122 — near full deflection on both axes** — throttle 1991 |
| 2 | link back (t_L = 469.33 s, 13.5 s after the disarm) | pilot's channels (MANUAL, throttle stick 1200, RC9 high); `rc_link_lost` cleared at t_L + 0.71 s |
| 2 | ARM switch left on | **FAILSAFE stayed active 188 s**, until the pilot switched ARM off; INAV never re-armed by itself |

**What this confirms (R16 S3, corrected 11-12)**:
1. A link loss of more than ~1.2 s while engaged is an **in-air disarm**: motor off, and the surfaces stay **wherever the xiao's
   last command left them** until the link returns; in run 2 that was near full roll and pitch. Unpowered with the surfaces
   deflected, the aircraft would spiral or dive until the TX is back. This is the strongest argument in the DROP vs LAND debate (R-1).
2. When the link returns, the pilot has the surfaces at once (motor off). Re-arming needs: ≥ 3 s of good link, **the ARM switch off**
   (`failsafe.c:599`: failsafe does not exit until it is), then ARM on with low throttle. No automatic re-arm; at least two switch
   actions (×2).
3. The xiao does not resume after the re-arm while RC9 stays high; RC9 low → high is required (×1, 2.2 s armed).
4. **Stage 1 = 1.19 s, at full throttle** (×3). MSP can't show it (INAV updates the RC_LINK / FAILSAFE_SYSTEM arming flags only while
   disarmed, and the FAILSAFE box comes on with the procedure, i.e. the disarm), so it was read from the INAV blackbox, pulled over
   MSP with the new read-only [`fc_blackbox_dump.py`](fc_blackbox_dump.py) (7.29 MB in 128 s, 55.7 kB/s; 8 logs, all
   `INAV 8.0.0 (38ff0d29)`). From INAV's loss detection (`rxSignalReceived` → 0) to the last frame (the disarm):

   | blackbox log | run | loss detected → `rxFlightChannelsValid` 0 | → disarm | motor during stage 1 |
   |---|---|---|---|---|
   | .06 (16:43 GPS time) | the operator's dry run | +0.30 s | **+1.20 s** | 1990 (the xiao's command) |
   | .07 | run 1 | +0.30 s | **+1.19 s** | 1990 |
   | .08 | run 2 | +0.30 s | **+1.19 s** | 1987 |

   `failsafePhase` reads IDLE in every logged frame: the log stops in the cycle the procedure starts. The receiver's own detection
   delay (TX off → `rxSignalReceived` 0) is not in the log; it adds to the 1.19 s. The blackbox clock (GPS) runs ~86 s ahead of
   the host's watcher timestamps (log .07 starts 16:45:48 for an arm the watcher saw at 16:47:13).

**New finding, xiao log**: both runs printed `Flight log: SpanSummary write failed (span 1)`. On a disarm the arm-transition block
closes the flight file ([msplink.cpp:729](../../xiao/src/msplink.cpp#L729)) before `stopAutoc` writes the span summary
([msplink.cpp:758](../../xiao/src/msplink.cpp#L758)). Any disarm while engaged (DROP, or a pilot disarm with RC9 still high) loses the
span summary; the per-tick records are intact. Filed in [BACKLOG](../BACKLOG.md) § 045 deferrals.

### B2 / B3 — mode switches while engaged (RC6, RC8)

Same setup (prop off, TX linked, **3D fix indoors, ~21 sats**); arm in MANUAL, launch, RC9 high (path 0), then RC8 to POSHOLD
(mid), RTH (high), low; RC9 low; disarm. [`fc_watch.py`](fc_watch.py) now also watches NAV POSHOLD / RTH and RC8
([CSV](bench-20261113/fc_watch_testB2B3_modes.csv)); INAV blackbox log `.09` (GPS time 20:00:14) pulled with
`fc_blackbox_dump.py --start 7293264` (606 kB, 11 s); xiao flight #11.

| t (blackbox) | event | measured |
|---|---|---|
| 4.87 s | engaged; **the pilot's RC6 sat at MANUAL throughout** | INAV flew ACRO (MANUAL off) on the xiao's RC6 = 1500: **B2 confirmed**, RC6 does nothing while engaged (also true of every engagement today, all armed in MANUAL) |
| 11.42 s | **RC8 → POSHOLD** | INAV **accepted NAV POSHOLD** (`navState` 7) and **took the throttle and surfaces from the xiao at once**: throttle 1988 (xiao) → 1500 → 1835 → **2000** (nav, demanding a climb on the bench), servos to nav's (servo0 1801, servo1 1983–2000) |
| same tick | the xiao | logged **"INAV failsafe activated (7.3 s) - disabling autoc"**: its hard-coded `MSP_MODE_FAILSAFE` bit 19 is article 1's **NAV ALTHOLD** box, which POSHOLD lights. Stopped, stop latch set |
| 11.42 → 12.81 s | the override's hold | the merged channels froze at the xiao's last values (as Test B) but **nav ignored them**: motor stayed nav's (2000) |
| 12.81 s (+1.38 s) | override stops merging | the pilot's channels return; **the pilot's RC6 MANUAL cancels POSHOLD** (`navState` → idle), throttle 1111 (stick), servos centred |
| 14.16 s | RC8 → RTH | box selected, but **not executed** (`navState` idle): the pilot's MANUAL blocks nav modes. The operator saw nothing move. ⇒ **RTH while engaged not exercised** |
| 19.35 s / 25.7 s | RC8 low / RC9 low | "Servo reset detected"; disarm at 29.5 s |

**What this shows**:
1. **With a valid nav estimate, RC8 POSHOLD is an immediate escape**: nav overrides the companion's merged channels the moment it
   engages (no 1.4 s hold on the motor or surfaces). The xiao stops too, but only because it misreads the NAV ALTHOLD bit as
   failsafe (R16 S1/S2 inference, now measured; ladder **L1**). With a bad estimate INAV refuses POSHOLD silently (R8), the bit
   never lights, and nothing happens: still **not** an escape to brief as reliable. RC9 low stays the escape.
2. **After the xiao's stop, the pilot's RC6 position decides what happens 1.4 s later**: in MANUAL, nav is cancelled and the
   pilot has raw surfaces and the throttle stick; in ACRO or ANGLE, POSHOLD would carry on. Brief point.
3. The xiao **never reads INAV's real FAILSAFE box** (position 29 here). Under DROP it stops on the disarm, so this costs nothing
   today; under LAND (armed through the failsafe) the xiao would not notice the failsafe at all. Input to the R-1 decision.
4. RTH while engaged: untested (it would need RC6 off MANUAL during the hand-back). *(inf.)* RTH also lights NAV ALTHOLD, so the
   xiao would stop the same way.

### GPS signal loss attempt (H6 variant) — not achieved

A metal pot over the article's GPS (Matek M8Q, its own patch antenna), disarmed: **18–19 → 11–13 sats, the 3D fix kept**
([CSV](bench-20261113/fc_watch_testB3rth_gps.csv), `fc_watch.py --gps`; the `eph_m` column reads −1 there, a length-check bug
fixed after the run). An open-bottomed pot doesn't attenuate enough to drop below `gps_min_sats` 6. Options: foil all round the
module (underside included) or a closed tin; otherwise the outdoor session / re-fly exercises GPS loss for real. The planned RTH
run did not happen (the TX went off at 20:14).
