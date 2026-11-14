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
