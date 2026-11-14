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
