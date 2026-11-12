# Contract: BLE flight-log download, v2

Service `F1706000-…` (unchanged UUIDs): control `…6001` (write), data `…6002` (notify), status `…6003` (notify).

## Step 1 — instrumentation (ships first, protocol unchanged)

- Firmware status lines: `MTU:<att_mtu>` at connect; `STAT:<offset>:<short_writes>:<max_loop_ms>:<max_next_gap_ms>` every 16
  chunks and in `DOWNLOAD_DONE`.
- Page log (in memory, **Copy log** button): READY time; per chunk: send time, receive time, `byteLength` vs expected, offset;
  errors with offset; disconnect offset and elapsed time; summary of bytes, seconds, kB/s, CRC result.

## Step 2 — protocol v2

| page → firmware (control) | firmware → page |
|---|---|
| `DL2:<file>:<offset>:<window>` | status `READY2:<size>:<chunk>:<mtu>` (chunk = MTU − 3 − 4) |
| — | data notifications, each `[u32 offset][payload]`, sent back-to-back up to `window` chunks unacknowledged |
| `ACK:<offset>` (the highest contiguous byte received) | the window slides; resend from `offset` on a gap |
| `ABORT` | stop, clear transfer state |
| — | status `DONE2:<size>:<crc32>` at the end; `ERR:<reason>:<offset>` on failure |

Rules:
- The firmware **requests MTU 247** after connect. If a notify's returned length is < chunk + 4 it **stops and reports
  `ERR:SHORT_NOTIFY`** (fail loud; never truncate silently).
- **Resume**: a new `DL2` with the next needed offset continues; the page keeps the received buffer across reconnects, keyed by
  file name + size.
- **Inactivity timeout**: no ACK for 5 s ⇒ the firmware clears transfer state and returns flash to idle (fixes the wedge at
  `flash_logger.cpp:491`).
- While a download runs **on the ground (disarmed)**, `controllerUpdate` skips MSP polling. Arming still stops BLE as today.
- Whole-file CRC32 is still checked at the end.

Acceptance: spec SC-002 (~300 kB < 60 s with the phone ~3 m from the aircraft; an interrupted download resumes and is byte-identical to a wired read).

## As built (2026-11-12, tasks T018–T022; measured from the training host's Linux central)

- **v1 (`DL:` / `NEXT`) is removed** (clean cut, Constitution III). Control accepts write **and write-without-response** (the
  page sends ACKs without response).
- `DL2:<file>:<offset>:<window>` → `READY2:<size>:<chunk>:<mtu>`, chunk = min(MTU − 7, 240). The firmware does **not** start an
  MTU exchange (ArduinoBLE's peripheral path can't); centrals do (BlueZ: 23 at connect, 247 by `READY2`). MTU 23 ⇒ 16 B chunks
  (works, slowly); MTU ≤ 7 ⇒ `ERR:MTU`.
- Data: `[u32 offset LE][payload]`, one notification each, up to `window` in flight. **The page defaults to window 2; the
  firmware caps it at 8.** Measured: window 2 = 15.6 kB/s with no loss; 4 = 17.0 kB/s with a few recovered gaps; 8 = heavy loss
  and a disconnect, because ArduinoBLE's 256 B HCI receive ring drops events when a burst fills it.
- Page rules: accept in order; ignore a duplicate by offset; ACK the highest contiguous byte every max(1, window / 2) chunks,
  at the end, and on a gap or a duplicate at most once per 100 ms; **one GATT write at a time** (a newer ACK while one is in
  flight marks pending, and the latest value goes next).
- Firmware: go back to the ACK on a non-advancing ACK (once per 100 ms) or after 400 ms without progress; no ACK for 5 s ⇒
  `ERR:INACTIVE:<offset>` and the flash returns to idle. A notify shorter than chunk + 4 ⇒ `ERR:SHORT_NOTIFY:<offset>`.
- End: `STAT:<acked>:<notifies>:<rewinds>:<max_loop_ms>:<read_ms>:<notify_ms>`, then `DONE2:<size>:<crc32>`. The CRC is a
  separate whole-file pass (resume and go-back-N send bytes out of order). Errors: `ERR:<MTU|NOFILE|RANGE|SEEK|READ|SHORT_NOTIFY|
  INACTIVE|BUSY|BADCMD>:<offset>`; `ABORTED` after `ABORT`.
- Resume: a drop keeps the page's buffer, keyed by file name with its size checked at `READY2` (a changed size restarts from 0).
  Reconnect and press Download again: the page sends `DL2:<file>:<expected>`.
- The firmware asks for a 7.5–15 ms connection interval (`BLE.setConnectionInterval(6, 12)`); BlueZ granted ~10–12 ms.
  2M PHY is not exposed by ArduinoBLE. While a download runs on the ground, the main loop polls INAV at 2 Hz instead of every NN
  tick: not skipped outright, because the state poll is how an arm is seen, and arming stops BLE.
