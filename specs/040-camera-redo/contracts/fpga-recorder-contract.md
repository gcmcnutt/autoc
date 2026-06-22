# Contract — Lattice CrossLink-NX FPGA Recorder Gateware

**Spec ref**: FR-2.4 (timestamp source), FR-2.5 (SD recording), FR-2.6 (status LED), Principle V
**Reference impl**: `firmware/flight-recorder/rtl/` (Verilog) + Propel project at `firmware/flight-recorder/propel-project/`
**Toolchain**: Lattice Propel (RISC-V soft-CPU + SDIO driver) + Lattice Radiant for gateware build

## Single-line description

> Configure the camera once over I²C, ingest MIPI frames into an SDRAM ring buffer, write chunked binary clips to microSD via 4-bit SDIO, blink a status LED. Phase 1 — no DSP.

## Block diagram

```
                       ┌─────────────────────────────────────────────────────┐
                       │ LIFCL-40 FPGA fabric                                 │
                       │                                                       │
[Camera Module]        │  ┌─────────────────┐    ┌─────────────────────────┐ │
OG0VA / OV9281         │  │ MIPI D-PHY      │    │ Soft RISC-V CPU         │ │
  │ MIPI CSI-2 ────────┼──┤ Hard IP         │    │ (Propel reference)       │ │
  │ (1- or 2-lane)     │  │ → Bayer/Mono    │    │  - I²C camera config     │ │
  │                    │  │   pixel stream  │    │  - SDIO driver           │ │
  │ I²C control ───────┼──┼──────I²C────────┼────┤  - Status-LED FSM        │ │
  │                    │  └────────┬────────┘    │  - Chunk-header writer   │ │
  │                    │           │             │  - Timestamp counter     │ │
  │ Sensor clock       │           ▼             └─────────────┬───────────┘ │
  │ (internal,         │  ┌─────────────────┐                  │             │
  │  free-running)     │  │ Line-buffer     │                  │             │
  │                    │  │ (BRAM/LRAM      │                  │             │
  │                    │  │  double-buf)    │                  │             │
  │                    │  └────────┬────────┘                  │             │
  │                    │           │                            │             │
  │                    │           ▼                            │             │
  │                    │  ┌─────────────────┐                  │             │
  │                    │  │ HyperRAM        │                  │             │
  │                    │  │ Controller      │  ◄───────────────┘             │
  │                    │  │ + ring-buffer   │                                 │
  │                    │  │ mgr             │                                 │
  │                    │  └────────┬────────┘                                 │
  │                    │           │                                          │
  │                    │           ▼                                          │
  │                    │  ┌─────────────────┐                                 │
  │                    │  │ SD writer       │                                 │
  │                    │  │ (4-bit SDIO     │                                 │
  │                    │  │  ≥25 MHz)       │                                 │
  │                    │  └────────┬────────┘                                 │
  │                    │           │                                          │
  └────────────────────┘           ▼                                          │
                       ┌─────────────────────────────────────────────────────┘
                       │
                  [microSD V30/V60]
```

## Boot sequence (FPGA power-up)

| Step | Action | Status LED |
|---|---|---|
| 1 | FPGA bitstream loads from EVN board flash | (off) |
| 2 | Soft-CPU starts; initialize HyperRAM controller | RED solid |
| 3 | Mount SD card (verify CMD0 + CMD8 + CMD55/41 init) | RED solid → GREEN blinking 250 ms |
| 4 | Configure sensor over I²C: 320×240 cropped ROI, manual exposure + gain, fps mode (240 or 480 per config switch), bit-depth (8 or 10) | GREEN blinking |
| 5 | Pre-allocate the next session's clip file (write file header to first sector; reserve ~10 min of contiguous sectors) | GREEN blinking |
| 6 | Enable MIPI ingest; start the ring-buffer manager | GREEN blinking (250 ms init pattern) |
| 7 | First chunk's worth of frames lands in HyperRAM; SD writer starts writing | (transient — LED may flash off then resume init blink) |
| 8 | First SDIO chunk-write CMD25 completes | **First GREEN heartbeat pulse fires** (~100 ms ON) — this is the "alive + recording" transition |
| 9 | Steady-state: every CMD25 completion → ~100 ms GREEN pulse | **GREEN heartbeat blink ~1 Hz @ 240 fps mode / ~2 Hz @ 480 fps mode** |

Fault paths: any SD error / ring overrun / sensor disconnect → RED solid or YELLOW blinking per FR-2.6 table.

## Timestamp source contract (FR-2.4)

- **Authoritative clock**: FPGA-internal 50 MHz oscillator counter (LIFCL-40 has on-chip oscillator + PLLs; the soft-CPU drives a 64-bit µs counter from a PLL-divided clock).
- **Sensor frame-counter** (if any from OG0VA / OV9281): captured as a secondary field; NOT authoritative.
- **Per-frame timestamp**: on each MIPI frame-end event, latch the µs counter; store as `chunk_timestamp_us` (first frame in chunk) + per-frame delta (uint16).
- **Per-chunk timestamp**: equal to the per-frame timestamp of the first frame in that chunk.
- **Wrap-around**: 64-bit µs counter never wraps in practice (~580 000 years).

## Ring buffer contract (FR-2.5)

| Parameter | Value | Source |
|---|---|---|
| Total ring size | ≥250 ms of worst-case frame data ≈ 6 MB (480 fps × 10-bit packed) | FR-2.5 |
| Backing memory | EVN-board HyperRAM (sized per R3 exec-research; expected 8-16 MB) | R3 |
| Producer | MIPI ingest, writes complete frames as they arrive | continuous at sensor fps |
| Consumer | SD writer, reads in chunk-sized blocks (1 sec ≈ 240 or 480 frames) | bursty per SD multi-block-write cycle |
| Overflow handling | Producer wins, oldest unwritten frame is dropped. Drop count is tracked + surfaced via status LED (YELLOW blink on drop). |

## SD chunk-write atomicity contract

Per FR-2.5: each chunk is independently parseable. To achieve this:

1. SD writer prepares one chunk worth of bytes in a contiguous HyperRAM region.
2. Compute CRC-32 over the chunk's body; append as trailing 4 bytes.
3. Issue SD `CMD25` (multi-block write) covering all sectors of this chunk (round up to next 512-byte boundary; pad with zeros — pad bytes are inside `chunk_size_bytes` so the loader skips them naturally).
4. On `CMD25` completion → chunk is durable; next chunk starts at the byte-aligned next offset.

## Fault handling — reset and continue (revised 2026-05-17)

The recorder **MUST NOT halt on recoverable faults**. Frame data dropped during a reset is acceptable; a recorder that freezes for the rest of the flight is not. The contract distinguishes three fault classes:

### Class 1 — Transient / recoverable (continue same file)

| Fault | Detect | Reset action | Sentinel written |
|---|---|---|---|
| Single `CMD25` timeout / NACK | SDIO controller error flag | Retry `CMD25` once at the same offset | If retry succeeds: `SDIO_WRITE_ERROR_TRANSIENT` sentinel inserted before the next data chunk. If retry fails → escalate to Class 2 |
| Ring-buffer overrun (producer faster than consumer) | HyperRAM ring manager write-pointer overlaps read-pointer | Increment drop-counter; advance read-pointer past the lost frames; continue ingest | `RING_BUFFER_OVERRUN` sentinel after the consumer catches up |
| Sensor I²C glitch (config register read-back mismatch) | Periodic readback check by soft-CPU | Re-issue sensor init sequence over I²C | `SENSOR_I2C_RECOVERED` sentinel if re-init succeeded |
| MIPI D-PHY desync | MIPI hard-IP loss-of-lock signal | Re-train MIPI link (~10 ms) | `MIPI_DESYNC_RECOVERED` sentinel if re-train succeeded |

Recovery action sequence on any Class 1 fault:
1. Pause MIPI ingest into the ring (let in-flight frames drain into HyperRAM).
2. Status LED → YELLOW blinking (100 ms period).
3. Execute the reset action above.
4. Compose the fault-sentinel chunk in HyperRAM (`frame_count = 0`, `fault_code = <Class 1 code>`, `chunk_timestamp_us = FPGA µs at fault detection`).
5. Issue `CMD25` for the sentinel chunk.
6. Resume MIPI ingest.
7. Status LED → GREEN solid (recording).

Total reset time target: **≤200 ms**. Frame loss bounded to this window.

### Class 2 — Unrecoverable but card present (seal file)

| Fault | Detect | Reset action |
|---|---|---|
| SD-full | SDIO write-protect or out-of-space response from card | Stop |
| Repeated `CMD25` failure (retry + SD re-init both fail) | Two consecutive fault sequences within 1 second | Stop |
| HyperRAM controller fault (memory subsystem unrecoverable) | HyperRAM controller error flag | Stop |

Sequence:
1. Write a final `FINAL_UNRECOVERABLE` fault-sentinel chunk with `fault_code = 0x000000FF`.
2. Issue SDIO `CMD12` (stop transmission) + ensure all in-flight writes have completed (`CMD13` status poll).
3. Status LED → RED solid.
4. Halt the soft-CPU's recording loop. The bitstream stays loaded — operator can power-cycle to reset, or pull the card.

### Class 3 — Brown-out (no recovery possible)

| Fault | Detect | Action |
|---|---|---|
| Board-level UVLO trip (supervisor signal on a GPIO) | Edge-triggered IRQ on supervisor pin | Issue `CMD12` only — best-effort; no sentinel possible because power is being cut |

Frames in-flight in HyperRAM but not yet committed to SD are lost (~1 sec worst case). The file is left as whatever was last on the SD card; loader's chunk-magic-mismatch stop is the recovery path.

### State invariants the recorder MUST preserve

- **The pre-allocated file is never closed in the middle of a `CMD25`** — the SDIO driver waits for the in-flight CMD25 to complete (or fail) before any state transition.
- **No two chunks share a byte offset** — each fault sentinel + each subsequent data chunk lives at a unique, sequential file offset.
- **Sentinel chunks always precede the subsequent data chunks at the same `chunk_timestamp_us` or later** — sentinels mark "from here forward, something changed."
- **Status LED accurately reflects current state** at all times (no stale GREEN-solid during a reset).

### Testing contract (P2/P3 gates)

- **Synthetic SD-write-error injection**: a config flag in the bitstream lets the soft-CPU artificially fail every Nth `CMD25`. Bench-run a 1-min recording with N=200 (a fault every ~5 sec); verify (a) recorder produces a file with embedded sentinels, (b) the loader successfully ingests it + reports the fault events in metadata, (c) total frame-loss is bounded to <200 ms per sentinel.
- **Synthetic ring-overrun**: throttle the SDIO write rate via a config flag. Verify overrun sentinels appear at the expected cadence + recording continues.
- **SD-full test**: pre-fill an SD card to within ~50 MB of full; bench-record until the card fills. Verify `FINAL_UNRECOVERABLE` sentinel + clean LED-RED-solid stop.
- **Power-yank test**: bench-record for 30 sec, yank USB-C mid-write. Verify the file (whatever survived) loads cleanly via the loader's chunk-magic-mismatch stop path.

These four tests collectively validate the resilience contract before US6.

## File header

Written once at session start, immediately after SD initialization, before the first MIPI frame. Per `data-format.md`:

```
[0..3]   magic = 0x0BEAC031
[4..5]   format_version = 1
[6..7]   header_size = 16
[8..15]  session_start_us (FPGA-counter snapshot when SD-mount completes)
```

## Status-LED contract (FR-2.6)

Per the FR-2.6 blink-code table. State machine driven by the soft-CPU. RED/GREEN/YELLOW are the three native colors on the EVN board's onboard tri-color LED (repurposed).

### Heartbeat blink — soft-CPU liveness via chunk-flush pulses (revised 2026-05-17)

The "recording, no errors" state is **NOT** a steady GREEN-solid; it is a **GREEN heartbeat blink driven by the SDIO chunk-flush event**:

```c
// Inside the soft-CPU's main recording loop, immediately after CMD25 success:
on_chunk_flush_complete() {
    led_set(GREEN, ON);
    schedule_after_ms(100, [] { led_set(GREEN, OFF); });
    // next pulse will fire on the next CMD25 completion (~1 sec or ~0.5 sec later)
}
```

- Pulse width: ~100 ms ON (visible to the operator's eye without strobing artifacts)
- Pulse rate: equal to chunk-flush rate = **~1 Hz at 240 fps mode (1-sec chunks), ~2 Hz at 480 fps mode (0.5-sec chunks)** — both well within the 1–4 Hz human-visible heartbeat range
- **Soft-CPU hang detection**: if the main loop stops calling `on_chunk_flush_complete`, the LED freezes at its last state (typically solid GREEN if the pulse was high when the hang hit, or solid off if the hang hit between pulses). **The operator's visual cue is absence of blink**, not any active fault-state indicator. Combined with the FR-2.6 pre-takeoff checklist ("confirm GREEN heartbeat blink before throttle-up"), this catches the silent-hang failure mode without requiring a hardware watchdog. (Optional future: add a true hardware watchdog as belt-and-suspenders in 031-fpga.)

### Fault-state overrides

The heartbeat-pulse pattern is overridden during fault states per the FR-2.6 table:
- YELLOW 100 ms blink during a Class 1 transient-fault recovery window (~200 ms) — replaces the heartbeat for that window; heartbeat resumes after the fault sentinel is committed.
- RED solid on Class 2 unrecoverable fault — overrides forever (no return to heartbeat).
- LED is unchanged on Class 3 brown-out (no firmware control once power is gone).

## Performance budget

| Path | Required | Available |
|---|---|---|
| MIPI ingest | 480 × 320 × 240 × 1.25 B = **46.1 MB/s** sustained | LIFCL-40 MIPI hard IP: 1.5 Gbps/lane × 2 lanes = **375 MB/s** raw → ample |
| HyperRAM W+R | 46.1 + 46.1 = **92.2 MB/s** aggregate | HyperRAM 200 MHz DDR ×8 = **400 MB/s** raw → ample (margin for soft-CPU overhead) |
| SDIO write | **46.1 MB/s** sustained | 4-bit SDIO × 25 MHz × 4 bits = **12.5 MB/s** floor; Propel's optimized SDIO hits **40 MB/s** in published reference → **need to verify the EVN board's SDIO + a SanDisk Extreme Pro V60 card hit 46 MB/s sustained** |

**Bench gate**: P3 first scenario sweep — verify the SD writer hits sustained 46 MB/s with no drops at the 480 fps 10-bit mode. If it doesn't, fall back to: (a) 480 fps 8-bit mode (37 MB/s) or (b) 240 fps 10-bit (23 MB/s) or (c) 240 fps 8-bit (18 MB/s). Mode is a config field; can be reduced without firmware re-flash.

## What the gateware does NOT do (Phase 1)

- No DSP. No background subtract, no CCA, no correlator, no centroid extraction. That's all 031-fpga.
- No I²C output to a host. No (x, y, CEP) emission.
- No multi-camera support. Single-sensor only.
- No body-rate feed-forward, no IMU input.
- No mid-flight bitstream reconfiguration. Fixed at boot.

These boundaries are what makes Phase 1 small enough to deliver alongside the hardware bring-up.
