# Recorder Status-LED Reference Card

> **MOVED 2026-06-22 from `specs/031-beacon-camera/`. Parked 040 camera-redo reference — predates the 031 1-bit phase and is likely stale; re-validate (20 Hz / 480 fps / 200 Hz / 75 ms baseline + 031 field findings) on 040 restart. See [`README.md`](README.md).**

**FR ref**: FR-2.6 (status indicator) + Clarifications Session 2026-05-17 (heartbeat blink design).
**Hardware**: Onboard R/G/B tri-color LED on the Lattice CrossLink-NX-EVN board, GPIO-driven by the soft-CPU.
**Operator workflow**: glance at the LED before throttle-up. **GREEN heartbeat blink at ~1-2 Hz is the "go" signal**. Anything else = investigate.

## Quick reference (print + tape to recorder housing)

| LED behavior | Meaning | Operator action |
|---|---|---|
| **RED solid** (immediately after power-on) | Pre-init — sensor + filesystem coming up | Wait 1-2 seconds |
| **GREEN blinking, 250 ms period** (steady fast blink, regular) | Init phase — sensor enumerated, FS mounted, clip pre-allocating | Wait for steady-on transition |
| **GREEN heartbeat pulse (~100 ms ON, ~1-2 Hz cadence)** | **RECORDING — chunk-flushes succeeding** | **GO FOR TAKEOFF** |
| **YELLOW blinking, 100 ms period** (rapid alert) | Mid-flight transient fault (SDIO retry, ring overrun, sensor desync) — Class 1 recoverable | Continue; heartbeat returns after recovery (~200 ms) |
| **RED solid** (after recording was active) | Unrecoverable fault: SD-full / persistent SDIO failure / HyperRAM error — Class 2 | LAND IMMEDIATELY. File is sealed; pull card on ground |
| **RED blinking, 250 ms period** | SD card missing or unreadable at boot | Power down; reseat card or use a different card; reboot |
| **No change visible — LED frozen at any color, no blink** | **Main loop hung** — soft-CPU stopped servicing the chunk-flush event | DO NOT TAKEOFF / LAND. Power-cycle the recorder and observe boot sequence |

## Why "heartbeat blink" instead of "solid GREEN"

A truly silent failure mode would be: the soft-CPU hangs (firmware bug, stack overflow, deadlock) while the LED is held high. The operator sees GREEN solid and assumes recording is fine. By tying the GREEN-on state to an actual chunk-flush event (CMD25 completion), a hung main loop simply stops emitting pulses — the LED freezes at whatever state it was last set to, and **the operator's eye is trained to look for the *blink*, not the *color***. No hardware watchdog required.

## Heartbeat-blink cadence vs capture mode

| Capture mode | Chunk period | Heartbeat pulse rate |
|---|---|---|
| 240 fps, 8-bit | ~1 sec | ~1 Hz |
| 240 fps, 10-bit | ~1 sec | ~1 Hz |
| 480 fps, 8-bit | ~0.5 sec | ~2 Hz |
| 480 fps, 10-bit | ~0.5 sec | ~2 Hz |

Both rates are comfortably in the 1–4 Hz human-visible "heartbeat" range. A double-flash within ~500 ms is the upper bound.

## Pre-flight checklist (operator)

1. Insert SD card. Power up recorder via USB-C / carrier-craft battery.
2. Watch the LED progression: RED-solid → GREEN-blink-250ms → first GREEN pulse.
3. Once first pulse fires, observe ≥3 pulses to confirm steady cadence.
4. **If cadence is wrong** (e.g., one pulse then frozen, or pulses at much lower rate than expected): investigate before flight.
5. **Throttle-up only after confirming the heartbeat blink is steady.**

## Post-flight inspection (loader-side)

The recorder's status LED tells you what happened during the flight at the operator-visible level. For the per-chunk fault list, ingest the clip via the loader and inspect `metadata['fault_events']`:

```python
from beacon_loader import load_clip
frames, metadata = load_clip("session-001.clip")
for ev in metadata["fault_events"]:
    print(f"  t={ev['timestamp_us']/1e6:.3f}s  {ev['fault_name']}  (offset={ev['offset_bytes']})")
```
