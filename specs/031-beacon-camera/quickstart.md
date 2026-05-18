# Quickstart — 031 Beacon-Camera Phase 1

Three end-to-end walk-throughs. None require a printed PCB.

> Prereqs assumed: Linux PC, `avr-gcc`, `pymcuprog`, `python3.11`, Lattice Propel + Radiant installed, USB-UART cable, parts from `verified-bom.md` in hand, 3D-printed half-cube enclosures from `cad/beacon-half-cube.stl`.

---

## (a) Build one beacon pod (P2 deliverable)

Time budget: ~2 hours including soldering.

1. **Print the half-cube enclosure**: from `cad/beacon-half-cube.stl` on any FDM printer (PLA, ~30% infill, no support, 0.2 mm layer). Verify the 5 LED indents are clear; the inboard battery cavity slides in a fresh 1S 100 mAh pack with friction-fit retention.
2. **Cut a 25 × 25 mm perfboard** to fit the cube's interior PCB shelf. Pre-drill a 1 mm hole at each LED indent's interior wall for LED-lead routing.
3. **Solder the boost driver subassembly**:
   - LM3410-Y on SOT-23-to-DIP adapter at the cube's outboard end.
   - 22 µH inductor + MBR130 Schottky + 4.7 µF output cap in the standard boost topology around the IC.
   - 0.62 Ω sense resistor between LED-string return and the LM3410-Y FB pin.
4. **Solder the MCU subassembly**:
   - ATtiny412 on adapter (or use a Curiosity Nano dev module for first build with flying leads).
   - 1 µF + 100 nF V_BAT decoupling.
   - Wire the DIM-out GPIO directly to the LM3410-Y DIM pin (no resistor needed — DIM is high-impedance input).
   - Wire 2× code-select GPIOs to jumper pads (solder a bridge for "0" bits; leave open for "1" bits).
5. **Solder the supervisor + UVLO chain**:
   - Voltage supervisor IC (MCP1316T or equivalent) on adapter.
   - Open-drain output to the LM3410-Y EN pin via a 10 kΩ pull-up to V_BAT.
   - Result: supervisor pulls EN LOW at 3.3 V trip; releases (open-drain) above 3.4 V.
6. **Solder the 5 LEDs**: Lumileds L1IZ at each indent, leads routed back through the pre-drilled holes to PCB pads. Wire in series: apex → side1 → side2 → side3 → side4 → sense-resistor return. Use 32-AWG solid magnet wire for compactness.
7. **Solder the diagnostic LED + JST-PH socket** on the inboard face. Diagnostic LED is parallel to the MCU's DIM-out GPIO (same edge timing → blink at 100 Hz when emitting).
8. **Flash the firmware**: `cd firmware/beacon-pod && make flash CODE_ID=0` (pod A) or `CODE_ID=1` (pod B). UPDI cable connects to the ATtiny412's UPDI pin during programming only.
9. **Bench verify per FR-1.5**:
   - Insert a charged 1S battery.
   - Scope-probe the LED-string current sense resistor: expect 100 Hz chip rate, 15-chip code period (150 ms), 7-8 ON chips out of 15, 0/300 mA clean transitions.
   - Visually confirm diagnostic LED blinks at chip rate.
   - Probe the LM3410-Y output rail: expect ~9.5 V stable when DIM is HIGH, drops to ~0 V (output cap discharge) when DIM is LOW.
   - Capture 200 ms of DIM waveform → `firmware/beacon-pod/tests/scope-trace-decode.py` → expect `PASS: code 0` (or whatever was flashed).
10. **Pull the battery**: confirm all LED emission stops within ≤50 ms (FR-1.7 #2).

Pod is ready for paired-pod orthogonality verification (FR-3.2) once the second pod is built.

---

## (b) Build one recorder system (P3 deliverable)

Time budget: ~4 hours.

1. **Receive the Lattice CrossLink-NX-EVN** (LIFCL-40-9BG400C). Verify per the board user guide:
   - Onboard HyperRAM present + capacity ≥8 MB (R3 exec-research item).
   - microSD slot routed to 4-bit SDIO.
   - USB-C powers the board at 5 V.
2. **Mount the camera module**:
   - Bench bring-up: Arducam B0162 (OV9281) via the 22-pin MIPI flex shipped in the EVN kit.
   - **Check the Arducam B0162 bundled lens for an IR-cut filter** (visual: cyan/blue tint; functional: point an IR remote, see if pixels light up). If IR-cut present → strip the filter ring with a small screwdriver OR swap with the m12lenses.com PT-02120 (no IR-cut).
   - Install the 850 nm bandpass filter (Edmund #65-679 or Thorlabs FB850-10) between lens and sensor, sandwiched in the M12 holder.
3. **Wire power input**:
   - Pololu D24V10F5 buck regulator: input 7–36 V (3S LiPo OK), output 5 V / 1 A → eval board USB-C (or barrel jack if board user guide allows direct 9–12 V).
   - Splice into the carrier craft's 3S LiPo XT60 (or whatever the carrier provides).
4. **Build the Propel project**:
   - `cd firmware/flight-recorder/propel-project && propelbuilder build .`
   - Flash the bitstream to the EVN board's onboard flash via JTAG/USB.
5. **Bench bring-up** (NOT on the carrier yet):
   - Power the EVN board from a benchtop 5 V supply (USB-C).
   - Insert a SanDisk Extreme Pro V30 (or V60 for 480 fps modes) microSD.
   - Watch the status LED sequence: RED solid → GREEN blinking → GREEN heartbeat blink (~1–2 Hz).
   - Aim the camera at a Lumileds L1IZ-equipped beacon pod ~1 m away.
   - Wait 30 seconds.
   - Power down the board cleanly (USB unplug).
   - Pull the SD card → mount on the PC → confirm `<session-id>.clip` + `<session-id>.json` are present + non-empty.
6. **Ingest the test clip**:
   - `cd tools/beacon-loader && pip install -e .`
   - `python -c "from beacon_loader import load_clip; f, m = load_clip('/path/to/session.clip'); print(f.shape, m['recovered_frame_count'])"`
   - Expect output like `(7200, 240, 320) 7200` for a 30 s @ 240 fps clip.
7. **Mount the recorder on the carrier craft**:
   - 3D-print or hand-cut the camera-front mount; tape to whatever surface gives a forward-facing view.
   - **Double-back tape (or velcro) the EVN board** to the carrier's deck / canopy / payload tray — wherever there's flat space and airflow.
   - Run the MIPI flex between camera and board (keep it short, <10 cm if possible).
   - Verify status LED is visible from outside the carrier (light-pipe or exposed mount).
   - Per FR-2.6: GREEN heartbeat blink (~1–2 Hz) before throttle-up is the operator's go signal.

Recorder is now ready for bench scenario sweeps (S1–S9) and US6.

---

## (c) Record S1 + ingest in Python (P3 first-pass deliverable)

Time budget: 20 min.

1. Power up a beacon pod (insert 1S LiPo). Confirm diagnostic LED blinking at chip rate.
2. Power up the recorder (USB-C or carrier-craft power). Wait for status LED → GREEN heartbeat blink (~1–2 Hz).
3. Aim camera at pod at **1 m, indoor, dim ambient** (scenario S1 per FR-5.1).
4. Record for 30 seconds.
5. Power down both (pull battery on pod; USB unplug on recorder).
6. Pull SD card.
7. On the PC:
   ```bash
   cd tools/beacon-loader
   python - <<'PY'
   from beacon_loader import load_clip
   import numpy as np
   frames, metadata = load_clip('/path/to/sd-card/<session-id>.clip')
   print(f"Frames: {frames.shape}")
   print(f"Mean pixel value: {frames.mean():.1f}")
   print(f"Max pixel value: {frames.max()}")
   print(f"Bit depth: {metadata['capture_mode']['bit_depth']}")
   print(f"Recording mode: {metadata['recording_mode']}")
   # Quick: look for the brightest pixel per frame — should be the beacon blob
   brightest = frames.reshape(frames.shape[0], -1).argmax(axis=1)
   ys, xs = np.unravel_index(brightest, (240, 320))
   print(f"Brightest-pixel trajectory: x range {xs.min()}-{xs.max()}, y range {ys.min()}-{ys.max()}")
   PY
   ```
8. Expect: bright pixel cluster near the center, tight x/y range (pod is stationary). This is your end-to-end "the chain works" smoke check.
9. Open `bench-logs/<date>-S1.md`. Fill in: clip ID, observed beacon visibility, blob shape, frame drops (loader reports `fully_intact: True/False`). Per FR-5.2.

If steps 7-8 yield numpy arrays + non-zero brightness centered in the frame, the optical chain is alive. Move on to S2-S9.

---

## Where to go from here

- **P2 bench gate**: FR-3.3 EMC + FR-3.4 eye-safety + FR-1.7 UVLO bench-verifies. Document in `eye-safety-measurements.md`.
- **P3 bench gate**: complete S1-S9 scenario sweeps; one bench log entry per scenario.
- **P4 US6**: paired-craft test flight — see [spec.md §US6](./spec.md) for the operational model + acceptance criteria.
- **P5 close-out**: write `SMOKE_REPORT.md` + `outcome.md`; tag the EVN gateware bitstream as the 031-fpga ingest input.
