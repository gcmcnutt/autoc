# Quickstart — 031 Beacon-Camera Phase 1

> **MOVED 2026-06-22 from `specs/031-beacon-camera/`. Parked 040 camera-redo reference — predates the 031 1-bit phase and is likely stale; re-validate (20 Hz / 480 fps / 200 Hz / 75 ms baseline + 031 field findings) on 040 restart. See [`README.md`](README.md).**

Four end-to-end walk-throughs. None require a printed PCB. **Section (0) is a recommended pre-flight** that validates the boost-converter and MCU gold-code generation on a breadboard before committing parts and time to the cube-mounted perfboard build in §(a).

> Prereqs assumed: Linux PC, `avr-gcc`, `pymcuprog`, `python3.11`, Lattice Propel + Radiant installed, USB-UART cable, parts from `verified-bom.md` in hand, 3D-printed half-cube enclosures from `cad/beacon-half-cube.stl`.

---

## (0) Phase 1a eval-board pre-flight (recommended before §(a))

Time budget: ~3 hours including breadboard wiring, the one-time XNANO mod, and scope setup. The eval substitutes the ATtiny412 + UPDI + 1S LiPo sub-assembly with a Microchip **ATtiny416-XNANO** evaluation kit (mEDBG-over-USB programmer + USB-CDC virtual COM + edge-connector breakout — all on one board) **while keeping the same TPS3839 supervisor** as the target circuit. The XNANO's USB-5V is isolated from the target rail via a one-time cut of an internal 0Ω strap (R100), so we can ramp the bench supply down to demonstrate the low-voltage cutout without losing mEDBG debug.

Validates three orthogonal properties before committing to perfboard layout:

1. **Firmware-side gold-code generation** on real silicon — see "firmware portability" below.
2. **Boost-converter regulation** — constant-current loop regulates the 5-LED string at 306 mA via the 0.62 Ω sense resistor; V_LED auto-regulates to ~10 V.
3. **Low-voltage cutout** — slowly ramp VIN down past ~3.0 V, U3 (TPS3839) trips, DIM goes LOW, LEDs go dark. Restore voltage → clean restart.

**Schematic + BOM live in [`cad/beacon-eval/`](../../cad/beacon-eval/)** (separate from the cube pod):

- [`cad/beacon-eval/beacon-eval-schematic.pdf`](../../cad/beacon-eval/beacon-eval-schematic.pdf) — printable schematic
- [`cad/beacon-eval/beacon-eval.kicad_sch`](../../cad/beacon-eval/beacon-eval.kicad_sch) — KiCad 10 source
- [`cad/beacon-eval/verified-bom-eval.md`](../../cad/beacon-eval/verified-bom-eval.md) — eval-only BOM. Most lines reuse the parent target BOM (LM3410X, L1, D1, R1, R2, C1, C5, C6, **TPS3839 supervisor**, 5× L1IZ-0850); additions are the XNANO eval kit (DigiKey `ATTINY416-XNANO-ND`, ~$11), one extra 100 nF cap (C7 supervisor decoupling), and a few rig consumables.

### Firmware portability — ATtiny412 ↔ ATtiny416, same source under PlatformIO

The eval restricts pin assignments to **only the GPIOs common to both chips: `PA0`, `PA1`, `PA2`, `PA3`, `PA6`, `PA7`**. Pins PA4/PA5 and all PB*/PC* on the ATtiny416 are off-limits — they don't exist on the ATtiny412. This means the same `.c` source compiles for both targets with no `#ifdef` pin remapping; just rebuild with a different `[env:...]` in `platformio.ini`:

```ini
[env:beacon-eval]
platform = atmelmegaavr
board = ATtiny416_xnano       ; megaTinyCore
upload_protocol = xplainedmini_updi   ; XNANO mEDBG over USB

[env:beacon-target]
platform = atmelmegaavr
board = ATtiny412              ; production hardware
upload_protocol = serialupdi   ; or jtag2updi
```

Critical pin: **PA3 = DIM control** (open-drain GPIO that pulls DIM low to disable the boost). Same pin on both chips, same firmware bit, same JTAG2UPDI flash. The eval cannot lie about firmware behavior.

### Bring-up summary (full sequence in the BOM doc)

1. **One-time XNANO mod**: cut R100 (0Ω strap) inside the XNANO board to isolate VTG from USB-5V. Verify continuity is broken.
2. **Build the analog sub-assembly** on perfboard / SOT-23-to-DIP adapters: U1 (LM3410X) + L1 (22 µH) + D1 (MBR130) + C1 (4.7 µF) + R1 (0.62 Ω) + R2 (10 k) + C5 (2.2 µF) + C6 (100 nF), with the boost switching loop physically tight. **U3 (TPS3839) + C7 (100 nF)** mount adjacent, with U3.RESET wired into DIM alongside R2.bot and U1.DIM (wired-AND topology, matches target).
3. **Hand-reflow 5× L1IZ-0850** in series on a small carrier PCB (OSH Park ~$5/3 boards), or stripboard fallback.
4. **Wire to breadboard**: J2 ← bench 5 V/3 A; J3 ↔ LED carrier; XNANO J200.1 (VTG) ↔ VIN_5V rail; XNANO J200.5 (PA3) ↔ DIM net; XNANO J200.20 (GND) ↔ common ground.
5. **Power-on smoke test** (no firmware): R2 pull-up holds DIM HIGH, U3 releases (VIN > 3.0 V), boost runs. V_LED ramps to ~10 V, LEDs glow IR (visible on phone camera), V across R1 = 190 mV ± 5%.
6. **Low-V cutout test**: ramp bench supply VIN down. At ~3.0 V, U3 trip → DIM LOW → LEDs dark. Restore VIN → clean restart. mEDBG over USB stays alive throughout (independent power domain).
7. **Flash gold-code firmware to ATtiny416** via XNANO mEDBG: `pio run -e beacon-eval -t upload`.
8. **Verify on scope**: DIM signal at U1.4 shows 4-code PN sequence at chip rate; current envelope across R1 tracks DIM with spec'd transitions.
9. **Rebuild for target**: `pio run -e beacon-target` produces the same binary (just different fuses) — proceed to §(a).

**Out of scope for §(0)** (defer to §(a)): battery + charge protection, cube enclosure mechanical assembly, perfboard EMI layout / SW-node loop area, eye-safety photometric polar plot, paired-pod orthogonality verification.

---

## (a) Build one beacon pod (P2 deliverable)

Time budget: ~2 hours including soldering.

1. **Print the half-cube enclosure**: from `cad/beacon-half-cube.stl` on any FDM printer (PLA, ~30% infill, no support, 0.2 mm layer). Verify the 5 LED indents are clear; the inboard battery cavity slides in a fresh 1S 100 mAh pack with friction-fit retention.
2. **Cut a 25 × 25 mm perfboard** to fit the cube's interior PCB shelf. Pre-drill a 1 mm hole at each LED indent's interior wall for LED-lead routing.
3. **Solder the boost driver subassembly**:
   - **LM3410X** (NOT LM3410Y — datasheet has them swapped vs the prior spec) on SOT-23-to-DIP adapter at the cube's outboard end.
   - 22 µH inductor + MBR130 Schottky + 4.7 µF output cap in the standard boost topology around the IC.
   - 0.62 Ω sense resistor between LED-string return and the LM3410X FB pin.
4. **Solder the MCU subassembly**:
   - ATtiny412 on adapter (or use a Curiosity Nano dev module for first build with flying leads).
   - 1 µF + 100 nF V_BAT decoupling.
   - Wire the DIM-out GPIO to the LM3410X DIM pin. **Configure the MCU GPIO as open-drain in firmware** so it joins the wired-AND topology on DIM (chip=1 releases to high-Z; chip=0 drives LOW).
   - Wire 2× code-select GPIOs to jumper pads (solder a bridge for "0" bits; leave open for "1" bits).
5. **Solder the supervisor + UVLO chain** (DIM-wired-AND topology — corrected 2026-05-18 per LM3410 datasheet — LM3410 has NO separate EN pin, DIM is the only shutdown control):
   - Voltage supervisor IC (MCP1316T or equivalent, **open-drain output required**) on adapter.
   - Connect supervisor open-drain output to the **LM3410X DIM pin** (same node as the MCU GPIO).
   - Add a **10 kΩ pull-up resistor from DIM to V_BAT**.
   - Result: when V_BAT > 3.3 V the supervisor releases (high-Z), pull-up holds DIM HIGH unless the MCU's open-drain GPIO pulls it LOW per the code LUT. When V_BAT ≤ 3.3 V the supervisor pulls DIM LOW, forcing the LM3410X into ~80 nA shutdown regardless of MCU state.
6. **Solder the 5 LEDs**: Lumileds L1IZ at each indent, leads routed back through the pre-drilled holes to PCB pads. Wire in series: apex → side1 → side2 → side3 → side4 → sense-resistor return. Use 32-AWG solid magnet wire for compactness.
7. **Solder the diagnostic LED + JST-PH socket** on the inboard face. Diagnostic LED is parallel to the MCU's DIM-out GPIO (same edge timing → blink at 100 Hz when emitting).
8. **Flash the firmware**: `cd firmware/beacon-pod && make flash CODE_ID=0` (pod A) or `CODE_ID=1` (pod B). UPDI cable connects to the ATtiny412's UPDI pin during programming only.
9. **Bench verify per FR-1.5**:
   - Insert a charged 1S battery.
   - Scope-probe the LED-string current sense resistor: expect 100 Hz chip rate, 15-chip code period (150 ms), 7-8 ON chips out of 15, 0/300 mA clean transitions.
   - Visually confirm diagnostic LED blinks at chip rate.
   - Probe the LM3410X output rail: expect ~9.5 V stable when DIM is HIGH, drops to ~0 V (output cap discharge) when DIM is LOW.
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
   - `cd specs/040-camera-redo/beacon-loader && pip install -e .`
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
   cd specs/040-camera-redo/beacon-loader
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
- **P4 US6**: paired-craft test flight — see [spec.md §US6](../031-beacon-camera/spec.md) for the operational model + acceptance criteria.
- **P5 close-out**: write `SMOKE_REPORT.md` + `outcome.md`; tag the EVN gateware bitstream as the 031-fpga ingest input.
