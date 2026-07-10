# Phase 0 Research — 031 Beacon-Camera

**Status**: this doc captures *current best-knowledge* resolutions for each Plan Technical-Context NEEDS-CLARIFICATION item, plus the *exact* execution-time research that must precede each P1 task. Some items resolve to a single recommended option here; others are marked **NEEDS-EXEC-RESEARCH** because the answer requires a WebFetch / datasheet pull / vendor inquiry that should happen at P1 execution, not at plan time.

---

## R1 — CAD MCP Server

**Decision**: **FreeCAD MCP** (operator-confirmed 2026-05-17).

**Rationale**:
- FreeCAD has a mature parametric solid-modeling kernel (OpenCASCADE Technology) with clean STEP + STL export.
- Operator already uses FreeCAD in the workflow — no new CAD tool to learn.
- A FreeCAD MCP server exposes the FreeCAD Python API (sketch creation, parametric features, boolean ops, file export) over MCP, letting Claude Code drive enclosure design programmatically while the operator visually inspects via the FreeCAD GUI.
- Source-of-truth is the `.FCStd` project file (binary, but FreeCAD's Python history allows scripted reconstruction). STEP and STL are exported artifacts.

**Output convention**:
- `cad/source/beacon-half-cube.FCStd` — editable FreeCAD project, single source of truth
- `cad/beacon-half-cube.step` — exported STEP for downstream PCB-mount / mechanical design
- `cad/beacon-half-cube.stl` — exported STL for FDM printing

**Alternatives considered + rejected**:
- **OpenSCAD MCP**: code-driven authorship is LLM-friendly, but operator preference for FreeCAD wins; FreeCAD's STEP export is cleaner for downstream tooling.
- **CADQuery / build123d MCP**: Python-API CAD on the same OCCT kernel — interesting alternative, but FreeCAD's GUI for visual inspection is the deciding factor.
- **Hand-authoring `.FCStd` files** (skip the MCP): rejected — `.FCStd` is a binary archive, not LLM-editable directly; MCP integration is the only practical way to drive it from Claude Code.

**NEEDS-EXEC-RESEARCH at P1**: confirm the specific FreeCAD MCP package being used (e.g., `freecad-mcp` on npm/PyPI, or a custom local server). The plan assumes one of:
- `mcp-server-freecad` (community) — wraps FreeCAD's Python console
- Custom local MCP server using the FreeCAD Python API directly

Identify the tool names + argument schema once the MCP is connected so Phase 1 enclosure work can begin.

---

## R2 — ATtiny412 Toolchain

**Decision**: **avr-gcc + serialUPDI + pymcuprog** (DIY path, open-source toolchain).

**Rationale**:
- avr-gcc is the standard open-source C compiler for AVR; supports ATtiny412 via the modern `atmega_attiny`-family device packs.
- UPDI (Unified Program/Debug Interface) is Microchip's 1-wire serial programming protocol; flash-able from any USB-UART adapter with a 4.7 kΩ pull-up resistor (the "serialUPDI" trick) — no dedicated PICkit required.
- `pymcuprog` (Microchip's Python utility) is the host-side flash tool; cross-platform, MIT-licensed.
- Total tooling cost: ~$5 for a USB-UART + a resistor. Already in operator's parts bin.

**Alternatives considered**:
- **Microchip MPLAB X + PICkit 4**: commercial / proprietary IDE, but the most "supported" path. Rejected because (a) MPLAB X is heavyweight for a 4-file firmware project, (b) PICkit 4 is ~$80, (c) serialUPDI is well-documented and works.
- **Arduino IDE with megaTinyCore**: easier setup, but adds Arduino abstractions over what is a pure register-poke firmware. Rejected as over-abstracted for a 100 Hz timer ISR + GPIO write.
- **PlatformIO + megaTinyCore** (noted in `cad/beacon-eval/` commit 4321ec5 as a "build the same source for ATtiny416_xnano (eval) and ATtiny412 (target)" path): **POSSIBLE REVISIT** *(noted 2026-05-20)*. Attractive for the eval-vs-target portability story but the same Arduino-abstraction concern applies; also adds a PlatformIO config file + the megaTinyCore dependency. Defer the decision to P1a firmware bring-up — if avr-gcc + serialUPDI proves clunky for the dual-target build, switch to PlatformIO then. Either way the same source code (per the PA0/1/2/3/6/7 portability discipline) compiles.

**Build artifacts**: `firmware/beacon-pod/Makefile` invokes `avr-gcc -mmcu=attiny412` and produces a `.hex`; `make flash` invokes `pymcuprog write -t uart -d attiny412 -u /dev/ttyUSB0 -f beacon-pod.hex`.

---

## R3 — Lattice CrossLink-NX-EVN Onboard Memory

**NEEDS-EXEC-RESEARCH at P1**: pull the LIFCL-40-EVN board user guide + schematic from [https://www.latticesemi.com](https://www.latticesemi.com); confirm:

1. Onboard volatile memory part number (likely **HyperRAM** or **LPDDR2/3 SDRAM** — most LIFCL-40 eval boards in this class ship with 64-128 Mbit HyperRAM = 8-16 MB).
2. DMA fabric path from MIPI-D-PHY-input to that memory.
3. Read-side bandwidth from memory to SDIO (≥46.1 MB/s required for the 480 fps 10-bit-packed worst case).

**Current best-knowledge**: the CrossLink-NX-EVN ("Voyager") board ships with **HyperRAM** (~8-16 MB), which is sufficient for a 6 MB ring buffer with comfortable margin. The HyperRAM interface to LIFCL-40 fabric is well-trodden in Lattice's reference designs (Propel's "video pipeline" examples use HyperRAM as the line buffer).

**Decision (conditional)**: if exec-time research confirms ≥8 MB HyperRAM with DMA fabric path: use it for the ring buffer. If memory is smaller, the ring-buffer size in FR-2.5 needs revisiting (could fall back to 240 fps 8-bit only, which fits in <2 MB).

**Risk**: if the EVN board lacks sufficient onboard memory, P3 stalls. Mitigation: order an EVN board with a piggyback HyperRAM expansion (Lattice sells these), or fall back to a different eval board with confirmed memory (e.g., the LIFCL-40 + Lattice Pmod HyperRAM expansion).

---

## R4 — OG0VA Sourcing Reality

**NEEDS-EXEC-RESEARCH at P1**: place inquiry with OmniVision OEM channel for OG0VA CameraCubeChip module availability + lead time + MOQ. Parallel inquiry to design-house resellers (Leopard Imaging, e-con Systems, similar) for stocked OG0VA modules.

**Current best-knowledge**: OG0VA is a relatively new OmniVision part (released 2023 timeframe), aimed at the drone / AR-glasses market. OEM channel typically requires direct sales contact (no DigiKey stock); lead time 6-12 weeks; MOQ may be in the hundreds for design-house resellers.

**Decision**: **ship the recorder build with OV9281 via Arducam B0162 as the primary first-flight path**, with OG0VA as the **upgrade target** for later P3/P4 iterations if it lands. The Arducam B0162 is in DigiKey/Mouser stock at ~$30-50; same 60% QE @ 850 nm; OV9281 has been used in similar IR-pulsed-LED beacon projects (drone follow-me applications). The bench-bring-up + bench-scenario-sweep can proceed on OV9281 without waiting for OG0VA.

**Rationale**: don't gate P2/P3 on OEM-channel lead time. Field results in P4 (US6) are more important than which-specific-sensor; the optical chain is sensor-agnostic at the same QE class.

**Alternatives considered**: ST VD55G1 (dropped per spec 2026-05-14 — less NIR ecosystem); waiting for OG0VA (rejected — schedule risk).

---

## R5 — Hand-Prototype Breakouts

**NEEDS-EXEC-RESEARCH at P1**: identify currently-stocked breakouts (Adafruit / SparkFun / Pololu / DigiKey searches) for each BOM line. The list below is the current-best-knowledge starting point.

| BOM line | Hand-prototype candidate | Notes |
|---|---|---|
| **TI LM3410-X boost LED driver** | **Adafruit-style boost driver breakout** (LM3410 or similar) — likely not stocked as a named breakout; **fallback**: solder the LM3410-X bare SOT-23-5 to a SOT-23-to-DIP adapter + dead-bug build on perfboard. **Alt**: **Pololu / SparkFun "constant-current LED driver" boards** (PT4115, AL8807, etc.) — if a substitute IC is acceptable for hand-prototype, use it. | The LM3410-X is not a "common breakout" part; bare-SOT-23 + adapter is the most likely path. |
| **ATtiny412 MCU** | **Adafruit ATtiny breakout** (if stocked) OR **Microchip ATtiny412 Curiosity Nano** (~$10, includes UPDI programmer + USB) | The Curiosity Nano is the recommended bring-up board — has the MCU + UPDI + USB in one. Solder a flying-wire harness for DIM + code-select. |
| **Voltage supervisor IC (MCP1316T)** | Bare-SOT-23-5 on adapter; dead-bug on perfboard | Same pattern as LM3410. |
| **22 µH shielded inductor** | Through-hole equivalent (Coilcraft DR0810 or similar through-hole 22 µH) for prototype; SMT for final | Through-hole simplifies hand-wiring. |
| **MBR130 Schottky** | Through-hole DO-41 equivalent (1N5817 is the through-hole standard 1A Schottky) | Direct substitute. |
| **0.62 Ω sense resistor** | Through-hole 1206 SMT on adapter, OR a parallel pair of 1.2 Ω through-hole | Power dissipation < 60 mW — easy in any package. |
| **JST-PH 2.0 mm socket** | DigiKey stocked through-hole socket | Through-hole preferred for solder strength on hand-wired build. |
| **1S 100 mAh LiPo** | Amazon B083NWXLTK or DigiKey-stocked Tinywhoop pack | Direct part as spec'd. |
| **Code-select** | 2× wire jumpers to GND/VCC for assembly-time selection | Skip the DIP switch in the hand-prototype; just hardwire 2 bits. |
| **0603 diagnostic LED + 1 kΩ series resistor** | Through-hole 3 mm green LED + 1 kΩ resistor for prototype | Easier to wire; same function. |
| **Lumileds L1IZ-0850000000000** | Same part — order via DigiKey 7243418 | No hand-prototype substitute; this is the optical heart. |

**Decision**: build the first pod on a **25 × 25 mm 0.1" pitch perfboard** (cut to fit inside the half-cube) using bare ICs on SOT-23-to-DIP adapters + through-hole passives where possible. The Curiosity Nano is used for *programming + breadboard bring-up only*; the production pod has a bare ATtiny412 on the perfboard. Hand-wire the 5 LEDs in series with short jumper wires routed to each cube face's indent.

**Rationale**: avoids PCB fab lead time + cost; lets the operator iterate on dimming / EMI / battery-life behavior with parts in hand. Trade-off: hand-built prototype has higher EMI signature than a designed PCB (no controlled-impedance traces, no ground-pour); FR-3.3 gate catches it.

**Output**: `hand-prototype-guide.md` codifies the parts list + wiring + step-by-step.

---

## R6 — Lens Sourcing

**NEEDS-EXEC-RESEARCH at P1**: contact `contact@commonlands.com` for custom-lens quote + lead time. Place inquiry as a P1 first-week task because lead time (~4-6 weeks) gates US6.

**Prototype-path lens** (for bench bring-up BEFORE Commonlands lands):
- **m12lenses.com PT-02120** ($30, days lead) — generic 145° fisheye M12. **Critical first-receiver action**: visually inspect for IR-cut filter (cyan/blue tint in reflection); functional test with an IR remote control; if IR-cut present, strip the filter or swap the lens.
- **Edmund Optics #65-679** (10 mm dia, 850 nm CWL, 10 nm FWHM, ~$90) OR **Thorlabs FB850-10** (1" dia, ~$70) — discrete bandpass filter, placed between lens and sensor.

**Decision**: order both paths in parallel at P1. Bench bring-up uses the prototype lens + discrete filter while waiting for Commonlands. The discrete filter (10 nm FWHM) actually has *better* daylight rejection than the integrated 30 nm Commonlands option — the prototype path may end up being the production-recorder path if Commonlands lead time is unacceptable.

**Alternative considered**: ship only the Commonlands path — rejected because of lead-time gating.

---

## R7 — NFR-4 Clock-Drift Simulation Tooling

**Decision**: **plain `numpy` Python script** in `specs/031-beacon-camera/nfr4-clockdrift-sim/`.

**Rationale**: the simulation is small — a few hundred LOC. Generates a Gold-code template, applies chip-rate offset, generates a synthetic received signal at simulated SNR, runs matched-filter correlation across hypothesis offsets, computes acquisition probability via Monte Carlo. No need for GNU Radio or Matlab; numpy + scipy.signal cover everything.

**Output**: `specs/031-beacon-camera/nfr4-clockdrift-sim/sim.py` + a `simulation-results.md` doc with the decision (crystal vs multi-hypothesis vs internal-RC) recorded as a Decisions-Locked spec update.

**Alternatives considered**:
- **GNU Radio**: overkill; the signal-processing graph is too simple to warrant the GNU Radio framework overhead.
- **Matlab**: requires Matlab license; numpy is free + reproducible across hardware.

---

## R8 — 1S LiPo Specific Part

**NEEDS-EXEC-RESEARCH at P1**: confirm **Amazon B083NWXLTK** is still available; if not, identify a DigiKey-stocked Tinywhoop-class 1S 100 mAh 20C pack (e.g., search "1S 100mAh JST-PH 2.0").

**Decision (current)**: **Tinywhoop-class 1S 100 mAh 20C pouch** with JST-PH 2.0 mm pigtail. Specific stocked candidates:
- **GNB / BetaFPV / EMAX 1S 250 mAh** packs (slightly larger; ~3 g vs 2 g; provides more runtime margin — could be a better choice if 6 mm is fine and 2 g extra is acceptable).
- **Amazon "Tinywhoop battery 1S 100mAh"** — multiple vendors, generic packs, ~$2-3/pack.

**Decision (firm)**: order **3-4 packs minimum** to have spares for cycling during bench work + flight.

---

## R9 — Lattice Toolchain (Radiant vs Propel)

**Decision**: **Lattice Propel for Phase 1** (camera-config + SD-record); transition to Radiant-only if Propel proves too coarse-grained for the 031-fpga DSP pipeline later.

**Rationale**:
- **Propel** is Lattice's RISC-V soft-CPU + reference-design framework. Phase 1 needs: I²C camera config (a software-level task) + MIPI ingest (gateware) + SDRAM ring (gateware) + SDIO writer (gateware with a software-driver layer). Propel ships a reference design that has SDIO drivers + a basic camera-ingest example; integration time is hours to days vs weeks for from-scratch Radiant HDL.
- **Radiant** (HDL-only) is the lower-level option; gives more control but no soft-CPU support. 031-fpga (Phase 2) DSP pipeline is gateware-only and may want Radiant — but Phase 1 doesn't need that level.

**Output**: `firmware/flight-recorder/propel-project/` is the Phase-1 project root.

**NEEDS-EXEC-RESEARCH at P1**: confirm Propel reference designs for camera ingest + SDIO exist and target the LIFCL-40 (vs CertusPro-NX or other Lattice parts).

**Alternative considered**: Radiant-only — rejected for Phase 1 because the SDIO driver effort is non-trivial; Propel's pre-built SDIO driver saves 1-2 weeks.

---

## R10 — FreeCAD MCP Invocation Pattern from Claude Code

**NEEDS-EXEC-RESEARCH at P1**: confirm with operator the specific FreeCAD MCP server's tool names + argument schema once it's connected.

**Expected pattern** (FreeCAD MCP per R1):

```
1. Claude Code invokes the FreeCAD MCP — likely tool flavors:
   - create_document / open_document
   - add_box / add_cylinder / add_sketch (parametric primitives)
   - boolean_cut / boolean_fuse / boolean_common
   - export_step / export_stl
   - save_document

2. For the beacon half-cube enclosure, the script flow is:
   a. open or create cad/source/beacon-half-cube.FCStd
   b. create a 25 × 25 × 13 mm half-cube (Part.Box minus open inboard face)
   c. add 5 × LED indent pockets (per face normal) — cylindrical Pad/Pocket features
   d. add the inboard battery cavity (rectangular pocket, ~4 × 22 × 6 mm)
   e. add the inboard slide-in opening (rectangular cut on the inboard face)
   f. add the diagnostic-LED light-pipe slot (small cylindrical hole, inboard face)
   g. add code-select access slot (rectangular cut, inboard face)
   h. save_document; export_step; export_stl

3. Operator visually inspects in the FreeCAD GUI (already running locally as the
   MCP backend); provides geometric feedback that Claude Code translates into
   parameter changes on the next iteration.

4. Iterate until the enclosure prints cleanly + fits the BOM.
```

**Fallback** if FreeCAD MCP turns out to be non-functional at P1: hand-author a Python script that drives the FreeCAD CLI (`freecadcmd script.py`), which exercises the same FreeCAD Python API. Same `.FCStd` + STEP + STL artifacts, slower iteration loop (no live GUI feedback during scripted edits).

**Decision**: drive the enclosure design via the FreeCAD MCP whenever it's available; fall back to `freecadcmd` scripted invocation otherwise. **Artifacts are identical either way; only the iteration loop UX differs.**

---

## R11 — Undervoltage Cutoff + LED-Driver Failsafe (1S LiPo brown-out protection)

**Decision**: **Topological failsafe + firmware ADC undervoltage cutoff + watchdog timer.** Three layers, no supervisor IC, no divider resistors. Specifically:

1. **Schematic change** — invert `R2` (10 kΩ on DIM net) from pull-UP to V_BAT to **pull-DOWN to GND**. Any MCU-offline state (reset, boot, brown-out, hang-in-input-mode) leaves DIM low → LM3410X in 80 nA shutdown → **LEDs OFF**.
2. **Firmware ADC undervoltage cutoff** — measure V_BAT via the ATtiny412's internal 1.1 V bandgap reference (zero external parts), trip at 3.6 V firmware-set threshold (which corresponds to ~3.5 V real V_BAT after Vref drift + sag), drive PA3 LOW, halt the MCU.
3. **Watchdog timer** — ~250 ms timeout, petted in the main loop. If firmware hangs (including the rare "hangs while driving PA3 HIGH" case), WDT resets the MCU within 250 ms; PA3 returns to high-Z; the inverted topology pulls DIM low; LEDs OFF.

**Context**: The beacon runs directly off a 1S LiPo (3.0 – 4.2 V) with no LDO. The LM3410X boost LED driver is a constant-current regulator that will keep pulling the same ~306 mA from the battery as VIN sags — eventually causing a battery undervoltage event that can damage the cell (LiPo cycle-life degrades sharply below ~3.0 V) and may brown out the MCU. Need a deterministic cutoff that kills the LED driver before the battery is over-discharged. Operator-locked threshold: **≥ 3.5 V** (rest-voltage 30%-SOC reserve; sag-corrected trip is ~3.4 V under load).

**Why this beats a dedicated supervisor IC** (per the FR-1.7 #4 spec-revision discussion 2026-05-20):

| Aspect | Dedicated supervisor (original FR-1.7 #4) | This (topology + ADC + WDT) |
|---|---|---|
| BOM | +1 IC + decoupling cap + (often) divider resistors | **0 parts** (R2 already in the BOM, just routed differently) |
| Threshold flexibility | Fixed-suffix variants (TPS3839 caps at 3.08 V then jumps to 4.38 V — no 3.5 V part exists in the family) | **Firmware-set, exact**, tunable without re-spinning hardware |
| Cost | $0.30 – $0.80 + footprint area on the cube perfboard | $0 |
| MCU-offline coverage | Hardware-only (the supervisor is independent of MCU) | **Topological** (the schematic itself is the failsafe; covers reset / boot / brown-out / hang-in-input-mode) |
| Hang-while-driving-HIGH coverage | Hardware (supervisor still trips) | **WDT-bounded** (250 ms timeout → MCU reset → PA3 high-Z → topology takes over) |
| Defense in depth | One layer (supervisor) | **Three layers** (topology + ADC threshold + WDT) |

**Schematic change detail — the inverted DIM topology**:

Original (per current schematic + spec FR-1.2.1 BOM table):
```
V_BAT ──┬── R2 (10 kΩ) ──┬── DIM (LM3410X pin 4)
        │                │
        │                └── MCU PA3 (open-drain emulated: release for chip=1, LOW for chip=0)
        │
        └── LM3410X VIN
```
DIM default with PA3 high-Z = HIGH → LEDs ON. **Wrong polarity for failsafe.**

Revised:
```
DIM (LM3410X pin 4) ──┬── R2 (10 kΩ) ── GND
                      │
                      └── MCU PA3 (push-pull active-HIGH: HIGH for chip=1, LOW for chip=0)
```
DIM default with PA3 high-Z = LOW → LEDs OFF. **Correct failsafe polarity.** Single trace change. Same R2 value (10 kΩ stays — sets the DIM pull-down current at ~0.4 mA when PA3 drives HIGH, which is fine; LM3410X DIM input current is ~100 nA, negligible).

Firmware change is symmetric: PA3 configured as **push-pull output**, drive HIGH for chip=1, LOW for chip=0. No more open-drain emulation needed.

**MCU-offline truth table** (the failsafe guarantee):

| MCU state | PA3 hardware state | DIM (with new R2 pull-DOWN) | LM3410X | LEDs |
|---|---|---|---|---|
| Normal operation, chip=1 | drive HIGH | HIGH | running | ON |
| Normal operation, chip=0 | drive LOW (or release) | LOW | shutdown | OFF |
| Reset (any source: POR, BOD, WDT, software) | input/high-Z | LOW (pulled down) | shutdown | **OFF** ✓ |
| Boot (before firmware configures DDR) | input/high-Z | LOW (pulled down) | shutdown | **OFF** ✓ |
| Brown-out (V_BAT < BOD ~2.6 V) | held in reset → high-Z | LOW (pulled down) | shutdown | **OFF** ✓ |
| Firmware hang in input mode | input/high-Z | LOW (pulled down) | shutdown | **OFF** ✓ |
| Firmware hang while driving HIGH | held HIGH **briefly** | HIGH ⚠️ | running ⚠️ | ON ⚠️ |
|   ↳ after ≤ 250 ms WDT timeout: | reset → high-Z | LOW (pulled down) | shutdown | **OFF** ✓ |

The single non-self-protecting failure mode ("MCU hangs while driving PA3 HIGH") is **bounded to ≤ 250 ms by WDT** — well below the timescale at which a 1S LiPo discharges meaningfully.

**Implementation pattern** (firmware, ≈ 40 lines of C):

```
init:
    fuses: BODLEVEL = ~2.6 V (provides MCU reset before V_BAT damages cell)
    WDT.CTRLA = PERIOD_256CLK  // ~256 ms timeout
    PA3 DDR  = output, push-pull
    PA3 OUT  = LOW             // safe state until UV check passes
    ADC config: Vref = VDD, input channel = internal 1.1 V bandgap

low-V check (run once after boot, before enabling code modulation):
    sample ADC; vbat_mV = 1126400 / raw
    if vbat_mV < 3600: enter SLEEP_POWER_DOWN  // refuse to start

main loop:
    drive chip pattern on PA3 at 100 Hz
    WDT.STATUS = 0xA5   // pet the watchdog
    every 100 ms (timer ISR):
        sample ADC
        if vbat_mV < 3600 for 5 consecutive ticks (= 500 ms):
            PA3 = LOW                       // explicit shutdown
            sleep mode = POWER_DOWN         // ~100 nA hold, only battery removal wakes
```

**Trade-offs accepted**:

- **Internal 1.1 V reference factory accuracy: ±3 % @ 25 °C, ±4 % over temp.** At 3.5 V real threshold, worst-case measurement error is ±140 mV. **Mitigation**: firmware trip at **3.6 V** (not 3.5 V) so even worst-case Vref drift still leaves actual V_BAT ≥ ~3.46 V at cutoff. Optional per-unit cal pass: bench-measure offset → store in EEPROM.
- **Cutoff response time** ≈ 500 ms debounce (vs supervisor's µs). Acceptable — battery sag is a seconds-scale phenomenon, not µs.
- **Hang-while-driving-HIGH window**: ≤ 250 ms (WDT period) of "LEDs stay on past true cutoff threshold." At 306 mA worst-case continuous LED draw, that's ≤ 0.02 mAh per hang event ≈ ≤ 0.02 % of a 100 mAh pack. Negligible.

**BOM impact (target pod, `cad/beacon-pod/`)**:

- **REMOVE**: U3 supervisor (was MCP1316T-29HE / TPS3839K33 / APX803-31SAG candidates in the original BOM). Saves 1 SOT-23 footprint + sourcing line.
- **REMOVE**: supervisor decoupling cap (per spec FR-1.2.1 BOM table line "Supervisor decoupling 100 nF"). Saves 1 0603 footprint.
- **REWIRE only** (no part change): R2 (10 kΩ DIM pull-up → DIM pull-down). Same part, opposite endpoint. Update FR-1.2.1 BOM-table description from "to V_BAT" → "to GND".
- **No other parts change** — MCU PA3 still the sole driver of DIM (was already the sole non-supervisor driver), just push-pull active-HIGH instead of open-drain emulated.

**BOM impact (eval rig, `cad/beacon-eval/`)**: same — remove U3 (TPS3839L30DBVT line) + C7 + flip R2 routing. Eval rig should mirror the final target topology so the eval validates exactly what ships.

**Discovered errors in current `verified-bom-eval.md` that this revision moots**:

1. `TPS3839L30DBVT` — listed threshold "3.0 V" is wrong; per TPS3839 datasheet (SBVS193C) L30 = **2.63 V** not 3.0 V.
2. Package `DBVT` (SOT-23-5) doesn't exist for TPS3839 — only `DBZ` (SOT-23-3) or `DQN` (X2SON-4).
3. RESET output is push-pull, not open-drain — would bus-contend with MCU PA3 on the original wired-AND DIM topology.

All three errors disappear with U3 removed.

**Spec changes required** (queued for explicit operator sign-off before landing):

- **FR-1.7 #4** — rewrite from "mandatory hardware-level UVLO at 3.3 V, MCU-independent, supervisor IC" to "firmware ADC cutoff at 3.5 V real + topological failsafe via DIM pull-down + WDT-bounded hang protection." Draft in scratch under [R11.A] below.
- **FR-1.2.1 BOM table** — delete "Voltage supervisor" + "Supervisor decoupling" rows; update "DIM-line pull-up" row → "DIM-line pull-DOWN to GND"; update MCU row to remove "MCU does NOT participate in the UVLO cutoff path" sentence (it now is the cutoff path).
- **`spec.md` text-narrative §241-276 ASCII schematic** — redraw without the supervisor branch on DIM; document the new truth table.
- **§122 pre-flight checklist** — change "(c) hardware UVLO at 3.3 V confirmed per FR-1.7 #4" → "(c) firmware UVLO + WDT bench-verified per FR-1.7 #4."
- **§423 battery-runtime calculation** — adjust from "4.2 V → 3.3 V UVLO" → "4.2 V → 3.5 V UVLO" (~10 % capacity reserved instead of ~5 %; revised runtime ~12 min instead of 13 min).

**NEEDS-EXEC-RESEARCH at P1a (firmware bring-up)**:

- Confirm ATtiny412 ADC channel number for the internal 1.1 V reference (datasheet §30 — MUXPOS register encoding).
- Confirm exact BODLEVEL fuse value for ~2.6 V trip and that BOD operates in chosen sleep mode.
- Confirm WDT behavior in target sleep mode (some MCUs gate WDT off in deep sleep — we need WDT running during normal operation, not in the post-cutoff sleep state).
- Bench-verify the inverted topology end-to-end: ramp bench supply down on the eval rig (after R100 cut + R2 reroute), confirm LEDs cut off cleanly at firmware threshold, then physically reset the eval-board MCU mid-emission and confirm LEDs go off within the WDT period.
- Bench-calibrate Vref accuracy per board after assembly; record offset → store in EEPROM byte 0x00 for runtime correction (optional precision pass).

**Alternatives considered + rejected (this revision)**:

- **Original R11 v1 (firmware-only ADC, no topology change)**: rejected — leaves the failsafe direction wrong (high-Z = LEDs ON). Violates FR-1.7 #4 intent even with BOD backstop, because BOD reset still leaves PA3 high-Z → DIM HIGH → LEDs ON during the boot window before firmware re-asserts PA3 LOW.
- **Dedicated supervisor IC (TPS3700 / MCP1317T / similar)**: rejected — adds BOM cost + footprint area to solve a problem the topology + WDT already covers at zero parts cost. The original spec rationale (hardware-only fail-safe) is satisfied differently — by the schematic-level pull-down topology — rather than by an external chip.
- **Hybrid (supervisor at 3.0 V + firmware at 3.5 V)**: rejected — gives strictly better cell protection in the hang-while-driving-HIGH window, but the operator-accepted trade-off is "WDT bounds the damage to ≤ 0.02 % per incident, simpler BOM wins" (see Spec Revision FR-1.7 #4 above).
- **Analog Comparator hardware-trigger mode**: rejected — would require external R-divider on a GPIO (defeats BOM savings) AND limit available GPIOs. Bandgap-via-ADC is strictly better.

---

### [R11.A] Draft replacement text for spec.md §458 — FR-1.7 #4

*(For operator review. Replace the existing "4. Battery low-voltage cutoff (LiPo protection — safety-critical):" paragraph; everything else in FR-1.7 stays as-is.)*

> 4. **Battery low-voltage cutoff (LiPo protection — safety-critical)** *(revised 2026-05-20 per R11)*: the pod SHALL stop driving the LEDs when the 1S cell drops to **3.5 V** (firm UVLO threshold). This cutoff is implemented as **three defense-in-depth layers**, none requiring an external supervisor IC. Implementation contract:
>
>    a. **Topological failsafe** (Layer 1, always-on): the LM3410X DIM net is held LOW by a 10 kΩ resistor to GND. The MCU drives DIM HIGH (push-pull) to enable the LED string; any MCU-offline state (POR, BOD reset, WDT reset, software reset, brown-out, or firmware hang in GPIO input mode) leaves PA3 high-impedance, R2 pulls DIM low, and the LM3410X enters its ~80 nA shutdown state. **This is the primary fail-safe** — the schematic topology itself prevents stranded-LED-on conditions.
>
>    b. **Firmware ADC cutoff** (Layer 2, normal-operation): the MCU samples its supply voltage every 100 ms via the internal 1.1 V bandgap reference channel (ratiometric measurement: V_BAT = 1.1 V × 1024 / ADC_raw, using V_BAT itself as Vref). On 5 consecutive readings below **3.6 V threshold** (firmware-set; 100 mV margin above the 3.5 V spec to absorb Vref ±4 % drift), the MCU drives PA3 LOW and enters POWER_DOWN sleep (~100 nA hold). Wake-up only on battery removal + re-insertion.
>
>    c. **Watchdog timer** (Layer 3, hang-protection): the MCU's internal WDT is enabled with a **≤ 250 ms timeout**, petted in the main loop. If firmware hangs while PA3 is driving HIGH (the only Layer 1 fail-safe gap), WDT resets the MCU within 250 ms; PA3 returns to high-Z; Layer 1 takes over. **Maximum "stranded LEDs on past true UVLO" window: 250 ms ≈ 0.02 mAh ≈ 0.02 % of a 100 mAh pack.**
>
>    **Threshold**: **3.5 V real V_BAT** (firmware trip set at 3.6 V to absorb Vref drift) with implicit hysteresis (once tripped, MCU stays in POWER_DOWN sleep — no re-engagement until battery removal cycles power).
>
>    **Reasoning for 3.5 V vs original 3.3 V** (revised 2026-05-20): 3.5 V at rest ≈ ~20 % SOC; under-load sag-corrected trip is ~3.4 V → ~10 % SOC. Preserves cell cycle-life better than the 3.3 V threshold (which approached 5 % SOC under load) and gives the operator a "land now" warning window before complete cutoff. Runtime cost: ~1 min off the original ~13 min estimate per 100 mAh pack.
>
>    **Reasoning for firmware + WDT vs original "hardware supervisor IC, MCU-independent"** (revised 2026-05-20): the *intent* of MCU-independent UVLO — that no firmware failure mode strands the LED driver running — is preserved by the topological-failsafe layer (R2 pull-down + push-pull active-HIGH MCU GPIO). MCU is the cutoff path, but the schematic is the failsafe. The only non-self-protecting failure case (MCU hangs while actively driving HIGH) is bounded by the WDT to ≤ 250 ms / ≤ 0.02 % cell impact per incident. Operator-accepted trade-off: simpler BOM (no supervisor IC, no decoupling cap) + tunable threshold (no part-suffix decoding hazard) at the cost of accepting per-hang ≤ 0.02 % cell wear.
>
>    **NOT a separate supervisor IC**: previous wording mandating MCP1316T / TPS3839 / APX803 is withdrawn. Those parts may not exist at the required 3.5 V threshold in the right package + output-type combination (the TPS3839 family caps at 3.08 V then jumps to 4.38 V; the family is also push-pull, not open-drain as the original wired-AND topology required). Going firmware-side avoids the part-survey rabbit-hole entirely.
>
>    **Bench-verify**: (a) on the eval rig (`cad/beacon-eval/`, after R100 cut + R2 reroute), ramp the bench supply down from 4.0 V to 3.0 V; scope the LM3410X DIM pin + V_LED rail to confirm cutoff at 3.5 V real V_BAT with the 500 ms debounce window; (b) physically issue a soft-reset to the MCU mid-emission, confirm DIM goes low within ≤ 1 ms (POR delay) + WDT period ≤ 250 ms ≈ ≤ 250 ms total; (c) verify post-cutoff battery quiescent current is < 100 µA (MCU sleep + LM3410X shutdown + dim-line resistor leakage).

---

## Summary of P1 critical-path resolutions

| Item | Resolved | Outstanding (NEEDS-EXEC-RESEARCH) |
|---|---|---|
| R1 CAD MCP | FreeCAD MCP (primary) / `freecadcmd` scripted (fallback) | Confirm specific MCP package + tool schema at exec |
| R2 ATtiny412 toolchain | avr-gcc + serialUPDI + pymcuprog | None |
| R3 EVN onboard memory | Likely 8-16 MB HyperRAM (sufficient) | Confirm via board user guide |
| R4 OG0VA sourcing | Ship OV9281 / Arducam B0162 primary; OG0VA as later upgrade | OEM-channel inquiry for OG0VA |
| R5 Hand-prototype | Curiosity Nano for MCU bring-up; bare-SOT-23 + adapter for ICs; perfboard 25 × 25 mm | Confirm Adafruit / SparkFun / Pololu currently-stocked breakouts |
| R6 Lens | Commonlands custom (production); m12lenses + Edmund/Thorlabs (prototype) | Commonlands quote + lead time |
| R7 NFR-4 sim | numpy script in `specs/031-beacon-camera/nfr4-clockdrift-sim/` | None |
| R8 1S LiPo | Tinywhoop 1S 100 mAh, JST-PH 2.0, multiple-vendor | Confirm Amazon B083NWXLTK availability |
| R9 Lattice toolchain | Propel (P1) | Confirm Propel reference designs target LIFCL-40 |
| R10 FreeCAD MCP invocation | FreeCAD MCP (primary), `freecadcmd` scripted fallback | Confirm MCP tool schema at exec |
| R11 Undervoltage cutoff | **Topological failsafe (DIM pull-DOWN to GND) + firmware ADC at 3.6 V + WDT ≤ 250 ms** (no supervisor IC, no decoupling cap, R2 just rewired) | Confirm ADC MUXPOS for bandgap channel + BODLEVEL fuse + WDT behavior in target sleep mode at P1a firmware bring-up. Pending spec edit on FR-1.7 #4 per draft [R11.A] |

**Net**: 3 items are fully resolved (R2, R7); 7 items have a *decision* with exec-time confirmation needed (R1, R3, R4, R5, R6, R8, R9, R10). None are blocking the plan structure — all P1 contracts can be written now; vendor / part details get plugged in as the audit completes.
