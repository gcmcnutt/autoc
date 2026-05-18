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
| **TI LM3410-Y boost LED driver** | **Adafruit-style boost driver breakout** (LM3410 or similar) — likely not stocked as a named breakout; **fallback**: solder the LM3410-Y bare SOT-23-5 to a SOT-23-to-DIP adapter + dead-bug build on perfboard. **Alt**: **Pololu / SparkFun "constant-current LED driver" boards** (PT4115, AL8807, etc.) — if a substitute IC is acceptable for hand-prototype, use it. | The LM3410-Y is not a "common breakout" part; bare-SOT-23 + adapter is the most likely path. |
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

**Decision**: **plain `numpy` Python script** in `tools/nfr4-clockdrift-sim/`.

**Rationale**: the simulation is small — a few hundred LOC. Generates a Gold-code template, applies chip-rate offset, generates a synthetic received signal at simulated SNR, runs matched-filter correlation across hypothesis offsets, computes acquisition probability via Monte Carlo. No need for GNU Radio or Matlab; numpy + scipy.signal cover everything.

**Output**: `tools/nfr4-clockdrift-sim/sim.py` + a `simulation-results.md` doc with the decision (crystal vs multi-hypothesis vs internal-RC) recorded as a Decisions-Locked spec update.

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

## Summary of P1 critical-path resolutions

| Item | Resolved | Outstanding (NEEDS-EXEC-RESEARCH) |
|---|---|---|
| R1 CAD MCP | FreeCAD MCP (primary) / `freecadcmd` scripted (fallback) | Confirm specific MCP package + tool schema at exec |
| R2 ATtiny412 toolchain | avr-gcc + serialUPDI + pymcuprog | None |
| R3 EVN onboard memory | Likely 8-16 MB HyperRAM (sufficient) | Confirm via board user guide |
| R4 OG0VA sourcing | Ship OV9281 / Arducam B0162 primary; OG0VA as later upgrade | OEM-channel inquiry for OG0VA |
| R5 Hand-prototype | Curiosity Nano for MCU bring-up; bare-SOT-23 + adapter for ICs; perfboard 25 × 25 mm | Confirm Adafruit / SparkFun / Pololu currently-stocked breakouts |
| R6 Lens | Commonlands custom (production); m12lenses + Edmund/Thorlabs (prototype) | Commonlands quote + lead time |
| R7 NFR-4 sim | numpy script in `tools/nfr4-clockdrift-sim/` | None |
| R8 1S LiPo | Tinywhoop 1S 100 mAh, JST-PH 2.0, multiple-vendor | Confirm Amazon B083NWXLTK availability |
| R9 Lattice toolchain | Propel (P1) | Confirm Propel reference designs target LIFCL-40 |
| R10 FreeCAD MCP invocation | FreeCAD MCP (primary), `freecadcmd` scripted fallback | Confirm MCP tool schema at exec |

**Net**: 3 items are fully resolved (R2, R7); 7 items have a *decision* with exec-time confirmation needed (R1, R3, R4, R5, R6, R8, R9, R10). None are blocking the plan structure — all P1 contracts can be written now; vendor / part details get plugged in as the audit completes.
