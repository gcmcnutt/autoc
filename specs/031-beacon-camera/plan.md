# Implementation Plan: 031 Beacon-Camera Optical Perception — Phase 1

**Branch**: `031-beacon-camera` | **Date**: 2026-05-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/home/gmcnutt/autoc-beacon/specs/031-beacon-camera/spec.md`

## Summary

Phase 1 of 031 delivers a **bench- and field-runnable beacon-camera setup that records raw video of two coded-IR beacons at flight-relevant ranges and dynamics** ([spec.md §Overview](./spec.md)). No simulator code is touched. No `(x, y, CEP)` extraction. No NN-in-the-loop. The deliverable is **photons → raw clip on SD card → Python-loadable numpy array** end-to-end, validated by one paired-craft test flight (US6).

**Approach** — operator direction 2026-05-17:
- **Multi-eval-board de-risk strategy**: prove each chunk of work on its own off-the-shelf eval board *before* the integrated hand-built path. Specifically:
  - **Beacon firmware** → bring up first on an ATtiny412 **Curiosity Nano** dev board + breadboard with the LED string + boost driver IC + scope. Validates timing + LUT-bit pattern + chip rate before any pod is hand-built.
  - **Camera + optical chain** → bring up first on an **off-the-shelf USB-UVC camera module** (Arducam B0264 USB shield + B0162 OV9281, or equivalent). Live-streams to a host PC, gives instant first-light + EMI debug + exposure tuning without any FPGA gateware effort. **This is the FR-4.1 bench-mode path** (see Clarifications Session 2026-05-17).
  - **Recording format** → exercised end-to-end via a Python `tools/beacon-viewer/` utility that consumes UVC frames + writes the canonical `.clip` format. Loader contract is round-tripped before flight gateware lands.
  - **Flight-mode FPGA gateware** → the long-lead Lattice CrossLink-NX-EVN path (FR-4.1b) is built **in parallel** to the bench-mode work, de-risked by the bench-mode optical + format proofs.
- **Hand-prototype first, no PCB spin** until the optical chain is proven on dev modules. The first beacon pod is point-to-point wiring on a perfboard inside the 3D-printed half-cube; PCB design is deferred to a follow-on or 031-integration.
- **CAD-via-MCP** for the enclosure work — connect the **FreeCAD MCP server** at P1 execution time and use it as the back-part / mount-design mechanism. FreeCAD's parametric solid-modeling + native STEP/STL export is the chosen CAD toolchain.
- **Datasheet audit phase** is explicit — every BOM part gets its datasheet pulled and cross-checked against FR-1.2.1 claims (pinout, footprint, height, soft-start, DIM/EN, UVLO, wavelength bin) before any hardware is ordered.
- **Receiver is camera-separate-from-FPGA**, both tape-mounted to whatever airframe is available (test quad, hb1, or similar). Production-weight perception hardware is explicitly out of scope; mount-on-whatever-flies is the design point.

**Phasing** (operator direction):

| Phase | Output |
|---|---|
| **P1 — Design + audit + first-eval-board bring-up** | Verified BOM, hand-prototype build guide, CAD enclosure (STEP/STL via FreeCAD MCP), NFR-4 clock-drift simulation, `data-format.md`, `recorder-status-codes.md`, `eye-safety-measurements.md` (template). **Eval-board first-light**: USB-UVC camera (Arducam B0264 + B0162) running + Python `tools/beacon-viewer/` displaying live frames + recording to `.clip` format; ATtiny412 Curiosity Nano blinking a test LED at the 100 Hz chip rate + verified by scope-trace decode. Optical and code paths proven on commodity eval boards before any hand-built hardware. |
| **P2 — Beacon hand-build + bench** | First pod hand-built per build guide; FR-3.3 EMC + FR-3.4 eye-safety + FR-1.7 UVLO verifications PASS; second pod built; FR-1.5 orthogonality verified. Bench-mode UVC camera from P1 is the verification rig (live-display of the pod's IR emission + recovered Gold code). |
| **P3 — Flight-mode recorder build + bench scenario sweep** | Lattice EVN gateware (`firmware/flight-recorder/`) producing first frames to SD; FR-2.4/2.5/2.6/2.7 working; Class 1/2/3 fault-handling bench-injected + verified; FR-2.7 mount on a tape carrier. **Bench scenarios S1–S9 (FR-5.1)** recorded via two paths in parallel: (a) UVC bench-mode for fast iteration, (b) Lattice flight-mode for the flight-format proof. FR-4.3 Python loader ingests every clip from both paths. |
| **P4 — US6 Beacon Test Flight 1** | One paired-craft session ≥5 min air time on flight-mode (Lattice EVN) recorder; clip passes the 3-criterion US6 acceptance gate |
| **P5 — Close-out + handoff** | Bench-log + outcome.md; tag the EVN camera-ingest gateware as the 031-fpga input; archive recordings for 031-noise-cal follow-on |

## Technical Context

**Languages / Toolchains**:
- **Embedded MCU**: C (avr-gcc + serialUPDI programmer for ATtiny412); flashed via UPDI 1-wire serial — no proprietary tool required. Microchip MPLAB X is the IDE alternative.
- **FPGA gateware (recorder)**: Lattice Radiant (the LIFCL-40 toolchain) or Lattice Propel for the soft-CPU + SD-record reference design. **Bitstream is camera-config + bulk SD record only in Phase 1**; the five-stage DSP pipeline is 031-fpga's job.
- **Python 3.11**: FR-4.3 clip loader, JSON sidecar handler, NFR-4 clock-drift simulation, hand-rolled DSP scripts for US6 acceptance.
- **C++17**: only if a simulator hook for the recorded clips is added in Phase 1 (not currently planned — that's 031-noise-cal). If added, integrates under the existing `src/eval/` umbrella per constitution.
- **CAD**: **FreeCAD MCP server** — invoked from Claude Code at P1 execution time, produces STEP + STL files. FreeCAD project files (`.FCStd`) checked into `cad/source/`.

**Primary Dependencies**:
- **Hardware (beacon)**: Lumileds L1IZ-0850000000000 ×5, TI LM3410-X, ATtiny412, 22 µH shielded SMD inductor, MBR130 Schottky, 1S LiPo 100 mAh JST-PH 2.0 pack, decoupling caps, code-select jumpers (or CUI DSM-02), 0603 diagnostic LED. Hand-prototype substitutes: Pololu / SparkFun / Adafruit breakouts (see R5). *(revised 2026-05-20 per FR-1.7 #4 / R11: external voltage supervisor IC removed — UVLO is now firmware ADC + topological failsafe + WDT.)*
- **Hardware (recorder)**: Lattice CrossLink-NX-EVN (LIFCL-40-9BG400C), OmniVision OG0VA (primary) or OV9281 via Arducam B0162 (backup), Commonlands custom M12 NIR-corrected 120° lens with integrated 850 ± 5 nm CWL / ≤30 nm FWHM bandpass filter (prototype path: m12lenses.com PT-02120 + Edmund Optics #65-679 or Thorlabs FB850-10), SanDisk Extreme Pro microSD V30 (240 fps) / V60 (480 fps), Pololu D24V10F5 5V buck.
- **Software**: `numpy` (loader), `pytest` (loader contract test), `cereal` (only if a C++ schema is added — not currently planned), `inih` (N/A this phase).
- **Sourcing channels**: DigiKey / Mouser (most parts), Arducam direct (B0162), Lattice direct (EVN board), OmniVision OEM channel for OG0VA, Commonlands direct for custom lens.

**Storage**:
- **On-pod**: nothing persistent (LUT compiled into MCU flash).
- **On-recorder**: microSD card with **FR-4.2 chunked binary format** (`format_version` uint16 per chunk per Principle V) + JSON sidecar. Pre-allocated file + direct-sector writes per FR-2.5.
- **Ground analysis**: clip files + sidecars in `specs/031-beacon-camera/clips/` (or external store for large sets) — not source-tree-tracked beyond a small set of golden test clips.

**Testing**:
- **Python (loader, sim hooks)**: `pytest` — contract test for `load_clip()` round-trip, schema-version mismatch fail-loud test, JSON-sidecar validation.
- **C++ (if added)**: GoogleTest per constitution.
- **MCU firmware**: oscilloscope-verified chip-rate output (FR-1.5(a)); LUT-bit pattern decoded by Python from a captured scope trace.
- **FPGA gateware**: simulation testbench (Lattice ModelSim / VUnit equivalent) for the SD-record path; bring-up bench check that frames-counted-out == frames-counted-in.
- **Hardware bench-verifications (FR-3.x)**: stand-in for the constitution's "tests" — physical measurements with documented PASS/FAIL criteria (EMC, eye-safety, UVLO, orthogonality, emission pattern). Bench log per FR-5.2 is the test-result-of-record.

**Target Platform**:
- **Beacon pod**: ATtiny412 + LM3410-X boost driver hand-wired inside a 3D-printed half-cube enclosure, tape-mounted to a flat ~2.5 cm wing-tip outboard face (target craft: hb1-class or any RC fixed-wing with wing-tip access).
- **Recorder**: Lattice CrossLink-NX-EVN + camera module, **double-back-tape (or velcro) mounted** to whatever surface a carrier craft offers. First flights: **test quad, hb1, or any platform with ~100 g payload margin** — mount-on-whatever-flies is the explicit design intent.
- **Ground**: Linux PC (operator workstation) for Python loader + analysis.

**Project Type**: hardware-firmware-optics integration with a small Python toolchain. Atypical for the spec template; closer to "embedded systems + sidecar tooling" than a software-only feature.

**Performance Goals**:
- Beacon: 100 Hz chip rate ±0.1%, 5-LED series at 300 mA constant current, ~14 min runtime / 100 mAh charge, IEC 62471 RG0 ≤10 s at 200 mm.
- Camera: 320×240 @ 240 fps (baseline) and 480 fps (high-rate); 8-bit and 10-bit raw both selectable; manual AGC + manual exposure via I²C, auto-AGC disabled.
- Recorder SD throughput: 18.4 MB/s (240 fps 8-bit baseline) up to 46.1 MB/s (480 fps 10-bit-packed worst case), sustained with no frame drops.
- US6 acceptance: one ≥5 min paired-craft flight session producing one clip loadable + DSP-pass-through + visualization-pass per the three FR-4 criteria.

**Constraints**:
- Per-pod mass ≤6 g (target 4.5–5 g per FR-1.2.1 BOM).
- Recorder system mass ~70–100 g (carrier-craft path; production-weight optimization is deferred to 031-integration).
- EMI: gyro broadband RMS bump ≤3 dB, RSSI drop ≤2 dB, link-quality drop ≤5 % (FR-3.3).
- LiPo UVLO at 3.5 V real V_BAT (firmware ADC trip at 3.6 V) + topological failsafe (DIM pull-down to GND) + WDT ≤ 250 ms (FR-1.7 #4, revised 2026-05-20 per R11).
- Beacon MCU clock drift ±5 % bounded; NFR-4 simulation chooses crystal vs multi-hypothesis correlator before PCB freeze.
- Eye safety: IEC 62471 RG0 at ≤10 s exposure @ 200 mm (FR-1.7 + US6 gate).
- LED + filter + sensor wavelength binning: all at 850 ± 5 nm CWL (FR-1.1 + FR-2.2).
- No PCB spin in Phase 1 (operator direction 2026-05-17) — hand-prototype only.

**Scale / Scope**:
- 2 beacon pods (one per wing on target craft) + 1 spare pod for swap if US6 build fails one.
- 1 recorder system (1 camera, 1 FPGA eval board, 1 SD card, 1 buck regulator).
- 9 bench scenarios (S1–S9 per FR-5.1).
- 1 paired-craft test flight (US6).
- 5 sub-deliverable docs (data-format.md, eye-safety-measurements.md, recorder-status-codes.md, hand-prototype-guide.md, verified-bom.md).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Phase-1 Posture | Status |
|---|---|---|
| **I. Testing-First** | Python loader: pytest contract tests written before the loader implementation. MCU firmware: scope-trace decode is the hardware-side "test" for chip-rate + LUT correctness. FPGA gateware: ModelSim/VUnit testbench for the SD-record path. Hardware bench-verifications (FR-3.x) are the test-of-record for the optical/electrical claims; documented in `eye-safety-measurements.md` + bench log. | ✅ PASS |
| **II. Build Stability** | Python loader integrates under existing tooling. Any C++ added (none planned in Phase 1) goes through `rebuild.sh` per constitution. FPGA bitstream + MCU firmware are out-of-tree builds — not in the `rebuild.sh` path. | ✅ PASS |
| **III. No Compatibility Shims** | Greenfield. Format-version 1 of `data-format.md` is the only on-disk contract introduced. No back-compat wrappers anywhere. | ✅ PASS |
| **IV. Unified Build** | Python loader sits in `tools/beacon-loader/` or similar — does not duplicate cereal/inih/GoogleTest declarations. FPGA + MCU builds are explicitly separate (different toolchains; can't unify with CMake). | ✅ PASS |
| **V. Versioned Persistence Artifacts** | **`data-format.md` is the on-ramp**: `format_version` uint16 at a stable parseable offset per chunk; FR-4.3 Python loader is the reference implementation with **fail-loud on version mismatch** (no silent default-init, no truncation). Write-side: FPGA recorder writes `format_version = 1` directly into each chunk header. Schema covers raw 8/10-bit, per-frame µs timestamp from FPGA monotonic counter, per-chunk header, JSON sidecar metadata. | ✅ PASS |
| **VI. Type-Domain Discipline** | Phase 1 does NOT touch `src/eval/` or `src/nn/`. The Python loader returns `numpy.ndarray` (host-side, not eval-pipeline) — `gp_scalar` / `gp_vec3` / `gp_fitness` aliases are irrelevant. If 031-noise-cal (future) adds a C++ hook to consume clips inside the simulator, Principle VI applies there. | ✅ N/A this phase |

**Gate result**: PASS. No violations to justify. Proceeding to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/031-beacon-camera/
├── spec.md                         # ✅ written; clarified + analyzed 2026-05-17
├── plan.md                         # ⬅ this file
├── research.md                     # Phase 0 output
├── data-model.md                   # Phase 1 output (data-format.md schema + JSON sidecar schema)
├── quickstart.md                   # Phase 1 output (build-one-pod + build-one-recorder walk-through)
├── contracts/
│   ├── data-format.md              # canonical chunked-binary clip format (versioned)
│   ├── python-loader.md            # FR-4.3 load_clip() interface contract
│   ├── mcu-firmware-contract.md    # ATtiny412 chip-rate output + LUT-bit timing contract
│   ├── fpga-recorder-contract.md   # FPGA → SD-card byte format + chunk-write atomicity
│   └── json-sidecar-schema.json    # JSON sidecar schema (consumed + emitted by loader)
├── verified-bom.md                 # P1 datasheet-audit output (each part datasheet-checked)
├── hand-prototype-guide.md         # P1 step-by-step assembly (perfboard + breakouts + wiring)
├── eye-safety-measurements.md      # P1 template, P2-populated IEC 62471 measurements
├── recorder-status-codes.md        # P1 LED blink-code reference card
├── cad/
│   ├── beacon-half-cube.step       # 3D enclosure, primary CAD output (via FreeCAD MCP)
│   ├── beacon-half-cube.stl        # FDM-printable mesh (exported from FreeCAD)
│   ├── recorder-mount-template.stl # Optional tape-on adapter for the eval board
│   └── source/
│       └── beacon-half-cube.FCStd  # FreeCAD parametric project (source-of-truth)
├── bench-logs/                     # P2/P3/P4 bench-log entries per FR-5.2 (one .md per session)
├── clips/                          # Phase-1 recorded clips (golden small set; bulk archived externally)
└── tasks.md                        # NOT created by /speckit.plan — /speckit.tasks output
```

### Source Code (repository root)

The repo is multi-language (autoc C++17, xiao C/C++ via PlatformIO, Python analysis scripts). Phase 1 adds **firmware + tools subtrees** alongside the existing structure:

```text
firmware/
├── beacon-pod/                     # ATtiny412 firmware (avr-gcc + serialUPDI)
│   ├── src/
│   │   ├── main.c                  # boot → read code-select → 100 Hz timer ISR → DIM toggle
│   │   ├── gold_codes.c            # 4× N=15 Gold-code LUTs
│   │   └── config.h                # chip-rate, LUT length, GPIO pin mapping
│   ├── Makefile                    # avr-gcc invocation, UPDI flash target
│   └── tests/
│       └── scope-trace-decode.py   # consumes a captured scope CSV, asserts the LUT bit sequence
└── flight-recorder/                # Lattice LIFCL-40 gateware (Radiant project)
    ├── rtl/
    │   ├── camera_config_i2c.v     # OG0VA/OV9281 register init
    │   ├── mipi_ingest.v           # MIPI CSI-2 → BRAM double-buffer
    │   ├── sdram_ring.v            # external SDRAM ring buffer (per F14 sweep)
    │   ├── sd_writer.v             # 4-bit SDIO direct-sector writes
    │   └── status_led.v            # FR-2.6 blink-code state machine
    ├── tb/
    │   └── sd_writer_tb.v          # ModelSim testbench
    └── propel-project/             # Lattice Propel project file

tools/
├── beacon-loader/                  # Python loader + simulator hooks
│   ├── beacon_loader/
│   │   ├── __init__.py
│   │   ├── loader.py               # load_clip(path) → (np.ndarray, dict)
│   │   ├── schema.py               # format_version validation + JSON sidecar schema
│   │   └── chunk.py                # chunked-binary read implementation
│   ├── tests/
│   │   ├── test_loader_contract.py # FR-4.3 round-trip test
│   │   ├── test_version_mismatch.py# Principle V fail-loud test
│   │   ├── test_loader_resilience.py # fault-sentinel + brown-out tests
│   │   └── test_sidecar_schema.py  # JSON sidecar validation
│   └── pyproject.toml
└── beacon-viewer/                  # Bench-mode live UVC viewer + recorder (FR-4.1)
    ├── beacon_viewer/
    │   ├── __init__.py
    │   ├── uvc_capture.py          # V4L2 / pyuvc capture from Arducam UVC USB
    │   ├── live_display.py         # Real-time frame display (Qt or matplotlib)
    │   └── record_to_clip.py       # Write captured frames into FR-4.2 .clip format
    ├── tests/
    │   └── test_record_to_clip.py  # Bench-mode recordings round-trip via beacon-loader
    └── pyproject.toml

# No changes to existing autoc/, crrcsim/, xiao/, src/ in Phase 1.
```

**Structure Decision**: **multi-subtree** (firmware/ + tools/ + specs/) because the deliverable spans MCU firmware, FPGA gateware, host-side Python, hardware enclosures, and operational docs. The existing repo already segregates `xiao/` (PlatformIO embedded), `crrcsim/` (third-party-style C++ build), and top-level `src/` + `scripts/` (autoc evolution + Python analysis); the new `firmware/` and `tools/` subtrees follow that segregation pattern. No new top-level CMakeLists.txt entries are needed in Phase 1 (firmware builds out-of-tree; Python loader has its own pyproject.toml).

## Complexity Tracking

No constitution violations. Hand-prototype-instead-of-PCB is a *simplification* of the spec's Plan Dep B.2 (PCB layout), driven by operator direction 2026-05-17. PCB design is deferred — not a violation, just a re-phasing.

| Risk | Why it's accepted | Mitigation |
|---|---|---|
| Hand-prototype circuit may have higher EMI than a designed PCB | EMI envelope is *radiated-only* with path C (no airframe connection); FR-3.3 bench gate catches it before US6 | If FR-3.3 fails, fall back to designed PCB before US6 |
| CAD MCP server not yet identified | The user has committed to wiring one in at P1 execution time | Plan flags this as R10 in research.md; P1 cannot complete without it |
| OG0VA OEM-channel lead time unknown | Spec already names OV9281 / Arducam B0162 as the bench-bring-up backup; ship with the backup, hot-swap if OG0VA lands in time | OV9281 is the de-risk path; OG0VA is the upgrade |
| 1.6 MHz boost-converter EMI vs FC gyros | Identical risk to all FPV/drone boost converters; well-understood mitigation set | FR-1.6 mitigations + FR-3.3 bench gate |

---

## Phase 0: Research

See [research.md](./research.md) for the resolution of each NEEDS CLARIFICATION item.

Top-level research items identified during Technical Context fill:

| ID | Question |
|---|---|
| R1 | FreeCAD MCP server — confirm install/connection at P1 execution; identify the specific MCP package being used |
| R2 | ATtiny412 toolchain: avr-gcc + serialUPDI cable (DIY) vs Microchip MPLAB X + PICkit (commercial)? |
| R3 | Lattice CrossLink-NX-EVN onboard SDRAM/HyperRAM part + capacity + DMA path — sufficient for the 6 MB ring buffer? |
| R4 | OG0VA OEM-channel sourcing reality (lead time + MOQ) — gates the P2/P3 decision to ship OG0VA primary or OV9281 backup |
| R5 | Hand-prototype breakouts: identify Adafruit / SparkFun / Pololu / DigiKey breakouts for LM3410-X (or compatible) and ATtiny412 — must fit in the 2.5 × 2.5 × 1.3 cm enclosure. *(revised 2026-05-20 per R11: voltage supervisor IC dropped from BOM, no breakout needed.)* |
| R6 | Commonlands lens lead time + Edmund/Thorlabs filter availability + the m12lenses.com PT-02120 IR-cut status — drives the lens-side P2 schedule |
| R7 | NFR-4 clock-drift simulation: numpy vs GNU Radio vs Matlab; pick the one with fastest "write a matched filter sweep" path |
| R8 | 1S LiPo 100 mAh pack: confirm Amazon B083NWXLTK or pick a named DigiKey-stocked equivalent for supply-chain reliability |
| R9 | Lattice toolchain: Radiant (HDL-direct) vs Propel (with a soft-CPU + driver lib for SDIO) — which has a closer-to-out-of-the-box camera-ingest + SD-record reference design? |
| R10 | CAD-MCP invocation pattern from Claude Code: how does the planner reliably hand off "build a 25 × 25 × 13 mm half-cube with these features" and receive STEP/STL back? |

---

## Phase 1: Design & Contracts

**Prerequisites**: research.md complete (all NEEDS CLARIFICATION resolved).

### Deliverables

| Artifact | Source | Contract role |
|---|---|---|
| `data-model.md` | This plan | Entity definitions: ClipFile, Chunk, Frame, JSONSidecar; relationships; validation rules |
| `contracts/data-format.md` | Spec FR-4.2 + Principle V | Canonical chunked-binary byte layout with `format_version` uint16 per chunk |
| `contracts/python-loader.md` | Spec FR-4.3 | `load_clip(path) → (np.ndarray, dict)` signature + error contract (fail-loud on schema mismatch) |
| `contracts/mcu-firmware-contract.md` | Spec FR-1.3 | Chip-rate timing + LUT-bit GPIO signal at the DIM pin |
| `contracts/fpga-recorder-contract.md` | Spec FR-2.5 + FR-2.4 | MIPI ingest → SDRAM ring → SD chunk-write atomicity; FPGA-monotonic timestamp source authority |
| `contracts/json-sidecar-schema.json` | Spec FR-4.2 | JSON schema for the per-clip sidecar |
| `quickstart.md` | This plan | Three walk-throughs: (a) build one pod, (b) build one recorder, (c) record S1 and ingest with the loader |
| `verified-bom.md` | R5 + datasheet audit | Per-part datasheet citations + cross-check against spec claims (one row per part) |
| `hand-prototype-guide.md` | R5 + CAD output | Parts list (breakouts) + wiring diagram + step-by-step assembly inside the half-cube |
| `cad/beacon-half-cube.{step,stl}` + `cad/source/beacon-half-cube.FCStd` | FreeCAD MCP per R1/R10 | 3D enclosure with battery cavity, slide-in opening, 5 LED indents, light-pipe slot for diagnostic LED, code-select access slot. FreeCAD project file is the editable source-of-truth |
| `eye-safety-measurements.md` (template) | Spec FR-1.7 + FR-3.4 | Empty measurement table to be populated in P2 |
| `recorder-status-codes.md` | Spec FR-2.6 | LED blink-code reference card |

### Agent-context update

Run after Phase 1 artifacts land:

```bash
.specify/scripts/bash/update-agent-context.sh claude
```

This refreshes `CLAUDE.md`'s "Active Technologies" / "Recent Changes" sections with the new 031-beacon-camera entries (Python loader + FPGA gateware + ATtiny412 firmware + CAD MCP integration).

---

## What `/speckit.plan` does NOT do

- Does not run the datasheet audit itself — `verified-bom.md` is a *plan-output template* whose body is filled in P1 execution.
- Does not call the CAD MCP — the planner only commits to using one; the actual STEP/STL files appear during P1 execution.
- Does not write firmware or gateware — only the per-component contracts.
- Does not produce `tasks.md` — that's `/speckit.tasks`.
