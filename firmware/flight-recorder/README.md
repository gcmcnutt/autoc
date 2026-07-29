# 031-beacon-camera — Flight-recorder FPGA gateware

**Target board**: Lattice CrossLink-NX-EVN ("Voyager") with **LIFCL-40-9BG400C** FPGA.
**Toolchain**: Lattice **Propel** (RISC-V soft-CPU + reference designs) per [research.md R9](../../specs/031-beacon-camera/research.md).
**Bitstream role (Phase 1)**: camera-config (I²C) + MIPI ingest + HyperRAM ring buffer + 4-bit SDIO chunked write to microSD + status-LED state machine. NO DSP.
**Bitstream role (Phase 2 / 031-fpga)**: same board, new gateware — adds the five-stage perception pipeline + I²C output.

## Directory layout

```
firmware/flight-recorder/
├── rtl/                            # Verilog source modules
│   ├── camera_config_i2c.v         # OG0VA / OV9281 register init
│   ├── mipi_ingest.v               # MIPI CSI-2 → BRAM double-buffer
│   ├── sdram_ring.v                # HyperRAM ring buffer manager
│   ├── sd_writer.v                 # 4-bit SDIO CMD25 chunked writes
│   └── status_led.v                # FR-2.6 heartbeat-blink + fault FSM
├── tb/                             # Testbenches (Icarus Verilog OR Lattice ModelSim)
│   └── sd_writer_tb.v              # SD writer chunk-format verification
└── propel-project/                 # Lattice Propel project — NOT in this repo
    └── README.md                   # Setup steps (operator runs Propel locally)
```

## Bring-up steps (operator)

See [quickstart.md (b)](../../specs/040-camera-redo/camera-hardware-phase/quickstart.md) for the full walk-through.

1. Receive the CrossLink-NX-EVN board (Lattice direct / Mouser).
2. Install Lattice Propel + Radiant + the LIFCL-40 device support pack.
3. Create the Propel project under `propel-project/` (operator step — Propel GUI walks through soft-CPU + peripheral selection).
4. Add this repo's `rtl/` modules to the project source list.
5. Wire the soft-CPU's main loop per `contracts/fpga-recorder-contract.md`:
   - Camera I²C config at boot
   - MIPI ingest enable
   - Ring-buffer ↔ SD-writer orchestration
   - Fault-handling FSM (Class 1/2/3 per `contracts/data-format.md`)
   - Status-LED heartbeat pulse on every CMD25 completion
6. Build the bitstream; flash to onboard flash via JTAG/USB.
7. Bench-verify per the M3.x milestones in [tasks.md](../../specs/031-beacon-camera/tasks.md).

## Why Propel + Radiant, not Radiant alone

Per [research.md R9](../../specs/031-beacon-camera/research.md): Propel's reference SDIO driver + the camera-ingest example shortcut weeks of HDL work. Radiant alone is the cleaner long-term path for 031-fpga's DSP pipeline (gateware-only, no soft-CPU).

## What's in this repo vs what stays in Propel

- **In repo** (`rtl/`, `tb/`): Verilog source for the recorder-specific modules + testbenches — version-controlled + reviewable.
- **Not in repo** (`propel-project/`): Lattice's binary project files + IP-block instantiations + toolchain-generated outputs — operator regenerates from Propel locally. A README in `propel-project/` documents the regeneration steps.

This split mirrors how the existing `xiao/.pio/` is gitignored while `xiao/src/` is committed.
