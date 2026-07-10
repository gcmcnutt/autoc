# Lattice Propel project — setup steps (operator)

This directory holds the Lattice Propel project for the 031-beacon-camera flight recorder. The project files themselves are toolchain-generated and not version-controlled (see `.gitignore`).

## One-time setup

1. Install Lattice Propel SDK + Radiant from [latticesemi.com](https://www.latticesemi.com/) (free registration required).
2. Install the LIFCL-40 device pack.
3. `cd firmware/flight-recorder/propel-project/`
4. Launch Propel, create a new project targeting `LIFCL-40-9BG400C`.
5. Add IP blocks per `contracts/fpga-recorder-contract.md`:
   - MIPI D-PHY hard IP (2 lanes)
   - HyperRAM controller
   - SDIO controller (4-bit mode, ≥25 MHz)
   - RISC-V soft-CPU (Propel default core)
   - GPIO for status LED
6. Add the Verilog modules from `../rtl/` to the source list.
7. Wire the soft-CPU's main loop per [contracts/fpga-recorder-contract.md](../../../specs/031-beacon-camera/contracts/fpga-recorder-contract.md).
8. Build the bitstream → flash to onboard flash via JTAG/USB.

## What's gitignored

- `*.lpf`, `*.rdf`, `*.synproj`, `impl/`, `obj/`, `build/`, `*.bit`, `*.jed`, `*.gen/` — toolchain-generated.

## What's committed

- `README.md` (this file).
- A Propel `project.tcl` script (when one exists) that regenerates the project structure from the committed `rtl/` sources. **Not yet written** — operator authors this after first project creation, so future fresh checkouts can reproduce the build.
