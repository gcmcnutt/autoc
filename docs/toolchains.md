# autoc Toolchains & Build Environment

Durable map of *where code builds and how to drive each toolchain*. Referenced from
[`CLAUDE.md`](../CLAUDE.md) so a session can fetch it on demand instead of carrying it in every context.
Keep this current when a toolchain, host, or invocation path changes.

## Host topology

- **Primary dev session**: WSL2 (Ubuntu) on a **Windows host**. The repo, git, C++/Python tooling, and
  spec-kit all live here.
- **WSL ↔ Windows interop is the integration mechanism** — this session invokes Windows `.exe` directly
  (no SSH). `/mnt/c` is the Windows C: drive, writable from WSL. **Trap:** Windows tools reject
  `\\wsl.localhost\…` (UNC) working directories — keep any Windows-tool build sandbox on `/mnt/c`, source
  in the repo, and copy/sync between them.
- **Training box**: `promaxgb10-4331` via Tailscale — see auto-memory `reference_training_box_ssh`. Used for
  evolution runs / heavy compute, not local builds.

## Toolchains

| Domain | Toolchain | Where it runs | How driven |
|---|---|---|---|
| **autoc / crrcsim** (C++17) | CMake + GoogleTest, Eigen, cereal, inih | WSL native | `./rebuild.sh` (constitution: unified build) |
| **Python 3.11** (analysis, sim, loaders) | venv / system py + numpy, pytest | WSL native | per-tool `pyproject.toml` / scripts |
| **xiao** (telemetry/AHRS MCU) | PlatformIO (arduino-mbed) | WSL native | `pio run` in `xiao/` |
| **Beacon emitter MCU** (031) | ATtiny412 — avr-gcc + serialUPDI (1-wire UPDI) | WSL native | `firmware/beacon-pod/Makefile`. *PlatformIO path for the emitter is a later option (operator note 2026-06-22).* |
| **FPGA — 031 acquisition correlator** | **Lattice Diamond 3.14** (MachXO2 / STEP-MXO2, `LCMXO2-4000HE`) | **Windows host** (`C:\lscc\diamond\3.14`, licensed) | from WSL via interop: `pnmainc.exe <build.tcl>` (verified working). **Details: [`specs/031-beacon-camera/fpga-toolchain-plan.md`](../specs/031-beacon-camera/fpga-toolchain-plan.md)** |
| **FPGA — HDL simulation** | `iverilog`/`vvp` (or Verilator) — open source | WSL native | golden vectors from `specs/031-beacon-camera/acquisition-sim/sim.py` |
| **FPGA — 040 camera redo** (deferred) | Lattice **Radiant + Propel** (CrossLink-NX / LIFCL-40) | Windows host | **NOT installed** — install only when 040 restarts |

## Artifact stores

- S3 per-mode buckets `autoc-m1` / `autoc-m2` / `autoc-eval`; milestone-preserve prefixes per auto-memory
  `reference_autoc_storage_keeper_runs`.

## Notes

- The FPGA correlator (031) targets a board the operator already owns (STEP-MXO2). Diamond is the *correct*
  tool for it — Radiant is only for the deferred 040 camera path; don't conflate them.
- Board programming (USB-JTAG) happens on the Windows side (Diamond owns the USB device); WSL needs no USB
  passthrough.
