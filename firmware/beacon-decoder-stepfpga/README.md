# beacon-decoder-stepfpga — StepFPGA gateware (build-from-WSL)

Acquisition-correlator gateware for the 1-bit receiver, built on the **Windows-host Lattice Diamond**
from this **WSL** tree. Design rationale: [`../../specs/031-beacon-camera/fpga-toolchain-plan.md`](../../specs/031-beacon-camera/fpga-toolchain-plan.md).

## Layout (source-of-truth here; build is transient)

```
rtl/         *.v        — RTL source (committed)        ← edit here, from WSL
constraints/ *.lpf      — pin/timing constraints (committed)
tb/                     — testbenches (iverilog co-sim, F2)
build.tcl               — Diamond batch script (prj_project/prj_src/prj_run)
build.sh                — WSL driver: sync → Diamond via interop → .jed back  (+ --flash)
build/                  — copied-back .jed + reports  (GITIGNORED)
```
The actual Diamond build runs in a **transient sandbox** at `/mnt/c/fpga-build/stepfpga/` (NOT in this
tree, NOT committed) because Diamond rejects `\\wsl$` UNC working dirs. `build.sh` manages it.

## Usage

```bash
./build.sh            # sync + build → build/stepfpga_impl1.jed
./build.sh --flash    # also program the board (copies the .jed onto the STEPLink D: volume)
```

## How it works (VERIFIED end-to-end from WSL, 2026-06-23)

1. `cp -u` syncs `rtl/`+`constraints/`+`build.tcl` into `/mnt/c/fpga-build/stepfpga/` (only-if-changed →
   preserves Diamond's incremental detection; `cp -f` would force a full rebuild every run).
2. **`pnmainc.exe` is invoked directly** (binfmt interop) with the shell **CWD on `/mnt/c`** so Windows
   sees a native `C:\…` dir, not a UNC path. It runs `build.tcl`:
   - `prj_project new -dev LCMXO2-4000HC-4MG132C -synthesis synplify` (or `prj_project open` if it exists)
   - `prj_src add`, `prj_impl option top blink`
   - `prj_run Export -impl impl1 -task Jedecgen` → Translate→Synthesis→Map→PAR→`bitgen -jedec`
   - **`-task Jedecgen`** is required: `-task Bitgen` yields a `.bit` (SRAM-style, wrong for flash MachXO2);
     no `-task` runs timing only and emits no bitstream.
3. Diamond writes outputs to the **`impl1/` subdir**; `build.sh` copies `impl1/stepfpga_impl1.jed` back to
   `build/`.
4. **Flash (`--flash`)**: **WSL cannot see the STEPLink `D:` volume** (it is not auto-mounted, and
   `sudo mount -t drvfs D: /mnt/d` does not surface it), so the drop is a **Windows-side copy over interop**:
   `cmd.exe /c copy /Y stepfpga_impl1.jed D:\` — copying onto `D:` programs + restarts the board.

Incremental confirmed: a no-op rerun reports *"Nothing is executed for the Export - Jedecgen process"*; a
one-line RTL change re-runs the chain and changes the `.jed`.

## Status

- **Build path: PROVEN** from WSL (this `blink` is the F1 toolchain-proof design — divides the 12 MHz clock
  onto one LED; replaced by the correlator RTL once bring-up proceeds).
- **Flash + on-board blink: pending** a board-connected run (`--flash`) — also the chance to confirm the
  active pin constraints (see below).
- **Known refinement**: a *second* `.lpf` added via `prj_src add` is auto-`excluded`; the active preference
  file is the project's `stepfpga.lpf`. So `constraints/blink.lpf`'s `clk=C1 / led=N13` LOCATEs are **not yet
  applied** (Diamond auto-places I/O) — fix before the flash so the intended LED lights. See toolchain-plan §3.5.
