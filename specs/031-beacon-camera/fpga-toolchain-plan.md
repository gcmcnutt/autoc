# 031 — FPGA Toolchain & Bring-Up Plan (StepFPGA / MachXO2)

**Status**: DRAFT, 2026-06-22
**Scope**: how we develop the **acquisition-correlator gateware** for the 1-bit receiver
([acquisition-research-plan.md](acquisition-research-plan.md) §5) from this repo, and build/flash it on the
**Windows host's Lattice toolchain**. Covers the dev-environment architecture (WSL2 ↔ Windows),
the build/sim loop, and the first gateware target (MCP3201 soft-decision correlator).

> This plan governs the FPGA half of the receiver. The analog front end is fixed by
> [`cad/beacon-receiver/`](../../cad/beacon-receiver/collector-schematic.md) (the bench tester);
> the FPGA sits behind the MCP3201 ADC. Gateware source lands in
> `firmware/beacon-decoder-stepfpga/` (per [tasks.md T012c](tasks.md), now elevated to the ADC path).

---

## 0. Key finding — the installed tool is the right tool (Diamond, not Radiant)

The Windows host has **Lattice Diamond 3.14** (`C:\lscc\diamond\3.14`, valid `license.txt`) and **no Radiant**.
That is **correct for current 031**, because 031 was re-scoped to the **single-photodiode / MachXO2** phase:

| Feature | FPGA | Toolchain | Status here |
|---|---|---|---|
| **031 (current)** — 1-bit acquisition correlator | **STEP-MXO2 = Lattice MachXO2-4000HC** (CSBGA132, speed-4, Commercial — per the threeN1 synthesis logs, §3.6; confirm board label at F1) (owned) | **Diamond** | ✅ installed, licensed, MachXO2 (`ispfpga/xo2c00*`) supported |
| **040 (deferred)** — camera redo | CrossLink-NX / LIFCL-40 | Radiant + Propel | ⛔ not installed; not needed until 040 |

**Consequence:** there is *no toolchain gap* for the work in front of us. The "install Radiant" line in
[`firmware/flight-recorder/README.md`](../../firmware/flight-recorder/README.md) is a **040** concern and is
out of scope for the current correlator gateware. Radiant gets installed when (and only when) 040 restarts.

---

## 1. Dev-environment architecture — stay in WSL2, drive Diamond via interop

**Decision: develop in this WSL2 session; build on Windows Diamond through WSL interop. No SSH, no second
Windows-side Claude session, no second checkout.** Verified working this session (see §6).

### Why WSL2 over a native-Windows session
- Repo, git, the C++/Python toolchain, and `acquisition-sim/` all already live in WSL2. A native-Windows
  session would split-brain the checkout or pay `\\wsl$` UNC penalties for every git/build operation.
- WSL **interop** lets this session invoke Windows `.exe` directly — Diamond's batch CLI runs from here.
- Board programming is **STEPLink mass-storage, not JTAG** (see §3.5): the STEP-MXO2 mounts as a USB
  drive; copying the `.jed` flashes it. **WSL cannot see `D:`** (verified 2026-06-23) — the flash is a
  Windows-side `cmd.exe` copy over interop (§3.5); no USB passthrough / `usbipd` needed either way.
- Fast HDL simulation uses **open-source tools in WSL** (license-free, scriptable, repo-test-friendly).

### The one trap, and how it's sidestepped
Diamond chokes on `\\wsl.localhost\...` working directories (Windows tools reject UNC CWDs — observed this
session). **Fix:** Diamond's *build sandbox* lives on the Windows drive at `/mnt/c/fpga-build/031-stepfpga/`
(native `C:\fpga-build\031-stepfpga\` — no UNC), while **source-of-truth stays in the git repo**. A wrapper
syncs source in and copies reports/bitstream back.

```
repo (WSL, git, source-of-truth)                 Windows side (build only)
firmware/beacon-decoder-stepfpga/
  ├── rtl/      *.v          ── build.sh ──►  /mnt/c/fpga-build/031-stepfpga/   (C:\ native paths)
  ├── tb/       *_tb.v                            ├── <generated .tcl>
  ├── constraints/ *.lpf                          └── pnmainc.exe build.tcl  (synth→map→par→export)
  ├── build/    (gitignored — copied-back reports + .jed)  ◄────── copy back .jed + *.par/*.twr reports
  └── build.sh, sim.sh, README.md
```

---

## 2. The two-loop workflow

**Fast loop (seconds, WSL, no license)** — functional verification of the correlator logic:
`iverilog` + `vvp` (or Verilator) run testbenches in `tb/`. Stimulus and golden lock-vectors come from
[`acquisition-sim/sim.py`](acquisition-sim/sim.py) — the sim already models the Gold-code matched filter, so
it emits the chip/ADC stream and the expected lock decisions; the HDL testbench asserts the gateware matches.
This satisfies the constitution's testing-first principle and the plan's "Python golden vectors" approach.

**Slow loop (minutes, Windows Diamond via interop)** — synthesis, place-and-route, timing, bitstream:
`build.sh` syncs `rtl/` + `constraints/` to the `/mnt/c` sandbox, emits a Diamond Tcl script
(`prj_project new -dev LCMXO2-4000HC-4MG132C -synthesis synplify`; `prj_src add` sources;
`prj_run Export -impl impl1 -task Jedecgen` — verified working, see §3.7), runs it via `pnmainc.exe`, and
copies the `impl1/*.jed` + reports back into `build/`.

**Program (STEPLink mass-storage — NOT JTAG)** — see §3.5. The STEP-MXO2 enumerates as a USB drive
(Windows `D:\`, label **STEPLink**); **copy the `.jed` onto `D:\`** → it self-flashes + restarts. No Diamond
Programmer, no `ddtcmd`, no JTAG. WSL cannot see `D:`, so `build.sh --flash` flashes with a Windows-side
`cmd.exe /c copy /Y <jed> D:\` over interop (§3.5).

Open-source fallback (no Diamond at all): `yosys + nextpnr-machxo2 + prjtrellis` runs fully in WSL. Keep as
a backup; Diamond is primary for MachXO2 timing closure. Programming is copy-to-STEPLink either way.

### 3.5 Programming path — STEPLink copy-to-drive via a Windows-side copy

The board is **not** flashed over JTAG. When connected it presents a USB **mass-storage volume** (Windows
`D:\`, label **STEPLink**); dropping a `.jed` onto it triggers the on-board flash + a device restart — the
STEPFPGA education-kit convention.

- **WSL CANNOT see `D:` (verified 2026-06-23).** It is not auto-mounted and `sudo mount -t drvfs D: /mnt/d`
  does not surface it (STEPLink is a non-standard removable volume). The board IS connected — `D:` exists
  on the **Windows** side (`cmd.exe /c dir D:\` works) — WSL just can't reach it via drvfs.
- **So the flash is a Windows-side copy over interop**, not a `/mnt/d` write:
  `cmd.exe /c copy /Y stepfpga_impl1.jed D:\` (run with the shell CWD on `/mnt/c` so the relative filename
  resolves and there's no UNC). This is what [`build.sh --flash`](../../firmware/beacon-decoder-stepfpga/build.sh) does.
- **Prior art**: a complete Diamond-built `.jed` at
  `C:\Users\gcmcn\OneDrive\Documents\FPGA\threeN1\threeN1_threeN1.jed` was programmed by copy-to-`D:\`.

The build half of this loop is **VERIFIED end-to-end from WSL (2026-06-23)** — see §3.7 + the
[`firmware/beacon-decoder-stepfpga/`](../../firmware/beacon-decoder-stepfpga/README.md) tree. The flash
half is coded but pending a board-connected `--flash` run.

---

## 3. First gateware target — MCP3201 soft-decision correlator

Per operator direction (2026-06-22) the first RTL targets the **ADC soft-decision path**, not the optional
comparator hard-1-bit baseline — matching the bench tester in
[`cad/beacon-receiver/`](../../cad/beacon-receiver/collector-schematic.md). The comparator (`COMP_OUT`) stays
an optional later add-on.

### FPGA-facing interface (J1 "STEP-MXO2 IF", from the schematic)
The MCP3201 is a **read-only 3-wire SPI ADC** — the FPGA is SPI **master**:

| Signal | MCP3201 pin | Direction (FPGA) | Notes |
|---|---|---|---|
| `SPI_CS`  | CS/SHDN (5) | output | low = convert+shift; high = idle/shutdown |
| `SPI_CLK` | CLK (7)     | output | ≤1.6 MHz for ~100 kS/s; 16 clocks per sample |
| `SPI_DOUT`| Dout (6)    | input  | 12-bit pseudo-diff result, MSB-first after null bit |
| `+3V3`, `GND` | 8 / 4   | power  | shared rail |

The ADC reads `(TIA_OUT − VBIAS) = I_photo·R1` directly (pseudo-differential cancels the bias). DC-tap keeps
the ambient pedestal for AGC/envelope studies; AC-tap strips solar DC — jumper-selected on the board, so the
gateware must tolerate both (running-mean removal handles the DC pedestal in either case).

### Gateware blocks (MachXO2-4000HC — 4320 LUTs, 12 MHz crystal, ~1000× LUT headroom)
1. **Clock/PLL** — derive the SPI bit clock and the sample tick from the 12 MHz crystal (MachXO2 has a PLL).
2. **MCP3201 SPI master** — 16-clock frame per sample → 12-bit unsigned word at the chosen sample rate
   (baseline ~100 kS/s ⇒ ~1000× oversampling of the 200 Hz chip; sweepable).
3. **DC / AGC tracker** — running mean (IIR) → subtract → signed soft sample; track gain/scale for the
   soft-decision threshold. Handles both DC and AC coupling jumper settings.
4. **Per-beacon chip tracking (DPLL ×2)** — **one independent self-syncing chip-rate/phase loop per
   beacon** (acquisition-research-plan §5). The two emitters free-run on separate ±5% internal-RC
   oscillators (~10% apart, drifting independently of each other *and* the receiver), so each beacon's
   path searches + locks **its own** chip rate/phase and accumulates samples-per-chip on that recovered
   timebase. **Do NOT assume a shared chip clock between the two beacons** — a single shared integrator
   would alias their relative slip and break honest two-code separation. (Baseline 200 Hz; the single-PD
   bench oversamples ~1000× so the per-beacon search is cheap.)
5. **Soft-decision correlator (per beacon)** — sliding 15-chip Gold-code matched filter on **each beacon's
   own recovered timebase**, **erasure-aware** (flips cost 2, erasures cost 1 — mark saturated/faded chips
   as erasures), codes A/B (extensible to 4). Renormalize by valid-chip count.
6. **Acquisition/lock FSM (per beacon)** — tentative → confirmed lock ladder; **early/partial-code
   acquisition** at ~70% of the code (the lever that keeps re-acquisition inside the control-loop budget).
   **Independent per-beacon** lock state + locked-rate value + correlation-margin (SNR proxy) output.
7. **Telemetry** — UART @ 115200 streaming `{frame_counter, raw/decimated ADC, corr_A, corr_B, lock_state,
   margin}` to the host laptop (matches the Stage 0/1 "stream to laptop over UART" logging path).
8. **Indicators** — LED per code-locked; 7-seg chip/lock readout for at-a-glance bench status.

### What the sim provides as golden reference
[`acquisition-sim/sim.py`](acquisition-sim/sim.py) already produces the predicted curves (time-to-lock vs
fraction-of-code, erasure/flip response, two-code CDMA separation). Extend it to dump (a) a sample-stream
stimulus file and (b) expected per-chip soft values + lock decisions, consumed by the `tb/` testbench.

---

### 3.6 Confirmed process flow (from the threeN1 logs)

The Diamond UI "process" clicks are just `prj_run <Stage>` Tcl calls — captured for the known-good
**threeN1** project in [`toolchain-logs/`](toolchain-logs/) (operator prior art). This is the exact
sequence `build.sh`'s `build.tcl` drives via `pnmainc.exe`:

| Stage | Tcl call | Underlying Diamond exe(s) | Output |
|---|---|---|---|
| Translate | `prj_run Translate -impl <impl>` | `edif2ngd` (.edi→.ngo) → `ngdbuild` (.ngo→.ngd) | `.ngd` |
| Synthesis | `prj_run Synthesis -impl <impl>` | `synpwrap -prj <impl>_synplify.tcl` (Synplify Pro) | `.edi` |
| Map | `prj_run Map -impl <impl>` | `map -a MachXO2 -p LCMXO2-4000HC -t CSBGA132 -s 4 -oc Commercial …` (pins via `-lpf source/pin_assignments.lpf`) | `_map.ncd` |
| PAR | `prj_run PAR -impl <impl>` | `par` | `.ncd` |
| Export | `prj_run Export -impl <impl>` | `tmcheck` → `bitgen … -jedec` | **`.jed`** |

All exes live under `C:/lscc/diamond/3.14/…`; licensing succeeded in this run (bitgen logged
`license_securityIP OK`), so the WSL→interop invocation already has the license env it needs. The logs
also fix the device string (LCMXO2-**4000HC** / CSBGA132 / speed-4 / Commercial) used by `prj_create`.

## 4. Bring-up milestones

| # | Milestone | Done when |
|---|---|---|
| **F0** | **Scaffold** `firmware/beacon-decoder-stepfpga/{rtl,tb,constraints,build}` + README documenting the interop flow; `.gitignore` for `build/` artifacts. | Tree exists, mirrors `flight-recorder/` convention. |
| **F0.5** ✅ | **Validate `prj_run`-from-WSL** (research spike — drive Diamond from Claude Code in WSL → native Windows, source in a WSL tree). | **DONE 2026-06-23.** `pnmainc.exe` driven from WSL (CWD on `/mnt/c` → no UNC); `build.tcl` = `prj_project new` + `prj_src add` + `prj_run Export -task Jedecgen` → a valid `.jed`. License-env worked under interop. The threeN1 logs supplied device/flow/part; proven directly via the F1 blink. |
| **F1** ✅ | **Prove the loop end-to-end** with a trivial design — `build.sh` sync (`cp -u`) → `pnmainc.exe` → `impl1/*.jed` copy-back → **`cmd.exe /c copy /Y *.jed D:\` (STEPLink)** → board self-flashes + restarts → blink. | **DONE 2026-06-23.** 8-LED ripple `blink` built from WSL, LEDs LOCATEd to real sites (N13…P9, clk=C1), flashed via the STEPLink Windows-copy — **operator confirmed the board blinking**. Incremental confirmed (no-op rerun skips; 1-line change → new `.jed`). Recipe: [`firmware/beacon-decoder-stepfpga/`](../../firmware/beacon-decoder-stepfpga/README.md). |
| **F2** | **Sim harness** — `iverilog`/`vvp` installed in WSL; `sim.py` emits stimulus + golden vectors; a `*_tb.v` asserts a reference module against them; `sim.sh` runs it. | `sim.sh` passes a known-good correlator vector. |
| **F3** | **MCP3201 SPI master** + UART telemetry — clock real samples off the board, stream raw ADC to the laptop. | Live ADC envelope visible on the host over UART. |
| **F4** | **Soft-decision correlator + lock FSM** (§3 blocks 3–6), verified in sim (F2) then on hardware against a live emitter. | Lock LED tracks a real Gold-code emitter; margin/telemetry sane vs sim predictions. |
| **F5** | **Two-code CDMA + early acquisition** sweeps — feed Stage 0/1 of the research arc. | A/B separated on one detector; partial-code lock measured vs `sim.py`. |

F1 is the critical de-risk: it isolates "can WSL drive Diamond and flash the board" from "is my correlator
correct," so a toolchain problem never masquerades as a logic bug.

---

## 5. Open decisions

1. **DC vs AC coupling default for bench** — board jumper-selects; gateware tolerates both via running-mean
   removal. Pick DC for the bench / AC for daylight field (matches schematic note 2)?
2. **Sample rate / oversampling** — baseline 100 kS/s (MCP3201) ⇒ ~1000× at 200 Hz. ADS7042 @ 1 MS/s only
   if finer oversampling sweeps are wanted (BOM already notes this fork).
3. **Telemetry framing** — raw-ADC firehose vs decimated soft-chips vs lock-events-only. Suggest a runtime
   mode register so one bitstream serves all three (raw for first light, soft-chips for tuning, events for
   long runs).
4. **Open-source path** — keep `yosys+nextpnr-machxo2` as a documented fallback, or wire it up in parallel
   from the start? (Recommend: document only; Diamond primary.)

---

## 6. Verified this session (2026-06-22; STEPLink + prior-art added 2026-06-23)

- `C:\lscc\diamond\3.14` present; `license/license.txt` present; **no Radiant** anywhere.
- Diamond MachXO2 device support present (`ispfpga/xo2c00`, `xo2c00a`, `xo2c00ap`, `xo2c00p`).
- WSL **interop works**: ran `pnmainc.exe <script.tcl>` from WSL → executed Tcl, exit 0, CWD reported as
  native `C:/...`. This is the load-bearing capability — Diamond's batch flow is drivable from this session.
- `/mnt/c` is **writable** from WSL (build sandbox viable).
- No Windows `sshd.exe` (irrelevant — interop replaces it).
- Receiver FPGA interface confirmed from [`beacon-receiver-schematic.pdf`](../../cad/beacon-receiver/beacon-receiver-schematic.pdf):
  J1 = `{+3V3, GND, SPI_CS, SPI_CLK, SPI_DOUT}` to the MCP3201 (read-only 3-wire SPI).
- **(2026-06-23) FULL loop verified from WSL** — see §3.7. `cmd.exe` interop works; `pnmainc.exe` builds a
  `.jed` (CWD on `/mnt/c` to dodge UNC); **`D:` is NOT visible to WSL**, so the flash is a Windows-side
  `cmd.exe /c copy /Y <jed> D:\`. An 8-LED-ripple `blink` was built + flashed end-to-end and the operator
  confirmed the board blinking.

## 3.7 Proven build/flash recipe (VERIFIED 2026-06-23)

The make-or-break questions are answered — the whole loop runs from WSL with **no GUI, no JTAG**:

1. **Chain-out**: WSL invokes `pnmainc.exe` (and `cmd.exe`) directly via binfmt interop. ✅
2. **No-UNC**: run with the shell CWD on the `/mnt/c` sandbox so Windows sees `C:\…`, not `\\wsl$`. ✅
3. **Build**: `prj_project new -dev LCMXO2-4000HC-4MG132C -synthesis synplify` → `prj_src add` → `prj_impl
   option top` → **`prj_run Export -impl impl1 -task Jedecgen`** (Jedecgen, *not* Bitgen → `.bit`). Outputs
   land in `impl1/`. ✅
4. **Incremental**: `cp -u` source sync (not `cp -f`) keeps Diamond's mtime tracking — no-op rerun skips
   ("Nothing is executed…"); a 1-line RTL change re-runs the chain and changes the `.jed`. ✅
5. **Flash**: `cmd.exe /c copy /Y <jed> D:\` (WSL can't see `D:`). ✅ — operator-confirmed board blink.

Source-of-truth + the runnable `build.sh`/`build.tcl` live in
[`firmware/beacon-decoder-stepfpga/`](../../firmware/beacon-decoder-stepfpga/README.md). Remaining nuance:
a 2nd `.lpf` added via `prj_src` is auto-excluded, so `build.tcl` injects the constraints into the active
`<proj>.lpf` (only when changed, to preserve incremental).
