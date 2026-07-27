# HDL simulation harness (iverilog)

WSL-native functional simulation of the correlator gateware — **no Diamond / Windows / board needed**. This is
the counterpart to the on-FPGA validation in [`../correlator-sim-plan.md`](../correlator-sim-plan.md): when the
board isn't in hand (or for fast regression of a datapath change), simulate the *actual* RTL here.

## Run it

```bash
./run.sh          # compile + run the s6 testbench, print PASS/FAIL
./run.sh --gtk    # also open the VCD in gtkwave (if installed)
```

Expected tail:

```
code A : LOCK    (q0=9 ...)   lock@ ~3.1 ms
code B : LOCK    (q1=9 ...)   lock@ ~6.2 ms
inject : K1/K2 drove code-A margin down to q0=8 (from 9) — injector path WORKS
RESULT: PASS  (RAM windows decode A+B; injectors modulate margin)
```

## What it proves

- **Code A locks** off the digital inject → the **circular-buffer RAM windows** (`wp`/`rp`/`rp0`, s6) reproduce
  the old shift-register behavior. This is the reason the harness exists: verifying that datapath change without
  the board.
- **Code B locks** off an emulated MCP3201 (the bit-exact `mcp3201_model` driven by a free-running CODE1
  stimulus) → the real-ADC read path + the per-code DPLL tracking an independent clock.
- **K1/K2 injectors** modulate code-A margin → the error-injection path is wired.

## How it's built

| File | Role |
|---|---|
| `tb_s6.v` | testbench: drives `clk12`, instantiates `s6_top`, hangs `mcp3201_model` on the SPI bus as the "real" ADC, watches `dut.st0/st1` (lock FSMs), runs an acquire → code-B → inject → recover script |
| `osch_stub.v` | behavioral stand-in for the Lattice **OSCH** hard oscillator (iverilog can't elaborate the primitive). SIM-only. |
| `run.sh` | `iverilog -g2012 -DSIM …` + `vvp` |

Sources pulled from `../experiments/`: `s6.v`, `spi_mcp3201.v` (`spi_mcp3201_reader` + `mcp3201_model`),
`uart_bcn.v` (`bcn_tx` + `uart_rx`).

## `+define+SIM` time-scaling (the key trick)

At real rates a code period is ~1.85 M `clk12` cycles, so a lock is ~15 M cycles — too slow to iterate on.
`s6.v` guards its clock dividers with `` `ifdef SIM `` and scales them **÷100, keeping their ratio**, so the
decode datapath (samples/chip = 2.4, window `L`=74, DPLL slip math) is **bit-identical** while a lock lands in
a few ms of sim time:

| Constant | Synth (`ifndef SIM`) | Sim (`-DSIM`) |
|---|---|---|
| `EDIV_NOM` (emitter chip rate) | 266000 (200 Hz) | 2660 (20 kHz) |
| `FDIV` (ADC sample rate) | 25000 (480 Hz) | 250 (48 kHz) |
| `SCLK_HALF` (SPI clock) | 120 (50 kHz) | 2 (3 MHz) |

`SCLK_HALF` scales too so one SPI frame (≈16·2·`SCLK_HALF`) still fits inside a sample tick (`< FDIV`). The
Diamond build defines nothing → uses the real values → the synthesized bitstream is unchanged (verified: 1338
slices identical with/without the `` `ifdef `` present).

## What the testbench already covers (`tb_s6.v`)

A single scripted run: acquire A → bring up code-B stimulus → inject K1/K2 errors → recover → **+5% code-B
clock skew** (DPLL must re-track, lock holds) → **1/4 code-B amplitude** (AGC, lock holds). Adjustable knobs in
the TB: `tchip` (code-B chip period → skew) and `ampB` (code-B swing → attenuation).

## Extending

- Skew/DPLL: change `tchip` (or send `'E'` over `rxd`) and assert lock holds / `rateB` shifts.
- Burst dropout: send `'K'`+span over `rxd` and assert HOLD + flywheel coast.
- New DUT (s7…): copy `tb_s6.v` → `tb_s7.v`, retarget the top + `run.sh`.
- **Bit-exact golden cross-check (deferred):** the existing Python models are *not* bit-comparable —
  `a4d_model.py` is an analytical link-budget model and `sim.py` is a chip-level bipolar Monte-Carlo (argmax
  over phases), both at a different abstraction than the RTL's oversampled, DC-removed, soft-decision,
  Leff-Bresenham datapath. A true numerical cross-check needs a **bit-exact Python twin of the RTL datapath**
  (DC removal, signed accumulation, Leff chip-advance) dumping `corr0/corr1` per window — a separate task; lock
  correctness is currently proven behaviorally (sim + hardware, across inject/skew/weak).
