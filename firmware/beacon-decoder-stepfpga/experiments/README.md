# experiments/ — timing + utilization stress (does the toolchain give us edge-of-envelope feedback?)

A research exercise (not flown) to confirm Diamond's reports give actionable feedback **before** the real
correlator pushes the MachXO2-4000HC (4320 LUTs) into the timing-critical / near-full regime — e.g. the open
question of whether two independent per-beacon DPLLs + correlators fit (acquisition-research-plan §5, fpga
§3 "FPGA fit").

- [`stress.v`](stress.v) — `LANES` parallel copies of a `DEPTH`-deep serial 32-bit add/rotate/xor chain,
  XOR-reduced to one registered bit (nothing trims away). `DEPTH` → combinational-path length (**timing**);
  `LANES × DEPTH` → LUT count (**utilization**). MachXO2 has no hard multipliers, so it's all fabric.
- [`stress_pins.lpf`](stress_pins.lpf) — a `FREQUENCY` constraint on `clk` to drive trce.
- [`run_stress.sh`](run_stress.sh) → `stress_build.tcl` — build through PAR+trce (no flash); dumps the
  `.mrp` (utilization) and `.twr` (timing) summaries. Runs over the same proven WSL→Diamond interop loop.

## Findings (2026-06-23)

### Utilization feedback (.mrp "Design Summary") — clear, per-resource
| Design | LUT4s | SLICEs | result |
|---|---|---|---|
| LANES=14, DEPTH=8 (112 stages) | **13049 / 4320 = 302%** | 6525/2160 = 302% | **over capacity** — map flags 3× oversize; won't place |
| LANES=4, DEPTH=8 (32 stages) | **3683 / 4320 = 85%** | 1842/2160 = 85% (carry 52%) | fits near the edge; places + routes |

The report breaks out every resource class — LUT4s, SLICEs (logic/RAM/carry), PFU/PIO registers, **block
RAMs 0/10, PLLs 0/2, DCCA/CLKDIV/…** — so when the correlator lands we'll see exactly which resource binds
first (likely LUTs/carry for the matched filters, EBR for sample buffers).

### Timing feedback (.twr) — Fmax, logic levels, slack, met/not-met
| `FREQUENCY clk` constraint | Actual (max) | Levels | result |
|---|---|---|---|
| **400 MHz** (gross over-ask) | **12.675 MHz** | **59** | **not met** — 4096 setup errors; score 3.1e8 (huge −slack) |
| **12.6 MHz** (≈ knee) | **13.351 MHz** | **45** | **met (marginal)** — 0 errors, "All preferences were met", ~0.75 MHz (~6%) margin |

trce names the **maximum achievable frequency** for the constraint, the **critical-path logic-level count**,
the **number of failing paths**, and a cumulative-negative-slack score. That's exactly the feedback to tune
against: lower the ask toward the reported Fmax, or pipeline to cut the level count. (`400 MHz` isn't
reachable on this fabric — the *point* was to see the report, not hit it; a real high-speed clock comes from
the sysCLOCK PLL.)

**Subtlety worth knowing:** the reported max-Fmax is **placement-dependent, not absolute** — 12.675 MHz under
the 400 MHz ask (59 levels) vs 13.351 MHz under the 12.6 MHz ask (45 levels). Timing-driven PAR optimizes the
critical path *toward the active constraint*, so always read Fmax against the constraint you actually intend,
and re-check after constraint changes.

## Pipelined 32×32 multiplier — how fast? (100 MHz yes, 200 MHz no)

`mul.v` (pipelined multiply, retiming-enabled in `mul_build.tcl`) + `ceiling.v` (fabric-ceiling probe),
swept by `run_mul.sh`. MachXO2 has **no hard multipliers**, so a 32×32 multiply is pure fabric.

| Design | Fmax | bound by |
|---|---|---|
| Combinational 32×32 multiply (`STAGES=1`) | **~33 MHz** | carry/ripple propagation through the partial-product adder tree + final ~64-bit CPA |
| `a*b` + N output pipeline regs + Synplify retiming (`STAGES=8/16/32`) | **~27–34 MHz** (flat) | retiming did **not** restructure it — the soft multiplier is one blob; the output shift-register gives retiming no logic to move *between* stages |
| Deep 1-LUT-level, no-carry pipeline (`ceiling.v`, 40 stages) | **~150 MHz** | raw fabric ceiling (FF + 1 LUT + routing) on this -4 part |

**Conclusions:**
- The combinational-multiply ceiling (~30 MHz) is **carry-propagation bound**, not register-bound — adding
  *output* registers (even with `-retiming true`) doesn't help, because the registers sit after the blob.
- The **fabric ceiling is ~150 MHz** here. So **100 MHz is reachable** for a multiply **only if** it is
  **structurally carry-save pipelined** (3:2 compressors, ≤~1.5 LUT levels/stage, ripple broken, register
  per row + a pipelined final CPA). **200 MHz is not reachable** on this part at any depth.
- For genuine high-speed multiply, use **hard DSP blocks** — the MachXO2 has none; the **040 CrossLink-NX
  does** (hundreds of MHz). On MachXO2, budget the correlator around the ~150 MHz fabric ceiling and keep
  arithmetic stages shallow / carry-save.

## Carry-save multiplier — 100 MHz PROVEN (+ datasheet cross-check)

`csa.v` / `run_csa.sh`: structural carry-save pipelined WxW multiplier (3:2 compressors, register every
GROUP rows, final carry-propagate add). Measured (trce, worst-case, harness fixed — see below):

| config | Fmax | LUT | REG |
|---|---|---|---|
| **W=32, GROUP=1** | **125 MHz** | 2330 (54%) | 2499 (54%) |
| W=16, GROUP=1 | ~125 MHz | 535 (12%) | 698 (15%) |

**100 MHz is proven** for a true 32×32 multiply; ~125 MHz achieved, approaching the ~150 MHz fabric ceiling.

**Deployed + run on hardware (2026-06-23):** `selftest.v` clocks the CSA 32×32 multiplier from the
**PLL at 108 MHz** (`fast_pll.v`, 12→108 MHz; production-style), streams LFSR operands, and compares each
product to a golden `a*b` reference, counting mismatches (`deploy_selftest.sh` → flash). Result on the
STEP-MXO2: PLL **locked**, **zero errors** — the multiplier computes correctly at 108 MHz on real silicon.
**Also hardware-verified at 132 MHz** (PLL retuned `CLKFB_DIV` 9→11, 12×11=132 MHz; VCO 660): ripple clean,
**zero errors** — beyond the 125 MHz worst-case bench figure (real silicon beats worst-case timing, and Fmax
floats with placement). To probe the actual failure edge, push to 144/156 MHz (`CLKFB_DIV` 12/13, VCO ≤ 800).
_Gotcha: a `timeout`-killed `pnmainc` leaves orphaned Windows `par.exe`/etc. holding file locks → next build
can't clean the sandbox. Fix: `cmd.exe /c taskkill /F /IM par.exe …` + a fresh sandbox dir (or don't timeout)._
(Note: the board LEDs are **active-low**; the display encodes status as motion/blink to stay polarity-robust
— ripple = running-clean, all-blink = error. The combinational `a*b` reference is a multicycle path that
settles during the operand hold-window — trce flags it "not met" but it's functionally correct; the CSA
multiplier's own 108 MHz domain meets timing, internal max 150 MHz.)

**Measurement gotcha (important):** the first CSA runs read 80–116 MHz — but the critical path was the **test
harness**, not the multiplier: a single `led <= ^prod` (64→1 XOR) was 8.6 ns / **77% routing** (the wide fan-in
routes across the chip). Pipelining that reduction (64→32→8→1) revealed the multiplier's true 125 MHz. Lesson:
a wide output reduction can mask the DUT — pipeline/narrow it.

**Datasheet cross-check** (MachXO2 Family Data Sheet p55–56, Register-to-Register Performance, HC/HE; trce
reports the *worst-case* column):

| block | typical (p55) | worst-case (p56) | our match |
|---|---|---|---|
| 16-bit adder | 297 MHz | **134 MHz** | W=16 CSA ≈ 125 MHz |
| 64-bit counter | 161 MHz | **77 MHz** | the masked "80 MHz" CPA path |
| 16-bit counter | 324 MHz | 148 MHz | fabric ceiling ≈ 150 MHz |
| 16:1 MUX | 412 MHz | 191 MHz | — |

Our empirical numbers line up with the datasheet's worst-case reg-to-reg figures — wide carry paths (64-bit)
sit ~77 MHz, shallow logic ~134–150 MHz. The carry-save structure keeps the ripple short (only the final
add), which is why a 32×32 CSA mult (125 MHz) beats a naive 64-bit ripple (~77 MHz).

**Resource reality (the "2–4 correlators adds up fast" question):** one 32×32 CSA multiply ≈ **54%** of the
4320-LUT part — so **only one full 32×32 fits**. A 16×16 is ~12% (several fit). For 2–4 correlators at 480 fps:
keep multiplies narrow (correlation is MAC of small values), time-multiplex one multiplier, or lean on the
**CrossLink-NX hard DSP blocks** for the 040 video channel. To push a 32×32 toward 150 MHz, pipeline the final
CPA into ~16-bit chunks (134 MHz/chunk per the datasheet).

## Takeaway for the correlator
- The toolchain **does** surface both edges loudly: a one-line max-Fmax + level-count for timing, a
  per-resource %/over-capacity table for fit. Build through `prj_run PAR` (no flash) is enough to get both.
- When the correlator is written, gate it with a `FREQUENCY` constraint at the real control/sample clock and
  watch Levels + Fmax; watch LUT/carry/EBR % for the two-DPLL fit question.
- **Throughput vs replication:** the CSA multiplier is *fully pipelined* (1 result/cycle, latency ≈ depth);
  a too-slow final CPA is *pipelined* (split into ~16-bit chunks), not duplicated — throughput stays 1/cycle.
  Carry a `valid` bit down a shift register of the same latency; an FSM is only for operand scheduling.
- **Time-multiplex, don't replicate:** data rate is ~kHz (480 fps × oversampling), the multiplier is ~125 MHz
  → hundreds of multiply-slots per sample. **One** pipelined multiplier, time-shared, serves all taps of all
  correlators (MAC into per-correlator accumulators). Provision one (or a narrow one), not 2–4 — which
  dissolves the "54% of the part per multiply" worry. Replicate only when you run out of cycles/sample (≈3
  orders of magnitude of headroom here).
