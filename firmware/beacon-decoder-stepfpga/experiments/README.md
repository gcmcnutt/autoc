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

## Takeaway for the correlator
- The toolchain **does** surface both edges loudly: a one-line max-Fmax + level-count for timing, a
  per-resource %/over-capacity table for fit. Build through `prj_run PAR` (no flash) is enough to get both.
- When the correlator is written, gate it with a `FREQUENCY` constraint at the real control/sample clock and
  watch Levels + Fmax; watch LUT/carry/EBR % for the two-DPLL fit question.
