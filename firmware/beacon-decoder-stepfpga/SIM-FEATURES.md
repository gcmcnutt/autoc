# Correlator-Sim — FPGA feature reference

The **in-FPGA, soup-to-nuts correlator simulation harness** on the STEP-MXO2 (MachXO2-4000HC-4MG132C). It
generates coded beacons, models the optical/analog channel, samples it through a real-protocol ADC + SPI, runs
the actual correlator (the DUT), and exposes everything on the board's I/O + a bidirectional USB link — so the
beacon receiver can be developed and stress-tested with **no analog hardware**.

- **Current build:** `experiments/s3.v` (top `s3_top`) + `spi_mcp3201.v` + `uart_bcn.v` — milestone **S6 + DPLL**
  (two emitters + noise + DIP/USB control + per-code clock-estimation loop).
- **Resource:** **2075 / 4320 LUT4 (48 %)**, all timing met.
- **Plan / roadmap:** [`correlator-sim-plan.md`](correlator-sim-plan.md) · **research/study:**
  [`specs/031-beacon-camera/tasks.md` §A4d](../../specs/031-beacon-camera/tasks.md) · **predictive model:**
  [`specs/031-beacon-camera/acquisition-sim/a4d_model.py`](../../specs/031-beacon-camera/acquisition-sim/a4d_model.py)

---

## 1. Signal chain

```
 OSCH 53.2 MHz (RC, ±5%, unsync)                12 MHz xtal (receiver domain, precise)
   │                                                     │
   ├─ emitter A: Gold CODE0 @ 200 Hz chip  ─┐            │
   ├─ emitter B: Gold CODE1 @ ~194/206 Hz  ─┤  CDC       ▼
   │            (skewed divisor, ±3%)        ├──────► analog model:  sum(enabled) → band-limit (ramp)
   └─ random bit-error injection (A & B)    ─┘            │           + white noise + DC floor → clamp 12-bit
                                                          ▼
                                          virtual MCP3201 (bit-exact) ── soft SPI master (50 kHz, 480 Hz/sample)
                                                          │  CS/SCLK/DOUT mirrored to J1 (P3/M4/N4) for scope
                                                          ▼
                                          DUAL correlator (DUT): 36-sample matched filter ×2 (CODE0, CODE1)
                                          DC-removal + AGC ratio + per-code min-lock/limited-hold FSM
                                                          │
                       ┌──────────────────┬──────────────┼───────────────────┬─────────────────┐
                       ▼                  ▼               ▼                   ▼                 ▼
                   8 LEDs (q bars)   RGB L/R (lock)   7-seg (quality)   P8/N8 (scope)   UART BCN @40 Hz → COM3
```

---

## 2. Clocking — independent & unsynchronized (by design)

| Domain | Source | Rate | Role |
|---|---|---|---|
| Emitters | internal **OSCH** | 53.2 MHz (RC, **±5 %**) | beacon chip clocks — *not* synced to the receiver |
| Receiver / correlator | **12 MHz crystal** (C1) | 12 MHz | sampling, SPI, correlator, telemetry |

Both emitters share the OSCH (so their relative offset is the clean divisor ratio), but the OSCH-vs-xtal
mismatch makes each emitter **slip steadily** against the sample clock — the real NFR-4 drift. Scope the SPI
CS (P3) against the emitter epoch (N8) to see the slip.

---

## 3. Sources (summed into the one virtual ADC)

| Src | Code / model | Chip clock | Enable |
|---|---|---|---|
| **A** | Gold **CODE0** = `000001101111011` (15-bit) | OSCH / 266000 = **200 Hz** | DIP1 / cmd[0] |
| **B** | Gold **CODE1** = `110011100000001` (15-bit) | OSCH / EDIV_B = **~194 Hz (−3 %)** or **~206 Hz (+3 %)** (DIP4) | DIP2 / cmd[1] |
| **N** | per-sample white noise (LFSR, ~±512) | — | DIP3 / cmd[2] |

- **2.4 samples per chip** at 480 Hz sampling (the camera-frame cadence; 200 Hz chip × 2.4 = 480).
- **Bit-error injection** flips **1 (K1)** or **2 (K2)** random chips per code period — independent positions on
  **both** A and B (LFSR-drawn each emitter's epoch).
- Emitter A's (post-corruption) code → **P8**, its clean epoch → **N8** (scope).

---

## 4. Analog front end (`adc_level`)

`adc = clamp₁₂( bandlimit( PED + ampA·(±A) + ampB·(±B) ) + noise + floor )`

- **Pedestal** 1536; **signal amplitude** ±600 (±200 when *weak*); **noise** ±512; **DC floor** +400.
- **Band-limit (ramp):** 1-pole IIR LPF (`LPF_SH=9`, run at clk12/8 ≈ 1.5 MHz) — models the PD/TIA bandwidth so
  chip edges **ramp** (edge samples land mid-ramp), not ideal squares.
- **Clamp** to 12-bit (0–4095) — realistic ADC saturation under stacked sources.

---

## 5. Virtual MCP3201 + soft SPI master (`spi_mcp3201.v`)

- **Bit-exact MCP3201 model:** sample-and-hold on CS↓, ~1.5-clk window, null bit, B11→B0 MSB-first, LSB-repeat
  if over-clocked; edge-driven (tolerates a paused clock → validates an 8-bit hard-IP two-byte read later).
- **Soft SPI master/reader:** 16-clock frame, `SCLK = 50 kHz` (inside the MCP3201 10 kHz–1.6 MHz charge-valid
  window), CPOL=0; 12-bit code = `raw[12:1]` (standard 2-byte-driver alignment).
- **Sample rate 480 Hz** (fetcher-gated) — the camera cadence; SCLK only dequeues each frame.
- **Scope mirror:** CS→**P3**, SCLK→**M4**, DOUT→**N4** (real J1 pins).
- **Verified** by `experiments/mcp3201_test.v` — an LFSR-injected decode self-test (walking-dot = PASS).

---

## 6. Correlator (the DUT) — dual-code

- **Matched filter ×2:** a 36-sample sliding window correlated against CODE0 and CODE1 (upsampled ±1 template,
  36→15 slot map). Signed-accumulate (no multiplier needed for a ±1 code); shared window/DC/energy.
- **DC-removal:** slow IIR mean (τ ≈ 256 samples) subtracted before correlation.
- **AGC:** quality `q = |corr| / energy` → **signal-level independent** 0–9 match ratio. `|corr|` makes code
  bit-order & polarity irrelevant. (Weak signal / raised floor therefore stay locked.)
- **Per-code lock FSM:** SEARCH → ACQUIRING (`MINLOCK=2` consecutive good periods to CONFIRM) → LOCKED → HOLD
  (coast `HOLDMAX=2` bad periods) → re-acquire. `GOOD=6` of 9 (above the Gold cross-corr floor ~5, below a
  1-bit-error level ~7).
- **Discrimination:** only the emitted code locks; the other sits at its true cross-corr/noise level → live
  CDMA separation when both A and B are on.
- **Per-code DPLL (clock estimation + flywheel):** an IIR mean of the per-period peak-phase slip estimates each
  beacon's chip **rate** vs the precise xtal. Updates only while LOCKED → **frozen (held) through outages = the
  frequency flywheel**. Reported in telemetry (`rateA`/`rateB`, offset-binary). HW-verified: tracks a commanded
  emitter-B sweep 191→206 Hz monotonically (~3–5 Hz / calibration-grade); A↔B cross-pull visible at N=15.

---

## 7. Controls

### DIP switches (local demo) — ⚠️ "**ON**" label = **mute** (source on when switch **OFF**)
| DIP | Pin | Function |
|---|---|---|
| DIP1 | M7 | enable **code A** |
| DIP2 | M8 | enable **code B** |
| DIP3 | M9 | enable **noise** |
| DIP4 | M10 | code-B clock **skew** (−3 % / +3 %) |

### Momentary buttons (active-low)
| K | Pin | Function |
|---|---|---|
| K1 | L14 | inject **1-bit** error (both codes) |
| K2 | M13 | inject **2-bit** error (both codes) |
| K3 | M14 | **ceiling** down — weak signal (amplitude → ⅓) |
| K4 | N14 | **floor** up — raise DC baseline |

### USB command override (richer; supersedes the switches)
- `+` (0x2B) → **REMOTE** (USB owns; switches ignored) — shown as a **blue tint** on both RGB LEDs.
- `-` (0x2D) → **LOCAL** (switches resume).
- `0x80 | mask` → 7-bit knob set: `[0]enA [1]enB [2]enN [3]inj-1bit [4]inj-2bit [5]weak [6]floor`.
- `F`(0x46) + value → set **emitter-B frequency** (~±10 % over the byte; value 128 = nominal) — for DPLL/rate testing.
- Extensible to further parametric commands (per-source magnitude, skew, SNR) later.
- Driver: `host/cmd_read.sh <mask> [sec]` / `cmd_read.ps1 -Mask -Freq -Local`.

---

## 8. Displays

| Output | Pins | Meaning |
|---|---|---|
| **8 LEDs** | N13,M12,P12,M11,P11,N10,N9,P9 | q-bars: left nibble = code-A quality, right nibble = code-B |
| **RGB left** | M2,N2,P2 | code-A lock: red=search · yellow=acquiring · green=locked · green-blink=hold · +blue=REMOTE |
| **RGB right** | M3,N3,P4 | code-B lock (same scheme) |
| **7-seg d1** | A11,B12,H2,H1,J1,B14,C12 (+enableLd1=C9) | code-A quality **0–9** (threeN1 segment map, active-high, enable active-low) |
| **7-seg d2** | B9,A9,E2,E1,F2,C11,A10 (+enableLd2=A12) | code-B quality **0–9** |

---

## 9. Telemetry + commands (USB-CDC, no usbipd — board stays on Windows)

- **TX** `BCN` frame on **A2** → STEPLink LPC → **COM3**, 115200 8N1, **~40 Hz** (control-loop family):
  `BCN,<seq>,<adc>,<corrA>,<lockA>,<marginA>,<corrB>,<lockB>,<marginB>,<rateA>,<rateB>\n`
  (`lock` 0=no/1=tentative/2=confirmed; `margin` = the 0–9 quality; `rate` = DPLL rate, offset-binary →
  `chip_rate_hz()` in the host parser).
- **RX** commands on **A3**.
- **Host tools** (`host/`): `monitor.sh` (read/log), `beacon_telemetry/` (parser + tests), `cmd_read.sh`
  (command + read), `transition.sh` (capture a lock edge for LOS/acquisition timing).

---

## 10. Pin map (full)

| Signal | Pin | | Signal | Pin |
|---|---|---|---|---|
| clk12 (xtal) | C1 | | spi_cs (scope) | P3 |
| DIP1 enA / sw1 | M7 | | spi_sclk (scope) | M4 |
| DIP2 enB | M8 | | spi_do (scope) | N4 |
| DIP3 enN | M9 | | code A → scope | P8 (I/O14) |
| DIP4 B-skew | M10 | | epoch A → scope | N8 (I/O15) |
| K1 1-bit | L14 | | UART TX (BCN) | A2 |
| K2 2-bit | M13 | | UART RX (cmd) | A3 |
| K3 ceiling | M14 | | 8 LEDs | N13,M12,P12,M11,P11,N10,N9,P9 |
| K4 floor | N14 | | RGB L / R | M2,N2,P2 / M3,N3,P4 |
| 7-seg d1 (+en) | A11,B12,H2,H1,J1,B14,C12 (C9) | | 7-seg d2 (+en) | B9,A9,E2,E1,F2,C11,A10 (A12) |

All switch/button inputs `PULLMODE=UP`, `LVCMOS33`.

---

## 11. Measured baselines (N=15, telemetry @40 Hz)

| Metric | Value |
|---|---|
| Acquire (cold → confirmed) | ~200 ms (MINLOCK=2) |
| LOS (signal → lock=0), clean | ~200 ms |
| LOS with noise | ~300 ms |
| Pure-noise false-alarm (green) | ~0 / 10 s (q floor ≈3; MINLOCK rejects it) |
| A+B (CDMA) | both confirm ≥95 % |
| A+B + noise | both ~75 % confirmed (wrong-channel locks appear) |
| A+B + 2-bit (both) | both q≈6 / ~75 % |

These are the N=15 numbers the **A4d code-length study** (N=31/63) aims to improve — see the predictive model
(`a4d_model.py` / `a4d-model-results.md`).

---

## 12. Build / flash

- **Build+flash:** `experiments/deploy_s3.sh` — WSL → Diamond (`pnmainc.exe`, CWD on `/mnt/c`) → `.jed` copied
  onto the STEPLink `D:` volume (`cmd.exe /c copy /Y … D:\`). No JTAG, no usbipd.
- **Read telemetry:** `host/monitor.sh COM3 5` (board stays on Windows the whole time).

---

## 13. Not yet implemented (roadmap)

- **Per-beacon DPLL** + **frequency flywheel** (hold rate ~10 s through outages → phase-only re-acquire) — the
  highest-value next RTL (A4c / A4d-2).
- **Partial / progressive correlator** — ½-code-word candidate detection (low-latency acquire tier).
- **Code-length parametrization** (N=31/63) — the A4d-1 sweep.
- **Richer parametric commands** (per-source magnitude, skew, SNR) over the existing RX path.
- **Hard EFB SPI** (deferred to backlog; soft SPI is sufficient — EFB likelier repurposed for SD-card writes).
- **7-seg decimal-point** REMOTE indicator (needs a DP pin; using blue-RGB tint for now).

---

## 14. File index

| File | Role |
|---|---|
| `experiments/s3.v` | top — emitters, analog model, SPI, dual correlator, displays, telemetry, command RX |
| `experiments/spi_mcp3201.v` | `mcp3201_model` (bit-exact slave) + `spi_mcp3201_reader` (soft master) |
| `experiments/uart_bcn.v` | `uart_tx` + `bcn_tx` (telemetry) + `uart_rx` (commands) |
| `experiments/mcp3201_test.v` | SPI decode self-test (LFSR-injected, walking-dot PASS) |
| `experiments/s3_pins.lpf`, `s3_build.tcl`, `deploy_s3.sh` | constraints, Diamond build, build+flash |
| `host/monitor.sh`, `cmd_read.sh`, `transition.sh`, `beacon_telemetry/` | telemetry read, commands, edge capture, parser |
