# 031 — Emitter Toolchain & Bring-Up Plan (ATtiny412 beacon-pod)

**Status**: DRAFT, 2026-06-22
**Scope**: how we build + flash the **beacon-emitter firmware** (the ATtiny412 that clocks the Gold code
onto the LED driver's DIM pin per [acquisition-research-plan.md](acquisition-research-plan.md) §5 + the
emitter [`cad/beacon-eval/`](../../cad/beacon-eval/verified-bom-eval.md)). Companion to the
[FPGA toolchain plan](fpga-toolchain-plan.md) — the two halves of the 1-bit receiver/emitter pair.

> **Where the novelty is.** The emitter toolchain is **WSL-native and familiar** — it's the same shape as
> [`../../xiao/`](../../xiao/) (PlatformIO) and the avr-gcc path is already built and working. The
> genuinely-new/external toolchain for 031 is the **FPGA** one (Windows-side Lattice Diamond, *outside*
> this VSCode/WSL session — see [fpga-toolchain-plan.md](fpga-toolchain-plan.md)). This doc exists so the
> emitter's PlatformIO option is a real, evaluable path, not a hand-wave.

---

## 0. The MCU and why it shapes the toolchain choice

ATtiny412 (tinyAVR 0-series): **4 KB flash, 256 B SRAM, 20/16 MHz internal oscillator (no crystal)**,
SOIC-8 / SOT-23, single-wire **UPDI** programming. The flight firmware is tiny and timing-critical: a
200 Hz timer ISR walking a 15-chip Gold-code LUT onto one GPIO (the DIM line), plus the FR-1.7 UVLO
(ADC bandgap sample + sleep) and WDT. Internal-RC clock accuracy is the NFR-4 concern, not a toolchain one.

That profile (4 KB part, deterministic ISR, no crystal) is why **bare-C is the v1 build** and **PlatformIO
is the ergonomic option**, not the other way round.

## 1. Path A — avr-gcc + serialUPDI (COMMITTED, works today)

Bare-C, no Arduino core. Already implemented:
[`firmware/beacon-pod/Makefile`](../../firmware/beacon-pod/Makefile) + one-time setup in
[`firmware/beacon-pod/SETUP.md`](../../firmware/beacon-pod/SETUP.md).

```bash
cd firmware/beacon-pod/
make CODE_ID=0            # builds beacon-pod.hex (bare avr-gcc, -Os)
make flash CODE_ID=0      # pymcuprog write -t uart -u /dev/ttyUSB0 (serialUPDI)
```

- Toolchain: `gcc-avr avr-libc binutils-avr` (apt) + `pymcuprog` (pip) + a USB-UART with a 4.7 kΩ
  TX↔UPDI pull-up (the "serialUPDI" trick), or an ATtiny412 Curiosity Nano (`/dev/ttyACM0`, built-in UPDI).
- **Pros**: minimal flash (no core overhead on a 4 KB part), deterministic ISR timing, zero framework
  dependency, already validated. **This is the v1 path.**

## 2. Path B — PlatformIO (the "later option", xiao-style)

For IDE integration, serial monitor, library ecosystem, and a one-command build/upload that matches the
[`../../xiao/`](../../xiao/) workflow. tinyAVR is supported by the official **`atmelmegaavr`** platform with
**megaTinyCore** (SpenceKonde's Arduino core for tinyAVR 0/1/2-series). Drop-in `platformio.ini`:

```ini
; firmware/beacon-pod/platformio.ini  (Path B — optional; mirrors xiao/platformio.ini conventions)
[env:ATtiny412]
platform        = atmelmegaavr
board           = ATtiny412
; framework     = arduino           ; megaTinyCore — DELIBERATELY NOT USED (Arduino/OS rejected, see §3).
                                    ; bare-C: no framework — PIO wraps the existing avr-gcc sources.
board_build.f_cpu       = 20000000L
board_hardware.oscillator = internal ; no crystal — matches the part + NFR-4 internal-RC analysis
upload_protocol = serialupdi        ; pymcuprog under the hood — same cable as Path A
upload_port     = /dev/ttyUSB0      ; /dev/ttyACM0 for a Curiosity Nano
upload_speed    = 230400
build_flags     = -DCODE_ID=0       ; pod A; CODE_ID=1 for pod B (parallels the Makefile var)
monitor_speed   = 115200            ; diagnostic UART if the firmware exposes one
```

```bash
cd firmware/beacon-pod/
pio run                       # build
pio run -t upload             # flash via serialUPDI
pio device monitor            # serial monitor (xiao-style)
```

- **Two sub-variants**: (B1) `framework = arduino` (megaTinyCore — easiest, but the Arduino core costs flash
  + adds ISR jitter from core housekeeping; fine on a 4 KB part for this tiny program but verify the chip-rate
  ISR timing on a scope); (B2) PlatformIO as a *build/upload wrapper around the existing bare-C* (no
  framework) — keeps Path A's determinism, gains PIO ergonomics, but is a less-trodden PIO config.
- **megaTinyCore install** is automatic on first `pio run` (PlatformIO fetches the platform + core); the
  serialUPDI uploader reuses the **same cable/Nano as Path A** — no new hardware.

## 3. Decision (operator-confirmed 2026-06-22)

**Bare C, no Arduino, no OS** — the app (200 Hz Gold-code ISR + UVLO + WDT on a 4 KB part) needs none of
it. So:

| Use | Path |
|---|---|
| **Production-pod firmware (v1) — build of record** | **Path A (avr-gcc + serialUPDI Makefile)** — bare-C, minimal, deterministic ISR. |
| PlatformIO | **Not adopted for the emitter.** Its main draw (megaTinyCore / Arduino ecosystem + IDE monitor) is exactly the OS/core layer being rejected; for bare-C it adds little over the Makefile. The §2 `platformio.ini` is kept **only as a documented fallback** — a *bare-C-under-PIO* wrapper (no `framework`) if PIO ergonomics are ever wanted. Do **not** pull in `framework = arduino`. |

The 412 is a valid PlatformIO target, **but not via the Arduino framework** — bare code is the right fit
here. Path A stays the build/flash of record.

## 4. Bring-up milestone (emitter "E1" — prove the loop before the logic)

Mirroring the FPGA plan's **F1** discipline (isolate "can I flash the chip" from "is my code right"):

| # | Milestone | Done when |
|---|---|---|
| **E1** | **Prove the flash loop** with a trivial blink (toggle the diagnostic-LED GPIO off a timer, no Gold code). | A `.hex` builds (Path A *and*, if adopted, Path B) and flashes to the breadboard ATtiny412 / Curiosity Nano; the LED blinks — UPDI cable + pin map + upload confirmed before any code logic. |
| **E2** | **Gold-code ISR** — 200 Hz timer walking the 15-chip LUT onto the DIM GPIO; diagnostic blink at chip rate. | Scope on the LED-string sense node shows the 200 Hz, 5 ms-chip, 15-chip code (FR-1.5(a) `make decode`). |
| **E3** | **UVLO + WDT** — ADC bandgap sample → 3.6 V firmware trip → PA3 low + sleep; WDT ≤250 ms. | Bench-verified per FR-1.7 #4 truth table. |

E1 is the de-risk: a UPDI/cable problem must never masquerade as firmware bug.

## 5. Relationship to the FPGA toolchain

Both halves follow **"prove the loop before the logic"** (emitter E1 ↔ FPGA F1). The split:

| | Emitter | FPGA |
|---|---|---|
| Where it runs | **WSL-native** (apt/pip/PlatformIO, all in this session) | **Windows host** (Diamond), driven via WSL interop — *outside* this VSCode session |
| Novelty | Low — familiar (xiao-shaped); avr-gcc path already works | **High — the genuinely-new toolchain** for 031 |
| Programmer | serialUPDI over USB-UART (or Curiosity Nano) | USB-JTAG, Windows-native (Diamond Programmer) |
| Doc | this file | [fpga-toolchain-plan.md](fpga-toolchain-plan.md) |
</content>
