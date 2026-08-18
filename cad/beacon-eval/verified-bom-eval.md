# Verified BOM — 031 Beacon-Camera Phase **1a** Eval-Board (Printable Order Checklist)

**Audit date**: 2026-05-20
**Status**: Bench bring-up rig. Verifies (a) MCU gold-code generation on real silicon, (b) boost-converter + LED-string electrical behavior matches spec, (c) low-voltage cutout via firmware ADC + topological failsafe + WDT *(revised 2026-05-20 per [research.md R11](../../specs/031-beacon-camera/research.md#r11--undervoltage-cutoff--led-driver-failsafe-1s-lipo-brown-out-protection); was "supervisor IC" before)*.
**Schematic**: [`beacon-eval-schematic.pdf`](beacon-eval-schematic.pdf) (KiCad 10 source: [`beacon-eval.kicad_sch`](beacon-eval.kicad_sch))

> **What this is**: a *minimal*, *breadboarded* eval rig that uses the actual target Phase-1 beacon-pod circuit (LM3410X boost + sense-resistor LED driver + 5× Luxeon L1IZ-0850 IR LEDs in series) but **substitutes the ATtiny412 + UPDI + battery sub-assembly with a Microchip ATtiny416-XNANO evaluation kit**, with the XNANO's USB power isolated from the target rail via a one-time cut of an internal 0Ω strap.
>
> **What this is NOT**: a battery-powered, EMI-tight target-pod assembly. The cube enclosure, magnet-wire LED harness, and final perfboard EMI layout are explicitly out of scope for Phase 1a — they're part of Phase 1 §(a). This rig validates *components, topology, and firmware* before we commit to perfboard layout.
>
> ⚠️ **Revised 2026-05-20 per FR-1.7 #4 / R11**: the TPS3839 supervisor IC (U3) and its decoupling cap (C7) are **removed** from the design; UVLO is now firmware ADC on the ATtiny416's internal 1.1 V bandgap (zero external parts) + a topological pull-DOWN of DIM to GND (so any MCU-offline state = LEDs OFF) + watchdog timer (≤ 250 ms timeout). The KiCad schematic was updated in commit `c30a9b6` to match — supervisor branch deleted, R2 flipped to pull-down. BOM lines below reflect the revised parts list — **do NOT order the supervisor (EV-A11) or the extra 100 nF cap for it (the second 100 nF that was listed under EV-A10)**.
>
> ➕ **Added 2026-06-17 (acquisition-research loop)**: the **single-IR-sensor receiver front-end** (photodiode → TIA → ADC → on-hand Lattice **STEP-MXO2** MachXO2; no purchase) is specified in [`../beacon-receiver/eval-loop-bom.md`](../beacon-receiver/eval-loop-bom.md). Its parts are all DigiKey — **batch that receiver cart with §EV-A here in one checkout** so the complete emitter→receiver loop orders together; receiver bring-up + the photometer→ADC→FPGA *physical simulation* then proceeds in parallel. The MachXO2 has **no analog input**, so the external ADC is the primary digitizer — amplitude capture is what enables the AGC / soft-decision / erasure / partial-acquisition studies (a comparator would discard it).

---


> **Bench vs FLIGHT (2026-08-17)** — `beacon-eval.kicad_sch` is the BENCH sheet (XNANO-416 on J1). It now
> carries the **OVP clamp: D2 15 V zener (cathode→V_OUT, anode→FB) + R3 1 k in series FB→sense** — same
> on both builds. The **flight cube differs in two components (L1, R1) + the MCU block** (D1 is the same
> SS1030 on both — corrected 2026-08-17):
>
> | Ref | Bench (this sheet) | Flight cube |
> |---|---|---|
> | L1 | 22 µH B82464 | **4.7 µH SPM4020T-4R7M-LR** (order-03 C-9) |
> | R1 sense | 3.74 Ω (51 mA) | **0.62 Ω CRL1206 (306 mA)** |
> | D1 Schottky | SS1030 SOD-123 (the ONLY part bought — MBR130 was the original spec, out of stock 2026-06-18, substituted like-for-like; the sheet's "MBR130" label was stale) | same SS1030 |
> | MCU | XNANO ATtiny416 dev board via J1 | **bare ATtiny412 SOIC-8** (order-03 C-26), serial-UPDI header, `-DBOOT_HALF_RATE` per cube |
> | Input net | as drawn | + damped bulk leg (3–4× 10 µF electrolytic) + 4.7 µF ceramic at VIN (the 4.7 µH bring-up lesson) |
>
> No separate flight sheet yet — the topology is identical; a `beacon-flight` sheet gets drawn when the
> cube board is laid out (A7-2).

## What the eval validates

In three orthogonal checks:

1. **Gold-code generation** — ATtiny416 firmware drives DIM (PA3) with the 4-code PN-sequence at the spec'd chip rate. Verify on scope at U1 pin 4 (DIM).
2. **Boost-stage regulation** — LM3410X drives the 5-LED string via the sense resistor at the FB reference (190 mV). **Bench value (2026-07): R1 = 3.74 Ω 1% → ~51 mA** (6× the field R_sense) to protect the un-heatsunk breadboard LED string from the field-power thermal load; **field/target value is 0.62 Ω → 306 mA**, restored once the SMT thermal carrier (EV-C1 / Luxeonstar SZ-01-R8-class MCPCB) is mounted. Verify I_LED at R1 (scope across R1 shows 190 mV ± 5% at either value; V_LED auto-regulates — ~11 V at the bench current, ~10 V at field).
3. **Low-voltage cutout** *(revised 2026-05-20 per R11)* — slowly turn the bench supply down from 5 V. When VIN drops below the firmware threshold (3.6 V firmware-set, corresponds to 3.5 V real after Vref drift), the ATtiny416 firmware drives PA3 LOW + enters POWER_DOWN sleep, R2 (now pull-down to GND) ensures DIM stays LOW even after MCU sleeps, and the LED string goes dark within the ~500 ms ADC debounce window. mEDBG debug stays alive on its independent USB-5 V domain throughout. Also bench-verify: (a) physical MCU soft-reset mid-emission → LEDs off within ≤ 1 ms (POR + topology); (b) deliberately-hung firmware variant → WDT reset within ≤ 250 ms → LEDs off.
4. **Open-LED over-voltage clamp** *(added 2026-07-16 — a boost CC driver with an open string rails V_OUT toward
   the 24 V V_SW abs-max / 25 V C1 rating; a bench mishap in this failure class killed an XNANO)* — per the
   LM3410 datasheet OVP application (TI SNVS541): **D3 = 15 V zener** (BZX55C15 DO-35 or 1N4744A DO-41, THT),
   cathode → V_OUT (C1 top), anode → FB pin; **R4 = 1 kΩ 5%** in series between the FB pin and the R1 sense
   node (LED-string cathode stays on the sense node). Normal operation: zener dark, R4 carries only FB bias
   (~100 nA → 0.1 mV on the 190 mV setpoint — negligible, same at bench 3.74 Ω or field 0.62 Ω sense). String
   opens: V_OUT clamps at ~V_Z+0.2 V ≈ 15.2 V with I_Z ≈ 190 mV/R4 ≈ 190 µA (~3 mW — cool indefinitely).
   ⚠️ R4 is NOT optional: zener straight to the sense node makes the loop drive the FULL programmed current
   through it (306 mA at field → ~4.6 W). Bench-verify: pull the LED header live → V_OUT settles ~15 V, no
   heating; replug → 190 mV across R1 resumes.

End-to-end check: when DIM modulates per gold code, LED current envelope tracks the modulation with the spec'd turn-on/turn-off transitions (see [`spec.md`](../../specs/031-beacon-camera/spec.md) §FR-1).

---

## XNANO modification (one-time, required)

The ATtiny416-XNANO ships with its target VCC (VTG) hard-wired to USB-5V via an internal 0Ω resistor (**R100**, marked on the XNANO PCB). To power the target ATtiny416 from our bench-controlled VIN_5V rail — so we can deliberately ramp VIN down to trigger the firmware ADC cutoff — **cut R100** with a hobby knife or hot-air rework once before first use.

After the cut:
- **XNANO target side** (ATtiny416 + onboard user LED on PB5 + user button on PB4) = powered by our bench supply via J1 pin 1 (VTG).
- **XNANO debug side** (ATmega32U4 mEDBG, USB Micro-B, status LED) = still powered by USB-5V on a fully isolated rail. UPDI programming and CDC virtual COM work normally even when we deliberately collapse VIN_5V to trigger the cutout test.

This is a destructive board mod — the XNANO can be restored with a 0Ω 0603 jumper if desired later.

---

## Firmware portability (drop-in ATtiny412 ↔ ATtiny416)

The eval restricts its pin assignments to **only the GPIOs that exist on both ATtiny412 (8-SOIC, 6 GPIOs) and ATtiny416 (20-SOIC, 14 GPIOs)** — namely `PA0/1/2/3/6/7`. Pins `PA4`, `PA5`, and all `PB*`/`PC*` are off-limits because they don't exist on the ATtiny412. Same source, same register accesses, no `#ifdef` pin remapping — just rebuild for either chip via:

```ini
; firmware/beacon-pod/platformio.ini

[env:beacon-eval]
platform = atmelmegaavr
board = ATtiny416_xnano       ; via megaTinyCore
framework = arduino
upload_protocol = xplainedmini_updi   ; uses XNANO's onboard mEDBG over USB

[env:beacon-target]
platform = atmelmegaavr
board = ATtiny412              ; production hardware
framework = arduino
upload_protocol = serialupdi   ; or jtag2updi via external programmer
```

Pin role assignments (identical on both chips):

| Function | Pin | XNANO J200 pin | Notes |
|---|---|---|---|
| UPDI / programming | PA0 | J200.2 | mEDBG on XNANO; SerialUPDI on target |
| UART TX (optional debug) | PA1 | J200.3 | CDC over mEDBG on XNANO |
| UART RX (optional debug) | PA2 | J200.4 | CDC over mEDBG on XNANO |
| **DIM control (push-pull active-HIGH)** *(revised 2026-05-20 per R11)* | **PA3** | **J200.5** | Wired to U1 DIM; R2 pulls DIM DOWN to GND (topological failsafe: PA3 high-Z → DIM LOW → LEDs OFF) |
| ADC undervoltage cutoff *(NEW 2026-05-20 per R11)* | internal 1.1 V bandgap on ADC0 | n/a (internal channel) | No GPIO consumed; firmware reads bandgap ratio against V_BAT (= Vref) |
| CSEL0 (code-select jumper) | PA6 | J200.8 | Optional on eval; firmware reads at boot |
| CSEL1 (code-select jumper) | PA7 | J200.9 | Optional on eval; firmware reads at boot |

> **Why this matters**: when the eval validates a gold-code firmware build, the *exact same `.hex`* (modulo `board=` line in `platformio.ini` regenerating fuses) flashes to the production ATtiny412 with zero code changes. No pin-mapping table, no `#ifdef MCU_XNANO`. This is the discipline that lets the eval rig actually pre-validate flight firmware.

---

## BOM (printable, per-line check + receive)

> Tracking convention: top-level `[ ]` = ordered. Sub-`[ ] received` = in hand.
> Cart-letter grouping matches the same vendor batches as the parent [`verified-bom.md`](../../specs/031-beacon-camera/verified-bom.md) so eval and target orders can be batched in one DigiKey/Mouser checkout.

### Cart §EV-A — DigiKey (eval kit + reused target parts)

- [ ] **EV-A1** ATtiny416-XNANO evaluation kit — Microchip **ATtiny416 Xplained Nano** (DigiKey: `ATTINY416-XNANO-ND`, ~$11). Includes mEDBG over USB-Micro-B, user LED, user button, 20-pin edge header. **One-time R100 cut required before use** (see "XNANO modification" above).
  - [ ] received — notes:
- [ ] **EV-A2** USB-A to USB-Micro-B cable (data, ≥0.5 m). Likely have on hand. If ordering: any quality cable, ~$5.
  - [ ] received — notes:
- [ ] **EV-A3 = target A1** TI **LM3410XMF-NOPB** SOT-23-5 boost LED driver, qty 1. **Reuse from target order** (parent BOM Cart §A line A1; order +1 spare).
  - [ ] received — notes:
- [ ] **EV-A4 = target A5** **22 µH boost inductor, Isat ≥ 1.5 A** (boost peak ~1.2 A), qty 1 (+1 spare). **Primary (in stock, ordered): TDK/EPCOS `B82464G4223M000`** (22 µH ±20%, shielded SMD). Alt: Coilcraft `MSS1038-223MLC` (Isat 2.34 A, shielded). *(Original `DR0810-223ML` radial-THT appears non-stock/delisted as of 2026-06-18.)* ⚠️ Do NOT use Würth 7447745220 (only 1.05 A, backordered to Oct-2026). Any in-stock 22 µH @ Isat ≥ 1.5 A works; LM3410X also tolerates 10–15 µH @ ≥1.5 A as a fallback.
  - [ ] received — notes:
- [ ] **EV-A5 = target A6** Schottky boost rectifier, qty 1 (+1 spare). Original **MBR130T1G** (SOD-123) **OUT OF STOCK 2026-06-18 → substitute Panjit `SS1030_R1_00001`** (DigiKey `3757-SS1030_R1_00001CT-ND`, 30 V Schottky, **SOD-123** — same package as MBR130). ✅ like-for-like boost rectifier; **confirm forward rating ≥ 1 A** (boost peak ~1.2 A; MBR130 was 1 A SOD-123).
  - [ ] received — notes:
- [ ] **EV-A6 = target A7** sense resistor, 1206, qty 1. **Field/target: 0.62 Ω 1%** (Vishay `CRL1206-FW-R620ELF` or equiv → 306 mA) — reuse from target order (parent A7; +1 spare). **Bench de-rate (2026-07): fit 3.74 Ω 1% (6×) → ~51 mA** to protect the un-heatsunk breadboard LEDs; swap back to 0.62 Ω once the SMT thermal carrier is mounted.
  - [ ] received — notes:
- [ ] **EV-A7 = target A13** 10 kΩ 0603 (DIM **pull-DOWN to GND** R2 — *revised 2026-05-20 per FR-1.7 #4 / R11; was pull-up to V_BAT*), qty 1. Reuse generic from parent A13.
  - [ ] received — notes:
- [ ] **EV-A8 = target A8** Samsung `CL31B475KAHNNNE` 4.7 µF/25 V X7R 1206 cap (V_LED output bulk C1), qty 1. **Reuse from target order** (parent A8; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A9 = target A12** Samsung `CL10B225KP8NNNC` 2.2 µF X7R cap (boost VIN bulk C5), qty 1. **Reuse from target order** (parent A12; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A10 = target A11** 100 nF 0603 X7R (boost VIN HF decoupling C6), qty 1. Reuse generic from parent A11. *(Revised 2026-05-20 per R11: was qty 2 to cover supervisor decoupling C7; supervisor removed → qty 1 only.)*
  - [ ] received — notes:
- ~~**EV-A11** TI TPS3839L30DBVT 3.0 V open-drain voltage supervisor~~ *(removed 2026-05-20 per FR-1.7 #4 / R11: UVLO via firmware ADC + topological failsafe + WDT — no supervisor IC needed. Additionally, the previous BOM line had three errors that the removal moots: TPS3839L30 = 2.63 V not 3.0 V per datasheet; DBVT package code doesn't exist (only DBZ SOT-23-3 or DQN X2SON-4); TPS3839 output is push-pull not open-drain.)*
- [ ] **EV-A12 = target A14** JST-PH 2.0 mm 2-pin THT socket (JST `S2B-PH-K-S` or equiv) — battery / bench-supply input header J2 on the eval, mates with the flight battery's standard JST-PH 2-pin pigtail. **Reuse from target order** (parent BOM Cart §A line A14; +1 spare). On the eval, plug in either a charged 1S LiPo (for battery-as-switch testing) or a bench-supply harness with a JST-PH male connector (for controlled VIN ramps during firmware-ADC cutoff verification).
  - [ ] received — notes:
- [ ] **EV-A13** *(added 2026-07-16 — open-LED OVP, orthogonal-check #4)* **15 V zener D3**, THT: `BZX55C15`
  (DO-35, 500 mW) or `1N4744A` (DO-41, 1 W), qty 1 (+1 spare). Cathode → V_OUT (C1 top), anode → FB pin.
  Clamps an open-string boost runaway at ~15.2 V (vs 24 V V_SW abs-max / 25 V C1). Dissipates ~3 mW in clamp
  (with R4 — see EV-A14). Also add to the **field/target pod BOM** when it next revs.
  - [ ] received — notes:
- [ ] **EV-A14** *(added 2026-07-16, pairs with EV-A13)* **1 kΩ 5% R4**, THT or 0603, qty 1 — series FB-pin →
  R1-sense-node. Limits zener clamp current to ~190 µA. ⚠️ NOT optional: without it the loop drives the full
  programmed LED current through the zener (~4.6 W at field 306 mA).
  - [ ] received — notes:

### Cart §EV-B — DigiKey (Lumileds — match target order)

- [ ] **EV-B1 = target B1** Lumileds **Luxeon IR Compact L1IZ-0850** 850 nm IR LED, qty **5** for one carrier, +5 spares = qty **10 total**. **Reuse from target order** (parent BOM Cart §B); the eval consumes 5 LEDs identical to the target part.
  - [ ] received — notes:

### Cart §EV-C — OSH Park / JLCPCB (LED carrier PCB)

- [ ] **EV-C1** Custom 2-layer PCB carrier for 5× L1IZ-0850 in series with a 2-pin 0.1″ header that ties into the V_LED net and the FB-sense return on the eval rig. Dimensions ~20 × 60 mm. **Action**: produce a small KiCad PCB layout (separate task — not in this Phase-1a scope; deferred until eval needs the carrier physically built). Initial validation can be done with **flying-lead** L1IZ reflows on a stripboard or perfboard if PCB lead time is a blocker. *(Revised 2026-05-20: schematic no longer has a dedicated J3 carrier connector; LEDs wire directly into the V_LED → FB chain. The carrier PCB just bridges the V_LED out to the LED-string-anode net and the LED-string-cathode back to FB-sense.)*
  - [ ] received — notes:

### Cart §EV-D — Amazon / on-hand (rig consumables)

- [ ] **EV-D1** Solderless breadboard (any standard 830-tie-point board, ~$5). Likely on hand.
  - [ ] received — notes:
- [ ] **EV-D2** Jumper wire kit (M-M, M-F, 10cm and 20cm assortment, ~$5). Likely on hand.
  - [ ] received — notes:
- [ ] **EV-D3** SOT-23-5→DIP adapter board (for LM3410X breakout), qty 2 (U1 + spare). DigiKey: **SchmalzTech `ST-SOT23-5`** (exact SOT-23-5), or Chip Quik **`PA0089`** / **`PCB3007-1`**, or SparkFun **`BOB-00717`** (a SOT-23-6 adapter also fits the 5-pin part — leave one pad unused). *(Corrected 2026-06-18: the prior `1188-1018-ND` was wrong — that DK number is an Olimex USB cable, not an adapter.)* *(Revised 2026-05-20 per R11: was qty 3 to cover U1 + U3; supervisor removed → qty 2 only.)*
  - [ ] received — notes:
- [ ] **EV-D4** SOD-123 to DIP-2 adapter (for MBR130; or just solder the SOD-123 directly to a perfboard scrap with magnet wire). Likely on hand / ad-hoc.
  - [ ] received — notes:
- [ ] **EV-D5** Bench DC supply, 5 V / 3 A min, **with adjustable voltage** (required so we can ramp VIN down to trigger the firmware ADC cutoff at ~3.5 V real V_BAT). Common Rigol/Riden bench supplies fit. **Required on-hand**.
  - [ ] received — notes:
- [ ] **EV-D6** Oscilloscope, ≥20 MHz bandwidth, 2 channels min. **Required on-hand** for gold-code waveform validation.
  - [ ] received — notes:

---

## Bring-up sequence (suggested order)

1. **Cut R100 on the XNANO** (one-time mod). Verify with a multimeter: continuity between USB-5V test point and VTG (J200.1) should be **broken** after the cut.
2. **Solder the boost-converter sub-assembly** onto a SOT-23-to-DIP perfboard or stripboard: U1 (LM3410X) + L1 + D1 + C1 + R1 + R2 + C5 + C6, in a tight switching-loop topology (U1↔L1↔D1↔C1↔GND loop physically <5 mm if possible — FR-1.6 EMI mitigation). **R2 wires from U1.DIM to GND (pull-DOWN, not pull-up to V_BAT)** *(revised 2026-05-20 per R11)*. No supervisor IC.
3. **Hand-reflow 5× L1IZ-0850** in series on a small carrier PCB (OSH Park, ~$5 for 3 boards), or a stripboard fallback.
4. **Wire to breadboard** *(revised 2026-05-20)*: bench supply 5 V/3 A (or 1S LiPo on its JST-PH pigtail) → **J2** (the new JST-PH 2-pin socket) which feeds the VIN_5V rail. LED-string carrier ties V_LED node → 5× LED series chain → FB-sense node. XNANO J200.1 (VTG, after R100 cut) → VIN_5V rail; XNANO J200.5 (PA3) → DIM net; XNANO J200.20 (GND) → common ground rail. (Bench supply and XNANO VTG both end up on the same VIN_5V net — either source works; the R100 cut just decouples USB from VTG so VTG follows VIN_5V instead.)
5. **Power-on smoke test** *(revised 2026-05-20 per R11)* — with no firmware loaded, the MCU's PA3 is high-Z by default, R2 pulls DIM LOW, U1 is in shutdown, LEDs are OFF. *This is the failsafe in action.* To smoke-test the boost: temporarily jumper PA3 to V_BAT directly (manually overriding the topological failsafe):
   - Set bench supply to 5 V, current limit 1.5 A.
   - Jumper U1.DIM directly to V_BAT (bypasses MCU + R2).
   - Apply power: U1 starts switching, V_LED rail ramps to ~10 V, the IR LED string glows (visible on a phone camera since 850 nm is barely visible to human eye), I_in ≈ 0.7-0.8 A.
   - Measure V across R1 = 190 mV ± 5% to confirm constant-current loop is regulating.
   - Remove the jumper before continuing.
6. **Low-voltage cutout test** *(revised 2026-05-20 per R11)*: with firmware flashed (see §7) and the MCU driving DIM HIGH at idle, slowly turn bench supply voltage down from 5 V. At ≈3.5 V real V_BAT (firmware-detected at 3.6 V), MCU should drive PA3 LOW + sleep → DIM low → boost shuts down → LED string goes dark within ~500 ms ADC debounce. Restore voltage and power-cycle to confirm clean restart. Also: (a) during emission, issue a soft-reset over UPDI — LEDs should go OFF within ≤ 1 ms (POR + topology); (b) load a deliberately-hung firmware variant — WDT should reset within ≤ 250 ms → LEDs OFF. mEDBG over USB should be unaffected throughout all of these.
7. **Flash gold-code firmware to ATtiny416** via XNANO mEDBG over USB-Micro-B (PlatformIO `pio run -e beacon-eval -t upload`).
8. **Verify on scope**: at U1 pin 4 (DIM) the waveform should be the 4-code PN sequence at the spec'd chip rate. At R1 (sense), the current envelope should track DIM with the spec'd turn-on/turn-off times.
9. **Once §1-§8 pass**: rebuild the same firmware for `env:beacon-target` and proceed to §(a) of [`quickstart.md`](../../specs/040-camera-redo/camera-hardware-phase/quickstart.md) to commit to the cube-mounted target hardware.

---

## What the eval intentionally does NOT cover

| Out of scope for Phase 1a eval | Validated in |
|---|---|
| 1S LiPo battery + charge protection | Phase 1 target pod |
| Mechanical: cube apex LED placement + 32 AWG magnet-wire harness | Phase 1 target pod assembly |
| EMI compliance + final perfboard layout / SW-node loop area | Phase 1 target pod assembly |
| Eye-safety photometric verification (4-LED redundancy, polar plot) | See [`eye-safety-measurements.md`](../../specs/031-beacon-camera/eye-safety-measurements.md) |
| Camera pipeline + recorder + decode/correlator firmware | Phase 2 / Phase 3 |

---

## Cross-reference

- Schematic: [`beacon-eval-schematic.pdf`](beacon-eval-schematic.pdf) (KiCad 10.0.3 source `beacon-eval.kicad_sch`)
- Custom symbols: [`beacon-eval.kicad_sym`](beacon-eval.kicad_sym) (LM3410X SOT-23-5)
- Project lib table: [`sym-lib-table`](sym-lib-table)
- Parent target BOM: [`../../specs/031-beacon-camera/verified-bom.md`](../../specs/031-beacon-camera/verified-bom.md)
- Parent feature spec: [`../../specs/031-beacon-camera/spec.md`](../../specs/031-beacon-camera/spec.md)
- Parent quickstart: [`../../specs/040-camera-redo/camera-hardware-phase/quickstart.md`](../../specs/040-camera-redo/camera-hardware-phase/quickstart.md)
