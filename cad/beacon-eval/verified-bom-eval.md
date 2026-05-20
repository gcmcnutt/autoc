# Verified BOM — 031 Beacon-Camera Phase **1a** Eval-Board (Printable Order Checklist)

**Audit date**: 2026-05-20
**Status**: Bench bring-up rig. Verifies (a) MCU gold-code generation on real silicon, (b) boost-converter + LED-string electrical behavior matches spec, (c) low-voltage cutout via supervisor IC.
**Schematic**: [`beacon-eval-schematic.png`](beacon-eval-schematic.png) (also [`beacon-eval-schematic.pdf`](beacon-eval-schematic.pdf); KiCad 10 source: [`beacon-eval.kicad_sch`](beacon-eval.kicad_sch))

> **What this is**: a *minimal*, *breadboarded* eval rig that uses the actual target Phase-1 beacon-pod circuit (LM3410X boost + sense-resistor LED driver + TPS3839 low-V supervisor + 5× Luxeon L1IZ-0850 IR LEDs in series) but **substitutes the ATtiny412 + UPDI + battery sub-assembly with a Microchip ATtiny416-XNANO evaluation kit**, with the XNANO's USB power isolated from the target rail via a one-time cut of an internal 0Ω strap.
>
> **What this is NOT**: a battery-powered, EMI-tight target-pod assembly. The cube enclosure, magnet-wire LED harness, and final perfboard EMI layout are explicitly out of scope for Phase 1a — they're part of Phase 1 §(a). This rig validates *components, topology, and firmware* before we commit to perfboard layout.

---

## What the eval validates

In three orthogonal checks:

1. **Gold-code generation** — ATtiny416 firmware drives DIM (PA3) with the 4-code PN-sequence at the spec'd chip rate. Verify on scope at U1 pin 4 (DIM).
2. **Boost-stage regulation** — LM3410X drives the 5-LED string at constant 306 mA via the 0.62 Ω sense resistor. Verify V_LED ≈ 10 V (auto-regulated, not preset) at C1, and I_LED at R1 (scope across R1 should show 190 mV ± 5%).
3. **Low-voltage cutout** — slowly turn the bench supply down from 5 V. When VIN drops below the TPS3839's nominal trip threshold (3.0 V variant `TPS3839L30DBVT`), U3 pulls DIM LOW via its open-drain output, and the entire LED string goes dark within the supervisor's response time. mEDBG debug stays alive on its independent USB-5 V domain throughout.

End-to-end check: when DIM modulates per gold code, LED current envelope tracks the modulation with the spec'd turn-on/turn-off transitions (see [`spec.md`](../../specs/031-beacon-camera/spec.md) §FR-1).

---

## XNANO modification (one-time, required)

The ATtiny416-XNANO ships with its target VCC (VTG) hard-wired to USB-5V via an internal 0Ω resistor (**R100**, marked on the XNANO PCB). To power the target ATtiny416 from our bench-controlled VIN_5V rail — so the supervisor can demonstrate cutout — **cut R100** with a hobby knife or hot-air rework once before first use.

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
| **DIM control (open-drain)** | **PA3** | **J200.5** | Wired to U1 DIM via R2 pull-up |
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
- [ ] **EV-A4 = target A5** Coilcraft **DR0810-223ML** 22 µH shielded inductor, qty 1. **Reuse from target order** (parent A5; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A5 = target A6** **MBR130T1G** Schottky SOD-123, qty 1. **Reuse from target order** (parent A6; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A6 = target A7** 0.62 Ω 1% 1206 sense resistor (Vishay `CRL1206-FW-R620ELF` or equiv), qty 1. **Reuse from target order** (parent A7; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A7 = target A13** 10 kΩ 0603 (DIM pull-up R2), qty 1. Reuse generic from parent A13.
  - [ ] received — notes:
- [ ] **EV-A8 = target A8** Samsung `CL31B475KAHNNNE` 4.7 µF/25 V X7R 1206 cap (V_LED output bulk C1), qty 1. **Reuse from target order** (parent A8; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A9 = target A12** Samsung `CL10B225KP8NNNC` 2.2 µF X7R cap (boost VIN bulk C5), qty 1. **Reuse from target order** (parent A12; +1 spare).
  - [ ] received — notes:
- [ ] **EV-A10 = target A11** 100 nF 0603 X7R (boost VIN HF decoupling C6 **AND** supervisor decoupling C7), qty 2. Reuse generic from parent A11. *(One extra over the original target BOM since the eval now has a supervisor.)*
  - [ ] received — notes:
- [ ] **EV-A11** TI **TPS3839L30DBVT** 3.0 V open-drain voltage supervisor, SOT-23-5, qty 1 (DigiKey: search "TPS3839L30DBVT"; ~$0.70). *Alternate to MCP1316T-23JE which we use on the target pod — same topology, same SOT-23-5 footprint, identical schematic role.* This part **is** also in the parent target BOM as the U3 alternate; if you're already ordering the MCP1316T for the target, use that and skip this line.
  - [ ] received — notes:

### Cart §EV-B — DigiKey (Lumileds — match target order)

- [ ] **EV-B1 = target B1** Lumileds **Luxeon IR Compact L1IZ-0850** 850 nm IR LED, qty **5** for one carrier, +5 spares = qty **10 total**. **Reuse from target order** (parent BOM Cart §B); the eval consumes 5 LEDs identical to the target part.
  - [ ] received — notes:

### Cart §EV-C — OSH Park / JLCPCB (LED carrier PCB)

- [ ] **EV-C1** Custom 2-layer PCB carrier for 5× L1IZ-0850 in series with a 2-pin 0.1″ header that mates with J3 on the eval breadboard rig. Dimensions ~20 × 60 mm. **Action**: produce a small KiCad PCB layout (separate task — not in this Phase-1a scope; deferred until eval needs the carrier physically built). Initial validation can be done with **flying-lead** L1IZ reflows on a stripboard if PCB lead time is a blocker.
  - [ ] received — notes:

### Cart §EV-D — Amazon / on-hand (rig consumables)

- [ ] **EV-D1** Solderless breadboard (any standard 830-tie-point board, ~$5). Likely on hand.
  - [ ] received — notes:
- [ ] **EV-D2** Jumper wire kit (M-M, M-F, 10cm and 20cm assortment, ~$5). Likely on hand.
  - [ ] received — notes:
- [ ] **EV-D3** SOT-23-5 to DIP-6 adapter board (for LM3410X **and** TPS3839 breakouts; SchmartBoard or Aries Electronics, ~$2 each, qty 3 to cover U1, U3, plus one spare). DigiKey: `1188-1018-ND` or similar.
  - [ ] received — notes:
- [ ] **EV-D4** SOD-123 to DIP-2 adapter (for MBR130; or just solder the SOD-123 directly to a perfboard scrap with magnet wire). Likely on hand / ad-hoc.
  - [ ] received — notes:
- [ ] **EV-D5** Bench DC supply, 5 V / 3 A min, **with adjustable voltage** (required so we can ramp VIN down to trigger the supervisor cutout). Common Rigol/Riden bench supplies fit. **Required on-hand**.
  - [ ] received — notes:
- [ ] **EV-D6** Oscilloscope, ≥20 MHz bandwidth, 2 channels min. **Required on-hand** for gold-code waveform validation.
  - [ ] received — notes:

---

## Bring-up sequence (suggested order)

1. **Cut R100 on the XNANO** (one-time mod). Verify with a multimeter: continuity between USB-5V test point and VTG (J200.1) should be **broken** after the cut.
2. **Solder the boost-converter sub-assembly** onto a SOT-23-to-DIP perfboard or stripboard: U1 (LM3410X) + L1 + D1 + C1 + R1 + R2 + C5 + C6, in a tight switching-loop topology (U1↔L1↔D1↔C1↔GND loop physically <5 mm if possible — FR-1.6 EMI mitigation). U3 supervisor + C7 mount adjacent to U1, with U3.RESET wired into the DIM node alongside R2.bot and U1.DIM.
3. **Hand-reflow 5× L1IZ-0850** in series on a small carrier PCB (OSH Park, ~$5 for 3 boards), or a stripboard fallback.
4. **Wire to breadboard**: bench supply 5 V/3 A → J2 (VIN_5V rail), LED carrier → J3, XNANO J200.1 (VTG) → VIN_5V rail, XNANO J200.5 (PA3) → DIM net, XNANO J200.20 (GND) → common ground rail.
5. **Power-on smoke test** (no firmware, MCU just sitting in default state):
   - Set bench supply to 5 V, current limit 1.5 A.
   - Apply power: U3 RESET releases (VIN > 3.0 V), R2 pulls DIM HIGH, U1 starts switching. V_LED rail should ramp to ~10 V, the IR LED string glows (visible on a phone camera since 850 nm is barely visible to human eye), I_in ≈ 0.7-0.8 A.
   - Measure V across R1 = 190 mV ± 5% to confirm constant-current loop is regulating.
6. **Low-voltage cutout test**: slowly turn bench supply voltage down from 5 V. At ≈3.0 V, U3 trip → DIM goes LOW → boost shuts down → LED string goes dark. Restore voltage to confirm clean restart. mEDBG over USB should be unaffected throughout.
7. **Flash gold-code firmware to ATtiny416** via XNANO mEDBG over USB-Micro-B (PlatformIO `pio run -e beacon-eval -t upload`).
8. **Verify on scope**: at U1 pin 4 (DIM) the waveform should be the 4-code PN sequence at the spec'd chip rate. At R1 (sense), the current envelope should track DIM with the spec'd turn-on/turn-off times.
9. **Once §1-§8 pass**: rebuild the same firmware for `env:beacon-target` and proceed to §(a) of [`quickstart.md`](../../specs/031-beacon-camera/quickstart.md) to commit to the cube-mounted target hardware.

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

- Schematic: [`beacon-eval-schematic.png`](beacon-eval-schematic.png) / [`beacon-eval-schematic.pdf`](beacon-eval-schematic.pdf) (KiCad 10.0.3 source `beacon-eval.kicad_sch`)
- Custom symbols: [`beacon-eval.kicad_sym`](beacon-eval.kicad_sym) (LM3410X SOT-23-5)
- Project lib table: [`sym-lib-table`](sym-lib-table)
- Parent target BOM: [`../../specs/031-beacon-camera/verified-bom.md`](../../specs/031-beacon-camera/verified-bom.md)
- Parent feature spec: [`../../specs/031-beacon-camera/spec.md`](../../specs/031-beacon-camera/spec.md)
- Parent quickstart: [`../../specs/031-beacon-camera/quickstart.md`](../../specs/031-beacon-camera/quickstart.md)
