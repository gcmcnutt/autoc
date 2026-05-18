# Verified BOM — 031 Beacon-Camera Phase 1 (Printable Order Checklist)

**Audit date**: 2026-05-18
**Status**: Cart-ready for ordering. Spec corrections (§0) queued as task **T007a** to apply before US1 hand-build.

> **Print this for the order phase.** Check off each line as it's ordered + received. Cart letters group items by vendor for batched checkout.

---

## Contents

- [§0 — Critical spec corrections discovered during audit](#0-critical-spec-corrections-discovered-during-audit) ⚠️ must apply via T007a
- [§A — Mouser/DigiKey: beacon-pod electronics + tools](#a--mouserdigikey-beacon-pod-electronics--tools)
- [§B — DigiKey: Lumileds LEDs](#b--digikey-lumileds-leds)
- [§C — Amazon: consumables + LiPos](#c--amazon-consumables--lipos)
- [§D — Camera + recorder (Arducam + Lattice)](#d--camera--recorder-arducam--lattice)
- [§E — Long-lead (Commonlands + OG0VA)](#e--long-lead-commonlands--og0va)
- [§F — IR-sensor smoke-test rig (beacon-decode-first path)](#f--ir-sensor-smoke-test-rig-beacon-decode-first-path) 🆕
- [§G — One-time tooling](#g--one-time-tooling)
- [§H — Budget summary + sourcing notes](#h--budget-summary--sourcing-notes)
- [§I — Spec correction queue (T007a)](#i--spec-correction-queue-t007a)

---

## §0 — Critical spec corrections discovered during audit

⚠️ **These were found while pulling the LM3410 datasheet (TI SNVS541H, pages 1+3+5). Apply via T007a before US1 hand-build (T032).**

### §0.1 — LM3410 part-variant SWAPPED in spec

Spec/plan calls **LM3410-Y** the 1.6 MHz variant. Datasheet says:
- **LM3410**X** = 1.6 MHz** ← what we want (enables smaller 22 µH inductor)
- **LM3410**Y** = 525 kHz**

**Order: `LM3410XMF-NOPB` (SOT-23-5).**

### §0.2 — LM3410 has NO separate EN pin; DIM IS the shutdown control

Spec says "supervisor holds EN HIGH, MCU drives DIM". Datasheet (page 3 Pin Functions): SOT-23-5 has only `SW, GND, FB, VIN, DIM` — no EN. DIM is *"Dimming and shutdown control input. Logic high enables operation."*

**Corrected wiring**: supervisor open-drain output + MCU push-pull GPIO both wire onto DIM via a pull-up resistor (wired-AND). Soft-start = 20 µs (datasheet page 5, SU parameter) is 0.2% of a 10 ms chip → no design impact at 100 Hz chip rate.

### §0.3 — V_OUT max is 24 V (spec said 38 V)

Datasheet page 4 Section 6.3: V_SW abs max = 24 V. Our 9.5 V LED string is well under — no design impact, but spec figure is wrong.

### §0.4 — Switch current limit is 2.8 A typ (spec said 1.6 A)

Datasheet page 5 Section 6.5: I_CL = 2.1 min / **2.8 typ A**. More margin than spec claimed. No issue.

---

## §A — Mouser/DigiKey: beacon-pod electronics + tools

**Quantity sizing**: 3 complete pod builds (2 pods + 1 spare), with margin where consumed.

```
=========================================================================
  CART A — MOUSER OR DIGIKEY (1-2 day shipping)
=========================================================================
```

- [ ] **A1**  LM3410**X**MF-NOPB ×6 — boost LED driver, SOT-23-5, 1.6 MHz, **NOT -Y** ........... ~$15
- [ ] **A2**  ATTINY412-SSFR ×6 — ATtiny412 SOIC-8 (production-pod MCU) ........................... ~$4
- [ ] **A3**  DM080104 ×1 — ATtiny412 Curiosity Nano dev kit (first-flash bring-up; one-time) ..... ~$10
- [ ] **A4**  Voltage supervisor 3.3V trip, open-drain ×6 — pick one in stock:
  - MCP1316T-29LE/OT (Microchip), OR
  - TPS3839K33DBVR (TI), OR
  - APX803-31SAG (Diodes) .......................................................................... ~$3
- [ ] **A5**  22 µH shielded SMD inductor, I_sat ≥1.5 A ×6 — Coilcraft DR0810-223ML or similar ... ~$7
- [ ] **A6**  MBR130T1G Schottky 1A 30V ×6 ......................................................... ~$2
- [ ] **A7**  0.62 Ω 1% 1206 sense resistor ×6 .................................................... ~$3
- [ ] **A8**  4.7 µF / 25 V X7R 1206 (V_LED bulk) ×6 .............................................. ~$3
- [ ] **A9**  22 µF / 10 V X7R 1210 (V_BAT bulk) ×6 ............................................... ~$3
- [ ] **A10** 1 µF X7R 0603 ×12 .................................................................... ~$1
- [ ] **A11** 100 nF X7R 0603 ×30 (MCU decoupling, supervisor decoupling, driver-VIN) ............. ~$2
- [ ] **A12** 2.2 µF X7R 0603 (boost-driver VIN decoupling) ×6 .................................... ~$1
- [ ] **A13** 10 kΩ 0603 (DIM-line pull-up per §0.2) ×6 ........................................... <$1
- [ ] **A14** JST-PH 2.0 mm 2-pin THT socket ×6 — S2B-PH-K-S or equivalent ...................... ~$2
- [ ] **A15** 0603 green visible-light LED ×6 (mandatory diagnostic per FR-1.2.1) ................. ~$1
- [ ] **A16** 1 kΩ 0603 (diagnostic LED series R) ×6 .............................................. <$1
- [ ] **A17** SOT-23-to-DIP adapter PCBs ×12 — Schmartboard 204-0008-01 or generic Amazon ........ ~$6
- [ ] **A18** Perfboard, 25×25 mm cuts ×1 sheet — Vector Electronics 169P44WE ..................... ~$8
- [ ] **A19** USB-UART adapter ×1 (FT232 / CP2102) — for serialUPDI to bare ATtiny412 ............. ~$8
- [ ] **A20** 4.7 kΩ resistor ×3 — serialUPDI pull-up .............................................. <$1
- [ ] **A21** Header pins 0.1" male ×1 strip — for UPDI programming hookup ......................... ~$2

**Cart A subtotal**: ~$80

---

## §B — DigiKey: Lumileds LEDs

```
=========================================================================
  CART B — DIGIKEY (Lumileds — bin-code-verify-at-checkout)
=========================================================================
```

- [ ] **B1**  Lumileds L1IZ-0850000000000 ×20 — Luxeon IR Compact 850 nm, DigiKey **7243418** ... ~$54
  - **At checkout**: confirm the wavelength bin code suffix. Need **850 ± 5 nm bin** for filter match.
  - **Datasheet**: DS190 (Lumileds direct: <https://lumileds.com/wp-content/uploads/files/DS190.pdf>)
  - DigiKey product page lists 150° "viewing angle" — verify HPBW vs the spec's 130° claim from the datasheet appendix; if different, [`plot_led_configs.py`](plot_led_configs.py) re-run may be warranted.

**Cart B subtotal**: ~$54

---

## §C — Amazon: consumables + LiPos

```
=========================================================================
  CART C — AMAZON (consumables + batteries)
=========================================================================
```

- [ ] **C1**  1S LiPo, 100 mAh, 20C, JST-PH 2.0 pigtail ×6 — Tinywhoop-class (B083NWXLTK or
              BetaFPV / EMAX equivalent) ......................................................... ~$18
- [ ] **C2**  1S USB-pigtail LiPo charger ×1 (if not in stock) — ISDT N8 / HOTA D6 / generic ..... ~$15
- [ ] **C3**  32 AWG magnet wire ×1 roll — Belden 8051 or equivalent (~10 m) ..................... ~$8
- [ ] **C4**  Double-back tape OR velcro hook+loop ×1 roll — 3M VHB or generic ................... ~$10

**Cart C subtotal**: ~$50

---

## §D — Camera + recorder (Arducam + Lattice)

```
=========================================================================
  CART D — ARDUCAM DIRECT + LATTICE DIRECT + ASSORTED
=========================================================================
```

- [ ] **D1**  Arducam B0162 OV9281 1MP Global Shutter MIPI module ×1 + 1 spare ................. ~$70
- [ ] **D2**  Arducam B0264 USB-UVC Camera Shield ×1 + 1 spare (bench-mode UVC streaming) ...... ~$90
- [ ] **D3**  Lattice CrossLink-NX-EVN board (LIFCL-40-EVN) ×1 — Lattice direct or Mouser ..... ~$300-400
  - **At order**: confirm onboard HyperRAM / SDRAM (need ≥6 MB for FR-2.5 ring), microSD slot
    with 4-bit SDIO, USB-C port. Vendor pages 403'd in audit; verify from latticesemi.com user guide.
- [ ] **D4**  m12lenses.com PT-02120 145° M12 fisheye lens ×1 + 1 spare ........................ ~$60
  - **First-receiver check (task T044)**: confirm absence of IR-cut filter.
- [ ] **D5**  850 nm bandpass filter ×1 + 1 spare — pick whichever is in stock:
  - Edmund Optics #65-679 (10 mm dia, 850 ± 5 nm CWL, 10 nm FWHM), OR
  - Thorlabs FB850-10 (1" dia, 850 ± 2 nm CWL, 10 ± 2 nm FWHM) ................................ ~$140-180
- [ ] **D6**  SanDisk Extreme Pro V30 microSD 64 GB ×2 + 1 spare ................................ ~$75
- [ ] **D7**  Pololu D24V10F5 5V 1A buck ×1 + 1 spare ........................................... ~$20
- [ ] **D8**  MIPI flex cable (if not shipped with B0264 kit) ×1 + 1 spare ...................... ~$10
- [ ] **D9**  XT60 pigtail for carrier-LiPo input to Pololu buck ×1 ............................. ~$5

**Cart D subtotal**: ~$770-930 (heavily Lattice + filter dominated)

---

## §E — Long-lead (place inquiry now)

```
=========================================================================
  CART E — LONG-LEAD INQUIRIES (email/phone — may not ship in Phase 1)
=========================================================================
```

- [ ] **E1**  Commonlands custom M12 lens — email `contact@commonlands.com`:
  > "Quote request: M12 lens, 120° HFOV, F/2.0, NIR-corrected for 850 nm.
  > Integrated bandpass filter: CWL = 850 ± 5 nm, FWHM ≤ 30 nm (10 nm preferred).
  > Quantity: 1 prototype + quote for production volume."
  - Expected lead time: 4-6 weeks.
- [ ] **E2**  OmniVision OG0VA — contact OmniVision OEM channel:
  > "Inquiry: OG0VA bare-die or CameraCubeChip module availability for prototype
  > quantity (1-5 units). Lead time + MOQ?"
  - Alternative: contact a design-house reseller (Leopard Imaging, e-con Systems).
  - Phase 1 ships on OV9281+B0162 from §D; OG0VA is the optional upgrade.

**Cart E subtotal**: $0-300 (deferrable; depends on lead-time outcomes)

---

## §F — IR-sensor smoke-test rig (beacon-decode-first path) 🆕

> **Purpose** (operator direction 2026-05-18): get the beacon optical chain validated FIRST, before the camera/recorder is built. A photodiode + scope (or photodiode + small Lattice FPGA dev kit) is enough to confirm the beacon emits a recoverable 15-bit Gold code at 100 Hz chip rate. This is the **FR-1.5(b) photodiode-level verification** elevated to a primary smoke-test gate.

**Decode paths** enabled by this cart:
- **Path 1 — Scope-direct**: photodiode → load resistor → scope screen → operator visually confirms 100 Hz chip rate + Gold-code pattern.
- **Path 2 — Phone-direct**: smartphone IR camera (already in §G `T6`) — quickly confirm pod is emitting; coarse decode by eye.
- **Path 3 — Small-Lattice-FPGA decode**: photodiode → comparator → 1-bit FPGA input → onboard automated 15-chip Gold-code correlator → onboard LED or UART reports recovered code. **Operator already owns a small Lattice dev kit** (model TBD — see [post-order Q in tasks.md](tasks.md)).

```
=========================================================================
  CART F — IR SENSOR + SUPPORT PARTS (fast Amazon / DigiKey ship)
=========================================================================
```

- [ ] **F1**  IR photodiode, 850 nm peak, fast response — pick ONE primary + 1 backup:
  - **Vishay BPW34** ×4 (silicon PIN PD, 5 mm through-hole, ~$1.50 each), OR
  - **Osram SFH 213 FA** ×4 (with daylight filter built in, ~$2 each — better for outdoor sun),
    OR
  - **Vishay BPV10NF** ×4 (NPN phototransistor, ~$1 each — single-component analog out) ...... ~$6-10
- [ ] **F2**  **TI OPT101P** ×2 — integrated photodiode + transimpedance amp, 14 kHz BW,
              voltage output, single-supply 2.7-36 V. Ideal "drop into breadboard + scope" sensor ... ~$10
- [ ] **F3**  1 MΩ 0603 SMT OR 1 MΩ 1/4 W axial ×4 — load resistor for bare-PD path ............ <$1
- [ ] **F4**  0.1 µF film/ceramic ×4 — DC-block for AC-coupled scope view of beacon modulation .. <$1
- [ ] **F5**  LM393 dual comparator ×2 — converts analog PD signal → digital edge for FPGA input  ~$1
              (alternative: 74HC14 hex Schmitt-trigger inverter ×1 ............................... ~$0.50)
- [ ] **F6**  Solderless breadboard, 400-tie-point ×1 ............................................ ~$5
- [ ] **F7**  Jumper wire kit (M-M, M-F) ........................................................ ~$8
- [ ] **F8**  9 V battery + clip OR 5V USB-power supply ×1 — bench supply for OPT101 or
              comparator (often you have this) .................................................. ~$3
- [ ] **F9**  3 mm acrylic + standoffs (optional — mount PD facing pod at known distance) ........ ~$5

**Cart F subtotal**: ~$30-40 — cheapest cart, fastest ship (2-day Amazon Prime + DigiKey 2-day for parts).

**Order Cart F FIRST**: arrives before the LiPos finish charging. Validates the beacon emits a recoverable code from any one pod the moment that pod is hand-built (T033) — no need to wait for the rest of the camera/recorder chain.

---

## §G — One-time tooling

```
=========================================================================
  CART G — ONE-TIME TOOLING (mostly already in operator's bench)
=========================================================================
```

- [ ] **G1**  Oscilloscope ≥1 MHz BW — for FR-1.5 scope verification (already operator's) ........ have ✓
- [ ] **G2**  Soldering iron + flux + 60/40 solder (already operator's) .......................... have ✓
- [ ] **G3**  3D printer + PLA filament (already operator's) .................................... have ✓
- [ ] **G4**  Smartphone (FR-3.4 IR-emission qualitative check) (already operator's) ............. have ✓
- [ ] **G5**  Bench programmable supply 0-5 V at ≥1 A (FR-1.7 UVLO verify) (already operator's) .. have ✓
- [ ] **G6**  Bench multimeter (already operator's) .............................................. have ✓
- [ ] **G7**  Calibrated NIR power meter — **DEFERRED** per [eye-safety-measurements.md §5](eye-safety-measurements.md) .. SKIP

---

## §H — Budget summary + sourcing notes

| Cart | Description | Estimated spend | Lead time |
|---|---|---|---|
| §A | Beacon-pod electronics + tools | ~$80 | 1-2 days |
| §B | Lumileds LEDs | ~$54 | 1-2 days (in stock at audit) |
| §C | LiPos + consumables | ~$50 | 2-3 days |
| §D | Camera + Lattice EVN + lens + filter + SD + buck | ~$770-930 | 1-2 weeks (Lattice EVN may be longer) |
| §E | Commonlands lens + OG0VA (long-lead inquiries) | $0-300 | 4-12 weeks |
| **§F** | **IR-sensor smoke-test rig (NEW — order FIRST)** | **~$30-40** | **1-2 days** |
| §G | One-time tooling | ~$0 (mostly stock) | — |
| **Phase 1 total** | | **~$1000-1400** | |

### Sourcing reality check (from this audit)

- **DigiKey** stock confirmed: Lumileds L1IZ (36 937 units in stock at audit time).
- **Vendor pages 403'd / timed out** during WebFetch: Lattice EVN board, Arducam direct, ATtiny412 DigiKey URL (returned wrong product), Microchip Curiosity Nano page. Operator confirms current stock + prices at order time. No part is known to be discontinued.
- **OG0VA**: deferred to later sourcing inquiry — OEM channel only, not DigiKey-stocked, expected 6-12 week lead.

### Suggested ordering sequence

1. **TODAY**: §F (IR sensor — fast ship, unblocks beacon-only smoke test)
2. **THIS WEEK**: §A + §B + §C (beacon-pod parts)
3. **THIS WEEK or as ready**: §D (camera + Lattice EVN — confirm onboard memory before ordering)
4. **THIS WEEK**: §E inquiries (long-lead — start clock now even if you don't commit to ordering)

---

## §I — Spec correction queue (T007a)

Per §0 above, before US1 hand-build (T032) the following edits must be applied. This is the punch-list for task T007a:

- [ ] **I1** spec.md FR-1.2 ASCII diagram: replace `EN ←─ supervisor` + `DIM ←─ MCU` with `DIM ←─ wired-AND (MCU push-pull HIGH + supervisor open-drain LOW + 10 kΩ pull-up to V_BAT)`.
- [ ] **I2** spec.md FR-1.2.1 BOM driver-IC row:
  - "TI LM3410-Y (primary, 1.6 MHz fsw)" → "TI LM3410**X** (primary, 1.6 MHz fsw)"
  - Remove "EN pin held HIGH by supervisor" claim; replace with DIM-wired-AND wording
  - "up to 38 V output" → "up to 24 V output (V_SW abs max per datasheet)"
  - "1.6 A peak switch" → "2.8 A typ switch current limit"
- [ ] **I3** spec.md FR-1.7 #4 UVLO contract: supervisor open-drain output gates DIM (not EN); MCU's GPIO is overridden low when supervisor trips; under UVLO the whole IC enters 80 nA shutdown.
- [ ] **I4** spec.md Decisions-Locked LED-drive topology row: similar wiring correction + LM3410-X.
- [ ] **I5** spec.md Clarifications Session 2026-05-17 "warm/strict-on/off" Q&A: revise the "boost stays warm via supervisor-held EN" framing to "DIM-only control with 20 µs soft-start (negligible at 100 Hz chip rate, satisfies the strict-on/off design intent)".
- [ ] **I6** contracts/mcu-firmware-contract.md: update the LM3410-Y/EN/DIM wiring paragraph; firmware itself unchanged (MCU still writes one GPIO at chip rate).
- [ ] **I7** plan.md: any mentions of LM3410-Y → LM3410-X.

Estimated effort: ~20 minutes of mechanical edits. Output: one commit titled `fix(031 phase 1): LM3410-Y → -X + DIM-only architecture per datasheet audit`.

---

**End of printable BOM.** Print + use during ordering. After receiving each part, check the box. Items with **vendor-side-verify** notes should be confirmed live on the vendor page at checkout.
