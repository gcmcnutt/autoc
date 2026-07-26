# Beacon Bench — Order 03 (COMBINED, 2026-07-24 — absorbs the never-submitted Order 02)

**What this is**: the third bench order, now the SINGLE active list — neither the 2026-07-05 Order-02
DigiKey cart nor the original Order-03 was ever submitted, so they are combined and deduped here.
Follow-on to [Order 01](verified-bom-eval.md) / [receiver BOM](../beacon-receiver/eval-loop-bom.md).
Provenance tags (was O2-x / O3-x) kept for the paper trail.

> **History absorbed from Order 02**: a few items were bought separately (Amazon): the **1S 150 mAh
> bench LiPo** (was O2-14, ✔ purchased — bench substitute; flight uses SPMX1501S50) and **SMT prototyping
> breadboards that arrived WRONG** (no SOT-23 footprint → the ST-SOT23-5 adapters below are the fix).
> Items marked **⚠ verify on-hand** may have arrived in that shipment — check the drawer before carting.

**Drivers**: (1) attrition from the IR bring-up blowouts (LM3410X, one XNANO); (2) OVP clamp
(EV-A13/A14); (3) **flight-emitter cube @ 306 mA + daylight ground receiver (Option C) → 100 m field
test**; (4) stop stalling on missing passives (kits).

---

## A — OVP clamp (EV-A13/A14)
- [ ] **C-1** *(O3-1)* 15 V zener THT `BZX55C15` (DO-35; alt `1N4744A` DO-41) — qty **5**. Cathode→V_OUT,
  anode→FB; clamp ≈15.2 V.
  - [ ] received — notes:
- [ ] **C-2** *(O3-2)* 1 kΩ 5 % ¼ W THT — qty **10** (R4 series FB→sense; ⚠ NOT optional with the zener).
  - [ ] received — notes:

## B — Attrition + mounting
- [ ] **C-3** *(O3-3)* **LM3410XMF-NOPB** SOT-23-5 — qty **4**.
  - [ ] received — notes:
- [ ] **C-4** *(O3-4+O3-12 merged)* **ATTINY416-XNANO** — qty **2** (one casualty; new units need their own
  admin `usbipd bind --force` + R100 cut).
  - [ ] received — notes:
- [ ] **C-5** *(O3-5)* **SS1030** Schottky SOD-123 (DK `3757-SS1030_R1_00001CT-ND`) — qty **3**.
  - [ ] received — notes:
- [ ] **C-6** *(O3-6+O3-10 merged)* **SOT-23-5→DIP adapters** SchmalzTech `ST-SOT23-5` — qty **8**
  (also the fix for the wrong Order-02 SMT breadboards; Chip Quik `PA0089`/`PCB3007-1` alt).
  - [ ] received — notes:
- [ ] **C-7** *(O3-13)* **SOIC-8→DIP adapters** qty **6** (ST-SOIC-8 / PA0018 / Adafruit 1212) — the
  ATtiny412 is SOIC-8 only; ⚠ verify the parent-order `ATTINY412-SSFR` ×6 arrived, add spares if not.
  - [ ] received — notes:

## C — Flight-emitter cube (306 mA)
- [ ] **C-8** *(O3-14)* **Thin double-sided copper-clad FR4** 0.5–0.8 mm, 1 oz+ (MG Chemicals small sheet),
  qty 1 — shear into ~10×10 mm single-LED tiles (knife-slit islands, stitch faces); 5 tiles glue into the
  cube. ~15 °C rise at field power per tile — validate one before gluing five.
  - [ ] received — notes:
- [ ] **C-9** *(O3-16)* **Flight boost inductor** 4.7 µH shielded, Isat ≥ 2 A, 4×4×2 mm class — Coilcraft
  **XFL4020-472ME** qty **3** (alt XAL4020-103 10 µH). Replaces the 10×10 mm 22 µH brick for flight.
  - [ ] received — notes:
- [ ] **C-10** *(O3-15)* **Battery connector 1.25 mm, male PCB side** — flight pack = **Spektrum
  SPMX1501S50** (std LiPo, 4.20 V charge; "PH 1.25 Ultra Micro" = UMX = **PicoBlade-compatible**, NOT real
  JST-PH). Molex **53047-0210** THT qty **6**, or hobby UMX male pigtails.
  - [ ] received — notes:
- [ ] **C-11** *(O3-7 = O2-3 merged; optional)* **Luxeonstar SZ-01-R8** MCPCB L1IZ-0850 ×≤5 + small heatsink
  — the no-reflow alternative to C-8 tiles for field power. Order only if tiles disappoint.
  - [ ] received — notes:
- [ ] **C-12** *(O3-9; conditional)* **L1IZ-0850** spares ×5 — only if the string took blowout damage
  (10 bought on order-01; check first).
  - [ ] received — notes:
- [ ] **C-13** *(O2-1)* Sense resistor **3.74 Ω 1 % 1206** ×3 (bench 51 mA de-rate; one already on the
  bench — these are spares) + field **0.62 Ω** from order-01 stock. ⚠ verify on-hand.
  - [ ] received — notes:

## D — Daylight receiver / 100 m optics (Option C selected 2026-07-24)
- [ ] **C-14** *(O3-11 = O2-13 merged)* **850 nm bandpass filter + collection lens** — REQUIRED for daylight:
  **open choice** (collector-schematic-daylight.md Q1): Thorlabs **FBH850-40** (40 nm, passes ~85 % of the
  LED's 30 nm line, ambient ÷7) vs **FB850-10** (10 nm, beacon ×0.3, ambient ÷25 — direct-sun insurance).
  Lean 40 nm primary; + **M12 lens** (m12lenses.com PT-02120 class) + M12 holder/mount.
  - [ ] received — notes:
- [ ] **C-15** *(O2-7; demoted by Option C)* TSSOP/MSOP-8→DIP adapter (Aries LCQT-TSSOP8) ×2 — only if the
  OPA381 precision-TIA path revives; Option C needs no precision TIA. Optional.
  - [ ] received — notes:

## E — Bench consumables
- [ ] **C-16** *(O3-17, absorbs O2-9/O2-10)* **Passives assortment kits (THT)**: resistor kit ¼ W 1 %
  metal-film 10 Ω–1 MΩ; MLCC kit 10 pF–1 µF; electrolytic kit 1–1000 µF; small film-cap kit 10 nF–1 µF.
  Amazon-class ~$10–15 ea, qty 1 each. (Covers the O2-era 100 nF radial + 10 µF bulk lines too.)
  - [ ] received — notes:
- [ ] **C-17** *(O2-11)* **High-temp wire**: 30 AWG silver-PTFE multi-color kit (SMT jumpers, iron-proof) +
  22–24 AWG silicone stranded ×2 colors (power leads). ⚠ verify on-hand (may have arrived).
  - [ ] received — notes:
- [ ] **C-18** *(O2-12)* **Bare tinned bus wire** 22 AWG spool (+24 AWG optional) — vias/stitching on the
  copper boards. ⚠ verify on-hand.
  - [ ] received — notes:
- [ ] **C-19** *(O2-8)* Solderless breadboard **BB830** (DK `2864-BB830-ND`) ×2. ⚠ verify on-hand.
  - [ ] received — notes:
- [ ] **C-20** *(O2-2)* **Copper SMT proto board w/ ground plane** (BusBoard SMT3U or MG 540 copper-clad
  5″×3″) ×1–2 — field-power thermal + the ground plane for the jitter measurement. ⚠ the Order-02 attempt
  arrived WRONG (no SOT-23) — C-6 adapters are the mounting fix; this board is still wanted for plane/thermal.
  - [ ] received — notes:

## F — RC-vs-crystal stability study (gated by the copper-board load test; unchanged from O2)
- [ ] **C-21** *(O2-4)* 32.768 kHz watch crystal (CFS-206/AB26T, CL 12.5 pF) ×3 — XOSC32K on PB2/PB3;
  ⚠ 416-only (412 has no TOSC → ≥14-pin tinyAVR for a shipping pod on this path).
  - [ ] received — notes:
- [ ] **C-22** *(O2-5)* Crystal load caps: a few 18 pF + 6.8 pF.
  - [ ] received — notes:
- [ ] **C-23** *(O2-6; fallback only)* 20.000 MHz CMOS XO half-can ×2 + 3.3 V LDO ×1 — order only if the
  XO route is pursued (EXTCLK→PA3 conflict: DIM moves to PA6/PA7).
  - [ ] received — notes:

## Separate supplier
- [ ] **C-24** *(O3-8, O2-14 caveat)* **1S 100 mAh flight pack** — the definitive RC-osc jitter test wants
  flight-pack internal resistance (the 150 mAh bench pack reads optimistic).
  - [ ] received — notes:

## Already on hand — do NOT re-order
- BPV10NF (primary PD), BPW34 (wide-FOV alt), 5× L1IZ-0850 on the bench string, 1S 150 mAh bench LiPo (✔),
  MCP6022/MCP3201/MCP1525 receiver ICs, 22 µH bench inductor.

## Related but $0 (firmware/fuses — not parts)
- ~~F_CPU 20→10 MHz~~ **DONE** (ca1fe78) · ~~BOD fuse~~ **DONE** (0x48) · ~~firmware UVLO~~ **DONE &
  verified 3.48 V** (07d195f) — all three closed since the original Order-03 draft.

## Open decisions gated by this order
1. Filter FWHM (C-14): 40 nm vs 10 nm — Set-B lamp tests + first Option-C field test inform.
2. RC vs 32k-crystal vs XO (F section) — still gated on the copper-board load-test jitter measurement.
