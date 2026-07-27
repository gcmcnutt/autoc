# Beacon Bench — Order 03 (COMBINED, 2026-07-24 — absorbs the never-submitted Order 02)

**What this is**: the third bench order, now the SINGLE active list. **Reconciled 2026-07-26 against the
two actual DigiKey packlists**: **DK order 1** = invoice 127789318 (ordered 18-Jun-2026, 26 lines — the
main emitter+receiver buy) and **DK order 2** = invoice 128637262 (ordered 05-Jul-2026, 5 lines — a small
slice of the Order-02 cart DID ship: 3.74 Ω ×10, SMT3U ×2, BB830 ×1, 100 nF radial ×50, 10 µF radial ×10;
the rest of that cart was never submitted). Follow-on to [Order 01](verified-bom-eval.md) /
[receiver BOM](../beacon-receiver/eval-loop-bom.md). Provenance tags (was O2-x / O3-x) kept.
**Qty policy**: sweet spot is **10** for cheap jellybean lines (operator 2026-07-26) — stop nickel-and-diming.

> **History**: the **1S 150 mAh bench LiPo** came via Amazon (was O2-14, ✔ — bench substitute; flight uses
> SPMX1501S50). The **SMT3U proto boards** (DK order 2) turned out SOIC-footprint-only — WRONG for SOT-23
> parts → the ST-SOT23-5 adapters below are the fix. Answer the **Inventory cross-check** section at the
> bottom before carting — order 1 delivered qty-10 reels of most "attrition" parts, so several replenish
> lines below are CONDITIONAL on a drawer count, not automatic.

**Drivers**: (1) attrition from the IR bring-up blowouts (LM3410X, one XNANO); (2) OVP clamp
(EV-A13/A14); (3) **flight-emitter cube @ 306 mA + daylight ground receiver (Option C) → 100 m field
test**; (4) stop stalling on missing passives (kits).

---

## A — OVP clamp (EV-A13/A14)
- [ ] **C-1** *(O3-1)* 15 V zener THT `BZX55C15` (DO-35; alt `1N4744A` DO-41) — qty **10** (sweet spot; was 5).
  Cathode→V_OUT, anode→FB; clamp ≈15.2 V. Not in either DK packlist — genuinely needed.
  - [ ] received — notes:
- [ ] **C-2** *(O3-2)* 1 kΩ 5 % ¼ W THT — qty **10** (R4 series FB→sense; ⚠ NOT optional with the zener).
  Kit overlap: the C-16 resistor kit covers 1 k THT — skip this line if the kits go in the same cart.
  - [ ] received — notes:

## B — Attrition + mounting
- [X] **C-3** *(O3-3)* ~~LM3410XMF-NOPB replenish~~ — **DROPPED 2026-07-26** (Q1: enough survivors of the
  ×10 from order 1).
- [ ] **C-4** *(O3-4+O3-12 merged)* **ATTINY416-XNANO** — qty **2** (order 1 delivered ×2, ONE CONFIRMED
  BROKEN → one working unit; new units need their own admin `usbipd bind --force` + R100 cut).
  - [ ] received — notes:
- [X] **C-5** *(O3-5)* ~~SS1030 Schottky replenish~~ — **DROPPED 2026-07-26** (Q2: enough survivors of the
  ×10 from order 1).
- [ ] **C-6** *(O3-6+O3-10 merged)* **SOT-23-5→DIP adapters** SchmalzTech `ST-SOT23-5` — qty **10** (sweet
  spot; was 8). CONFIRMED short: order 1 brought only ×2 SparkFun 00717, and the SMT3U boards are
  SOIC-only (Chip Quik `PA0089`/`PCB3007-1` alt).
  - [ ] received — notes:
- [X] **C-7** *(O3-13)* ~~SOIC-8→DIP adapters~~ — **DROPPED 2026-07-26** (Q3: no ATtiny412s owned — they
  were in neither DK packlist; the 416 XNANO path carries everything. If the 412 shipping-pod path ever
  revives, order chips + adapters together).

## C — Flight-emitter cube (306 mA)
- [ ] **C-8** *(O3-14)* **Thin double-sided copper-clad FR4** 0.5–0.8 mm, 1 oz+ (MG Chemicals small sheet),
  qty 1 — shear into ~10×10 mm single-LED tiles (knife-slit islands, stitch faces); 5 tiles glue into the
  cube. ~15 °C rise at field power per tile — validate one before gluing five.
  - [ ] received — notes:
- [ ] **C-9** *(O3-16)* **Flight boost inductor** 4.7 µH shielded, Isat ≥ 2 A, 4×4×2 mm class — Coilcraft
  **XFL4020-472ME** qty **10** (sweet spot; ~$2 ea; alt XAL4020-103 10 µH). Replaces the 10×10 mm 22 µH
  brick for flight (22 µH bricks: ×10 on hand from order 1).
  - [ ] received — notes:
- [ ] **C-10** *(O3-15)* **Battery connector 1.25 mm, male PCB side** — flight pack = **Spektrum
  SPMX1501S50** (std LiPo, 4.20 V charge; "PH 1.25 Ultra Micro" = UMX = **PicoBlade-compatible**, NOT real
  JST-PH). Molex **53047-0210** THT qty **10** (sweet spot; was 6), or hobby UMX male pigtails. NB: the
  ×10 `S2B-PH-K-S` from order 1 are JST-PH **2.0 mm** — wrong pitch for the UMX pack, don't confuse them.
  - [ ] received — notes:
- [ ] **C-11** *(O3-7 = O2-3 merged; optional)* **Luxeonstar SZ-01-R8** MCPCB L1IZ-0850 ×≤5 + small heatsink
  — the no-reflow alternative to C-8 tiles for field power. Order only if tiles disappoint.
  - [ ] received — notes:
- [ ] **C-12** *(O3-9; NOW FIRM — operator 2026-07-26)* **L1IZ-0850** qty **10** (sweet spot). The 5-LED
  bench string is a **permanent bench fixture** — it does NOT donate to the flight cube. Cube takes 5,
  rest are spares/second-cube stock.
  - [ ] received — notes:
- [X] **C-13** *(O2-1)* ~~Sense resistor 3.74 Ω 1 % 1206~~ — **✔ CLOSED: DK order 2 delivered ×10**
  (`541-3.74FFCT-ND`, 1/4 W); field 0.62 Ω also confirmed ×10 in order 1 (`CRL1206-FW-R620ELF`, 1/2 W).
  - [X] received — notes: packlists 128637262 line 1 / 127789318 line 6.

## D — Daylight receiver / 100 m optics (Option C selected 2026-07-24)
- [ ] **C-14** *(O3-11 = O2-13 merged; **DECIDED 2026-07-26 → BUDGET M12 STACK**)* **850 nm bandpass
  filter + collection lens** for the ~100 m single-pixel test. Thorlabs (FBH850-40 / FB850-10) is
  **DEFERRED to the 040 camera** — the nicer filter earns its keep on the 2-D array; the single-pixel
  test uses the cheap M12/CCTV ecosystem (which is also what 040 will mount, so the holder/lens carry
  forward). **Sensor at the focal plane = BPW34** (flat 2.65 mm chip, no dome — clean image target;
  its unfiltered visible response is exactly what the bandpass fixes). The domed BPV10NF stays the
  bare/finder sensor — its built-in optic fights an external lens (±20° acceptance, tiny target).
  Order (~$25–30 total):
  - **M12 lens holder**, metal, PCB-mount (m12lenses.com / Amazon generic, ~$4) — screws to a perfboard
    scrap with the BPW34 soldered dead-center under it.
  - **M12 board lens 16 mm f/1.6–2.0** (~10 mm aperture, ~$10) — FOV ≈ 2.65/16 ≈ **9.5°** (16 m capture
    width @ 100 m); aperture gain ≈ ×11 over bare BPW34 → with 306 mA emitters ≈ **95–100 m** reach.
    Focus by telemetry: slide the thread while watching `corrB` peak.
  - **850 nm bandpass discs 12.5 mm ×1.0 mm ×2** — Quanmin (Amazon, ~$12/pair); drop into the holder's
    internal step behind the lens. Realistically 30–50 nm FWHM, ±10 nm CWL — fine vs the LED's 30 nm line.
  - *Alt single-part (operator-found, mount VERIFIED M12×0.5)*: AliExpress "ELP HD M12 Lens, 850 nm IR
    narrow-pass filter" (~$10, item 3256809007490501) — lens+filter in one, screws into any standard M12
    holder; **pick the longest-focal SKU (16 mm; 12 mm acceptable)** — the 2.8/3.6/6 mm variants are
    wide-FOV webcam glass with too little aperture for 100 m. Typical F2.0 @ 16 mm → 8 mm aperture ≈ ×7
    over bare BPW34 → ~75–80 m; still fine. Buying this makes the Quanmin discs optional (backup/second rig).
    *Step-up if CWL tolerance worries*: Commonlands CBP850, M12-threaded (~$25).
  - *Headroom option if 100 m is thin*: 25 mm plano-convex (~$10, DIY tube) ≈ ×70 → ×8 range, FOV ~4°.
  ⚠ **couples to the Option-C R_load** (pedestal ceiling 3.3 V/R): cheap discs are **40 nm-class →
  R_load 22 k** (47 k reserved for a true 10 nm filter) — see the daylight doc's pedestal table. Note the
  9.5° FOV itself is ambient rejection (sky-only background when aimed at a flying beacon).
  - [ ] received — notes:
- [ ] **C-15** *(O2-7; demoted by Option C)* TSSOP/MSOP-8→DIP adapter (Aries LCQT-TSSOP8) ×2 — only if the
  OPA381 precision-TIA path revives (the OPA381 chips themselves are ✔ on hand, ×2 from order 1); Option C
  needs no precision TIA. Optional.
  - [ ] received — notes:
- [X] **C-25** *(NEW 2026-07-25)* ~~1N4148 small-signal diodes~~ — **DROPPED 2026-07-26** (Q5: on hand in
  the drawer; D2/D3 clamps covered).

## E — Bench consumables
- [ ] **C-16** *(O3-17, absorbs O2-9/O2-10; TRIMMED 2026-07-26)* **Passives assortment kits (THT)** —
  the two that still matter: **resistor kit ¼ W 1 % metal-film 10 Ω–1 MΩ** (Option-C R_load 47 k/22 k,
  R5 1–2 k, plus C-2's 1 k — on-hand resistors are 0603 SMD only) and **MLCC kit 10 pF–1 µF** (C5 820 pF
  and friends). Amazon-class ~$10–15 ea. The electrolytic + film-cap kits are now OPTIONAL: DK order 2
  delivered **100 nF radial ×50 + 10 µF radial ×10** which covers C6/C7/C8 for many builds.
  - [ ] received — notes:
- [X] **C-17** *(O2-11)* ~~High-temp wire~~ — **DROPPED 2026-07-26** (Q6: wire stock confirmed on hand).
- [X] **C-18** *(O2-12)* ~~Bare tinned bus wire~~ — **DROPPED 2026-07-26** (Q6: on hand).
- [ ] **C-19** *(O2-8; HALF-CLOSED)* Solderless breadboard **BB830** — **✔ ×1 received (DK order 2)**.
  Order 1 more only if two simultaneous rigs are actually wanted (emitter + receiver already split boards).
  - [ ] received — notes: ×1 in packlist 128637262 line 3.
- [ ] **C-20** *(O2-2; REVISED)* **Ground-plane / thermal copper board** — the SMT3U ×2 that arrived
  (DK order 2) are SOIC-pad boards with NO plane and NO SOT-23 (C-6 adapters = the mounting fix). Still
  wanted for plane/thermal: **MG 540 copper-clad 5″×3″** ×1–2 — or shear it from the C-8 FR4 sheet and
  skip this line.
  - [ ] received — notes: SMT3U ×2 ✔ (usable as SOIC carriers only).

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

## Already on hand — do NOT re-order (reconciled against both DK packlists, 2026-07-26)
- **Receiver**: BPV10NF ×2, BPW34 ×10, MCP6022 ×2, MCP3201 ×2, MCP1525 ×2, OPA381 ×2, 1 MΩ 3296W
  trimmer ×2.
- **Emitter**: L1IZ-0850 ×10 (5 on the bench string), LM3410X ×10 − attrition, SS1030 ×10 − attrition,
  22 µH B82464 ×10, 0.62 Ω 1206 ×10, 3.74 Ω 1206 ×10, XNANO ×2 − 1 casualty.
- **Passives (0603 SMD, ×10 each)**: 10 k, 100 k, 1 M, 10 M, 2 pF C0G, 100 nF, 1 µF (0805), 2.2 µF,
  4.7 µF (1206). **THT**: 100 nF radial ×50, 10 µF radial ×10.
- **Misc**: JST-PH 2.0 mm S2B ×10 (NOT the UMX 1.25 mm pitch — see C-10), SOT23→DIP ×2 (short — C-6),
  SMT3U ×2, BB830 ×1, TTL-232R-3V3 cable, microSD breakout ×2, 1S 150 mAh bench LiPo (Amazon),
  1N4148 drawer stock, PTFE 30 AWG + silicone 22–24 AWG + bare bus wire, 1 M 3296W trimmers ×2 (✔ both
  alive).

## Inventory cross-check — ANSWERED by operator 2026-07-26

- [X] **Q1 — LM3410X**: enough survivors → **C-3 dropped**.
- [X] **Q2 — SS1030**: enough survivors → **C-5 dropped**.
- [X] **Q3 — ATtiny412**: none owned (in neither packlist) → **C-7 dropped** (revive chips+adapters
  together if that path returns).
- [X] **Q4 — L1IZ-0850**: **the 5-LED bench string stays bench permanently** → the cube needs its own
  LEDs → **C-12 FIRM at ×10**.
- [X] **Q5 — 1N4148**: on hand → **C-25 dropped**.
- [X] **Q6 — wire**: on hand → **C-17/C-18 dropped**.
- [X] **Q7 — 1 M trimmers**: both alive (re-filed; Option C uses fixed R).

## Related but $0 (firmware/fuses — not parts)
- ~~F_CPU 20→10 MHz~~ **DONE** (ca1fe78) · ~~BOD fuse~~ **DONE** (0x48) · ~~firmware UVLO~~ **DONE &
  verified 3.48 V** (07d195f) — all three closed since the original Order-03 draft.

## Open decisions gated by this order
1. ~~Filter FWHM (C-14): 40 nm vs 10 nm~~ — **RESOLVED 2026-07-26: budget 40 nm-class (M12 stack) for
   the single-pixel test → Option-C R_load = 22 k**; true 10 nm (Thorlabs) deferred to the 040 camera
   where the 2-D array earns it. (Pedestal-ceiling math in the daylight doc.)
2. RC vs 32k-crystal vs XO (F section) — still gated on the copper-board load-test jitter measurement.
