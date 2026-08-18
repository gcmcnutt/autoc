# Beacon Bench — Order 03 (COMBINED, 2026-07-24 — absorbs the never-submitted Order 02)

**What this is**: the third bench order, now the SINGLE active list. **Reconciled 2026-07-26 against the
two actual DigiKey packlists**: **DK order 1** = invoice 127789318 (ordered 18-Jun-2026, 26 lines — the
main emitter+receiver buy) and **DK order 2** = invoice 128637262 (ordered 05-Jul-2026, 5 lines — a small
slice of the Order-02 cart DID ship: 3.74 Ω ×10, SMT3U ×2, BB830 ×1, 100 nF radial ×50, 10 µF radial ×10;
the rest of that cart was never submitted). Follow-on to [Order 01](verified-bom-eval.md) /
[receiver BOM](../beacon-receiver/eval-loop-bom.md). Provenance tags (was O2-x / O3-x) kept.
**Qty policy**: sweet spot is **10** for cheap jellybean lines (operator 2026-07-26) — stop nickel-and-diming.
**AMZ-1 receipt 2026-07-30**: Amazon order **111-5521786-5077025** (placed 26-Jul-2026, delivered 30-Jul-2026,
8 lines) landed — closes C-2/C-6/C-8/C-15/C-16/C-20, half-closes C-14 (holder only) and C-22. The 8th line
(1/2 AA LS14250 ×4) is **not part of this bench order** — unrelated household batteries.
**DK-3 receipt 2026-08-02**: **DK order 3** = invoice **129837577** / sales order 100616351 (ordered
26-Jul-2026, 6 lines) landed — closes C-1/C-10/C-12, closes C-4 (see qty caveat), adds an SMD 1 k to C-2.
⚠ **C-9 short-shipped: 9 of 10 inductors — 1 BACKORDERED, arriving as a separate shipment** (packlist line 4,
`445-174477-1-ND`); watch for the second box and update C-9 when it lands.

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
- [X] **C-1** *(O3-1)* 15 V zener THT `BZX55C15` — **✔ RECEIVED ×10 (DK-3 line 1)**: Vishay
  `BZX55C15-TAP` (`BZX55C15-TAPGICT-ND`), 15 V 500 mW, **DO-204AH = DO-35** axial as specced.
  Cathode→V_OUT, anode→FB; clamp ≈15.2 V.
  - [X] received — notes: ×10, packlist 129837577 line 1.
- [X] **C-2** *(O3-2)* ~~1 kΩ 5 % ¼ W THT — qty 10~~ — **✔ CLOSED twice over**: (a) the C-16 resistor kit
  (BOJACK 50-value 1350 pc, 1 % ¼ W metal film, 0 Ω–5.6 MΩ; AMZ-1) supplies the **THT** part, and (b) DK-3
  line 2 delivered KOA `RK73B2ATTD102J` ×10 — 1 k 5 % ¼ W but **0805 SMD, not THT**. Use the kit part for
  breadboard/DO-35-zener work; keep the 0805s for the SMD emitter build. R4 series FB→sense is covered.
  - [X] received — notes: kit AMZ-1 2026-07-30 (THT); 0805 ×10 packlist 129837577 line 2.

## B — Attrition + mounting
- [X] **C-3** *(O3-3)* ~~LM3410XMF-NOPB replenish~~ — **DROPPED 2026-07-26** (Q1: enough survivors of the
  ×10 from order 1).
- [X] **C-4** *(O3-4+O3-12 merged)* **ATTINY416-XNANO** — **✔ RECEIVED ×1 (DK-3 line 3)**. ⚠ **Qty caveat**:
  the line asked ×2 but the cart was placed for ×1 (ordered 1 / shipped 1 — not a short-ship). Bench now
  has **2 working units** (1 new + 1 survivor of order 1's pair), which meets the actual need; the spare
  margin is gone, so order another if a second casualty happens. New unit needs its own admin
  `usbipd bind --force` + R100 cut before use.
  - [X] received — notes: ×1, packlist 129837577 line 3 (lot RAY255200016).
- [X] **C-5** *(O3-5)* ~~SS1030 Schottky replenish~~ — **DROPPED 2026-07-26** (Q2: enough survivors of the
  ×10 from order 1).
- [X] **C-6** *(O3-6+O3-10 merged)* **SOT-23-5→DIP adapters** — **✔ RECEIVED (AMZ-1)**: Cermant
  **SOT23-6 / SC70-6 → DIP** breakout ×20 (substitute for the SchmalzTech `ST-SOT23-5`; a SOT-23-5 part
  lands on 5 of the 6 pads — leave the unused pad open). Supersedes the ×2 SparkFun 00717 shortfall and
  the SOIC-only SMT3U boards.
  - [X] received — notes: ×20, AMZ-1 2026-07-30. Verify pin-1 orientation on the first build.
- [X] **C-7** *(O3-13)* ~~SOIC-8→DIP adapters~~ — **DROPPED 2026-07-26** (Q3: no ATtiny412s owned — they
  were in neither DK packlist; the 416 XNANO path carries everything. If the 412 shipping-pod path ever
  revives, order chips + adapters together).

## C — Flight-emitter cube (306 mA)
- [X] **C-8** *(O3-14)* **Thin double-sided copper-clad FR4** — **✔ RECEIVED (AMZ-1)**: uxcell 70×50 mm
  double-sided copper-clad FR4 ×5, **1.0 mm** thick (spec asked 0.5–0.8 mm — 1 mm is stiffer/heavier per
  tile; acceptable, but weigh a finished tile before committing the cube). Shear into ~10×10 mm single-LED
  tiles (knife-slit islands, stitch faces); 5 tiles glue into the cube. ~15 °C rise at field power per
  tile — validate one before gluing five.
  - [X] received — notes: ×5 sheets @ 70×50×1.0 mm, AMZ-1 2026-07-30. ⚠ thickness 1.0 mm vs 0.5–0.8 mm spec.
- [X] **C-9** *(O3-16; **CLOSED 2026-08-07 — 10 of 10**)* **Flight boost inductor** 4.7 µH shielded,
  Isat ≥ 2 A, 4×4×2 mm class — supplied as TDK **`SPM4020T-4R7M-LR`** (`445-174477-1-ND`), **not** the
  specced Coilcraft XFL4020-472ME: 4.7 µH, **2.5 A**, DCR **147.2 mΩ**, 4.0×4.0×2.0 mm metal-composite
  shielded — meets the spec envelope (Isat 2.5 A > 2 A required). Replaces the 10×10 mm 22 µH brick for
  flight (22 µH bricks: ×10 on hand from order 1).
  - [X] received — notes: ×9 received 2026-08-02 (lot 5017524072) + **backordered 10th received
    2026-08-07** — full ×10 in hand. **DigiKey side of order-03 is now fully closed.**
- [X] **C-10** *(O3-15)* **Battery connector 1.25 mm, male PCB side** — **✔ RECEIVED ×10 (DK-3 line 5)**:
  Molex **`0530470210`** (`WM1731-ND`), 2-pos vertical header, 1.25 mm — exactly as specced. Flight pack =
  **Spektrum SPMX1501S50** (std LiPo, 4.20 V charge; "PH 1.25 Ultra Micro" = UMX = **PicoBlade-compatible**,
  NOT real JST-PH). NB: the ×10 `S2B-PH-K-S` from order 1 are JST-PH **2.0 mm** — wrong pitch for the UMX
  pack, don't confuse them.
  - [X] received — notes: ×10, packlist 129837577 line 5.
- [ ] **C-11** *(O3-7 = O2-3 merged; **TRIGGERED 2026-08-10**)* **Luxeonstar pre-mounted stars ×5–6** —
  the trigger fired on ASSEMBLY, not thermal (A7-1: 1″ tile = only 5–8 °C rise at 306 mA, PASS — but
  hand-mounting the bottom-pad Luxeon Z without SMT gear is fragile vs flight vibration). **Prefer
  SM-01-R8 (5 mm Micro-Z base)** for the 1″ tiles; SZ-01-R8 (10 mm) alt. Factory-reflowed die +
  hand-solderable tabs + epoxy star→tile = robust, and the calibrated emitter is unchanged.
  ⚠ **NO DigiKey part number exists (checked 2026-08-07)** — mounted-star modules are Luxeonstar
  (Quadica) direct only: SZ-01-R8 (10 mm base, this line), **SM-01-R8 (5 mm Micro-Z1 base — lighter,
  likely the better cube fallback)**, SZ-05-R8 (4-up 20 mm, bench). DigiKey carries only the bare
  emitter (`1416-1997-1-ND`, ×20 on hand).
  - [ ] received — notes:
- [X] **C-12** *(O3-9; NOW FIRM — operator 2026-07-26)* **L1IZ-0850** — **✔ RECEIVED ×10 (DK-3 line 6)**:
  Lumileds `L1IZ-0850000000000` (`1416-1997-1-ND`). The 5-LED bench string is a **permanent bench
  fixture** — it does NOT donate to the flight cube. Cube takes 5, rest are spares/second-cube stock.
  - [X] received — notes: ×10, packlist 129837577 line 6 (lot 251027611586). Bench total now ×20 with
    order 1's reel (5 committed to the bench string).
- [X] **C-13** *(O2-1)* ~~Sense resistor 3.74 Ω 1 % 1206~~ — **✔ CLOSED: DK order 2 delivered ×10**
  (`541-3.74FFCT-ND`, 1/4 W); field 0.62 Ω also confirmed ×10 in order 1 (`CRL1206-FW-R620ELF`, 1/2 W).
  - [X] received — notes: packlists 128637262 line 1 / 127789318 line 6.
- [ ] **C-26** *(NEW 2026-08-07; **DECIDED 2026-08-10: ATtiny412**; **ORDER NOW 2026-08-17 — gates the two-cube build**)* **Bare flight MCU = ATTINY412
  SOIC-8 ×10** (sweet-spot qty, ~$0.60 ea) — operator call: minimal pod, back to the original design
  target (schematic.md/verified-bom were 412 from the start). Consequences, accepted: **no TOSC pins →
  the crystal path is FORECLOSED for the pod → RC-osc is committed** (the decoder side already carries
  it: per-beacon DPLL + measured +2.6 % skew tolerance, bench-verified). **F-section (C-21/C-22/C-23)
  closes as MOOT** — the RC-vs-crystal study's only purpose was this decision. Firmware: 416→412 is a
  pin remap (same core/peripherals); SOIC-8→DIP adapters ✔ on hand (C-15); programming = serial UPDI
  (TTL-232R-3V3 + 1–4.7 k + `pymcuprog`), no new hardware.
  - [ ] received — notes:

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
  ✅ **THE ELP ALL-IN-ONE IS THE RIGHT PART, and now for an optical reason (2026-08-04).** The 40 nm-class
  decision was taken on cost; it is also correct on physics. A dielectric bandpass blue-shifts with AOI,
  and behind the lens the marginal ray arrives at arctan(1/2F#) — 17.4° at F/1.6, 14.0° at F/2.0 — needing
  a **≥26 nm / ≥17 nm** passband respectively just to keep the outer pupil in band. A true 10 nm filter
  mounted internally would stop the lens down and throw away the aperture gain it was bought for. The
  Thorlabs deferral to the 040 camera stands, but note it must be **front-mounted** there too, not dropped
  behind the optic. Full working: [`lensed_pd_range.py`](../../specs/031-beacon-camera/lensed_pd_range.py) §7.
  📊 **Predicted reach, anchored on the 41 ft bench lock** (not on datasheet flux — see the script's
  calibration note). Aperture gain is referenced to the **BPV10NF** the flux constant was measured on;
  BPW34 at the focal plane sets FOV (9.8°) and ambient, not signal:

  | configuration | F/1.6 | F/2.0 | F/2.4 |
  |---|---:|---:|---:|
  | 1 die @ 306 mA | **115 m** | 92 m | 77 m |
  | flight cube 5 @ 306 mA, face-on | 145 m | 116 m | 96 m |
  | flight cube 5 @ 306 mA, edge-on | 174 m | 139 m | 116 m |

  Consistent with this entry's own ~75–100 m estimate, and **100 m is reachable at every F/#** for the
  full cube. Range scales **linearly with pupil diameter**, so F/# is the dominant unknown — measure the
  aperture gain functionally on the first lensed run rather than deriving it (bench-journal item 7).
  ⚠️ **But the sun disc in the field is over EVERY R_load ceiling once the lens is fitted** (209–952 µA vs
  70/150/330 µA at 47 k/22 k/10 k). The 22 k pairing rule below buys linearity for *diffuse* sun only. See
  the lens-era note in the daylight doc and the max-energy-gate work item (bench journal 2a).
  🔴 **PRIORITY RAISED 2026-08-02 by field test #4**: at ~6× emitter current (6 × 50 mA) the link locks to
  ~20 ft with the PD **shaded** but fails outright with the PD **in direct sun** — same emitter, same
  distance, shadow alone flips it. Compression at the PD sits upstream of every other multiplier, so more
  emitter current cannot buy back direct-sun operation. **The bandpass is the gate on the whole range
  roadmap, not an optimization.** (Bench-journal field test #4.)
  **STATUS 2026-08-05 — LENS ARRIVED, IN VALIDATION**: ✔ **M12 lens holder received (AMZ-1)** — uxcell
  S-mount ×10, 20 mm screw spacing. ✔ **Lens received 2026-08-05**: barrel `16mm 5MP 1/2.5" IR` —
  **integrated-filter style** (opaque to visible, passes 850: emitters glow Bayer-purple through it), so
  no separate bandpass needed for this rig; entrance pupil eyeballs ~8 mm ⇒ F/2-class as hoped.
  ⚠ **NOT yet validated as an imager**: back-focus/focus hunt ongoing (bench-journal lens arm) — flat ×1.7
  gain in the first PD sweeps, image-plane localization in progress via knife-edge / Pi-cam-v2.1-bare-sensor
  route. **Hold the "received ✔" tick until it demonstrably focuses**; if it fails the sensor test it goes
  back and a replacement 16 mm F/2 M12 IR lens gets ordered.
  ⚠ **couples to the Option-C R_load** (pedestal ceiling 3.3 V/R): cheap discs are **40 nm-class →
  R_load 22 k** (47 k reserved for a true 10 nm filter) — see the daylight doc's pedestal table. Note the
  9.5° FOV itself is ambient rejection (sky-only background when aimed at a flying beacon).
  - [ ] received — notes:
- [X] **C-15** *(O2-7; demoted by Option C)* TSSOP/MSOP-8→DIP adapter — **✔ RECEIVED (AMZ-1)**: Stargazer
  SMD→DIP breakout for **SOIC-8 / TSSOP-8 / MSOP-8 / VSOP-8** ×5 with gold-plated headers (substitute for
  the Aries LCQT-TSSOP8). The OPA381 precision-TIA path is now unblocked if it ever revives (OPA381 ×2 on
  hand); also retroactively covers the dropped C-7 SOIC-8→DIP need.
  - [X] received — notes: ×5, AMZ-1 2026-07-30.
- [X] **C-25** *(NEW 2026-07-25)* ~~1N4148 small-signal diodes~~ — **DROPPED 2026-07-26** (Q5: on hand in
  the drawer; D2/D3 clamps covered).

## E — Bench consumables
- [X] **C-16** *(O3-17, absorbs O2-9/O2-10; TRIMMED 2026-07-26)* **Passives assortment kits (THT)** —
  **✔ BOTH RECEIVED (AMZ-1)**: **BOJACK 50-value 1350 pc resistor kit**, 0 Ω–5.6 MΩ, 1 % ¼ W metal film
  (covers Option-C R_load 47 k/22 k, R5 1–2 k, C-2's 1 k) and **ALLECIN 24-value monolithic ceramic cap
  kit** 10 pF–10 µF (10/20/30/47/56/68/100/220/330/680 pF, 1/4.7/10/47/100 nF, 0.15–0.68 µF, 1/2.2/4.7/10 µF).
  ⚠ The cap kit has **no 820 pF** — nearest for C5 is 680 pF or 1 nF (or 680 pF ∥ 150 pF); pick when the
  Option-C filter corner is set. Electrolytic + film kits stay OPTIONAL (DK order 2's 100 nF ×50 / 10 µF ×10
  cover C6/C7/C8).
  - [X] received — notes: both kits, AMZ-1 2026-07-30.
- [X] **C-17** *(O2-11)* ~~High-temp wire~~ — **DROPPED 2026-07-26** (Q6: wire stock confirmed on hand).
- [X] **C-18** *(O2-12)* ~~Bare tinned bus wire~~ — **DROPPED 2026-07-26** (Q6: on hand).
- [X] **C-19** *(O2-8; CLOSED 2026-07-30)* Solderless breadboard **BB830** — **✔ ×1 received (DK order 2)**,
  plus **✔ Chanzon SYB-170 mini breadboards ×6 (170 tie points, adhesive back) from AMZ-1** — small
  single-block rigs (emitter head, receiver front-end) no longer compete for the BB830. No further
  breadboard purchases.
  - [X] received — notes: BB830 ×1 packlist 128637262 line 3; SYB-170 ×6 AMZ-1 2026-07-30.
- [X] **C-20** *(O2-2; CLOSED 2026-07-30 via C-8)* **Ground-plane / thermal copper board** — the MG 540
  5″×3″ line is **dropped**: the C-8 uxcell sheets (70×50 mm double-sided, ×5) are shear stock for both
  the LED tiles and any ground-plane/thermal coupon. The SMT3U ×2 (DK order 2) remain SOIC-pad carriers
  with no plane; C-6 adapters are the SOT-23 mounting fix.
  - [X] received — notes: covered by C-8 sheets, AMZ-1 2026-07-30; SMT3U ×2 ✔ (SOIC carriers only).

## F — RC-vs-crystal stability study — **CLOSED AS MOOT 2026-08-10** (C-26 decision: ATtiny412 = no
TOSC = RC-osc committed for the pod; the study's only purpose was this choice. Decoder side already
carries RC drift: per-beacon DPLL, +2.6 % skew bench-verified.)
- [X] **C-21** *(O2-4)* ~~32.768 kHz watch crystal ×3~~ — MOOT (412 has no TOSC).
- [X] **C-22** *(O2-5)* ~~exact 18 pF / 6.8 pF load caps~~ — MOOT (10/20/30 pF from the AMZ-1 kit remain
  general stock).
- [X] **C-23** *(O2-6)* ~~20 MHz XO + LDO fallback~~ — MOOT.

- [ ] **C-27** *(NEW 2026-08-17)* **850 nm bandpass DISC for the 1.8 mm fisheye** — Quanmin 12.5 mm × 1.0 mm
  pair (~$12, Amazon) or 10.4 mm variant: **measure the M12 holder's internal bore first.** **~40 nm-class,
  NOT 10 nm**: the LED line is ~30 nm (10 nm keeps only ~30 % of our signal) and fisheye rear CRA of 25–35°
  blue-shifts a passband ~5+ nm — half of a 10 nm window. Mount on the holder step, mechanical capture
  (O-ring/foam), no CA near optics. Purpose: empirical #1 (sky background) + outdoor camera work.
  - [ ] received — notes:

## Separate supplier
- [X] **C-24** *(O3-8, O2-14 caveat; **RESOLVED 2026-08-07**)* ~~1S 100 mAh flight pack~~ — **operator
  decision: the 1S 150 mAh packs ARE the flight hardware.** No 100 mAh buy; the bench pack and flight
  pack are now the same type, which also dissolves the original rationale (jitter test wanted flight-pack
  ESR — it now measures it by definition). Buy more of the same 150 mAh SKU only if pack count runs short.
  - [X] received — notes: closed by decision, no purchase.

## Already on hand — do NOT re-order (reconciled against both DK packlists, 2026-07-26)
- **Receiver**: BPV10NF ×2, BPW34 ×10, MCP6022 ×2, MCP3201 ×2, MCP1525 ×2, OPA381 ×2, 1 MΩ 3296W
  trimmer ×2.
- **Emitter**: L1IZ-0850 **×20** (order 1 ×10 + DK-3 ×10; 5 committed to the bench string), LM3410X ×10 −
  attrition, SS1030 ×10 − attrition, 22 µH B82464 ×10 (bench) + **SPM4020T-4R7M-LR 4.7 µH ×9 (flight, +1
  backordered)**, 0.62 Ω 1206 ×10, 3.74 Ω 1206 ×10, **BZX55C15 zener ×10**, **1 k 0805 ×10**,
  **Molex 53047-0210 UMX header ×10**, XNANO **×2 working** (1 survivor + 1 new).
- **Passives (0603 SMD, ×10 each)**: 10 k, 100 k, 1 M, 10 M, 2 pF C0G, 100 nF, 1 µF (0805), 2.2 µF,
  4.7 µF (1206). **THT**: 100 nF radial ×50, 10 µF radial ×10, **BOJACK resistor kit** (50 values,
  0 Ω–5.6 MΩ, 1 % ¼ W metal film, 1350 pc) and **ALLECIN MLCC kit** (24 values, 10 pF–10 µF) — both AMZ-1.
- **Misc**: JST-PH 2.0 mm S2B ×10 (NOT the UMX 1.25 mm pitch — see C-10), SOT23→DIP ×2 + **SOT23-6/SC70-6
  →DIP ×20 (AMZ-1)**, **SOIC/TSSOP/MSOP/VSOP-8→DIP ×5 (AMZ-1)**, SMT3U ×2, BB830 ×1 + **SYB-170 mini
  breadboards ×6 (AMZ-1)**, **copper-clad FR4 70×50×1.0 mm ×5 (AMZ-1)**, **M12 S-mount lens holders ×10
  (AMZ-1)**, TTL-232R-3V3 cable, microSD breakout ×2, 1S 150 mAh bench LiPo (Amazon),
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

## Still outstanding (updated 2026-08-07 — DigiKey FULLY CLOSED: C-9's 10th inductor landed)

Sections A and C complete; battery decision made (C-24: **150 mAh packs ARE flight hardware**, no buy).
Remaining:
1. **C-14 optics validation** — lens ARRIVED 2026-08-05 (integrated 850 filter confirmed; F/2-class
   pupil) but has NOT yet demonstrated focus — back-focus hunt in progress (bench-journal lens arm).
   If it fails the bare-sensor test → return + replacement buy. **Still the gate on the 100 m test.**
2. **C-26 bare flight MCU — DECIDED: ATTINY412 SOIC-8 ×10** (operator 2026-08-10). The one DigiKey buy
   left on this order; goes in the same cart as the order-04 era.
3. ~~C-21/C-23 F-section~~ — **CLOSED AS MOOT 2026-08-10** (412 = no TOSC = RC-osc committed; see §F).
4. **C-11** Luxeonstar MCPCB — only if the C-8 tiles disappoint · **C-19/C-16** closed, nothing to buy.
5. **Contingency only**: another XNANO (spare margin is gone — see C-4) · more 150 mAh packs if count
   runs short.

## Open decisions gated by this order
1. ~~Filter FWHM (C-14): 40 nm vs 10 nm~~ — **RESOLVED 2026-07-26: budget 40 nm-class (M12 stack) for
   the single-pixel test → Option-C R_load = 22 k**; true 10 nm (Thorlabs) deferred to the 040 camera
   where the 2-D array earns it. (Pedestal-ceiling math in the daylight doc.)
2. RC vs 32k-crystal vs XO (F section) — still gated on the copper-board load-test jitter measurement.
