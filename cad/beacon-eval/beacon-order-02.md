# Beacon Bench — Order 02 (2026-07-05)

**What this is**: the second bench parts order for the 031 beacon work — a follow-on to
**Order 01** (the initial eval build: emitter [`verified-bom-eval.md`](verified-bom-eval.md)
+ receiver [`../beacon-receiver/eval-loop-bom.md`](../beacon-receiver/eval-loop-bom.md)). Kept as a **separate
doc** so the purchase history reads chronologically instead of mutating a single BOM.

**Drivers**:
1. Move the emitter off the smoking breadboard to **field power (300 mA)** on a copper thermal board.
2. Build the platform to **measure RC-osc jitter under real pulsed load + battery sag** — jitter is
   *unknown* until this test (see Open Decisions).
3. Stage the **RC-vs-crystal** stability-comparison hardware.
4. Firm up the **receiver photodiode** front end.
5. Quieter breadboard prototyping (through-hole decoupling vs tiny SMT).

---

## DigiKey line items

### Emitter — bench current + field-power thermal
- [ ] **O2-1** Sense resistor **3.74 Ω 1 % 1206** — 6× the 0.62 Ω field value → **~51 mA bench** (protects the
  un-heatsunk breadboard LED string). Dissipates ≤ 58 mW even at field power → any 1206. Qty **3**.
  (Schematic R1 already changed to 3.74 Ω; field value 0.62 Ω preserved in the eval BOM.)
  - [ ] received — notes:
- [ ] **O2-2** **Copper SMT prototyping board w/ ground plane** — **BusBoard SMT3U** (SMT pads + solid copper
  ground plane; Amazon/BusBoard-direct) **or** DigiKey **MG Chemicals 540** (double-sided 1 oz copper-clad
  FR4, 5″×3″, Manhattan/dead-bug). Serves double duty: **field-power thermal path** for the LEDs **and** a
  clean **ground plane** so the jitter measurement isn't dominated by breadboard artifacts. Qty **1–2**.
  - [ ] received — notes:
- [ ] **O2-3** *(optional, LED-specific thermal)* **Luxeonstar SZ-01-R8** — L1IZ-0850 pre-reflowed on a
  10 mm² aluminum MCPCB, **4.8 °C/W**, thermal test point (0.42 W/LED → ~2 °C rise). Zero-reflow field-power
  LED mount. Qty up to **5** (series string) + a small finned heatsink. *Out-of-stock 2–7 day lead as of 2026-07.*
  - [ ] received — notes:

### Emitter — RC-vs-crystal stability comparison (decision **gated by the O2-2 load test**)
- [ ] **O2-4** **32.768 kHz watch crystal** (through-hole cylinder for breadboard; e.g. Citizen CFS-206 /
  Abracon AB26T, CL 12.5 pF) — driven by the 416's on-chip **XOSC32K** driver, which works the full
  1.8–5.5 V (LiPo-range clean) and uses **TOSC = PB2/PB3** (no PA3/DIM conflict). Qty **3**.
  **⚠ 416-only**: the 8-pin **ATtiny412 has no PORTB/TOSC** → a shipping pod on this path needs a ≥14-pin
  tinyAVR (e.g. ATtiny414/1614).
  - [ ] received — notes:
- [ ] **O2-5** Crystal load caps — a few **18 pF** and **6.8 pF** (match crystal CL). Breadboard stray detunes
  the absolute frequency, but **stability — the thing under test — is preserved** (the DPLL measures the
  actual rate). 
  - [ ] received — notes:
- [ ] **O2-6** *(fallback path — order only if pursuing the XO route)* **20.000 MHz CMOS XO**, TH half-can,
  **+ a small 3.3 V LDO**. Raw LiPo 3.5–4.2 V is out of range for standard 3.3 V (~≤3.6 V) / 5 V (≥4.5 V) XOs,
  so it must be regulated. Feeds **EXTCLK/PA3 → DIM must move to PA6/PA7**. (tinyAVR-1 has **no PLL**, so an
  external HF clock is the only crystal-grade *core* clock.) Qty **2** XO + **1** LDO.
  - [ ] received — notes:

### Receiver — front-end
- [ ] **O2-7** **8-pin TSSOP/MSOP-8 → DIP** adapter, 0.65 mm pitch (Aries **LCQT-TSSOP8** / SchmartBoard ez /
  Chip Quik). Unblocks the **OPA381 (MSOP-8)** precision-TIA option in the receiver BOM. Qty **2**.
  - [ ] received — notes:

### Bench general — breadboard + decoupling
- [ ] **O2-8** Solderless breadboard **BusBoard BB830** (DigiKey `2864-BB830-ND`), 830 tie-points, 4 rails.
  Qty **2**.
  - [ ] received — notes:
- [ ] **O2-9** Decoupling caps (HF) **100 nF 50 V X7R radial** (KEMET `C320C104K5R5TA`) — through-hole,
  breadboard-friendly vs the SMT 0603. Qty **25**.
  - [ ] received — notes:
- [ ] **O2-10** Bulk rail caps (LF/transient) **10 µF 25 V radial electrolytic** — hold the rail up during the
  ~1.27 A pulsed-LED load steps (pairs with the O2-9 HF caps; matters for the osc-stability measurement). Qty **5**.
  - [ ] received — notes:

### Prototyping consumables — wire (for the O2-2 copper board)
- [ ] **O2-11** **High-temp insulated wire** (won't recede under a 300–350 °C iron — the reason NOT to use
  Kynar/PVC wire-wrap wire, which is ~105 °C and melts back when you park the iron on it):
  - **PTFE (Teflon), silver-plated, 30 AWG** — fine point-to-point **SMT signal jumpers**. 200 °C rated,
    ~327 °C melt, holds its shape; the standard for hand-soldered rework (e.g. Alpha Wire PTFE, or a
    multi-color 30 AWG PTFE hookup kit). Qty **1 multi-color kit** or 1–2 spools.
  - **Silicone-insulated stranded, ~22–24 AWG** — flexible **power leads** (boost/LED path). 200 °C rated,
    very flexible, iron-tolerant. Qty **1–2 colors**.
  - [ ] received — notes:
- [ ] **O2-12** **Bare solid tinned-copper bus wire, ~22 AWG** (spool). **Thru-hole vias / ground-plane
  stitching** (push through a drilled hole, solder both faces) **+** heavier power jumpers — 22 AWG carries
  the 300 mA–1.27 A boost/LED path with ease. 22 AWG ≈ 0.64 mm → drill vias ~0.7–0.8 mm (#68–70). Add a
  **24 AWG** spool too if you want tighter vias / lighter signal runs. Qty **1 spool** (+1 if 24 AWG).
  - [ ] received — notes:

### Power source (bench)
- [x] **O2-14** **1S LiPo, 150 mAh, 20C, JST-PH 2.0 pigtail** — *purchased 2026-07 (Amazon)*. **Bench
  substitute** for the spec's 1S **100 mAh** (out of stock). Mates the **S2B-PH-K-S** header from Order 01.
  Fine for bench characterization; ~14 min at 50%-duty field power. **Caveat for the definitive jitter test**:
  150 mAh has slightly lower internal resistance than the 100 mAh flight pack → slightly *less* rail droop →
  an optimistic osc-pull reading. Develop/run the test on this, but take the **final RC-vs-crystal jitter
  measurement on an actual 100 mAh flight pack** (real worst-case droop) before deciding.
  - [x] received — notes: bench-substitute; flight-representative = 100 mAh (restock)

## Optional — separate supplier (Thorlabs / m12lenses)
- [ ] **O2-13** *(daytime-range test only)* Narrow **850 nm bandpass** filter (Thorlabs **FB850-10**,
  10 nm FWHM) **+** small **M12 collection lens** (m12lenses.com PT-02120). The BPV10NF's built-in daylight
  filter (780–1050 nm) already covers bench + pointed field; add this only for the rigorous daytime-at-100 m
  link-budget test.
  - [ ] received — notes:

---

## Already on hand — do NOT re-order
- **BPV10NF** (the "940 radial") — **primary receiver photodiode**: built-in daylight-blocking filter
  (780–1050 nm), ±20° + integrated dome lens, **11 pF** junction cap (clean TIA). Best for pointed bench/field.
- **BPW34** (the "900") — wide-FOV (±60°) alternate: **0.62 A/W @ 850 nm** (near-optimal) but **unfiltered**
  (needs O2-13 outdoors). "900" = peak-λ (really 920 nm), not a mislabel.
- **L1IZ-0850** IR LEDs (5) — emitter string. Vf ≈ 2.75 V @ 300 mA.

## Open decisions gated by this order
1. **416 VTG rail (3.3 V vs 5 V)** — sets the O2-6 XO/LDO voltage; only matters if the XO path is pursued.
   The production 412 runs **raw off 1S LiPo (3.5–4.2 V)**.
2. **RC vs 32 k-disciplined vs 20 MHz XO** — **UNDECIDED, gated by the field-power load-test jitter** on the
   O2-2 copper board. The earlier ~0.05 % figure was **USB-powered / no-load** and does **not** predict field
   jitter (pulsed 1.27 A on the shared rail + battery sag + thermal + boost switching). Build → run at 300 mA
   off a 1S cell → **measure** → then decide. See BACKLOG **[031 — STUDY]**.

## Rationale trail (this session)
- Sense-R 6× de-rate + power budget: emitter power/cooling (Vf 2.75 V @ 300 mA → ~1.27 A peak battery draw).
- Copper board = field-power thermal **and** the ground plane that makes the jitter measurement valid.
- Crystal notes: tinyAVR-1 has **no PLL/multiplier** (can't speed 32 k up to a core clock); **XOSC32K** is an
  on-chip *driver* for an *external* quartz crystal (not a built-in crystal); the internal 32 k **OSCULP32K**
  is an RC (~±30 %, useless for stability); the **412 lacks TOSC** pins.
