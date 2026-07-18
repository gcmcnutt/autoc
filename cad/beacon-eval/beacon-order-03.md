# Beacon Bench — Order 03 (prep 2026-07-16)

**What this is**: third bench parts order — follow-on to [Order 01](verified-bom-eval.md) /
[Order 02](beacon-order-02.md), kept separate so purchase history reads chronologically.

**Drivers**:
1. **Attrition**: the IR-drive bring-up cost a few parts (open-LED boost overvoltage before the OVP
   clamp existed — likely killed an LM3410X or two **and one ATtiny416-XNANO**).
2. **OVP provision** (verified-bom-eval **EV-A13/A14**, orthogonal check #4): clamp the open-string
   boost runaway at ~15 V so the next LED/connector letting go doesn't eat another 416.
3. **Mounting**: Order-02's SMT breadboards have **no SOT-23 footprint** → back to Order-01's
   SOT-23-5→DIP adapters as the LM3410X carrier (receiving note, beacon-order-02.md).

---

## DigiKey line items

### OVP clamp (new — EV-A13/A14)
- [ ] **O3-1** **15 V zener, THT**: `BZX55C15` (DO-35, 500 mW) — qty **5** (cheap; blowout-class part).
  Alt/heavier: `1N4744A` (DO-41, 1 W). Cathode → V_OUT, anode → FB pin. Clamp ≈ 15.2 V.
  - [ ] received — notes:
- [ ] **O3-2** **1 kΩ 5 % THT ¼ W** — qty **10** (R4, series FB→sense; generic — skip if drawer stock).
  ⚠️ NOT optional with the zener (see EV-A14: without it the clamp carries full LED current).
  - [ ] received — notes:

### Attrition replacements
- [ ] **O3-3** **LM3410XMF-NOPB** SOT-23-5 boost driver — qty **4** (order-01 A1 spares consumed in
  blowouts; the OVP makes these last longer now).
  - [ ] received — notes:
- [ ] **O3-4** **ATTINY416-XNANO** eval kit — qty **1–2** (one broken 2026-07; each new unit needs its own
  one-time admin `usbipd bind --force` + the R100 cut before target-rail testing).
  - [ ] received — notes:
- [ ] **O3-5** **SS1030 Schottky** (Panjit `SS1030_R1_00001`, DK `3757-SS1030_R1_00001CT-ND`, SOD-123)
  — qty **3** spares (boost rectifier sits on the node that rails during the open-LED event).
  - [ ] received — notes:

### Mounting (re-order of Order-01 EV-D3 — the working LM3410X carrier)
- [ ] **O3-6** **SOT-23-5→DIP adapter**: SchmalzTech **`ST-SOT23-5`** (exact footprint) — qty **4**
  (each blown LM3410X takes its adapter with it in practice; Chip Quik `PA0089`/`PCB3007-1` alt).
  - [ ] received — notes:

### Optional / carry-overs
- [ ] **O3-7** *(carry-over O2-3, was out of stock)* **Luxeonstar SZ-01-R8** MCPCB-mounted L1IZ-0850,
  qty up to **5** + small finned heatsink — required before restoring **field power (0.62 Ω / 306 mA)**;
  bench stays at 3.74 Ω / ~51 mA until this mounts.
  - [ ] received — notes:
- [ ] **O3-8** *(separate supplier — carry-over from O2-14 caveat)* **1S 100 mAh LiPo flight pack**
  (JST-PH) — the definitive RC-osc jitter measurement wants the real pack's internal resistance.
  - [ ] received — notes:
- [ ] **O3-9** *(optional)* **L1IZ-0850** IR LED spares — qty **5** if any of the string took damage in
  the blowouts (check before ordering; 10 were bought on order-01).
  - [ ] received — notes:

## § Flight-emitter cube + 100 m field receiver (added 2026-07-18)

> **Goal**: actual FLIGHT emitter hardware — 5× L1IZ-0850 at **306 mA** on 5 faces of a glued cube, real
> flight battery, OVP-hardened (O3-1/O3-2), firmware UVLO — plus a ground receiver with **IR filter +
> collection lens** → the **100 m field-conditions test**. Assemblable with what's below + on-hand stock.

- [ ] **O3-10** *(bump of O3-6)* **SOT-23-5→DIP adapters** (SchmalzTech `ST-SOT23-5`) — raise to qty **8
  total** (order-01 stock consumed; every LM3410X build eats one).
  - [ ] received — notes:
- [ ] **O3-11** **850 nm bandpass filter + collection lens** (promotes O2-13 from optional → REQUIRED for the
  100 m goal): Thorlabs **FB850-10** (Ø25 mm, 10 nm FWHM) + **M12 lens** (m12lenses.com PT-02120 class) +
  an M12 lens holder/mount. The lens is the ×10–25 range multiplier (optical-link-outcome.md); the filter is
  what makes daylight survivable.
  - [ ] received — notes:
- [ ] **O3-12** *(bump of O3-4)* **ATTINY416-XNANO** — firm qty **2**.
  - [ ] received — notes:
- [ ] **O3-13** **ATtiny412 + SOIC-8→DIP adapters**: the 412 has **NO DIP package** (tinyAVR-1 8-pin = SOIC-8
  `ATTINY412-SSFR` only — verify the ×6 from the parent order arrived; add spares if not). Order **SOIC-8→DIP
  adapters qty 6** (SchmalzTech ST-SOIC-8 / Chip Quik PA0018 / Adafruit 1212) for breadboard + cube harness work.
  - [ ] received — notes:
- [ ] **O3-14** **Single-LED thermal tiles (cube faces)**: **thin double-sided copper-clad FR4, 0.5–0.8 mm,
  1 oz+** (e.g. MG Chemicals small sheet) to shear into **~10×10 mm tiles**, one L1IZ-0850 hand-reflowed per
  tile (knife-slit the copper into anode/cathode islands; stitch both faces for spreading). 5 tiles glue up as
  cube faces. Thermal sanity at 306 mA/50 % duty ≈ 0.2 W avg/LED × ~70 °C/W for a 1 cm² tile ≈ **~15 °C rise**
  — acceptable; validate with the first tile before gluing five. (Alternative if lead time allows: Luxeonstar
  SZ-01-R8 MCPCB, O3-7.) Sheet qty: 1 (yields dozens of tiles + spares).
  - [ ] received — notes:
- [ ] **O3-16** **Flight-size boost inductor** — the order-01 22 µH (TDK B82464G4223M000, ~10×10×5 mm) is
  bench-fine but too big/heavy for the cube. At the LM3410**X**'s 1.6 MHz, **4.7–10 µH** is the right range:
  at 306 mA LED (I_in ≈ 1.05 A avg, peak ~1.3 A), 4.7 µH → ~0.3 A p-p ripple (≈30 %, textbook). Spec:
  **shielded, Isat ≥ 2 A, 4×4×2 mm class**. Primary: Coilcraft **XFL4020-472ME** (4.7 µH, Isat ~3 A) qty **3**;
  alt **XAL4020-103** (10 µH) or Murata/TDK equivalent. Keep the switching loop tight on the tile stack (FR-1.6).
  - [ ] received — notes:
- [ ] **O3-15** **Battery connector, 1.25 mm pitch, male PCB side** — ⚠️ naming trap: batteries sold as
  "JST PH 1.25" are usually **Molex PicoBlade-compatible** (real JST PH = 2.0 mm; JST's true 1.25 mm family is
  GH, which has a latch). **Verify against the actual battery plug before ordering** (photo/measure; latch ⇒ GH).
  Default: Molex PicoBlade **53047-0210** (THT vertical) qty **6** (+ 53048-0210 SMT alt). These flight packs
  replace the JST-PH 2.0 harness from order-01 on the pod side.
  - [ ] received — notes:

## Related but $0 (firmware/fuses — not parts)
- **F_CPU 20→10 MHz** + **BOD fuse ~2.6 V** (tasks.md **A2-pwr**): the 416 at 20 MHz is out of its
  ≥4.5 V speed-grade over the whole 1S range; 10 MHz is in-spec 2.7–5.5 V. No hardware needed.
- **Firmware-ADC UVLO @ 3.5 V** (T027–T031, R11): the *designed* low-battery cutout — not yet written;
  the 2026-07-16 ramp confirmed nothing trips until the LM3410X's own 2.7 V floor.
