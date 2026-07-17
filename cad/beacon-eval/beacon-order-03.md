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

## Related but $0 (firmware/fuses — not parts)
- **F_CPU 20→10 MHz** + **BOD fuse ~2.6 V** (tasks.md **A2-pwr**): the 416 at 20 MHz is out of its
  ≥4.5 V speed-grade over the whole 1S range; 10 MHz is in-spec 2.7–5.5 V. No hardware needed.
- **Firmware-ADC UVLO @ 3.5 V** (T027–T031, R11): the *designed* low-battery cutout — not yet written;
  the 2026-07-16 ramp confirmed nothing trips until the LM3410X's own 2.7 V floor.
