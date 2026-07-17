# Collector (Receiver) Schematic — wire-level

**Status**: DRAFT design, 2026-06-18 — wire-level reference for the single-IR-sensor collector. KiCad capture is the next step (`cad/beacon-receiver/beacon-receiver.kicad_sch`, not yet created).
**Spec ref**: [acquisition-research-plan.md](../../specs/031-beacon-camera/acquisition-research-plan.md) §5 (receiver chain), [eval-loop-bom.md](eval-loop-bom.md) (parts).
**Chain**: `PD → TIA → [DC tap | AC-couple] → ADC → StepFPGA`, plus an optional hard-1-bit comparator.

Single 3.3 V rail (from the StepFPGA or a bench/USB 3.3 V). Signal BW is low (~100–200 Hz code), so the design is forgiving — jellybean parts, breadboard-friendly.

## Reference-designator ↔ BOM cross-reference

| Refdes | Description | Part |
|---|---|---|
| D1 | Photodiode (swappable) | **BPV10NF** (narrow-aimed 100 m) **or BPW34** (wide-FOV all-attitude) — via J2 so it can sit in the cone on a cable |
| U1 | Dual op-amp | **MCP6022-I/P** — A = TIA, B = Vbias buffer |
| U2 | 12-bit SPI ADC | **MCP3201-CI/P** (pseudo-differential: IN+ = signal, IN− = Vbias) |
| U3 | 2.5 V voltage reference | **MCP1525** → ADC Vref + Vbias divider source |
| U4 | Comparator (optional) | **LM393P** (or TLV3201) — hard-1-bit baseline |
| R1 | TIA feedback (gain) | **1 MΩ** fixed, or the **Bourns 3296 1 MΩ trimpot** for a continuous sweep |
| C1 | TIA compensation | **2 pF** C0G across R1 |
| R2, R3 | Vbias divider off 2.5 V → ~0.5 V | 40 kΩ / 10 kΩ (or any ratio giving ~0.5 V) |
| C2 | AC-couple cap (optional path) | **1 µF** film/X7R |
| R4 | AC re-bias to Vbias | 1 MΩ |
| R5, R6 | Comparator threshold (optional) | divider/trimpot off 2.5 V |
| C3..C6 | Decoupling | 1 µF + 100 nF at each IC supply |
| J1 | StepFPGA interface header | 3V3, GND, SPI_CS, SPI_CLK, SPI_DOUT, COMP_OUT |
| J2 | Photodiode header / cable | PD_A (anode), PD_K (cathode) |
| J3 | Power in (if not from FPGA) | 3V3, GND |

## Nets

| Net | Pins | Role |
|---|---|---|
| **3V3** | U1/V+, U2/VDD(8), U3/VIN, U4/V+, C3..C6/+ , J1, J3 | supply |
| **GND** | U1/V−gnd ref, U2/VSS(4), U3/GND, U4/GND, dividers, C*/−, J1, J2, J3 | ground |
| **VREF2V5** | U3/OUT, U2/Vref(1), R2 top, R5 top | clean 2.5 V — ADC full-scale + bias/threshold source |
| **VBIAS** (~0.5 V) | R2/R3 node → U1B/+in; U1B/out (buffered) → U1A/+in, U2/IN−(3), R4 | TIA quiescent + ADC IN− (pseudo-diff cancels the offset) |
| **PD_NODE** | D1 (via J2) ↔ U1A/−in (1), R1, C1 | TIA summing junction (virtual Vbias) |
| **TIA_OUT** | U1A/out, R1, C1, C2 (AC path), → ADC_IN (DC jumper) | TIA output = Vbias + I_photo·R1 |
| **ADC_IN** | U2/IN+(2), [jumper: TIA_OUT (DC) or AC node], U4/−in | what the ADC samples |
| **AC_NODE** | C2/out, R4 (→ VBIAS) | DC-stripped code (optional) |
| **SPI_CS / SPI_CLK / SPI_DOUT** | U2/5, U2/7, U2/6 → J1 → StepFPGA hard-SPI | ADC readout |
| **COMP_OUT** | U4/out → J1 → StepFPGA GPIO | optional hard-1-bit |

## ASCII schematic

```
 3V3 ──┬───────────────┬──────────────┬─────────────┬─────── 3V3
       │               │              │             │
     [U3 MCP1525]    [U1 MCP6022]   [U2 MCP3201]  [U4 LM393]
       VIN            V+ (8)          VDD (8)       V+
       OUT ── VREF2V5 ─┬─────────────► Vref (1)      │
       GND             │   R5/R6 div ──────────────► −in (thresh, opt)
        │              │
        │            R2 40k
        │              ├── VBIAS(~0.5V) ──► U1B +in
        │            R3 10k
        │              │        U1B out ─┬─► U1A +in
       GND            GND                ├─► U2 IN− (3)   [pseudo-diff ref]
                                         └─► R4 ─┐
                                                 │
   D1 PD (via J2)                                │
     anode/cathode ── PD_NODE ──► U1A −in (1)    │
                          │                      │
                          ├── R1 1MΩ (trimpot) ──┤  (feedback)
                          └── C1 2pF ────────────┤
                                                 │
                       U1A out = TIA_OUT ────────┴───┬──────────────► (DC jumper) ─┐
                                                     │                              │
                                                C2 1µF ── AC_NODE ─ R4→VBIAS ──(AC jumper)─┤
                                                                                    │
                                                                            ADC_IN ─┴─► U2 IN+ (2)
                                                                                    └─► U4 −in (opt)

   U2 MCP3201:  CS(5) CLK(7) DOUT(6) ──► J1 ──► StepFPGA hard-SPI
   U4 LM393:    out ─────────────────► J1 ──► StepFPGA GPIO (COMP_OUT, optional)
```

## Design notes

1. ~~**Pseudo-differential ADC is the trick.**~~ ⚠️ **RETRACTED 2026-07-16** — the MCP3201's IN− is
   spec-limited to ±100 mV about VSS (see revised choice #3 below), so IN− = VBIAS is out of spec.
   **As-built: IN− = GND (single-ended), IN+ = TIA output, Vref = 2.5 V.** The ADC reads
   `VBIAS + I_photo·R1` absolute; the decoder's DC tracker strips the ~0.45 V pedestal.
2. **DC vs AC jumper.** DC tap (TIA_OUT → ADC) keeps the ambient pedestal → AGC/envelope studies (but bright sun can eat range). AC tap (C2 + R4 re-bias) strips solar DC → full range for the code. Keep both reachable; pick per experiment.
3. **TIA gain.** R1 = 1 MΩ start (covers ~0.1–2 µA). Use the 1 MΩ trimpot to sweep gain for the link-budget / saturation study. C1 = 2 pF tames ringing (omit if none).
4. **PD on a cable (J2).** Lets the detector live in the cone/lens assembly while the board stays on the bench/airframe. **Orientation (HW-verified 2026-07-16): CATHODE → PD_NODE (TIA −in); ANODE → VBIAS (+in) per this schematic** — a zero-bias photovoltaic-mode TIA (no dark-current offset; the junction capacitance penalty is irrelevant at 200 Hz chips). *Anode → GND also works* (puts ~VBIAS of reverse bias on the PD; same output polarity). Either way illumination drives TIA_OUT *above* VBIAS and dark rests at VBIAS. **Swapping anode↔cathode fails**: the junction ends up ~VBIAS forward-biased — diffusion capacitance smears the edges (slow rising ramp), ambient pulls the output *below* VBIAS, and the code is undecodable (measured live, both ways, on first-light day). NB with anode→VBIAS the photocurrent is sourced by the VBIAS node — keep the divider stiff + bypassed (≥1 µF). If lead identity is in doubt, use the solar-cell test (DMM mV, illuminated: positive terminal = anode); don't trust flat/lead-length conventions.
5. **Comparator is optional** — skip for first bring-up; the ADC subsumes it.
6. **StepFPGA (MachXO2) has no analog input** — the MCP3201 is mandatory. SPI → its hard-SPI block.
7. Low BW (~100–200 Hz) → MCP6022 (10 MHz) is wildly fast enough; stability easy.

## Design choices to review (open forks)

1. **Bias buffer: dual op-amp vs bypassed divider.** As drawn, op-amp B buffers VBIAS to low-Z (clean for the ADC IN− sampling kickback). *Simpler alternative*: drop the buffer, drive VBIAS straight from the R2/R3 divider with a **bypass cap to GND** (low-Z at the sampling frequency). Removes a half-IC + wiring; the divider's ~8 kΩ source + a 1 µF bypass is fine at 100 kS/s. Leaning toward this for v1 — fewer parts, and it sidesteps the multi-unit op-amp.
2. **DC vs AC coupling.** DC tap (TIA_OUT → ADC) keeps the ambient pedestal for **AGC/envelope** studies but bright sun can rail the TIA; AC tap strips solar DC for **full code range**. Jumper both, or pick DC for the bench / AC for daylight field?
3. **ADC reference: pseudo-diff vs single-ended.** ⚠️ **REVISED 2026-07-16 (bench)**: the MCP3201's
   pseudo-differential IN− is spec-limited to **±100 mV about VSS** (datasheet 21290 — it cancels ground
   offsets, not a 0.45 V reference), so IN− = VBIAS is OUT OF SPEC even though it can appear to work.
   **Use single-ended (IN− = GND) — the as-built config.** The VBIAS pedestal (~0.45 V ≈ 18 % of range)
   is stripped by the decoder's DC tracker; if the range is ever needed, the in-spec offset-removal path
   is the AC-couple (C2 + R4), not IN−.
4. **Vref: MCP1525 2.5 V vs VDD.** A dedicated 2.5 V ref gives stable, noise-free full-scale (better SNR numbers); VDD (3.3 V) is simpler but noisier. Recommend the MCP1525.
5. **ADC speed.** MCP3201 @ 100 kS/s = ~1000× oversampling at 100 Hz chip — plenty. ADS7042 @ 1 MS/s only if you want finer oversampling sweeps.
6. **Comparator (U4): include or omit v1?** The ADC subsumes it; recommend omit for first bring-up.

## Next step (KiCad capture)
Capture as `beacon-receiver.kicad_sch` — stock symbols `Device:R/C/D_Photo`, `Amplifier_Operational:MCP6022`, `Analog_ADC:MCP3201`, generic connectors; `MCP1525` needs a generic 3-pin ref stand-in (not in stock libs). **Note**: the auto-capture tool mishandles the MCP6022's two units (overlapping placement shorts VBIAS) — so capture the **single-op-amp / bypassed-divider** variant (choice 1) to keep it clean, or place unit B by hand in the KiCad GUI.
