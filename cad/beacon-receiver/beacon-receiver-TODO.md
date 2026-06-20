# beacon-receiver — schematic cleanup TODO

Rough KiCad capture done via MCP (`beacon-receiver.kicad_sch` + `beacon-receiver-schematic.pdf`). Connectivity is captured via net labels; the items below need the KiCad GUI (the auto-capture tool can't place multi-unit symbols or custom parts). Topology + design rationale: [collector-schematic.md](collector-schematic.md).

## Must-fix (ERC errors)
- [ ] **Place the MCP6022 power unit (unit C)** and wire pins **4 → GND, 8 → +3V3**. Right now the amp's power pins aren't placed → ERC "input power pins in unit C not placed". (The +3V3/GND labels currently floating near U1 are the symptom.)
- [ ] **Place + tie off the unused 2nd amp (U1 unit B)** — e.g. `+in → VBIAS`, `−in → out` (unity follower), output left unconnected. This also clears the netlist artifact where `U1/5,6,7` currently ride on `VBIAS/PD_NODE/TIA_OUT` (an unplaced-unit coordinate collision; resolves once unit B is properly placed).

## Add (not in stock libs)
- [ ] **U3 = MCP1525 2.5 V ref** — add a 3-pin series-ref symbol driving `VREF2V5` from +3V3 (mirror the custom `LM3410X` symbol approach used on beacon-eval), **or** tie `VREF2V5 → +3V3` (use VDD as Vref) for v1.

## Tidy
- [ ] **Move-to-grid / re-layout** — clears the "off connection grid" warnings (artifacts of auto-placed coordinates); spread components for readability.
- [ ] **J1 pin 6 (`COMP_OUT`)** — leave a no-connect flag, or wire to the optional comparator (U4) if included.
- [ ] **Footprints** — assign footprints (the parts were placed symbol-only).

## Open design choices (see collector-schematic.md §"Design choices to review")
- [ ] **Coupling (fork #2)**: currently the **DC tap** (TIA_OUT → ADC IN+) is drawn. To add the **AC-couple** option: 1 µF series cap from TIA_OUT + a re-bias resistor (1 MΩ) to VBIAS into the ADC IN+, on a jumper. Decide DC-only / AC-only / jumper-both.
- [ ] **Comparator (U4)**: omitted in v1 (ADC subsumes it). Add LM393P + threshold divider if a hard-1-bit baseline is wanted.

## Verify after cleanup
- [ ] Re-run ERC → 0 errors.
- [ ] Confirm nets: `PD_NODE`, `TIA_OUT`, `VBIAS`, `VREF2V5`, `SPI_CS/CLK/DOUT`, `+3V3`, `GND`.
