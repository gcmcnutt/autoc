# Beacon-Pod Schematic (wire-level)

**Spec ref**: [FR-1.2 (driver topology)](spec.md), [FR-1.2.1 (BOM)](spec.md), [FR-1.7 #4 (UVLO contract)](spec.md), [verified-bom.md §A + §B + §C](verified-bom.md).
**Status**: ⚠️ **STALE / PRE-R11 (flagged 2026-06-18) — DO NOT hand-build from this doc.** It still shows the **U3 supervisor (MCP1316), R2 as a pull-UP to V_BAT, and a 3-sink wired-AND DIM**. All of that was **superseded by R11** (2026-05-20): supervisor removed, UVLO is firmware-ADC + WDT, and **R2 is now a pull-DOWN from DIM to GND** (topological failsafe). The **authoritative, R11-correct, ERC-verified schematic is [`../../cad/beacon-eval/beacon-eval.kicad_sch`](../../cad/beacon-eval/beacon-eval.kicad_sch)** (verified 2026-06-18: `DIM` = R2/2 + U1/4 + J1/5; `R2/1` → GND; 0 ERC errors). This pod-level doc needs a full R11 rewrite before any production-pod build. _Original status: Phase-1 hand-prototype schematic — not a PCB design._
**Authoring date**: 2026-05-18 (post-T007a correction; **pre-R11**).

## Reference-designator ↔ BOM-line cross-reference

| Refdes | Description | BOM line | Vendor part |
|---|---|---|---|
| BT1 | 1S LiPo battery | [C1](verified-bom.md) | Tinywhoop 100 mAh JST-PH 2.0 |
| J1 | JST-PH 2.0 mm 2-pin THT socket | [A14](verified-bom.md) | S2B-PH-K-S or equivalent |
| U1 | Boost LED driver | [A1](verified-bom.md) | TI **LM3410XMF-NOPB** (SOT-23-5) |
| U2 | MCU | [A2](verified-bom.md) | Microchip ATtiny412 (SOIC-8) |
| U3 | Voltage supervisor (3.3 V trip, open-drain) | [A4](verified-bom.md) | MCP1316T-29LE/OT (operator-confirmed) |
| L1 | Boost inductor 22 µH shielded | [A5](verified-bom.md) | Coilcraft DR0810-223ML or similar |
| D1 | Boost rectifier Schottky | [A6](verified-bom.md) | MBR130T1G (operator sub'd MBR130T**3**G) |
| D2 | Diagnostic visible-green LED | [A15](verified-bom.md) | 0603 green LED |
| LED1..LED5 | NIR 850 nm LEDs (series-strung in 5-LED ring) | [B1](verified-bom.md) | Lumileds L1IZ-0850000000000 |
| R1 | LED-current sense resistor 0.62 Ω 1% | [A7](verified-bom.md) | CRL1206-FW-R620ELF (operator-confirmed) |
| R2 | DIM-net pull-up 10 kΩ | [A13](verified-bom.md) | Generic 0603 |
| R3 | Diagnostic LED series 1 kΩ | [A16](verified-bom.md) | Generic 0603 |
| C1 | V_LED bulk 4.7 µF / 25 V X7R 1206 | [A8](verified-bom.md) | Samsung CL31B475KAHNNNE (operator-confirmed) |
| C2 | V_BAT bulk 22 µF / 10 V X7R 1210 | [A9](verified-bom.md) | TDK C3225X7R1A226K230AC (operator-confirmed) |
| C3 | MCU VDD decoupling 1 µF | [A10](verified-bom.md) | Samsung CL10B105KP8NNNC (operator-confirmed) |
| C4 | MCU VDD decoupling 100 nF (HF) | [A11](verified-bom.md) | Generic 0603 |
| C5 | Boost VIN decoupling 2.2 µF | [A12](verified-bom.md) | Samsung CL10B225KP8NNNC (operator-confirmed) |
| C6 | Boost VIN decoupling 100 nF (HF) | [A11](verified-bom.md) | Generic 0603 |
| C7 | Supervisor VCC decoupling 100 nF | [A11](verified-bom.md) | Generic 0603 |
| JP1, JP2 | Code-select jumpers (PCB pads / wire bridges to GND) | — (no BOM part — solder bridges or short wires) | n/a |
| J2 | UPDI programming header (3-pin: UPDI, GND, VCC) | [A21](verified-bom.md) | 0.1″ male strip |

**BOM lines NOT on the pod**: A3 (Curiosity Nano dev kit — bench-only), A17 (SOT-23-to-DIP adapters — hand-prototype mechanical only), A18 (perfboard — substrate, not a circuit element), A19/A20 (USB-UART + 4.7 kΩ — programming cable, not on pod). Cart C2 (LiPo charger) is also off-pod.

---

## Net list

Each net + every refdes pin that touches it. Use this to verify "every wire goes somewhere expected" during hand-build.

### V_BAT (1S LiPo rail, 3.0 – 4.2 V)

| Component | Pin | Role |
|---|---|---|
| BT1 (+) | via J1 pin 1 | source |
| J1 | pin 1 | battery socket positive terminal |
| U1 (LM3410X) | pin 5 VIN | boost driver input |
| U2 (ATtiny412) | pin 1 VDD | MCU supply (no LDO — direct from V_BAT) |
| U3 (supervisor) | VCC pin | supervisor supply + sense |
| R2 | one side | DIM pull-up source (R2 other side → DIM net) |
| C2 | + | V_BAT bulk cap (22 µF) |
| C3 | + | MCU decoupling (1 µF) |
| C4 | + | MCU decoupling (100 nF, HF) |
| C5 | + | Boost VIN decoupling (2.2 µF) |
| C6 | + | Boost VIN decoupling (100 nF, HF) |
| C7 | + | Supervisor VCC decoupling (100 nF) |

### GND (common ground)

| Component | Pin | Role |
|---|---|---|
| BT1 (–) | via J1 pin 2 | battery return |
| J1 | pin 2 | battery socket negative |
| U1 (LM3410X) | pin 2 GND | driver ground (+ thermal pad if any) |
| U2 (ATtiny412) | pin 8 GND | MCU ground |
| U3 (supervisor) | GND pin | supervisor ground |
| C1, C2, C3, C4, C5, C6, C7 | – | bulk + decoupling returns |
| D1 (Schottky) | — | (no direct GND — anode is on SW, cathode is on V_LED) |
| R1 | one side | sense-resistor low side (other side → LED-string-return / FB) |
| R3 | one side | diagnostic LED cathode (D2 cathode → GND) |
| D2 (diag LED) | cathode | (via R3) |
| JP1, JP2 (if installed) | — | code-select to GND for bit=0 |

### V_LED (boost-converter output, ~9.5 V regulated)

| Component | Pin | Role |
|---|---|---|
| D1 (Schottky) | cathode | boost rectifier output |
| C1 | + | V_LED bulk cap (4.7 µF / 25 V) |
| LED1 | anode (apex face) | first LED in series string |

### SW (LM3410X switching node, ~9.5 V swinging at 1.6 MHz)

| Component | Pin | Role |
|---|---|---|
| U1 (LM3410X) | pin 1 SW | internal NMOS switch node |
| L1 | one terminal | other terminal → V_BAT |
| D1 (Schottky) | anode | other terminal (cathode) → V_LED |

> ⚠️ **High-dI/dt node** — keep this loop physically tight (FR-1.6 mitigation #3). Boost inductor → Schottky → output cap → ground return all in a compact triangle on the perfboard.

### LED string (V_LED → ground through R1 → FB)

```
V_LED ─→ LED1 (apex)  ─→ LED2 (side) ─→ LED3 (side) ─→ LED4 (side) ─→ LED5 (side) ─→ [LED string return]
                                                                                       │
                                                                                       ▼
                                                                                  R1 0.62 Ω ─→ FB pin (U1)
                                                                                       │
                                                                                       ▼
                                                                                      GND
```

LED current is regulated by U1's FB pin tracking the 190 mV internal reference across R1: I_LED = 190 mV / 0.62 Ω ≈ **306 mA** (within the FR-1.4 300 mA spec).

### DIM (wired-AND: pull-up + 2 open-drain sinks)

| Component | Pin | Drive mode |
|---|---|---|
| U1 (LM3410X) | pin 4 DIM | input (high-Z; ~100 nA input current) |
| R2 | other side | pull-up to V_BAT (one side on V_BAT) |
| U2 (ATtiny412) | PA3 (pin 7) | open-drain emulated (DIRSET=drive LOW for chip=0, DIRCLR=release for chip=1) |
| U3 (supervisor) | OUT (open-drain) | pulls LOW at V_BAT ≤ 3.3 V UVLO; high-Z otherwise |

Truth table:

| supervisor | MCU PA3 | DIM | LM3410X state | LEDs |
|---|---|---|---|---|
| high-Z (V_BAT OK) | high-Z (chip = 1) | HIGH (pull-up) | active | ON @ 300 mA |
| high-Z (V_BAT OK) | LOW (chip = 0) | LOW (MCU pulls) | shutdown (~80 nA) | OFF |
| LOW (UVLO trip) | any | LOW (supervisor pulls) | shutdown (~80 nA) | OFF |

### CSEL0, CSEL1 (2-bit code-select — read by MCU once at boot)

| Component | Pin | Role |
|---|---|---|
| U2 (ATtiny412) | PA1 (pin 4) — CSEL0 | input with internal pull-up enabled |
| U2 (ATtiny412) | PA2 (pin 5) — CSEL1 | input with internal pull-up enabled |
| JP1 (optional bridge) | — | short PA1 → GND for code-id bit 0 = 0 |
| JP2 (optional bridge) | — | short PA2 → GND for code-id bit 1 = 0 |

Code-id encoding: floating (= pulled HIGH via MCU internal pull-up) = bit 1; bridged to GND = bit 0. Pod A = code 0 (JP1 + JP2 both bridged to GND); Pod B = code 1 (JP1 bridged, JP2 open).

### LED_DIAG (diagnostic visible LED)

| Component | Pin | Role |
|---|---|---|
| U2 (ATtiny412) | PA6 (pin 2) | output, push-pull, toggled in the ISR with the same chip bit as PA3 |
| R3 | one side | series resistor (1 kΩ) → D2 anode |
| D2 | anode | (via R3) |
| D2 | cathode | GND |

When MCU is alive and emitting, D2 visibly blinks at the 100 Hz chip rate (operator sees ~50% duty pulsing). Independent of whether the IR LEDs actually emit (supervisor could be pulling DIM low while MCU still toggles PA6).

### UPDI (programming-only; idle in normal operation)

| Component | Pin | Role |
|---|---|---|
| U2 (ATtiny412) | PA0 (pin 6) | UPDI pin (default function) |
| J2 (3-pin programming header) | pin 1 UPDI, pin 2 GND, pin 3 V_BAT | serialUPDI cable connects here for flashing |

---

## ASCII schematic (top-down view)

```
                                                                                 
                        ┌────────── V_BAT rail ──────────────────────────────────┐
                        │                                                         │
                        │                                                         │
    BT1 (+) ──── J1 pin 1                                                         │
        │                │                                                         │
       1S               C2                                                         │
       LiPo            22µF                                                        │
        │                │                                                         │
    BT1 (–) ──── J1 pin 2 ─── GND rail ────────────────────────────────────────────┤
                                                                                   │
                                                                                   │
   ═══════════════════════════════════════════════════════════════════════         │
                                                                                   │
    ┌─── BOOST CONVERTER ────────────────────────────────────────────────┐         │
    │                                                                    │         │
    │              L1 22µH       D1 MBR130                                │        │
    │   V_BAT ────┬────────► SW ◄═══════► V_LED ─────────────────────────┼─────────┤
    │             │              ▲                              │        │         │
    │             │              │                              C1       │         │
    │           U1 LM3410X       │                            4.7µF      │         │
    │             VIN (pin 5)    │                              │        │         │
    │             SW  (pin 1) ───┘                              │        │         │
    │             GND (pin 2) ── GND                            │        │         │
    │             FB  (pin 3) ──────────────────────────┐       │        │         │
    │             DIM (pin 4) ──── DIM net              │       │        │         │
    │                              ▲                    │       │        │         │
    │                              │                    │       │        │         │
    │   C5 2.2µF + C6 100nF                             │       │        │         │
    │   between VIN and GND        │                    │       │        │         │
    │                              │                    │       │        │         │
    └──────────────────────────────┼────────────────────┼───────┼────────┘         │
                                   │                    │       │                  │
                                   │                    │       ▼                  │
   ═══════════════════════════════ │ ═══════════════════│═══════════════════════════│
                                   │                    │       │                  │
    ┌─── LED STRING ─────────────  │                    │       ▼                  │
    │                              │                    │      LED1 (apex)         │
    │                              │                    │       │                  │
    │                              │                    │      LED2 (side)         │
    │                              │                    │       │                  │
    │                              │                    │      LED3 (side)         │
    │                              │                    │       │                  │
    │                              │                    │      LED4 (side)         │
    │                              │                    │       │                  │
    │                              │                    │      LED5 (side)         │
    │                              │                    │       │                  │
    │                              │                    │      R1 0.62Ω            │
    │                              │                    └──────┤                   │
    │                              │                           │                   │
    │                              ▼                           ▼                   │
    │                             GND  ←─────────────────── (sense + return)       │
    └──────────────────────────────────────────────────────────────────────         │
                                                                                   │
   ═══════════════════════════════════════════════════════════════════════         │
                                                                                   │
    ┌─── DIM NET (wired-AND) ─────────────────────────────────────────────┐        │
    │                                                                     │        │
    │     V_BAT ───── R2 10kΩ ────┬── U1 DIM (pin 4)                      │        │
    │                              │                                      │        │
    │     U2 PA3 ─── (open-drain emulated) ────┤                          │        │
    │                              │                                      │        │
    │     U3 OUT ─── (open-drain, supervisor) ─┤                          │        │
    │                                                                     │        │
    │     Truth: HIGH only if all 3 release; LOW if any pull              │        │
    └─────────────────────────────────────────────────────────────────────┘        │
                                                                                   │
   ═══════════════════════════════════════════════════════════════════════         │
                                                                                   │
    ┌─── MCU ──────────────────────────────────────────────────────────┐           │
    │                                                                  │           │
    │   U2 ATtiny412 (SOIC-8)                                          │           │
    │      pin 1 VDD ──────────── V_BAT                                │           │
    │      pin 2 PA6 ── R3 1kΩ ── D2 (green LED) ── GND  (LED_DIAG)    │           │
    │      pin 3 PA7 ─── (spare)                                       │           │
    │      pin 4 PA1 ── (CSEL0)  ─ JP1 ── GND (bridged = code bit 0)   │           │
    │      pin 5 PA2 ── (CSEL1)  ─ JP2 ── GND                          │           │
    │      pin 6 PA0 ── UPDI ── J2 (programming header, idle in flight)│           │
    │      pin 7 PA3 ── DIM-out (open-drain emulated) ── DIM net       │           │
    │      pin 8 GND ── GND                                            │           │
    │                                                                  │           │
    │   C3 1µF + C4 100nF between VDD and GND                          │           │
    │                                                                  │           │
    └──────────────────────────────────────────────────────────────────┘           │
                                                                                   │
   ═══════════════════════════════════════════════════════════════════════         │
                                                                                   │
    ┌─── SUPERVISOR ───────────────────────────────────────────────────┐           │
    │                                                                  │           │
    │   U3 MCP1316T-29LE/OT (or alternates)                            │           │
    │      VCC ── V_BAT                                                │           │
    │      GND ── GND                                                  │           │
    │      OUT (open-drain) ── DIM net (pulls LOW at V_BAT ≤ 3.3V)     │           │
    │      MR / NC pins per chosen-part datasheet (tie inactive)       │           │
    │                                                                  │           │
    │   C7 100nF between VCC and GND                                   │           │
    │                                                                  │           │
    └──────────────────────────────────────────────────────────────────┘           │
                                                                                   │
   ═══════════════════════════════════════════════════════════════════════         │
                                                                                   │
                                                                                   ▼
                                                                                  GND
```

---

## Hand-build assembly order (suggested — from inside to outside the half-cube)

1. **Solder the boost-converter loop FIRST** on the perfboard, in a tight triangle: U1 (LM3410X) — L1 — D1 — C1. Keep this loop physical < 5 mm if possible. (FR-1.6 mitigation: tight switch-node loop minimizes radiated EMI.)
2. Solder R1 (sense resistor) between U1 pin 3 (FB) and the future LED-string-return pad.
3. Solder U2 (ATtiny412) + C3 + C4 decoupling at the inboard-PCB-end.
4. Solder U3 (supervisor) + C7 decoupling near U2 (keep both close to V_BAT entry).
5. Wire the DIM net: V_BAT → R2 → DIM pad, with U1 pin 4 + U2 PA3 + U3 OUT all joined at the DIM pad.
6. Wire CSEL0 + CSEL1 + UPDI programming header.
7. Solder J1 (JST-PH socket) at the inboard face.
8. Mount the 5 Lumileds in the cube face indents, wire in series with 32 AWG magnet wire (LED1 apex → LED2 → LED3 → LED4 → LED5 → R1 return).
9. Wire D2 (diag LED) + R3 from U2 PA6 to inboard-face light-pipe slot.
10. Bench-verify per [quickstart.md (a)](../040-camera-redo/camera-hardware-phase/quickstart.md): insert battery → pod boots ≤100 ms with diag LED blinking → scope LED-string current per FR-1.5(a) → FR-1.7 UVLO bench → FR-3.3 EMC bench.

---

## What's NOT in this schematic (deliberately)

- **PCB layout decisions** (trace widths, copper pours, via placement) — deferred to a future 031-integration PCB spin.
- **Test points** beyond the obvious ones (V_BAT, V_LED, SW, DIM, FB) — operator probes anywhere on the perfboard.
- **ESD protection** — Phase 1 single-build pods; if a future variant goes into volume, add TVS diodes on V_BAT + LED string.
- **Fusing on V_BAT** — 100 mAh 1S LiPo's 20 C rating + the 800 mA peak inductor current is within the cell's protection envelope; no separate fuse needed for Phase 1.
- **Battery thermistor / fuel gauge** — out of scope; operator pulls + recharges by hand.

---

## Cross-check at receiving

When a Cart §A or §B part arrives, verify:

1. **Find the refdes** in the table above (column 1 → BOM line column 4).
2. **Check the package** matches what the schematic assumes (e.g., LM3410X must be SOT-23-5 / DBV — if you got the WSON or MSOP variant, the pinout in the net list is different — adjust before soldering).
3. **For substituted parts** (anything operator picked from the "or equivalent" alternatives — already happening per the notes column in `verified-bom.md`), confirm pin function + footprint match.
4. **Record in `verified-bom.md` notes** any substitution that affects the schematic (e.g., MBR130T**3**G instead of T**1**G — verify pinout matches per the substitute's datasheet; both are usually SOD-123 with the same anode/cathode convention but always verify).

Mismatches surfaced at receiving avoid the much worse fail mode of "blue smoke at first power-on".
